#include "abot_accessory/protocol.hpp"
#include "abot_accessory/serial_device.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace abot_accessory {

class AccessoryDriver final : public rclcpp::Node {
 public:
  AccessoryDriver() : Node("accessory_driver") {
    rcl_interfaces::msg::ParameterDescriptor enabled_description;
    enabled_description.description = "Open and exclusively lock the accessory serial port";
    enabled_description.read_only = true;
    const bool enabled = declare_parameter<bool>("enabled", false, enabled_description);

    rcl_interfaces::msg::ParameterDescriptor port_description;
    port_description.description = "Accessory serial device path; must be absolute when set";
    port_description.read_only = true;
    const std::string port = declare_parameter<std::string>("port", "", port_description);

    rcl_interfaces::msg::ParameterDescriptor baud_description;
    baud_description.description = "Accessory protocol baud rate (fixed at 9600)";
    baud_description.read_only = true;
    const int baud_rate = declare_parameter<int>("baud_rate", 9600, baud_description);

    rcl_interfaces::msg::ParameterDescriptor timeout_description;
    timeout_description.description = "Deadline for writing one complete 8-byte frame, in ms";
    timeout_description.read_only = true;
    const int write_timeout_ms = declare_parameter<int>("write_timeout_ms", 250,
                                                        timeout_description);

    rcl_interfaces::msg::ParameterDescriptor shutdown_description;
    shutdown_description.description =
        "Send the stop frame on shutdown if this process successfully sent shoot";
    shutdown_description.read_only = true;
    stop_on_shutdown_ = declare_parameter<bool>("stop_on_shutdown", true,
                                                shutdown_description);

    if (write_timeout_ms < 10 || write_timeout_ms > 5000) {
      throw std::invalid_argument("write_timeout_ms must be between 10 and 5000");
    }
    if (!port.empty() && port.front() != '/') {
      throw std::invalid_argument("port must be a non-empty absolute path when specified");
    }
    if (enabled && port.empty()) {
      throw std::invalid_argument("enabled=true requires an absolute port path");
    }
    if (baud_rate != 9600) {
      throw std::invalid_argument("baud_rate must be 9600 for this accessory protocol");
    }
    write_timeout_ = std::chrono::milliseconds(write_timeout_ms);

    if (!enabled) {
      RCLCPP_INFO(get_logger(), "accessory disabled; no serial port opened and no frame sent");
    } else {
      std::string error;
      if (serial_.open_device(port, baud_rate, error)) {
        RCLCPP_INFO(get_logger(), "locked accessory port %s at 9600 baud; no frame sent",
                    port.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "accessory port unavailable: %s", error.c_str());
        open_error_ = error;
      }
    }

    shoot_service_ = create_service<std_srvs::srv::Trigger>(
        "/accessory/shoot",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          handle_shoot(response);
        });
    stop_service_ = create_service<std_srvs::srv::Trigger>(
        "/accessory/stop",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          handle_stop(response);
        });
  }

  ~AccessoryDriver() override {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (stop_on_shutdown_ && may_be_active_ && serial_.is_open()) {
      std::string error;
      if (serial_.write_frame(protocol::kStopFrame, write_timeout_, error)) {
        RCLCPP_INFO(get_logger(), "sent stop frame before closing accessory port");
      } else {
        RCLCPP_ERROR(get_logger(), "failed to stop accessory on shutdown: %s", error.c_str());
      }
    }
    serial_.close_device();
  }

 private:
  void respond_not_ready(std_srvs::srv::Trigger::Response &response) const {
    response.success = false;
    if (!open_error_.empty()) {
      response.message = "accessory serial port unavailable: " + open_error_;
    } else {
      response.message = "accessory disabled; start with enabled:=true and an absolute port";
    }
  }

  void handle_shoot(const std::shared_ptr<std_srvs::srv::Trigger::Response> &response) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_.is_open()) {
      respond_not_ready(*response);
      return;
    }
    std::size_t written = 0;
    std::string error;
    if (!serial_.write_frame(protocol::kShootFrame, write_timeout_, error, &written)) {
      may_be_active_ = may_be_active_ || written > 0;
      response->success = false;
      response->message = "shoot frame write failed: " + error;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    may_be_active_ = true;
    response->success = true;
    response->message = "shoot frame sent (55 01 12 00 00 00 01 69)";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void handle_stop(const std::shared_ptr<std_srvs::srv::Trigger::Response> &response) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_.is_open()) {
      respond_not_ready(*response);
      return;
    }
    std::size_t written = 0;
    std::string error;
    if (!serial_.write_frame(protocol::kStopFrame, write_timeout_, error, &written)) {
      response->success = false;
      response->message = "stop frame write failed: " + error;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    may_be_active_ = false;
    response->success = true;
    response->message = "stop frame sent (55 01 11 00 00 00 01 68)";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  SerialDevice serial_;
  std::chrono::milliseconds write_timeout_{250};
  bool stop_on_shutdown_{true};
  bool may_be_active_{false};
  std::string open_error_;
  std::mutex serial_mutex_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shoot_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
};

}  // namespace abot_accessory

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<abot_accessory::AccessoryDriver>());
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("accessory_driver"), "invalid configuration: %s",
                 exception.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
