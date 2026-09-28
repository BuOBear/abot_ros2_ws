#include "abot_hardware/protocol.hpp"
#include "abot_hardware/serial_session.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace abot_hardware {
namespace {

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Clock = std::chrono::steady_clock;

struct Settings {
  std::string port;
  int baud_rate{};
  int command_timeout_ms{};
  int response_timeout_ms{};
  int reconnect_delay_ms{};
  int command_period_ms{};
  int odom_period_ms{};
  int imu_period_ms{};
  bool publish_imu{};
  bool publish_mag{};
  std::string odom_frame;
  std::string base_frame;
  std::string imu_frame;
  double max_vx_mps{};
  double max_vy_mps{};
  double max_wz_radps{};
  double odom_pose_x_stddev_m{};
  double odom_pose_y_stddev_m{};
  double odom_pose_yaw_stddev_rad{};
  double odom_twist_vx_stddev_mps{};
  double odom_twist_vy_stddev_mps{};
  double odom_twist_wz_stddev_radps{};
  double imu_accel_stddev_mps2{};
  double imu_gyro_stddev_radps{};
  double imu_mag_stddev_tesla{};
};

bool valid_frame(const std::string &frame) {
  return !frame.empty() && frame[0] != '/' && frame.find(' ') == std::string::npos;
}

diagnostic_msgs::msg::KeyValue key_value(const std::string &key, const std::string &value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

}  // namespace

class BaseDriver : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit BaseDriver(const rclcpp::NodeOptions &options)
  : rclcpp_lifecycle::LifecycleNode("abot_hardware", options) {
    declare_parameter<std::string>("port", "/dev/abot");
    declare_parameter<int>("baud_rate", 921600);
    declare_parameter<int>("command_timeout_ms", 250);
    declare_parameter<int>("response_timeout_ms", 100);
    declare_parameter<int>("reconnect_delay_ms", 500);
    declare_parameter<int>("command_period_ms", 50);
    declare_parameter<int>("odom_period_ms", 50);
    declare_parameter<int>("imu_period_ms", 20);
    declare_parameter<bool>("publish_imu", true);
    declare_parameter<bool>("publish_mag", false);
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_footprint");
    declare_parameter<std::string>("imu_frame", "imu_link");
    declare_parameter<double>("max_vx_mps", 2.0);
    declare_parameter<double>("max_vy_mps", 2.0);
    declare_parameter<double>("max_wz_radps", 5.0);
    declare_parameter<double>("odom_pose_x_stddev_m", 1.0);
    declare_parameter<double>("odom_pose_y_stddev_m", 1.0);
    declare_parameter<double>("odom_pose_yaw_stddev_rad", 1.0);
    declare_parameter<double>("odom_twist_vx_stddev_mps", 0.1);
    declare_parameter<double>("odom_twist_vy_stddev_mps", 0.1);
    declare_parameter<double>("odom_twist_wz_stddev_radps", 0.2);
    declare_parameter<double>("imu_accel_stddev_mps2", 0.5);
    declare_parameter<double>("imu_gyro_stddev_radps", 0.1);
    declare_parameter<double>("imu_mag_stddev_tesla", 1e-5);
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &parameter : parameters) {
          if (parameter.get_name() == "use_sim_time" && parameter.as_bool()) {
            result.successful = false;
            result.reason = "physical base hardware requires use_sim_time=false";
            break;
          }
          if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
            result.successful = false;
            result.reason = "hardware parameters change only while unconfigured";
            break;
          }
        }
        return result;
      });
  }

  ~BaseDriver() override { stop(); }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    if (get_parameter("use_sim_time").as_bool()) {
      RCLCPP_ERROR(get_logger(), "physical base hardware requires use_sim_time=false");
      return CallbackReturn::FAILURE;
    }
    Settings settings;
    settings.port = get_parameter("port").as_string();
    settings.baud_rate = get_parameter("baud_rate").as_int();
    settings.command_timeout_ms = get_parameter("command_timeout_ms").as_int();
    settings.response_timeout_ms = get_parameter("response_timeout_ms").as_int();
    settings.reconnect_delay_ms = get_parameter("reconnect_delay_ms").as_int();
    settings.command_period_ms = get_parameter("command_period_ms").as_int();
    settings.odom_period_ms = get_parameter("odom_period_ms").as_int();
    settings.imu_period_ms = get_parameter("imu_period_ms").as_int();
    settings.publish_imu = get_parameter("publish_imu").as_bool();
    settings.publish_mag = get_parameter("publish_mag").as_bool();
    settings.odom_frame = get_parameter("odom_frame").as_string();
    settings.base_frame = get_parameter("base_frame").as_string();
    settings.imu_frame = get_parameter("imu_frame").as_string();
    settings.max_vx_mps = get_parameter("max_vx_mps").as_double();
    settings.max_vy_mps = get_parameter("max_vy_mps").as_double();
    settings.max_wz_radps = get_parameter("max_wz_radps").as_double();
    settings.odom_pose_x_stddev_m = get_parameter("odom_pose_x_stddev_m").as_double();
    settings.odom_pose_y_stddev_m = get_parameter("odom_pose_y_stddev_m").as_double();
    settings.odom_pose_yaw_stddev_rad = get_parameter("odom_pose_yaw_stddev_rad").as_double();
    settings.odom_twist_vx_stddev_mps = get_parameter("odom_twist_vx_stddev_mps").as_double();
    settings.odom_twist_vy_stddev_mps = get_parameter("odom_twist_vy_stddev_mps").as_double();
    settings.odom_twist_wz_stddev_radps = get_parameter("odom_twist_wz_stddev_radps").as_double();
    settings.imu_accel_stddev_mps2 = get_parameter("imu_accel_stddev_mps2").as_double();
    settings.imu_gyro_stddev_radps = get_parameter("imu_gyro_stddev_radps").as_double();
    settings.imu_mag_stddev_tesla = get_parameter("imu_mag_stddev_tesla").as_double();

    const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
    const auto period = [](int value) { return value >= 10 && value <= 1000; };
    if (settings.port.empty() || settings.port[0] != '/' ||
        (settings.baud_rate != 115200 && settings.baud_rate != 230400 &&
         settings.baud_rate != 460800 && settings.baud_rate != 500000 &&
         settings.baud_rate != 576000 && settings.baud_rate != 921600 &&
         settings.baud_rate != 1000000) ||
        !period(settings.command_period_ms) || !period(settings.odom_period_ms) ||
        !period(settings.imu_period_ms) ||
        settings.command_timeout_ms < settings.command_period_ms ||
        settings.command_timeout_ms > 2000 ||
        settings.response_timeout_ms < 10 || settings.response_timeout_ms > 1000 ||
        settings.reconnect_delay_ms < 100 || settings.reconnect_delay_ms > 10000 ||
        !valid_frame(settings.odom_frame) || !valid_frame(settings.base_frame) ||
        !valid_frame(settings.imu_frame) ||
        !positive(settings.max_vx_mps) || settings.max_vx_mps > 327.67 ||
        !positive(settings.max_vy_mps) || settings.max_vy_mps > 327.67 ||
        !positive(settings.max_wz_radps) || settings.max_wz_radps > 327.67 ||
        !positive(settings.odom_pose_x_stddev_m) ||
        !positive(settings.odom_pose_y_stddev_m) ||
        !positive(settings.odom_pose_yaw_stddev_rad) ||
        !positive(settings.odom_twist_vx_stddev_mps) ||
        !positive(settings.odom_twist_vy_stddev_mps) ||
        !positive(settings.odom_twist_wz_stddev_radps) ||
        !positive(settings.imu_accel_stddev_mps2) ||
        !positive(settings.imu_gyro_stddev_radps) ||
        !positive(settings.imu_mag_stddev_tesla)) {
      RCLCPP_ERROR(get_logger(), "invalid hardware parameter; see docs/BASE.md");
      return CallbackReturn::FAILURE;
    }
    settings_ = settings;
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel_odom", rclcpp::QoS(10).reliable());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", rclcpp::SensorDataQoS());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics", rclcpp::QoS(10).reliable());
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr command) { receive_command(*command); });
    RCLCPP_INFO(get_logger(), "configured for %s at %d baud", settings_.port.c_str(), settings_.baud_rate);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    std::string error;
    if (!connect_session(error)) {
      RCLCPP_ERROR(get_logger(), "activation failed: %s", error.c_str());
      return CallbackReturn::FAILURE;
    }
    odom_pub_->on_activate();
    imu_pub_->on_activate();
    mag_pub_->on_activate();
    diagnostics_pub_->on_activate();
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      minimum_command_sequence_ = command_sequence_;
      accept_commands_ = true;
    }
    stop_requested_ = false;
    try {
      worker_ = std::thread([this]() { run(); });
    } catch (const std::exception &exception) {
      RCLCPP_ERROR(get_logger(), "could not start serial worker: %s", exception.what());
      stop();
      odom_pub_->on_deactivate();
      imu_pub_->on_deactivate();
      mag_pub_->on_deactivate();
      diagnostics_pub_->on_deactivate();
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "base driver active; awaiting a fresh velocity command");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    stop();
    odom_pub_->on_deactivate();
    imu_pub_->on_deactivate();
    mag_pub_->on_deactivate();
    diagnostics_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    stop();
    cmd_sub_.reset();
    odom_pub_.reset();
    imu_pub_.reset();
    mag_pub_.reset();
    diagnostics_pub_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    stop();
    return CallbackReturn::SUCCESS;
  }

private:
  Settings settings_;
  SerialSession session_;
  std::thread worker_;
  std::atomic_bool stop_requested_{false};
  std::mutex sleep_mutex_;
  std::condition_variable sleep_cv_;
  std::mutex command_mutex_;
  protocol::Velocity command_{};
  Clock::time_point command_received_{};
  uint64_t command_sequence_{};
  uint64_t minimum_command_sequence_{};
  bool command_valid_{false};
  bool accept_commands_{false};
  uint64_t timeouts_{};
  uint64_t reconnects_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

  bool connect_session(std::string &error) {
    if (!session_.open_device(settings_.port, settings_.baud_rate, error)) return false;
    const auto timeout = std::chrono::milliseconds(settings_.response_timeout_ms);
    const auto version = session_.request(protocol::Id::version, 32, timeout);
    if (!version) {
      error = "no valid firmware version frame before timeout";
      session_.close_device();
      ++timeouts_;
      return false;
    }
    // A zero frame must reach the board before this serial session accepts new
    // commands. A reconnect therefore cannot replay a pre-disconnect Twist.
    if (!send_velocity(protocol::Velocity{})) {
      error = "could not send initial zero velocity";
      session_.close_device();
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      minimum_command_sequence_ = command_sequence_;
    }
    return true;
  }

  void receive_command(const geometry_msgs::msg::Twist &twist) {
    const protocol::Velocity velocity{twist.linear.x, twist.linear.y, twist.angular.z};
    const bool valid = std::isfinite(velocity.x_mps) && std::isfinite(velocity.y_mps) &&
      std::isfinite(velocity.yaw_radps) &&
      std::abs(velocity.x_mps) <= settings_.max_vx_mps &&
      std::abs(velocity.y_mps) <= settings_.max_vy_mps &&
      std::abs(velocity.yaw_radps) <= settings_.max_wz_radps &&
      std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
      std::isfinite(twist.angular.y) &&
      twist.linear.z == 0.0 && twist.angular.x == 0.0 && twist.angular.y == 0.0;
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!accept_commands_) return;
    ++command_sequence_;
    command_valid_ = valid;
    command_ = valid ? velocity : protocol::Velocity{};
    command_received_ = Clock::now();
    if (!valid) RCLCPP_WARN(get_logger(), "invalid or excessive cmd_vel; commanding zero");
  }

  protocol::Velocity current_velocity() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!command_valid_ || command_sequence_ <= minimum_command_sequence_ ||
        Clock::now() - command_received_ >
          std::chrono::milliseconds(settings_.command_timeout_ms)) {
      return {};
    }
    return command_;
  }

  bool send_velocity(const protocol::Velocity &velocity) {
    const auto data = protocol::encode_velocity(velocity);
    if (!data) return false;
    return session_.send(protocol::Id::set_velocity,
      std::vector<uint8_t>(data->begin(), data->end()),
      std::chrono::milliseconds(settings_.response_timeout_ms));
  }

  void publish_odom(const protocol::Odom &data) {
    nav_msgs::msg::Odometry message;
    message.header.stamp = now();  // receipt time; firmware protocol has no timestamp
    message.header.frame_id = settings_.odom_frame;
    message.child_frame_id = settings_.base_frame;
    message.pose.pose.position.x = data.x_m;
    message.pose.pose.position.y = data.y_m;
    message.pose.pose.orientation.z = std::sin(data.yaw_rad * 0.5);
    message.pose.pose.orientation.w = std::cos(data.yaw_rad * 0.5);
    message.twist.twist.linear.x = data.velocity.x_mps;
    message.twist.twist.linear.y = data.velocity.y_mps;
    message.twist.twist.angular.z = data.velocity.yaw_radps;
    message.pose.covariance[0] = std::pow(settings_.odom_pose_x_stddev_m, 2);
    message.pose.covariance[7] = std::pow(settings_.odom_pose_y_stddev_m, 2);
    message.pose.covariance[14] = message.pose.covariance[21] =
      message.pose.covariance[28] = 1e6;
    message.pose.covariance[35] = std::pow(settings_.odom_pose_yaw_stddev_rad, 2);
    message.twist.covariance[0] = std::pow(settings_.odom_twist_vx_stddev_mps, 2);
    message.twist.covariance[7] = std::pow(settings_.odom_twist_vy_stddev_mps, 2);
    message.twist.covariance[14] = message.twist.covariance[21] =
      message.twist.covariance[28] = 1e6;
    message.twist.covariance[35] = std::pow(settings_.odom_twist_wz_stddev_radps, 2);
    odom_pub_->publish(message);
  }

  void publish_imu(const protocol::Imu &data) {
    const auto stamp = now();
    if (settings_.publish_imu) {
      sensor_msgs::msg::Imu message;
      message.header.stamp = stamp;
      message.header.frame_id = settings_.imu_frame;
      message.orientation_covariance[0] = -1.0;
      message.linear_acceleration.x = data.acceleration_mps2[0];
      message.linear_acceleration.y = data.acceleration_mps2[1];
      message.linear_acceleration.z = data.acceleration_mps2[2];
      message.angular_velocity.x = data.angular_velocity_radps[0];
      message.angular_velocity.y = data.angular_velocity_radps[1];
      message.angular_velocity.z = data.angular_velocity_radps[2];
      message.linear_acceleration_covariance[0] =
        message.linear_acceleration_covariance[4] =
        message.linear_acceleration_covariance[8] =
          std::pow(settings_.imu_accel_stddev_mps2, 2);
      message.angular_velocity_covariance[0] =
        message.angular_velocity_covariance[4] =
        message.angular_velocity_covariance[8] =
          std::pow(settings_.imu_gyro_stddev_radps, 2);
      imu_pub_->publish(message);
    }
    if (settings_.publish_mag) {
      sensor_msgs::msg::MagneticField message;
      message.header.stamp = stamp;
      message.header.frame_id = settings_.imu_frame;
      message.magnetic_field.x = data.magnetic_field_tesla[0];
      message.magnetic_field.y = data.magnetic_field_tesla[1];
      message.magnetic_field.z = data.magnetic_field_tesla[2];
      message.magnetic_field_covariance[0] =
        message.magnetic_field_covariance[4] =
        message.magnetic_field_covariance[8] =
          std::pow(settings_.imu_mag_stddev_tesla, 2);
      mag_pub_->publish(message);
    }
  }

  void publish_diagnostics(bool connected) {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "abot_hardware/serial";
    status.hardware_id = settings_.port;
    status.level = connected ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = connected ? "connected" : "disconnected; retrying";
    status.values.push_back(key_value("timeouts", std::to_string(timeouts_)));
    status.values.push_back(key_value("reconnects", std::to_string(reconnects_)));
    status.values.push_back(key_value("rejected_frames", std::to_string(session_.rejected_frames())));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  void wait_or_stop(std::chrono::milliseconds duration) {
    std::unique_lock<std::mutex> lock(sleep_mutex_);
    sleep_cv_.wait_for(lock, duration, [this]() { return stop_requested_.load(); });
  }

  void run() {
    auto next_command = Clock::now();
    auto next_odom = Clock::now();
    auto next_imu = Clock::now();
    auto next_diagnostics = Clock::now();
    int odom_failures = 0;
    int imu_failures = 0;
    // Feedback transactions are synchronous. Check the command deadline after
    // each one so a slow odom and a slow IMU reply cannot both postpone a stop.
    const auto send_due_command = [this, &next_command]() {
      if (Clock::now() < next_command) return true;
      if (!send_velocity(current_velocity())) return false;
      next_command = Clock::now() + std::chrono::milliseconds(settings_.command_period_ms);
      return true;
    };
    while (!stop_requested_) {
      if (!session_.is_open()) {
        publish_diagnostics(false);
        std::string error;
        if (!connect_session(error)) {
          RCLCPP_WARN(get_logger(), "serial reconnect failed: %s", error.c_str());
          wait_or_stop(std::chrono::milliseconds(settings_.reconnect_delay_ms));
          continue;
        }
        ++reconnects_;
        odom_failures = imu_failures = 0;
        next_command = next_odom = next_imu = Clock::now();
        next_diagnostics = Clock::now();
        RCLCPP_INFO(get_logger(), "serial connection restored; awaiting a fresh command");
      }

      if (!send_due_command()) { disconnect(); continue; }
      if (Clock::now() >= next_odom) {
        const auto frame = session_.request(protocol::Id::get_odom, 16,
          std::chrono::milliseconds(settings_.response_timeout_ms));
        const auto odom = frame ? protocol::decode_odom(*frame) : std::nullopt;
        if (odom) {
          publish_odom(*odom);
          odom_failures = 0;
        } else {
          ++timeouts_;
          if (++odom_failures >= 3) { disconnect(); continue; }
        }
        next_odom = Clock::now() + std::chrono::milliseconds(settings_.odom_period_ms);
      }
      if (stop_requested_) break;
      if (!send_due_command()) { disconnect(); continue; }
      if ((settings_.publish_imu || settings_.publish_mag) && Clock::now() >= next_imu) {
        const auto frame = session_.request(protocol::Id::get_imu, 36,
          std::chrono::milliseconds(settings_.response_timeout_ms));
        const auto imu = frame ? protocol::decode_imu(*frame) : std::nullopt;
        if (imu) {
          publish_imu(*imu);
          imu_failures = 0;
        } else {
          ++timeouts_;
          if (++imu_failures >= 3) { disconnect(); continue; }
        }
        next_imu = Clock::now() + std::chrono::milliseconds(settings_.imu_period_ms);
      }
      if (stop_requested_) break;
      if (!send_due_command()) { disconnect(); continue; }
      if (Clock::now() >= next_diagnostics) {
        publish_diagnostics(true);
        next_diagnostics = Clock::now() + 1s;
      }
      const auto next_due = std::min({next_command, next_odom,
        (settings_.publish_imu || settings_.publish_mag) ? next_imu : Clock::now() + 1s,
        next_diagnostics});
      if (next_due > Clock::now()) {
        const auto wait = std::chrono::duration_cast<std::chrono::milliseconds>(next_due - Clock::now());
        wait_or_stop(std::max(wait, 1ms));
      }
    }
  }

  void disconnect() {
    // A bad feedback transaction does not necessarily mean writes have failed.
    // Try to stop the board before releasing the port; send() has a deadline.
    if (session_.is_open() && !send_velocity(protocol::Velocity{})) {
      RCLCPP_ERROR(get_logger(), "failed to send zero velocity during disconnect");
    }
    session_.close_device();
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      minimum_command_sequence_ = command_sequence_;
    }
    RCLCPP_WARN(get_logger(), "serial feedback/command failed; reconnecting");
    wait_or_stop(std::chrono::milliseconds(settings_.reconnect_delay_ms));
  }

  void stop() {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      accept_commands_ = false;
      command_valid_ = false;
      minimum_command_sequence_ = command_sequence_;
    }
    stop_requested_ = true;
    sleep_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (session_.is_open() && !send_velocity(protocol::Velocity{})) {
      RCLCPP_ERROR(get_logger(), "failed to send zero velocity during stop");
    }
    session_.close_device();
  }
};

}  // namespace abot_hardware

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<abot_hardware::BaseDriver>(rclcpp::NodeOptions{});
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
