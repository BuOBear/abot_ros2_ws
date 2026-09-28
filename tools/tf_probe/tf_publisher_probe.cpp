// Integration-only observer: associate every TF child with its actual DDS writer.
#include <iomanip>
#include <iostream>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("p0_tf_publisher_probe");
  auto callback = [](tf2_msgs::msg::TFMessage::ConstSharedPtr message,
      const rclcpp::MessageInfo & info) {
      std::ostringstream gid;
      for (auto value : info.get_rmw_message_info().publisher_gid.data) {
        gid << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(value);
      }
      for (const auto & transform : message->transforms) {
        std::cout << transform.child_frame_id << ' ' << transform.header.frame_id
                  << ' ' << gid.str() << std::endl;
      }
    };
  auto dynamic = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(100), callback);
  auto fixed = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(100).transient_local(), callback);
  rclcpp::spin(node);
  rclcpp::shutdown();
}
