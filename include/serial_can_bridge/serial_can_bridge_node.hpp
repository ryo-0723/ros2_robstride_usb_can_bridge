#pragma once

#include "serial_can_bridge/serial_bridge.hpp"

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace serial_can_bridge
{
class SerialCanBridgeNode : public rclcpp::Node
{
  public:
    explicit SerialCanBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  private:
    void transmit(const can_msgs::msg::Frame& frame);
    void receive();

    std::string frame_id_;
    std::unique_ptr<SerialBridge> serial_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace serial_can_bridge
