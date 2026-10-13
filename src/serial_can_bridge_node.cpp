#include "serial_can_bridge/serial_can_bridge_node.hpp"
#include "serial_can_bridge/protocol.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace serial_can_bridge
{
SerialCanBridgeNode::SerialCanBridgeNode(const rclcpp::NodeOptions& options)
    : Node("serial_can_bridge", options)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto device = declare_parameter<std::string>("device", "/dev/ttyUSB0", descriptor);
    const auto baud = declare_parameter<int64_t>("baud_rate", 921600, descriptor);
    const auto tx = declare_parameter<std::string>("tx_topic", "to_can_bus", descriptor);
    const auto rx = declare_parameter<std::string>("rx_topic", "from_can_bus", descriptor);
    frame_id_ = declare_parameter<std::string>("frame_id", "can", descriptor);
    const auto period = declare_parameter<double>("poll_period", 0.001, descriptor);
    const auto timeout = declare_parameter<double>("write_timeout", 0.1, descriptor);
    const auto tx_queue_size = declare_parameter<int64_t>("tx_queue_size", 100, descriptor);
    const auto rx_queue_size = declare_parameter<int64_t>("rx_queue_size", 100, descriptor);
    const auto max_age = declare_parameter<double>("tx_queue_max_age", 0.1, descriptor);

    if (device.empty() || baud <= 0 || baud > std::numeric_limits<int>::max() ||
        !std::isfinite(period) || period < 0.000001 || period > 1.0 || !std::isfinite(timeout) ||
        timeout < 0.001 || timeout > 60.0 || tx_queue_size <= 0 || rx_queue_size <= 0 ||
        static_cast<uint64_t>(tx_queue_size) > std::numeric_limits<std::size_t>::max() ||
        static_cast<uint64_t>(rx_queue_size) > std::numeric_limits<std::size_t>::max() ||
        !std::isfinite(max_age) || max_age < 0.001 || max_age > 60.0)
    {
        throw std::invalid_argument(
            "Expected nonempty device, positive baud_rate, poll_period (1us..1s), "
            "positive queue sizes, and write_timeout/tx_queue_max_age (1ms..60s)");
    }
    serial_ = std::make_unique<SerialBridge>(
        device, static_cast<unsigned int>(baud),
        std::chrono::milliseconds(static_cast<int64_t>(std::ceil(timeout * 1000.0))),
        static_cast<std::size_t>(tx_queue_size), static_cast<std::size_t>(rx_queue_size),
        std::chrono::milliseconds(static_cast<int64_t>(std::ceil(max_age * 1000.0))),
        [clock = get_clock()]() { return clock->now().nanoseconds(); });

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    publisher_ = create_publisher<can_msgs::msg::Frame>(rx, qos);

    subscription_ = create_subscription<can_msgs::msg::Frame>(
        tx, qos, [this](can_msgs::msg::Frame::ConstSharedPtr frame) { transmit(*frame); });

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
        [this]() { receive(); });
    serial_->open();
    RCLCPP_INFO(get_logger(), "USB-CAN: %s at %lld baud; %s -> serial -> %s", device.c_str(),
                static_cast<long long>(baud), tx.c_str(), rx.c_str());
}

void SerialCanBridgeNode::transmit(const can_msgs::msg::Frame& frame)
{
    SerialFrame packet;
    try
    {
        packet = encode_frame(frame);
    }
    catch (const std::invalid_argument& error)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Dropping CAN frame: %s",
                             error.what());
        return;
    }
    serial_->write(packet);
}

void SerialCanBridgeNode::receive()
{
    const auto stats = serial_->queue_statistics();
    if (stats.tx_overflow != last_queue_statistics_.tx_overflow ||
        stats.tx_expired != last_queue_statistics_.tx_expired ||
        stats.rx_overflow != last_queue_statistics_.rx_overflow)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Dropped queued frames (total): TX overflow=%llu, TX expired=%llu, RX overflow=%llu",
            static_cast<unsigned long long>(stats.tx_overflow),
            static_cast<unsigned long long>(stats.tx_expired),
            static_cast<unsigned long long>(stats.rx_overflow));
        last_queue_statistics_ = stats;
    }
    // @note 大量受信で他のコールバックを妨げないよう、1回の処理数を制限
    for (std::size_t count = 0; count < 100; ++count)
    {
        const auto packet = serial_->read_with_timestamp();
        if (!packet)
        {
            break;
        }
        if (auto frame = decode_frame(packet->data))
        {
            frame->header.stamp = rclcpp::Time(packet->received_at_ns, get_clock()->get_clock_type());
            frame->header.frame_id = frame_id_;
            publisher_->publish(*frame);
        }
    }
}
} // namespace serial_can_bridge
