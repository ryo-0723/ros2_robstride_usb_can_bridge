#include "serial_can_bridge/serial_can_bridge_node.hpp"
#include "serial_can_bridge/protocol.hpp"

#include <gtest/gtest.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>
#include <chrono>
#include <functional>
#include <limits>
#include <mutex>
#include <rcl/time.h>

using namespace std::chrono_literals;
using serial_can_bridge::SerialCanBridgeNode;

namespace
{
class NodeTest : public testing::Test
{
  protected:
    static void SetUpTestSuite()
    {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite()
    {
        rclcpp::shutdown();
    }
    void SetUp() override
    {
        int slave;
        char name[128];
        ASSERT_EQ(openpty(&master_, &slave, name, nullptr, nullptr), 0);
        device_ = name;
        close(slave);
    }
    void TearDown() override
    {
        close(master_);
    }
    rclcpp::NodeOptions options(std::vector<rclcpp::Parameter> parameters = {})
    {
        parameters.emplace_back("device", device_);
        return rclcpp::NodeOptions().parameter_overrides(parameters);
    }
    bool wait_for(rclcpp::executors::SingleThreadedExecutor& executor,
                  const std::function<bool()>& predicate)
    {
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (!predicate())
        {
            executor.spin_some();
            if (std::chrono::steady_clock::now() >= deadline)
            {
                return false;
            }
            std::this_thread::sleep_for(1ms);
        }
        return true;
    }
    int master_{-1};
    std::string device_;
};

TEST_F(NodeTest, DefaultsAndExplicitOverrides)
{
    auto node = std::make_shared<SerialCanBridgeNode>(options());
    EXPECT_STREQ(node->get_name(), "serial_can_bridge");
    EXPECT_EQ(node->get_parameter("baud_rate").as_int(), 921600);
    EXPECT_EQ(node->get_parameter("write_timeout").as_double(), 0.1);
    EXPECT_EQ(node->get_parameter("tx_queue_size").as_int(), 100);
    EXPECT_EQ(node->get_parameter("rx_queue_size").as_int(), 100);
    EXPECT_EQ(node->get_parameter("tx_queue_max_age").as_double(), 0.1);
    EXPECT_EQ(node->get_parameter("tx_topic").as_string(), "to_can_bus");
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("poll_period", 0.01)).successful);
    node.reset();
    node = std::make_shared<SerialCanBridgeNode>(
        options({rclcpp::Parameter("baud_rate", 115200), rclcpp::Parameter("write_timeout", 0.2),
                 rclcpp::Parameter("rx_topic", "custom_rx"),
                 rclcpp::Parameter("tx_queue_size", 3), rclcpp::Parameter("rx_queue_size", 4),
                 rclcpp::Parameter("tx_queue_max_age", 0.2)}));
    EXPECT_EQ(node->get_parameter("baud_rate").as_int(), 115200);
    EXPECT_EQ(node->get_parameter("write_timeout").as_double(), 0.2);
    EXPECT_EQ(node->get_parameter("rx_topic").as_string(), "custom_rx");
    EXPECT_EQ(node->get_parameter("tx_queue_size").as_int(), 3);
    EXPECT_EQ(node->get_parameter("rx_queue_size").as_int(), 4);
    EXPECT_EQ(node->get_parameter("tx_queue_max_age").as_double(), 0.2);
}

TEST_F(NodeTest, RejectsInvalidParameters)
{
    EXPECT_THROW(SerialCanBridgeNode(options({rclcpp::Parameter("baud_rate", 0)})),
                 std::invalid_argument);
    for (const auto* name : {"tx_queue_size", "rx_queue_size"})
    {
        EXPECT_THROW(SerialCanBridgeNode(options({rclcpp::Parameter(name, 0)})),
                     std::invalid_argument);
        EXPECT_THROW(SerialCanBridgeNode(options({rclcpp::Parameter(name, -1)})),
                     std::invalid_argument);
    }
    for (const auto value : {0.0, -0.1, 61.0, std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW(SerialCanBridgeNode(options({rclcpp::Parameter("tx_queue_max_age", value)})),
                     std::invalid_argument);
    }
    EXPECT_THROW(SerialCanBridgeNode(options({rclcpp::Parameter("write_timeout", 0.0)})),
                 std::invalid_argument);
    EXPECT_THROW(SerialCanBridgeNode(options(
                     {rclcpp::Parameter("poll_period", std::numeric_limits<double>::quiet_NaN())})),
                 std::invalid_argument);
}

TEST_F(NodeTest, ConvertsTopicsToSerialAndSerialToTopics)
{
    auto node = std::make_shared<SerialCanBridgeNode>(options());
    auto peer = std::make_shared<rclcpp::Node>("serial_test_peer");
    auto publisher = peer->create_publisher<can_msgs::msg::Frame>("to_can_bus", 100);
    std::optional<can_msgs::msg::Frame> received;
    auto subscription = peer->create_subscription<can_msgs::msg::Frame>(
        "from_can_bus", 100, [&](can_msgs::msg::Frame::ConstSharedPtr msg) { received = *msg; });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(peer);
    ASSERT_TRUE(wait_for(executor,
                         [&]()
                         {
                             return publisher->get_subscription_count() > 0 &&
                                    subscription->get_publisher_count() > 0;
                         }));
    can_msgs::msg::Frame frame;
    frame.id = 0x1200fd01;
    frame.is_extended = true;
    pollfd fd{master_, POLLIN, 0};
    for (const uint8_t dlc : {8, 4, 0})
    {
        frame.dlc = dlc;
        frame.data = {5, 0x70, 0, 0, 1, 0, 0, 0};
        received.reset();
        publisher->publish(frame);
        ASSERT_TRUE(wait_for(executor, [&]() { return poll(&fd, 1, 0) > 0; }));
        serial_can_bridge::SerialFrame packet(9U + dlc);
        ASSERT_EQ(::read(master_, packet.data(), packet.size()),
                  static_cast<ssize_t>(packet.size()));
        EXPECT_EQ(packet, serial_can_bridge::encode_frame(frame));
        ASSERT_EQ(::write(master_, packet.data(), packet.size()),
                  static_cast<ssize_t>(packet.size()));
        ASSERT_TRUE(wait_for(executor, [&]() { return received.has_value(); }));
        EXPECT_EQ(received->id, frame.id);
        for (std::size_t i = 0; i < frame.data.size(); ++i)
        {
            EXPECT_EQ(received->data[i], i < dlc ? frame.data[i] : 0);
        }
        EXPECT_EQ(received->dlc, dlc);
        EXPECT_TRUE(received->is_extended);
        EXPECT_EQ(received->header.frame_id, "can");
        EXPECT_GT(rclcpp::Time(received->header.stamp).nanoseconds(), 0);
    }
    frame.dlc = 9;
    publisher->publish(frame);
    const auto deadline = std::chrono::steady_clock::now() + 50ms;
    while (std::chrono::steady_clock::now() < deadline)
    {
        executor.spin_some();
        std::this_thread::sleep_for(1ms);
    }
    EXPECT_EQ(poll(&fd, 1, 0), 0);
}

TEST_F(NodeTest, TimestampKeepsRosReceiveTimeAfterClockJump)
{
    auto node = std::make_shared<SerialCanBridgeNode>(options());
    auto peer = std::make_shared<rclcpp::Node>("timestamp_test_peer");
    std::optional<can_msgs::msg::Frame> received;
    auto subscription = peer->create_subscription<can_msgs::msg::Frame>(
        "from_can_bus", 100, [&](can_msgs::msg::Frame::ConstSharedPtr msg) { received = *msg; });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(peer);
    ASSERT_TRUE(wait_for(executor, [&]() { return subscription->get_publisher_count() > 0; }));
    const auto clock = node->get_clock();
    {
        std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
        ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
        ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 100000000000LL), RCL_RET_OK);
    }
    can_msgs::msg::Frame frame;
    frame.id = 0x300;
    frame.is_extended = true;
    frame.dlc = 4;
    frame.data = {0, 0, 0x0b, 0xb8, 0, 0, 0, 0};
    const auto packet = serial_can_bridge::encode_frame(frame);
    ASSERT_EQ(::write(master_, packet.data(), packet.size()), static_cast<ssize_t>(packet.size()));
    // @note ROSの配信処理を止め、受信キューで待たせた後に時計を進める
    std::this_thread::sleep_for(100ms);
    {
        std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
        ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 200000000000LL), RCL_RET_OK);
    }
    ASSERT_TRUE(wait_for(executor, [&]() { return received.has_value(); }));
    EXPECT_EQ(received->header.stamp.sec, 100);
    EXPECT_EQ(received->header.stamp.nanosec, 0U);
    EXPECT_EQ(received->header.frame_id, "can");
    EXPECT_EQ(received->dlc, 4);
}
} // namespace
