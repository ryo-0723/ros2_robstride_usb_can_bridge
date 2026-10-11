#include "serial_can_bridge/serial_bridge.hpp"

#include <gtest/gtest.h>
#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using serial_can_bridge::SerialBridge;

namespace
{
class SerialBridgeTest : public testing::Test
{
  protected:
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
    bool wait_for(const std::function<bool()>& predicate)
    {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (!predicate())
        {
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

serial_can_bridge::SerialFrame packet(uint8_t value)
{
    return {0x41, 0x54, 0, 0, 0, 4, 8, value, 0, 0, 0, 0, 0, 0, 0, 0x0d, 0x0a};
}

TEST_F(SerialBridgeTest, QueuedFixedSizeWritesKeepOrder)
{
    SerialBridge serial(device_, 115200, 1s);
    serial.open();
    std::vector<uint8_t> expected;
    for (int i = 0; i < 100; ++i)
    {
        const auto frame = packet(static_cast<uint8_t>(i));
        expected.insert(expected.end(), frame.begin(), frame.end());
        serial.write(frame);
    }
    std::vector<uint8_t> transmitted;
    ASSERT_TRUE(wait_for(
        [&]()
        {
            pollfd descriptor{master_, POLLIN, 0};
            if (poll(&descriptor, 1, 0) > 0)
            {
                uint8_t buffer[512];
                const auto count = ::read(master_, buffer, sizeof(buffer));
                if (count > 0)
                {
                    transmitted.insert(transmitted.end(), buffer, buffer + count);
                }
            }
            return transmitted.size() >= expected.size();
        }));
    EXPECT_EQ(transmitted, expected);
    EXPECT_FALSE(serial.error());
    EXPECT_THROW(serial.write({}), std::invalid_argument);
}

TEST_F(SerialBridgeTest, ReceiveReturnsOnlyCompleteValidatedPackets)
{
    SerialBridge serial(device_, 115200);
    serial.open();
    EXPECT_FALSE(serial.read());
    auto first = packet(0x80);
    first[8] = 0x41;
    first[9] = 0x54;
    first[10] = 0x0d;
    first[11] = 0x0a;
    auto bad = first;
    bad.back() = 0;
    std::vector<uint8_t> prefix{0, 0xff, 0x41};
    prefix.insert(prefix.end(), bad.begin(), bad.end());
    prefix.insert(prefix.end(), first.begin(), first.begin() + 10);
    ASSERT_EQ(::write(master_, prefix.data(), prefix.size()), static_cast<ssize_t>(prefix.size()));
    std::this_thread::sleep_for(20ms);
    EXPECT_FALSE(serial.read());
    const auto second = packet(0xff);
    std::vector<uint8_t> suffix(first.begin() + 10, first.end());
    suffix.insert(suffix.end(), second.begin(), second.end());
    ASSERT_EQ(::write(master_, suffix.data(), suffix.size()), static_cast<ssize_t>(suffix.size()));
    std::vector<serial_can_bridge::SerialFrame> received;
    ASSERT_TRUE(wait_for(
        [&]()
        {
            if (auto frame = serial.read())
            {
                received.push_back(*frame);
            }
            return received.size() == 2;
        }));
    EXPECT_EQ(received[0], first);
    EXPECT_EQ(received[1], second);
    EXPECT_FALSE(serial.read());
}

TEST_F(SerialBridgeTest, BlockedFixedSizeWritesReportTimeout)
{
    SerialBridge serial(device_, 115200, 200ms);
    serial.open();
    // Fill the PTY output without draining it; a queued packet will block.
    for (int i = 0; i < 4096; ++i)
    {
        serial.write(packet(0x55));
    }
    EXPECT_FALSE(serial.error());
    ASSERT_TRUE(wait_for([&]() { return bool(serial.error()); }));
    EXPECT_EQ(serial.error(), boost::asio::error::timed_out);
    EXPECT_THROW(serial.write(packet(0xbb)), boost::system::system_error);
    EXPECT_THROW(serial.read(), boost::system::system_error);
    serial.close();
    serial.open();
    EXPECT_FALSE(serial.error());
}

TEST_F(SerialBridgeTest, CloseCancelsPendingWritesAndReopenHasNoStaleQueue)
{
    SerialBridge serial(device_, 115200, 10s);
    serial.open();
    for (int i = 0; i < 4096; ++i)
    {
        serial.write(packet(0x55));
    }
    const auto started = std::chrono::steady_clock::now();
    serial.close();
    EXPECT_LT(std::chrono::steady_clock::now() - started, 1s);
    EXPECT_THROW(serial.write(packet(0xbb)), std::logic_error);
    const auto flags = fcntl(master_, F_GETFL);
    ASSERT_NE(flags, -1);
    ASSERT_NE(fcntl(master_, F_SETFL, flags | O_NONBLOCK), -1);
    uint8_t buffer[4096];
    while (::read(master_, buffer, sizeof(buffer)) > 0)
    {
    }
    serial.open();
    const auto frame = packet(0x42);
    serial.write(frame);
    std::vector<uint8_t> output;
    ASSERT_TRUE(wait_for(
        [&]()
        {
            const auto count = ::read(master_, buffer, sizeof(buffer));
            if (count > 0)
            {
                output.insert(output.end(), buffer, buffer + count);
            }
            return output.size() >= frame.size();
        }));
    EXPECT_EQ(output, std::vector<uint8_t>(frame.begin(), frame.end()));
    EXPECT_FALSE(serial.error());
}

TEST_F(SerialBridgeTest, ReopenDiscardsPartialReceivePacket)
{
    SerialBridge serial(device_, 115200);
    serial.open();
    const auto frame = packet(0x42);
    ASSERT_EQ(::write(master_, frame.data(), 10), 10);
    std::this_thread::sleep_for(20ms);
    EXPECT_FALSE(serial.read());
    serial.close();
    serial.open();
    ASSERT_EQ(::write(master_, frame.data() + 10, 7), 7);
    ASSERT_EQ(::write(master_, frame.data(), frame.size()), 17);
    std::optional<serial_can_bridge::SerialFrame> result;
    ASSERT_TRUE(wait_for(
        [&]()
        {
            result = serial.read();
            return bool(result);
        }));
    EXPECT_EQ(*result, frame);
    EXPECT_FALSE(serial.read());
}
} // namespace
