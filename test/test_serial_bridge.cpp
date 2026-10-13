#include "serial_can_bridge/serial_bridge.hpp"

#include <gtest/gtest.h>
#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
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
    /// @brief 疑似端末の出力を満たし、実機なしで送信待ちを作る
    bool fill_output()
    {
        const int slave = ::open(device_.c_str(), O_WRONLY | O_NONBLOCK);
        if (slave < 0)
        {
            return false;
        }
        const std::array<uint8_t, 4096> filler{};
        const auto deadline = std::chrono::steady_clock::now() + 1s;
        auto last_progress = std::chrono::steady_clock::now();
        bool filled = false;
        while (std::chrono::steady_clock::now() < deadline)
        {
            const auto count = ::write(slave, filler.data(), filler.size());
            if (count > 0)
            {
                last_progress = std::chrono::steady_clock::now();
            }
            else if (count < 0 && errno == EAGAIN)
            {
                // @note 疑似端末内部の転送が落ち着くまで、出力が詰まった状態を確認する
                if (std::chrono::steady_clock::now() - last_progress >= 20ms)
                {
                    filled = true;
                    break;
                }
                std::this_thread::sleep_for(1ms);
            }
            else
            {
                break;
            }
        }
        close(slave);
        return filled;
    }
    /// @brief 出力を読み出し、指定件数のATパケットが揃うまで待つ
    bool collect_output(std::vector<serial_can_bridge::SerialFrame>& frames, std::size_t expected)
    {
        serial_can_bridge::SerialFrameParser parser;
        return wait_for([&]()
                        {
                            pollfd descriptor{master_, POLLIN, 0};
                            if (poll(&descriptor, 1, 0) > 0)
                            {
                                uint8_t bytes[4096];
                                const auto count = ::read(master_, bytes, sizeof(bytes));
                                if (count > 0)
                                {
                                    const auto packets = parser.feed(bytes, count);
                                    frames.insert(frames.end(), packets.begin(), packets.end());
                                }
                            }
                            return frames.size() >= expected;
                        });
    }
    int master_{-1};
    std::string device_;
};

serial_can_bridge::SerialFrame packet(uint8_t value)
{
    return {0x41, 0x54, 0, 0, 0, 4, 8, value, 0, 0, 0, 0, 0, 0, 0, 0x0d, 0x0a};
}

TEST_F(SerialBridgeTest, QueuedMixedLengthWritesKeepOrder)
{
    SerialBridge serial(device_, 115200, 1s);
    serial.open();
    std::vector<uint8_t> expected;
    for (int i = 0; i < 100; ++i)
    {
        auto frame = packet(static_cast<uint8_t>(i));
        const auto dlc = static_cast<uint8_t>(i % 9);
        frame[6] = dlc;
        frame.resize(9U + dlc);
        frame[frame.size() - 2] = 0x0d;
        frame.back() = 0x0a;
        expected.insert(expected.end(), frame.begin(), frame.end());
        serial.write(frame);
    }
    std::vector<uint8_t> transmitted;
    ASSERT_TRUE(wait_for([&]() {
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
    const serial_can_bridge::SerialFrame second{0x41, 0x54, 0,    0,    0,    4,   4,
                                                0xff, 0x41, 0x0d, 0x0a, 0x0d, 0x0a};
    std::vector<uint8_t> suffix(first.begin() + 10, first.end());
    suffix.insert(suffix.end(), second.begin(), second.end());
    ASSERT_EQ(::write(master_, suffix.data(), suffix.size()), static_cast<ssize_t>(suffix.size()));
    std::vector<serial_can_bridge::SerialFrame> received;
    ASSERT_TRUE(wait_for([&]() {
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
    SerialBridge serial(device_, 115200, 200ms, 4096, 100, 10s);
    serial.open();
    // @note 疑似端末を読み出さずに送信バッファを満たし、タイムアウトを発生させる
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
    SerialBridge serial(device_, 115200, 10s, 4096, 100, 10s);
    serial.open();
    for (int i = 0; i < 4096; ++i)
    {
        serial.write(packet(0x55));
    }
    const auto started = std::chrono::steady_clock::now();
    serial.close();
    EXPECT_LT(std::chrono::steady_clock::now() - started, 1s);
    EXPECT_EQ(serial.queue_statistics().tx_pending, 0U);
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
    ASSERT_TRUE(wait_for([&]() {
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
    ASSERT_TRUE(wait_for([&]() {
        result = serial.read();
        return bool(result);
    }));
    EXPECT_EQ(*result, frame);
    EXPECT_FALSE(serial.read());
}

TEST_F(SerialBridgeTest, TransmitOverflowKeepsNewestWaitingPackets)
{
    SerialBridge serial(device_, 115200, 3s, 3, 100, 10s);
    serial.open();
    ASSERT_TRUE(fill_output());
    serial.write(packet(1));
    ASSERT_TRUE(wait_for([&]() { return serial.queue_statistics().tx_pending == 0; }));
    for (uint8_t value = 2; value <= 5; ++value)
    {
        serial.write(packet(value));
        EXPECT_LE(serial.queue_statistics().tx_pending, 3U);
    }
    const auto stats = serial.queue_statistics();
    EXPECT_EQ(stats.tx_pending, 3U);
    EXPECT_EQ(stats.tx_overflow, 1U);
    EXPECT_EQ(stats.tx_expired, 0U);
    std::vector<serial_can_bridge::SerialFrame> frames;
    ASSERT_TRUE(collect_output(frames, 4));
    ASSERT_EQ(frames.size(), 4U);
    EXPECT_EQ(frames[0], packet(1));
    EXPECT_EQ(frames[1], packet(3));
    EXPECT_EQ(frames[2], packet(4));
    EXPECT_EQ(frames[3], packet(5));
    EXPECT_FALSE(serial.error());
    serial.close();
    serial.open();
    EXPECT_EQ(serial.queue_statistics().tx_overflow, 0U);
}

TEST_F(SerialBridgeTest, ExpiredPacketsAreDiscardedBeforeNextSend)
{
    SerialBridge serial(device_, 115200, 3s, 3, 100, 100ms);
    serial.open();
    ASSERT_TRUE(fill_output());
    serial.write(packet(1));
    ASSERT_TRUE(wait_for([&]() { return serial.queue_statistics().tx_pending == 0; }));
    serial.write(packet(2));
    serial.write(packet(3));
    ASSERT_EQ(serial.queue_statistics().tx_pending, 2U);
    std::this_thread::sleep_for(150ms);
    ASSERT_EQ(serial.queue_statistics().tx_pending, 2U);
    std::vector<serial_can_bridge::SerialFrame> frames;
    ASSERT_TRUE(collect_output(frames, 1));
    ASSERT_TRUE(wait_for([&]() { return serial.queue_statistics().tx_expired == 2; }));
    serial.write(packet(4));
    ASSERT_TRUE(collect_output(frames, 2));
    ASSERT_EQ(frames.size(), 2U);
    EXPECT_EQ(frames[0], packet(1));
    EXPECT_EQ(frames[1], packet(4));
    EXPECT_FALSE(serial.error());
}

TEST_F(SerialBridgeTest, EnqueueDiscardsExpiredWaitingPackets)
{
    SerialBridge serial(device_, 115200, 3s, 3, 100, 100ms);
    serial.open();
    ASSERT_TRUE(fill_output());
    serial.write(packet(1));
    ASSERT_TRUE(wait_for([&]() { return serial.queue_statistics().tx_pending == 0; }));
    serial.write(packet(2));
    serial.write(packet(3));
    ASSERT_EQ(serial.queue_statistics().tx_pending, 2U);
    std::this_thread::sleep_for(150ms);
    ASSERT_EQ(serial.queue_statistics().tx_pending, 2U);
    serial.write(packet(4));
    EXPECT_EQ(serial.queue_statistics().tx_pending, 1U);
    EXPECT_EQ(serial.queue_statistics().tx_expired, 2U);
    std::vector<serial_can_bridge::SerialFrame> frames;
    ASSERT_TRUE(collect_output(frames, 2));
    ASSERT_EQ(frames.size(), 2U);
    EXPECT_EQ(frames[0], packet(1));
    EXPECT_EQ(frames[1], packet(4));
}

TEST_F(SerialBridgeTest, ReceiveOverflowKeepsNewestPackets)
{
    SerialBridge serial(device_, 115200, 1s, 100, 3);
    serial.open();
    std::vector<uint8_t> bytes;
    for (uint8_t value = 0; value < 5; ++value)
    {
        const auto frame = packet(value);
        bytes.insert(bytes.end(), frame.begin(), frame.end());
    }
    ASSERT_EQ(::write(master_, bytes.data(), bytes.size()), static_cast<ssize_t>(bytes.size()));
    ASSERT_TRUE(wait_for([&]() { return serial.queue_statistics().rx_overflow == 2; }));
    EXPECT_EQ(serial.queue_statistics().rx_pending, 3U);
    for (uint8_t value = 2; value < 5; ++value)
    {
        const auto received = serial.read();
        ASSERT_TRUE(received);
        EXPECT_EQ(*received, packet(value));
    }
    EXPECT_FALSE(serial.read());
    serial.close();
    serial.open();
    EXPECT_EQ(serial.queue_statistics().rx_pending, 0U);
    EXPECT_EQ(serial.queue_statistics().rx_overflow, 0U);
}

TEST_F(SerialBridgeTest, RejectsInvalidQueueSettings)
{
    EXPECT_THROW(SerialBridge(device_, 115200, 1s, 0, 100), std::invalid_argument);
    EXPECT_THROW(SerialBridge(device_, 115200, 1s, 100, 0), std::invalid_argument);
    EXPECT_THROW(SerialBridge(device_, 115200, 1s, 100, 100, 0ms), std::invalid_argument);
}
} // namespace
