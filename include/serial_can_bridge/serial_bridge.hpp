#pragma once

#include "serial_can_bridge/serial_frame.hpp"

#include <boost/asio.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <optional>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace serial_can_bridge
{
// Variable-length AT serial packet transport. CAN conversion is handled separately.
class SerialBridge
{
  public:
    SerialBridge(const std::string& device, unsigned int baud_rate,
                 std::chrono::milliseconds write_timeout = std::chrono::milliseconds(100));
    ~SerialBridge();
    SerialBridge(const SerialBridge&) = delete;
    SerialBridge& operator=(const SerialBridge&) = delete;

    // Open as 8N1 with no flow control and start the receive thread.
    void open();
    void close();
    // Queue bytes and return without waiting for transmission.
    // Timeout starts when this item begins writing, excluding queue wait.
    void write(SerialFrame data);
    // First asynchronous I/O failure; cleared by open().
    boost::system::error_code error() const;
    // Return one validated packet without waiting; nullopt means no new packet.
    std::optional<SerialFrame> read();

  private:
    void start_read();
    void start_write();
    void fail(const boost::system::error_code& error);

    std::string device_;
    unsigned int baud_rate_;
    std::chrono::milliseconds write_timeout_;
    boost::asio::io_context io_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_;
    boost::asio::serial_port port_;
    boost::asio::steady_timer write_timer_;
    std::thread thread_;
    // Serialize public lifecycle calls and scheduling of writes.
    std::mutex lifecycle_mutex_;
    std::atomic<bool> accepting_{false};
    // Accessed only by the I/O thread, or after it has been joined.
    std::deque<SerialFrame> write_queue_;
    SerialFrame active_write_{};
    bool writing_{false};
    std::size_t write_id_{0};
    std::array<uint8_t, 4096> buffer_{};
    mutable std::mutex receive_mutex_;
    SerialFrameParser parser_;
    std::deque<SerialFrame> received_;
    boost::system::error_code io_error_;
};
} // namespace serial_can_bridge
