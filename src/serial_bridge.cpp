#include "serial_can_bridge/serial_bridge.hpp"

#include <stdexcept>
#include <utility>

namespace serial_can_bridge
{
SerialBridge::SerialBridge(const std::string& device, unsigned int baud_rate,
                           std::chrono::milliseconds write_timeout)
    : device_(device), baud_rate_(baud_rate), write_timeout_(write_timeout),
      work_(boost::asio::make_work_guard(io_)), port_(io_), write_timer_(io_)
{
    if (device.empty() || baud_rate == 0 || write_timeout.count() <= 0)
    {
        throw std::invalid_argument(
            "device must not be empty; baud_rate and write_timeout must be positive");
    }
}

SerialBridge::~SerialBridge()
{
    close();
}

void SerialBridge::open()
{
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (thread_.joinable())
    {
        throw std::logic_error("serial port is already open");
    }
    io_.restart();
    try
    {
        port_.open(device_);
        using Options = boost::asio::serial_port_base;
        port_.set_option(Options::baud_rate(baud_rate_));
        port_.set_option(Options::character_size(8));
        port_.set_option(Options::parity(Options::parity::none));
        port_.set_option(Options::stop_bits(Options::stop_bits::one));
        port_.set_option(Options::flow_control(Options::flow_control::none));
        {
            std::lock_guard<std::mutex> receive_lock(receive_mutex_);
            received_.clear();
            io_error_.clear();
        }
        parser_.reset();
        accepting_ = true;
        start_read();
        thread_ = std::thread([this]() { io_.run(); });
    }
    catch (...)
    {
        accepting_ = false;
        boost::system::error_code ignored;
        port_.close(ignored);
        io_.poll(); // Drain cancellation if thread creation failed.
        throw;
    }
}

void SerialBridge::close()
{
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    // Stop and join before accessing the shared serial_port from this thread.
    accepting_ = false;
    io_.stop();
    if (thread_.joinable())
    {
        thread_.join();
    }
    boost::system::error_code ignored;
    write_timer_.cancel();
    port_.close(ignored);
    io_.restart();
    io_.poll(); // Complete cancelled reads/writes before reusing their buffers.
    write_queue_.clear();
    writing_ = false;
}

void SerialBridge::write(SerialFrame data)
{
    if (!is_valid_serial_frame(data))
    {
        throw std::invalid_argument("Expected a valid 17-byte EL05 serial frame");
    }
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    const auto failure = error();
    if (failure)
    {
        throw boost::system::system_error(failure, "serial connection failed");
    }
    if (!accepting_)
    {
        throw std::logic_error("serial port is not open");
    }
    boost::asio::post(io_,
                      [this, data]()
                      {
                          if (!accepting_)
                          {
                              return;
                          }
                          write_queue_.push_back(data);
                          if (!writing_)
                          {
                              start_write();
                          }
                      });
}

boost::system::error_code SerialBridge::error() const
{
    std::lock_guard<std::mutex> lock(receive_mutex_);
    return io_error_;
}

void SerialBridge::start_write()
{
    writing_ = true;
    const auto id = ++write_id_;
    active_write_ = write_queue_.front();
    write_queue_.pop_front();
    write_timer_.expires_after(write_timeout_);
    write_timer_.async_wait(
        [this, id](const boost::system::error_code& error)
        {
            // An expired timer may already be queued when cancel() is called.
            if (!error && accepting_ && writing_ && id == write_id_)
            {
                fail(boost::asio::error::timed_out);
            }
        });
    boost::asio::async_write(port_, boost::asio::buffer(active_write_),
                             [this](const boost::system::error_code& error, std::size_t)
                             {
                                 write_timer_.cancel();
                                 if (!accepting_)
                                 {
                                     return;
                                 }
                                 if (error)
                                 {
                                     fail(error);
                                     return;
                                 }
                                 writing_ = false;
                                 if (!write_queue_.empty())
                                 {
                                     start_write();
                                 }
                             });
}

void SerialBridge::fail(const boost::system::error_code& error)
{
    {
        std::lock_guard<std::mutex> lock(receive_mutex_);
        if (!io_error_)
        {
            io_error_ = error;
        }
    }
    accepting_ = false;
    write_timer_.cancel();
    boost::system::error_code ignored;
    port_.close(ignored);
    // A timed out write may have sent a prefix: discard pending data instead of
    // continuing a potentially broken byte stream. Call close()/open() to recover.
    write_queue_.clear();
    writing_ = false;
}

std::optional<SerialFrame> SerialBridge::read()
{
    std::lock_guard<std::mutex> lock(receive_mutex_);
    // Deliver already received bytes before reporting a subsequent disconnect.
    if (received_.empty() && io_error_)
    {
        throw boost::system::system_error(io_error_, "read serial bytes");
    }
    if (received_.empty())
    {
        return std::nullopt;
    }
    auto result = received_.front();
    received_.pop_front();
    return result;
}

void SerialBridge::start_read()
{
    port_.async_read_some(boost::asio::buffer(buffer_),
                          [this](const boost::system::error_code& error, std::size_t count)
                          {
                              const auto packets = parser_.feed(buffer_.data(), count);
                              {
                                  std::lock_guard<std::mutex> lock(receive_mutex_);
                                  received_.insert(received_.end(), packets.begin(), packets.end());
                              }
                              if (error && accepting_)
                              {
                                  fail(error);
                              }
                              else if (!error && accepting_)
                              {
                                  start_read();
                              }
                          });
}
} // namespace serial_can_bridge
