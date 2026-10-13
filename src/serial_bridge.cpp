#include "robstride_usb_can_bridge/serial_bridge.hpp"

#include <stdexcept>
#include <utility>

namespace robstride_usb_can_bridge
{
SerialBridge::SerialBridge(const std::string& device, unsigned int baud_rate,
                           std::chrono::milliseconds write_timeout,
                           std::size_t tx_queue_size, std::size_t rx_queue_size,
                           std::chrono::milliseconds tx_queue_max_age, ReceiveClock receive_clock)
    : device_(device), baud_rate_(baud_rate), write_timeout_(write_timeout),
      tx_queue_size_(tx_queue_size), rx_queue_size_(rx_queue_size),
      tx_queue_max_age_(tx_queue_max_age),
      receive_clock_(std::move(receive_clock)),
      work_(boost::asio::make_work_guard(io_)), port_(io_), write_timer_(io_)
{
    if (device.empty() || baud_rate == 0 || write_timeout.count() <= 0 ||
        tx_queue_size == 0 || rx_queue_size == 0 || tx_queue_max_age.count() <= 0)
    {
        throw std::invalid_argument(
            "device must not be empty; baud_rate, queue sizes and timeouts must be positive");
    }
    if (!receive_clock_)
    {
        receive_clock_ = []()
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        };
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
            rx_overflow_ = 0;
        }
        {
            std::lock_guard<std::mutex> send_lock(send_mutex_);
            write_queue_.clear();
            write_scheduled_ = false;
            tx_overflow_ = 0;
            tx_expired_ = 0;
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
        // @note スレッド作成失敗時も、受信のキャンセル処理を完了させる
        io_.poll();
        throw;
    }
}

void SerialBridge::close()
{
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    // @note ポートを操作する前に送受信スレッドの終了を待つ
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
    // @note バッファを再利用する前に送受信のキャンセル処理を完了させる
    io_.poll();
    {
        std::lock_guard<std::mutex> send_lock(send_mutex_);
        write_queue_.clear();
        write_scheduled_ = false;
    }
    writing_ = false;
}

void SerialBridge::write(SerialFrame data)
{
    if (!is_valid_serial_frame(data))
    {
        throw std::invalid_argument("Expected a valid AT serial frame with DLC <= 8");
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
    std::lock_guard<std::mutex> send_lock(send_mutex_);
    // @note 通信失敗と送信受付が重なった場合も、停止後のキュー追加を防ぐ
    if (!accepting_)
    {
        throw std::logic_error("serial port is not open");
    }
    const auto now = std::chrono::steady_clock::now();
    discard_expired_writes(now);
    if (write_queue_.size() >= tx_queue_size_)
    {
        write_queue_.pop_front();
        ++tx_overflow_;
    }
    write_queue_.push_back({std::move(data), now});
    if (!write_scheduled_)
    {
        boost::asio::post(io_, [this]()
                          {
                              if (accepting_)
                              {
                                  start_write();
                              }
                          });
        write_scheduled_ = true;
    }
}

void SerialBridge::discard_expired_writes(std::chrono::steady_clock::time_point now)
{
    while (!write_queue_.empty() && now - write_queue_.front().queued_at >= tx_queue_max_age_)
    {
        write_queue_.pop_front();
        ++tx_expired_;
    }
}

SerialBridge::QueueStatistics SerialBridge::queue_statistics() const
{
    std::scoped_lock lock(send_mutex_, receive_mutex_);
    return {write_queue_.size(), received_.size(), tx_overflow_, tx_expired_, rx_overflow_};
}

boost::system::error_code SerialBridge::error() const
{
    std::lock_guard<std::mutex> lock(receive_mutex_);
    return io_error_;
}

void SerialBridge::start_write()
{
    {
        std::lock_guard<std::mutex> send_lock(send_mutex_);
        discard_expired_writes(std::chrono::steady_clock::now());
        if (write_queue_.empty())
        {
            write_scheduled_ = false;
            return;
        }
        active_write_ = std::move(write_queue_.front().data);
        write_queue_.pop_front();
    }
    writing_ = true;
    const auto id = ++write_id_;
    write_timer_.expires_after(write_timeout_);
    write_timer_.async_wait(
        [this, id](const boost::system::error_code& error)
        {
            // @note cancel()後も期限切れの処理が残るため、送信IDで古いタイマーを除外する
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
                                 start_write();
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
    // @note 送信途中で失敗した可能性があるため、未送信データを破棄する
    // @note 通信の再開にはclose()後にopen()を呼ぶ
    {
        std::lock_guard<std::mutex> send_lock(send_mutex_);
        write_queue_.clear();
        write_scheduled_ = false;
    }
    writing_ = false;
}

std::optional<SerialFrame> SerialBridge::read()
{
    if (auto received = read_with_timestamp())
    {
        return std::move(received->data);
    }
    return std::nullopt;
}

std::optional<SerialBridge::ReceivedFrame> SerialBridge::read_with_timestamp()
{
    std::lock_guard<std::mutex> lock(receive_mutex_);
    // @note 通信エラーがあっても、受信済みのパケットを先に返す
    if (received_.empty() && io_error_)
    {
        throw boost::system::system_error(io_error_, "read serial bytes");
    }
    if (received_.empty())
    {
        return std::nullopt;
    }
    auto result = std::move(received_.front());
    received_.pop_front();
    return result;
}

void SerialBridge::start_read()
{
    port_.async_read_some(boost::asio::buffer(buffer_),
                          [this](const boost::system::error_code& error, std::size_t count)
                          {
                              // @note 分割受信ではパケット末尾を受信した回の時刻を使う
                              const auto received_at = count > 0 ? receive_clock_() : 0;
                              const auto packets = parser_.feed(buffer_.data(), count);
                              {
                                  std::lock_guard<std::mutex> lock(receive_mutex_);
                                  for (const auto& packet : packets)
                                  {
                                      if (received_.size() >= rx_queue_size_)
                                      {
                                          received_.pop_front();
                                          ++rx_overflow_;
                                      }
                                      received_.push_back({packet, received_at});
                                  }
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
} // namespace robstride_usb_can_bridge
