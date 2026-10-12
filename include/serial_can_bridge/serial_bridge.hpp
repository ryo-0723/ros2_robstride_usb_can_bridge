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
/**
 * @brief ATパケットの送受信を専用スレッドで処理する
 * @note CANフレームとの変換はprotocol.hppの関数で行う
 */
class SerialBridge
{
  public:
    /**
     * @brief シリアル接続の設定を保持する（ポートはまだ開かない）
     * @param device シリアルデバイスのパス
     * @param baud_rate シリアル通信速度
     * @param write_timeout 各パケットの送信タイムアウト
     * @throws std::invalid_argument deviceが空、または速度・タイムアウトが正でない場合
     */
    SerialBridge(const std::string& device, unsigned int baud_rate,
                 std::chrono::milliseconds write_timeout = std::chrono::milliseconds(100));
    /// @brief 通信を停止してポートを閉じる
    ~SerialBridge();
    SerialBridge(const SerialBridge&) = delete;
    SerialBridge& operator=(const SerialBridge&) = delete;

    /**
     * @brief 8N1・フロー制御なしでポートを開き、送受信スレッドを起動する
     * @throws std::logic_error すでにポートが開いている場合
     * @throws boost::system::system_error ポートのオープンや設定に失敗した場合
     * @note 前回の受信データと通信エラーを消去する
     */
    void open();
    /// @brief 送受信スレッドを停止し、ポートと未送信キューを解放する
    void close();
    /**
     * @brief パケットを送信キューへ追加し、送信完了を待たずに戻る
     * @param data 検証済みのATパケット
     * @throws std::invalid_argument パケットが不正な場合
     * @throws std::logic_error ポートが開いていない場合
     * @throws boost::system::system_error 通信エラーが記録されている場合
     * @note タイムアウトはキュー待ちを含まず、各パケットの送信開始から計測する
     */
    void write(SerialFrame data);
    /**
     * @brief 最初に発生した非同期通信エラーを取得する
     * @return 記録されたエラー（未発生なら空のエラーコード）
     * @note エラーはopen()で消去する
     */
    boost::system::error_code error() const;
    /**
     * @brief 受信キューから1パケット取り出し、受信待ちは行わない
     * @return 検証済みのATパケット、受信キューが空ならstd::nullopt
     * @throws boost::system::system_error 受信キューが空で通信エラーがある場合
     */
    std::optional<SerialFrame> read();

  private:
    /// @brief 次の非同期受信を開始する
    void start_read();
    /// @brief 送信キューの先頭を取り出し、タイムアウト付きで送信する
    void start_write();
    /**
     * @brief 最初のエラーを記録し、通信と未送信キューを停止する
     * @param error 発生した通信エラー
     */
    void fail(const boost::system::error_code& error);

    std::string device_;
    unsigned int baud_rate_;
    std::chrono::milliseconds write_timeout_;
    boost::asio::io_context io_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_;
    boost::asio::serial_port port_;
    boost::asio::steady_timer write_timer_;
    std::thread thread_;
    /// @brief ポートの開閉と送信の受付を直列化するロック
    std::mutex lifecycle_mutex_;
    std::atomic<bool> accepting_{false};
    // @note 以下の送信状態は送受信スレッド、またはその終了後にだけ操作する
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
