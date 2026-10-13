#pragma once

#include "robstride_usb_can_bridge/serial_frame.hpp"

#include <boost/asio.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <optional>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace robstride_usb_can_bridge
{
/**
 * @brief ATパケットの送受信を専用スレッドで処理する
 * @note CANフレームとの変換はprotocol.hppの関数で行う
 */
class SerialBridge
{
  public:
    /// @brief 受信が完了した時刻をナノ秒で返す関数（送受信スレッドから呼ぶため例外を投げないこと）
    using ReceiveClock = std::function<int64_t()>;

    /** @brief ATパケットと、パケット末尾を受信した時刻 */
    struct ReceivedFrame
    {
        SerialFrame data;
        int64_t received_at_ns{0};
    };

    /** @brief キューの現在件数と、接続開始以降の破棄件数 */
    struct QueueStatistics
    {
        std::size_t tx_pending{0};
        std::size_t rx_pending{0};
        uint64_t tx_overflow{0};
        uint64_t tx_expired{0};
        uint64_t rx_overflow{0};
    };

    /**
     * @brief シリアル接続の設定を保持する（ポートはまだ開かない）
     * @param device シリアルデバイスのパス
     * @param baud_rate シリアル通信速度
     * @param write_timeout 各パケットの送信タイムアウト
     * @param tx_queue_size 送信待ちの最大件数（送信中の1件は含まない）
     * @param rx_queue_size 未読の受信パケットの最大件数
     * @param tx_queue_max_age 送信受付から送信開始までの有効期限
     * @param receive_clock 受信時刻を取得する関数（未指定ならシステム時刻）
     * @throws std::invalid_argument deviceが空、または速度・件数・時間が正でない場合
     */
    SerialBridge(const std::string& device, unsigned int baud_rate,
                 std::chrono::milliseconds write_timeout = std::chrono::milliseconds(100),
                 std::size_t tx_queue_size = 100, std::size_t rx_queue_size = 100,
                 std::chrono::milliseconds tx_queue_max_age = std::chrono::milliseconds(100),
                 ReceiveClock receive_clock = {});
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
     * @note 期限切れと上限超過時は古い送信待ちを破棄し、送信中のパケットは維持する
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
    /**
     * @brief 受信時刻付きで1パケット取り出す
     * @return 受信パケットと保存した時刻、キューが空ならstd::nullopt
     * @throws boost::system::system_error 受信キューが空で通信エラーがある場合
     * @note 時刻はシリアル受信完了時に保存し、キューから取り出しても変更しない
     */
    std::optional<ReceivedFrame> read_with_timestamp();
    /// @brief キュー件数と破棄件数を取得する（open()で破棄件数をリセット）
    QueueStatistics queue_statistics() const;

  private:
    struct PendingWrite
    {
        SerialFrame data;
        std::chrono::steady_clock::time_point queued_at;
    };
    /// @brief 期限切れの送信待ちを破棄する（send_mutex_を取得して呼ぶ）
    void discard_expired_writes(std::chrono::steady_clock::time_point now);
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
    std::size_t tx_queue_size_;
    std::size_t rx_queue_size_;
    std::chrono::milliseconds tx_queue_max_age_;
    ReceiveClock receive_clock_;
    boost::asio::io_context io_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_;
    boost::asio::serial_port port_;
    boost::asio::steady_timer write_timer_;
    std::thread thread_;
    /// @brief ポートの開閉と送信の受付を直列化するロック
    std::mutex lifecycle_mutex_;
    std::atomic<bool> accepting_{false};
    mutable std::mutex send_mutex_;
    std::deque<PendingWrite> write_queue_;
    /// @brief 起動通知の重複を防ぎ、Asioの通知キューも増え続けないようにする
    bool write_scheduled_{false};
    uint64_t tx_overflow_{0};
    uint64_t tx_expired_{0};
    // @note 以下の送信状態は送受信スレッド、またはその終了後にだけ操作する
    SerialFrame active_write_{};
    bool writing_{false};
    std::size_t write_id_{0};
    std::array<uint8_t, 4096> buffer_{};
    mutable std::mutex receive_mutex_;
    SerialFrameParser parser_;
    std::deque<ReceivedFrame> received_;
    uint64_t rx_overflow_{0};
    boost::system::error_code io_error_;
};
} // namespace robstride_usb_can_bridge
