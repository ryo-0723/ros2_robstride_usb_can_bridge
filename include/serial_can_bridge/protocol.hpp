#pragma once

#include "serial_can_bridge/serial_frame.hpp"

#include <can_msgs/msg/frame.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace serial_can_bridge
{
/**
 * @brief CANフレームをRobStride USB-CANのATパケットへ変換する
 * @param frame 拡張データフレーム（29ビットID、DLC 0〜8）
 * @return AT + 変換済みID（ビッグエンディアン）+ DLC + データ + CRLF
 * @throws std::invalid_argument ID・DLCが範囲外、または標準・RTR・エラーフレームの場合
 */
SerialFrame encode_frame(const can_msgs::msg::Frame& frame);

/**
 * @brief ATパケットを検証してCANフレームへ変換する
 * @param bytes 1フレーム分のATパケット
 * @return CANフレーム、不正なパケットならstd::nullopt
 * @note データ配列の未使用部分はゼロ埋めし、ROSヘッダーは呼び出し側で設定する
 */
std::optional<can_msgs::msg::Frame> decode_frame(const SerialFrame& bytes);

/**
 * @brief シリアル受信データからCANフレームを取り出す
 * @note 接続ごとに1インスタンスを用意し、同じスレッドから呼び出す
 */
class FrameDecoder
{
  public:
    /**
     * @brief 不正なデータを読み飛ばし、完成したフレームを返す
     * @param bytes 今回受信したデータ
     * @return CANフレームの一覧（ROSヘッダーは呼び出し側で設定）
     * @note 未完成のパケットは次の呼び出しまで保持する
     */
    std::vector<can_msgs::msg::Frame> decode(const std::vector<uint8_t>& bytes);
    /// @brief 再接続に備えて未完成のパケットを破棄する
    void reset();

  private:
    SerialFrameParser parser_;
};
} // namespace serial_can_bridge
