#pragma once

#include "serial_can_bridge/serial_bridge.hpp"

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace serial_can_bridge
{
/// @brief ROSのCANトピックとシリアル通信を中継するノード
class SerialCanBridgeNode : public rclcpp::Node
{
  public:
    /**
     * @brief 起動時パラメータを読み込み、トピックとシリアル接続を初期化する
     * @param options ノード名やパラメータの設定
     * @throws std::invalid_argument パラメータが許容範囲外の場合
     * @throws boost::system::system_error シリアル接続に失敗した場合
     */
    explicit SerialCanBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  private:
    /**
     * @brief CANフレームを変換し、不正なフレームは警告して破棄する
     * @param frame 送信トピックから受け取ったCANフレーム
     */
    void transmit(const can_msgs::msg::Frame& frame);
    /// @brief 受信キューから最大100フレームを取り出し、時刻とframe_idを付けて配信する
    void receive();

    std::string frame_id_;
    std::unique_ptr<SerialBridge> serial_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace serial_can_bridge
