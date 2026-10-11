# serial_can_bridge

RobStride USB-CANのATモードと `can_msgs/msg/Frame` を変換するROS 2ノードです。
拡張データフレーム・DLC 8に対応し、シリアルパケットは17バイト固定です。
パッケージ名・実行ファイル名・既定のノード名は `serial_can_bridge` です。

## 起動

コンテナ内でビルドして環境を読み込みます。

```bash
cd /ros2_ws
make build PACKAGES=serial_can_bridge
source install/setup.bash
```

YAMLを使わず、ノードの既定値で起動できます。

```bash
ros2 run serial_can_bridge serial_can_bridge
ros2 launch serial_can_bridge serial_can_bridge.launch.py
```

YAMLを指定する場合は次のように起動します。

```bash
ros2 launch serial_can_bridge serial_can_bridge.launch.py \
  params_file:=/ros2_ws/src/serial_can_bridge/config/serial_bridge.yaml
```

launch引数でYAMLの一部だけを上書きできます。優先順位はlaunch引数、YAML、ノードの既定値です。

```bash
ros2 launch serial_can_bridge serial_can_bridge.launch.py \
  params_file:=/ros2_ws/src/serial_can_bridge/config/serial_bridge.yaml \
  device:=/dev/ttyUSB1
```

直接実行でもROSのパラメータ引数を使えます。

```bash
ros2 run serial_can_bridge serial_can_bridge --ros-args -p device:=/dev/ttyUSB1
ros2 run serial_can_bridge serial_can_bridge --ros-args \
  --params-file /ros2_ws/src/serial_can_bridge/config/serial_bridge.yaml
```

## パラメータ

| 名前 | 既定値 | 内容 |
|---|---|---|
| `device` | `/dev/ttyUSB0` | シリアルデバイス |
| `baud_rate` | `921600` | ボーレート |
| `tx_topic` | `to_can_bus` | CAN送信メッセージの購読先 |
| `rx_topic` | `from_can_bus` | CAN受信メッセージの配信先 |
| `frame_id` | `can` | 受信メッセージの `header.frame_id` |
| `poll_period` | `0.001` | 受信キューを確認する間隔（秒） |
| `write_timeout` | `0.1` | 各パケットの送信開始からのタイムアウト（秒） |

パラメータは起動時に読み込み、稼働中の変更はできません。
受信メッセージの時刻はノードが取り出した時刻です。QoSはReliable、履歴100件です。
送信はキューへの追加だけで戻り、受信コールバックは1回につき最大100フレームを配信します。
不正なCANフレームは警告して破棄します。デバイスを開けない場合、通信エラー、送信タイムアウト時はエラー終了します。自動再接続は未実装です。

現在のノードには送信間隔の調整機能はありません。実機のアダプタで連続送信時の取りこぼしがないかは別途確認が必要です。
