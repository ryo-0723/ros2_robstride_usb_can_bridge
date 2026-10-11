# serial_can_bridge

RobStride USB-CANのATモードと `can_msgs/msg/Frame` を変換するROS 2ノードです。
拡張データフレーム・DLC 8に対応し、シリアルパケットは17バイト固定です。
標準ID、RTR、エラーフレーム、DLCが8以外のフレームには対応していません。
パッケージ名・実行ファイル名・既定のノード名は `serial_can_bridge` です。

## ビルド

ROS 2、colcon、rosdepを導入し、ROS 2の環境を読み込んでから実行してください。
主な依存パッケージは `rclcpp`、`can_msgs`、Boostです。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ryo-0723/ros2_serial_can_bridge.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select serial_can_bridge
source install/setup.bash
```

## 接続の準備

アダプタをATモードにし、CAN側のビットレートを接続先に合わせて設定してください。
このノードはモード切り替えやCAN側のビットレート設定を行いません。
`baud_rate` はPCとアダプタ間のシリアル通信速度です。

CAN配線と終端抵抗を確認し、シリアルデバイスへの読み書き権限を用意してください。
デバイス名が変わる場合は、`/dev/serial/by-id/` 以下のパスを `device` に指定できます。

## 起動

YAMLを使わず、ノードの既定値で起動できます。

```bash
ros2 run serial_can_bridge serial_can_bridge
ros2 launch serial_can_bridge serial_can_bridge.launch.py
```

YAMLを指定する場合は次のように起動します。

```bash
ros2 launch serial_can_bridge serial_can_bridge.launch.py \
  params_file:="$(ros2 pkg prefix serial_can_bridge)/share/serial_can_bridge/config/serial_bridge.yaml"
```

launch引数でYAMLの一部だけを上書きできます。優先順位はlaunch引数、YAML、ノードの既定値です。

```bash
ros2 launch serial_can_bridge serial_can_bridge.launch.py \
  params_file:="$(ros2 pkg prefix serial_can_bridge)/share/serial_can_bridge/config/serial_bridge.yaml" \
  device:=/dev/ttyUSB1
```

直接実行でもROSのパラメータ引数を使えます。

```bash
ros2 run serial_can_bridge serial_can_bridge --ros-args -p device:=/dev/ttyUSB1
ros2 run serial_can_bridge serial_can_bridge --ros-args \
  --params-file "$(ros2 pkg prefix serial_can_bridge)/share/serial_can_bridge/config/serial_bridge.yaml"
```

## トピック

どちらもメッセージ型は `can_msgs/msg/Frame` です。

| トピック | ノードの動作 |
|---|---|
| `to_can_bus` | 購読したフレームをCANへ送信 |
| `from_can_bus` | CANから受信したフレームを配信 |

起動後、別のターミナルで環境を読み込むと受信内容を確認できます。

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /from_can_bus
```

送信例です。IDとデータは接続先の仕様に合わせて変更してください。

```bash
ros2 topic pub --once /to_can_bus can_msgs/msg/Frame \
  '{id: 1, is_extended: true, is_rtr: false, is_error: false, dlc: 8, data: [0, 0, 0, 0, 0, 0, 0, 0]}'
```

## パラメータ

| 名前 | 既定値 | 内容 |
|---|---|---|
| `device` | `/dev/ttyUSB0` | シリアルデバイス |
| `baud_rate` | `921600` | シリアル通信のボーレート |
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

## テスト

ビルド後、ワークスペースのルートで実行します。

```bash
colcon test --packages-select serial_can_bridge
colcon test-result --verbose
```

テストはパケット変換、疑似端末でのシリアル通信、ROSトピックとの連携を確認します。
疑似端末のテストはLinux向けで、実機アダプタは不要です。実機での通信確認は別途必要です。

## ライセンス・問い合わせ

MITライセンスです。詳細は [LICENSE](LICENSE) を参照してください。
不具合の報告は [Issues](https://github.com/ryo-0723/ros2_serial_can_bridge/issues) へお願いします。
使用したOS、ROS 2のバージョン、アダプタの型番、実行コマンド、エラーログを添えてください。
