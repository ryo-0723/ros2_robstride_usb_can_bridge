# robstride_usb_can_bridge

RobStride社が販売するUSB-CANアダプタのAT形式と、ROS 2の `can_msgs/msg/Frame` を変換するノード

SLCANには非対応
SocketCANの `can0` などは使用せず、`/dev/ttyUSB0` などのシリアルデバイスへ直接接続

- 対応：拡張データフレーム、DLC 0〜8（シリアルパケットは9 + DLCバイト）
- 非対応：標準ID、RTR、エラーフレーム、DLCが8を超えるフレーム
- パッケージ名・実行ファイル名・既定のノード名：`robstride_usb_can_bridge`

## ビルド

ROS 2、colcon、rosdepを導入し、ROS 2の環境を読み込んでから実行
主な依存パッケージ：`rclcpp`、`can_msgs`、Boost

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ryo-0723/ros2_serial_can_bridge.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select robstride_usb_can_bridge
source install/setup.bash
```

## 接続の準備

- アダプタをATモードにし、CAN側のビットレートを接続先に合わせて設定
- CAN配線・終端抵抗・シリアルデバイスへの読み書き権限を確認
- デバイス名が変わる場合は、`device` に `/dev/serial/by-id/` 以下のパスを指定

モード切り替えとCAN側のビットレート設定はアダプタ側で行います
`baud_rate` はPCとアダプタ間のシリアル通信速度

## 起動

既定値で起動する場合

```bash
ros2 run robstride_usb_can_bridge robstride_usb_can_bridge
ros2 launch robstride_usb_can_bridge robstride_usb_can_bridge.launch.py
```

YAMLを指定する場合

```bash
ros2 launch robstride_usb_can_bridge robstride_usb_can_bridge.launch.py \
  params_file:="$(ros2 pkg prefix robstride_usb_can_bridge)/share/robstride_usb_can_bridge/config/robstride_usb_can_bridge.yaml"
```

launch引数でYAMLの値を上書きする場合（優先順位：launch引数 > YAML > ノードの既定値）

```bash
ros2 launch robstride_usb_can_bridge robstride_usb_can_bridge.launch.py \
  params_file:="$(ros2 pkg prefix robstride_usb_can_bridge)/share/robstride_usb_can_bridge/config/robstride_usb_can_bridge.yaml" \
  device:=/dev/ttyUSB1
```

直接実行でパラメータを指定する場合

```bash
ros2 run robstride_usb_can_bridge robstride_usb_can_bridge --ros-args -p device:=/dev/ttyUSB1
ros2 run robstride_usb_can_bridge robstride_usb_can_bridge --ros-args \
  --params-file "$(ros2 pkg prefix robstride_usb_can_bridge)/share/robstride_usb_can_bridge/config/robstride_usb_can_bridge.yaml"
```

## トピック

メッセージ型：`can_msgs/msg/Frame`

| トピック | ノードの動作 |
|---|---|
| `to_can_bus` | 購読したフレームをCANへ送信 |
| `from_can_bus` | CANから受信したフレームを配信 |

受信確認（ノード起動後、別のターミナルで実行）

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /from_can_bus
```

送信例（IDとデータは接続先の仕様に合わせて変更）
`dlc` が有効なデータ長で、`data` は常に8要素
たとえばDLC 4なら先頭4バイトだけを送信し、受信時は残り4バイトをゼロで埋めます

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
| `tx_queue_size` | `100` | 送信待ちの上限件数（送信中の1件を除く、1以上） |
| `rx_queue_size` | `100` | 未読の受信キューの上限件数（1以上） |
| `tx_queue_max_age` | `0.1` | 送信受付から送信開始までの有効期限（秒、0.001〜60） |

- パラメータは起動時に読み込み、稼働中の変更は不可
- `header.stamp` はパケット末尾をPC側でシリアル受信した時刻（ノードのROS時計を使用）
- 受信キューで待機しても時刻は保持し、一括受信したフレームには同じ時刻を設定
- QoSはReliable、履歴100件
- 送信はキューへの追加後に戻り、受信はコールバック1回につき最大100フレームを配信
- 送受信キューが上限に達した場合は最も古い待機フレームを破棄
- 送信待ちの期限切れは、送信受付時と次の送信開始時に破棄
- 破棄件数は警告ログで確認可能（送信上限・送信期限・受信上限の累計）
- 不正なCANフレームは警告して破棄
- デバイスを開けない場合、通信エラー、送信タイムアウト時はエラー終了（自動再接続なし）

送信間隔の調整機能は未実装のため、連続送信時の取りこぼしは実機で確認が必要
キューの破棄は未送信データのみが対象で、送信中やOS・アダプタ側のバッファに渡したデータは取り消しません

## テスト

ビルド後、ワークスペースのルートで実行

```bash
colcon test --packages-select robstride_usb_can_bridge
colcon test-result --verbose
```

確認内容：パケット変換、疑似端末でのシリアル通信、ROSトピックとの連携

疑似端末のテストはLinux向けで、実機アダプタは不要
実機での通信確認は別途必要

## ライセンス・問い合わせ

ライセンス：MIT（[LICENSE](LICENSE)）

不具合の報告先：[Issues](https://github.com/ryo-0723/ros2_serial_can_bridge/issues)
OS、ROS 2のバージョン、アダプタの型番、実行コマンド、エラーログを添えてください
