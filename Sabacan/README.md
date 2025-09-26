# sabacan - CAN通信ドライバパッケージ (V1/V2/GPIO/Power対応)

## 概要

`sabacan`は、さばね技研が開発した統一CANプロトコルに対応したROS 2ドライバパッケージです。
このパッケージは`ros2_socketcan`と連携し、CANバスを介してロボマス制御基板（V1/V2）、GPIO基板、および電源管理機能との通信を行い、モーター制御、センサーデータの送受信、非常停止（EMS）制御を可能にします。

## パッケージ構成

このリポジトリは主に以下のパッケージで構成されています。

  * `sabacan`: メインのROS 2ノードが含まれるパッケージ。
  * `sabacan_msgs`: カスタムメッセージとサービスの定義が含まれるパッケージ。

### ノード一覧

`sabacan`パッケージは、機能ごとに独立した4つのノードを提供します。

1.  **`sabacan_robomas_node` (旧基板: V1用)**

      * **機能**: 旧ロボマスモーター制御基板(V1)専用ノード。
      * **対応モーター**: Robomas, VESC。
      * **制御モード**: `VELOCITY` (速度制御), `POSITION` (位置制御), `OPEN` (PWM制御)。
      * **モーター数**: 最大4つ。

2.  **`sabacan_robomasv2_node` (新基板: V2用)**

      * **機能**: 新ロボマスモーター制御基板(V2)専用ノード。
      * **特徴**: トルク制御、外乱オブザーバ、各種ゲインの動的変更など、V1より高度な制御が可能です。
      * **対応モーター**: Robomas (C610/C620), VESC。
      * **制御モード**: `TORQUE` (トルク制御), `VELOCITY` (速度制御), `POSITION` (位置制御), VESC各種モード。
      * **モーター数**: 最大4つ。

3.  **`sabacan_gpio_node`**

      * **機能**: GPIO制御基板専用ノード。
      * **対応ピン**: 9ピン。
      * **ピンタイプ**: `PWM_OUT`, `PWM_IN`, `ESC`。
      * **用途**: PWM出力、ESC制御、デジタル入出力。

4.  **`sabacan_power_node`**

      * **機能**: 電源管理および非常停止（EMS）信号をCANバス全体に送信する専用ノード。
      * **用途**: `/sabacan_power_ref` トピックを介して受け取った指令に基づき、接続されている全ての基板に対して一斉に非常停止またはその解除を指示します。

## アーキテクチャ

```
ROS2アプリケーション
        |
        +-- /sabacan_robomas_ref{board_id} (sabacan_msgs/SabacanRobomasRef)
        |
        +-- /sabacan_gpio_ref{board_id} (sabacan_msgs/SabacanGPIORef)
        |
        +-- /sabacan_power_ref (sabacan_msgs/SabacanPowerRef)
        |
        +-- /set_robomas_gains (sabacan_msgs/SetRobomasGains) ... V2用サービス
        |
        +-- /sabacan_robomas_reset (sabacan_msgs/SabacanReset) ... V1/V2/GPIO用リセット
        ↓
    sabacan_robomas_node (V1) / sabacan_robomasv2_node (V2) / sabacan_gpio_node / sabacan_power_node
        |
        +-> /sabacan_robomas_status{board_id} (sabacan_msgs/SabacanRobomasStatus)
        ↓
        /to_can_bus (can_msgs/Frame)
        ↑
        /from_can_bus (can_msgs/Frame)
        ↓
    ros2_socketcan
        ↓ CAN Bus (統一プロトコル)
    さばね基板 (ロボマス制御基板V1/V2, GPIO基板)
```

## 使用方法

### 1\. システム起動

#### CANインターフェース設定

使用するCANインターフェース（例: `can0`）を起動します。通信速度は**1Mbps**です。

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

#### 送信キューのサイズを増やす（推奨）

高負荷時にUSB-CANがハングアップする問題の対策として、送信キューのサイズを増やします。

```bash
sudo ip link set can0 down
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
```

#### ros2\_socketcan起動

`ros2_socketcan`を起動し、ROSトピックとCANバスをブリッジします。

```bash
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0
```

### 2\. パッケージのビルドと実行

#### ビルド

```bash
cd /path/to/ros2_ws
colcon build --packages-select sabacan_msgs sabacan
source install/setup.bash
```

#### ノード起動

目的に応じて必要なノードを起動します。
`board_id`は、ノード起動時に必ず設定してください。起動後にコマンドラインから設定しても正しく反映されません。

```bash
# 【新基板 V2】ロボマス制御ノードを起動
ros2 run sabacan sabacan_robomasv2_node --ros-args -p board_id:=0

# 【旧基板 V1】ロボマス制御ノードを起動
ros2 run sabacan sabacan_robomas_node --ros-args -p board_id:=1

# GPIO制御ノードを起動
ros2 run sabacan sabacan_gpio_node --ros-args -p board_id:=2

# 電源・非常停止ノードを起動
ros2 run sabacan sabacan_power_node
```

### 3\. モーター制御・非常停止の例 (トピック)

#### V2基板: 速度制御

モーター0を2.0 rad/sで回転させます。`control_type`が`VELOCITY`に設定されている必要があります。

```bash
# パラメータで制御モードを設定 (例)
ros2 param set /sabacan_robomasv2_node control_type "['VELOCITY', 'VELOCITY', 'VELOCITY', 'VELOCITY']"
# 指令値を送信
ros2 topic pub /sabacan_robomas_ref0 sabacan_msgs/msg/SabacanRobomasRef '{motor_number: 0, ref: 2.0}'
```

#### 非常停止 (EMS) 制御

CANバスに接続された全てのデバイスに非常停止信号を送ります。

```bash
# 非常停止を有効化
ros2 topic pub /sabacan_power_ref sabacan_msgs/msg/SabacanPowerRef '{is_ems: true}'

# 非常停止を解除
ros2 topic pub /sabacan_power_ref sabacan_msgs/msg/SabacanPowerRef '{is_ems: false}'
```

### 4\. V2基板向けゲイン設定 (サービス)

`sabacan_robomasv2_node`では、サービスを通じて各種ゲインやパラメータを動的に変更できます。

```bash
# モーター0の速度制御ゲインと外乱オブザーバのパラメータを設定
ros2 service call /set_robomas_gains sabacan_msgs/srv/SetRobomasGains '{
  motor_number: 0,
  set_dob_param: true,
  dob_load_j: 0.0006,
  dob_load_d: 0.0005,
  dob_cutoff_freq: 10.0,
  set_speed_gains: true,
  speed_gain_p: 0.6,
  speed_gain_i: 0.3,
  speed_gain_d: 0.0,
  torque_lim: 6.0
}'
```

### 5\. パラメータの初期化 (リセットサービス)

ノード起動後やパラメータ変更後に、基板側の設定を確実に反映させるためにリセットサービスを呼び出すことを推奨します。

```bash
# V2基板ノードのパラメータを再送信して初期化
ros2 service call /sabacan_robomas_reset sabacan_msgs/srv/SabacanReset '{}'
```

## パラメータ設定

### `sabacan_robomasv2_node` (新基板 V2)

多くのパラメータを持ち、柔軟な設定が可能です。

  * `board_id`: ボードID (0-15、必須)。
  * `motor_type`: モーター種別の配列。`"C610"`, `"C620"`, `"VESC"`から選択。
  * `control_type`: 制御モードの配列。
      * DJIモーター用: `"TORQUE"`, `"VELOCITY"`, `"POSITION"`
      * VESC用: `"PWM"`, `"CURRENT"`, `"VELOCITY"`, `"POSITION"`, `"DISABLE"`
  * `dob_en`: 外乱オブザーバの有効/無効 (bool配列)。
  * `abs_enc_en`: アブソリュートエンコーダの有効/無効 (bool配列)。
  * `md_guess_en`: モータパラメータ推定の有効/無効 (bool配列)。
  * `load_j`, `load_d`, `dob_cf`: 外乱オブザーバのパラメータ (double配列)。
  * `speed_gain_p`, `speed_gain_i`, `speed_gain_d`: 速度制御PIDゲイン (double配列)。
  * `torque_lim`: 速度制御時のトルク制限値 (double配列)。
  * `pos_gain_p`, `pos_gain_i`, `pos_gain_d`: 位置制御PIDゲイン (double配列)。
  * `speed_lim`: 位置制御時の速度制限値 (double配列)。
  * `monitor_period`: 状態の送信周期 (ms)。
  * `monitor_reg`: 送信する状態レジスタのビットマスク (int64配列)。

### `sabacan_robomas_node` (旧基板 V1)

  * `board_id`: ボードID (0-15)。
  * `motor_type`: モーター種別の配列。`"Robomas"`, `"VESC"`から選択。
  * `control_type`: 制御モードの配列。`"VELOCITY"`, `"POSITION"`, `"OPEN"`から選択。
  * `speed_gain_p`, `speed_gain_i`, `speed_gain_d`: 速度制御PIDゲイン (double配列)。
  * `pos_gain_p`, `pos_gain_i`, `pos_gain_d`: 位置制御PIDゲイン (double配列)。
  * `speed_lim`: 位置制御時の速度制限値 (double配列)。

### `sabacan_gpio_node`

  * `board_id`: ボードID (0-15)。
  * `pin_type`: 9ピンそれぞれのタイプを配列で指定 (`"PWM_OUT"`, `"PWM_IN"`, `"ESC"`)。

### `sabacan_power_node`

  * このノードには設定可能なパラメータはありません。

## メッセージ & サービス仕様

### メッセージ

  * **`sabacan_msgs/msg/SabacanRobomasRef`**

      * `uint8 motor_number`: モーター番号 (0-3)
      * `float32 ref`: 指令値（制御モードに応じてトルク, 速度, 位置などを指定）

  * **`sabacan_msgs/msg/SabacanRobomasStatus`**

      * `uint8 motor_number`: モーター番号
      * `string motor_type`: モーター種別
      * `string control_type`: 制御モード
      * `bool motor_state`: モーターの状態
      * `float32 torque`: 現在のトルク
      * `float32 speed`: 現在の速度
      * `float32 pos`: 現在の位置
      * `float32 abs_pos`: 絶対位置
      * `float32 abs_speed`: 絶対速度
      * `int32 abs_turn_cnt`: 絶対エンコーダの回転数

  * **`sabacan_msgs/msg/SabacanGPIORef`**

      * `uint8 pin_number`: ピン番号 (0-8)
      * `int32 pwm_period`: PWM周期 (us)
      * `float32 duty`: デューティ比

  * **`sabacan_msgs/msg/SabacanPowerRef`**

      * `bool is_ems`: `true`で非常停止、`false`で解除。

### サービス

  * **`sabacan_msgs/srv/SetRobomasGains` (V2ノード用)**

      * リクエストでモーター番号と各種ゲイン値を指定し、動的に設定を変更できます。

  * **`sabacan_msgs/srv/SabacanReset`**

      * 引数なしで呼び出すと、ノードが持つ現在のパラメータを基板に再送信し、状態を初期化します。

## 依存関係

  * **ROS 2 Packages**: `rclcpp`, `can_msgs`, `std_msgs`, `sabacan_msgs`。
  * **External Packages**: `ros2_socketcan`。

## トラブルシューティング

  * **CANインターフェースが見つからない**: `ip link show`で`can0`等が存在するか確認してください。
  * **モーターが動かない**:
    1.  `board_id`が基板のDIPスイッチ設定と一致しているか確認してください。
    2.  `ros2 topic echo /to_can_bus`でCANフレームが送信されているか確認してください。
    3.  `sabacan_robomas_reset`サービスを呼び出して、パラメータを再初期化してみてください。
  * **大量送信で通信が途絶える**: 上記「送信キューのサイズを増やす」を試してください。それでも解決しない場合は、USBポートの変更やセルフパワーのUSBハブの使用を検討してください。