# `sabacan_led_node` の使い方

`sabacan_led_node` は、LED基板（RGBLED: WS2812 ×3）をROS 2で制御するためのノードです。
LEDのモード切替（NORMAL/EMG）と、LEDの発光指令（範囲指定 + RGB）に対応しています。

## 1. ノードの起動

ノードを起動する際は、ROSパラメータとして `board_id` を**必ず指定**する必要があります。

```bash
# ros2_ws でビルドとソース
cd /path/to/ros2_ws
colcon build --packages-select sabacan_msgs sabacan
source install/setup.bash

# ノード起動 (board_id=0 の例)
ros2 run sabacan sabacan_led_node --ros-args -p board_id:=0
````

## 2. パラメータ設定

ノード起動時、または `ros2 param set` コマンドでパラメータを設定できます。
各パラメータ送信ごとに約10msのディレイを入れてあります。

主要なパラメータ:
  * `board_id` (int64, **必須**): 基板のCAN ID (0〜9)。
  * `enable_auto_transition` (bool, デフォルト: true): 電源基板の状態（`PCU_STATE`）でEMGへ自動遷移する機能の有効/無効。
  * `emg_blink_period` (int64, デフォルト: 0): 非常停止時の点滅周期 [ms]。0の場合は常時点灯。
  * `emg_color` (int64配列, デフォルト: `[0x40, 0x00, 0x00]`): 非常停止時の色 `[R,G,B]`（各0〜255）。3つのRGBLEDに同じ色を設定します。
  * `enable_monitor_period` (bool, デフォルト: true): 周期モニタの有効/無効。
  * `monitor_period` (int64, デフォルト: 50): モニタ周期 [ms]。
  * `monitor_reg1` (int64, デフォルト: 0): GPIO用モニタのビットマスク（reg ID 0x00〜0x3F）。
  * `monitor_reg2` (int64, デフォルト: 0): LED用モニタのビットマスク（reg ID 0x40〜0x7F）。

**例: 起動時にEMG点滅を有効化（赤、500ms点滅）**
```bash
ros2 run sabacan sabacan_led_node --ros-args -p board_id:=0 \
  -p emg_color:="[255, 0, 0]" \
  -p emg_blink_period:=500
```

**例: 起動後に点滅周期を変更**
```bash
ros2 param set /sabacan_led_node emg_blink_period 200
```

## 3. LEDモード切替 (トピック)

  * トピック: `/sabacan_led_mode<board_id>`
  * メッセージ: `sabacan_msgs/msg/SabacanLEDMode`
    * `led_mode` (uint8)
      * `0`: NORMAL
      * `1`: EMG

**例 (board_id=0 をEMGモードにする):**
```bash
ros2 topic pub --once /sabacan_led_mode0 sabacan_msgs/msg/SabacanLEDMode "{led_mode: 1}"
```

## 4. LED発光指令 (トピック)

  * トピック: `/sabacan_led_ref<board_id>`
  * メッセージ: `sabacan_msgs/msg/SabacanLEDRef`
    * `pin_number` (uint8): LED番号（0〜2）
    * `start` (uint8): 先頭LEDインデックス（0始まり）
    * `length` (uint8): 光らせるLED数（全体なら `255`）
    * `r,g,b` (uint8): RGB（各0〜255）

**例 (board_id=0、pin_number=0、全LEDを赤):**
```bash
ros2 topic pub --once /sabacan_led_ref0 sabacan_msgs/msg/SabacanLEDRef \
"{pin_number: 0, start: 0, length: 255, r: 255, g: 0, b: 0}"
```

**例2 (board_id=0、pin_number=0、前から10から15個目のLEDを青にする):**
前から10個目は0インデックスだと9になる。  
また、指令値は16進数表記にも対応している。
```bash
ros2 topic pub --once /sabacan_led_ref0 sabacan_msgs/msg/SabacanLEDRef \
"{pin_number: 0, start: 9, length: 5, r: 0x00, g: 0xff, b: 0x00}"
```


## 5. リセット (サービス)

パラメータ一式を基板へ再送したいときはリセットサービスを呼び出します。

  * サービス: `/sabacan_led_reset<board_id>`
  * 型: `sabacan_msgs/srv/SabacanReset`

**例 (board_id=0 をリセット):**
```bash
ros2 service call /sabacan_led_reset sabacan_msgs/srv/SabacanReset "{}"
```

## 6. CANトピック

下記は共通です。
  * publish: `/to_can_bus` (`can_msgs/msg/Frame`)
  * subscribe: `/from_can_bus` (`can_msgs/msg/Frame`)

