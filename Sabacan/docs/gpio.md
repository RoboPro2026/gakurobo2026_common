# `sabacan_gpio_node` の使い方

`sabacan_gpio_node` は、GPIO制御基板をROS 2で制御するためのノードです。1つの基板で9つのGPIOを操作することができます。出力はPWMやESC駆動用パルスなどに対応しています。

## 1. ノードの起動

ノードを起動する際は、ROSパラメータとして `board_id` を**必ず指定**する必要があります。

> **補足**  
> GPIO基板でも初期化時に複数パラメータを連続送信すると稀に設定漏れが起きたため、`sabacan_gpio_node` では各パラメータ送信ごとに約10msのディレイを入れるようになりました。起動がわずかに遅くなりますが、安定化のための仕様です。また、delayの間は処理が止まるので注意。

```bash
# ros2_ws でビルドとソース
cd /path/to/ros2_ws
colcon build --packages-select sabacan_msgs sabacan
source install/setup.bash

# ノード起動 (board_id=2 の例)
ros2 run sabacan sabacan_gpio_node --ros-args -p board_id:=2
````

## 2\. パラメータ設定

ノード起動時、または `ros2 param set` コマンドでパラメータを設定できます。

**例: ピン0をPWM出力 (1kHz)、ピン8をESC出力、その他を入力に設定**

```bash
ros2 run sabacan sabacan_gpio_node --ros-args \
  -p board_id:=2 \
  -p pin_type:="['OUTPUT_PWM','INPUT','INPUT','INPUT','INPUT','INPUT','INPUT','INPUT','OUTPUT_ESC']" \
  -p pwm_freq:="[1000, 0, 0, 0, 0, 0, 0, 0, 0]"
```

**主要なパラメータ:**
プログラムの都合上、パラメータを変更した際に、一瞬だけ出力が止まったり、不安定な挙動になる可能性があるので注意。  
なにか特別な理由がない限り、pin_typeやpwm_freqは動作中に変更するという運用はあまりしないほうがいいかも
  * `board_id` (int64, **必須**): 基板のCAN ID (0〜9)。
  * `pin_type` (string配列, デフォルト: 全て `"INPUT"`):
      * `"INPUT"`: デジタル入力。
      * `"OUTPUT_PWM"`: PWM出力。
      * `"OUTPUT_ESC"`: ESC制御。
      * `"OUTPUT_SERVO"`: サーボモーター制御。
  * `pwm_freq` (int64配列, デフォルト: 全て `0`): PWM周波数 (Hz)。`OUTPUT_PWM`/`OUTPUT_SERVO` では **0より大きい値** が必要。
  * `monitor_period` (int, デフォルト: 50): 入力状態のパブリッシュ周期 (ms)。
  * `enable_monitor_period` (bool, デフォルト: true): 周期パブリッシュの有効/無効。
  * `monitor_reg` (int64): 基板に周期送信を要求するレジスタのビットマスク。

## 3\. ピンへの指令 (トピック)

ピンのモード (`pin_type`) に応じて、適切なトピックにメッセージを送信します。

  * **PWM出力 (`OUTPUT_PWM`) / サーボ (`OUTPUT_SERVO`)**

      * トピック: `/sabacan_gpio_ref_float<board_id>`
      * メッセージ: `sabacan_msgs/msg/SabacanGPIORefFloat`
          * `pin_number` (uint8): ピン番号 (0-8)
          * `ref_float` (float32): 指令値 (PWMデューティ比 0.0〜1.0)
      * **例 (board\_id=2, ピン0, デューティ比 0.5):**
        ```bash
        ros2 topic pub /sabacan_gpio_ref_float2 sabacan_msgs/msg/SabacanGPIORefFloat '{pin_number: 0, ref_float: 0.5}'
        ```

  * **ESC出力 (`OUTPUT_ESC`)**

      * トピック: `/sabacan_gpio_ref_int<board_id>`
      * メッセージ: `sabacan_msgs/msg/SabacanGPIORefInt`
          * `pin_number` (uint8): ピン番号 (0-8)
          * `ref_int` (int32): 指令値 (ESC値 50〜100)
            * ただし、使うESCによっては値の範囲が変わることがあるので注意。
      * **例 (board\_id=2, ピン8, ESC値 75):**
        ```bash
        ros2 topic pub /sabacan_gpio_ref_int2 sabacan_msgs/msg/SabacanGPIORefInt '{pin_number: 8, ref_int: 75}'
        ```

## 4\. 入力状態の確認 (トピック)

`INPUT` モードのピンの状態は、`/sabacan_gpio_status<board_id>` トピックで `SabacanGPIOStatus` メッセージとして配信されます。

  * **メッセージ:**
      * `pin_number` (uint8): ピン番号
      * `input` (bool): 入力状態 (true/false)
  * **例 (board\_id=2):**
    ```bash
    ros2 topic echo /sabacan_gpio_status2
    ```

## 5\. パラメータの再初期化 (サービス)

`/sabacan_gpio_reset` サービス (型: `sabacan_msgs/srv/SabacanReset`) を呼び出すと、現在のROSパラメータが基板に再送信されます。

```bash
ros2 service call /sabacan_gpio_reset sabacan_msgs/srv/SabacanReset '{}'
```
