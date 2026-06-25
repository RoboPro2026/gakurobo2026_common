# `sabacan_gpio_node` の使い方

`sabacan_gpio_node` は、GPIO制御基板をROS 2で制御するためのノードです。1つの基板で9つのGPIOを操作することができます。出力はPWMやESC駆動用パルスなどに対応しています。

## 1. ノードの起動

ノードを起動する際は、ROSパラメータとして `board_id` を**必ず指定**する必要があります。

> **補足**  
> 初期化時に複数パラメータを連続送信すると設定漏れが起きるため、`sabacan_gpio_node` では各パラメータ送信ごとに約10msの遅延を入れています。起動に数百ミリ秒余分にかかりますが、通信を安定させるための仕様です。また、delayの間は処理が止まるので注意が必要です。

```bash
# ros2_ws でビルドとソース
cd /path/to/ros2_ws
colcon build --packages-select sabacan_msgs sabacan
source install/setup.bash

# ノード起動 (board_id=2 の例)
ros2 run sabacan sabacan_gpio_node --ros-args -p board_id:=2
````

> **補足**
> - 本ノードは `/to_can_bus` へ CAN フレームを publish し、`/from_can_bus` を subscribe します。CAN ブリッジ（例: socketcan ブリッジ）が別途必要です。

## 2\. パラメータ設定

ノード起動時、または `ros2 param set` コマンドでパラメータを設定できます。

**例: ピン0をPWM出力 (1kHz)、ピン1を180度サーボ、ピン2を270度サーボ、ピン8をESC出力、その他を入力に設定**

```bash
ros2 run sabacan sabacan_gpio_node --ros-args \
  -p board_id:=2 \
  -p pin_type:="['OUTPUT_PWM','OUTPUT_SERVO','OUTPUT_SERVO','INPUT','INPUT','INPUT','INPUT','INPUT','OUTPUT_ESC']" \
  -p pwm_freq:="[1000, 0, 0, 0, 0, 0, 0, 0, 0]" \
  -p servo_max_angle:="[180, 270, 180, 180, 180, 180, 180, 180, 180]"
```

**主要なパラメータ:**
プログラムの都合上、パラメータを変更した際に、一瞬だけ出力が止まったり、不安定な挙動になる可能性があるので注意。  
なにか特別な理由がない限り、pin_typeやpwm_freqは動作中に変更するという運用はあまりしないほうがいいかも
  * `board_id` (int64, **必須**): 基板のCAN ID (0〜9)。
  * `enable_initialize`（bool、デフォルト：`true`） ノード起動時にCANの初期化データを送信するか選択する。
  * `publish_timer_rate` (double, デフォルト: `100.0`): `/sabacan_gpio_status<board_id>` を周期送信するレート [Hz]。
  * `pin_type` (string配列, デフォルト: 全て `"INPUT"`):
      * `"INPUT"`: デジタル入力。
      * `"OUTPUT_PWM"`: PWM出力。
      * `"OUTPUT_ESC"`: ESC制御。
      * `"OUTPUT_SERVO_SG90"`: サーボモーター制御。
  * `pwm_freq` (int64配列, デフォルト: 全て `0`): PWM周波数 (Hz)。`OUTPUT_PWM`で使用する。`OUTPUT_SERVO_SG90`のときは、sabacan_gpio_node内で、自動で50Hzに設定される。  
  * `servo_min_angle`[deg]: サーボの最小角度（0以上の整数値）。int64配列で初期値0。
  * `servo_max_angle`[deg]: サーボの最大角度（0以上の整数値）。int64配列で初期値180。270度サーボの場合は270にする。
  * `servo_min_pulse_width`[us]: サーボの最小角度のときのパルス幅。int64配列で初期値500us。使うサーボによって微妙に異なるので、事前に確認すること。
  * `servo_max_pulse_width`[us]: サーボの最大角度のときのパルス幅。int64配列で初期値2500us。使うサーボによって微妙に異なるので、事前に確認すること。
  * `monitor_period` (int, デフォルト: 50): 入力状態のパブリッシュ周期 (ms)。
  * `enable_monitor_period` (bool, デフォルト: true): 周期パブリッシュの有効/無効。
  * `monitor_reg` (int64): 基板に周期送信を要求するレジスタのビットマスク。

## 3\. ピンへの指令 (トピック)

ピンのモード (`pin_type`) に応じて、適切なトピックにメッセージを送信します。

  * **PWM出力 (`OUTPUT_PWM`)**

      * トピック: `/sabacan_gpio_ref_float<board_id>`
      * メッセージ: `sabacan_msgs/msg/SabacanGPIORefFloat`
          * `pin_number` (uint8): ピン番号 (0-8)
          * `ref_float` (float32): 指令値 (PWMデューティ比 0.0〜1.0)
      * **例 (board\_id=2, ピン0, デューティ比 0.5):**
        ```bash
        ros2 topic pub /sabacan_gpio_ref_float2 sabacan_msgs/msg/SabacanGPIORefFloat '{pin_number: 0, ref_float: 0.5}'
        ```


  * **サーボ出力 (`OUTPUT_SERVO`)**

      * トピック: `/sabacan_gpio_ref_int<board_id>`
      * メッセージ: `sabacan_msgs/msg/SabacanGPIORefInt`
          * `pin_number` (uint8): ピン番号 (0-8)
          * `ref_int` (int32): 指令値 (角度の整数値)
            * ただし、使うESCによっては値の範囲が変わることがあるので注意。
            * サーボは謎の微調整をすることを考慮し、範囲外の値も入力できるようにしてある。
      * **例 (board\_id=2, ピン8, 角度 90[deg]):**
        ```bash
        ros2 topic pub /sabacan_gpio_ref_int2 sabacan_msgs/msg/SabacanGPIORefInt '{pin_number: 8, ref_int: 90}'
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

## トラブルシューティング
よくある症状ごとに、原因の切り分けと対処をまとめました。

### 1. ノードは起動するが状態が流れてこない
- `board_id` 未設定または重複: 起動時に必須。`ros2 param get /sabacan_gpio_node board_id` で確認。
- CANブリッジ未起動: `/to_can_bus` と `/from_can_bus` が存在するか確認。`ros2 topic list | rg can_bus`。
- 物理層: 電源/CAN-H/CAN-L/GND共通/終端120Ω×2（合成約60Ω）。`ip -details -statistics link show can0` でIF `UP`とビットレートを確認。

### 2. 入力が常にfalse/変化しない
- `pin_type` が `INPUT` になっているか。対象ピンに対してのみ `/sabacan_gpio_status<board_id>` がpublishされます。
- プルアップ/プルダウンの外付けや信号レベルを再確認。

### 3. PWM/ESCが反応しない・不安定
- `pin_type` が `OUTPUT_PWM` or `OUTPUT_ESC` か。
- `pwm_freq` が対象ピンで正の値か（Hz）。0や未設定だとDuty計算不可。
- PWMのDutyは `/sabacan_gpio_ref_float<board_id>`、ESCは `/sabacan_gpio_ref_int<board_id>` で送る。
- ESC値は50〜100の範囲。値外は無視されます。

### 4. 何をやってもうまく行かないときは最小構成で試す
プログラムのどこかにバグが仕込まれている可能性があるので、最小構成のプログラムで試してみましょう。  
cansendやcandumpで直接CAN通信をして、基板もしくはROS 2ノードの動作を確認するのも有用です。  

## monitor_regの設定例
GPIO基板は単一の64bitマスク（`monitor_reg`）で周期送信するレジスタ群を指定します。
- 対象レジスタ（can_define.h/namespace GPIO）とID:
  - `PORT_MODE=0x01`, `PORT_READ=0x02`, `PORT_WRITE=0x03`, `PORT_INT_EN=0x04`, `ESC_MODE_EN=0x05`
  - `PWM_PERIOD=0x10`（ch0〜8は `PWM_PERIOD+n`）, `PWM_DUTY=0x20`（ch0〜8は `PWM_DUTY+n`）
- 設定方法: `monitor_reg |= (1ULL << RegisterID)`
  - 例) `PORT_READ`のみ: `1<<0x02 = 0x4`
  - 例) `PWM_DUTY`（全ch）: `1<<0x20 = 0x0000000100000000`
- 有効化: `enable_monitor_period=true` のときに `monitor_period`/`monitor_reg` が反映（デフォルトtrue）。

monitor_periodはms指定。周波数f[Hz]で送りたい場合は `monitor_period ≒ round(1000/f)`。
- 20Hz→50ms、50Hz→20ms、100Hz→10ms（上げすぎるとCAN負荷増）。
- 特に理由がなければ、20Hz~100Hzの間に設定すればいいと思います。
- 範囲: 0〜65535ms。0は多くの実装で「無効」。

### 1. PORT_READ（入力だけを監視）
基本的に、INPUTのときはこの設定にすればすべてのピンの状態が読めるようになります。  
ロボマス制御基板とは違って1度のパケットにすべてのピンの情報が収まっているので、使っていないピンのデータを送るか、送らないかを考慮する必要もありません。  
gpio_node側でINPUTのときはPORT_INT_ENを有効になるようにしているので、monitor_periodが遅くてデータを読み飛ばしてしまうことはありません。  
値が変化したときはmonitor_periodの周期でなくても値は送信されます。
- reg = `0x4`（`PORT_READ`=0x4）
```bash
ros2 param set /sabacan_gpio_node monitor_reg 0x4
ros2 param set /sabacan_gpio_node monitor_period 50   # 20Hz相当
```

### 2. PORT_READ + PWM_DUTY（入力と出力Dutyの両方を監視）
- reg = `0x0000000100000004`（`PORT_READ`=0x4 OR `PWM_DUTY`=0x0000000100000000）
```bash
ros2 param set /sabacan_gpio_node monitor_reg 0x0000000100000004
ros2 param set /sabacan_gpio_node monitor_period 50
```

### 3. PWM_DUTYのみ（出力Dutyだけ確認したい）
- reg = `0x0000000100000000`（`PWM_DUTY`=0x0000000100000000）
```bash
ros2 param set /sabacan_gpio_node monitor_reg 0x0000000100000000
ros2 param set /sabacan_gpio_node monitor_period 50
```

### 4. 何も送らない（モニタ停止）
- reg = `0x0`（または `enable_monitor_period=false` や `monitor_period=0`）
```bash
ros2 param set /sabacan_gpio_node monitor_reg 0x0
ros2 param set /sabacan_gpio_node monitor_period 0
```

### 5. ｓ
- `PORT_READ`のIDは0x02 → `1<<0x02 = 0x4`
- `PWM_DUTY`のIDは0x20 → `1<<0x20 = 0x0000000100000000`
- 結合はOR: 例（2の構成）`0x4 | 0x0000000100000000 = 0x0000000100000004`

## その他
どうして、SabacanGPIORefやSabacanGpioStatusの中にpin_numberが含まれているの？  
1つの同一トピックにまとめるのではなく、9つ分のトピック用意したほうがスマートじゃない？

A. そのとおりです。しかし、すでにsabacan_msgsに依存するプログラムが多く書かれているので、いまさら変更することができないという、歴史的経緯があります。  
SabacanGPIORefとSabacanGPIOStatusはそのままでは扱いにくいので、1モータ1トピックに変換するノードを書いて、使っている人が多いです。  

どうして、入力ピンの論理を反転させる機能はないの？  
A. sabacan_robomasv2_nodeには回転方向を反転させる機能がないから。sabacan_robomasv2_nodeに反転機能を追加するのが面倒だったし、もっと上位のレイヤーでそういうことはやったほうが扱いやすそうだと思ったから。  
