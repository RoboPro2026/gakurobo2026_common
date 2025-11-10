# `sabacan_robomasv2_node` の使い方

`sabacan_robomasv2_node` は、ロボマス制御基板 V2（新基板）をROS 2で制御するためのノードです。1つの基板で4つのモーターを回すことができます。
C610、C620、VESCのトルク制御（オープンループ）、速度制御、位置制御が可能です。

## 1. ノードの起動

ノードを起動する際は、ROSパラメータとして `board_id` を**必ず指定**する必要があります。

> **補足**  
> 起動直後に大量のパラメータをCAN経由で書き込むと基板側が取りこぼすことがあったため、`sabacan_robomasv2_node` は各パラメータ送信のあとに約10msのディレイを入れるようになりました。起動に数百ミリ秒余分にかかりますが、通信を安定させるための仕様です。また、delayのあとは処理が止まるので注意

```bash
# ros2_ws でビルドとソース
cd /path/to/ros2_ws
colcon build --packages-select sabacan_msgs sabacan
source install/setup.bash

# ノード起動 (board_id=0 の例)
ros2 run sabacan sabacan_robomasv2_node --ros-args -p board_id:=0
````

board\_idは0\~9の範囲で設定してください。

## 2\. パラメータ設定

ノード起動時、または `ros2 param set` コマンドでパラメータを設定できます。

**例: モーター0を速度制御（C610）、モーター1を位置制御（C620）、モーター2をトルク制御（VESC）に設定**

```bash
# 下記はパラメータ設定の例であり、デフォルト値とは異なります
ros2 run sabacan sabacan_robomasv2_node --ros-args -p board_id:=0 \
  -p motor_type:="['C610', 'C620', 'VESC', 'C610']" \
  -p control_type:="['VELOCITY', 'POSITION', 'TORQUE', 'VELOCITY']"
```

**主要なパラメータ:**
各種パラメータはプログラムを動作中でも、変更可能です。

  * `board_id` (int64, **必須**): 基板のCAN ID (0〜9)。
  * `motor_type` (string配列): 各モーター(0〜3)の種別。
      * 選択肢: `"C610"`, `"C620"`, `"VESC"`。
      * **初期値**: `["C610", "C610", "C610", "C610"]`。
  * `control_type` (string配列): 各モーターの制御モード。
      * DJIモーターの場合: `"TORQUE"`, `"VELOCITY"` (または `"SPEED"`), `"POSITION"` (または `"POS"`)。
      * VESCの場合: `"DISABLE"`, `"PWM"`, `"CURRENT"`, `"SPEED"` (または `"VELOCITY"`), `"POSITION"` (または `"POS"`)。
      * トルク制御の指令値単位は **[Nm]** です。
      * 速度制御はデフォルトではPID制御で、指令値単位は **[rad/s]** です。`dob_en`を有効にした場合、制御器は外乱オブザーバーになります。
      * 位置制御の指令値単位は **[rad]** です。マイナーループに速度制御器を持ちます。
      * VESCのPWM制御の場合、指令値は **-1〜1**。電流制御の場合、指令値は **[A]**です。
      * **初期値**: `["VELOCITY", "VELOCITY", "VELOCITY", "VELOCITY"]`。
  * `dob_en` (bool配列): 外乱オブザーバ (DOB) の有効/無効。
      * **初期値**: `[false, false, false, false]`。
  * `abs_enc_en` (bool配列): アブソリュートエンコーダの有効/無効。
      * **初期値**: `[false, false, false, false]`。
  * `md_guess_en` (bool配列): モータパラメータ推定機能の有効/無効。
      * **初期値**: `[false, false, false, false]`。
  * `abs_gear_ratio` (double配列): アブソリュートエンコーダのギア比。
      * **初期値**: `[1.0, 1.0, 1.0, 1.0]`。
      * モーターの出力軸からアブソに至るまでの減速比　回転方向が反転するなら負の値を入力
      * ロボマス内臓のエンコーダーを使うときは関係ないので注意
  * `cal_rq` (bool配列): キャリブレーション要求フラグ。
      * **初期値**: `[false, false, false, false]`。
  * `load_j` (double配列): DOBのイナーシャパラメータ [$kg \cdot m^2$]。
      * **初期値**: `[0.0005, 0.0005, 0.0005, 0.0005]`。
  * `load_d` (double配列): DOBの粘性摩擦パラメータ [$N \cdot m \cdot s/rad$]。
      * **初期値**: `[0.0004, 0.0004, 0.0004, 0.0004]`。
  * `dob_cf` (double配列): DOBのカットオフ周波数 [Hz]。
      * **初期値**: `[5.0, 5.0, 5.0, 5.0]`。
  * `speed_gain_p` (double配列): 速度制御の **Pゲイン**。
      * **初期値**: `[0.5, 0.5, 0.5, 0.5]`。
  * `speed_gain_i` (double配列): 速度制御の **Iゲイン**。
      * **初期値**: `[0.2, 0.2, 0.2, 0.2]`。
  * `speed_gain_d` (double配列): 速度制御の **Dゲイン**。
      * **初期値**: `[0.0, 0.0, 0.0, 0.0]`。
  * `torque_lim` (double配列): 速度制御時の **トルク制限値** [Nm]。
      * **初期値**: `[5.0, 5.0, 5.0, 5.0]`。
  * `pos_gain_p` (double配列): 位置制御の **Pゲイン**。
      * **初期値**: `[6.0, 6.0, 6.0, 6.0]`。
  * `pos_gain_i` (double配列): 位置制御の **Iゲイン**。
      * **初期値**: `[3.0, 3.0, 3.0, 3.0]`。
  * `pos_gain_d` (double配列): 位置制御の **Dゲイン**。
      * **初期値**: `[0.0, 0.0, 0.0, 0.0]`。
  * `speed_lim` (double配列): 位置制御時の **速度制限値** [rad/s]。
      * **初期値**: `[30.0, 30.0, 30.0, 30.0]`。
  * `abs_turn_cnt` (int64配列): アブソリュートエンコーダの回転数オフセット。
      * **初期値**: `[0, 0, 0, 0]`。
  * `monitor_period` (int): 状態フィードバックの送信周期 (ms)。
      * **初期値**: `50`。
  * `enable_monitor_period` (bool): 周期送信の有効/無効。
      * **初期値**: `true`。
  * `monitor_reg1` (int64配列): 周期送信するレジスタのビットマスク (下位64bit)。
      * **初期値 (各要素)**: `(1<<1) | (1<<16) | (1<<32) | (1<<48) | (1<<58) | (1<<59) | (1<<60)` (MOTOR\_STATE, TRQ, SPD, POS, ABS\_POS, ABS\_SPD, ABS\_TURN\_CNT)。
  * `monitor_reg2` (int64配列): 周期送信するレジスタのビットマスク (上位64bit)。
      * **初期値 (各要素)**: `(1<<2) | (1<<3) | (1<<4)` (VESC\_VOLTAGE, VESC\_CURRENT, VESC\_ERPM)。

## 3\. モーターへの指令 (トピック)

`/sabacan_robomas_ref<board_id>` トピックに `SabacanRobomasRef` メッセージを送信します。

  * **メッセージ:**
      * `motor_number` (uint8): モーター番号 (0-3)。
      * `ref` (float32): 指令値（制御モードに応じた値）。
  * **例: 速度指令 (board\_id=0, モーター1, 3.14 rad/s)**
      * `control_type` パラメータが `VELOCITY` に設定されている必要があります。
    <!-- end list -->
    ```bash
    ros2 topic pub /sabacan_robomas_ref0 sabacan_msgs/msg/SabacanRobomasRef '{motor_number: 1, ref: 3.14}'
    ```
  * **例: トルク指令 (board\_id=0, モーター0, 0.5 Nm)**
      * `control_type` パラメータが `TORQUE` に設定されている必要があります。
    <!-- end list -->
    ```bash
    ros2 topic pub /sabacan_robomas_ref0 sabacan_msgs/msg/SabacanRobomasRef '{motor_number: 0, ref: 0.5}'
    ```

## 4\. モーター状態の確認 (トピック)

`/sabacan_robomas_status<board_id>` トピックで `SabacanRobomasStatus` メッセージが定期的に配信されます。

  * **メッセージ:**
      * `motor_number`, `motor_type`, `control_type`, `motor_state`, `torque`, `speed`, `pos`, `abs_pos`, `abs_speed`, `abs_turn_cnt`, `vesc_voltage`, `vesc_current`, `vesc_erpm`。
  * **例: 状態確認 (board\_id=0)**
    ```bash
    ros2 topic echo /sabacan_robomas_status0
    ```

## 5\. ゲインの動的変更 (サービス)

`/set_robomas_gains` サービス (型: `sabacan_msgs/srv/SetRobomasGains`) を呼び出して、ゲインやDOBパラメータを変更します。ROSパラメータ (`ros2 param set`) を用いても変更することができます。

  * **リクエスト:**
      * `motor_number` (uint8)。
      * 変更したいパラメータの値 (例: `speed_gain_p`)。
      * 変更を有効にするフラグ (例: `set_speed_gains: true`)。
  * **例: ゲイン変更 (board\_id=0, モーター0)**
    ```bash
    ros2 service call /set_robomas_gains sabacan_msgs/srv/SetRobomasGains '{
      motor_number: 0,
      set_speed_gains: true,
      speed_gain_p: 0.6,
      speed_gain_i: 0.3,
      torque_lim: 6.0,
      set_dob_param: true,
      dob_load_j: 0.0006,
      dob_load_d: 0.0005,
      dob_cutoff_freq: 10.0
    }'
    ```

## 6\. パラメータの再初期化 (サービス)

`/sabacan_robomas_reset` サービス (型: `sabacan_msgs/srv/SabacanReset`) を呼び出すと、現在のROSパラメータが基板に再送信されます。

```bash
ros2 service call /sabacan_robomas_reset sabacan_msgs/srv/SabacanReset '{}'
```
