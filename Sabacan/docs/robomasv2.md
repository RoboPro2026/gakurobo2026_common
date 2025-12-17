# `sabacan_robomasv2_node` の使い方

`sabacan_robomasv2_node` は、ロボマス制御基板 V2（新基板）をROS 2で制御するためのノードです。1つの基板で4つのモーターを回すことができます。
C610、C620、VESCのトルク制御（オープンループ）、速度制御、位置制御が可能です。

## 1. ノードの起動

ノードを起動する際は、ROSパラメータとして `board_id` を**必ず指定**する必要があります。

> **補足**  
> 起動直後に大量のパラメータをCAN経由で書き込むと基板側が取りこぼすことがあったため、`sabacan_robomasv2_node` は各パラメータ送信のあとに約10msのディレイを入れています。起動に1秒程度かかりますが、通信を安定させるための仕様です。また、delayの間は処理が止まるので注意が必要です。

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
  * `vesc_pole` (int64配列): VESCのモータの極数。これを正しく設定しないと、速度指令値がerpmなのでおかしくなる。
      * **初期値**: `[14, 14, 14, 14]`。
  * `monitor_period` (int): 状態フィードバックの送信周期 (ms)。
      * **初期値**: `50`。
  * `enable_monitor_period` (bool): 周期送信の有効/無効。
      * **初期値**: `true`。
  * `monitor_reg1` (int64配列): 周期送信するレジスタのビットマスク (下位64bit)。
      * **初期値 (各要素)**: `(1<<1) | (1<<16) | (1<<32) | (1<<48) | (1<<58) | (1<<59) | (1<<60)` (MOTOR\_STATE, TRQ, SPD, POS, ABS\_POS, ABS\_SPD, ABS\_TURN\_CNT)。
  * `monitor_reg2` (int64配列): 周期送信するレジスタのビットマスク (上位64bit)。
      * **初期値 (各要素)**: `(1<<2) | (1<<3) | (1<<4)` (VESC\_VOLTAGE, VESC\_CURRENT, VESC\_ERPM)。

monitor_reg周りでトラブルが発生したら、トラブルシューティングの項を参考にすること。

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
    ```
    uint8 motor_number
    string motor_type
    string control_type
    bool motor_state
    float32 torque # Nm
    float32 speed # rad/s
    float32 pos # rad
    float32 abs_pos # rad
    float32 abs_speed # rad/s
    int32 abs_turn_cnt # turn count
    float32 vesc_voltage # V
    float32 vesc_current # A
    float32 vesc_speed # rad/s
    ```
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

## トラブルシューティング

よくある症状ごとに、原因の切り分けと対処をまとめました。手元の配線/設定からROSノード/バス負荷まで、上から順に確認してください。

### 1. ノードは起動するが何もフィードバックが来ない
- `board_id` 未設定または重複: 起動時に必須です。`ros2 param get /sabacan_robomasv2_node board_id` で確認。
- CANブリッジ未起動: 本ノードは `/to_can_bus` へ送信し、`/from_can_bus` を購読します。`ros2 topic list | rg can_bus` でトピックが存在するか、CANブリッジ（例: socketcanブリッジ）が動いているかを確認。
- 物理層: 基板の電源、CAN-H/CAN-L、GNDの共通、終端抵抗（両端120Ω）が正しいかを確認。両端間の抵抗は約60Ωになるのが目安。
- IF設定: `ip -details -statistics link show can0` で `can0` が `UP` か、ビットレートが期待どおり（例: 1Mbps）かを確認。

### 2. 一部のフィードバック項目だけ欠落する/途切れる
- バス負荷過多の可能性が高いです。まず負荷を計測:
  ```bash
  canbusload can0@1000000
  # 例) can0@1000000  1015  119400  30560  11%
  #     2列目=パケット/秒, 5列目=バス負荷(%)
  ```
- 目安: 初期設定（monitor_period=50ms, 既定monitor_reg）で基板1枚あたり≒10%（指令100Hz時）。
- 経験則: 5000 pkt/s を超えるとロスが出やすい。100%超は確実にロス。
- 対処（効果が高い順）:
  - `monitor_period` を増やす（例: 50ms→100ms）。デバッグ用途は20〜100Hzで十分。
  - `monitor_reg1/2` から不要項目を外す（ABS系やVESC系など未使用データを外す）。
  - 指令トピック `/sabacan_robomas_ref*` の送信周期を落とす（目安≤100Hz）。`ros2 topic hz` で送信周期を確認。
  - 複数基板に一斉に大量の書き込みをしない。連続で32パケットを超える送信は10ms程度の待ちを入れる。

### 3. 指令は出しているがモータが動かない
- `motor_type` と `control_type` の組合せを再確認。DJI系は `TORQUE/VELOCITY(PID)/POSITION`、VESCは `DISABLE/PWM/CURRENT/SPEED(PID)/POSITION`。
- ゲイン/制限値: `speed_gain_*`, `torque_lim`, `pos_gain_*`, `speed_lim` が極端に小さくないか。
- `motor_number` の指定ミスに注意。`sabacan_robomas_status<board_id>` で該当番号の状態を確認。
- VESC使用時は `control_type` でVESCモードが正しく切り替わることを確認。
  - VESCの速度制御モードの場合、指令値が小さいと回らないことがある。必要に応じてVESC TOOLで設定を確認すること。
### 4. パラメータを書いた直後に取りこぼしが起きる
- 起動直後の大量設定は基板側で取りこぼす場合があります。本ノードは送信毎に約10msのディレイを入れていますが、外部から大量に `ros2 param set` やサービス呼び出しを連打しないでください。必要なら自分の側でも10ms程度の待ちを入れる。

### 5. CANレイヤの確認コマンド
- インタフェース: `ip -details -statistics link show can0`（ビットレート/エラーカウンタ）。
- バス負荷: `canbusload can0@1000000`。
- 生の受信確認: `candump -tz can0`（エラーが増える、全く流れない、IDが想定外などを確認）。

### 6. ログが多すぎて処理が重い
- 起動オプションでログを下げる: `--ros-args --log-level warn`。ログ出力はCPU/IO負荷になるため、デバッグ以外では抑制を推奨。

### 7. ハードウェアの要点（最終チェック）
- これが一番よくあるミスです。
- 終端抵抗: バス両端に120Ω（合成で約60Ω）。基板のジャンパで切替可能。
- 配線: ツイストペア推奨、GND共通、配線長は必要最小限に。
- 速度: 標準は1Mbps。全機器を同一設定に。

### 8. 何をやってもうまく行かないときは最小構成で試す
プログラムのどこかにバグが仕込まれている可能性があるので、最小構成のプログラムで試してみましょう。  
cansendやcandumpで直接CAN通信をして、基板もしくはROS 2ノードの動作を確認するのも有用です。  

## monitor_regの設定例
monitor_regは各モータごとの64bitビットマスクです。配列はモータ0〜3の順。
- reg1対象(0x00〜0x3F): MOTOR_STATE(0x01), TRQ(0x10), SPD(0x20), POS(0x30), ABS_POS(0x3A), ABS_SPD(0x3B), ABS_TURN_CNT(0x3C)
- reg2対象(0x40〜0x7F): VESC_VOLTAGE(0x42), VESC_CURRENT(0x43), VESC_ERPM(0x44)
- 計算式: reg1は OR(1 << RegisterID), reg2は OR(1 << (RegisterID - 0x40))
- 例中はモータ0のみ設定。他は0。必要に応じて各要素へ同じ値を入れてください。
- 有効化: `enable_monitor_period=true` のときに `monitor_period`/`monitor_reg*` が適用されます（デフォルトtrue）。

monitor_periodの使い分けは、PC側で処理する場合は最大100Hz（10ms）。デバッグ用途は20〜100Hz程度に抑え、CAN負荷を見ながら調整してください。

### 1. SPDをモニターする場合（速度制御の場合）
- motor_stateあり: reg1 = 0x0000000100000002, reg2 = 0x0（SPD+MOTOR_STATE）
- motor_stateなし: reg1 = 0x0000000100000000, reg2 = 0x0（SPDのみ）
```bash
# motor_stateあり
ros2 param set /sabacan_robomasv2_node monitor_reg1 "[0x0000000100000002, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_reg2 "[0x0, 0, 0, 0]"
# motor_stateなし
ros2 param set /sabacan_robomasv2_node monitor_reg1 "[0x0000000100000000, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_reg2 "[0x0, 0, 0, 0]"
```

### 2. SPDとTRQをモニターする場合（速度制御調整用）
- motor_stateあり: reg1 = 0x0000000100010002（SPD+TRQ+MOTOR_STATE）
- motor_stateなし: reg1 = 0x0000000100010000（SPD+TRQ）
```bash
# motor_stateあり
ros2 param set /sabacan_robomasv2_node monitor_reg1 "[0x0000000100010002, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_reg2 "[0x0, 0, 0, 0]"
# motor_stateなし
ros2 param set /sabacan_robomasv2_node monitor_reg1 "[0x0000000100010000, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_reg2 "[0x0, 0, 0, 0]"
```

### 3. POSをモニターする場合（位置制御の場合）
- motor_stateあり: reg1 = 0x0001000000000002（POS+MOTOR_STATE）
- motor_stateなし: reg1 = 0x0001000000000000（POSのみ）
```bash
# motor_stateあり
ros2 param set /sabacan_robomasv2_node monitor_reg1 "[0x0001000000000002, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_reg2 "[0x0, 0, 0, 0]"
# motor_stateなし
ros2 param set /sabacan_robomasv2_node monitor_reg1 "[0x0001000000000000, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_reg2 "[0x0, 0, 0, 0]"
```

### 4. SPDとPOSをモニターする場合（位置制御調整用）
- motor_stateあり: reg1 = 0x0001000100000002（SPD+POS+MOTOR_STATE）
- motor_stateなし: reg1 = 0x0001000100000000（SPD+POS）
```bash
# motor_stateあり
ros2 param set /sabacan_robomasv2_node monitor_reg1 "[0x0001000100000002, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_reg2 "[0x0, 0, 0, 0]"
# motor_stateなし
ros2 param set /sabacan_robomasv2_node monitor_reg1 "[0x0001000100000000, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_reg2 "[0x0, 0, 0, 0]"
```

### 5. TRQとSPDとPOSをモニターする場合（位置制御調整用orデバッグ用）
- motor_stateあり: reg1 = 0x0001000100010002（TRQ+SPD+POS+MOTOR_STATE）
- motor_stateなし: reg1 = 0x0001000100010000（TRQ+SPD+POS）
```bash
# motor_stateあり
ros2 param set /sabacan_robomasv2_node monitor_reg1 "[0x0001000100010002, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_reg2 "[0x0, 0, 0, 0]"
# motor_stateなし
ros2 param set /sabacan_robomasv2_node monitor_reg1 "[0x0001000100010000, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_reg2 "[0x0, 0, 0, 0]"
```

### 6. ABS_SPDとABS_POSのみを知りたい場合（設置エンコーダなど）
ABS_POS + ABS_SPD（必要ならMOTOR_STATEも加える）
- reg1 = 0x0C00000000000000（+MOTOR_STATEで 0x0C00000000000002）, reg2 = 0x0
```bash
# motor_state無し
ros2 param set /sabacan_robomasv2_node monitor_reg1 "[0x0C00000000000000, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_reg2 "[0x0, 0, 0, 0]"
# motor_stateも入れる場合（推奨）
ros2 param set /sabacan_robomasv2_node monitor_reg1 "[0x0C00000000000002, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_reg2 "[0x0, 0, 0, 0]"
```

### 7. VESCのフィードバックデータを100Hzで欲しい場合
VESC_VOLTAGE + VESC_CURRENT + VESC_ERPM（reg2側）。実機は100Hz未満の可能性あり。
- reg1 = 0x0, reg2 = 0x1C, monitor_period = 10(ms)
```bash
ros2 param set /sabacan_robomasv2_node monitor_reg1 "[0x0, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_reg2 "[0x1C, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_period 10
```

### 8. モータがつながっているかを知りたいとき
安全に考慮するなら、常にmotor_stateは送り続けて、falseの場合はロボット全体を止めるみたいな感じにしたほうがいいと思います。
MOTOR_STATEのみ
- reg1 = 0x2, reg2 = 0x0
```bash
ros2 param set /sabacan_robomasv2_node monitor_reg1 "[0x2, 0, 0, 0]"
ros2 param set /sabacan_robomasv2_node monitor_reg2 "[0x0, 0, 0, 0]"
```

### monitor_regの計算方法について
実際に書き込んだ結果はリトルエンディアンとなるので注意
補足: 16進指定（0x...）はROS 2のYAMLパーサでintとして解釈されます。10進で指定しても構いません。

#### 対象レジスタIDとビット位置
- reg1側（0x00〜0x3Fをそのままビット番号として使う）
  - MOTOR_STATE: 0x01 → ビット1（1<<1 = 0x2）
  - TRQ: 0x10 → ビット16（1<<16 = 0x0000000000010000）
  - SPD: 0x20 → ビット32（1<<32 = 0x0000000100000000）
  - POS: 0x30 → ビット48（1<<48 = 0x0001000000000000）
  - ABS_POS: 0x3A → ビット58（1<<58 = 0x0400000000000000）
  - ABS_SPD: 0x3B → ビット59（1<<59 = 0x0800000000000000）
  - ABS_TURN_CNT: 0x3C → ビット60（1<<60 = 0x1000000000000000）
- reg2側（0x40〜0x7Fは 0x40 を引いてからビット番号にする）
  - VESC_VOLTAGE: 0x42 → 0x42-0x40=2 → ビット2（1<<2 = 0x4）
  - VESC_CURRENT: 0x43 → 3 → ビット3（1<<3 = 0x8）
  - VESC_ERPM: 0x44 → 4 → ビット4（1<<4 = 0x10）

#### 手計算の例（SPDとMOTOR_STATE）
1) SPDのIDは0x20=32 → reg1のビット32を立てる → 1<<32 = 4294967296 = 0x0000000100000000
2) MOTOR_STATEは0x01=1 → 1<<1 = 2 = 0x2
3) ORで結合 → 0x0000000100000000 | 0x2 = 0x0000000100000002

#### 手計算の例（ABS_POSとABS_SPDのみにする）
1) ABS_POSは0x3A=58 → 1<<58 = 0x0400000000000000
2) ABS_SPDは0x3B=59 → 1<<59 = 0x0800000000000000
3) ORで結合 → 0x0C00000000000000（必要ならMOTOR_STATE=0x2もOR）

#### 手計算の例（VESC系：電圧・電流・ERPM）
1) それぞれ 0x42,0x43,0x44 なので 0x40 を引いて 2,3,4
2) reg2で 1<<2=0x4, 1<<3=0x8, 1<<4=0x10
3) ORで結合 → 0x4|0x8|0x10 = 0x1C（reg2側に設定）

### monitor_periodの計算方法（周波数↔周期）
- 単位はミリ秒。基板には`monitor_period[ms]`として書き込みます。
- 周波数`f[Hz]`で送りたい場合の目安: `monitor_period = round(1000 / f)`
  - 20Hz → 50ms
  - 50Hz → 20ms
  - 100Hz → 10ms（上げすぎるとCAN負荷増）
- 設定範囲: 0〜65535（ms）。0はファーム側で「無効」として扱われる実装が多いです。
  - 完全に止めたい場合: `enable_monitor_period=true`の状態で`monitor_period=0`を設定、または`monitor_reg1/2`を0にする。
- パケット目安: 1周期で選択レジスタの数だけCANフレームが出ます。
  - 例）1モータでSPD+MOTOR_STATEの2項目、100Hzなら 2 × 100 = 200 pkt/s。
  - 4モータすべて同設定なら 200 × 4 = 800 pkt/s。`canbusload`の値と見比べて調整してください。

## その他
どうして、SabacanRobomasRefやSabacanRobomasStatusの中にmotor_numberが含まれているの？  
1つの同一トピックにまとめるのではなく、4つ分のトピック用意したほうがスマートじゃない？

A. そのとおりです。しかし、すでにsabacan_msgsに依存するプログラムが多く書かれているので、いまさら変更することができないという、歴史的経緯があります。  
SabacanRobomasRefとSabacanRobomasStatusはそのままでは扱いにくいので、1モータ1トピックに変換するノードを書いて、使っている人が多いです。  

どうして、入力ピンの論理を反転させる機能はないの？  
A. sabacan_robomasv2_nodeに反転機能を追加するのが面倒だったし、もっと上位のレイヤーでそういうことはやったほうが扱いやすそうだと思ったから。  
