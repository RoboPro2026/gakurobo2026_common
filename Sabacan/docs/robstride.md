# `sabacan_robstride_node` の使い方

`sabacan_robstride_node` は、Robstride モーター（RS05 / EL05）を CAN 経由で ROS 2 から制御するノードです。
指令（MIT / 電流 / 速度 / 位置(PP/CSP)）をトピックで受け取り、状態（トルク / 速度 / 角度）を publish します。

## 1. ノードの起動

ノードを起動する際は、ROSパラメータとして `board_id` を**必ず指定**する必要があります。

```bash
# ros2_ws でビルドとソース
cd /path/to/ros2_ws
colcon build --packages-select sabacan_msgs sabacan
source install/setup.bash

# ノード起動 (board_id=127 の例)
ros2 run sabacan sabacan_robstride_node --ros-args -p board_id:=127
```

> **補足**
> - 本ノードは `/to_can_bus` へ CAN フレームを publish し、`/from_can_bus` を subscribe します。CAN ブリッジ（例: socketcan ブリッジ）が別途必要です。
> - ログが非常に多いので、運用時は `--ros-args --log-level warn` を推奨します。

## 2. 主要パラメータ

起動時または `ros2 param set` で変更できます。

- `board_id` (int64, **必須**): Robstride の CAN ID（`0`〜`255`）。
- `can_master_id` (int64, デフォルト: `253`): マスター側の CAN ID。
- `robstride_type` (string, デフォルト: `"RS05"`): `"RS05"` / `"EL05"`。
- `enable_initialize` (bool, デフォルト: `true`): 起動時に初期化コマンド（パラメータ書き込み等）を送るか。
- `publish_timer_rate` (double, デフォルト: `100.0`): `/sabacan_robstride_status<board_id>` を周期送信するレート [Hz]。

制御パラメータ（ノードが Robstride に書き込みます）:
- `velocity_mode_limit_cur` (float, デフォルト: `11.0`): 速度モードの電流制限（`LIMIT_CUR`）。
- `velocity_mode_acc_rad` (float, デフォルト: `20.0`): 速度モードの加速度（`ACC_RAD`）。
- `csp_mode_limit_spd` (float, デフォルト: `50.0`): CSP モードの速度制限（`LIMIT_SPD`）。
- `pp_mode_vel_max` (float, デフォルト: `10.0`): PP モードの最大速度（`VEL_MAX`）。
- `pp_mode_acc_set` (float, デフォルト: `10.0`): PP モードの加速度（`ACC_SET`）。
- `espcan_time_ms` (int, デフォルト: `10`): フィードバック周期の設定値（内部で `EPSCAN_TIME` に変換して書き込みます）。

## 3. 指令の送信 (トピック)

- トピック: `/sabacan_robstride_ref<board_id>`
- 型: `sabacan_msgs/msg/SabacanRobstrideRef`

メッセージ定義:
```text
string control_type
# MITモードの指令値
float32 mit_torque # (RS05の場合は-5.5~5.5, EL05の場合は-6.0~6.0 N)
float32 mit_pos # (RS05とEL05の場合は-4pi~4pi rad)
float32 mit_speed # (RS05とEL05の場合は-50.0~50.0rad/s)
float32 mit_kp # (RS05とEL05の場合は0.0~500.0)
float32 mit_kd # (RS05とEL 0.0~500.0)
# 電流モードの指令値（範囲の制限なし）
float32 current_ref
# 速度モードの指令値（範囲の制限なし）
float32 velocity_ref
# 位置制御モード(PP)の指令値（範囲の制限なし）
float32 pp_angle_ref
# 位置制御モード(CSP)の指令値（範囲の制限なし）
float32 csp_angle_ref
```

`control_type` は以下のいずれかを指定します:
- `"MIT"`: `mit_torque`, `mit_pos`, `mit_speed`, `mit_kp`, `mit_kd` を使用
- `"CURRENT"`: `current_ref` を使用
- `"VELOCITY"`: `velocity_ref` を使用
- `"PP"`: `pp_angle_ref` を使用
- `"CSP"`: `csp_angle_ref` を使用

各種モードの説明はデータシートを読んでください。  

### 3.1 MIT 指令（board_id=127）
$$
\tau = K_p (\theta_{ref} - \theta) + K_d (v_{ref} - v) + \tau_{ff}
$$
MITモードとは、位置、速度、トルク（FF）の指令値を同時に入力できる制御。  
MITモードの場合、位置の指令値に入れられる範囲はRobstride 05の場合、-12.54radから12.54radに制限されるので、MITモードは無限回転には対応していません。
```bash
ros2 topic pub --once /sabacan_robstride_ref127 sabacan_msgs/msg/SabacanRobstrideRef "{
  control_type: 'MIT',
  mit_torque: 0.2,
  mit_pos: 0.0,
  mit_speed: 0.0,
  mit_kp: 10.0,
  mit_kd: 0.5
}"
```

### 3.2 電流指令（board_id=127）
```bash
ros2 topic pub --once /sabacan_robstride_ref127 sabacan_msgs/msg/SabacanRobstrideRef "{
  control_type: 'CURRENT',
  current_ref: 1.0
}"
```

### 3.3 速度指令（board_id=127）  
このモードでは台形加速をして、指令値に追従する。  
最大電流は`velocity_mode_limit_cur`で制限される。  
加速度は`velocity_mode_acc_rad`となる。
```bash
ros2 topic pub --once /sabacan_robstride_ref127 sabacan_msgs/msg/SabacanRobstrideRef "{
  control_type: 'VELOCITY',
  velocity_ref: 5.0
}"
```

### 3.4 PP 位置指令（board_id=127）
位置指令値は無限回転に対応。
なめらかに加速をし、指定した位置に移動する。  
移動時の最大速度は`pp_mode_vel_max`となり、速度は加速度`pp_mode_acc_set`で変化する。
```bash
ros2 topic pub --once /sabacan_robstride_ref127 sabacan_msgs/msg/SabacanRobstrideRef "{
  control_type: 'PP',
  pp_angle_ref: 1.57
}"
```

### 3.5 CSP 位置指令（board_id=127）
位置指令値は無限回転に対応。
最大速度で位置に追従する。
最大速度は`csp_mode_limit_spd`となる。PPモードとは異なり、速度は急激に変化する。
```bash
ros2 topic pub --once /sabacan_robstride_ref127 sabacan_msgs/msg/SabacanRobstrideRef "{
  control_type: 'CSP',
  csp_angle_ref: 1.57
}"
```

## 4. 状態の取得 (トピック)

- トピック: `/sabacan_robstride_status<board_id>`
- 型: `sabacan_msgs/msg/SabacanRobstrideStatus`

メッセージ定義:
```text
string control_type # 制御方式
string motor_mode_status # 現在のmode、RESET, CALIBURATION,RUNの3つ
float32 torque # Nm
float32 speed  # rad/s
float32 pos    # rad
```

例（board_id=127）:
```bash
ros2 topic echo /sabacan_robstride_status127
```

> **pos について**
> - ドライバの角度情報を元に、多回転を考慮した「積算角」を publish します（簡易的に wrap を見て回転数をカウント）。

## 5. ゲイン/制限値の変更 (サービス)

- サービス: `/set_robstride_gains`
- 型: `sabacan_msgs/srv/SetRobstrideGains`

`SetRobstrideGains`:
```text
bool set_limit_cur
float32 limit_cur
---
bool success
string message
```

例: 速度モードの電流制限を変更
```bash
ros2 service call /set_robstride_gains sabacan_msgs/srv/SetRobstrideGains "{
  set_limit_cur: true,
  limit_cur: 12.0
}"
```

## 6. パラメータの再初期化 (サービス)

- サービス: `/sabacan_robstride_reset`
- 型: `sabacan_msgs/srv/SabacanReset`

```bash
ros2 service call /sabacan_robstride_reset sabacan_msgs/srv/SabacanReset "{}"
```

## 7. CANトピック（共通）

- publish: `/to_can_bus` (`can_msgs/msg/Frame`)
- subscribe: `/from_can_bus` (`can_msgs/msg/Frame`)

## トラブルシューティング

### 1. ノードは起動するが status が来ない
- CAN ブリッジ未起動/トピック名不一致: `ros2 topic list | rg can_bus` で `/to_can_bus` と `/from_can_bus` を確認。
- `board_id` の設定漏れ/重複: 必須です。起動コマンドの `-p board_id:=...` を見直してください。
- 物理層: 電源、CAN-H/CAN-L、GND共通、終端抵抗（両端 120Ω / 両端間で約 60Ω）を確認。

### 2. CPU が重い / 遅い
- 本ノードは送受信・publish ごとに `INFO` ログを出します。`--ros-args --log-level warn` を付けて起動してください。

# その他解説資料
https://nagaokaroboconproject.esa.io/posts/338
