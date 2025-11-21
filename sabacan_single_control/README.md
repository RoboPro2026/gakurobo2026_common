# これは何

`board_id`と`motor_number`を指定して単体のモータを制御するノード

制御方式と指令値を受け取って、サバ缶にパブリッシュする

# `sabacan_single_control_node.cpp`

- publish: `/sabacan_robomas_ref<board_id>`
- subscribe:

  - `/sabacan_robomas_ref<board_id>/motor<motor_number>` (制御指令)
  - `/sabacan_robomas_status<board_id>` (フィードバック)

- パラメータ

  - `board_id`: ボードの ID。デフォルトは`0`
  - `motor_number`: モータの番号。デフォルトは`0`
  - `control_mode`: 制御モード。`"ROS"`または`"SABANE"`。デフォルトは`"SABANE"`
  - `control_cycle`: 制御周期[Hz]。デフォルトは`100.0`
  - `control_type`: 制御方式。`"VELOCITY"`, `"POSITION"`, `"CURRENT"`, `"TORQUE"`。デフォルトは`"VELOCITY"`

- メッセージ型
  - `/sabacan_robomas_ref<board_id>`: `sabacan_msgs::msg::SabacanRobomasRef`
  ```
  uint8 motor_number
  float32 ref
  ```
  - `/sabacan_robomas_ref<board_id>/motor<motor_number>`: `sabacan_single_control_msgs::msg::SabacanRobomasSingleRef`
  ```
  string control_type
  float32 ref
  ```
  - `/sabacan_robomas_status<board_id>`: `sabacan_msgs::msg::SabacanRobomasStatus`
  ```
  uint8 motor_number
  string motor_type
  string control_type
  bool motor_state
  float32 torque
  float32 speed
  float32 pos
  float32 abs_pos
  float32 abs_speed
  int32 abs_turn_cnt
  float32 vesc_voltage
  float32 vesc_current
  float32 vesc_erpm
  ```

# 制御モード

- `SABANE`: 基板側で制御。さばね謹製 PID & DOB
- `ROS`: ROS2 側で制御。全部１から制御したい人向け。まだ実装してない。
