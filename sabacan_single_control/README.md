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
  - `change_mode_delay`: モード変更後の待機時間[s]。デフォルトは`0.2`。この期間中は指令値の送信を停止する。
  - `control_type_update_method`: `control_type` の切替方法。`"parameter"` または `"service"`。デフォルトは互換性維持のため `"parameter"`。推奨は `"service"`。

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
  float32 vesc_speed
  ```

# 制御モード

- `SABANE`: 基板側で制御。さばね謹製 PID & DOB
- `ROS`: ROS2 側で制御。全部１から制御したい人向け。まだ実装してない。

# control_type の切替方法

- `parameter`:
  - 互換性維持用の旧方式。
  - `/sabacan_robomasv2_node_id<board_id>/get_parameters` と `/set_parameters` を使って `control_type` 配列を書き換える。
- `service`:
  - 推奨方式。
  - `SetRobomasGains` service を使って、対象モータ 1 軸だけの `control_type` を更新する。
  - 複数の `single_control_node` が同じ基板を同時に扱う場合でも、配列の read-modify-write 競合を避けやすい。

## service 方式の remap

`control_type_update_method:="service"` を使う場合、`sabacan_single_control_node` の service client 名 `set_robomas_gains` を、実際の基板別 service 名へ remap する。

- `ros2 run` の例:
  ```bash
  ros2 run sabacan_single_control sabacan_single_control_node --ros-args \
    -p board_id:=2 \
    -p motor_number:=0 \
    -p control_type_update_method:=service \
    -r set_robomas_gains:=/set_robomas_gains_id2
  ```

- launch の例:
  ```python
  remappings=[
      ("set_robomas_gains", f"set_robomas_gains_id{board_id}"),
  ]
  ```

同時に、`sabacan_robomasv2_node` 側でも service server 名 `set_robomas_gains` を同じ基板別 service 名へ remap しておく必要がある。

- `sabacan_robomasv2_node` の launch 例:
  ```python
  remappings=[
      ("set_robomas_gains", f"set_robomas_gains_id{board_id}"),
  ]
  ```

- `ros2 run` の例:
  ```bash
  ros2 run sabacan sabacan_robomasv2_node --ros-args \
    -p board_id:=2 \
    -r set_robomas_gains:=/set_robomas_gains_id2
  ```

つまり service 方式では、client 側の `sabacan_single_control_node` と server 側の `sabacan_robomasv2_node` の両方で、同じ `set_robomas_gains_id<board_id>` へ揃えて remap する。

`r1_bringup.launch.py` と `test_bringup.launch.py` では、`sabacan_single_control_node` 側の remap を設定済みで、`r1_bringup.launch.py` では `sabacan_robomasv2_node` 側の remap も設定済みです。

# 動作テスト

## モード切替時の安全機能テスト

モーターが暴走しないか、以下の手順で確認できます。

1. ノードを起動（例: board_id=0, motor_number=0）
   ```bash
   ros2 run sabacan_single_control sabacan_single_control_node --ros-args -p board_id:=0 -p motor_number:=0
   ```

2. 速度制御で回転させる
   ```bash
   ros2 topic pub /sabacan_robomas_ref0/motor0 sabacan_single_control_msgs/msg/SabacanRobomasSingleRef "{control_type: 'VELOCITY', ref: 10.0}" -1
   ```

3. 位置制御に切り替える（暴走リスクが高い操作）
   - 変更直後に安全な待機時間(`change_mode_delay`)が挿入され、その間指令値が送信されないため、急激な動き（位置0への収束）が発生しないことを確認します。
   ```bash
   ros2 topic pub /sabacan_robomas_ref0/motor0 sabacan_single_control_msgs/msg/SabacanRobomasSingleRef "{control_type: 'POSITION', ref: 0.0}" -1
   ```
