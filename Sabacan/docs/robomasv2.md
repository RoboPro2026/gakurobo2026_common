# `sabacan_robomasv2_node` の使い方

`sabacan_robomasv2_node` は、ロボマス制御基板 V2（新基板）をROS 2で制御するためのノードです。トルク制御、外乱オブザーバ、動的なゲイン設定など、V1に比べて高度な機能を提供します。

## 1. ノードの起動

ノードを起動する際は、ROSパラメータとして `board_id` を**必ず指定**する必要があります。

```bash
# ros2_ws でビルドとソース
cd /path/to/ros2_ws
colcon build --packages-select sabacan_msgs sabacan
source install/setup.bash

# ノード起動 (board_id=0 の例)
ros2 run sabacan sabacan_robomasv2_node --ros-args -p board_id:=0
````

board_idは0~9の範囲で設定してください。  

## 2\. パラメータ設定

ノード起動時、または `ros2 param set` コマンドでパラメータを設定できます。

**例: モーター0をトルク制御、モーター1を速度制御に設定**

```bash
ros2 param set /sabacan_robomasv2_node control_type "['TORQUE', 'VELOCITY', 'VELOCITY', 'VELOCITY']"
```

**主要なパラメータ:**

  * `board_id` (int64, **必須**): 基板のCAN ID (0〜9)。
  * `motor_type` (string配列): `"C610"`, `"C620"`, `"VESC"`。
  * `control_type` (string配列):
      * DJIモーターの場合: `"TORQUE"`, `"VELOCITY"`, `"POSITION"`。
      * VESCの場合: `"DISABLE"`, `"PWM"`, `"CURRENT"`, `"SPEED"`/`"VELOCITY"`, `"POSITION"`/`"POS"`。
  * `dob_en` (bool配列): 外乱オブザーバ (DOB) の有効/無効。
  * `abs_enc_en` (bool配列): アブソリュートエンコーダの有効/無効。
  * ゲイン・制限値: `speed_gain_p`, `speed_gain_i`, `speed_gain_d`, `torque_lim`, `pos_gain_p`, `pos_gain_i`, `pos_gain_d`, `speed_lim` (double配列)。
  * モニタリング: `monitor_period`, `enable_monitor_period`, `monitor_reg1`, `monitor_reg2`。

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

`/set_robomas_gains` サービス (型: `sabacan_msgs/srv/SetRobomasGains`) を呼び出して、ゲインやDOBパラメータを変更します。サービスではなく、パラメータを用いても変更することができます。  

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