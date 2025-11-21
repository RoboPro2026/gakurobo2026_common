# これは何

サバ缶パッケージから publish されるデータ 4 つをまとめて publish するパッケージ

これらのノードから pub されるデータを使用した制御は推奨しない

# `sabacan_robomasv2_debug_node.cpp`

`sabacan_robomasv2_node`のデータをデバッグしやすい方式に変換して pub するノード

- publish: `/sabacan_robomas_ref_debug<board_id>`, `/sabacan_robomas_status_debug<board_id>`

- subscribe: `/sabacan_robomas_ref<board_id>`, `/sabacan_robomas_status<board_id>`

- パラメータ

  - `board_id`: サバ缶基板の id
  - `publish_rate_hz`: デバッグデータを pub する周波数[Hz]

- メッセージ型
  - `sabacan_debug_msgs::msg::SabacanRobomasRefDebug`
  ```
  sabacan_msgs/SabacanRobomasRef[4] ref_array
  ```
  - `sabacan_debug_msgs::msg::SabacanRobomasStatusDebug`
  ```
  sabacan_msgs/SabacanRobomasStatus[4] status_array
  ```

# `sabacan_gpio_debug_node.cpp`

`sabacan_gpio_node`のデータをデバッグしやすい方式に変換して pub するノード

- publish: `/sabacan_gpio_ref_float_debug<board_id>`, `/sabacan_gpio_ref_int_debug<board_id>`, `/sabacan_gpio_status_debug<board_id>`

- subscribe: `/sabacan_gpio_ref_float<board_id>`, `/sabacan_gpio_ref_int<board_id>`, `/sabacan_gpio_status<board_id>`

- パラメータ

  - `board_id`: サバ缶基板の id
  - `publish_rate_hz`: デバッグデータを pub する周波数[Hz]

- メッセージ型
  - `sabacan_debug_msgs::msg::SabacanGPIORefDebug`
  ```
  sabacan_msgs/SabacanGPIORefFloat[8] ref_array
  ```
  - `sabacan_debug_msgs::msg::SabacanGPIORefIntDebug`
  ```
  sabacan_msgs/SabacanGPIORefInt[8] ref_array
  ```
  - `sabacan_debug_msgs::msg::SabacanGPIOResDebug`
  ```
  sabacan_msgs/SabacanGPIOStatus[8] res_array
  ```
