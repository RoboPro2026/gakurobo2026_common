# `sabacan_check_id_rq_node` の使い方

`sabacan_check_id_rq_node` は、CANバス上にいるSabacan基板のID/種別を問い合わせるシンプルなサンプルです。  
`CommonDataDriver` で共通レジスタ `CommonRegisterID::RQ` へリモートリクエスト（RTR）をブロードキャストし、`/from_can_bus` に流れてきた `RQ` 応答だけをデコードしてログ出力します。

## 1. ビルドと起動
ビルド
```bash
cd /path/to/ros2_ws
colcon build --packages-select sabacan_msgs sabacan
source install/setup.bash
```
1つ目のターミナルでros2_socketcanを起動
```bash
source install/setup.bash
sudo ./src/gakurobo2026_common/can_setup.bash
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0
```
2つ目のターミナルでsabacan_check_id_rq_nodeを起動
```bash
# 起動（デフォルトで1秒ごとにRQ送信）
source install/setup.bash
ros2 run sabacan sabacan_check_id_rq_node
```

## ログ出力例
1秒おきに、基板の種類(data_type)と基板の番号(board_id)が出力されます。  
他にもCANバス上にいろんな基板が接続されていると関係のないログも表示されます。
```
user@user-ThinkPad-E14-Gen-6 ~/ros2_ws
$ ros2 run sabacan sabacan_check_id_rq_node
[INFO] [1764990501.547031875] [sabacan_check_id_rq_node]: Sending CAN frame: ID=0xF00001
[INFO] [1764990501.547947712] [sabacan_check_id_rq_node]: Received CAN frame: ID=0xF00001, DLC=0, Data=[01 38 00 00 00 00 00 00]
[INFO] [1764990501.548106532] [sabacan_check_id_rq_node]: Received CAN frame: ID=0x20001, DLC=1, Data=[04 38 00 00 00 00 00 00]
[INFO] [1764990501.548126437] [sabacan_check_id_rq_node]: Received data_type = ROBOMAS_V2, board_id = 2
```