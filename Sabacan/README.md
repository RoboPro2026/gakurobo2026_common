# Sabacan

Sabacan は ROS 2 と CAN バスを繋ぐパッケージ群です。  
ロボマス制御基板・GPIO 基板・電源基板・LED 基板・Robstride モータを扱います。

## CAN のセットアップ

Sabacan ノードは `can_msgs/msg/Frame` を `from_can_bus` / `to_can_bus` トピックでやり取りします。  
CAN バスとの橋渡し方法は、ハードウェア構成によって2つの方式があります。

### 方式1: Ethernet-CAN ゲートウェイ（eth2can） — R1 推奨

TCP 経由で CAN ゲートウェイに接続します。`can_setup.bash` や SocketCAN ドライバは不要です。

```bash
# r1_bringup.launch.py から自動起動されるため、通常は手動操作不要
# 単体起動する場合:
ros2 run eth2can eth2can_node --ros-args -p device_ip:=192.168.1.100 -p device_port:=5000
```

ネットワーク設定（IP アドレス等）の詳細は [eth2can/README.md](../eth2can/README.md) を参照してください。

### 方式2: USB-CAN アダプタ（SocketCAN） — USB2CAN

USB 接続の CAN アダプタを使う方式です。まず CAN インターフェースを初期化してから ROS 2 ブリッジを起動します。

```bash
# CAN インターフェースの初期化（ビットレート 1Mbps）
sudo ./can_setup.bash

# ROS 2 SocketCAN ブリッジの起動
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0
```

`can_setup.bash` はリポジトリルートにあります。デフォルトのビットレートは `1,000,000 bps` です。

---

## 各基板のドキュメント

[ロボマス制御基板V2の使い方](./docs/robomasv2.md)

[GPIO基板の使い方](./docs/gpio.md)

[電源基板の使い方](./docs/power.md)

[LED基板の使い方](./docs/led.md)

[robstrideの使い方](./docs/robstride.md)

[example/id_rqを受信するサンプルプログラム](./docs/check_id_rq_node.md)
