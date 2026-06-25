# gakurobo2026_common

学生ロボコン2026 共通リポジトリ  
SabacanなどのR1とR2で共通で使うプログラムを管理するリポジトリです。  

## clone方法
```
cd ~/ros2_ws/src
git clone --recurse-submodules git@github.com:RoboPro2026/gakurobo2026_common.git
```

## submoduleの更新方法
```
# gakurobo2026_commonを更新
git pull origin main
# submoduleを更新
git submodule update --init --recursive
```

## CANのセットアップ

### ハードウェア CAN（socket_can / R2 など）

```bash
sudo ./can_setup.bash
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0
```

### Ethernet-CAN ゲートウェイ（eth2can / R1）

R1 では TCP 経由で CAN ゲートウェイに接続する `eth2can_node` を使います。  
`can_setup.bash` や `socket_can_bridge` は不要です。  
`r1_bringup.launch.py` から自動的に起動されます。詳細は [eth2can/README.md](eth2can/README.md) を参照してください。

## YDLIDARをbuild
1. YDLidar-SDKをビルドする（参考：https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_build_and_install.md）
```
cd YDLidar-SDK
mkdir build
cd build
cmake ..
make
sudo make install
```

2. ydlidar_ros2_driverをビルドする
```
colcon build --symlink-install
```

3. 必要かどうかわからないおまじない
```
sudo chmod 666 /dev/ttyUSB0
```
```
source install/setup.bash

```


# 各種ドキュメント

[sabacan(CANパッケージ)の使い方](Sabacan/README.md)

[sabacan_debugの使い方](sabacan_debug/README.md)

[sabacan_single_controlの使い方(ロボマス制御を扱いやすくするパッケージ)](sabacan_single_control/README.md)

[bno086の使い方](bno086/README.md)

[eth2can（Ethernet-CAN ゲートウェイブリッジ）の使い方](eth2can/README.md)

[単眼Lidar SDM15の使い方、ほかのYDLidarも同様の使い方です](https://nagaokaroboconproject.esa.io/posts/334)


