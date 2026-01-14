# gakurobo2026_common

学生ロボコン2026 共通リポジトリ  
SabacanなどのR1とR2で共通で使うプログラムを管理するリポジトリです。  

## clone方法
```
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/RoboPro2026/gakurobo2026_common.git
```

## submoduleの更新方法
```
# gakurobo2026_commonを更新
git pull origin main
# submoduleを更新
git submodule update --init --recursive
```

## CANのセットアップ
```
sudo ./can_setup.bash
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0
```

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