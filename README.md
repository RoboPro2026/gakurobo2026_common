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