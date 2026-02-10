# `sabacan_power_node` の使い方

`sabacan_power_node` は、電源基板をROS 2で扱うためのノードです。電源基板の状態（`PCU_STATE` / 電圧 / 電流）を定期的にpublishし、非常停止（EMS）の発報/解除をトピック経由で行えます。

## 1. ノードの起動

ノードを起動する際は、ROSパラメータとして `board_id` を**必ず指定**する必要があります。  
※電源基板はID0を設定とする前提で作られています。  
そもそも、他のIDにDIPスイッチをいじって変更することはできません。  

```bash
# ros2_ws でビルドとソース
cd /path/to/ros2_ws
colcon build --packages-select sabacan_msgs sabacan
source install/setup.bash

# ノード起動 (board_id=0 の例)
ros2 run sabacan sabacan_power_node --ros-args -p board_id:=0
```

## 2. パラメータ設定

ノード起動時に、電源基板へ初期パラメータを書き込みます（`power_init()`）。
各パラメータ送信ごとに約10msのディレイを入れてあります。

主要なパラメータ:
  * `board_id` (int64, **必須**): 基板のCAN ID (0〜9)。
  * `enable_initialize`（bool、デフォルト：`true`） ノード起動時にCANの初期化データを送信するか選択する。
  * `cell_n` (int64, デフォルト: 6): リポのセル数。
  * `ex_ems_trg` (int64, デフォルト: 0): 異常時にソフトウェア非常停止を入れるかのビット設定。
    * bit2: OVA_EMS_EN, bit3: UVA_EMS_EN, bit4: OIA_EMS_EN
  * `common_ems_en` (bool, デフォルト: true): 非常停止がかかった時にCommonRegの非常停止を発報するか。
  * `v_limit_high` (double, デフォルト: 4.3): セル当たりの過電圧閾値 [V]。
  * `v_limit_low` (double, デフォルト: 3.7): セル当たりの低電圧閾値 [V]。
  * `i_limit` (double, デフォルト: 100.0): 過電流閾値 [A]。
  * `enable_monitor_period` (bool, デフォルト: true): 周期モニタの有効/無効。
  * `monitor_period` (int64, デフォルト: 50): モニタ周期 [ms]。
  * `monitor_reg` (int64, デフォルト: `PCU_STATE/OUT_V/OUT_I` を含む): モニタするレジスタのビットマスク。

## 3. 非常停止の指令 (トピック)

  * トピック: `/sabacan_power_ref<board_id>`
  * メッセージ: `sabacan_msgs/msg/SabacanPowerRef`
    * `is_ems` (bool): `true` で非常停止、`false` で非常停止解除

**例 (board_id=0 で非常停止):**
```bash
ros2 topic pub -1 /sabacan_power_ref0 sabacan_msgs/msg/SabacanPowerRef "{is_ems: true}"
```

**例 (board_id=0 で非常停止解除):**
```bash
ros2 topic pub -1 /sabacan_power_ref0 sabacan_msgs/msg/SabacanPowerRef "{is_ems: false}"
```

## 4. 状態の取得 (トピック)

`sabacan_power_node` は電源基板の状態を周期的にpublishします。

  * トピック: `/sabacan_power_status<board_id>`
  * メッセージ: `sabacan_msgs/msg/SabacanPowerStatus`
    * `pcu_state` (float32)
    * `out_v` (float32)
    * `out_i` (float32)

**例 (board_id=0 の状態を表示):**
```bash
ros2 topic echo /sabacan_power_status0
```

## 5. CANトピック

下記は共通です。
  * publish: `/to_can_bus` (`can_msgs/msg/Frame`)
  * subscribe: `/from_can_bus` (`can_msgs/msg/Frame`)

## monitor_regの設定例

電源基板の `monitor_reg` は64bitマスクで、`(1ULL << RegisterID)` のORで指定します。
（`RegisterID` が大きいレジスタは64bitマスクで表現できないため、通常は `0x00`〜`0x3F` の範囲で使います。）

対象レジスタ（`can_define.h/namespace Power`）:
- `PCU_STATE=0x0001`, `OUT_V=0x0010`, `OUT_I=0x0020`

例) `PCU_STATE/OUT_V/OUT_I` をモニタ
- `reg = (1<<0x01) | (1<<0x10) | (1<<0x20)`
```bash
ros2 run sabacan sabacan_power_node --ros-args -p board_id:=0 \
  -p monitor_period:=50 \
  -p monitor_reg:=0x0000000100010002
```

