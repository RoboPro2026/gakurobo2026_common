string control_type
# MITモードの指令値
float32 mit_torque # (RS05の場合は-5.5~5.5, EL05の場合は-6.0~6.0 N)
float32 mit_pos # (RS05とEL05の場合は-4pi~4pi rad)
float32 mit_speed # (RS05とEL05の場合は-50.0~50.0rad/s)
float32 mit_kp # (RS05とEL05の場合は0.0~500.0)
float32 mit_kd # (RS05とEL 0.0~500.0)
# 電流モードの指令値（範囲の制限なし）
float32 current_ref
# 速度モードの指令値（範囲の制限なし）
float32 velocity_ref
# 位置制御モード(PP)の指令値（範囲の制限なし）
float32 pp_angle_ref
# 位置制御モード(CSP)の指令値（範囲の制限なし）
float32 csp_angle_ref