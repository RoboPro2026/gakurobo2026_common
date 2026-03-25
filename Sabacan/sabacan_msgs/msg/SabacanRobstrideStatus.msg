string control_type
float32 torque # Nm
float32 speed # rad/s
float32 pos # rad
float32 temperature # ℃
# モータのステータス
string motor_mode_status
# モータの故障情報
bool uncalibrated
bool gridlock_overload_fault
bool magnetic_coding_fault
bool overtemperature
bool three_phase_overcurrent_fault
bool undervoltage_fault