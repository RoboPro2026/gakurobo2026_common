# PWM入力のフィードバックデータ
std_msgs/Header header
uint8 pin_number   # 0..8
float32 frequency  # Hz（計算不能時は0）
float32 duty       # 0.0..1.0（計算不能時は0）
uint16 pwm_period  # 基板が保持するソフトウェアPWM周期（参考値）
uint16 pwm_duty    # 基板が保持するソフトウェアPWM duty（参考値）




