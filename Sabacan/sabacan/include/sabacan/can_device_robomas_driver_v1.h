/**
 * @file can_device_robomas_driver_v1.h
 * @author Yamaguchi Yudai
 * @brief ロボマス制御基板V1のプログラム
 * @version 0.1
 * @date 2025-08-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "sabacan/can_device.h"

class RobomasDriverV1 : public CanDevice
{
public:
  static constexpr int N = 4;
  uint8_t motor_type[N];
  uint8_t control_type[N];
  float gear_ratio[N];
  bool motor_state[N];
  float current_pwm[N];  //(電流)
  float pwm_target[N];   //(電流目標値)
  float current_speed[N];
  float speed_target[N];
  float pwm_lim[N];
  float speed_gain_p[N];
  float speed_gain_i[N];
  float speed_gain_d[N];
  float current_pos[N];
  float pos_target[N];
  float speed_lim[N];
  float pos_gain_p[N];
  float pos_gain_i[N];
  float pos_gain_d[N];
  float current_abs_pos[N];
  float current_abs_speed[N];
  bool abs_enc_inv[N];
  int32_t abs_turn_cnt[N];

  uint16_t can_timeout;
  uint16_t monitor_period;
  uint64_t monitor_reg[N];

  RobomasDriverV1(std::shared_ptr<CanDriver> _can_driver, int _board_id)
  : CanDevice(_can_driver, _board_id, DataType::ROBOMAS_V1)
  {
  }

  void setMotorType(int n, uint8_t _motor_type)
  {
    tx(((n & 0xF) << 8) | RobomasV1::MOTOR_TYPE, _motor_type);
  }

  void setControlType(int n, uint8_t _control_type)
  {
    tx(((n & 0xF) << 8) | RobomasV1::CONTROL_TYPE, _control_type);
  }

  void setGearRatio(int n, float _val) { tx(((n & 0xF) << 8) | RobomasV1::CONTROL_TYPE, _val); }

  void setCanTimeout(uint16_t _val) { tx(RobomasV1::CAN_TIMEOUT, _val); }

  void setPwmTarget(int n, float _target) { tx(((n & 0xF) << 8) | RobomasV1::PWM_TARGET, _target); }

  void setSpeedTarget(int n, float _target)
  {
    tx(((n & 0xF) << 8) | RobomasV1::SPEED_TARGET, _target);
  }

  void setPwmLim(int n, float _val) { tx(((n & 0xF) << 8) | RobomasV1::PWM_LIM, _val); }

  void setSpeedGainP(int n, float _val) { tx(((n & 0xF) << 8) | RobomasV1::SPEED_PID_P, _val); }

  void setSpeedGainI(int n, float _val) { tx(((n & 0xF) << 8) | RobomasV1::SPEED_PID_I, _val); }

  void setSpeedGainD(int n, float _val) { tx(((n & 0xF) << 8) | RobomasV1::SPEED_PID_D, _val); }

  void setPosTarget(int n, float _target)
  {
    tx(((n & 0xF) << 8) | RobomasV1::POSITION_TARGET, _target);
  }

  void setSpeedLim(int n, float _val) { tx(((n & 0xF) << 8) | RobomasV1::SPEED_LIM, _val); }

  void setPosGainP(int n, float _val) { tx(((n & 0xF) << 8) | RobomasV1::POSITION_PID_P, _val); }

  void setPosGainI(int n, float _val) { tx(((n & 0xF) << 8) | RobomasV1::POSITION_PID_I, _val); }

  void setPosGainD(int n, float _val) { tx(((n & 0xF) << 8) | RobomasV1::POSITION_PID_D, _val); }

  void setAbsEncInv(int n, bool is_abs_enc_inv)
  {
    tx(((n & 0xF) << 8) | RobomasV1::ABS_ENC_INV, is_abs_enc_inv);
  }

  void setAbsTurnCnt(int n, int32_t _val) { tx(((n & 0xF) << 8) | RobomasV1::ABS_TURN_CNT, _val); }

  void setMonitorPeriod(uint16_t _monitor_period)
  {
    tx(RobomasV1::MONITOR_PERIOD, _monitor_period);
  }

  void setMonitorReg(int n, uint64_t _monitor_reg)
  {
    tx(((n & 0xF) << 8) | RobomasV1::MONITOR_REGISTER, _monitor_reg);
  }

  bool receive(CanFrame frame)
  {
    int id = frame.register_id & 0xff;
    int n = (frame.register_id & 0xf00) >> 8;
    if (!(0 <= n && n < N)) {
      return false;
    }
    switch (id) {
      case RobomasV1::MOTOR_TYPE:
        assign(&motor_type[n], frame.data);
        break;
      case RobomasV1::CONTROL_TYPE:
        assign(&control_type[n], frame.data);
        break;
      case RobomasV1::GEAR_RATIO:
        assign(&gear_ratio[n], frame.data);
        break;
      case RobomasV1::MOTOR_STATE:
        assign(&motor_state[n], frame.data);
        break;
      case RobomasV1::CAN_TIMEOUT:
        assign(&can_timeout, frame.data);
        break;
      case RobomasV1::PWM:
        assign(&current_pwm[n], frame.data);
        break;
      case RobomasV1::PWM_TARGET:
        assign(&pwm_target[n], frame.data);
        break;
      case RobomasV1::SPEED:
        assign(&current_speed[n], frame.data);
        break;
      case RobomasV1::SPEED_TARGET:
        assign(&speed_target[n], frame.data);
        break;
      case RobomasV1::PWM_LIM:
        assign(&pwm_lim[n], frame.data);
        break;
      case RobomasV1::SPEED_PID_P:
        assign(&speed_gain_p[n], frame.data);
        break;
      case RobomasV1::SPEED_PID_I:
        assign(&speed_gain_i[n], frame.data);
        break;
      case RobomasV1::SPEED_PID_D:
        assign(&speed_gain_d[n], frame.data);
        break;
      case RobomasV1::POSITION:
        assign(&current_pos[n], frame.data);
        break;
      case RobomasV1::POSITION_TARGET:
        assign(&pos_target[n], frame.data);
        break;
      case RobomasV1::SPEED_LIM:
        assign(&speed_lim[n], frame.data);
        break;
      case RobomasV1::POSITION_PID_P:
        assign(&pos_gain_p[n], frame.data);
        break;
      case RobomasV1::POSITION_PID_I:
        assign(&pos_gain_i[n], frame.data);
        break;
      case RobomasV1::POSITION_PID_D:
        assign(&pos_gain_d[n], frame.data);
        break;
      case RobomasV1::POSITION_ABS:
        assign(&current_abs_pos[n], frame.data);
        break;
      case RobomasV1::SPEED_ABS:
        assign(&current_abs_speed[n], frame.data);
        break;
      case RobomasV1::ABS_ENC_INV:
        assign(&abs_enc_inv[n], frame.data);
        break;
      case RobomasV1::ABS_TURN_CNT:
        assign(&abs_turn_cnt[n], frame.data);
        break;
      case RobomasV1::MONITOR_PERIOD:
        assign(&monitor_period, frame.data);
        break;
      case RobomasV1::MONITOR_REGISTER:
        assign(&monitor_reg[n], frame.data);
        break;
      default:
        return false;
    }

    return true;
  }
};