/**
 * @file can_device_robomas_driver_v1.h
 * @author Yamaguchi Yudai
 * @brief ロボマス制御基板V2のプログラム
 * @version 0.1
 * @date 2025-08-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "sabacan/can_device.h"

class RobomasDriverV2 : public CanDevice
{
public:
  static constexpr int N = 4;
  bool motor_state[N] = {0};
  uint8_t control[N] = {0};
  uint8_t control_mode[N] = {0};
  uint8_t control_motor[N] = {0};
  bool dob_en[N] = {0};
  bool abs_enc_en[N] = {0};
  bool md_guess_en[N] = {0};
  float abs_gear_ratio[N] = {0};
  int8_t cal_rq[N] = {0};
  float load_j[N] = {0};
  float load_d[N] = {0};
  float dob_cf[N] = {0};
  float omega_n[N] = {0};
  uint16_t can_timeout = {0};
  float current_torque[N] = {0};
  float trq_target[N] = {0};
  float current_speed[N] = {0};
  float speed_target[N] = {0};
  float torque_lim[N] = {0};
  float speed_gain_p[N] = {0};
  float speed_gain_i[N] = {0};
  float speed_gain_d[N] = {0};
  float current_pos[N] = {0};
  float pos_target[N] = {0};
  float speed_lim[N] = {0};
  float pos_gain_p[N] = {0};
  float pos_gain_i[N] = {0};
  float pos_gain_d[N] = {0};
  float abs_pos[N] = {0};
  float abs_speed[N] = {0};
  int32_t abs_turn_cnt[N] = {0};
  uint8_t vesc_mode[N] = {0};
  float vesc_target[N] = {0};
  float vesc_voltage[N] = {0};
  float vesc_current[N] = {0};
  float vesc_erpm[N] = {0};
  uint16_t monitor_period = 0;
  uint64_t monitor_reg1[N] = {0};
  uint64_t monitor_reg2[N] = {0};

  RobomasDriverV2(std::shared_ptr<CanDriver> _can_driver, int _board_id)
  : CanDevice(_can_driver, _board_id, DataType::ROBOMAS_V2)
  {
  }

  void setControl(
    int n, uint8_t _control_mode, uint8_t _control_motor, bool _dob_en, bool _abs_enc_en,
    bool _md_guess_en)
  {
    uint8_t _control = 0;
    _control |= _control_mode & 0x03;
    _control |= (_control_motor & 0x01) << 2;
    _control |= (uint8_t)_dob_en << 4;
    _control |= (uint8_t)_abs_enc_en << 5;
    _control |= (uint8_t)_md_guess_en << 6;
    tx(((n & 0xF) << 8) | RobomasV2::CONTROL, _control);
  }

  void setAbsGearRatio(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::ABS_GEAR_RATIO, val); }

  /**
   * @brief キャリブレーションのリクエスト
   *
   */
  void setCalRq(int n, int8_t val) { tx((n & 0xF) << 8 | RobomasV2::CAL_RQ, val); }

  void setLoad_J(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::LOAD_J, val); }

  void setLoad_D(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::LOAD_D, val); }

  /**
   * @brief 外乱オブザーバーのカットオフ周波数
   *
   */
  void setDob_CF(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::DOB_CF, val); }

  void setOmega_N(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::OMEGA_N, val); }

  void setCanTimeout(uint16_t val) { tx(RobomasV2::CAN_TIMEOUT, val); }

  void setTorqueTarget(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::TRQ_TARGET, val); }

  void setSpeedTarget(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::SPD_TARGET, val); }

  void setTorqueLimit(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::TRQ_LIM, val); }

  void setSpeedGainP(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::SPD_GAIN_P, val); }

  void setSpeedGainI(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::SPD_GAIN_I, val); }

  void setSpeedGainD(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::SPD_GAIN_D, val); }

  void setPosTarget(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::POS_TARGET, val); }

  void setSpeedLim(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::SPD_LIM, val); }

  void setPosGainP(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::POS_GAIN_P, val); }

  void setPosGainI(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::POS_GAIN_I, val); }

  void setPosGainD(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::POS_GAIN_D, val); }

  void setAbsTurnCnt(int n, int32_t val) { tx((n & 0xF) << 8 | RobomasV2::ABS_TURN_CNT, val); }

  void setVescMode(int n, uint8_t val) { tx((n & 0xF) << 8 | RobomasV2::VESC_MODE, val); }

  void setVescTarget(int n, float val) { tx((n & 0xF) << 8 | RobomasV2::VESC_TARGET, val); }

  void setMonitorPeriod(uint16_t val) { tx(RobomasV2::MONITOR_PERIOD, val); }

  void setMonitorReg1(int n, uint64_t val) { tx((n & 0xF) << 8 | RobomasV2::MONITOR_REG1, val); }

  void setMonitorReg2(int n, uint64_t val) { tx((n & 0xF) << 8 | RobomasV2::MONITOR_REG2, val); }

  bool receive(CanFrame frame)
  {
    int id = frame.register_id & 0xff;
    int n = (frame.register_id & 0xf00) >> 8;

    // 違う基板へのメッセージは無視する
    if (frame.data_type != this->data_type) return false;
    if (frame.board_id != this->board_id) return false;
    // motor_numberが範囲外の場合は無視する
    if (!(0 <= n && n < N)) return false;

    switch (id) {
      case RobomasV2::MOTOR_STATE:
        assign(&motor_state[n], frame.data);
        break;
      case RobomasV2::CONTROL:
        assign(&control[n], frame.data);
        control_mode[n] = control[n] & 0x03;
        control_motor[n] = (control[n] >> 2) & 0x03;
        dob_en[n] = (control[n] >> 4) & 0x01;
        abs_enc_en[n] = (control[n] >> 5) & 0x01;
        md_guess_en[n] = (control[n] >> 6) & 0x01;
        break;
      case RobomasV2::ABS_GEAR_RATIO:
        assign(&abs_gear_ratio[n], frame.data);
        break;
      case RobomasV2::CAL_RQ:
        assign(&cal_rq[n], frame.data);
        break;
      case RobomasV2::LOAD_J:
        assign(&load_j[n], frame.data);
        break;
      case RobomasV2::LOAD_D:
        assign(&load_d[n], frame.data);
        break;
      case RobomasV2::DOB_CF:
        assign(&dob_cf[n], frame.data);
        break;
      case RobomasV2::OMEGA_N:
        assign(&omega_n[n], frame.data);
        break;
      case RobomasV2::CAN_TIMEOUT:
        assign(&can_timeout, frame.data);
        break;
      case RobomasV2::TRQ:
        assign(&current_torque[n], frame.data);
        break;
      case RobomasV2::TRQ_TARGET:
        assign(&trq_target[n], frame.data);
        break;
      case RobomasV2::SPD:
        assign(&current_speed[n], frame.data);
        break;
      case RobomasV2::SPD_TARGET:
        assign(&speed_target[n], frame.data);
        break;
      case RobomasV2::TRQ_LIM:
        assign(&torque_lim[n], frame.data);
        break;
      case RobomasV2::SPD_GAIN_P:
        assign(&speed_gain_p[n], frame.data);
        break;
      case RobomasV2::SPD_GAIN_I:
        assign(&speed_gain_i[n], frame.data);
        break;
      case RobomasV2::SPD_GAIN_D:
        assign(&speed_gain_d[n], frame.data);
        break;
      case RobomasV2::POS:
        assign(&current_pos[n], frame.data);
        break;
      case RobomasV2::POS_TARGET:
        assign(&pos_target[n], frame.data);
        break;
      case RobomasV2::SPD_LIM:
        assign(&speed_lim[n], frame.data);
        break;
      case RobomasV2::POS_GAIN_P:
        assign(&pos_gain_p[n], frame.data);
        break;
      case RobomasV2::POS_GAIN_I:
        assign(&pos_gain_i[n], frame.data);
        break;
      case RobomasV2::POS_GAIN_D:
        assign(&pos_gain_d[n], frame.data);
        break;
      case RobomasV2::ABS_POS:
        assign(&abs_pos[n], frame.data);
        break;
      case RobomasV2::ABS_SPD:
        assign(&abs_speed[n], frame.data);
        break;
      case RobomasV2::ABS_TURN_CNT:
        assign(&abs_turn_cnt[n], frame.data);
        break;
      case RobomasV2::VESC_MODE:
        assign(&vesc_mode[n], frame.data);
        break;
      case RobomasV2::VESC_TARGET:
        assign(&vesc_target[n], frame.data);
        break;
      case RobomasV2::VESC_VOLTAGE:
        assign(&vesc_voltage[n], frame.data);
        break;
      case RobomasV2::VESC_CURRENT:
        assign(&vesc_current[n], frame.data);
        break;
      case RobomasV2::VESC_ERPM:
        assign(&vesc_erpm[n], frame.data);
        break;
      case RobomasV2::MONITOR_PERIOD:
        assign(&monitor_period, frame.data);
        break;
      case RobomasV2::MONITOR_REG1:
        assign(&monitor_reg1[n], frame.data);
        break;
      case RobomasV2::MONITOR_REG2:
        assign(&monitor_reg2[n], frame.data);
        break;
      default:
        return false;
    }
    return true;
  }
};