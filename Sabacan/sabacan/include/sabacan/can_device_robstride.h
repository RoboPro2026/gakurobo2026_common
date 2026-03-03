/**
 * @file can_device_robstride.h
 * @author Yudai Yamaguchi (yudai.yy0804@gmail.com)
 * @brief robstride用のCANデバイスクラス
 * @note 実装の参考：https://github.com/RobStride/robstride_actuator_bridge/tree/master
 * @version 0.1
 * @date 2026-03-01
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "sabacan/can_device.h"

class RobstrideDriver : public CanDevice
{
public:
  uint8_t can_id = 0;
  uint8_t can_master_id = 0;
  uint8_t motor_mode_status = 0;
  float torque = 0.0f;
  float speed = 0.0f;
  float angle = 0.0f;
  uint16_t raw_angle = 0;
  float kp = 0.0f;
  float kd = 0.0f;
  float temperature = 0.0f;
  float prev_angle = 0.0f;
  float target_angle = 0.0f;
  RobstrideType robstride_type;

  // 受信データ
  uint8_t run_mode = 0;
  float iq_ref = 0.0f;
  float spd_ref = 0.0f;
  float limit_torque = 0.0f;
  float cur_kp = 0.0f;
  float cur_ki = 0.0f;
  float cur_filt_gain = 0.0f;
  float loc_ref = 0.0f;
  float limit_spd = 0.0f;
  float limit_cur = 0.0f;
  float mech_pos = 0.0f;
  float vbus = 0.0f;
  float loc_kp = 0.0f;
  float spd_kp = 0.0f;
  float spd_ki = 0.0f;
  float spd_filt_gain = 0.0f;
  float acc_rad = 0.0f;
  float vel_max = 0.0f;
  float acc_set = 0.0f;
  uint16_t epscan_time = 0;
  uint32_t can_timeout = 0;
  uint8_t zero_sta = 0;
  float add_offset = 0.0;

  static constexpr uint8_t PRIVATE_PROTOCOL = 0;
  static constexpr uint8_t CANOPEN_PROTOCOL = 1;
  static constexpr uint8_t MIT_PROTOCOL = 2;

private:
  float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
  {
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
  }

  int float_to_uint(float x, float x_min, float x_max, int bits)
  {
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
      x = x_max;
    else if (x < x_min)
      x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
  }

  float Byte_to_float(uint8_t * bytedata)
  {
    uint32_t data = bytedata[7] << 24 | bytedata[6] << 16 | bytedata[5] << 8 | bytedata[4];
    float data_float = *(float *)(&data);
    return data_float;
  }

public:
  RobstrideDriver(
    std::shared_ptr<CanDriver> _can_driver, int _can_id, int _can_master_id,
    RobstrideType _robstride_type)
  : CanDevice(_can_driver, 0, 0)  // data_typeはrobstrideなので0に設定
  {
    robstride_type = _robstride_type;
    can_id = _can_id;
    can_master_id = _can_master_id;
  }

  // ========== Private Protocol ==========
  void setOperationControlMode_MotorControlInstruction(
    float torque, float angle, float speed, float kp, float kd)
  {
    uint16_t u16_angle = 0, u16_speed = 0, u16_torque = 0, u16_kp = 0, u16_kd = 0;
    if (robstride_type == RobstrideType::RS05 || robstride_type == RobstrideType::EL05) {
      u16_torque = float_to_uint(torque, RS05::T_MIN, RS05::T_MAX, 16);
      u16_angle = float_to_uint(angle, RS05::P_MIN, RS05::P_MAX, 16);
      u16_speed = float_to_uint(speed, RS05::V_MIN, RS05::V_MAX, 16);
      u16_kp = float_to_uint(kp, RS05::KP_MIN, RS05::KP_MAX, 16);
      u16_kd = float_to_uint(kd, RS05::KD_MIN, RS05::KD_MAX, 16);
    }
    uint32_t id = 0x1 << 24 | u16_torque << 8 | can_id;
    uint8_t data[8] = {0};
    data[0] = u16_angle >> 8;
    data[1] = u16_angle & 0xFF;
    data[2] = u16_speed >> 8;
    data[3] = u16_speed & 0xFF;
    data[4] = u16_kp >> 8;
    data[5] = u16_kp & 0xFF;
    data[6] = u16_kd >> 8;
    data[7] = u16_kd & 0xFF;
    uint8_t dlc = 8;
    bool is_remote_frame = false;
    bool is_ext_id = true;
    can_driver->tx(id, data, dlc, is_remote_frame, is_ext_id);
  }

  void setMotorEnabledToRun(void)
  {
    uint32_t id = 0x3 << 24 | can_master_id << 8 | can_id;
    uint8_t data[8] = {0};
    uint8_t dlc = 8;
    bool is_remote_frame = false;
    bool is_ext_id = true;
    can_driver->tx(id, data, dlc, is_remote_frame, is_ext_id);
  }

  /**
   * @brief Set the Motor Stops Running object
   * 
   * @param val 0のときはデータがcleanされる。1のときはfaultもクリアされる
   */
  void setMotorStopsRunning(uint8_t val)
  {
    uint32_t id = 0x4 << 24 | can_master_id << 8 | can_id;
    uint8_t data[8] = {0};
    data[0] = val;
    uint8_t dlc = 8;
    bool is_remote_frame = false;
    bool is_ext_id = true;
    can_driver->tx(id, data, dlc, is_remote_frame, is_ext_id);
  }

  void setMotorMechanicalZero(void)
  {
    uint32_t id = 0x5 << 24 | can_master_id << 8 | can_id;
    uint8_t data[8] = {0};
    data[0] = 1;
    uint8_t dlc = 8;
    bool is_remote_frame = false;
    bool is_ext_id = true;
    can_driver->tx(id, data, dlc, is_remote_frame, is_ext_id);
  }

  void setMotorActivelyReportsFrames(bool enable)
  {
    uint32_t id = 0x18 << 24 | can_master_id << 8 | can_id;
    uint8_t data[8] = {0};
    data[0] = 1;
    data[1] = 2;
    data[2] = 3;
    data[3] = 4;
    data[4] = 5;
    data[5] = 6;
    data[6] = enable ? 1 : 0;
    // data[7]は何を送ればいいかわからないので、適当に0を送信
    data[7] = 0;
    uint8_t dlc = 8;
    bool is_remote_frame = false;
    bool is_ext_id = true;
    can_driver->tx(id, data, dlc, is_remote_frame, is_ext_id);
  }

  void setSingleParameterRead(uint16_t index)
  {
    uint32_t id = 0x11 << 24 | can_master_id << 8 | can_id;
    uint8_t data[8] = {0};
    data[0] = index & 0xff;
    data[1] = index >> 8;
    uint8_t dlc = 8;
    bool is_remote_frame = false;
    bool is_ext_id = true;
    can_driver->tx(id, data, dlc, is_remote_frame, is_ext_id);
  }

  void setSingleParameterWrite_uint32(uint16_t index, uint32_t value)
  {
    uint32_t id = 0x12 << 24 | can_master_id << 8 | can_id;
    uint8_t data[8] = {0};
    data[0] = index & 0xff;
    data[1] = index >> 8;
    data[4] = value & 0xff;
    data[5] = (value >> 8) & 0xff;
    data[6] = (value >> 16) & 0xff;
    data[7] = (value >> 24) & 0xff;
    uint8_t dlc = 8;
    bool is_remote_frame = false;
    bool is_ext_id = true;
    can_driver->tx(id, data, dlc, is_remote_frame, is_ext_id);
  }

  void setSingleParameterWrite_uint8(uint16_t index, uint8_t value)
  {
    uint32_t id = 0x12 << 24 | can_master_id << 8 | can_id;
    uint8_t data[8] = {0};
    data[0] = index & 0xff;
    data[1] = index >> 8;
    data[4] = value & 0xff;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    uint8_t dlc = 8;
    bool is_remote_frame = false;
    bool is_ext_id = true;
    can_driver->tx(id, data, dlc, is_remote_frame, is_ext_id);
  }

  void setSingleParameterWrite_uint16(uint16_t index, uint16_t value)
  {
    uint32_t u32_value = value;
    setSingleParameterWrite_uint32(index, u32_value);
  }

  void setSingleParameterWrite_float(uint16_t index, float value)
  {
    uint32_t id = 0x12 << 24 | can_master_id << 8 | can_id;
    uint8_t data[8] = {0};
    data[0] = index & 0xff;
    data[1] = index >> 8;
    memcpy(&data[4], &value, 4);
    uint8_t dlc = 8;
    bool is_remote_frame = false;
    bool is_ext_id = true;
    can_driver->tx(id, data, dlc, is_remote_frame, is_ext_id);
  }

  uint32_t getSingleParameterRead_uint32(uint8_t * data)
  {
    return (uint32_t)data[4] | ((uint32_t)data[5] << 8) | ((uint32_t)data[6] << 16) |
           ((uint32_t)data[7] << 24);
  }

  uint8_t getSingleParameterRead_uint8(uint8_t * data)
  {
    return (uint8_t)getSingleParameterRead_uint32(data);
  }

  uint16_t getSingleParameterRead_uint16(uint8_t * data)
  {
    return (uint16_t)getSingleParameterRead_uint32(data);
  }

  float getSingleParameterRead_float(uint8_t * data)
  {
    uint32_t u32_value = getSingleParameterRead_uint32(data);
    return *(float *)(&u32_value);
  }

  void receiveMotorFeedbackData(uint32_t id, uint8_t * data)
  {
    motor_mode_status = (id >> 22) & 0x3;
    angle = uint16_to_float((data[0] << 8) | data[1], RS05::P_MIN, RS05::P_MAX, 16);
    speed = uint16_to_float((data[2] << 8) | data[3], RS05::V_MIN, RS05::V_MAX, 16);
    torque = uint16_to_float((data[4] << 8) | data[5], RS05::T_MIN, RS05::T_MAX, 16);
    temperature = (float)((data[6] << 8) | data[7]) * 0.1;
  }

  bool receive(uint32_t id, uint8_t * data, uint8_t dlc, bool is_remote_frame, bool is_ext_id)
  {
    // Single Parameter Readの受信処理のみ行う
    if (is_remote_frame == true) return false;  // リモートフレームは無視する
    if (is_ext_id == false) return false;       // 拡張IDでないメッセージは無視する

    // type 2のデータを受信した場合
    if (
      ((id & 0xFF000000) >> 24 == 0x2) && ((id & 0xFF00) >> 8 == can_id) &&
      (id & 0x000000FF) == can_master_id) {
      receiveMotorFeedbackData(id, data);
      return true;
    }
    // type 24のデータを受信した場合
    if (
      ((id & 0xFF000000) >> 24 == 0x18) && ((id & 0xFF00) >> 8 == can_id) &&
      (id & 0x000000FF) == can_master_id) {
      receiveMotorFeedbackData(id, data);
      return true;
    }

    if ((id & 0xFF000000) >> 24 != 0x11)
      return false;  // Single Parameter Readのメッセージでない場合は無視する
    // Single Parameter Readの受信のみ、can_idとcan_master_idの順番が逆なので注意
    if (((id & 0x0000FF00) >> 8) != can_id) return false;  // can_idが違う場合は無視する
    uint16_t index = (data[1] << 8) | data[0];
    switch (index) {
      case RobstrideIndex::RUN_MODE:
        run_mode = getSingleParameterRead_uint8(data);
        break;
      case RobstrideIndex::IQ_REF:
        iq_ref = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::SPD_REF:
        spd_ref = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::LIMIT_TORQUE:
        limit_torque = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::CUR_KP:
        cur_kp = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::CUR_KI:
        cur_ki = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::CUR_FILT_GAIN:
        cur_filt_gain = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::LOC_REF:
        loc_ref = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::LIMIT_SPD:
        limit_spd = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::LIMIT_CUR:
        limit_cur = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::MECH_POS:
        mech_pos = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::VBUS:
        vbus = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::LOC_KP:
        loc_kp = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::SPD_KP:
        spd_kp = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::SPD_KI:
        spd_ki = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::SPD_FILT_GAIN:
        spd_filt_gain = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::ACC_RAD:
        acc_rad = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::VEL_MAX:
        vel_max = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::ACC_SET:
        acc_set = getSingleParameterRead_float(data);
        break;
      case RobstrideIndex::EPSCAN_TIME:
        epscan_time = getSingleParameterRead_uint16(data);
        break;
      case RobstrideIndex::CAN_TIMEOUT:
        can_timeout = getSingleParameterRead_uint32(data);
        break;
      case RobstrideIndex::ZERO_STA:
        zero_sta = getSingleParameterRead_uint8(data);
        break;
      case RobstrideIndex::ADD_OFFSET:
        add_offset = getSingleParameterRead_float(data);
        break;
      default:
        return false;  // indexが違う場合は無視する
    }
    return true;
  }

  // ========== MIT Protocol ==========

  // void setProtocolSwitching(uint8_t val)
  // {
  //   uint32_t id = 0xfff;
  //   uint8_t data[8] = {1, 2, 3, 4, 5, 6, val, 0};
  //   uint8_t dlc = 8;
  //   bool is_remote_frame = false;
  //   bool is_ext_id = true;
  //   can_driver->tx(id, data, dlc, is_remote_frame, is_ext_id);
  // }

  // // command

  // void setEnableMotorOperation(void)
  // {
  //   uint32_t id = board_id;
  //   uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
  //   uint8_t dlc = 8;
  //   bool is_remote_frame = false;
  //   bool is_ext_id = false;
  //   can_driver->tx(id, data, dlc, is_remote_frame, is_ext_id);
  // }

  // void setStopMotorOperation(void)
  // {
  //   uint32_t id = board_id;
  //   uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
  //   uint8_t dlc = 8;
  //   bool is_remote_frame = false;
  //   bool is_ext_id = false;
  //   can_driver->tx(id, data, dlc, is_remote_frame, is_ext_id);
  // }

  // /**
  //  * @brief 指令値を送信する
  //  *
  //  * @param angle
  //  * @param speed
  //  * @param torque
  //  * @param kp
  //  * @param kd
  //  */
  // void setMITDynamicParameter(float angle, float speed, float torque, float kp, float kd)
  // {
  //   uint32_t id = board_id;
  //   uint8_t data[8] = {0};
  //   if (robstride_type == RobstrideType::RS05 || robstride_type == RobstrideType::EL05) {
  //     data[0] = float_to_uint(angle, RS05::P_MIN, RS05::P_MAX, 16) >> 8;
  //     data[1] = float_to_uint(angle, RS05::P_MIN, RS05::P_MAX, 16) & 0xFF;
  //     data[2] = float_to_uint(speed, RS05::V_MIN, RS05::V_MAX, 16) >> 8;
  //     data[3] = float_to_uint(speed, RS05::V_MIN, RS05::V_MAX, 16) & 0xFF;
  //     data[4] = float_to_uint(kp, RS05::KP_MIN, RS05::KP_MAX, 16) >> 8;
  //     data[5] = float_to_uint(kp, RS05::KP_MIN, RS05::KP_MAX, 16) & 0xFF;
  //     data[6] = float_to_uint(kd, RS05::KD_MIN, RS05::KD_MAX, 16) >> 8;
  //     data[7] = float_to_uint(kd, RS05::KD_MIN, RS05::KD_MAX, 16) & 0xFF;
  //   }
  //   uint8_t dlc = 8;
  //   bool is_remote_frame = false;
  //   bool is_ext_id = false;
  //   can_driver->tx(id, data, dlc, is_remote_frame, is_ext_id);
  // }

  // bool receive(uint32_t id, uint8_t * data, uint8_t dlc, bool is_remote_frame, bool is_ext_id)
  // {
  //   if (dlc != 8) return false;         // データ長が8でないメッセージは無視する
  //   if (is_remote_frame) return false;  // リモートフレームは無視する
  //   if (is_ext_id) return false;        // 拡張IDのメッセージは無視する

  //   if (id == board_id) {
  //     uint16_t u16_angle = (data[1] << 8) | data[2];
  //     uint16_t u16_speed = (data[3] << 8) | ((data[3] & 0xf0) >> 4);
  //     uint16_t u16_torque = ((data[3] & 0x0f) << 8) | data[4];
  //     uint16_t u16_tmp_kp = (data[5] << 8) | data[6];

  //     if (robstride_type == RobstrideType::RS05 || robstride_type == RobstrideType::EL05) {
  //       raw_angle = u16_angle;
  //       angle = uint16_to_float(u16_angle, RS05::P_MIN, RS05::P_MAX, 16);
  //       speed = uint16_to_float(u16_speed, RS05::V_MIN, RS05::V_MAX, 12);
  //       torque = uint16_to_float(u16_torque, RS05::T_MIN, RS05::T_MAX, 12);
  //     }
  //   }

  //   return true;
  // }
};