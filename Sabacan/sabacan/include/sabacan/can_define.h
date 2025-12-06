/**
 * @file can_define.h
 * @author Yamaguchi Yudai
 * @brief CANの各種定義
 * @version 0.1
 * @date 2025-08-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cstdint>

struct CanFrame
{
  bool is_request;
  uint8_t priority;
  uint8_t data_type;
  uint8_t board_id;
  uint16_t register_id;
  uint8_t data[8];
  uint8_t len;

  bool operator==(const CanFrame & can_frame) const
  {
    bool ret = true;
    ret &= (is_request == can_frame.is_request);
    ret &= (priority == can_frame.priority);
    ret &= (data_type == can_frame.data_type);
    ret &= (board_id == can_frame.board_id);
    ret &= (register_id == can_frame.register_id);
    ret &= (len == can_frame.len);
    for (int i = 0; i < len; i++) {
      ret &= (data[i] == can_frame.data[i]);
    }
    return ret;
  }
};

namespace DataType
{
constexpr uint8_t COMMON = 0x0;
constexpr uint8_t DENGEN = 0x1;
constexpr uint8_t ROBOMAS_V1 = 0x2;
constexpr uint8_t GPIO = 0x3;
constexpr uint8_t ROBOMAS_V2 = 0x4;
constexpr uint8_t LED = 0x5;
constexpr uint8_t FORCE_READ = 0xF;
};  // namespace DataType

namespace CommonRegisterID
{
constexpr uint16_t RQ = 0x0001;
constexpr uint16_t EMS = 0x000E;
constexpr uint16_t RESET_EMS = 0x000F;
}  // namespace CommonRegisterID

// 各基板のバージョンごとに名前空間を定義
// そうすることで、基板のバージョンが更新されたときに、変更コストを最小限にできる。

namespace RobomasV1
{
constexpr uint16_t MOTOR_TYPE = 0x01;
constexpr uint16_t CONTROL_TYPE = 0x02;
constexpr uint16_t GEAR_RATIO = 0x03;
constexpr uint16_t MOTOR_STATE = 0x04;
constexpr uint16_t CAN_TIMEOUT = 0x05;
constexpr uint16_t PWM = 0x10;
constexpr uint16_t PWM_TARGET = 0x11;
constexpr uint16_t SPEED = 0x20;
constexpr uint16_t SPEED_TARGET = 0x21;
constexpr uint16_t PWM_LIM = 0x22;  // トルク制限
constexpr uint16_t SPEED_PID_P = 0x23;
constexpr uint16_t SPEED_PID_I = 0x24;
constexpr uint16_t SPEED_PID_D = 0x25;
constexpr uint16_t POSITION = 0x30;
constexpr uint16_t POSITION_TARGET = 0x31;
constexpr uint16_t SPEED_LIM = 0x32;
constexpr uint16_t POSITION_PID_P = 0x33;
constexpr uint16_t POSITION_PID_I = 0x34;
constexpr uint16_t POSITION_PID_D = 0x35;
constexpr uint16_t POSITION_ABS = 0x36;
constexpr uint16_t SPEED_ABS = 0x37;
constexpr uint16_t ABS_ENC_INV = 0x38;
constexpr uint16_t ABS_TURN_CNT = 0x39;
constexpr uint16_t MONITOR_PERIOD = 0xF0;
constexpr uint16_t MONITOR_REGISTER = 0xF1;

constexpr int MOTOR_TYPE_ROBOMAS = 0x00;
constexpr int MOTOR_TYPE_VESC = 0x01;

constexpr int CONTROL_TYPE_PWM_MODE = 0x00;
constexpr int CONTROL_TYPE_SPEED_MODE = 0x01;
constexpr int CONTROL_TYPE_POSITION_MODE = 0x02;
constexpr int CONTROL_TYPE_ABS_POSITION_MODE = 0x03;

constexpr float GEAR_RATIO_M2006 = 36.0f;
constexpr float GEAR_RATIO_M3508 = 19.0f;

}  // namespace RobomasV1

namespace RobomasV2
{
constexpr uint16_t MOTOR_STATE = 0x01;
constexpr uint16_t CONTROL = 0x02;
constexpr uint16_t ABS_GEAR_RATIO = 0x05;
constexpr uint16_t CAL_RQ = 0x06;
constexpr uint16_t LOAD_J = 0x07;
constexpr uint16_t LOAD_D = 0x08;
constexpr uint16_t DOB_CF = 0x09;
constexpr uint16_t CAN_TIMEOUT = 0x0F;
constexpr uint16_t TRQ = 0x10;
constexpr uint16_t TRQ_TARGET = 0x11;
constexpr uint16_t SPD = 0x20;
constexpr uint16_t SPD_TARGET = 0x21;
constexpr uint16_t TRQ_LIM = 0x22;
constexpr uint16_t SPD_GAIN_P = 0x23;
constexpr uint16_t SPD_GAIN_I = 0x24;
constexpr uint16_t SPD_GAIN_D = 0x25;
constexpr uint16_t POS = 0x30;
constexpr uint16_t POS_TARGET = 0x31;
constexpr uint16_t SPD_LIM = 0x32;
constexpr uint16_t POS_GAIN_P = 0x33;
constexpr uint16_t POS_GAIN_I = 0x34;
constexpr uint16_t POS_GAIN_D = 0x35;
constexpr uint16_t ABS_POS = 0x3A;
constexpr uint16_t ABS_SPD = 0x3B;
constexpr uint16_t ABS_TURN_CNT = 0x3C;
constexpr uint16_t VESC_MODE = 0x40;
constexpr uint16_t VESC_TARGET = 0x41;
constexpr uint16_t VESC_VOLTAGE = 0x42;
constexpr uint16_t VESC_CURRENT = 0x43;
constexpr uint16_t VESC_ERPM = 0x44;
constexpr uint16_t MONITOR_PERIOD = 0xF0;
constexpr uint16_t MONITOR_REG1 = 0xF1;
constexpr uint16_t MONITOR_REG2 = 0xF2;

constexpr int CONTROL_MODE_TRQ = 0x00;
constexpr int CONTROL_MODE_SPD = 0x01;
constexpr int CONTROL_MODE_POS = 0x02;

// モーターM2006、モータードライバC610の場合
constexpr int CONTROL_MOTOR_C610 = 0x00;
// モーターM3508、モータードライバC620の場合
constexpr int CONTROL_MOTOR_C620 = 0x01;
// VESC用、CANプロトコルにはないが、ROS 2のパラメータ設定時に使用
constexpr int CONTROL_MOTOR_VESC = 0xFF;

constexpr int VESC_MODE_DISABLE = 0x00;
constexpr int VESC_MODE_PWM = 0x01;
constexpr int VESC_MODE_CURRENT = 0x02;
constexpr int VESC_MODE_SPEED = 0x03;
constexpr int VESC_MODE_POSITION = 0x04;
}  // namespace RobomasV2

namespace GPIO
{

constexpr uint16_t PORT_MODE = 0x01;
constexpr uint16_t PORT_READ = 0x02;
constexpr uint16_t PORT_WRITE = 0x03;
constexpr uint16_t PORT_INT_EN = 0x04;
constexpr uint16_t ESC_MODE_EN = 0x05;
constexpr uint16_t PWM_PERIOD = 0x10;
constexpr uint16_t PWM_DUTY = 0x20;
constexpr uint16_t MONITOR_PERIOD = 0xf0;
constexpr uint16_t MONITOR_REG = 0xf1;

}  // namespace GPIO