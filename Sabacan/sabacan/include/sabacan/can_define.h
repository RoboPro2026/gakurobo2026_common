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
constexpr uint8_t POWER = 0x1;
constexpr uint8_t ROBOMAS_V1 = 0x2;
constexpr uint8_t GPIO = 0x3;
constexpr uint8_t ROBOMAS_V2 = 0x4;
constexpr uint8_t LED = 0x5;
constexpr uint8_t FORCE_READ = 0xF;
};  // namespace DataType

namespace CommonRegisterID
{
constexpr uint16_t NOP = 0x0000;
constexpr uint16_t RQ = 0x0001;
constexpr uint16_t EMS = 0x000E;
constexpr uint16_t RESET_EMS = 0x000F;
}  // namespace CommonRegisterID

// 各基板のバージョンごとに名前空間を定義
// そうすることで、基板のバージョンが更新されたときに、変更コストを最小限にできる。

namespace Power
{
constexpr uint16_t NOP = 0x0000;
constexpr uint16_t PCU_STATE = 0x0001;
constexpr uint16_t CELL_N = 0x0002;
constexpr uint16_t EX_EMS_TRG = 0x0003;
constexpr uint16_t EMS_RQ = 0x0004;
constexpr uint16_t COMMON_EMS_EN = 0x0005;
constexpr uint16_t OUT_V = 0x0010;
constexpr uint16_t V_LIMIT_HIGH = 0x0011;
constexpr uint16_t V_LIMIT_LOW = 0x0012;
constexpr uint16_t OUT_I = 0x0020;
constexpr uint16_t I_LIMIT = 0x0021;
constexpr uint16_t MONITOR_PERIOD = 0x00F0;
constexpr uint16_t MONITOR_REG = 0x00F1;

constexpr uint8_t PCU_STATE_EMS_BIT = 0;
constexpr uint8_t PCU_STATE_SOFT_EMS_BIT = 1;
constexpr uint8_t PCU_STATE_OVA_BIT = 2;
constexpr uint8_t PCU_STATE_UVA_BIT = 3;
constexpr uint8_t PCU_STATE_OIA_BIT = 4;

constexpr uint8_t EX_EMS_TRG_OVA_EMS_EN_BIT = 2;
constexpr uint8_t EX_EMS_TRG_UVA_EMS_EN_BIT = 3;
constexpr uint8_t EX_EMS_TRG_OIA_EMS_EN_BIT = 4;
}  // namespace Power

namespace RobomasV1
{
constexpr uint16_t NOP = 0x0000;
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
constexpr uint16_t NOP = 0x0000;
constexpr uint16_t MOTOR_STATE = 0x01;
constexpr uint16_t CONTROL = 0x02;
constexpr uint16_t ABS_GEAR_RATIO = 0x05;
constexpr uint16_t CAL_RQ = 0x06;
constexpr uint16_t LOAD_J = 0x07;
constexpr uint16_t LOAD_D = 0x08;
constexpr uint16_t DOB_CF = 0x09;
constexpr uint16_t OMEGA_N = 0x0A;
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
constexpr uint16_t NOP = 0x0000;
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

namespace LED
{
constexpr uint16_t NOP = 0x0000;
constexpr uint16_t LED_MODE = 0x0040;
constexpr uint16_t ENABLE_AUTO_TRANSITION = 0x0041;
constexpr uint16_t EMG_BLINK_PERIOD = 0x0042;
constexpr uint16_t EMG_COLOR = 0x0050;
constexpr uint16_t LED_REF = 0x0060;
constexpr uint16_t MONITOR_PERIOD = 0x00F0;
constexpr uint16_t MONITOR_REG1 = 0x00F1;
constexpr uint16_t MONITOR_REG2 = 0x00F2;

constexpr uint8_t MODE_NORMAL = 0x00;
constexpr uint8_t MODE_EMG = 0x01;
}  // namespace LED

namespace RS02
{
constexpr float P_MIN = -12.57f;
constexpr float P_MAX = 12.57f;
constexpr float V_MIN = -44.0f;
constexpr float V_MAX = 44.0f;
constexpr float KP_MIN = 0.0f;
constexpr float KP_MAX = 500.0f;
constexpr float KD_MIN = 0.0f;
constexpr float KD_MAX = 5.0f;
constexpr float T_MIN = -17.0f;
constexpr float T_MAX = 17.0f;
}  // namespace RS02

namespace RS05
{
constexpr float P_MIN = -12.57f;
constexpr float P_MAX = 12.57f;
constexpr float V_MIN = -50.0f;
constexpr float V_MAX = 50.0f;
constexpr float KP_MIN = 0.0f;
constexpr float KP_MAX = 500.0f;
constexpr float KD_MIN = 0.0f;
constexpr float KD_MAX = 5.0f;
constexpr float T_MIN = -5.5f;
constexpr float T_MAX = 5.5f;
}  // namespace RS05

namespace EL05
{
constexpr float P_MIN = -12.57f;
constexpr float P_MAX = 12.57f;
constexpr float V_MIN = -50.0f;
constexpr float V_MAX = 50.0f;
constexpr float KP_MIN = 0.0f;
constexpr float KP_MAX = 500.0f;
constexpr float KD_MIN = 0.0f;
constexpr float KD_MAX = 5.0f;
constexpr float T_MIN = -6.0f;
constexpr float T_MAX = 6.0f;
}  // namespace EL05

enum class RobstrideType
{
  RS02,
  RS05,
  EL05,
};

namespace RobstrideIndex
{
constexpr uint16_t RUN_MODE = 0x7005;
constexpr uint16_t IQ_REF = 0x7006;
constexpr uint16_t SPD_REF = 0x700A;
constexpr uint16_t LIMIT_TORQUE = 0x700B;
constexpr uint16_t CUR_KP = 0x7010;
constexpr uint16_t CUR_KI = 0x7011;
constexpr uint16_t CUR_FILT_GAIN = 0x7014;
constexpr uint16_t LOC_REF = 0x7016;
constexpr uint16_t LIMIT_SPD = 0x7017;
constexpr uint16_t LIMIT_CUR = 0x7018;
constexpr uint16_t MECH_POS = 0x7019;
constexpr uint16_t IQF = 0x701A;
constexpr uint16_t MECH_VEL = 0x701B;
constexpr uint16_t VBUS = 0x701C;
constexpr uint16_t LOC_KP = 0x701E;
constexpr uint16_t SPD_KP = 0x701F;
constexpr uint16_t SPD_KI = 0x7020;
constexpr uint16_t SPD_FILT_GAIN = 0x7021;
constexpr uint16_t ACC_RAD = 0x7022;
constexpr uint16_t VEL_MAX = 0x7024;
constexpr uint16_t ACC_SET = 0x7025;
constexpr uint16_t EPSCAN_TIME = 0x7026;
constexpr uint16_t CAN_TIMEOUT = 0x7028;
constexpr uint16_t ZERO_STA = 0x7029;
constexpr uint16_t ADD_OFFSET = 0x702B;
};  // namespace RobstrideIndex