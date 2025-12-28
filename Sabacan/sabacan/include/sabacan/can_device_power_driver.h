/**
 * @file can_device_power_driver.h
 * @author Yamaguchi Yudai
 * @brief 電源基板をCANで動かすプログラム
 * @version 0.1
 * @date 2025-12-28
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "sabacan/can_device.h"

class PowerDriver : public CanDevice
{
public:
  uint8_t pcu_state = 0;
  uint8_t cell_n = 0;
  uint8_t ex_ems_trg = 0;
  bool ems_rq = false;
  bool common_ems_en = true;
  float out_v = 0.0f;
  float v_limit_high = 0.0f;
  float v_limit_low = 0.0f;
  float out_i = 0.0f;
  float i_limit = 0.0f;
  uint16_t monitor_period = 0;
  uint64_t monitor_reg = 0;

  PowerDriver(std::shared_ptr<CanDriver> _can_driver, int _board_id)
  : CanDevice(_can_driver, _board_id, DataType::POWER)
  {
  }

  void setCellN(uint8_t val) { tx(Power::CELL_N, val); }

  void setExEmsTrg(uint8_t val) { tx(Power::EX_EMS_TRG, val); }

  void setEmsRq(bool val) { tx(Power::EMS_RQ, val); }

  void setCommonEmsEn(bool val) { tx(Power::COMMON_EMS_EN, val); }

  void setVLimitHigh(float val) { tx(Power::V_LIMIT_HIGH, val); }

  void setVLimitLow(float val) { tx(Power::V_LIMIT_LOW, val); }

  void setILimit(float val) { tx(Power::I_LIMIT, val); }

  void setMonitorPeriod(uint16_t val) { tx(Power::MONITOR_PERIOD, val); }

  void setMonitorReg(uint64_t val) { tx(Power::MONITOR_REG, val); }

  bool receive(CanFrame frame)
  {
    // 違う基板へのメッセージは無視する
    if (frame.data_type != this->data_type) return false;
    if (frame.board_id != this->board_id) return false;

    switch (frame.register_id) {
      case Power::PCU_STATE:
        assign(&pcu_state, frame.data);
        break;
      case Power::CELL_N:
        assign(&cell_n, frame.data);
        break;
      case Power::EX_EMS_TRG:
        assign(&ex_ems_trg, frame.data);
        break;
      case Power::EMS_RQ:
        assign(&ems_rq, frame.data);
        break;
      case Power::COMMON_EMS_EN:
        assign(&common_ems_en, frame.data);
        break;
      case Power::OUT_V:
        assign(&out_v, frame.data);
        break;
      case Power::V_LIMIT_HIGH:
        assign(&v_limit_high, frame.data);
        break;
      case Power::V_LIMIT_LOW:
        assign(&v_limit_low, frame.data);
        break;
      case Power::OUT_I:
        assign(&out_i, frame.data);
        break;
      case Power::I_LIMIT:
        assign(&i_limit, frame.data);
        break;
      case Power::MONITOR_PERIOD:
        assign(&monitor_period, frame.data);
        break;
      case Power::MONITOR_REG:
        assign(&monitor_reg, frame.data);
        break;
      default:
        return false;
    }
    return true;
  }
};
