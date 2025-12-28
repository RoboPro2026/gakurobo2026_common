/**
 * @file can_device_led_driver.h
 * @author Yudai Yamaguchi
 * @brief LED基板をCANで動かすプログラム
 * @version 0.1
 * @date 2025-12-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "sabacan/can_device.h"

class LEDDriver : public CanDevice
{
public:
  // GPIOの数
  static constexpr int N = 6;
  // LEDの数
  static constexpr int M = 3;
  uint8_t led_mode;
  bool enable_auto_transition;
  uint16_t emg_blink_period;
  uint32_t emg_color[M];
  uint64_t led_ref[M];
  uint16_t monitor_period;
  uint64_t monitor_reg1;
  uint64_t monitor_reg2;

  LEDDriver(std::shared_ptr<CanDriver> _can_driver, int _board_id)
  : CanDevice(_can_driver, _board_id, DataType::LED)
  {
  }

  void setLedMode(uint8_t val) { tx(LED::LED_MODE, val); }

  void setEnableAutoTransition(bool val) { tx(LED::ENABLE_AUTO_TRANSITION, val); }

  void setEmgBlinkPeriod(uint16_t val) { tx(LED::EMG_BLINK_PERIOD, val); }

  void setEmgColor(int m, uint8_t r, uint8_t g, uint8_t b)
  {
    uint32_t val = (uint32_t)r | ((uint32_t)g << 8) | ((uint32_t)b << 16);
    tx(LED::EMG_COLOR + m, val);
  }

  void setLedRef(int m, uint8_t start, uint8_t length, uint8_t r, uint8_t g, uint8_t b)
  {
    uint64_t val = (uint64_t)start | ((uint64_t)length << 8) | ((uint64_t)r << 16) |
                   ((uint64_t)g << 24) | ((uint64_t)b << 32);
    tx(LED::LED_REF + m, val);
  }

  void setMonitorPeriod(uint16_t val) { tx(LED::MONITOR_PERIOD, val); }

  void setMonitorReg1(uint64_t val) { tx(LED::MONITOR_REG1, val); }

  void setMonitorReg2(uint64_t val) { tx(LED::MONITOR_REG2, val); }

  bool receive(CanFrame frame)
  {
    // 違う基板へのメッセージは無視する
    if (frame.data_type != this->data_type) return false;
    if (frame.board_id != this->board_id) return false;

    switch (frame.register_id) {
      case LED::LED_MODE:
        assign(&led_mode, frame.data);
        break;
      case LED::ENABLE_AUTO_TRANSITION:
        assign(&enable_auto_transition, frame.data);
        break;
      case LED::EMG_BLINK_PERIOD:
        assign(&emg_blink_period, frame.data);
        break;
      case LED::MONITOR_PERIOD:
        assign(&monitor_period, frame.data);
        break;
      case LED::MONITOR_REG1:
        assign(&monitor_reg1, frame.data);
        break;
      case LED::MONITOR_REG2:
        assign(&monitor_reg2, frame.data);
        break;
      default:
        if (LED::EMG_COLOR <= frame.register_id && frame.register_id < LED::EMG_COLOR + M) {
          assign(&emg_color[frame.register_id - LED::EMG_COLOR], frame.data);
        } else if (LED::LED_REF <= frame.register_id && frame.register_id < LED::LED_REF + M) {
          assign(&led_ref[frame.register_id - LED::LED_REF], frame.data);
        } else {
          return false;
        }
    }
    return true;
  }
};