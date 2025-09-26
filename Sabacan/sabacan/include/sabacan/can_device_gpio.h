/**
 * @file can_device_gpio_driver.h
 * @author Yamaguchi Yudai
 * @brief GPIO基板をCANで動かすプログラム
 * @version 0.1
 * @date 2025-08-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "sabacan/can_device.h"

class GPIODriver : public CanDevice
{
public:
  static constexpr int N = 9;
  uint16_t port_mode;
  uint16_t port_read;
  uint16_t port_write;
  uint16_t port_int_en;
  uint16_t esc_mode_en;
  uint16_t pwm_period[N];
  uint16_t pwm_duty[N];
  uint16_t monitor_period;
  uint64_t monitor_reg;
  GPIODriver(std::shared_ptr<CanDriver> _can_driver, int _board_id) :
      CanDevice(_can_driver, _board_id, DataType::GPIO)
  {
  }
  void setPortMode(uint16_t val)
  {
    tx(GPIO::PORT_MODE, val);
  }

  void setPortWrite(uint16_t val)
  {
    tx(GPIO::PORT_WRITE, val);
  }

  void setPortIntEn(uint16_t val)
  {
    tx(GPIO::PORT_INT_EN, val);
  }

  void setEscModeEn(uint16_t val)
  {
    tx(GPIO::ESC_MODE_EN, val);
  }

  void setPwmPeriod(int n, uint16_t val)
  {
    tx(GPIO::PWM_PERIOD + n, val);
  }

  void setPwmDuty(int n, uint16_t val)
  {
    tx(GPIO::PWM_DUTY + n, val);
  }

  void setMonitorPeriod(uint16_t val)
  {
    tx(GPIO::MONITOR_PERIOD, val);
  }

  void setMonitorReg(uint64_t val)
  {
    tx(GPIO::MONITOR_REG, val);
  }

  bool receive(CanFrame frame)
  {
    switch (frame.register_id)
    {
    case GPIO::PORT_MODE:
      assign(&port_mode, frame.data);
      break;
    case GPIO::PORT_READ:
      assign(&port_read, frame.data);
      break;
    case GPIO::PORT_WRITE:
      assign(&port_write, frame.data);
      break;
    case GPIO::PORT_INT_EN:
      assign(&port_int_en, frame.data);
      break;
    case GPIO::ESC_MODE_EN:
      assign(&esc_mode_en, frame.data);
      break;
    case GPIO::MONITOR_PERIOD:
      assign(&monitor_period, frame.data);
      break;
    case GPIO::MONITOR_REG:
      assign(&monitor_reg, frame.data);
      break;
    default:
      if (frame.register_id <= GPIO::PWM_PERIOD && frame.register_id < GPIO::PWM_PERIOD + N)
      {
        assign(&pwm_period[(frame.register_id - GPIO::PWM_PERIOD) % N], frame.data);
      }
      else if (frame.register_id <= GPIO::PWM_DUTY && frame.register_id < GPIO::PWM_DUTY + N)
      {
        assign(&pwm_duty[(frame.register_id - GPIO::PWM_DUTY) % N], frame.data);
      }
      else
      {
        return false;
      }
    }
    return true;
  }
};