/**
 * @file can_device_common_data_driver.h
 * @author Yamaguchi Yudai
 * @brief
 * @version 0.1
 * @date 2025-08-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "sabacan/can_device.h"

class CommonDataDriver : public CanDevice
{
public:
  CommonDataDriver(std::shared_ptr<CanDriver> _can_driver) :
      CanDevice(_can_driver, 0x00, DataType::FORCE_READ)
  {
  }
  void id_rq()
  {
    tx_rq(CommonRegisterID::RQ);
  }
  void ems()
  {
    tx_rq(CommonRegisterID::EMS);
  }
  void reset_ems()
  {
    tx_rq(CommonRegisterID::RESET_EMS);
  }
};