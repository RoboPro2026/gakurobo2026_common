/**
 * @file can_device.h
 * @author Yamaguchi Yudai
 * @brief CANの基底クラス
 * @version 0.1
 * @date 2025-08-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>

#include "sabacan/can_define.h"

class CanDriver
{
public:
  using TxCallback = std::function<void(uint32_t, uint8_t *, uint8_t, bool, bool)>;

  void tx(uint32_t id, uint8_t * data, uint8_t dlc, bool is_remote_frame, bool is_ext_id = true)
  {
    if (tx_callback_) {
      tx_callback_(id, data, dlc, is_remote_frame, is_ext_id);
    }
  }

  void tx_frame(CanFrame frame)
  {
    uint32_t id =
      (((uint32_t)frame.priority & 0x0F) << 24) | (((uint32_t)frame.data_type & 0x0F) << 20) |
      (((uint32_t)frame.board_id & 0x0F) << 16) | (((uint32_t)frame.register_id & 0xFFFF));

    tx(id, frame.data, frame.len, frame.is_request, true);
  }

  CanFrame rx_frame(uint32_t id, uint8_t * data, uint8_t dlc, bool is_remote_frame, bool is_ext_id)
  {
    (void)dlc;        // 未使用パラメータ警告を抑制
    (void)is_ext_id;  // 未使用パラメータ警告を抑制
    CanFrame rx_frame;
    rx_frame.is_request = is_remote_frame;
    rx_frame.priority = (id & 0xf000000) >> 24;
    rx_frame.data_type = (id & 0xf00000) >> 20;
    rx_frame.board_id = (id & 0xf0000) >> 16;
    rx_frame.register_id = id & 0xffff;
    rx_frame.len = dlc;
    memcpy(rx_frame.data, data, 8);
    return rx_frame;
  }

  void register_tx_callback(TxCallback callback) { tx_callback_ = callback; }

private:
  TxCallback tx_callback_;  // 登録されたコールバック関数を保持するメンバー変数
};

class CanDevice
{
protected:
  std::shared_ptr<CanDriver> can_driver;
  uint8_t board_id;
  uint8_t data_type;
  CanDevice(std::shared_ptr<CanDriver> _can_driver, uint8_t _board_id, uint8_t _data_type)
  : can_driver(_can_driver), board_id(_board_id), data_type(_data_type)
  {
  }

  void tx_rq(uint16_t register_id)
  {
    CanFrame frame{
      .is_request = true,
      .priority = 0,
      .data_type = data_type,
      .board_id = board_id,
      .register_id = register_id,
      .data = {0},
      .len = 0};
    can_driver->tx_frame(frame);
  }
  template <typename T>
  void tx(uint16_t register_id, T data)
  {
    CanFrame frame{
      .is_request = false,
      .priority = 0,
      .data_type = data_type,
      .board_id = board_id,
      .register_id = register_id,
      .data = {0},
      .len = sizeof(T)};
    memcpy(frame.data, &data, sizeof(T));
    can_driver->tx_frame(frame);
  }
  template <typename T>
  void assign(T * data, uint8_t * receive_data)
  {
    memcpy(data, receive_data, sizeof(T));
  }

public:
  void setBoardId(int _board_id) { board_id = _board_id; }
};