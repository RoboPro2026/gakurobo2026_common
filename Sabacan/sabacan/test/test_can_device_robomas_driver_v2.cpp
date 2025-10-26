/**
 * @file test_can_device_robomas_driver_v2.cpp
 * @author Yamaguchi Yudai
 * @brief can_device_robomas_driver_v2のテスト
 * @version 0.1
 * @date 2025-10-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <cstring>  // memcpy
#include <future>   // std::promise, std::future
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "can_msgs/msg/frame.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/node_options.hpp"
#include "rcutils/logging.h"     // グローバルログレベル設定用
#include "sabacan/can_define.h"  // CAN ID定義
#include "sabacan/can_device_robomas_driver_v2.h"

using namespace std::chrono_literals;
std::shared_ptr<CanDriver> can_driver;
std::unique_ptr<RobomasDriverV2> robomas_driver;
uint8_t board_id;
CanFrame tx_frame;
int N = 4;

template <typename T>
CanFrame can_frame(uint16_t register_id, T data)
{
  CanFrame frame{
    .is_request = false,
    .priority = 0,
    .data_type = DataType::ROBOMAS_V2,
    .board_id = board_id,
    .register_id = register_id,
    .data = {0},
    .len = sizeof(T)};
  memcpy(frame.data, &data, sizeof(T));
  return frame;
}

template <typename T>
CanFrame can_frame_rq(uint16_t register_id, T data)
{
  CanFrame frame{
    .is_request = true,
    .priority = 0,
    .data_type = DataType::ROBOMAS_V2,
    .board_id = board_id,
    .register_id = register_id,
    .data = {0},
    .len = sizeof(T)};
  memcpy(frame.data, &data, sizeof(T));
  return frame;
}

class TestCanDeviceRobomasDriverV2 : public testing::TestWithParam<int>
{
protected:
  virtual void SetUp() override
  {
    // GTestのテストパラメータを取得
    board_id = GetParam();
    can_driver = std::make_shared<CanDriver>();
    robomas_driver = std::make_unique<RobomasDriverV2>(can_driver, board_id);
    // 送信コールバックには最後に送信したデータを変数に代入する
    can_driver->register_tx_callback(
      [&](uint32_t id, uint8_t * data, uint8_t dlc, bool is_remote, bool is_ext) {
        tx_frame.is_request = is_remote;
        tx_frame.priority = (id >> 24) & 0x0f;
        tx_frame.data_type = (id >> 20) & 0x0f;
        tx_frame.board_id = (id >> 16) & 0x0f;
        tx_frame.register_id = id & 0xffff;
        tx_frame.len = dlc;
        memcpy(tx_frame.data, data, 8);
      });
  }

  virtual void TearDown() override {}
};

uint8_t create_set_control(
  int n, uint8_t _control_mode, uint8_t _control_motor, bool _dob_en, bool _abs_enc_en,
  bool _md_guess_en)
{
  uint8_t _control = 0;
  _control |= _control_mode & 0x03;
  _control |= (_control_motor & 0x01) << 2;
  _control |= (uint8_t)_dob_en << 4;
  _control |= (uint8_t)_abs_enc_en << 5;
  _control |= (uint8_t)_md_guess_en << 6;
  return _control;
}

TEST_P(TestCanDeviceRobomasDriverV2, setControl)
{
  for (uint8_t n = 0; n < N; n++) {
    uint8_t tx_data = create_set_control(n, 1, 0, 1, 0, 1);
    robomas_driver->setControl(n, 1, 0, 1, 0, 1);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::CONTROL, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);

    tx_data = create_set_control(n, 3, 1, 0, 1, 0);
    robomas_driver->setControl(n, 3, 1, 0, 1, 0);
    expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::CONTROL, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

// 値の範囲はプロトコルで決まっていないので、コーナーケースはチェックしてない

TEST_P(TestCanDeviceRobomasDriverV2, setAbsGearRatio)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 123.456f * (n + 1);
    robomas_driver->setAbsGearRatio(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::ABS_GEAR_RATIO, tx_data);
    // ASSERT_NEAR(*(float *)expect_frame.data, *(float *)tx_frame.data, 1e-5);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, setCalRq)
{
  for (uint8_t n = 0; n < N; n++) {
    bool tx_data = false;
    robomas_driver->setCalRq(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::CAL_RQ, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
    tx_data = true;
    robomas_driver->setCalRq(n, tx_data);
    expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::CAL_RQ, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

// --- DOB (外乱オブザーバ) 関連 ---

TEST_P(TestCanDeviceRobomasDriverV2, setLoad_J)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 1.234f + n;  // モーターごとに異なる値
    robomas_driver->setLoad_J(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::LOAD_J, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, setLoad_D)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 0.567f + n;
    robomas_driver->setLoad_D(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::LOAD_D, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, setDob_CF)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 10.5f + n;
    robomas_driver->setDob_CF(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::DOB_CF, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, setCanTimeout)
{
  uint16_t tx_data = 1000;  // 1000ms
  robomas_driver->setCanTimeout(tx_data);
  // ボード全体の設定 (モーター番号なし)
  auto expect_frame = can_frame(RobomasV2::CAN_TIMEOUT, tx_data);
  ASSERT_EQ(expect_frame, tx_frame);
}

// --- トルク制御 (TRQ) 関連 ---

TEST_P(TestCanDeviceRobomasDriverV2, setTorqueTarget)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 0.5f + n * 0.1f;
    robomas_driver->setTorqueTarget(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::TRQ_TARGET, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

// --- 速度制御 (SPD) 関連 ---

TEST_P(TestCanDeviceRobomasDriverV2, setSpeedTarget)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 3.1415f * (n + 1);
    robomas_driver->setSpeedTarget(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::SPD_TARGET, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, setTorqueLimit)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 5.0f + n;
    robomas_driver->setTorqueLimit(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::TRQ_LIM, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, setSpeedGainP)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 0.5f + n * 0.1f;
    robomas_driver->setSpeedGainP(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::SPD_GAIN_P, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, setSpeedGainI)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 0.2f + n * 0.1f;
    robomas_driver->setSpeedGainI(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::SPD_GAIN_I, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, setSpeedGainD)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 0.01f * n;
    robomas_driver->setSpeedGainD(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::SPD_GAIN_D, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

// --- 位置制御 (POS) 関連 ---

TEST_P(TestCanDeviceRobomasDriverV2, setPosTarget)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 10.0f * (n + 1);
    robomas_driver->setPosTarget(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::POS_TARGET, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, setSpeedLim)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 30.0f + n;
    robomas_driver->setSpeedLim(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::SPD_LIM, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, setPosGainP)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 6.0f + n;
    robomas_driver->setPosGainP(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::POS_GAIN_P, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, setPosGainI)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 3.0f + n;
    robomas_driver->setPosGainI(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::POS_GAIN_I, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, setPosGainD)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 0.5f * n;
    robomas_driver->setPosGainD(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::POS_GAIN_D, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

// --- ABS (絶対位置) 関連 ---

TEST_P(TestCanDeviceRobomasDriverV2, setAbsTurnCnt)
{
  for (uint8_t n = 0; n < N; n++) {
    int32_t tx_data = -50 + n;
    robomas_driver->setAbsTurnCnt(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::ABS_TURN_CNT, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

// --- VESC 関連 ---

TEST_P(TestCanDeviceRobomasDriverV2, setVescMode)
{
  for (uint8_t n = 0; n < N; n++) {
    uint8_t tx_data = RobomasV2::VESC_MODE_SPEED;
    robomas_driver->setVescMode(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::VESC_MODE, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);

    tx_data = RobomasV2::VESC_MODE_CURRENT;
    robomas_driver->setVescMode(n, tx_data);
    expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::VESC_MODE, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, setVescTarget)
{
  for (uint8_t n = 0; n < N; n++) {
    float tx_data = 1000.0f * (n + 1);
    robomas_driver->setVescTarget(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::VESC_TARGET, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

// --- モニタリング 関連 ---

TEST_P(TestCanDeviceRobomasDriverV2, setMonitorPeriod)
{
  uint16_t tx_data = 50;  // 50ms
  robomas_driver->setMonitorPeriod(tx_data);
  // ボード全体の設定 (モーター番号なし)
  auto expect_frame = can_frame(RobomasV2::MONITOR_PERIOD, tx_data);
  ASSERT_EQ(expect_frame, tx_frame);
}

TEST_P(TestCanDeviceRobomasDriverV2, setMonitorReg1)
{
  for (uint8_t n = 0; n < N; n++) {
    uint64_t tx_data = 0x0123456789ABCDEFULL + n;
    robomas_driver->setMonitorReg1(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::MONITOR_REG1, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, setMonitorReg2)
{
  for (uint8_t n = 0; n < N; n++) {
    uint64_t tx_data = 0xFEDCBA9876543210ULL + n;
    robomas_driver->setMonitorReg2(n, tx_data);
    auto expect_frame = can_frame((n & 0xF) << 8 | RobomasV2::MONITOR_REG2, tx_data);
    ASSERT_EQ(expect_frame, tx_frame);
  }
}

// --- receive 関数（CANフレーム受信）のテスト ---

TEST_P(TestCanDeviceRobomasDriverV2, receiveMotorState)
{
  for (uint8_t n = 0; n < N; n++) {
    bool expect_data = n & 0x01;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::MOTOR_STATE, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    // bool 型なので ASSERT_EQ で OK
    ASSERT_EQ(expect_data, robomas_driver->motor_state[n]);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveControl)
{
  for (uint8_t n = 0; n < N; n++) {
    // Test 1: Mode 1 (SPD), Motor 0 (C610), DOB_EN=1, ABS_ENC=0, MD_GUESS=1
    uint8_t expect_data = create_set_control(n, 1, 0, 1, 0, 1);
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::CONTROL, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    // uint8_t と bool なので ASSERT_EQ で OK
    ASSERT_EQ(expect_data, robomas_driver->control[n]);
    ASSERT_EQ(1, robomas_driver->control_mode[n]);
    ASSERT_EQ(0, robomas_driver->control_motor[n]);
    ASSERT_EQ(true, robomas_driver->dob_en[n]);
    ASSERT_EQ(false, robomas_driver->abs_enc_en[n]);
    ASSERT_EQ(true, robomas_driver->md_guess_en[n]);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveAbsGearRatio)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 123.456f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::ABS_GEAR_RATIO, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    // float 型なので ASSERT_NEAR
    ASSERT_NEAR(expect_data, robomas_driver->abs_gear_ratio[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveCalRq)
{
  for (uint8_t n = 0; n < N; n++) {
    bool expect_data = n & 0x01;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::CAL_RQ, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    // bool 型なので ASSERT_EQ
    ASSERT_EQ(expect_data, robomas_driver->cal_rq[n]);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveLoadJ)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 1.234f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::LOAD_J, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    // float 型なので ASSERT_NEAR
    ASSERT_NEAR(expect_data, robomas_driver->load_j[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveLoadD)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 0.567f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::LOAD_D, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->load_d[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveDobCf)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 10.5f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::DOB_CF, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->dob_cutoff_freq[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveCanTimeout)
{
  uint16_t expect_data = 1000;
  auto rx_frame = can_frame(RobomasV2::CAN_TIMEOUT, expect_data);
  bool is_success = robomas_driver->receive(rx_frame);
  ASSERT_TRUE(is_success);
  // uint16_t 型なので ASSERT_EQ
  ASSERT_EQ(expect_data, robomas_driver->can_timeout);
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveTrq)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 0.123f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::TRQ, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->current_torque[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveTrqTarget)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 0.456f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::TRQ_TARGET, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->trq_target[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveSpd)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 3.14f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::SPD, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->current_speed[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveSpdTarget)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 5.5f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::SPD_TARGET, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->speed_target[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveTrqLim)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 8.0f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::TRQ_LIM, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->torque_lim[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveSpdGainP)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 0.51f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::SPD_GAIN_P, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->speed_gain_p[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveSpdGainI)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 0.21f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::SPD_GAIN_I, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->speed_gain_i[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveSpdGainD)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 0.01f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::SPD_GAIN_D, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->speed_gain_d[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receivePos)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 100.1f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::POS, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->current_pos[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receivePosTarget)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 100.2f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::POS_TARGET, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->pos_target[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveSpdLim)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 30.1f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::SPD_LIM, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->speed_lim[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receivePosGainP)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 6.1f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::POS_GAIN_P, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->pos_gain_p[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receivePosGainI)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 3.1f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::POS_GAIN_I, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->pos_gain_i[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receivePosGainD)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 0.51f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::POS_GAIN_D, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->pos_gain_d[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveAbsPos)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 1.2345f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::ABS_POS, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->abs_pos[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveAbsSpd)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 6.789f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::ABS_SPD, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->abs_speed[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveAbsTurnCnt)
{
  for (uint8_t n = 0; n < N; n++) {
    int32_t expect_data = -100 + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::ABS_TURN_CNT, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    // int32_t 型なので ASSERT_EQ
    ASSERT_EQ(expect_data, robomas_driver->abs_turn_cnt[n]);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveVescMode)
{
  for (uint8_t n = 0; n < N; n++) {
    uint8_t expect_data = RobomasV2::VESC_MODE_POSITION;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::VESC_MODE, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    // uint8_t 型なので ASSERT_EQ
    ASSERT_EQ(expect_data, robomas_driver->vesc_mode[n]);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveVescTarget)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 999.9f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::VESC_TARGET, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->vesc_target[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveVescVoltage)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 48.1f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::VESC_VOLTAGE, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->vesc_voltage[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveVescCurrent)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 10.5f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::VESC_CURRENT, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->vesc_current[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveVescErpm)
{
  for (uint8_t n = 0; n < N; n++) {
    float expect_data = 10000.0f + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::VESC_ERPM, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    ASSERT_NEAR(expect_data, robomas_driver->vesc_erpm[n], 1e-5);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveMonitorPeriod)
{
  uint16_t expect_data = 50;
  auto rx_frame = can_frame(RobomasV2::MONITOR_PERIOD, expect_data);
  bool is_success = robomas_driver->receive(rx_frame);
  ASSERT_TRUE(is_success);
  // uint16_t 型なので ASSERT_EQ
  ASSERT_EQ(expect_data, robomas_driver->monitor_period);
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveMonitorReg1)
{
  for (uint8_t n = 0; n < N; n++) {
    uint64_t expect_data = 0xAAAAAAAAAAAAAAAAULL + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::MONITOR_REG1, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    // uint64_t 型なので ASSERT_EQ
    ASSERT_EQ(expect_data, robomas_driver->monitor_reg1[n]);
  }
}

TEST_P(TestCanDeviceRobomasDriverV2, receiveMonitorReg2)
{
  for (uint8_t n = 0; n < N; n++) {
    uint64_t expect_data = 0xBBBBBBBBBBBBBBBBULL + n;
    auto rx_frame = can_frame((n & 0xF) << 8 | RobomasV2::MONITOR_REG2, expect_data);
    bool is_success = robomas_driver->receive(rx_frame);
    ASSERT_TRUE(is_success);
    // uint64_t 型なので ASSERT_EQ
    ASSERT_EQ(expect_data, robomas_driver->monitor_reg2[n]);
  }
}

// 不正なレジスタID
TEST_P(TestCanDeviceRobomasDriverV2, receiveInvalidRegisterId)
{
  // 存在しないレジスタID
  uint16_t invalid_reg_id = 0xFFFF;
  float expect_data = 1.0f;
  auto rx_frame = can_frame(invalid_reg_id, expect_data);

  // receive 関数は false を返すはず
  bool is_success = robomas_driver->receive(rx_frame);
  ASSERT_FALSE(is_success);

  // 不正なモーター番号 (n >= N)
  invalid_reg_id = (N << 8) | RobomasV2::SPD;  // n=4
  rx_frame = can_frame(invalid_reg_id, expect_data);
  is_success = robomas_driver->receive(rx_frame);
  ASSERT_FALSE(is_success);
}

// ... (INSTANTIATE_TEST_SUITE_P はファイル末尾に1つだけ記述) ...

INSTANTIATE_TEST_SUITE_P(BOARD_ID, TestCanDeviceRobomasDriverV2, ::testing::Range(0, 10));