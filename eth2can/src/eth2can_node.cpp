/**
 * @file eth2can_node.cpp
 * @author Yudai Yamaguchi (yudai.yy0804@gmail.com)
 * @brief Ethernet to CANのノード
 * @version 0.1
 * @date 2026-03-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <arpa/inet.h>
#include <errno.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

struct CanFrame
{
  uint32_t can_id;  // SocketCAN compatible flags inside
  uint8_t len;      // 0..64
  uint8_t flags;    // device-specific (optional)
  uint8_t rsv0;
  uint8_t rsv1;
  uint8_t data[64];
} __attribute__((packed));

struct GatewayPacket
{
  uint8_t channel;      // 0~2
  uint8_t reserved[3];  // 0
  CanFrame frame;
} __attribute__((packed));

class Eth2CanNode : public rclcpp::Node
{
public:
  static constexpr size_t CAN_MAX_DLEN = 8;

  /**
   * @brief 全てのデータを受信するまでrecvを繰り返す関数
   * 
   * @param fd 
   * @param buf 
   * @param n 
   * @return true 成功
   * @return false 失敗
   */
  bool recv_all(int fd, void * buf, size_t n)
  {
    uint8_t * p = (uint8_t *)buf;
    size_t got = 0;
    while (got < n) {
      const size_t remaining = n - got;
      const ssize_t r = recv(fd, p + got, remaining, 0);
      if (r == 0) return false;  // peer closed
      if (r < 0) {
        if (errno == EINTR) continue;
        return false;
      }
      got += static_cast<size_t>(r);
    }
    return true;
  }

  /**
   * @brief 全てのデータを送信するまでsendを繰り返す関数
   * 
   * @param fd 
   * @param buf 
   * @param n 
   * @return true 成功
   * @return false 失敗
   */
  bool send_all(int fd, void * buf, size_t n)
  {
    uint8_t * p = (uint8_t *)buf;
    size_t sent = 0;
    while (sent < n) {
      const size_t remaining = n - sent;
      const ssize_t w = send(fd, p + sent, remaining, 0);
      if (w <= 0) {
        if (w < 0 && errno == EINTR) continue;
        return false;
      }
      sent += static_cast<size_t>(w);
    }
    return true;
  }

  /**
   * @brief TCP接続を確立する関数
   * 
   * @param ip IPアドレスの文字列
   * @param port ポート番号
   * @return int 
   */
  int connect_tcp(const std::string & ip, int port)
  {
    int s = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) {
      RCLCPP_ERROR(get_logger(), "socket() failed: %s", std::strerror(errno));
      return -1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
      close(s);
      RCLCPP_ERROR(get_logger(), "inet_pton() failed for IP %s", ip.c_str());
      return -1;
    }

    if (connect(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      close(s);
      RCLCPP_ERROR(get_logger(), "connect() failed: %s", std::strerror(errno));
      return -1;
    }
    return s;
  }

  Eth2CanNode() : Node("eth2can_node")
  {
    this->declare_parameter("device_ip", "192.168.1.100");
    this->declare_parameter("device_port", 5000);

    this->get_parameter("device_ip", device_ip_);
    this->get_parameter("device_port", device_port_);

    // vectorをresize、CANの数は3つ
    from_can_bus_publisher_.resize(N);
    to_can_bus_subscription_.resize(N);
    // CANの数だけPublisherとSubscriptionを作成
    for (int i = 0; i < N; i++) {
      from_can_bus_publisher_[i] =
        this->create_publisher<can_msgs::msg::Frame>("from_can_bus" + std::to_string(i), 100);
      to_can_bus_subscription_[i] = this->create_subscription<can_msgs::msg::Frame>(
        "to_can_bus" + std::to_string(i), 10, [this, i](const can_msgs::msg::Frame::SharedPtr msg) {
          this->to_can_bus_callback(msg, i);
        });
    }

    running_.store(true);
    rx_thread_ = std::thread([this]() { this->rx_loop(); });
  }

  ~Eth2CanNode() override
  {
    running_.store(false);
    close_tcp();

    if (rx_thread_.joinable()) rx_thread_.join();
  }

  void close_tcp()
  {
    std::lock_guard<std::mutex> lk(tcp_mtx_);
    if (tcp_fd_ >= 0) {
      ::shutdown(tcp_fd_, SHUT_RDWR);
      ::close(tcp_fd_);
      tcp_fd_ = -1;
    }
  }

  int get_or_connect_tcp()
  {
    std::lock_guard<std::mutex> lk(tcp_mtx_);
    if (tcp_fd_ >= 0) return tcp_fd_;

    tcp_fd_ = connect_tcp(device_ip_, device_port_);
    return tcp_fd_;
  }

  void to_can_bus_callback(const can_msgs::msg::Frame::SharedPtr msg, int channel)
  {
    // can_msgs::msg::Frame -> GatewayPacket に変換
    GatewayPacket pkt;
    pkt.channel = (uint8_t)channel;

    // can_idの設定
    uint32_t can_id = 0;
    if (msg->is_extended) {
      can_id = (msg->id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    } else {
      can_id = (msg->id & CAN_SFF_MASK);
    }
    if (msg->is_rtr) {
      can_id |= CAN_RTR_FLAG;
    }
    if (msg->is_error) {
      can_id |= CAN_ERR_FLAG;
    }

    // 変数の代入
    pkt.frame.can_id = can_id;
    pkt.frame.len = (msg->dlc <= CAN_MAX_DLEN) ? static_cast<uint8_t>(msg->dlc)
                                               : static_cast<uint8_t>(CAN_MAX_DLEN);
    pkt.frame.flags = 0;
    pkt.frame.rsv0 = 0;
    pkt.frame.rsv1 = 0;

    // データを0で初期化
    for (int i = 0; i < 64; i++) {
      pkt.frame.data[i] = 0;
    }
    // データをコピー
    for (int i = 0; i < pkt.frame.len; i++) {
      pkt.frame.data[i] = msg->data[i];
    }

    int fd = -1;
    fd = get_or_connect_tcp();
    if (fd < 0) {
      RCLCPP_ERROR(get_logger(), "TCP is not connected, drop outgoing frame");
      return;
    }

    if (!send_all(fd, &pkt, sizeof(pkt))) {
      RCLCPP_ERROR(get_logger(), "TCP send failed, closing connection");
      close_tcp();
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Sent CAN frame: ID=0x%X, DLC=%d, Data=[%02X %02X %02X %02X %02X %02X %02X %02X]", msg->id,
      msg->dlc, msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5],
      msg->data[6], msg->data[7]);
  }

  void rx_loop()
  {
    while (running_.load()) {
      int fd = -1;
      fd = get_or_connect_tcp();
      if (fd < 0) {
        ::sleep(1);
        continue;
      }

      GatewayPacket pkt{};
      if (!recv_all(fd, &pkt, sizeof(pkt))) {
        RCLCPP_ERROR(get_logger(), "TCP recv failed, closing connection");
        close_tcp();
        ::sleep(1);
        continue;
      }

      // Convert GatewayPacket -> can_msgs/Frame
      const uint32_t can_id_host = pkt.frame.can_id;
      const bool is_extended = (can_id_host & CAN_EFF_FLAG) != 0;
      const bool is_rtr = (can_id_host & CAN_RTR_FLAG) != 0;

      can_msgs::msg::Frame msg;
      msg.header.stamp = now();
      msg.id = is_extended ? (can_id_host & CAN_EFF_MASK) : (can_id_host & CAN_SFF_MASK);
      msg.is_extended = is_extended;
      msg.is_rtr = is_rtr;
      msg.is_error = (can_id_host & CAN_ERR_FLAG) != 0;
      // can_msgsはcan_fdには対応していないので、データ長は最大8バイトとして扱う
      msg.dlc =
        (pkt.frame.len <= CAN_MAX_DLEN) ? pkt.frame.len : static_cast<uint8_t>(CAN_MAX_DLEN);

      // 一度すべてのデータを0にする
      for (size_t i = 0; i < msg.data.size(); i++) {
        msg.data[i] = 0;
      }
      // データをコピー
      for (size_t i = 0; i < msg.dlc; i++) {
        msg.data[i] = pkt.frame.data[i];
      }

      // publish
      int channel = pkt.channel;
      if (channel < 0 || channel >= N) {
        RCLCPP_ERROR(get_logger(), "Received packet with invalid channel %d", channel);
        continue;
      }
      from_can_bus_publisher_[channel]->publish(msg);

      RCLCPP_INFO(
        this->get_logger(),
        "Received CAN frame: ID=0x%X, DLC=%d, Data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
        msg.id, msg.dlc, msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],
        msg.data[5], msg.data[6], msg.data[7]);
    }
  }

  static constexpr int N = 3;
  static constexpr uint32_t CAN_EFF_FLAG = 0x80000000;
  static constexpr uint32_t CAN_RTR_FLAG = 0x40000000;
  static constexpr uint32_t CAN_ERR_FLAG = 0x20000000;
  static constexpr uint32_t CAN_EFF_MASK = 0x1FFFFFFF;
  static constexpr uint32_t CAN_SFF_MASK = 0x000007FF;

  std::vector<rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr> from_can_bus_publisher_;
  std::vector<rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr> to_can_bus_subscription_;

  std::atomic<bool> running_{false};
  std::thread rx_thread_;

  std::string device_ip_;
  int device_port_;

  std::mutex tcp_mtx_;
  int tcp_fd_{-1};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Eth2CanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
