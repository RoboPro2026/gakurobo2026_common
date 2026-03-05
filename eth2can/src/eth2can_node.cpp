/**
 * @file eth2can_node.cpp
 * @author Yudai Yamaguchi (yudai.yy0804@gmail.com)
 * @brief Ethernet to CANのノード
 * @version 0.1
 * @date 2026-03-04
 * * @copyright Copyright (c) 2026
 * */

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <netinet/tcp.h>
#include <poll.h>
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
  uint8_t len;      // 0..8
  uint8_t reserved[3];
  uint8_t data[8];
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
  static constexpr size_t GATEWAY_PACKET_SIZE = sizeof(GatewayPacket);

  static_assert(sizeof(CanFrame) == 16, "CanFrame must be 16 bytes");
  static_assert(sizeof(GatewayPacket) == 20, "GatewayPacket must be 20 bytes");

  /**
   * @brief 全てのデータをsendで送る関数
   * * @param fd 
   * @param buf 
   * @param n 
   * @return true 成功
   * @return false 失敗
   */
  bool send_all(int fd, void * buf, size_t n)
  {
    uint8_t * p = (uint8_t *)buf;

    // パケットは一括で送る。
    const ssize_t w = send(fd, p, n, 0);
    if (w < 0) {
      if (errno == EINTR) {
        if (!running_.load() || !rclcpp::ok()) return false;
        return true;  // consider interrupted send as success, since we don't want to retry
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) return false;
      return false;
    }
    return true;
  }

  /**
   * @brief TCP接続を確立する関数
   * * @param ip IPアドレスの文字列
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

    // Make connect cancellable-ish by using non-blocking connect + poll timeout.
    const int old_flags = fcntl(s, F_GETFL, 0);
    if (old_flags >= 0) {
      (void)fcntl(s, F_SETFL, old_flags | O_NONBLOCK);
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
      close(s);
      RCLCPP_ERROR(get_logger(), "inet_pton() failed for IP %s", ip.c_str());
      return -1;
    }

    const int rc = connect(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    if (rc < 0) {
      if (errno != EINPROGRESS) {
        close(s);
        RCLCPP_ERROR(get_logger(), "connect() failed: %s", std::strerror(errno));
        return -1;
      }

      pollfd pfd{};
      pfd.fd = s;
      pfd.events = POLLOUT;
      const int prc = poll(&pfd, 1, 1000);
      if (prc <= 0) {
        close(s);
        if (prc == 0) {
          RCLCPP_ERROR(get_logger(), "connect() timeout to %s:%d", ip.c_str(), port);
        } else {
          RCLCPP_ERROR(get_logger(), "poll() failed during connect: %s", std::strerror(errno));
        }
        return -1;
      }

      int so_error = 0;
      socklen_t so_error_len = sizeof(so_error);
      if (getsockopt(s, SOL_SOCKET, SO_ERROR, &so_error, &so_error_len) != 0) {
        close(s);
        RCLCPP_ERROR(get_logger(), "getsockopt(SO_ERROR) failed: %s", std::strerror(errno));
        return -1;
      }
      if (so_error != 0) {
        close(s);
        RCLCPP_ERROR(get_logger(), "connect() failed: %s", std::strerror(so_error));
        return -1;
      }
    }

    // Restore blocking mode (best effort).
    if (old_flags >= 0) {
      (void)fcntl(s, F_SETFL, old_flags);
    }

    // Set recv/send timeouts so shutdown doesn't hang on blocking syscalls.
    timeval tv{};
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    (void)setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    (void)setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (tcp_nodelay_) {
      int one = 1;
      if (setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) != 0) {
        RCLCPP_WARN(get_logger(), "setsockopt(TCP_NODELAY) failed: %s", std::strerror(errno));
      }
    }

    return s;
  }

  Eth2CanNode() : Node("eth2can_node")
  {
    this->declare_parameter("device_ip", "192.168.1.100");
    this->declare_parameter("device_port", 5000);
    this->declare_parameter("tcp_nodelay", true);

    this->get_parameter("device_ip", device_ip_);
    this->get_parameter("device_port", device_port_);
    this->get_parameter("tcp_nodelay", tcp_nodelay_);

    // vectorをresize、CANの数は3つ
    from_can_bus_publisher_.resize(N);
    to_can_bus_subscription_.resize(N);
    // CANの数だけPublisherとSubscriptionを作成
    for (int i = 0; i < N; i++) {
      from_can_bus_publisher_[i] =
        this->create_publisher<can_msgs::msg::Frame>("from_can_bus" + std::to_string(i), 500);
      to_can_bus_subscription_[i] = this->create_subscription<can_msgs::msg::Frame>(
        "to_can_bus" + std::to_string(i), 500,
        [this, i](const can_msgs::msg::Frame::SharedPtr msg) {
          this->to_can_bus_callback(msg, i);
        });
    }

    running_.store(true);
    rx_thread_ = std::thread([this]() { this->rx_loop(); });

    RCLCPP_INFO(
      get_logger(), "eth2can_node started with device IP %s and port %d", device_ip_.c_str(),
      device_port_);
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

    // データを0で初期化
    for (int i = 0; i < 8; i++) {
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
  }

  void rx_loop()
  {
    std::vector<uint8_t> rx_buf;
    rx_buf.reserve(8192);  // バッファの予約サイズを拡大

    int fd = -1;  // ループの外でfdを管理し、ロック競合を回避

    while (running_.load() && rclcpp::ok()) {
      if (fd < 0) {
        fd = get_or_connect_tcp();
        if (fd < 0) {
          std::this_thread::sleep_for(1s);
          continue;
        }
      }

      pollfd pfd{};
      pfd.fd = fd;
      pfd.events = POLLIN;

      const int prc = poll(&pfd, 1, 100);
      if (prc < 0) {
        if (errno == EINTR) continue;
        RCLCPP_WARN(
          get_logger(), "poll() failed in rx_loop, closing connection: %s", std::strerror(errno));
        close_tcp();
        fd = -1;  // エラー時はfdをリセット
        continue;
      }
      if (prc == 0) {
        continue;  // timeout
      }

      if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        RCLCPP_WARN(get_logger(), "TCP socket error/hup in rx_loop, closing connection");
        close_tcp();
        fd = -1;  // エラー時はfdをリセット
        continue;
      }
      if ((pfd.revents & POLLIN) == 0) {
        continue;
      }

      uint8_t tmp[4096];  // 一度に読み込むサイズを拡大してシステムコールを減らす
      const ssize_t r = recv(fd, tmp, sizeof(tmp), 0);
      if (r == 0) {
        RCLCPP_WARN(get_logger(), "TCP peer closed, closing connection");
        close_tcp();
        fd = -1;  // エラー時はfdをリセット
        continue;
      }
      if (r < 0) {
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
        RCLCPP_WARN(get_logger(), "TCP recv error, closing connection: %s", std::strerror(errno));
        close_tcp();
        fd = -1;  // エラー時はfdをリセット
        continue;
      }

      rx_buf.insert(rx_buf.end(), tmp, tmp + r);

      // TCP can split/coalesce packets; parse from a buffer like the Python version.
      size_t offset = 0;
      while (rx_buf.size() - offset >= GATEWAY_PACKET_SIZE) {
        GatewayPacket pkt{};
        std::memcpy(&pkt, rx_buf.data() + offset, GATEWAY_PACKET_SIZE);
        offset += GATEWAY_PACKET_SIZE;

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
        const int channel = pkt.channel;
        if (channel < 0 || channel >= N) {
          RCLCPP_ERROR(get_logger(), "Received packet with invalid channel %d", channel);
          continue;
        }
        from_can_bus_publisher_[channel]->publish(msg);
      }

      if (offset > 0) {
        rx_buf.erase(rx_buf.begin(), rx_buf.begin() + static_cast<std::ptrdiff_t>(offset));
      }
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
  bool tcp_nodelay_{true};

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