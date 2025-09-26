#include "sabacan/sabacan.h"
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "sabacan_msgs/msg/sabacan_gpio_ref.hpp"
#include "sabacan_msgs/srv/sabacan_reset.hpp"
#include "sabacan_msgs/msg/sabacan_gpio_res.hpp"
#include <memory>
#include <functional>
#include <string>
#include <algorithm>
#include <cstring>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class SabaneCanNode : public rclcpp::Node
{
public:
    SabaneCanNode() : Node("sabacan_gpio_node")
    {
        // パラメータの宣言
        this->declare_parameter("pin_type", std::vector<std::string>{"PWM_OUT", "PWM_OUT", "PWM_OUT", "PWM_OUT", "PWM_OUT", "PWM_OUT", "PWM_OUT", "PWM_IN", "ESC"});
        this->declare_parameter("board_id", uint8_t(0));
        // GPIOモニタ周期のパラメータを追加
        this->declare_parameter("monitor_period_ms", 50); // デフォルト50ms
        // モニタ周期設定の有効/無効を制御するパラメータ
        this->declare_parameter("enable_monitor_period", true); // デフォルトtrue
        // 安定性向上のためのパラメータ
        this->declare_parameter("gpio_monitor_timer_ms", 20); // GPIO監視タイマー周期（デフォルト20ms）
        this->declare_parameter("max_init_retries", 3); // 初期化最大リトライ回数
        this->declare_parameter("init_retry_delay_ms", 100); // 初期化リトライ間隔（デフォルト100ms）
        
        auto pin_type_param = this->get_parameter("pin_type").as_string_array();
        for (size_t i = 0; i < 9 && i < pin_type_param.size(); i++)
        {
            pin_type[i] = pin_type_param[i];
        }

        uint8_t board_id = this->get_parameter("board_id").as_int();
        int monitor_period_ms = this->get_parameter("monitor_period_ms").as_int();
        bool enable_monitor_period = this->get_parameter("enable_monitor_period").as_bool();
        int gpio_monitor_timer_ms = this->get_parameter("gpio_monitor_timer_ms").as_int();
        max_init_retries_ = this->get_parameter("max_init_retries").as_int();
        init_retry_delay_ms_ = this->get_parameter("init_retry_delay_ms").as_int();

        // CANデータ送信用のPublisher
        can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 100);

        // CANデータ受信用のSubscriber
        can_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
            "/from_can_bus", 100, std::bind(&SabaneCanNode::can_callback, this, std::placeholders::_1));

        // モータの指令値を受け取るSubscriber
        sabacan_ref_subscription_ = this->create_subscription<sabacan_msgs::msg::SabacanGPIORef>(
            "/sabacan_gpio_ref" + std::to_string(board_id), 100, std::bind(&SabaneCanNode::sabacan_ref_callback, this, std::placeholders::_1));

        // PWM入力のフィードバックをpublishするPublisher（ボード毎）
        sabacan_res_publisher_ = this->create_publisher<sabacan_msgs::msg::SabacanGPIORes>(
            "/sabacan_gpio_res" + std::to_string(board_id), 100);

        can_driver_ = std::make_shared<CanDriver>();

        common_data_driver_ = std::make_unique<CommonDataDriver>(can_driver_);
        gpio_driver_ = std::make_unique<GPIODriver>(can_driver_, board_id);

        can_driver_->register_tx_callback(
            [this](uint32_t id, uint8_t *data, uint8_t dlc, bool is_remote, bool is_ext)
            {
                this->tx(id, data, dlc, is_remote, is_ext);
            });

        // リセットサービスサーバーの作成
        reset_service_ = this->create_service<sabacan_msgs::srv::SabacanReset>(
            "sabacan_gpio_reset", 
            std::bind(&SabaneCanNode::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

        // 安定性向上: GPIO監視用タイマーの追加
        // gpio_monitor_timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(gpio_monitor_timer_ms),
        //     std::bind(&SabaneCanNode::gpio_monitor_callback, this));

        RCLCPP_INFO(this->get_logger(), "Calling gpio_init()...");
        try {
            gpio_init();
            RCLCPP_INFO(this->get_logger(), "gpio_init() completed successfully");
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO: %s", e.what());
            throw;
        }

        RCLCPP_INFO(this->get_logger(), "GPIO node initialized with monitor period: %d ms, enable_monitor_period: %s, gpio_monitor_timer: %d ms", 
                    monitor_period_ms, enable_monitor_period ? "true" : "false", gpio_monitor_timer_ms);
    }

    void tx(uint32_t id, uint8_t *data, uint8_t dlc, bool is_remote_frame, bool is_ext_id = true)
    {
        auto msg = std::make_unique<can_msgs::msg::Frame>();
        msg->header.stamp = this->get_clock()->now();
        msg->id = id;
        msg->is_extended = is_ext_id;
        msg->is_rtr = is_remote_frame;
        msg->dlc = dlc;
        for (int i = 0; i < 8; i++)
        {
            msg->data[i] = data[i];
        }
        // RCLCPP_INFO(this->get_logger(), "Sending CAN frame: ID=0x%X", msg->id);
        can_publisher_->publish(std::move(msg));
    }

private:
    // PWM 入力取り扱い用: ピン数定数
    static constexpr int N = 9;
    
    // 安定性向上: 簡素化されたCAN受信処理
    void can_callback(const can_msgs::msg::Frame::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received CAN frame: ID=0x%X, DLC=%d, Data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
        //             msg->id, msg->dlc, msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
        
        uint8_t data[8];
        for (int i = 0; i < 8; i++) data[i] = msg->data[i];
        CanFrame rx = can_driver_->rx_frame(msg->id, data, msg->dlc, msg->is_rtr, msg->is_extended);
        
        // GPIO ドライバ内部状態を更新
        bool handled = gpio_driver_->receive(rx);


        // 安定性向上: 簡素化されたPWM_IN処理
        if (handled)
        {
            RCLCPP_INFO(this->get_logger(), "PIN READ: %d", rx.register_id);
            const uint16_t reg = rx.register_id;
            int pin = -1;
            // if (GPIO::PWM_PERIOD <= reg && reg < GPIO::PWM_PERIOD + N)
            // {
            //     pin = (reg - GPIO::PWM_PERIOD) % N;
            //     uint16_t period; 
            //     memcpy(&period, rx.data, sizeof(uint16_t));
            //     last_pwm_period_[pin] = period;
            //     pwm_data_updated_[pin] = true; // データ更新フラグを設定
            // }
            // else if (GPIO::PWM_DUTY <= reg && reg < GPIO::PWM_DUTY + N)
            // {
            //     pin = (reg - GPIO::PWM_DUTY) % N;
            //     uint16_t duty; 
            //     memcpy(&duty, rx.data, sizeof(uint16_t));
            //     last_pwm_duty_[pin] = duty;
            //     pwm_data_updated_[pin] = true; // データ更新フラグを設定
            // }

            // PORT_READの処理を簡素化
            if (reg == GPIO::PORT_READ)
            {
                uint16_t port_read_val = 0;
                memcpy(&port_read_val, rx.data, sizeof(uint16_t));

                uint16_t changed_mask = static_cast<uint16_t>(port_read_val ^ last_port_read_);

                for(int i = 0; i < N; i++){
                    if(((changed_mask >> i) & 0x1) == 0) continue;
                    if(pin_type[i] != "PWM_IN") continue;
                    port_read_updated_ = true; // ポート読み取り更新フラグを設定
                    RCLCPP_ERROR(get_logger(), "PORT_READ: port_read_val=%d", port_read_val);
                    if(last_port_read_ != port_read_val) {
                        sabacan_msgs::msg::SabacanGPIORes out;
                        out.header.stamp = this->get_clock()->now();
                        out.pin_number = static_cast<uint8_t>(i);
                        out.frequency = 0;
                        out.duty = port_read_val >> i;
                        out.pwm_period = 0;
                        out.pwm_duty = 0;
                        sabacan_res_publisher_->publish(out);
                    }
                }
                last_port_read_ = port_read_val;
            }
        }
    }

    // 安定性向上: タイマーベースのGPIO監視処理
    void gpio_monitor_callback()
    {
        // PWM_IN ピンのフィードバック処理
        for (int i = 0; i < N; ++i)
        {
            if (pin_type[i] == "PWM_IN" && pwm_data_updated_[i])
            {
                // 周波数/デューティ計算（50kHz カウンタ前提）
                float frequency_hz = 0.0f;
                float duty_ratio = 0.0f;
                const uint16_t period_cnt = last_pwm_period_[i];
                const uint16_t duty_cnt = last_pwm_duty_[i];
                
                if (period_cnt > 0)
                {
                    frequency_hz = 50000.0f / static_cast<float>(period_cnt);
                    duty_ratio = std::min(1.0f, static_cast<float>(duty_cnt) / static_cast<float>(period_cnt));
                }

                sabacan_msgs::msg::SabacanGPIORes out;
                out.header.stamp = this->get_clock()->now();
                out.pin_number = static_cast<uint8_t>(i);
                out.frequency = frequency_hz;
                out.duty = duty_ratio;
                out.pwm_period = period_cnt;
                out.pwm_duty = duty_cnt;
                sabacan_res_publisher_->publish(out);
                
                pwm_data_updated_[i] = false; // フラグをリセット
            }
        }

        // PORT_READの変化検出処理（簡素化版）
        if (port_read_updated_)
        {
            uint16_t port_read_val = last_port_read_;
            const auto now = this->get_clock()->now();
            
            for (int i = 0; i < N; ++i)
            {
                if (pin_type[i] != "PWM_IN") continue;
                
                const bool new_level = ((port_read_val >> i) & 0x1) != 0;
                const bool prev_level = last_level_[i];
                
                if (new_level != prev_level)
                {
                    // 簡素化されたエッジ検出
                    if (!prev_level && new_level) // Rising edge
                    {
                        last_rise_time_[i] = now;
                    }
                    else if (prev_level && !new_level) // Falling edge
                    {
                        last_fall_time_[i] = now;
                        
                        // 周期とデューティの計算（タイマー周期ベース）
                        if (last_rise_time_[i].nanoseconds() > 0 && 
                            last_fall_time_[i].nanoseconds() > last_rise_time_[i].nanoseconds())
                        {
                            const double period_sec = (now - last_rise_time_[i]).seconds();
                            const double high_sec = (last_fall_time_[i] - last_rise_time_[i]).seconds();
                            
                            if (period_sec > 0.0)
                            {
                                float freq = static_cast<float>(1.0 / period_sec);
                                float duty = static_cast<float>(std::clamp(high_sec / period_sec, 0.0, 1.0));
                                
                                sabacan_msgs::msg::SabacanGPIORes out;
                                out.header.stamp = now;
                                out.pin_number = static_cast<uint8_t>(i);
                                out.frequency = freq;
                                out.duty = duty;
                                out.pwm_period = last_pwm_period_[i];
                                out.pwm_duty = last_pwm_duty_[i];
                                sabacan_res_publisher_->publish(out);
                            }
                        }
                    }
                    last_level_[i] = new_level;
                }
            }
            port_read_updated_ = false; // フラグをリセット
        }
    }

    void sabacan_ref_callback(const sabacan_msgs::msg::SabacanGPIORef::SharedPtr msg)
    {
        if (pin_type[msg->pin_number] == "PWM_OUT")
        {
            gpio_driver_->setPwmPeriod(msg->pin_number, msg->pwm_period);
            gpio_driver_->setPwmDuty(msg->pin_number, msg->duty);
            // 以前の状態を保持しつつ、対象ピンだけ更新
            port_write_shadow_ = static_cast<uint16_t>(port_write_shadow_ | (1u << msg->pin_number));
            gpio_driver_->setPortWrite(port_write_shadow_);
        }
        else if (pin_type[msg->pin_number] == "ESC")
        {
            // 以前の状態を保持しつつ、対象ピンだけ更新
            esc_mode_en_shadow_ = static_cast<uint16_t>(esc_mode_en_shadow_ | (1u << msg->pin_number));
            gpio_driver_->setEscModeEn(esc_mode_en_shadow_);
            gpio_driver_->setPwmDuty(msg->pin_number, msg->duty);
        }
        // RCLCPP_INFO(this->get_logger(), "Received SabacanGPIORef: pin_number=%d, pwm_period=%d, duty=%f", msg->pin_number, msg->pwm_period, msg->duty);
    }

    // 安定性向上: 強化されたGPIO初期化処理
    void gpio_init()
    {
        int retry_count = 0;
        
        while (retry_count < max_init_retries_)
        {
            try {
                RCLCPP_INFO(this->get_logger(), "Starting GPIO initialization (attempt %d/%d)...", 
                            retry_count + 1, max_init_retries_);
                
                // monitor_period_msパラメータを取得
                int monitor_period_ms = this->get_parameter("monitor_period_ms").as_int();
                bool enable_monitor_period = this->get_parameter("enable_monitor_period").as_bool();
                
                RCLCPP_INFO(this->get_logger(), "GPIO init parameters: monitor_period_ms=%d, enable_monitor_period=%s", 
                            monitor_period_ms, enable_monitor_period ? "true" : "false");
                
                // PWM_IN ピンのみ入力モードに設定し、PWM_PERIOD/PWM_DUTY を定期的に監視
                uint16_t port_mode_mask = 0;     // 1: 入力, 0: 出力
                uint16_t port_int_en_mask = 0;   // ピン変化割り込みの有効化（必要に応じて）
                uint64_t monitor_reg_mask = 0;   // 0x00..0x3F の reg ID をビットで指定

                for (int i = 0; i < N; ++i)
                {
                    if (pin_type[i] == "PWM_IN")
                    {
                        port_mode_mask |= static_cast<uint16_t>(1u << i);
                        port_int_en_mask |= static_cast<uint16_t>(1u << i);
                        // reg id: 0x10+i (PWM_PERIOD), 0x20+i (PWM_DUTY)
                        monitor_reg_mask |= (1ULL << (GPIO::PWM_PERIOD + i));
                        monitor_reg_mask |= (1ULL << (GPIO::PWM_DUTY + i));
                        RCLCPP_INFO(this->get_logger(), "Pin %d configured as PWM_IN", i);
                    }
                }

                RCLCPP_INFO(this->get_logger(), "Setting GPIO configuration: PORT_MODE=0x%04X, PORT_INT_EN=0x%04X", 
                            port_mode_mask, port_int_en_mask);

                // GPIO設定を段階的に適用（エラーハンドリング強化）
                RCLCPP_INFO(this->get_logger(), "Calling setPortMode(0x%04X)...", port_mode_mask);
                gpio_driver_->setPortMode(port_mode_mask);
                RCLCPP_INFO(this->get_logger(), "setPortMode completed");
                
                RCLCPP_INFO(this->get_logger(), "Calling setPortIntEn(0x%04X)...", port_int_en_mask);
                gpio_driver_->setPortIntEn(port_int_en_mask);
                RCLCPP_INFO(this->get_logger(), "setPortIntEn completed");
                
                // PORT_READ を割り込み通知で受ける想定。念のためモニタ登録にも追加
                monitor_reg_mask |= (1ULL << GPIO::PORT_READ);
                
                RCLCPP_INFO(this->get_logger(), "Setting monitor register mask: 0x%016llX", 
                            static_cast<unsigned long long>(monitor_reg_mask));
                
                // 定期フィードバックを有効化（パラメータで設定）
                RCLCPP_INFO(this->get_logger(), "Calling setMonitorReg(0x%016llX)...", 
                            static_cast<unsigned long long>(monitor_reg_mask));
                gpio_driver_->setMonitorReg(monitor_reg_mask);
                RCLCPP_INFO(this->get_logger(), "setMonitorReg completed");
                
                if (enable_monitor_period) {
                    RCLCPP_INFO(this->get_logger(), "Calling setMonitorPeriod(%d)...", monitor_period_ms);
                    gpio_driver_->setMonitorPeriod(monitor_period_ms);
                    RCLCPP_INFO(this->get_logger(), "setMonitorPeriod completed");
                    RCLCPP_INFO(this->get_logger(), "Monitor period set to %d ms", monitor_period_ms);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Monitor period disabled");
                }

                // 初期化成功時の状態リセット
                reset_gpio_state();
                // シャドウ状態の初期化
                port_mode_shadow_ = port_mode_mask;
                port_write_shadow_ = 0;
                esc_mode_en_shadow_ = 0;
                
                RCLCPP_INFO(this->get_logger(), "GPIO init completed successfully: PORT_MODE=0x%04X, PORT_INT_EN=0x%04X, MONITOR_REG=0x%016llX, MONITOR_PERIOD=%dms, ENABLE_MONITOR=%s",
                            port_mode_mask, port_int_en_mask, static_cast<unsigned long long>(monitor_reg_mask), monitor_period_ms, enable_monitor_period ? "true" : "false");
                
                // 成功時もループを継続（パラメータで設定した回数まで）
                retry_count++;
                RCLCPP_INFO(this->get_logger(), "GPIO initialization attempt %d/%d completed successfully", 
                            retry_count, max_init_retries_);
                
                // 最後の試行でない場合は待機してから次の試行
                if (retry_count < max_init_retries_) {
                    RCLCPP_INFO(this->get_logger(), "Waiting %d ms before next attempt...", init_retry_delay_ms_);
                    std::this_thread::sleep_for(std::chrono::milliseconds(init_retry_delay_ms_));
                }
                
            } catch (const std::exception& e) {
                retry_count++;
                RCLCPP_WARN(this->get_logger(), "GPIO initialization attempt %d/%d failed: %s", 
                            retry_count, max_init_retries_, e.what());
                
                if (retry_count >= max_init_retries_) {
                    RCLCPP_ERROR(this->get_logger(), "GPIO initialization failed after %d attempts", max_init_retries_);
                    throw std::runtime_error("GPIO initialization failed after maximum retries: " + std::string(e.what()));
                }
                
                // リトライ前の待機
                RCLCPP_INFO(this->get_logger(), "Waiting %d ms before retry...", init_retry_delay_ms_);
                std::this_thread::sleep_for(std::chrono::milliseconds(init_retry_delay_ms_));
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "GPIO initialization completed: %d attempts executed", max_init_retries_);
    }

    // 安定性向上: GPIO状態のリセット処理
    void reset_gpio_state()
    {
        for (int i = 0; i < N; ++i)
        {
            last_pwm_period_[i] = 0;
            last_pwm_duty_[i] = 0;
            last_level_[i] = false;
            last_rise_time_[i] = rclcpp::Time(0);
            last_fall_time_[i] = rclcpp::Time(0);
            pwm_data_updated_[i] = false;
        }
        last_port_read_ = 0;
        port_read_updated_ = false;
        
        RCLCPP_INFO(this->get_logger(), "GPIO state reset completed");
    }

    // リセットサービスコールバック
    void reset_callback(
        const std::shared_ptr<sabacan_msgs::srv::SabacanReset::Request> request,
        std::shared_ptr<sabacan_msgs::srv::SabacanReset::Response> response)
    {
        (void)request; // 未使用パラメータ警告を抑制
        try
        {
            RCLCPP_INFO(this->get_logger(), "Received reset request for GPIO node");
            
            // GPIO初期化処理を実行
            gpio_init();
            
            response->success = true;
            response->message = "GPIO node reset completed successfully";
            
            RCLCPP_INFO(this->get_logger(), "GPIO node reset completed");
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message = std::string("Error during GPIO reset: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "GPIO reset failed: %s", e.what());
        }
    }

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_;
    rclcpp::Subscription<sabacan_msgs::msg::SabacanGPIORef>::SharedPtr sabacan_ref_subscription_;
    rclcpp::Publisher<sabacan_msgs::msg::SabacanGPIORes>::SharedPtr sabacan_res_publisher_;
    rclcpp::Service<sabacan_msgs::srv::SabacanReset>::SharedPtr reset_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr gpio_monitor_timer_; // 安定性向上: GPIO監視用タイマー
    std::shared_ptr<CanDriver> can_driver_;
    std::unique_ptr<CommonDataDriver> common_data_driver_;

    std::string pin_type[9] = {"PWM_OUT", "PWM_OUT", "PWM_OUT", "PWM_OUT", "PWM_OUT", "PWM_OUT", "PWM_OUT", "PWM_IN", "ESC"};
    std::unique_ptr<GPIODriver> gpio_driver_;
    
    // PWM 入力の最新値を保持
    uint16_t last_pwm_period_[N] = {0};
    uint16_t last_pwm_duty_[N] = {0};
    
    // 安定性向上: 簡素化された状態管理
    bool last_level_[N] = {false};
    rclcpp::Time last_rise_time_[N];
    rclcpp::Time last_fall_time_[N];
    
    // 安定性向上: データ更新フラグ
    bool pwm_data_updated_[N] = {false};
    uint16_t last_port_read_ = 0;
    bool port_read_updated_ = false;
    
    // 安定性向上: 初期化パラメータ
    int max_init_retries_ = 3;
    int init_retry_delay_ms_ = 100;

    // 送信状態のシャドウ（前回値の保持）
    uint16_t port_mode_shadow_ = 0;   // 入出力モード（初期化時に設定）
    uint16_t port_write_shadow_ = 0;  // 出力マスク（PWM_OUT等）
    uint16_t esc_mode_en_shadow_ = 0; // ESCモード有効マスク
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SabaneCanNode>());
    rclcpp::shutdown();
    return 0;
}