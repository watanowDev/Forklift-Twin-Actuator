/**
 * @file signal_controller_node.cpp
 * @brief ROS2 신호 제어 노드 - /actuators/signal 토픽 구독하여 PatliteLedBuzzer 하드웨어 제어
 * 
 * 제조사 독립적인 신호 장치 제어를 위한 추상화된 노드
 * SignalCommand 메시지를 받아 21가지 시나리오에 따라 LED/Buzzer 제어
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <functional>

#include "fta_interfaces/msg/signal_command.hpp"
#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_types.hpp"
#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_hardware_interface.hpp"

using namespace fta_actuators;
using namespace fta_actuators::patlite_led_buzzer;

/**
 * @class SignalControllerNode
 * @brief /actuators/signal 토픽을 구독하여 신호 장치를 제어하는 ROS2 노드
 */
class SignalControllerNode : public rclcpp::Node
{
public:
    SignalControllerNode() : Node("signal_controller_node")
    {
        // 파라미터 선언
        this->declare_parameter<std::string>("driver_type", "usb_direct");
        this->declare_parameter<int>("vendor_id", 0x191a);
        this->declare_parameter<int>("product_id", 0x6001);
        
        // 파라미터 가져오기
        std::string driver_type = this->get_parameter("driver_type").as_string();
        int vendor_id = this->get_parameter("vendor_id").as_int();
        int product_id = this->get_parameter("product_id").as_int();
        
        // 하드웨어 드라이버 생성
        auto driver_type_enum = PatliteLedBuzzerHardwareFactory::parse_driver_type(driver_type);
        hardware_ = PatliteLedBuzzerHardwareFactory::create_driver(driver_type_enum);
        if (!hardware_) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to create hardware driver: %s", driver_type.c_str());
            throw std::runtime_error("Failed to create hardware driver");
        }
        
        // 하드웨어 초기화
        if (!hardware_->open()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to open PatliteLedBuzzer device (VID:0x%04x, PID:0x%04x)", 
                        vendor_id, product_id);
            throw std::runtime_error("Failed to open PatliteLedBuzzer device");
        }
        
        RCLCPP_INFO(this->get_logger(), "✅ PatliteLedBuzzer hardware initialized successfully");
        
        // /actuators/signal 구독자 생성 (QoS: Reliable)
        subscription_ = this->create_subscription<fta_interfaces::msg::SignalCommand>(
            "/actuators/signal",
            rclcpp::QoS(10).reliable(),
            std::bind(&SignalControllerNode::signal_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "🎯 Signal controller node started. Subscribing to /actuators/signal");
        
        // 시나리오 핸들러 맵 초기화
        init_scenario_handlers();
    }
    
    ~SignalControllerNode()
    {
        if (hardware_) {
            hardware_->close();
            RCLCPP_INFO(this->get_logger(), "🔌 PatliteLedBuzzer hardware closed");
        }
    }

private:
    /**
     * @brief /actuators/signal 토픽 콜백 함수
     */
    void signal_callback(const fta_interfaces::msg::SignalCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "📩 Received signal: action_type='%s', priority=%d", 
                   msg->action_type.c_str(), msg->priority);
        
        // 시나리오 핸들러 맵에서 해당 액션 찾기
        auto it = scenario_handlers_.find(msg->action_type);
        if (it != scenario_handlers_.end()) {
            // 핸들러 실행 (LED + Buzzer 제어)
            it->second();
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️  Unknown action_type: '%s'", msg->action_type.c_str());
        }
    }
    
    /**
     * @brief 시나리오별 핸들러 함수 맵 초기화
     */
    void init_scenario_handlers()
    {
        // 각 시나리오에 대한 핸들러 등록
        scenario_handlers_["CONTAINER_OK"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "✅ CONTAINER_OK: Green Continuous + Buzzer Continuous");
            execute_led_buzzer(LEDColor::GREEN, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1, 1);
        };
        
        scenario_handlers_["SIZE_CHECK_START"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "📏 SIZE_CHECK_START: Green Pattern3 + Buzzer Pattern2");
            execute_led_buzzer(LEDColor::GREEN, LEDPattern::PATTERN3, BuzzerPattern::PATTERN2, 1, 1);
        };
        
        scenario_handlers_["SIZE_MEASURE_OK"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "📐 SIZE_MEASURE_OK: Green Pattern6 + Buzzer Pattern1");
            execute_led_buzzer(LEDColor::GREEN, LEDPattern::PATTERN6, BuzzerPattern::PATTERN1, 1, 1);
        };
        
        scenario_handlers_["NO_QR_SIZE_MEASURE_OK"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "🟣 NO_QR_SIZE_MEASURE_OK: Purple Pattern6 + Buzzer Pattern1");
            execute_led_buzzer(LEDColor::PURPLE, LEDPattern::PATTERN6, BuzzerPattern::PATTERN1, 1, 1);
        };
        
        scenario_handlers_["QR_PICKUP"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "📦 QR_PICKUP: Green Pattern3 + Buzzer Pattern2");
            execute_led_buzzer(LEDColor::GREEN, LEDPattern::PATTERN3, BuzzerPattern::PATTERN2, 1, 1);
        };
        
        scenario_handlers_["QR_MEASURE_OK"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "✅ QR_MEASURE_OK: Green Continuous + Buzzer Continuous");
            execute_led_buzzer(LEDColor::GREEN, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1, 1);
        };
        
        scenario_handlers_["NO_QR_PICKUP"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "🟣 NO_QR_PICKUP: Purple Pattern3 + Buzzer Pattern2");
            execute_led_buzzer(LEDColor::PURPLE, LEDPattern::PATTERN3, BuzzerPattern::PATTERN2, 1, 1);
        };
        
        scenario_handlers_["NO_QR_MEASURE_OK"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "🟣 NO_QR_MEASURE_OK: Purple Continuous + Buzzer Continuous");
            execute_led_buzzer(LEDColor::PURPLE, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1, 1);
        };
        
        scenario_handlers_["SET_ITEM"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "🔵 SET_ITEM: Cyan Continuous + Buzzer Continuous");
            execute_led_buzzer(LEDColor::CYAN, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1, 1);
        };
        
        scenario_handlers_["SET_ITEM_NORMAL"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "🔵 SET_ITEM_NORMAL: Cyan Continuous + Buzzer OFF");
            execute_led_buzzer(LEDColor::CYAN, LEDPattern::CONTINUOUS, BuzzerPattern::OFF, 0, 0);
        };
        
        scenario_handlers_["SET_ITEM_PICKUP"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "🔵 SET_ITEM_PICKUP: Cyan Pattern3 + Buzzer Pattern2");
            execute_led_buzzer(LEDColor::CYAN, LEDPattern::PATTERN3, BuzzerPattern::PATTERN2, 1, 1);
        };
        
        scenario_handlers_["SET_ITEM_SIZE_CHECK_START"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "🔵 SET_ITEM_SIZE_CHECK_START: Cyan Pattern3 + Buzzer Pattern2");
            execute_led_buzzer(LEDColor::CYAN, LEDPattern::PATTERN3, BuzzerPattern::PATTERN2, 1, 1);
        };
        
        scenario_handlers_["SET_ITEM_MEASURE_OK"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "🔵 SET_ITEM_MEASURE_OK: Cyan Continuous + Buzzer Continuous");
            execute_led_buzzer(LEDColor::CYAN, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1, 1);
        };
        
        scenario_handlers_["CLEAR_ITEM"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "🔵 CLEAR_ITEM: Cyan Continuous + Buzzer Continuous");
            execute_led_buzzer(LEDColor::CYAN, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1, 1);
        };
        
        scenario_handlers_["DROP"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "📥 DROP: Green Continuous + Buzzer OFF");
            execute_led_buzzer(LEDColor::GREEN, LEDPattern::CONTINUOUS, BuzzerPattern::OFF, 0, 0);
        };
        
        scenario_handlers_["DEVICE_ERROR"] = [this]() {
            RCLCPP_ERROR(this->get_logger(), "❌ DEVICE_ERROR: Red Pattern2 + Buzzer Pattern4");
            execute_led_buzzer(LEDColor::RED, LEDPattern::PATTERN2, BuzzerPattern::PATTERN4, 1, 1);
        };
        
        scenario_handlers_["DEVICE_ERROR_CLEAR"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "✅ DEVICE_ERROR_CLEAR: Green Continuous + Buzzer Continuous");
            execute_led_buzzer(LEDColor::GREEN, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1, 1);
        };
        
        scenario_handlers_["CHECK_COMPLETE"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "✅ CHECK_COMPLETE: Green Continuous + Buzzer Continuous");
            execute_led_buzzer(LEDColor::GREEN, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1, 1);
        };
        
        scenario_handlers_["NO_QR_CHECK_COMPLETE"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "🟣 NO_QR_CHECK_COMPLETE: Purple Continuous + Buzzer Continuous");
            execute_led_buzzer(LEDColor::PURPLE, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1, 1);
        };
        
        scenario_handlers_["SET_ITEM_CHECK_COMPLETE"] = [this]() {
            RCLCPP_INFO(this->get_logger(), "🔵 SET_ITEM_CHECK_COMPLETE: Cyan Continuous + Buzzer Continuous");
            execute_led_buzzer(LEDColor::CYAN, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1, 1);
        };
        
        scenario_handlers_["INVALID_PLACE"] = [this]() {
            RCLCPP_WARN(this->get_logger(), "⚠️  INVALID_PLACE: Red Pattern6 + Buzzer Pattern3 (Pedestrian Detection)");
            execute_led_buzzer(LEDColor::RED, LEDPattern::PATTERN6, BuzzerPattern::PATTERN3, 1, 1);
        };
    }
    
    /**
     * @brief LED + Buzzer 동시 제어 실행
     */
    void execute_led_buzzer(LEDColor led_color, LEDPattern led_pattern,
                           BuzzerPattern buzzer_pattern, uint8_t count, uint8_t volume)
    {
        // LED 제어
        if (!hardware_->set_led(led_color, led_pattern)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to set LED");
        }
        
        // Buzzer 제어
        if (!hardware_->set_buzzer(buzzer_pattern, count, volume)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to set Buzzer");
        }
    }

    // ROS2 구독자
    rclcpp::Subscription<fta_interfaces::msg::SignalCommand>::SharedPtr subscription_;
    
    // PatliteLedBuzzer 하드웨어 인터페이스
    std::unique_ptr<PatliteLedBuzzerHardwareInterface> hardware_;
    
    // 시나리오별 핸들러 맵 (action_type -> handler function)
    std::unordered_map<std::string, std::function<void()>> scenario_handlers_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<SignalControllerNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("signal_controller_node"), 
                    "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
