#include "fta_actuators/led_buzzer_node.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

namespace fta_actuators
{

// ============================================
// 생성자
// ============================================
LEDBuzzerNode::LEDBuzzerNode(const rclcpp::NodeOptions & options)
: Node("led_buzzer_node", options),
  enabled_(true),
  volume_(50)
{
  // 파라미터 선언
  this->declare_parameter("enabled", true);
  this->declare_parameter("volume", 50);
  this->declare_parameter("device_path", "");
  
  // 파라미터 읽기
  enabled_ = this->get_parameter("enabled").as_bool();
  volume_ = this->get_parameter("volume").as_int();
  device_path_ = this->get_parameter("device_path").as_string();
  
  RCLCPP_INFO(this->get_logger(), "LED/Buzzer Node 시작");
  RCLCPP_INFO(this->get_logger(), "  - enabled: %s", enabled_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  - volume: %d", volume_);
  
  if (!enabled_) {
    RCLCPP_WARN(this->get_logger(), "LED/Buzzer 노드가 비활성화되었습니다");
    return;
  }
  
  // Patlite 드라이버 초기화
  driver_ = std::make_unique<PatliteDriver>();
  if (!initialize_device()) {
    RCLCPP_ERROR(this->get_logger(), "Patlite 디바이스 초기화 실패");
    publish_status("error", "Failed to initialize Patlite device", 1);
    return;
  }
  
  // ROS2 구독자 생성 (QoS1 - Reliable)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  action_sub_ = this->create_subscription<fta_interfaces::msg::ActionEvent>(
    "/actions/event",
    qos,
    std::bind(&LEDBuzzerNode::action_callback, this, std::placeholders::_1)
  );
  
  // ROS2 발행자 생성 (QoS1 - Reliable)
  status_pub_ = this->create_publisher<fta_interfaces::msg::ActuatorStatus>(
    "/actuators/status",
    qos
  );
  
  // 초기 상태 발행
  publish_status("idle", "LED/Buzzer node initialized");
  
  RCLCPP_INFO(this->get_logger(), "LED/Buzzer 노드 초기화 완료");
}

// ============================================
// 소멸자
// ============================================
LEDBuzzerNode::~LEDBuzzerNode()
{
  cleanup_device();
  RCLCPP_INFO(this->get_logger(), "LED/Buzzer 노드 종료");
}

// ============================================
// 디바이스 초기화
// ============================================
bool LEDBuzzerNode::initialize_device()
{
  if (!driver_->open_device()) {
    return false;
  }
  
  // 초기 LED 상태: 초록색 연속 점등
  driver_->set_light(LEDColor::GREEN, LEDPattern::CONTINUOUS);
  
  return true;
}

// ============================================
// 디바이스 정리
// ============================================
void LEDBuzzerNode::cleanup_device()
{
  if (driver_) {
    // LED/부저 끄기
    driver_->set_light(LEDColor::CLEAR, LEDPattern::OFF);
    driver_->set_buzzer(BuzzerPattern::STOP, 0, 0);
    driver_->close_device();
  }
}

// ============================================
// 액션 이벤트 콜백
// ============================================
void LEDBuzzerNode::action_callback(const fta_interfaces::msg::ActionEvent::SharedPtr msg)
{
  // 액추에이터 타입 확인
  if (msg->actuator_type != "led_buzzer") {
    return;  // 다른 액추에이터용 메시지는 무시
  }
  
  RCLCPP_INFO(this->get_logger(), "Action received: %s", msg->action.c_str());
  
  // 액션 처리
  bool success = handle_led_buzzer_action(msg->action, msg->parameters);
  
  if (success) {
    publish_status("active", "Action executed: " + msg->action);
  } else {
    publish_status("error", "Failed to execute action: " + msg->action, 2);
  }
}

// ============================================
// LED/Buzzer 액션 처리
// ============================================
bool LEDBuzzerNode::handle_led_buzzer_action(
  const std::string & action, 
  const std::string & parameters)
{
  if (!driver_ || !driver_->is_connected()) {
    RCLCPP_ERROR(this->get_logger(), "디바이스가 연결되지 않음");
    return false;
  }
  
  try {
    // JSON 파라미터 파싱
    auto params = json::parse(parameters);
    
    if (action == "set_light") {
      // LED 제어
      std::string color_str = params.value("color", "clear");
      std::string pattern_str = params.value("pattern", "off");
      
      LEDColor color = PatliteDriver::string_to_color(color_str);
      LEDPattern pattern = PatliteDriver::string_to_pattern(pattern_str);
      
      return driver_->set_light(color, pattern);
    }
    else if (action == "set_buzzer") {
      // 부저 제어
      std::string pattern_str = params.value("pattern", "stop");
      int count = params.value("count", 1);
      
      BuzzerPattern pattern = PatliteDriver::string_to_buzzer_pattern(pattern_str);
      
      return driver_->set_buzzer(pattern, volume_, count);
    }
    else if (action == "set_all") {
      // LED + 부저 동시 제어
      std::string color_str = params.value("color", "clear");
      std::string led_pattern_str = params.value("led_pattern", "off");
      std::string buzzer_pattern_str = params.value("buzzer_pattern", "stop");
      int count = params.value("count", 0);
      
      LEDColor color = PatliteDriver::string_to_color(color_str);
      LEDPattern led_pattern = PatliteDriver::string_to_pattern(led_pattern_str);
      BuzzerPattern buzzer_pattern = PatliteDriver::string_to_buzzer_pattern(buzzer_pattern_str);
      
      bool led_ok = driver_->set_light(color, led_pattern);
      bool buzzer_ok = driver_->set_buzzer(buzzer_pattern, volume_, count);
      
      return led_ok && buzzer_ok;
    }
    else if (action == "clear") {
      // 모두 끄기
      bool led_ok = driver_->set_light(LEDColor::CLEAR, LEDPattern::OFF);
      bool buzzer_ok = driver_->set_buzzer(BuzzerPattern::STOP, 0, 0);
      return led_ok && buzzer_ok;
    }
    else {
      RCLCPP_WARN(this->get_logger(), "알 수 없는 액션: %s", action.c_str());
      return false;
    }
  }
  catch (const json::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "JSON 파싱 오류: %s", e.what());
    return false;
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "예외 발생: %s", e.what());
    return false;
  }
}

// ============================================
// 상태 발행
// ============================================
void LEDBuzzerNode::publish_status(
  const std::string & status, 
  const std::string & message, 
  int error_code)
{
  if (!status_pub_) {
    return;
  }
  
  auto status_msg = fta_interfaces::msg::ActuatorStatus();
  status_msg.actuator_type = "led_buzzer";
  status_msg.status = status;
  status_msg.message = message;
  status_msg.error_code = error_code;
  status_msg.timestamp = this->now();
  
  status_pub_->publish(status_msg);
}

}  // namespace fta_actuators

// ============================================
// 메인 함수
// ============================================
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fta_actuators::LEDBuzzerNode)
