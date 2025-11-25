#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "fta_interfaces/msg/action_event.hpp"

using namespace std::chrono_literals;

// ============================================
// 테스트용 액션 발행 노드
// FTE 없이 LED/Buzzer를 테스트하기 위한 노드
// ============================================
class TestActionPublisher : public rclcpp::Node
{
public:
  TestActionPublisher()
  : Node("test_action_publisher")
  {
    // QoS1 (Reliable)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    
    publisher_ = this->create_publisher<fta_interfaces::msg::ActionEvent>(
      "/actions/event", 
      qos
    );
    
    // 5초마다 테스트 메시지 발행
    timer_ = this->create_wall_timer(
      5s, 
      std::bind(&TestActionPublisher::publish_test_action, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "테스트 액션 발행 노드 시작");
    RCLCPP_INFO(this->get_logger(), "5초마다 테스트 액션을 발행합니다");
  }

private:
  void publish_test_action()
  {
    auto msg = fta_interfaces::msg::ActionEvent();
    msg.actuator_type = "led_buzzer";
    msg.timestamp = this->now();
    
    // 테스트 시나리오 순환
    static int test_case = 0;
    
    switch (test_case % 5) {
      case 0:
        // 초록색 연속 점등
        RCLCPP_INFO(this->get_logger(), "[테스트 1] 초록색 연속 점등");
        msg.action = "set_all";
        msg.parameters = R"({
          "color": "green",
          "led_pattern": "continuous",
          "buzzer_pattern": "stop",
          "count": 0
        })";
        break;
        
      case 1:
        // 노란색 점멸
        RCLCPP_INFO(this->get_logger(), "[테스트 2] 노란색 점멸");
        msg.action = "set_light";
        msg.parameters = R"({
          "color": "yellow",
          "pattern": "flash1"
        })";
        break;
        
      case 2:
        // 빨간색 + 부저
        RCLCPP_INFO(this->get_logger(), "[테스트 3] 빨간색 + 부저");
        msg.action = "set_all";
        msg.parameters = R"({
          "color": "red",
          "led_pattern": "continuous",
          "buzzer_pattern": "pattern6",
          "count": 3
        })";
        break;
        
      case 3:
        // 부저만
        RCLCPP_INFO(this->get_logger(), "[테스트 4] 부저만");
        msg.action = "set_buzzer";
        msg.parameters = R"({
          "pattern": "pattern1",
          "count": 2
        })";
        break;
        
      case 4:
        // 모두 끄기
        RCLCPP_INFO(this->get_logger(), "[테스트 5] 모두 끄기");
        msg.action = "clear";
        msg.parameters = "{}";
        break;
    }
    
    publisher_->publish(msg);
    test_case++;
  }

  rclcpp::Publisher<fta_interfaces::msg::ActionEvent>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<TestActionPublisher>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
