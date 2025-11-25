// ============================================================================
// patlite_test.cpp
// Patlite USB 장치 인터페이스 테스트 노드
//
// 역할:
// 1. USB 장치 확인 (lsusb 명령어 실행)
// 2. "/patlite/command" 토픽을 구독하여 명령 수신
// 3. 받은 명령을 로그로 출력 (실제 제어는 나중에 구현)
//
// 실행: ros2 run fta_beginner patlite_test
// 테스트: ros2 topic pub --once /patlite/command std_msgs/msg/String "{data: 'LED_RED_ON'}"
// ============================================================================

#include <memory>
#include <string>
#include <cstdlib> // system() 함수 사용

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PatliteTestNode : public rclcpp::Node
{
public:
    PatliteTestNode()
        : Node("patlite_test_node")
    {
        // 노드 시작 시 USB 장치 확인
        check_usb_devices();

        // 명령을 받기 위한 Subscriber 생성
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/patlite/command",
            10,
            std::bind(&PatliteTestNode::command_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Patlite 테스트 노드가 시작되었습니다!");
        RCLCPP_INFO(this->get_logger(), "명령 토픽: /patlite/command");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "테스트 방법:");
        RCLCPP_INFO(this->get_logger(), "ros2 topic pub --once /patlite/command std_msgs/msg/String \"{data: 'LED_RED_ON'}\"");
        RCLCPP_INFO(this->get_logger(), "");
    }

private:
    // USB 장치 확인 함수
    void check_usb_devices()
    {
        RCLCPP_INFO(this->get_logger(), "USB 장치 확인 중...");
        RCLCPP_INFO(this->get_logger(), "==== lsusb 출력 시작 ====");

        // lsusb 명령어 실행 (Linux 시스템 명령어)
        // Windows에서는 작동하지 않으니 Ubuntu에서 실행하세요
        int result = std::system("lsusb");

        RCLCPP_INFO(this->get_logger(), "==== lsusb 출력 끝 ====");

        if (result == 0)
        {
            RCLCPP_INFO(this->get_logger(), "USB 장치 확인 완료!");
            RCLCPP_INFO(this->get_logger(), "위 목록에서 'Patlite' 또는 'LED' 관련 장치를 찾으세요.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "lsusb 명령어 실행 실패. Linux 시스템에서 실행하세요.");
        }
        RCLCPP_INFO(this->get_logger(), "");
    }

    // 명령 수신 콜백 함수
    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data;

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "명령 수신: '%s'", command.c_str());

        // 명령에 따른 동작 (현재는 로그만 출력)
        if (command == "LED_RED_ON")
        {
            RCLCPP_INFO(this->get_logger(), "→ 빨간색 LED를 켭니다 (시뮬레이션)");
            // TODO: 실제 USB 통신 구현
        }
        else if (command == "LED_GREEN_ON")
        {
            RCLCPP_INFO(this->get_logger(), "→ 초록색 LED를 켭니다 (시뮬레이션)");
        }
        else if (command == "LED_YELLOW_ON")
        {
            RCLCPP_INFO(this->get_logger(), "→ 노란색 LED를 켭니다 (시뮬레이션)");
        }
        else if (command == "LED_OFF")
        {
            RCLCPP_INFO(this->get_logger(), "→ 모든 LED를 끕니다 (시뮬레이션)");
        }
        else if (command == "BUZZER_ON")
        {
            RCLCPP_INFO(this->get_logger(), "→ 부저를 켭니다 (시뮬레이션)");
        }
        else if (command == "BUZZER_OFF")
        {
            RCLCPP_INFO(this->get_logger(), "→ 부저를 끕니다 (시뮬레이션)");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "→ 알 수 없는 명령입니다!");
            RCLCPP_INFO(this->get_logger(), "지원 명령: LED_RED_ON, LED_GREEN_ON, LED_YELLOW_ON, LED_OFF, BUZZER_ON, BUZZER_OFF");
        }

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "");
    }

    // 멤버 변수
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

// main 함수
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatliteTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
