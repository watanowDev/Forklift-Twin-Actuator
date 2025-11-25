// ============================================================================
// simple_publisher.cpp
// 가장 간단한 ROS2 Publisher 노드
//
// 역할: 1초마다 "/test_topic"에 메시지를 발행합니다.
// 실행: ros2 run fta_beginner simple_publisher
// ============================================================================

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals; // 1s, 500ms 같은 시간 표기 사용 가능

// SimplePublisher 클래스 정의
// rclcpp::Node를 상속받아서 ROS2 노드가 됩니다
class SimplePublisher : public rclcpp::Node
{
public:
    // 생성자: 노드 이름을 "simple_publisher"로 설정
    SimplePublisher()
        : Node("simple_publisher"), count_(0)
    {
        // Publisher 생성
        // - std_msgs::msg::String: 보낼 메시지 타입
        // - "test_topic": 토픽 이름
        // - 10: 메시지 큐 크기 (QoS 설정)
        publisher_ = this->create_publisher<std_msgs::msg::String>("test_topic", 10);

        // 타이머 생성: 1초마다 timer_callback 함수 호출
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&SimplePublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "SimplePublisher 노드가 시작되었습니다!");
        RCLCPP_INFO(this->get_logger(), "1초마다 메시지를 발행합니다: /test_topic");
    }

private:
    // 타이머 콜백 함수: 1초마다 자동으로 호출됩니다
    void timer_callback()
    {
        // 메시지 객체 생성
        auto message = std_msgs::msg::String();
        message.data = "안녕하세요! 메시지 번호: " + std::to_string(count_++);

        // 로그 출력
        RCLCPP_INFO(this->get_logger(), "발행 중: '%s'", message.data.c_str());

        // 메시지 발행
        publisher_->publish(message);
    }

    // 멤버 변수
    rclcpp::TimerBase::SharedPtr timer_;                            // 타이머 객체
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // Publisher 객체
    size_t count_;                                                  // 메시지 카운터
};

// main 함수: 프로그램 시작점
int main(int argc, char *argv[])
{
    // ROS2 초기화
    rclcpp::init(argc, argv);

    // SimplePublisher 노드 생성
    auto node = std::make_shared<SimplePublisher>();

    // 노드 실행 (Ctrl+C로 종료할 때까지 계속 실행)
    rclcpp::spin(node);

    // ROS2 종료
    rclcpp::shutdown();
    return 0;
}
