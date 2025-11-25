// ============================================================================
// simple_subscriber.cpp
// 가장 간단한 ROS2 Subscriber 노드
//
// 역할: "/test_topic"에서 메시지를 받아서 출력합니다.
// 실행: ros2 run fta_beginner simple_subscriber
// ============================================================================

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// SimpleSubscriber 클래스 정의
class SimpleSubscriber : public rclcpp::Node
{
public:
    // 생성자
    SimpleSubscriber()
        : Node("simple_subscriber")
    {
        // Subscriber 생성
        // - std_msgs::msg::String: 받을 메시지 타입
        // - "test_topic": 구독할 토픽 이름
        // - 10: 메시지 큐 크기
        // - std::bind(...): 메시지 받았을 때 호출할 함수
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "test_topic",
            10,
            std::bind(&SimpleSubscriber::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "SimpleSubscriber 노드가 시작되었습니다!");
        RCLCPP_INFO(this->get_logger(), "토픽 구독 중: /test_topic");
    }

private:
    // 토픽 콜백 함수: 메시지가 도착하면 자동으로 호출됩니다
    // msg: 받은 메시지 (포인터)
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // 받은 메시지 출력
        RCLCPP_INFO(this->get_logger(), "받은 메시지: '%s'", msg->data.c_str());
    }

    // 멤버 변수
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

// main 함수
int main(int argc, char *argv[])
{
    // ROS2 초기화
    rclcpp::init(argc, argv);

    // SimpleSubscriber 노드 생성
    auto node = std::make_shared<SimpleSubscriber>();

    // 노드 실행
    rclcpp::spin(node);

    // ROS2 종료
    rclcpp::shutdown();
    return 0;
}
