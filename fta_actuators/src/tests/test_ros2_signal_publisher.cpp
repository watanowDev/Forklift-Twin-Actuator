/**
 * @file test_ros2_signal_publisher.cpp
 * @brief ROS2 신호 제어 통합 테스트 - 21개 시나리오 순차 발행
 * 
 * signal_controller_node가 실행 중일 때, 이 프로그램이 /actuators/signal 토픽으로
 * 21개 시나리오 메시지를 순차적으로 발행하여 전체 통합 테스트 수행
 */

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include "fta_interfaces/msg/signal_command.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_ros2_signal_publisher");
    
    // /actuators/signal 토픽 발행자 생성 (QoS: Reliable)
    auto publisher = node->create_publisher<fta_interfaces::msg::SignalCommand>(
        "/actuators/signal", 
        rclcpp::QoS(10).reliable());
    
    RCLCPP_INFO(node->get_logger(), "====================================================");
    RCLCPP_INFO(node->get_logger(), "  ROS2 Signal Publisher Test - 21 Scenarios");
    RCLCPP_INFO(node->get_logger(), "====================================================");
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "📡 Publisher ready. Waiting 2 seconds for subscribers...");
    std::this_thread::sleep_for(2s);
    
    // 각 시나리오 순차 발행 (3초 간격)
    std::vector<std::string> scenarios = {
        "CONTAINER_OK",
        "SIZE_CHECK_START",
        "SIZE_MEASURE_OK",
        "NO_QR_SIZE_MEASURE_OK",
        "QR_PICKUP",
        "QR_MEASURE_OK",
        "NO_QR_PICKUP",
        "NO_QR_MEASURE_OK",
        "SET_ITEM",
        "SET_ITEM_NORMAL",
        "SET_ITEM_PICKUP",
        "SET_ITEM_SIZE_CHECK_START",
        "SET_ITEM_MEASURE_OK",
        "CLEAR_ITEM",
        "DROP",
        "DEVICE_ERROR",
        "DEVICE_ERROR_CLEAR",
        "CHECK_COMPLETE",
        "NO_QR_CHECK_COMPLETE",
        "SET_ITEM_CHECK_COMPLETE",
        "INVALID_PLACE"
    };
    
    int scenario_count = 1;
    for (const auto& scenario : scenarios) {
        RCLCPP_INFO(node->get_logger(), "");
        RCLCPP_INFO(node->get_logger(), "🔔 [%d/%zu] Publishing: %s", 
                   scenario_count, scenarios.size(), scenario.c_str());
        
        fta_interfaces::msg::SignalCommand msg;
        msg.action_type = scenario;
        msg.priority = 0;  // Default priority
        
        publisher->publish(msg);
        
        scenario_count++;
        std::this_thread::sleep_for(3s);  // 3초 대기 (하드웨어 동작 관찰 시간)
    }
    
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "====================================================");
    RCLCPP_INFO(node->get_logger(), "✅ 전체 시나리오 발행 완료! 총 %d개 시나리오 발행됨", scenario_count - 1);
    RCLCPP_INFO(node->get_logger(), "====================================================");
    
    rclcpp::shutdown();
    return 0;
}
