#ifndef FTA_ACTUATORS__PATLITE_LED_BUZZER_NODE_HPP_
#define FTA_ACTUATORS__PATLITE_LED_BUZZER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "fta_interfaces/msg/action_event.hpp"
#include "fta_interfaces/msg/actuator_status.hpp"
#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_driver.hpp"

namespace fta_actuators
{

  // ============================================
  // PatliteLedBuzzer LED/Buzzer 노드 클래스
  // ROS2 노드로 PatliteLedBuzzer 장치 제어
  // ============================================
  class PatliteLedBuzzerNode : public rclcpp::Node
  {
  public:
    explicit PatliteLedBuzzerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~PatliteLedBuzzerNode();

  private:
    // ROS2 Subscriber/Publisher
    rclcpp::Subscription<fta_interfaces::msg::ActionEvent>::SharedPtr action_sub_;
    rclcpp::Publisher<fta_interfaces::msg::ActuatorStatus>::SharedPtr status_pub_;

    // PatliteLedBuzzer ����̹�
    std::unique_ptr<PatliteLedBuzzerDriver> driver_;

    // �Ķ����
    bool enabled_;
    int volume_;
    std::string device_path_;

    // �ݹ� �Լ�
    void action_callback(const fta_interfaces::msg::ActionEvent::SharedPtr msg);

    // PatliteLedBuzzer ���� �Լ�
    bool initialize_device();
    void cleanup_device();
    bool handle_led_buzzer_action(const std::string &action, const std::string &parameters);

    // ���� ����
    void publish_status(
        const std::string &status,
        const std::string &message,
        int error_code = 0);
  };

} // namespace fta_actuators

#endif  // FTA_ACTUATORS__PATLITE_LED_BUZZER_NODE_HPP_

