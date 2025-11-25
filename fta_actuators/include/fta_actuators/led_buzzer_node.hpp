#ifndef FTA_ACTUATORS__LED_BUZZER_NODE_HPP_
#define FTA_ACTUATORS__LED_BUZZER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "fta_interfaces/msg/action_event.hpp"
#include "fta_interfaces/msg/actuator_status.hpp"
#include "fta_actuators/patlite/patlite_driver.hpp"

namespace fta_actuators
{

  // ============================================
  // LED/Buzzer ��� Ŭ����
  // ROS2 ��� Patlite ���� ���
  // ============================================
  class LEDBuzzerNode : public rclcpp::Node
  {
  public:
    explicit LEDBuzzerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~LEDBuzzerNode();

  private:
    // ROS2 Subscriber/Publisher
    rclcpp::Subscription<fta_interfaces::msg::ActionEvent>::SharedPtr action_sub_;
    rclcpp::Publisher<fta_interfaces::msg::ActuatorStatus>::SharedPtr status_pub_;

    // Patlite ����̹�
    std::unique_ptr<PatliteDriver> driver_;

    // �Ķ����
    bool enabled_;
    int volume_;
    std::string device_path_;

    // �ݹ� �Լ�
    void action_callback(const fta_interfaces::msg::ActionEvent::SharedPtr msg);

    // Patlite ���� �Լ�
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

#endif // FTA_ACTUATORS__LED_BUZZER_NODE_HPP_
