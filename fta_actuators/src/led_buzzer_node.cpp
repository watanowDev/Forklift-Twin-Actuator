#include "fta_actuators/led_buzzer_node.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

namespace fta_actuators
{

  // ============================================
  // ������
  // ============================================
  LEDBuzzerNode::LEDBuzzerNode(const rclcpp::NodeOptions &options)
      : Node("led_buzzer_node", options),
        enabled_(true),
        volume_(50)
  {
    // �Ķ���� ����
    this->declare_parameter("enabled", true);
    this->declare_parameter("volume", 50);
    this->declare_parameter("device_path", "");

    // �Ķ���� �б�
    enabled_ = this->get_parameter("enabled").as_bool();
    volume_ = this->get_parameter("volume").as_int();
    device_path_ = this->get_parameter("device_path").as_string();

    RCLCPP_INFO(this->get_logger(), "LED/Buzzer Node ����");
    RCLCPP_INFO(this->get_logger(), "  - enabled: %s", enabled_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  - volume: %d", volume_);

    if (!enabled_)
    {
      RCLCPP_WARN(this->get_logger(), "LED/Buzzer ��尡 ��Ȱ��ȭ�Ǿ����ϴ�");
      return;
    }

    // Patlite ����̹� �ʱ�ȭ
    driver_ = std::make_unique<PatliteDriver>();
    if (!initialize_device())
    {
      RCLCPP_ERROR(this->get_logger(), "Patlite ����̽� �ʱ�ȭ ����");
      publish_status("error", "Failed to initialize Patlite device", 1);
      return;
    }

    // ROS2 ������ ���� (QoS1 - Reliable)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    action_sub_ = this->create_subscription<fta_interfaces::msg::ActionEvent>(
        "/actions/event",
        qos,
        std::bind(&LEDBuzzerNode::action_callback, this, std::placeholders::_1));

    // ROS2 ������ ���� (QoS1 - Reliable)
    status_pub_ = this->create_publisher<fta_interfaces::msg::ActuatorStatus>(
        "/actuators/status",
        qos);

    // �ʱ� ���� ����
    publish_status("idle", "LED/Buzzer node initialized");

    RCLCPP_INFO(this->get_logger(), "LED/Buzzer ��� �ʱ�ȭ �Ϸ�");
  }

  // ============================================
  // �Ҹ���
  // ============================================
  LEDBuzzerNode::~LEDBuzzerNode()
  {
    cleanup_device();
    RCLCPP_INFO(this->get_logger(), "LED/Buzzer ��� ����");
  }

  // ============================================
  // Device initialization
  // ============================================
  bool LEDBuzzerNode::initialize_device()
  {
    if (!driver_->initialize())
    {
      return false;
    }

    // Initial LED status: Green CONTINUOUS
    driver_->set_light(LEDColor::GREEN, LEDPattern::CONTINUOUS);
    return true;
  }

  // ============================================
  // ����̽� ����
  // ============================================
  void LEDBuzzerNode::cleanup_device()
  {
    if (driver_)
    {
      // LED/���� ����
      driver_->set_light(LEDColor::CLEAR, LEDPattern::OFF);
      driver_->set_buzzer(BuzzerPattern::STOP, 0, 0);
      driver_->close_device();
    }
  }

  // ============================================
  // �׼� �̺�Ʈ �ݹ�
  // ============================================
  void LEDBuzzerNode::action_callback(const fta_interfaces::msg::ActionEvent::SharedPtr msg)
  {
    // ���߿����� Ÿ�� Ȯ��
    if (msg->actuator_type != "led_buzzer")
    {
      return; // �ٸ� ���߿����Ϳ� �޽����� ����
    }

    RCLCPP_INFO(this->get_logger(), "Action received: %s", msg->action.c_str());

    // �׼� ó��
    bool success = handle_led_buzzer_action(msg->action, msg->parameters);

    if (success)
    {
      publish_status("active", "Action executed: " + msg->action);
    }
    else
    {
      publish_status("error", "Failed to execute action: " + msg->action, 2);
    }
  }

  // ============================================
  // LED/Buzzer �׼� ó��
  // ============================================
  bool LEDBuzzerNode::handle_led_buzzer_action(
      const std::string &action,
      const std::string &parameters)
  {
    if (!driver_ || !driver_->is_connected())
    {
      RCLCPP_ERROR(this->get_logger(), "����̽��� ������� ����");
      return false;
    }

    try
    {
      // JSON �Ķ���� �Ľ�
      auto params = json::parse(parameters);

      if (action == "set_light")
      {
        // LED ����
        std::string color_str = params.value("color", "clear");
        std::string pattern_str = params.value("pattern", "off");

        LEDColor color = PatliteDriver::parse_color(color_str);
        LEDPattern pattern = PatliteDriver::parse_led_pattern(pattern_str);

        return driver_->set_light(color, pattern);
      }
      else if (action == "set_buzzer")
      {
        // ���� ����
        std::string pattern_str = params.value("pattern", "stop");
        int count = params.value("count", 1);

        BuzzerPattern pattern = PatliteDriver::parse_buzzer_pattern(pattern_str);

        return driver_->set_buzzer(pattern, volume_, count);
      }
      else if (action == "set_all")
      {
        // LED + ���� ���� ����
        std::string color_str = params.value("color", "clear");
        std::string led_pattern_str = params.value("led_pattern", "off");
        std::string buzzer_pattern_str = params.value("buzzer_pattern", "stop");
        int count = params.value("count", 0);

        LEDColor color = PatliteDriver::parse_color(color_str);
        LEDPattern led_pattern = PatliteDriver::parse_led_pattern(led_pattern_str);
        BuzzerPattern buzzer_pattern = PatliteDriver::parse_buzzer_pattern(buzzer_pattern_str);

        bool led_ok = driver_->set_light(color, led_pattern);
        bool buzzer_ok = driver_->set_buzzer(buzzer_pattern, volume_, count);

        return led_ok && buzzer_ok;
      }
      else if (action == "clear")
      {
        // ��� ����
        bool led_ok = driver_->set_light(LEDColor::CLEAR, LEDPattern::OFF);
        bool buzzer_ok = driver_->set_buzzer(BuzzerPattern::OFF, 0, 0);
        return led_ok && buzzer_ok;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "�� �� ���� �׼�: %s", action.c_str());
        return false;
      }
    }
    catch (const json::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "JSON �Ľ� ����: %s", e.what());
      return false;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "���� �߻�: %s", e.what());
      return false;
    }
  }

  // ============================================
  // ���� ����
  // ============================================
  void LEDBuzzerNode::publish_status(
      const std::string &status,
      const std::string &message,
      int error_code)
  {
    if (!status_pub_)
    {
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

} // namespace fta_actuators

// ============================================
// ���� �Լ�
// ============================================
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fta_actuators::LEDBuzzerNode)
