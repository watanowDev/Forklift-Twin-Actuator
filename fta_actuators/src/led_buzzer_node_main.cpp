#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "fta_actuators/led_buzzer_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<fta_actuators::LEDBuzzerNode>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
