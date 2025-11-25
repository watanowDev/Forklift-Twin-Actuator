cmake_minimum_required(VERSION 3.8)
project(fta_actuators)

# C++17 ǥ�� ���
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 ��Ű�� ã��
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(fta_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

# libusb ã�� (USB ��� ����)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# ��Ŭ��� ���丮
include_directories(
  include
  ${LIBUSB_INCLUDE_DIRS}
)

# ============================================
# ���̺귯��: Patlite USB ����̹�
# ============================================
add_library(patlite_usb_driver SHARED
  src/patlite/patlite_usb_driver.cpp
)

target_link_libraries(patlite_usb_driver
  ${LIBUSB_LIBRARIES}
)

target_include_directories(patlite_usb_driver PUBLIC
  ${LIBUSB_INCLUDE_DIRS}
)

# ============================================
# ���̺귯��: Patlite �ó����
# ============================================
add_library(patlite_scenarios SHARED
  src/patlite/patlite_scenarios.cpp
)

target_link_libraries(patlite_scenarios
  patlite_usb_driver
)

ament_target_dependencies(patlite_scenarios
  rclcpp
)

# ============================================
# ���̺귯��: Patlite ����̹� (���� ȣȯ��)
# ============================================
add_library(patlite_driver SHARED
  src/patlite/patlite_driver.cpp
)

target_link_libraries(patlite_driver
  patlite_usb_driver
  patlite_scenarios
)

ament_target_dependencies(patlite_driver
  rclcpp
)

# Windows/Linux ȣȯ
if(WIN32)
  # Windows: �߰� ���̺귯�� ���� (LoadLibrary ���)
else()
  # Linux: dl ���̺귯�� �ʿ� (dlopen ���)
  target_link_libraries(patlite_driver dl)
endif()

# ============================================
# ������Ʈ: LED Buzzer ���
# ============================================
add_library(led_buzzer_node_component SHARED
  src/led_buzzer_node.cpp
)

target_link_libraries(led_buzzer_node_component
  patlite_driver
)

ament_target_dependencies(led_buzzer_node_component
  rclcpp
  rclcpp_components
  fta_interfaces
  nlohmann_json
)

rclcpp_components_register_nodes(led_buzzer_node_component "fta_actuators::LEDBuzzerNode")

# ============================================
# ���� ����: LED Buzzer ���
# ============================================
add_executable(led_buzzer_node
  src/led_buzzer_node_main.cpp
)

target_link_libraries(led_buzzer_node
  led_buzzer_node_component
  patlite_driver
)

ament_target_dependencies(led_buzzer_node
  rclcpp
  fta_interfaces
)

# ============================================
# ���� ����: �׽�Ʈ �׼� ������
# ============================================
add_executable(test_action_publisher
  src/test_action_publisher.cpp
)

ament_target_dependencies(test_action_publisher
  rclcpp
  fta_interfaces
)

# ============================================
# ��ġ
# ============================================

# ���̺귯�� ��ġ
install(TARGETS
  patlite_usb_driver
  patlite_scenarios
  patlite_driver
  led_buzzer_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ���� ���� ��ġ
install(TARGETS
  led_buzzer_node
  test_action_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# ��� ���� ��ġ
install(DIRECTORY
  include/
  DESTINATION include
)

# Launch ���� ��ġ
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Config ���� ��ġ
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# ============================================
# �׽�Ʈ
# ============================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
