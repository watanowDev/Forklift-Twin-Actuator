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
# ���̺귯��: Patlite LED Buzzer USB ����̹�
# ============================================
add_library(patlite_led_buzzer_usb_driver SHARED
  src/patlite_led_buzzer/patlite_led_buzzer_led_buzzer_usb_driver.cpp
)

target_link_libraries(patlite_led_buzzer_usb_driver
  ${LIBUSB_LIBRARIES}
)

target_include_directories(patlite_led_buzzer_usb_driver PUBLIC
  ${LIBUSB_INCLUDE_DIRS}
)

# ============================================
# 라이브러리: Patlite Controller (C# 포트)
# ============================================
add_library(patlite_led_buzzer_controller SHARED
  src/patlite_led_buzzer/patlite_led_buzzer_led_buzzer_controller.cpp
)

ament_target_dependencies(patlite_led_buzzer_controller
  rclcpp
)

# ============================================
# 라이브러리: Patlite Hardware Factory
# ============================================
add_library(patlite_led_buzzer_hardware_factory SHARED
  src/patlite_led_buzzer/patlite_led_buzzer_led_buzzer_hardware_factory.cpp
  src/patlite_led_buzzer/patlite_led_buzzer_led_buzzer_nedll_driver.cpp
  src/patlite_led_buzzer/patlite_led_buzzer_led_buzzer_usb_direct_driver.cpp
)

ament_target_dependencies(patlite_led_buzzer_hardware_factory
  rclcpp
)

# libusb 및 dl 라이브러리 링크
target_link_libraries(patlite_led_buzzer_hardware_factory
  ${LIBUSB_LIBRARIES}
)

if(UNIX AND NOT APPLE)
  target_link_libraries(patlite_led_buzzer_hardware_factory dl)
endif()

# ============================================
# ���̺귯��: Patlite �ó����
# ============================================
add_library(patlite_led_buzzer_scenarios SHARED
  src/patlite_led_buzzer/patlite_led_buzzer_led_buzzer_scenarios.cpp
)

target_link_libraries(patlite_led_buzzer_scenarios
  patlite_led_buzzer_usb_driver
  patlite_led_buzzer_controller
)

ament_target_dependencies(patlite_led_buzzer_scenarios
  rclcpp
)

# ============================================
# ���̺귯��: Patlite ����̹� (���� ȣȯ��)
# ============================================
add_library(patlite_led_buzzer_driver SHARED
  src/patlite_led_buzzer/patlite_led_buzzer_led_buzzer_driver.cpp
)

target_link_libraries(patlite_led_buzzer_driver
  patlite_led_buzzer_usb_driver
  patlite_led_buzzer_scenarios
)

ament_target_dependencies(patlite_led_buzzer_driver
  rclcpp
)

# Windows/Linux ȣȯ
if(WIN32)
  # Windows: �߰� ���̺귯�� ���� (LoadLibrary ���)
else()
  # Linux: dl ���̺귯�� �ʿ� (dlopen ���)
  target_link_libraries(patlite_led_buzzer_driver dl)
endif()

# ============================================
# 컴포넌트: Patlite 노드
# ============================================
add_library(patlite_led_buzzer_node_component SHARED
  src/patlite_led_buzzer/patlite_led_buzzer_led_buzzer_node.cpp
)

target_link_libraries(patlite_led_buzzer_node_component
  patlite_led_buzzer_driver
)

ament_target_dependencies(patlite_led_buzzer_node_component
  rclcpp
  rclcpp_components
  fta_interfaces
  nlohmann_json
)

rclcpp_components_register_nodes(patlite_led_buzzer_node_component "fta_actuators::PatliteLedBuzzerNode")

# ============================================
# 독립 실행: Patlite 노드
# ============================================
add_executable(patlite_led_buzzer_node
  src/patlite_led_buzzer/patlite_led_buzzer_led_buzzer_node_main.cpp
)

target_link_libraries(patlite_led_buzzer_node
  patlite_led_buzzer_node_component
  patlite_led_buzzer_driver
)

ament_target_dependencies(patlite_led_buzzer_node
  rclcpp
  fta_interfaces
)



# ============================================
# 실행 파일: 신호 제어 노드 (ROS2 구독)
# ============================================
add_executable(signal_controller_node
  src/signal_controller_node.cpp
)

target_link_libraries(signal_controller_node
  patlite_led_buzzer_hardware_factory
  ${LIBUSB_LIBRARIES}
)

ament_target_dependencies(signal_controller_node
  rclcpp
  fta_interfaces
)

# ============================================
# 테스트: ROS2 신호 발행자 (통합 테스트용)
# ============================================
add_executable(test_ros2_signal_publisher
  src/tests/test_ros2_signal_publisher.cpp
)

ament_target_dependencies(test_ros2_signal_publisher
  rclcpp
  fta_interfaces
)

# ============================================
# ��ġ
# ============================================

# 라이브러리 설치
install(TARGETS
  patlite_led_buzzer_usb_driver
  patlite_led_buzzer_controller
  patlite_led_buzzer_hardware_factory
  patlite_led_buzzer_scenarios
  patlite_led_buzzer_driver
  patlite_led_buzzer_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 실행 파일 및 테스트 설치
install(TARGETS
  patlite_led_buzzer_node
  signal_controller_node
  test_ros2_signal_publisher
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
