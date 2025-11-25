cmake_minimum_required(VERSION 3.8)
project(fta_beginner)

# C++17 표준 사용
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 컴파일 경고 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 1. 간단한 Publisher 노드
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

# 2. 간단한 Subscriber 노드
add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)

# 3. Patlite 테스트 노드 (USB 확인용)
add_executable(patlite_test src/patlite_test.cpp)
ament_target_dependencies(patlite_test rclcpp std_msgs)

# 4. Patlite 시나리오 라이브러리 (fta_actuators에서 가져옴)
# 주의: 실제로는 fta_actuators 패키지에 있어야 하지만, 테스트용으로 여기에 포함
add_library(patlite_scenarios
  ../fta_actuators/src/patlite_scenarios.cpp
)
target_include_directories(patlite_scenarios PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../fta_actuators/include>
  $<INSTALL_INTERFACE:include>
)

# 5. 시나리오 테스트 프로그램
add_executable(test_scenarios src/test_scenarios.cpp)
target_link_libraries(test_scenarios patlite_scenarios)
target_include_directories(test_scenarios PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../fta_actuators/include>
  $<INSTALL_INTERFACE:include>
)

# 설치 규칙
install(TARGETS
  simple_publisher
  simple_subscriber
  patlite_test
  test_scenarios
  DESTINATION lib/${PROJECT_NAME})

# 린트 테스트
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
