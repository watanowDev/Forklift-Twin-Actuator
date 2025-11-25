// ============================================================================
// test_scenarios.cpp
// Patlite 시나리오 테스트 프로그램
//
// 사용법: ros2 run fta_beginner test_scenarios
// ============================================================================

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include "fta_actuators/patlite_scenarios.hpp"

using namespace fta_actuators;
using namespace std::chrono_literals;

void print_header(const std::string &title)
{
    std::cout << "\n";
    std::cout << "========================================" << std::endl;
    std::cout << title << std::endl;
    std::cout << "========================================" << std::endl;
}

void print_command(const std::string &scenario_name, const PatliteCommand &cmd)
{
    std::cout << std::left << std::setw(30) << scenario_name << " | "
              << "LED: " << std::setw(10) << led_color_to_string(cmd.led_color)
              << " " << std::setw(12) << led_pattern_to_string(cmd.led_pattern) << " | "
              << "Buzzer: " << std::setw(12) << buzzer_pattern_to_string(cmd.buzzer_pattern)
              << " x" << cmd.buzzer_count
              << std::endl;
}

void test_all_scenarios()
{
    print_header("전체 시나리오 테스트");

    PatliteScenarioMapper mapper;

    // 모든 시나리오 출력
    std::vector<PatliteScenario> scenarios = {
        PatliteScenario::CONTAINER_OK,
        PatliteScenario::SIZE_CHECK_START,
        PatliteScenario::SIZE_MEASURE_OK,
        PatliteScenario::NO_QR_SIZE_MEASURE_OK,
        PatliteScenario::QR_PICKUP,
        PatliteScenario::QR_MEASURE_OK,
        PatliteScenario::NO_QR_PICKUP,
        PatliteScenario::NO_QR_MEASURE_OK,
        PatliteScenario::SET_ITEM,
        PatliteScenario::SET_ITEM_NORMAL,
        PatliteScenario::SET_ITEM_PICKUP,
        PatliteScenario::SET_ITEM_SIZE_CHECK_START,
        PatliteScenario::SET_ITEM_MEASURE_OK,
        PatliteScenario::CLEAR_ITEM,
        PatliteScenario::DROP,
        PatliteScenario::DEVICE_ERROR,
        PatliteScenario::DEVICE_ERROR_CLEAR,
        PatliteScenario::CHECK_COMPLETE,
        PatliteScenario::NO_QR_CHECK_COMPLETE,
        PatliteScenario::SET_ITEM_CHECK_COMPLETE,
        PatliteScenario::INVALID_PLACE};

    std::cout << "\n총 " << scenarios.size() << "개의 시나리오:\n"
              << std::endl;
    std::cout << std::left << std::setw(30) << "시나리오" << " | LED 설정                   | Buzzer 설정" << std::endl;
    std::cout << std::string(90, '-') << std::endl;

    for (const auto &scenario : scenarios)
    {
        std::string name = scenario_to_string(scenario);
        PatliteCommand cmd = mapper.get_command(scenario);
        print_command(name, cmd);
    }
}

void test_context_functions()
{
    print_header("상황별 제어 함수 테스트");

    PatliteController controller;

    std::cout << "\n[테스트 1] 측정 시작 - 앱 물류 선택" << std::endl;
    controller.start_measuring_buzzer(true, "", false, false);
    std::this_thread::sleep_for(500ms);

    std::cout << "\n[테스트 2] 측정 시작 - QR 코드 있음" << std::endl;
    controller.start_measuring_buzzer(false, "wata-12345", false, false);
    std::this_thread::sleep_for(500ms);

    std::cout << "\n[테스트 3] 측정 시작 - QR 코드 없음" << std::endl;
    controller.start_measuring_buzzer(false, "", false, false);
    std::this_thread::sleep_for(500ms);

    std::cout << "\n[테스트 4] 측정 완료 - 앱 물류 선택" << std::endl;
    controller.finish_measuring_buzzer(true, "", false, false);
    std::this_thread::sleep_for(500ms);

    std::cout << "\n[테스트 5] 측정 완료 - QR 코드 있음" << std::endl;
    controller.finish_measuring_buzzer(false, "wata-12345", false, false);
    std::this_thread::sleep_for(500ms);

    std::cout << "\n[테스트 6] 사이즈 측정 완료 - QR 있음" << std::endl;
    controller.finish_measuring_size("wata-12345", false);
    std::this_thread::sleep_for(500ms);

    std::cout << "\n[테스트 7] 사이즈 측정 완료 - QR 없음" << std::endl;
    controller.finish_measuring_size("", false);
    std::this_thread::sleep_for(500ms);

    std::cout << "\n[테스트 8] 보행자 감지 경고" << std::endl;
    controller.alert_detect_person(false);
    std::this_thread::sleep_for(500ms);
}

void test_string_conversion()
{
    print_header("문자열 변환 테스트");

    PatliteScenarioMapper mapper;

    std::cout << "\n문자열로 시나리오 조회:" << std::endl;

    std::vector<std::string> test_names = {
        "CONTAINER_OK",
        "device_error",
        "Invalid_Place",
        "qr_pickup"};

    for (const auto &name : test_names)
    {
        try
        {
            PatliteCommand cmd = mapper.get_command_by_name(name);
            std::cout << "  ✓ " << name << " -> ";
            print_command("", cmd);
        }
        catch (const std::exception &e)
        {
            std::cout << "  ✗ " << name << " -> ERROR: " << e.what() << std::endl;
        }
    }
}

void show_usage_examples()
{
    print_header("ROS2 사용 예제");

    std::cout << "\n1. ROS2 토픽으로 시나리오 실행:" << std::endl;
    std::cout << "   ros2 topic pub --once /patlite/scenario std_msgs/msg/String \"{data: 'CONTAINER_OK'}\"" << std::endl;
    std::cout << "   ros2 topic pub --once /patlite/scenario std_msgs/msg/String \"{data: 'DEVICE_ERROR'}\"" << std::endl;
    std::cout << "   ros2 topic pub --once /patlite/scenario std_msgs/msg/String \"{data: 'QR_PICKUP'}\"" << std::endl;

    std::cout << "\n2. 상황별 제어 (컨텍스트 기반):" << std::endl;
    std::cout << "   ros2 topic pub --once /patlite/context std_msgs/msg/String " << std::endl;
    std::cout << "     \"{data: '{\\\"action\\\": \\\"start_measuring\\\", \\\"set_item\\\": true, \\\"qr_code\\\": \\\"\\\"}'}\"" << std::endl;

    std::cout << "\n3. 사용 가능한 모든 시나리오:" << std::endl;
    PatliteScenarioMapper mapper;
    auto names = mapper.get_scenario_names();
    int count = 0;
    for (const auto &name : names)
    {
        std::cout << "   - " << name << std::endl;
        if (++count >= 10)
        {
            std::cout << "   ... 외 " << (names.size() - 10) << "개" << std::endl;
            break;
        }
    }
}

void show_color_reference()
{
    print_header("LED 색상 참고표");

    std::cout << "\n상황별 색상 사용:\n"
              << std::endl;
    std::cout << "  🟢 GREEN (초록)   - 정상 작업 (QR 코드 있음)" << std::endl;
    std::cout << "  🟣 PURPLE (보라)  - QR 코드 없이 작업" << std::endl;
    std::cout << "  🔵 SKYBLUE (하늘) - 앱에서 물류 선택" << std::endl;
    std::cout << "  🔴 RED (빨강)     - 에러 또는 경고" << std::endl;
    std::cout << "  ⚪ CLEAR (끄기)   - LED 끄기" << std::endl;

    std::cout << "\n패턴 종류:\n"
              << std::endl;
    std::cout << "  - CONTINUOUS: 연속 점등" << std::endl;
    std::cout << "  - PATTERN2~6: 다양한 점멸 패턴" << std::endl;
    std::cout << "  - OFF: 꺼짐" << std::endl;
}

int main(int argc, char **argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "Patlite 시나리오 테스트 프로그램" << std::endl;
    std::cout << "========================================" << std::endl;

    // 명령줄 인자 처리
    if (argc > 1)
    {
        std::string command = argv[1];

        if (command == "list" || command == "-l")
        {
            test_all_scenarios();
        }
        else if (command == "test" || command == "-t")
        {
            test_context_functions();
        }
        else if (command == "string" || command == "-s")
        {
            test_string_conversion();
        }
        else if (command == "examples" || command == "-e")
        {
            show_usage_examples();
        }
        else if (command == "colors" || command == "-c")
        {
            show_color_reference();
        }
        else if (command == "help" || command == "-h")
        {
            std::cout << "\n사용법: test_scenarios [command]\n"
                      << std::endl;
            std::cout << "Commands:" << std::endl;
            std::cout << "  list, -l       전체 시나리오 목록 출력" << std::endl;
            std::cout << "  test, -t       상황별 제어 함수 테스트" << std::endl;
            std::cout << "  string, -s     문자열 변환 테스트" << std::endl;
            std::cout << "  examples, -e   ROS2 사용 예제 출력" << std::endl;
            std::cout << "  colors, -c     색상 참고표 출력" << std::endl;
            std::cout << "  help, -h       도움말 출력" << std::endl;
            std::cout << "\n옵션 없이 실행하면 모든 테스트를 수행합니다." << std::endl;
        }
        else
        {
            std::cerr << "알 수 없는 명령어: " << command << std::endl;
            std::cerr << "도움말: test_scenarios help" << std::endl;
            return 1;
        }
    }
    else
    {
        // 모든 테스트 실행
        test_all_scenarios();
        test_context_functions();
        test_string_conversion();
        show_usage_examples();
        show_color_reference();
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "테스트 완료!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
