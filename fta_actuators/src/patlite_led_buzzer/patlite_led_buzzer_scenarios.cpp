// ============================================================================
// patlite_scenarios.cpp
// PatliteLedBuzzer LED/Buzzer 상황별 제어 시나리오 구현
// ============================================================================

#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_scenarios.hpp"
#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_driver.hpp"
#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace fta_actuators
{

    // ============================================
    // PatliteLedBuzzerScenarioMapper 구현
    // ============================================

    PatliteLedBuzzerScenarioMapper::PatliteLedBuzzerScenarioMapper()
    {
        initialize_scenarios();
    }

    void PatliteLedBuzzerScenarioMapper::initialize_scenarios()
    {
        // C# StatusService_WATA.cs의 Pattlite_Buzzer_LED 함수 내용을 그대로 매핑

        // CONTAINER_OK: 초록색 연속 + 부저 연속 1회
        scenario_map_[PatliteLedBuzzerScenario::CONTAINER_OK] =
            PatliteLedBuzzerCommand(LEDColor::GREEN, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1);

        // SIZE_CHECK_START: 초록색 Pattern3 + 부저 Pattern2 1회
        scenario_map_[PatliteLedBuzzerScenario::SIZE_CHECK_START] =
            PatliteLedBuzzerCommand(LEDColor::GREEN, LEDPattern::PATTERN3, BuzzerPattern::PATTERN2, 1);

        // SIZE_MEASURE_OK: 초록색 Pattern6 + 부저 Pattern1 1회
        scenario_map_[PatliteLedBuzzerScenario::SIZE_MEASURE_OK] =
            PatliteLedBuzzerCommand(LEDColor::GREEN, LEDPattern::PATTERN6, BuzzerPattern::PATTERN1, 1);

        // NO_QR_SIZE_MEASURE_OK: 보라색 Pattern6 + 부저 Pattern1 1회
        scenario_map_[PatliteLedBuzzerScenario::NO_QR_SIZE_MEASURE_OK] =
            PatliteLedBuzzerCommand(LEDColor::PURPLE, LEDPattern::PATTERN6, BuzzerPattern::PATTERN1, 1);

        // QR_PICKUP: 초록색 Pattern3 + 부저 Pattern2 1회
        scenario_map_[PatliteLedBuzzerScenario::QR_PICKUP] =
            PatliteLedBuzzerCommand(LEDColor::GREEN, LEDPattern::PATTERN3, BuzzerPattern::PATTERN2, 1);

        // QR_MEASURE_OK: 초록색 연속 + 부저 연속 1회
        scenario_map_[PatliteLedBuzzerScenario::QR_MEASURE_OK] =
            PatliteLedBuzzerCommand(LEDColor::GREEN, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1);

        // NO_QR_PICKUP: 보라색 Pattern3 + 부저 Pattern2 1회
        scenario_map_[PatliteLedBuzzerScenario::NO_QR_PICKUP] =
            PatliteLedBuzzerCommand(LEDColor::PURPLE, LEDPattern::PATTERN3, BuzzerPattern::PATTERN2, 1);

        // NO_QR_MEASURE_OK: 보라색 연속 + 부저 연속 1회
        scenario_map_[PatliteLedBuzzerScenario::NO_QR_MEASURE_OK] =
            PatliteLedBuzzerCommand(LEDColor::PURPLE, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1);

        // SET_ITEM: 하늘색(Cyan) 연속 + 부저 연속 1회
        scenario_map_[PatliteLedBuzzerScenario::SET_ITEM] =
            PatliteLedBuzzerCommand(LEDColor::SKYBLUE, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1);

        // SET_ITEM_NORMAL: 하늘색(Cyan) 연속 + 부저 OFF
        scenario_map_[PatliteLedBuzzerScenario::SET_ITEM_NORMAL] =
            PatliteLedBuzzerCommand(LEDColor::SKYBLUE, LEDPattern::CONTINUOUS, BuzzerPattern::OFF, 1);

        // SET_ITEM_PICKUP: 하늘색(Cyan) Pattern3 + 부저 Pattern2 1회
        scenario_map_[PatliteLedBuzzerScenario::SET_ITEM_PICKUP] =
            PatliteLedBuzzerCommand(LEDColor::SKYBLUE, LEDPattern::PATTERN3, BuzzerPattern::PATTERN2, 1);

        // SET_ITEM_SIZE_CHECK_START: 하늘색(Cyan) Pattern3 + 부저 Pattern2 1회
        scenario_map_[PatliteLedBuzzerScenario::SET_ITEM_SIZE_CHECK_START] =
            PatliteLedBuzzerCommand(LEDColor::SKYBLUE, LEDPattern::PATTERN3, BuzzerPattern::PATTERN2, 1);

        // SET_ITEM_MEASURE_OK: 하늘색(Cyan) 연속 + 부저 연속 1회
        scenario_map_[PatliteLedBuzzerScenario::SET_ITEM_MEASURE_OK] =
            PatliteLedBuzzerCommand(LEDColor::SKYBLUE, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1);

        // CLEAR_ITEM: 하늘색(Cyan) 연속 + 부저 연속 1회
        scenario_map_[PatliteLedBuzzerScenario::CLEAR_ITEM] =
            PatliteLedBuzzerCommand(LEDColor::SKYBLUE, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1);

        // DROP: 초록색 연속 + 부저 OFF
        scenario_map_[PatliteLedBuzzerScenario::DROP] =
            PatliteLedBuzzerCommand(LEDColor::GREEN, LEDPattern::CONTINUOUS, BuzzerPattern::OFF, 1);

        // DEVICE_ERROR: 빨간색 Pattern2 + 부저 Pattern4 1회
        scenario_map_[PatliteLedBuzzerScenario::DEVICE_ERROR] =
            PatliteLedBuzzerCommand(LEDColor::RED, LEDPattern::PATTERN2, BuzzerPattern::PATTERN4, 1);

        // DEVICE_ERROR_CLEAR: 초록색 연속 + 부저 연속 1회
        scenario_map_[PatliteLedBuzzerScenario::DEVICE_ERROR_CLEAR] =
            PatliteLedBuzzerCommand(LEDColor::GREEN, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1);

        // CHECK_COMPLETE: 초록색 연속 + 부저 연속 1회
        scenario_map_[PatliteLedBuzzerScenario::CHECK_COMPLETE] =
            PatliteLedBuzzerCommand(LEDColor::GREEN, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1);

        // NO_QR_CHECK_COMPLETE: 보라색 연속 + 부저 연속 1회
        scenario_map_[PatliteLedBuzzerScenario::NO_QR_CHECK_COMPLETE] =
            PatliteLedBuzzerCommand(LEDColor::PURPLE, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1);

        // SET_ITEM_CHECK_COMPLETE: 하늘색(Cyan) 연속 + 부저 연속 1회
        scenario_map_[PatliteLedBuzzerScenario::SET_ITEM_CHECK_COMPLETE] =
            PatliteLedBuzzerCommand(LEDColor::SKYBLUE, LEDPattern::CONTINUOUS, BuzzerPattern::CONTINUOUS, 1);

        // INVALID_PLACE: 빨간색 Pattern6 + 부저 Pattern3 1회
        scenario_map_[PatliteLedBuzzerScenario::INVALID_PLACE] =
            PatliteLedBuzzerCommand(LEDColor::RED, LEDPattern::PATTERN6, BuzzerPattern::PATTERN3, 1);

        // 이름 매핑 초기화
        name_to_scenario_["CONTAINER_OK"] = PatliteLedBuzzerScenario::CONTAINER_OK;
        name_to_scenario_["SIZE_CHECK_START"] = PatliteLedBuzzerScenario::SIZE_CHECK_START;
        name_to_scenario_["SIZE_MEASURE_OK"] = PatliteLedBuzzerScenario::SIZE_MEASURE_OK;
        name_to_scenario_["NO_QR_SIZE_MEASURE_OK"] = PatliteLedBuzzerScenario::NO_QR_SIZE_MEASURE_OK;
        name_to_scenario_["QR_PICKUP"] = PatliteLedBuzzerScenario::QR_PICKUP;
        name_to_scenario_["QR_MEASURE_OK"] = PatliteLedBuzzerScenario::QR_MEASURE_OK;
        name_to_scenario_["NO_QR_PICKUP"] = PatliteLedBuzzerScenario::NO_QR_PICKUP;
        name_to_scenario_["NO_QR_MEASURE_OK"] = PatliteLedBuzzerScenario::NO_QR_MEASURE_OK;
        name_to_scenario_["SET_ITEM"] = PatliteLedBuzzerScenario::SET_ITEM;
        name_to_scenario_["SET_ITEM_NORMAL"] = PatliteLedBuzzerScenario::SET_ITEM_NORMAL;
        name_to_scenario_["SET_ITEM_PICKUP"] = PatliteLedBuzzerScenario::SET_ITEM_PICKUP;
        name_to_scenario_["SET_ITEM_SIZE_CHECK_START"] = PatliteLedBuzzerScenario::SET_ITEM_SIZE_CHECK_START;
        name_to_scenario_["SET_ITEM_MEASURE_OK"] = PatliteLedBuzzerScenario::SET_ITEM_MEASURE_OK;
        name_to_scenario_["CLEAR_ITEM"] = PatliteLedBuzzerScenario::CLEAR_ITEM;
        name_to_scenario_["DROP"] = PatliteLedBuzzerScenario::DROP;
        name_to_scenario_["DEVICE_ERROR"] = PatliteLedBuzzerScenario::DEVICE_ERROR;
        name_to_scenario_["DEVICE_ERROR_CLEAR"] = PatliteLedBuzzerScenario::DEVICE_ERROR_CLEAR;
        name_to_scenario_["CHECK_COMPLETE"] = PatliteLedBuzzerScenario::CHECK_COMPLETE;
        name_to_scenario_["NO_QR_CHECK_COMPLETE"] = PatliteLedBuzzerScenario::NO_QR_CHECK_COMPLETE;
        name_to_scenario_["SET_ITEM_CHECK_COMPLETE"] = PatliteLedBuzzerScenario::SET_ITEM_CHECK_COMPLETE;
        name_to_scenario_["INVALID_PLACE"] = PatliteLedBuzzerScenario::INVALID_PLACE;
    }

    PatliteLedBuzzerCommand PatliteLedBuzzerScenarioMapper::get_command(PatliteLedBuzzerScenario scenario) const
    {
        auto it = scenario_map_.find(scenario);
        if (it != scenario_map_.end())
        {
            return it->second;
        }

        // 기본값 반환 (초록색 연속 점등)
        return PatliteLedBuzzerCommand(LEDColor::GREEN, LEDPattern::CONTINUOUS, BuzzerPattern::OFF, 1);
    }

    PatliteLedBuzzerCommand PatliteLedBuzzerScenarioMapper::get_command_by_name(const std::string &scenario_name) const
    {
        // 대소문자 구분 없이 검색하기 위해 대문자로 변환
        std::string upper_name = scenario_name;
        std::transform(upper_name.begin(), upper_name.end(), upper_name.begin(), ::toupper);

        auto it = name_to_scenario_.find(upper_name);
        if (it != name_to_scenario_.end())
        {
            return get_command(it->second);
        }

        throw std::invalid_argument("Unknown scenario name: " + scenario_name);
    }

    std::vector<std::string> PatliteLedBuzzerScenarioMapper::get_scenario_names() const
    {
        std::vector<std::string> names;
        for (const auto &pair : name_to_scenario_)
        {
            names.push_back(pair.first);
        }
        return names;
    }

    // ============================================
    // PatliteLedBuzzerController 구현
    // ============================================

    PatliteLedBuzzerController::PatliteLedBuzzerController()
    {
        // 생성자
    }

    void PatliteLedBuzzerController::execute_scenario(PatliteLedBuzzerScenario scenario)
    {
        PatliteLedBuzzerCommand command = mapper_.get_command(scenario);
        execute_command(command);
    }

    void PatliteLedBuzzerController::execute_command(const PatliteLedBuzzerCommand &command)
    {
        // TODO: 실제 NeUsbController 또는 libusb를 사용한 하드웨어 제어
        // 현재는 로그만 출력

        // 나중에 여기서 실제 USB 통신 구현
        // NeUsbController::NE_SetLight(command.led_color, command.led_pattern);
        // NeUsbController::NE_SetBuz(command.buzzer_pattern, volume, command.buzzer_count);
    }

    // C#의 StartMeasuringBuzzer 함수와 동일
    void PatliteLedBuzzerController::start_measuring_buzzer(
        bool set_item,
        const std::string &qr_code,
        bool is_error,
        bool func_off)
    {
        if (func_off || is_error)
        {
            return; // 기능 OFF이거나 에러 상태면 아무것도 하지 않음
        }

        if (set_item)
        {
            // 앱에서 물류 선택한 경우
            execute_scenario(PatliteLedBuzzerScenario::SET_ITEM_PICKUP);
        }
        else if (!qr_code.empty())
        {
            // QR 코드가 있는 경우
            execute_scenario(PatliteLedBuzzerScenario::QR_PICKUP);
        }
        else
        {
            // QR 코드가 없는 경우
            execute_scenario(PatliteLedBuzzerScenario::NO_QR_PICKUP);
        }
    }

    // C#의 FinishMeasuringBuzzer 함수와 동일
    void PatliteLedBuzzerController::finish_measuring_buzzer(
        bool set_item,
        const std::string &qr_code,
        bool is_error,
        bool func_off)
    {
        if (func_off || is_error)
        {
            return;
        }

        if (set_item)
        {
            execute_scenario(PatliteLedBuzzerScenario::SET_ITEM_MEASURE_OK);
        }
        else if (!qr_code.empty())
        {
            execute_scenario(PatliteLedBuzzerScenario::QR_MEASURE_OK);
        }
        else
        {
            execute_scenario(PatliteLedBuzzerScenario::NO_QR_MEASURE_OK);
        }
    }

    // C#의 FinishMeasuringSize 함수와 동일
    void PatliteLedBuzzerController::finish_measuring_size(
        const std::string &qr_code,
        bool func_off)
    {
        if (func_off)
        {
            return;
        }

        if (!qr_code.empty())
        {
            execute_scenario(PatliteLedBuzzerScenario::SIZE_MEASURE_OK);
        }
        else
        {
            execute_scenario(PatliteLedBuzzerScenario::NO_QR_SIZE_MEASURE_OK);
        }
    }

    // C#의 AlertDetectPerson 함수와 동일
    void PatliteLedBuzzerController::alert_detect_person(bool func_off)
    {
        if (func_off)
        {
            return;
        }

        execute_scenario(PatliteLedBuzzerScenario::INVALID_PLACE);
    }

    // C#의 CheckExceptionBuzzer 함수와 동일
    void PatliteLedBuzzerController::check_exception_buzzer(
        bool set_item,
        const std::string &qr_code,
        bool func_off)
    {
        if (func_off)
        {
            return;
        }

        // 앱 물류 선택 X, QR 코드 X
        if (!set_item && qr_code.empty())
        {
            // TODO: Speaker 이벤트 발행 (qr_check_error)
            // _eventAggregator.GetEvent<SpeakerInfoEvent>().Publish(ePlayInfoSpeaker.qr_check_error);
        }
    }

    // ============================================
    // 헬퍼 함수 구현
    // ============================================

    std::string led_color_to_string(LEDColor color)
    {
        switch (color)
        {
        case LEDColor::CLEAR:
            return "CLEAR";
        case LEDColor::RED:
            return "RED";
        case LEDColor::YELLOW:
            return "YELLOW";
        case LEDColor::LEMON:
            return "LEMON";
        case LEDColor::GREEN:
            return "GREEN";
        case LEDColor::SKYBLUE:
            return "SKYBLUE";
        case LEDColor::BLUE:
            return "BLUE";
        case LEDColor::PURPLE:
            return "PURPLE";
        case LEDColor::PEACHBLOW:
            return "PEACHBLOW";
        case LEDColor::WHITE:
            return "WHITE";
        default:
            return "UNKNOWN";
        }
    }

    std::string led_pattern_to_string(LEDPattern pattern)
    {
        switch (pattern)
        {
        case LEDPattern::OFF:
            return "OFF";
        case LEDPattern::CONTINUOUS:
            return "CONTINUOUS";
        case LEDPattern::PATTERN2:
            return "PATTERN2";
        case LEDPattern::PATTERN3:
            return "PATTERN3";
        case LEDPattern::PATTERN4:
            return "PATTERN4";
        case LEDPattern::PATTERN5:
            return "PATTERN5";
        case LEDPattern::PATTERN6:
            return "PATTERN6";
        default:
            return "UNKNOWN";
        }
    }

    std::string buzzer_pattern_to_string(BuzzerPattern pattern)
    {
        switch (pattern)
        {
        case BuzzerPattern::OFF:
            return "OFF";
        case BuzzerPattern::CONTINUOUS:
            return "CONTINUOUS";
        case BuzzerPattern::PATTERN1:
            return "PATTERN1";
        case BuzzerPattern::PATTERN2:
            return "PATTERN2";
        case BuzzerPattern::PATTERN3:
            return "PATTERN3";
        case BuzzerPattern::PATTERN4:
            return "PATTERN4";
        case BuzzerPattern::PATTERN5:
            return "PATTERN5";
        case BuzzerPattern::PATTERN6:
            return "PATTERN6";
        default:
            return "UNKNOWN";
        }
    }

    std::string scenario_to_string(PatliteLedBuzzerScenario scenario)
    {
        switch (scenario)
        {
        case PatliteLedBuzzerScenario::CONTAINER_OK:
            return "CONTAINER_OK";
        case PatliteLedBuzzerScenario::SIZE_CHECK_START:
            return "SIZE_CHECK_START";
        case PatliteLedBuzzerScenario::SIZE_MEASURE_OK:
            return "SIZE_MEASURE_OK";
        case PatliteLedBuzzerScenario::NO_QR_SIZE_MEASURE_OK:
            return "NO_QR_SIZE_MEASURE_OK";
        case PatliteLedBuzzerScenario::QR_PICKUP:
            return "QR_PICKUP";
        case PatliteLedBuzzerScenario::QR_MEASURE_OK:
            return "QR_MEASURE_OK";
        case PatliteLedBuzzerScenario::NO_QR_PICKUP:
            return "NO_QR_PICKUP";
        case PatliteLedBuzzerScenario::NO_QR_MEASURE_OK:
            return "NO_QR_MEASURE_OK";
        case PatliteLedBuzzerScenario::SET_ITEM:
            return "SET_ITEM";
        case PatliteLedBuzzerScenario::SET_ITEM_NORMAL:
            return "SET_ITEM_NORMAL";
        case PatliteLedBuzzerScenario::SET_ITEM_PICKUP:
            return "SET_ITEM_PICKUP";
        case PatliteLedBuzzerScenario::SET_ITEM_SIZE_CHECK_START:
            return "SET_ITEM_SIZE_CHECK_START";
        case PatliteLedBuzzerScenario::SET_ITEM_MEASURE_OK:
            return "SET_ITEM_MEASURE_OK";
        case PatliteLedBuzzerScenario::CLEAR_ITEM:
            return "CLEAR_ITEM";
        case PatliteLedBuzzerScenario::DROP:
            return "DROP";
        case PatliteLedBuzzerScenario::DEVICE_ERROR:
            return "DEVICE_ERROR";
        case PatliteLedBuzzerScenario::DEVICE_ERROR_CLEAR:
            return "DEVICE_ERROR_CLEAR";
        case PatliteLedBuzzerScenario::CHECK_COMPLETE:
            return "CHECK_COMPLETE";
        case PatliteLedBuzzerScenario::NO_QR_CHECK_COMPLETE:
            return "NO_QR_CHECK_COMPLETE";
        case PatliteLedBuzzerScenario::SET_ITEM_CHECK_COMPLETE:
            return "SET_ITEM_CHECK_COMPLETE";
        case PatliteLedBuzzerScenario::INVALID_PLACE:
            return "INVALID_PLACE";
        default:
            return "UNKNOWN";
        }
    }

    LEDColor string_to_led_color(const std::string &str)
    {
        std::string upper = str;
        std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

        if (upper == "CLEAR")
            return LEDColor::CLEAR;
        if (upper == "RED")
            return LEDColor::RED;
        if (upper == "YELLOW")
            return LEDColor::YELLOW;
        if (upper == "LEMON")
            return LEDColor::LEMON;
        if (upper == "GREEN")
            return LEDColor::GREEN;
        if (upper == "SKYBLUE" || upper == "CYAN")
            return LEDColor::SKYBLUE;
        if (upper == "BLUE")
            return LEDColor::BLUE;
        if (upper == "PURPLE")
            return LEDColor::PURPLE;
        if (upper == "PEACHBLOW")
            return LEDColor::PEACHBLOW;
        if (upper == "WHITE")
            return LEDColor::WHITE;

        throw std::invalid_argument("Unknown LED color: " + str);
    }

    LEDPattern string_to_led_pattern(const std::string &str)
    {
        std::string upper = str;
        std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

        if (upper == "OFF")
            return LEDPattern::OFF;
        if (upper == "CONTINUOUS")
            return LEDPattern::CONTINUOUS;
        if (upper == "PATTERN2")
            return LEDPattern::PATTERN2;
        if (upper == "PATTERN3")
            return LEDPattern::PATTERN3;
        if (upper == "PATTERN4")
            return LEDPattern::PATTERN4;
        if (upper == "PATTERN5")
            return LEDPattern::PATTERN5;
        if (upper == "PATTERN6")
            return LEDPattern::PATTERN6;

        throw std::invalid_argument("Unknown LED pattern: " + str);
    }

    BuzzerPattern string_to_buzzer_pattern(const std::string &str)
    {
        std::string upper = str;
        std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

        if (upper == "OFF")
            return BuzzerPattern::OFF;
        if (upper == "CONTINUOUS")
            return BuzzerPattern::CONTINUOUS;
        if (upper == "PATTERN1")
            return BuzzerPattern::PATTERN1;
        if (upper == "PATTERN2")
            return BuzzerPattern::PATTERN2;
        if (upper == "PATTERN3")
            return BuzzerPattern::PATTERN3;
        if (upper == "PATTERN4")
            return BuzzerPattern::PATTERN4;
        if (upper == "PATTERN5")
            return BuzzerPattern::PATTERN5;
        if (upper == "PATTERN6")
            return BuzzerPattern::PATTERN6;

        throw std::invalid_argument("Unknown buzzer pattern: " + str);
    }

    PatliteLedBuzzerScenario string_to_scenario(const std::string &str)
    {
        std::string upper = str;
        std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

        if (upper == "CONTAINER_OK")
            return PatliteLedBuzzerScenario::CONTAINER_OK;
        if (upper == "SIZE_CHECK_START")
            return PatliteLedBuzzerScenario::SIZE_CHECK_START;
        if (upper == "SIZE_MEASURE_OK")
            return PatliteLedBuzzerScenario::SIZE_MEASURE_OK;
        if (upper == "NO_QR_SIZE_MEASURE_OK")
            return PatliteLedBuzzerScenario::NO_QR_SIZE_MEASURE_OK;
        if (upper == "QR_PICKUP")
            return PatliteLedBuzzerScenario::QR_PICKUP;
        if (upper == "QR_MEASURE_OK")
            return PatliteLedBuzzerScenario::QR_MEASURE_OK;
        if (upper == "NO_QR_PICKUP")
            return PatliteLedBuzzerScenario::NO_QR_PICKUP;
        if (upper == "NO_QR_MEASURE_OK")
            return PatliteLedBuzzerScenario::NO_QR_MEASURE_OK;
        if (upper == "SET_ITEM")
            return PatliteLedBuzzerScenario::SET_ITEM;
        if (upper == "SET_ITEM_NORMAL")
            return PatliteLedBuzzerScenario::SET_ITEM_NORMAL;
        if (upper == "SET_ITEM_PICKUP")
            return PatliteLedBuzzerScenario::SET_ITEM_PICKUP;
        if (upper == "SET_ITEM_SIZE_CHECK_START")
            return PatliteLedBuzzerScenario::SET_ITEM_SIZE_CHECK_START;
        if (upper == "SET_ITEM_MEASURE_OK")
            return PatliteLedBuzzerScenario::SET_ITEM_MEASURE_OK;
        if (upper == "CLEAR_ITEM")
            return PatliteLedBuzzerScenario::CLEAR_ITEM;
        if (upper == "DROP")
            return PatliteLedBuzzerScenario::DROP;
        if (upper == "DEVICE_ERROR")
            return PatliteLedBuzzerScenario::DEVICE_ERROR;
        if (upper == "DEVICE_ERROR_CLEAR")
            return PatliteLedBuzzerScenario::DEVICE_ERROR_CLEAR;
        if (upper == "CHECK_COMPLETE")
            return PatliteLedBuzzerScenario::CHECK_COMPLETE;
        if (upper == "NO_QR_CHECK_COMPLETE")
            return PatliteLedBuzzerScenario::NO_QR_CHECK_COMPLETE;
        if (upper == "SET_ITEM_CHECK_COMPLETE")
            return PatliteLedBuzzerScenario::SET_ITEM_CHECK_COMPLETE;
        if (upper == "INVALID_PLACE")
            return PatliteLedBuzzerScenario::INVALID_PLACE;

        throw std::invalid_argument("Unknown scenario: " + str);
    }

} // namespace fta_actuators
