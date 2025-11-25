// ============================================================================
// patlite_scenarios.hpp
// Patlite LED/Buzzer 상황별 제어 시나리오 정의
//
// C# StatusService_WATA.cs의 Pattlite_Buzzer_LED 함수를 C++로 포팅
// ============================================================================

#ifndef FTA_ACTUATORS__PATLITE_SCENARIOS_HPP_
#define FTA_ACTUATORS__PATLITE_SCENARIOS_HPP_

#include <string>
#include <vector>
#include <map>

namespace fta_actuators
{

    // ============================================
    // LED 색상 열거형
    // ============================================
    enum class LEDColor
    {
        CLEAR = 0,     // 꺼짐
        RED = 1,       // 빨강
        YELLOW = 2,    // 노랑
        LEMON = 3,     // 레몬
        GREEN = 4,     // 초록
        SKYBLUE = 5,   // 하늘색 (Cyan)
        BLUE = 6,      // 파랑
        PURPLE = 7,    // 보라
        PEACHBLOW = 8, // 복숭아색
        WHITE = 9      // 흰색
    };

    // ============================================
    // LED 패턴 열거형
    // ============================================
    enum class LEDPattern
    {
        OFF = 0,        // 꺼짐
        CONTINUOUS = 1, // 연속 점등
        PATTERN2 = 2,   // 패턴 2
        PATTERN3 = 3,   // 패턴 3
        PATTERN4 = 4,   // 패턴 4
        PATTERN5 = 5,   // 패턴 5
        PATTERN6 = 6    // 패턴 6
    };

    // ============================================
    // 부저 패턴 열거형
    // ============================================
    enum class BuzzerPattern
    {
        OFF = 0,        // 꺼짐
        CONTINUOUS = 1, // 연속음
        PATTERN1 = 2,   // 패턴 1
        PATTERN2 = 3,   // 패턴 2
        PATTERN3 = 4,   // 패턴 3
        PATTERN4 = 5,   // 패턴 4
        PATTERN5 = 6,   // 패턴 5
        PATTERN6 = 7    // 패턴 6
    };

    // ============================================
    // 상황별 제어 시나리오 열거형
    // C#의 ePlayBuzzerLed와 동일
    // ============================================
    enum class PatliteScenario
    {
        // 컨테이너 관련
        CONTAINER_OK, // 컨테이너 OK

        // 사이즈 측정 관련
        SIZE_CHECK_START,      // 사이즈 체크 시작
        SIZE_MEASURE_OK,       // 사이즈 측정 완료 (QR 있음)
        NO_QR_SIZE_MEASURE_OK, // 사이즈 측정 완료 (QR 없음)

        // QR 관련
        QR_PICKUP,            // QR 픽업
        QR_MEASURE_OK,        // QR 측정 완료
        NO_QR_PICKUP,         // QR 없이 픽업
        NO_QR_MEASURE_OK,     // QR 없이 측정 완료
        NO_QR_CHECK_COMPLETE, // QR 없이 체크 완료

        // 앱 물류 선택 (SET_ITEM) 관련
        SET_ITEM,                  // 앱에서 물류 선택
        SET_ITEM_NORMAL,           // 앱 물류 선택 (일반)
        SET_ITEM_PICKUP,           // 앱 물류 픽업
        SET_ITEM_SIZE_CHECK_START, // 앱 물류 사이즈 체크 시작
        SET_ITEM_MEASURE_OK,       // 앱 물류 측정 완료
        SET_ITEM_CHECK_COMPLETE,   // 앱 물류 체크 완료

        // 기타 작업
        CLEAR_ITEM,     // 아이템 클리어
        DROP,           // 드롭
        CHECK_COMPLETE, // 체크 완료

        // 에러 관련
        DEVICE_ERROR,       // 디바이스 에러
        DEVICE_ERROR_CLEAR, // 디바이스 에러 해제
        INVALID_PLACE       // 잘못된 위치 (보행자 감지)
    };

    // ============================================
    // 시나리오별 LED/Buzzer 설정 구조체
    // ============================================
    struct PatliteCommand
    {
        LEDColor led_color;
        LEDPattern led_pattern;
        BuzzerPattern buzzer_pattern;
        int buzzer_count;

        PatliteCommand()
            : led_color(LEDColor::GREEN), led_pattern(LEDPattern::CONTINUOUS), buzzer_pattern(BuzzerPattern::OFF), buzzer_count(1) {}

        PatliteCommand(LEDColor c, LEDPattern lp, BuzzerPattern bp, int count = 1)
            : led_color(c), led_pattern(lp), buzzer_pattern(bp), buzzer_count(count) {}
    };

    // ============================================
    // 시나리오 매핑 클래스
    // ============================================
    class PatliteScenarioMapper
    {
    public:
        PatliteScenarioMapper();

        // 시나리오에 해당하는 명령 가져오기
        PatliteCommand get_command(PatliteScenario scenario) const;

        // 시나리오 이름으로 명령 가져오기 (ROS2 토픽용)
        PatliteCommand get_command_by_name(const std::string &scenario_name) const;

        // 시나리오 이름 목록 가져오기
        std::vector<std::string> get_scenario_names() const;

    private:
        std::map<PatliteScenario, PatliteCommand> scenario_map_;
        std::map<std::string, PatliteScenario> name_to_scenario_;

        void initialize_scenarios();
    };

    // ============================================
    // 상황별 제어 함수들 (C#의 함수들과 대응)
    // ============================================
    class PatliteController
    {
    public:
        PatliteController();

        // 기본 시나리오 실행
        void execute_scenario(PatliteScenario scenario);

        // 측정 시작 시 부저/LED (StartMeasuringBuzzer)
        void start_measuring_buzzer(bool set_item, const std::string &qr_code, bool is_error, bool func_off);

        // 측정 완료 시 부저/LED (FinishMeasuringBuzzer)
        void finish_measuring_buzzer(bool set_item, const std::string &qr_code, bool is_error, bool func_off);

        // 사이즈 측정 완료 (FinishMeasuringSize)
        void finish_measuring_size(const std::string &qr_code, bool func_off);

        // 보행자 감지 경고 (AlertDetectPerson)
        void alert_detect_person(bool func_off);

        // 예외 체크 부저 (CheckExceptionBuzzer)
        void check_exception_buzzer(bool set_item, const std::string &qr_code, bool func_off);

    private:
        PatliteScenarioMapper mapper_;

        // 실제 하드웨어 제어 (나중에 구현)
        void execute_command(const PatliteCommand &command);
    };

    // ============================================
    // 헬퍼 함수: enum을 문자열로 변환
    // ============================================
    std::string led_color_to_string(LEDColor color);
    std::string led_pattern_to_string(LEDPattern pattern);
    std::string buzzer_pattern_to_string(BuzzerPattern pattern);
    std::string scenario_to_string(PatliteScenario scenario);

    // ============================================
    // 헬퍼 함수: 문자열을 enum으로 변환
    // ============================================
    LEDColor string_to_led_color(const std::string &str);
    LEDPattern string_to_led_pattern(const std::string &str);
    BuzzerPattern string_to_buzzer_pattern(const std::string &str);
    PatliteScenario string_to_scenario(const std::string &str);

} // namespace fta_actuators

#endif // FTA_ACTUATORS__PATLITE_SCENARIOS_HPP_
