/**
 * @file MyCobot.hpp
 * @brief High-level C++ API for myCobot.
 *
 * @copyright Elephant Robotics
 */

#ifndef MYCOBOTCPP_MYCOBOT_MYCOBOT_HPP
#define MYCOBOTCPP_MYCOBOT_MYCOBOT_HPP

#include <array>
#include <vector>
#include <map>
#include <memory>
#include <stdexcept>
#include <functional>
#include "MyCobotExport.hpp"

namespace mycobot
{
    // --- 기본 타입 정의 ---
    enum Axis : int
    {
        X = 1,
        Y,
        Z,
        RX,
        RY,
        RZ
    };
    enum Joint : int
    {
        J1 = 1,
        J2,
        J3,
        J4,
        J5,
        J6
    };
    constexpr const int Axes = 6;
    constexpr const int Joints = 6;
    using Coords = std::array<double, Axes>;
    using Angles = std::array<double, Joints>;
    using IntAngles = std::array<int, Joints>; // 실시간 데이터용 정수 배열

    constexpr const int DefaultSpeed = 50;

    class MYCOBOTCPP_API MyCobotException : public std::runtime_error
    {
    public:
        explicit MyCobotException(const std::string &message) : std::runtime_error(message) {}
    };

    class MYCOBOTCPP_API CommandException : public MyCobotException
    {
    public:
        // ★★★ 이 부분을 수정 또는 추가합니다. ★★★
        // std::string을 받는 생성자를 명시적으로 정의
        explicit CommandException(const std::string &message)
            : MyCobotException(message) {}
    };

    class MYCOBOTCPP_API InitializationException : public MyCobotException
    {
    public:
        // ★★★ 여기도 동일하게 수정 또는 추가합니다. ★★★
        explicit InitializationException(const std::string &message)
            : MyCobotException(message) {}
    };

    /**
     * @class MyCobot
     * @brief Main class that defines a high-level, easy-to-use API for myCobot.
     */
    class MYCOBOTCPP_API MyCobot
    {
    public:
        /**
         * @brief Get singleton instance of MyCobot and initialize the connection.
         */
        static MyCobot &I();
        MyCobot() = default;

        // --- [수정 5] 자동 폴링 제어 함수 추가 ---
        void startAutoPolling(int interval_ms = 50);
        void stopAutoPolling();

        // --- 기본 제어 (명령 전송) ---
        void PowerOn();
        void PowerOff();
        void StopRobot();
        void SetFreshMode(int mode);
        void InitialPose(int speed = DefaultSpeed);

        // --- 위치/각도 제어 (명령 전송) ---
        void WriteAngles(const Angles &angles, int speed = DefaultSpeed);
        void WriteAngle(Joint joint, double value, int speed = DefaultSpeed);
        void WriteCoords(const Coords &coords, int speed = DefaultSpeed, int mode = 0);

        void RequestCoords();
        void RequestAngles();
        void RequestSpeeds();
        void RequestJointLoad(Joint joint);
        void RequestIsMoving();

        // --- 캐시된 데이터 조회 (읽기) ---
        Angles PeekAngles() const;
        Coords PeekCoords() const;
        IntAngles PeekSpeeds() const;
        int PeekJointLoad(Joint joint) const;
        bool PeekIsMoving() const;

        // --- 그리퍼 제어 ---
        void SetGriper(int open);

    private:
        std::shared_ptr<class MyCobotImpl> impl{};
    };

    // ★★★ 클래스 바깥, 네임스페이스 안에 이 함수 선언을 추가합니다. ★★★
    /**
     * @brief 지정된 시간(밀리초) 동안 Qt 이벤트 루프를 처리하며 대기합니다.
     */
    void MYCOBOTCPP_API wait(int milliseconds);

} // namespace mycobot

#endif // MYCOBOTCPP_MYCOBOT_MYCOBOT_HPP
