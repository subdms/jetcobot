#ifndef ROBOSIGNAL_MYCOBOT_HPP
#define ROBOSIGNAL_MYCOBOT_HPP

#include <array>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <mutex>
#include <queue>

#include <QtSerialPort/qserialport.h>
#include <QTimer>
#include <QByteArray>
#include <QEventLoop>

#include "robosignal_global.hpp"
#include "Common.hpp"

namespace rc
{
    constexpr const int SERIAL_TIMEOUT = 1000;     // 동기 함수들의 기본 타임아웃 (ms)
    constexpr const int PRESENT_LOAD_ADDRESS = 60; // 부하 주소는 60 (0x3C)

    // ★★★ 요청할 데이터의 종류를 명확하게 정의하는 enum 클래스 ★★★
    enum class RequestType
    {
        REQ_Angles,
        REQ_Coords,
        REQ_Speeds,
        REQ_Loads,
        REQ_IsMoving,
        REQ_Voltages
    };

    class ROBOSIGNALSHARED_EXPORT MyCobot : public QObject
    {
        Q_OBJECT

    public:
        // --- 싱글톤 및 기본 설정 ---
        MyCobot(const MyCobot &) = delete;
        MyCobot &operator=(const MyCobot &) = delete;
        virtual ~MyCobot() = default;
        static MyCobot &Instance();

        // --- 연결 및 초기화 ---
        void Init();
        int Connect();
        int Disconnect();
        bool IsCncConnected();
        void SetFreshMode(int mode);

        // ======================================================================
        // API 그룹 1: 쓰기(Write) 및 직접 실행 함수 (Fire-and-Forget)
        // ======================================================================
        bool PowerOn();
        int TaskStop();
        int ProgramPause();
        int ProgramResume();
        void ReleaseAllServos(); // PowerOff 대신 이 함수를 사용합니다.
        void SetSpeed(int percentage);
        void WriteAngles(const Angles &angles, int speed);
        void WriteAngle(Joint joint, double value, int speed);
        void WriteCoords(const Coords &coords, int speed, int mode = 1);
        void WriteCoord(Axis axis, double value, int speed);
        void SetEncoders(const Angles &encoders, int speed);
        void SetEncoder(int joint, int val);
        int SetGriper(int open);

        // ======================================================================
        // API 그룹 2: 데이터 자동 수집 제어 (Autonomous Factory Control)
        // ======================================================================
        void startAutoPolling(int interval_ms = 50);
        void stopAutoPolling();

        // ★★★ [최종] 모든 데이터 요청을 이 함수 하나로 통일합니다. ★★★
        // 이 함수가 바로 '관제탑에 착륙 요청을 접수하는' 역할을 합니다.
        void scheduleRequest(RequestType request_type, Joint joint = Joint::J1);

        void RequestAngles();
        void RequestSpeeds();
        void RequestCoords();
        void RequestJointLoad(Joint joint);
        void RequestIsMoving();
        void RequestVoltages();

        // ======================================================================
        // API 그룹 3: 캐시된 데이터 조회 (Peek - Non-blocking)
        // ======================================================================
        Angles PeekAngles() const;
        Coords PeekCoords() const;
        IntAngles PeekSpeeds() const;
        Voltages PeekVoltages() const;
        int PeekJointLoad(Joint joint) const;
        bool PeekIsMoving() const; // CheckRunning의 비동기 버전

        // ======================================================================
        // API 그룹 4: 동기 데이터 읽기 (내부 테스트 및 특수 목적용)
        // ======================================================================
        bool IsPowerOn();
        bool IsAllServoEnabled();
        bool IsServoEnabled(Joint j);
        bool IsProgramPaused();
        bool IsInPosition(const Coords &coords, bool is_linear);
        double GetSpeed();
        Angles GetAngles();   // 동기 버전
        Angles GetEncoders(); // 동기 버전

    protected:
        MyCobot();

    private:
        // --- 내부 헬퍼 함수 ---
        std::vector<std::pair<unsigned char, QByteArray>> Parse(QByteArray &data);
        void ResetInPositionFlag();
        void SerialWrite(const QByteArray &data) const;
        int GetServoData(Joint joint, int data_id, int mode = 0);

    private slots:
        // --- Qt 슬롯 ---
        void HandleReadyRead();
        void HandleTimeout();
        void HandleError(QSerialPort::SerialPortError error);
        void pollNextData(); // 자동 폴링 타이머에 연결될 슬롯
        // ★★★ [신규] 큐에서 다음 요청을 처리하는 private 슬롯 ★★★
        void processNextRequestInQueue();

    signals:
        // --- 동기 함수들을 깨우기 위한 시그널들 ---
        void connectionLost();
        void pongReceived();
        void isInPositionReceived();
        void isPoweredOnReceived();
        void anglesReceived();
        void speedsReceived();
        void encodersReceived();
        void isAllServoEnabledReceived();
        void isServoEnabledReceived();
        void checkRunningReceived();
        void programPausedStatusReceived();
        void speedReceived();
        void servoDataReceived();
        void coordsReceived();

    private:
        // ★★★ "항공 관제탑"의 핵심 멤버 변수들 ★★★
        // 1. 요청 대기열 (착륙 대기 중인 비행기 목록)
        std::queue<std::pair<RequestType, Joint>> m_request_queue;
        // 2. 관제탑 상태 플래그 (활주로가 사용 중인지 여부)
        bool m_scheduler_is_busy{false};
        // --- 시리얼 통신 관련 ---
        QString m_port_name;
        int m_baud_rate;
        QSerialPort *serial_port{nullptr};
        QTimer *serial_timer{nullptr};
        QByteArray read_data{};
        QString m_last_error_string;

        // --- 자동 폴링 메커니즘 ---
        QTimer m_polling_timer;
        int m_polling_counter{0};

        // --- 로봇 상태 캐시 변수 ---
        bool is_controller_connected{false};
        bool is_powered_on{false};
        bool robot_is_moving{true};
        bool is_program_paused{false};
        bool is_all_servo_enabled{false};
        bool servo_enabled[Joints] = {false};
        double cur_speed{0.0};
        Angles cur_angles{};
        Coords cur_coords{};
        Angles cur_encoders{};
        IntAngles real_cur_speeds{};
        Voltages real_cur_voltages{};
        IntAngles real_cur_loads{};
        int last_servo_data_value{0};
        Joint m_last_requested_load_joint{J1};
        bool is_in_position{false};
    };

} // namespace rc

#endif // ROBOSIGNAL_MYCOBOT_HPP
