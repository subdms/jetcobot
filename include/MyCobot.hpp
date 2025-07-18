#ifndef ROBOSIGNAL_MYCOBOT_HPP
#define ROBOSIGNAL_MYCOBOT_HPP

#include <array>
#include <string>
#include <mutex>
#include <vector>
#include <map>
#include <functional>

#include <QtSerialPort/qserialport.h>
#include <QTimer>
#include <QByteArray>
#include <QEventLoop>

#include "robosignal_global.hpp"
#include "Common.hpp"

namespace rc
{
    constexpr const int SERIAL_TIMEOUT = 1000;
    constexpr const int PRESENT_LOAD_ADDRESS = 60; // 부하 주소는 60 (0x3C)

    class ROBOSIGNALSHARED_EXPORT MyCobot : public QObject
    {
        Q_OBJECT

    public:
        MyCobot(const MyCobot &) = delete;
        MyCobot &operator=(const MyCobot &) = delete;
        virtual ~MyCobot() = default;
        static MyCobot &Instance();

        // --- 연결 및 기본 설정 ---
        void Init();
        int Connect();
        int Disconnect();
        bool IsCncConnected();
        void SetFreshMode(int mode);
        long long MeasureRTT();
        int TaskStop();
        void FocusServo(Joint joint);
        void ReleaseAllServos();
        int ProgramPause();
        bool IsProgramPaused();
        int ProgramResume();

        // --- 로봇 제어 (쓰기) ---
        bool PowerOn();
        void SetSpeed(int percentage);
        void WriteAngles(const Angles &angles, int speed);
        void WriteAngle(Joint joint, double value, int speed);
        void WriteCoords(const Coords &coords, int speed, int mode = 0);
        void WriteCoord(Axis axis, double value, int speed);
        void SetEncoders(const Angles &encoders, int speed);
        void SetEncoder(int joint, int val);
        int SetGriper(int open);
        // ... 다른 모든 쓰기 함수들 ...

        // --- 로봇 상태 조회 (안전한 동기 읽기) ---
        bool IsPowerOn();
        bool IsAllServoEnabled();
        bool IsServoEnabled(Joint j);
        bool CheckRunning();
        bool IsInPosition(const Coords &coord, bool is_linear = true);
        bool IsInPositionEncoders(const Angles &encoders);
        // ... 다른 모든 동기 읽기 함수들 ...
        void RequestVoltages();
        void RequestAngles();
        void RequestSpeeds();
        void RequestCoords();
        // void RequestMultipleServoData(
        //     const std::vector<Joint> &joints,
        //     int data_id,
        //     std::function<void(const std::map<Joint, int> &)> callback);

        Angles PeekAngles() const;
        IntAngles PeekSpeeds() const;
        Voltages PeekVoltages() const;
        Coords PeekCoords() const;

        // ★★★ [신규] 단일 관절 부하를 위한 비동기 함수 선언 ★★★
        void RequestJointLoad(Joint joint);
        int PeekJointLoad(Joint joint) const; // 특정 관절의 캐시된 값을 반환

        // int GetJointLoad(Joint joint);
        double GetSpeed();
        bool StateCheck();
        Angles GetEncoders();
        int64_t GetCurrentTimeMs();

        // --- 서보 직접 제어 ---
        int GetServoData(Joint joint, int data_id, int mode = 0);

    protected:
        MyCobot();

    private:
        // --- 내부 함수 ---
        std::vector<std::pair<unsigned char, QByteArray>> Parse(QByteArray &data);
        void ResetInPositionFlag();
        void SerialWrite(const QByteArray &data) const;

    private slots:
        // --- Qt 슬롯 ---
        void HandleReadyRead();
        void HandleTimeout();
        void HandleError(QSerialPort::SerialPortError error);
        // void processNextDataRequest();

        // ★★★ 동기 함수들을 깨우기 위한 시그널들 ★★★
    signals:
        void connectionLost();
        void isPoweredOnReceived();
        // void anglesReceived();
        // void speedsReceived();
        void isAllServoEnabledReceived();
        void isServoEnabledReceived();
        void checkRunningReceived();
        void isInPositionReceived();
        void programPausedStatusReceived();
        void encodersReceived();
        void speedReceived();
        // void voltagesReceived();
        void servoDataReceived();
        // ... 다른 모든 동기 함수들을 위한 시그널 ...

    private:
        // 비동기 요청 체인을 관리하기 위한 멤버 변수들
        // std::atomic<bool> m_is_request_chain_running{false};
        // std::vector<Joint> m_request_queue;
        // int m_current_data_id{0};
        // std::map<Joint, int> m_temp_data_map;
        // std::function<void(const std::map<Joint, int> &)> m_final_callback;

        Joint m_last_requested_load_joint{J1};
        IntAngles real_cur_loads{};

        // --- 시리얼 통신 관련 ---
        QString m_last_error_string;
        QSerialPort *serial_port{nullptr};
        QTimer *serial_timer{nullptr};
        QByteArray read_data{};
        QString m_port_name; // ★★★ 수정 후: QString으로 변경 ★★★
        int m_baud_rate;

        // --- 로봇 상태 캐시 변수 ---
        bool is_controller_connected{false};
        bool is_powered_on{false};
        bool robot_is_moving{true};
        bool is_in_position{false};
        bool is_program_paused{false};
        bool is_all_servo_enabled{false};
        bool servo_enabled[Joints] = {false};
        double cur_speed{0.0};
        Angles cur_angles{};
        Coords cur_coords{};
        Angles cur_encoders{};
        IntAngles real_cur_speeds{};
        int last_servo_data_value{0};
        Voltages real_cur_voltages{};
        // ... 다른 모든 상태 캐시 변수들 ...
    };

} // namespace rc

#endif // ROBOSIGNAL_MYCOBOT_HPP
