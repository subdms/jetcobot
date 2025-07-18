#include "MyCobot.hpp"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <stdexcept>

#include <qeventloop.h>
#include <qnamespace.h>
#include <QCoreApplication>
#include <QDebug>
#include <QStringList>
#include <QThread>
#include <QtSerialPort/qserialport.h>
#include <QtSerialPort/qserialportinfo.h>
#include <QElapsedTimer>

#include "Common.hpp"
#include "Firmata.hpp"
#include "SystemInfo.hpp"
#define log_category ::rc::log::robot_controller
#include "log/Log.hpp"

namespace rc
{

    constexpr const double EMC_COMMAND_DELAY = 0.1;

    enum class ROBOT_STATE
    {
        COORDS_STATE,
        ANGLES_STATE,
        RUN_PROGRAME_STATE,
        OTHER_STATE,
    };

    MyCobot::MyCobot()                     // default 생성자 대신 다시 구현
        : m_port_name("/dev/ttyJETCOBOT"), // ★★★ 이니셜라이저 리스트 사용 ★★★
          m_baud_rate(1000000),
          m_last_error_string("") // 멤버 변수 선언 시 초기화했다면 생략 가능
    {
        // 객체 생성 및 시그널 연결 (프로그램 실행 중 한 번만 수행)
        serial_port = new QSerialPort(this);
        serial_timer = new QTimer(this);
        serial_timer->setSingleShot(true);

        connect(serial_port, &QSerialPort::readyRead, this, &MyCobot::HandleReadyRead);
        connect(serial_timer, &QTimer::timeout, this, &MyCobot::HandleTimeout);
    }

    MyCobot &MyCobot::Instance()
    {
        static MyCobot singleton;
        return singleton;
    }

    void MyCobot::Init()
    {
        InitFirmata();
        Connect(); // 이제 Connect를 호출
        RequestAngles();
        RequestCoords();
    }

    int MyCobot::Connect() // InitSerialPort -> Connect
    {
        if (serial_port->isOpen())
        {
            return 0; // 이미 연결됨, 성공
        }

        LogInfo << "Trying to connect to port: " << m_port_name;

        serial_port->setPortName(m_port_name);
        serial_port->setBaudRate(m_baud_rate);

        if (!serial_port->open(QIODevice::ReadWrite))
        {
            // ★★★ 예외 처리 추가 시작 ★★★
            QSerialPort::SerialPortError error_code = serial_port->error();
            std::string error_message = serial_port->errorString().toStdString();

            LogError << "Failed to open port " << m_port_name << ": " << serial_port->errorString();

            // 구체적인 오류 정보를 담아 예외를 던진다.
            // std::error_code를 사용하면, 고수준 API에서 잡아서 처리하기 용이하다.
            throw std::system_error(static_cast<int>(error_code),
                                    std::system_category(), // 포트 오류는 시스템 오류의 일종
                                    "Failed to open port " + m_port_name.toStdString() + ": " + error_message);

            // throw를 사용하면 이 아래 코드는 실행되지 않지만,
            // 예외를 사용하지 않는 경우를 위해 남겨둘 수 있습니다.
            // return -1;
            // ★★★ 예외 처리 추가 끝 ★★★
        }
        return 0; // 성공
    }

    int MyCobot::Disconnect()
    {
        LogTrace;

        // serial_port 포인터가 유효하고, 포트가 열려 있을 경우에만 close()를 호출
        if (serial_port && serial_port->isOpen())
        {
            serial_port->close();
            LogInfo << "Port closed.";
        }

        return 0;
    }

    bool MyCobot::IsCncConnected()
    {
        return serial_port->isOpen();
    }

    bool MyCobot::PowerOn()
    {
        // PowerOn 명령어(0x10) 패킷을 직접 생성합니다.
        // 이 명령어는 보통 별도의 파라미터가 없습니다.
        // [HEADER, HEADER, LEN(3), CMD(0x10), FOOTER]

        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(2));                // LEN 필드. 프로토콜에 따라 값이 다를 수 있음.
        command.append(char(Command::PowerOn)); // 0x10
        command.append(FIRMATA_FOOTER);

        // 생성된 명령어를 시리얼 포트로 직접 전송합니다.
        SerialWrite(command);

        // PowerOn 명령은 보통 별도의 응답(return value)이 없습니다.
        // 따라서 일단은 성공(true)으로 간주하고 반환합니다.
        //
        // 더 확실한 확인이 필요하다면, 잠시 딜레이를 준 후
        // IsPowerOn() 함수를 호출하여 실제 상태를 확인할 수 있습니다.
        // 예: QThread::msleep(100); return IsPowerOn();
        return true;
    }

    // 요청만 보내는 비동기 함수
    void MyCobot::RequestVoltages()
    {
        // 전압 요청 명령어(0xE3) 패킷을 생성하여 전송
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(2));                           // LEN
        command.append(char(Command::GET_SERVO_VOLTAGES)); // 0xE3
        command.append(FIRMATA_FOOTER);

        // 직접 전송 (응답은 나중에 HandleReadyRead가 처리)
        SerialWrite(command);
    }

    void MyCobot::RequestAngles()
    {
        SerialWrite(CommandGetAngles);
    }

    void MyCobot::RequestSpeeds()
    {
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(2));
        command.append(char(Command::GET_SERVO_SPEEDS));
        command.append(FIRMATA_FOOTER);
        SerialWrite(command);
    }

    void MyCobot::RequestCoords()
    {
        // GetCoords 명령어(0x23)는 파라미터가 필요 없습니다.
        // [HEADER, HEADER, LEN(3), CMD(0x23), FOOTER]
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(2));                  // LEN
        command.append(char(Command::GetCoords)); // 0x23
        command.append(FIRMATA_FOOTER);

        // 직접 전송 (응답은 나중에 HandleReadyRead가 처리)
        SerialWrite(command);
    }

    // 저장된 값을 보기만 하는 함수
    Angles MyCobot::PeekAngles() const
    {
        return cur_angles;
    }

    IntAngles MyCobot::PeekSpeeds() const
    {
        return real_cur_speeds;
    }

    Voltages MyCobot::PeekVoltages() const
    {
        // 로봇과 통신하지 않고, 현재 캐시된 값을 바로 반환
        return real_cur_voltages;
    }

    Coords MyCobot::PeekCoords() const
    {
        // 뮤텍스 등으로 보호할 필요 없이, 현재 캐시된 값을 바로 반환
        return cur_coords;
    }

    void MyCobot::WriteAngles(const Angles &angles, int speed)
    {
        // 1. 로봇이 새로운 목표 지점으로 이동을 시작하므로,
        //    '제자리에 도착한 상태'가 아님을 표시합니다. 이 로직은 유지합니다.
        ResetInPositionFlag();

        // 2. 명령어 패킷을 직접 생성합니다.
        // [HEADER, HEADER, LEN(15), CMD(0x22), J1_msb, J1_lsb, ..., J6_msb, J6_lsb, speed, FOOTER]
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(15)); // Length: 6 joints * 2 bytes + speed 1 byte = 13. 헤더 내 len/cmd 포함 시 15
        command.append(char(Command::WriteAngles));

        // 6개 관절의 각도 값을 각각 Big-Endian 2바이트로 변환하여 추가합니다.
        for (const double &angle : angles)
        {
            signed short centi_deg = static_cast<signed short>(angle * 100);
            command.append(static_cast<char>((centi_deg >> 8) & 0xFF)); // MSB (상위 바이트)
            command.append(static_cast<char>(centi_deg & 0xFF));        // LSB (하위 바이트)
        }

        // 속도 값을 추가합니다.
        command.append(static_cast<char>(speed));
        command.append(FIRMATA_FOOTER);

        // 3. 명령 큐를 거치지 않고 시리얼 포트에 직접 전송합니다.
        SerialWrite(command);
    }

    void MyCobot::WriteAngle(Joint joint, double value, int speed)
    {
        ResetInPositionFlag();

        // 1. 각도 값을 로봇이 이해하는 정수 형태로 변환합니다.
        short angle_val = static_cast<short>(value * 100);

        // 2. 명령어 패킷을 직접 생성합니다.
        // [HEADER, HEADER, LEN(5), CMD(0x21), joint, angle_msb, angle_lsb, speed, FOOTER]
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(6)); // ★★★ Length를 6에서 5로 수정 ★★★
        command.append(char(Command::WriteAngle));
        command.append(char(joint));

        // 각도 값을 Big-Endian 2바이트로 분할
        command.append(static_cast<char>((angle_val >> 8) & 0xFF));
        command.append(static_cast<char>(angle_val & 0xFF));

        // ★★★ 속도 값을 1바이트로 전송 ★★★
        command.append(static_cast<char>(speed));

        command.append(FIRMATA_FOOTER);

        // 3. 시리얼 포트에 직접 전송합니다.
        SerialWrite(command);
    }

    void MyCobot::WriteCoords(const Coords &coords, int speed, int mode)
    {
        // 1. 새로운 움직임이 시작되었음을 캐시에 알림 (기존 로직 유지)
        ResetInPositionFlag();

        // 2. 명령어 패킷을 직접 생성
        // [HEADER, HEADER, LEN, CMD(0x25), X,Y,Z,RX,RY,RZ, SPEED, MODE, FOOTER]
        // 데이터 길이: 좌표(6*2=12) + 속도(1) + 모드(1) = 14 바이트
        // LEN 필드 값: 데이터 길이(14) + CMD(1) + LEN(1) = 16 (프로토콜 규칙에 따라 조정 필요)
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(16));                   // LEN 필드 (규칙에 따라 15 또는 16일 수 있음)
        command.append(char(Command::WriteCoords)); // 0x25

        // 6개 좌표 값을 Big-Endian 2바이트로 변환하여 추가
        // X, Y, Z 좌표 (10을 곱함)
        for (int i = 0; i < 3; ++i)
        {
            signed short tenth_mm = static_cast<signed short>(coords[i] * 10);
            command.append(static_cast<char>((tenth_mm >> 8) & 0xFF));
            command.append(static_cast<char>(tenth_mm & 0xFF));
        }
        // RX, RY, RZ 좌표 (100을 곱함)
        for (int i = 3; i < rc::Axes; ++i)
        {
            signed short centi_deg = static_cast<signed short>(coords[i] * 100);
            command.append(static_cast<char>((centi_deg >> 8) & 0xFF));
            command.append(static_cast<char>(centi_deg & 0xFF));
        }

        // 속도와 모드 추가
        // 속도 계산 로직은 원본 코드를 따름. 펌웨어에서 % 단위로 받을 수 있음.
        command.append(static_cast<char>(speed * 100 / MaxLinearSpeed));
        command.append(static_cast<char>(mode)); // Mode. 원본 코드의 '2'를 그대로 사용.

        command.append(FIRMATA_FOOTER);

        // 3. 명령 큐를 거치지 않고 시리얼 포트에 직접 전송
        SerialWrite(command);
    }

    void MyCobot::WriteCoord(Axis axis, double value, int speed)
    {
        // 1. 새로운 움직임이 시작되었음을 캐시에 알림 (기존 로직 유지)
        ResetInPositionFlag();

        // 2. 명령어 패킷을 직접 생성
        // [HEADER, HEADER, LEN, CMD(0x24), AXIS, VALUE, SPEED, FOOTER]
        // 데이터 길이: 축 ID(1) + 값(2) + 속도(1) = 4 바이트
        // LEN 필드 값: 데이터 길이(4) + CMD(1) + LEN(1) = 6
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(6));                   // LEN 필드
        command.append(char(Command::WriteCoord)); // 0x24

        // 축 ID 추가
        command.append(char(axis));

        // 좌표 값을 Big-Endian 2바이트로 변환하여 추가
        // 원본 코드의 변환 로직(10 곱하기)을 그대로 따름
        signed short tenth_mm = static_cast<signed short>(value * 10);
        command.append(static_cast<char>((tenth_mm >> 8) & 0xFF));
        command.append(static_cast<char>(tenth_mm & 0xFF));

        // 속도 추가
        command.append(static_cast<char>(speed * 100 / MaxLinearSpeed));

        command.append(FIRMATA_FOOTER);

        // 3. 명령 큐를 거치지 않고 시리얼 포트에 직접 전송
        SerialWrite(command);
    }

    double MyCobot::GetSpeed()
    {
        // 1. GetSpeed 명령어(0x40)를 직접 보냄
        // [HEADER, HEADER, LEN(3), CMD(0x40), FOOTER]
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(2));                 // LEN
        command.append(char(Command::GetSpeed)); // 0x40
        command.append(FIRMATA_FOOTER);
        SerialWrite(command);

        // 2. 응답이 올 때까지 이벤트 루프를 돌며 대기 (동기화)
        QEventLoop loop;
        QTimer timer;
        timer.setSingleShot(true);

        // HandleReadyRead의 case Command::GetSpeed: 에서 이 시그널을 emit 해야 함
        connect(this, &MyCobot::speedReceived, &loop, &QEventLoop::quit); // 가상의 시그널
        connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);

        timer.start(SERIAL_TIMEOUT);
        loop.exec();

        // 3. HandleReadyRead에 의해 업데이트된 최신 속도 값을 반환
        return cur_speed;
    }

    void MyCobot::SetSpeed(int percentage)
    {
        // SetSpeed 명령어(0x41) 패킷을 직접 생성
        // [HEADER, HEADER, LEN(4), CMD(0x41), PERCENT, FOOTER]
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(3));                 // LEN 필드
        command.append(char(Command::SetSpeed)); // 0x41

        // 속도(백분율) 값을 1바이트로 추가
        command.append(static_cast<char>(percentage));

        command.append(FIRMATA_FOOTER);

        // 직접 전송
        SerialWrite(command);
    }

    bool MyCobot::StateCheck()
    {
        // 이 함수가 올바르게 동작하려면, IsPowerOn() 함수가
        // 반드시 동기식(요청-대기-응답)으로 구현되어 최신 상태를 반환해야 합니다.
#if defined ROBCTL_ATOMMAIN
        return IsCncConnected() && IsPowerOn();
#elif defined ROBCTL_PHOENIX
        return IsCncConnected();
#else
        return false; // 기본적으로 false를 반환하도록 안전장치 추가
#endif
    }

    int MyCobot::TaskStop()
    {
        LogTrace << ": TaskStop";

        // TaskStop 명령어(0x29) 패킷을 직접 생성
        // [HEADER, HEADER, LEN(3), CMD(0x29), FOOTER]
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(2));                 // LEN
        command.append(char(Command::TaskStop)); // 0x29
        command.append(FIRMATA_FOOTER);

        // 직접 전송
        SerialWrite(command);

        return 0;
    }

    bool rc::MyCobot::CheckRunning()
    {
        // 응답 수신 여부를 확인하기 위한 플래그
        bool response_received = false;

        // 1. 전체 로직을 try-catch로 감쌉니다.
        try
        {
            // CheckRunning 명령어(0x2B)를 직접 보냄
            QByteArray command;
            command.append(FIRMATA_HEADER);
            command.append(char(3));                     // LEN: CMD(1) + 자기자신(1) = 2. -> 프로토콜에 따라 3
            command.append(char(Command::CheckRunning)); // 0x2B
            command.append(FIRMATA_FOOTER);
            SerialWrite(command);

            // 응답이 올 때까지 이벤트 루프를 돌며 대기
            QEventLoop loop;
            QTimer timer;
            timer.setSingleShot(true);

            // 람다를 사용하여 응답 수신 플래그를 설정
            connect(this, &MyCobot::checkRunningReceived, &loop, [&]()
                    {
            response_received = true;
            loop.quit(); });
            connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);

            timer.start(SERIAL_TIMEOUT);
            loop.exec();
        }
        // 2. SerialWrite에서 발생할 수 있는 예외를 잡습니다.
        catch (const std::exception &e)
        {
            // 예외 메시지에 어떤 명령에서 실패했는지 명확한 맥락을 추가합니다.
            throw std::runtime_error("Failed to send CheckRunning command: " + std::string(e.what()));
        }

        // 3. 타임아웃 발생 여부를 확인하고 예외를 던집니다.
        if (!response_received)
        {
            throw std::runtime_error("Timeout: No response received for CheckRunning command.");
        }

        // 모든 과정이 성공했을 때만 HandleReadyRead에 의해 업데이트된 최신 값을 반환합니다.
        return robot_is_moving;
    }

    bool MyCobot::IsInPositionEncoders(const Angles &encoders)
    {
        // GetEncoders()가 동기식으로 구현되어 있다는 전제 하에 호출
        Angles currentEncoders = GetEncoders();
        return rc::CoordsEqual(currentEncoders, encoders, EncodersEpsilon);
    }

#include <stdexcept> // std::runtime_error
#include <string>    // std::string

    // ...

    bool MyCobot::IsInPosition(const Coords &coords, bool is_linear)
    {
        // 응답 수신 여부를 확인하기 위한 플래그
        bool response_received = false;

        // 1. 전체 로직을 try-catch로 감쌉니다.
        try
        {
            // IsInPosition 명령어(0x2A) 패킷 생성
            QByteArray command;
            command.append(FIRMATA_HEADER);
            // LEN 계산: Coords(12) + is_linear(1) + CMD(1) + 자기자신(1) = 15
            command.append(char(15));
            command.append(char(Command::IsInPosition)); // 0x2A

            // 파라미터 직렬화
            if (is_linear)
            {
                for (int i = 0; i < 3; ++i)
                {
                    signed short tenth_mm = static_cast<signed short>(coords[i] * 10);
                    command += static_cast<char>(tenth_mm >> 8);
                    command += static_cast<char>(tenth_mm & 0x00FF);
                }
                for (int i = 3; i < rc::Axes; ++i)
                {
                    signed short centi_deg = static_cast<signed short>(coords[i] * 100);
                    command += static_cast<char>(centi_deg >> 8);
                    command += static_cast<char>(centi_deg & 0x00FF);
                }
            }
            else
            {
                for (const double &angle : coords)
                {
                    signed short centi_deg = static_cast<signed short>(angle * 100);
                    command += static_cast<char>(centi_deg >> 8);
                    command += static_cast<char>(centi_deg & 0x00FF);
                }
            }
            command.append(static_cast<char>(is_linear));
            command.append(FIRMATA_FOOTER);

            // 명령어 전송
            SerialWrite(command);

            // 응답이 올 때까지 이벤트 루프를 돌며 대기
            QEventLoop loop;
            QTimer timer;
            timer.setSingleShot(true);

            // 람다를 사용하여 응답 수신 플래그를 설정
            connect(this, &MyCobot::isInPositionReceived, &loop, [&]()
                    {
            response_received = true;
            loop.quit(); });
            connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);

            timer.start(SERIAL_TIMEOUT);
            loop.exec();
        }
        // 2. SerialWrite에서 발생할 수 있는 예외를 잡습니다.
        catch (const std::exception &e)
        {
            // 예외 메시지에 어떤 명령에서 실패했는지 명확한 맥락을 추가합니다.
            throw std::runtime_error("Failed to send IsInPosition command: " + std::string(e.what()));
        }

        // 3. 타임아웃 발생 여부를 확인하고 예외를 던집니다.
        if (!response_received)
        {
            throw std::runtime_error("Timeout: No response received for IsInPosition command.");
        }

        // 모든 과정이 성공했을 때만 HandleReadyRead에 의해 업데이트된 최신 값을 반환합니다.
        return is_in_position;
    }

    void MyCobot::FocusServo(Joint joint)
    {
        // FocusServo 명령어(0x57) 패킷을 직접 생성
        // [HEADER, HEADER, LEN(4), CMD(0x57), JOINT_ID, FOOTER]
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(3));                   // LEN 필드
        command.append(char(Command::FocusServo)); // 0x57

        // 파라미터로 받은 관절 ID를 추가
        command.append(static_cast<char>(joint));

        command.append(FIRMATA_FOOTER);

        // 직접 전송
        SerialWrite(command);
    }

    // ★★★ [추가] 헤더에 선언되었지만 구현이 누락되었던 함수를 추가합니다. ★★★
    // 이 함수는 '안전한 동기' 방식으로 동작하며, GetJointLoad에서 사용됩니다.
    int MyCobot::GetServoData(Joint joint, int data_id, int mode)
    {
        QEventLoop loop;
        // GetServoData는 이제 GetJointLoad에서만 사용되므로,
        // 전용 시그널 대신 범용 servoDataReceived를 사용합니다.
        connect(this, &MyCobot::servoDataReceived, &loop, &QEventLoop::quit);

        QByteArray command;
        command.append(FIRMATA_HEADER);

        if (mode == 1)
        { // 2바이트 읽기
            command.append(char(5));
            command.append(char(Command::GetServoData));
            command.append(static_cast<char>(joint));
            command.append(static_cast<char>(data_id));
            command.append(static_cast<char>(mode));
        }
        else
        { // 1바이트 읽기
            command.append(char(4));
            command.append(char(Command::GetServoData));
            command.append(static_cast<char>(joint));
            command.append(static_cast<char>(data_id));
        }

        command.append(FIRMATA_FOOTER);
        SerialWrite(command);

        QTimer::singleShot(1000, &loop, &QEventLoop::quit);
        loop.exec();

        return last_servo_data_value;
    }

    void rc::MyCobot::ReleaseAllServos()
    {
        LogTrace;
        // ReleaseAllServos 명령어(0x13) 패킷을 직접 생성
        // [HEADER, HEADER, LEN(3), CMD(0x13), FOOTER]
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(2));                         // LEN 필드
        command.append(char(Command::ReleaseAllServos)); // 0x13
        command.append(FIRMATA_FOOTER);

        // 직접 전송
        SerialWrite(command);
    }

    int MyCobot::ProgramPause()
    {
        LogTrace;

        // ProgramPause 명령어(0x26) 패킷을 직접 생성
        // [HEADER, HEADER, LEN(3), CMD(0x26), FOOTER]
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(2));                     // LEN
        command.append(char(Command::ProgramPause)); // 0x26
        command.append(FIRMATA_FOOTER);

        // 직접 전송
        SerialWrite(command);
        return 0;
    }

    bool MyCobot::IsProgramPaused()
    {
        LogTrace;

        // 응답 수신 여부를 확인하기 위한 플래그
        bool response_received = false;

        // ★★★ 1. 전체 로직을 try-catch로 감쌉니다. ★★★
        try
        {
            // IsProgramPaused 명령어(0x27)를 직접 보냄
            QByteArray command;
            command.append(FIRMATA_HEADER);
            command.append(char(3));                        // LEN: CMD(1) + 자기자신(1) = 2. -> 프로토콜에 따라 3
            command.append(char(Command::IsProgramPaused)); // 0x27
            command.append(FIRMATA_FOOTER);
            SerialWrite(command);

            // 응답이 올 때까지 이벤트 루프를 돌며 대기
            QEventLoop loop;
            QTimer timer;
            timer.setSingleShot(true);

            // 람다를 사용하여 응답 수신 플래그를 설정
            connect(this, &MyCobot::programPausedStatusReceived, &loop, [&]()
                    {
            response_received = true;
            loop.quit(); });
            connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);

            timer.start(SERIAL_TIMEOUT);
            loop.exec();
        }
        // ★★★ 2. SerialWrite에서 발생할 수 있는 예외를 잡습니다. ★★★
        catch (const std::exception &e)
        {
            // 예외를 그대로 다시 던지거나, 더 구체적인 예외로 포장할 수 있습니다.
            throw std::runtime_error("Failed to send IsProgramPaused command: " + std::string(e.what()));
        }

        // ★★★ 3. 타임아웃 발생 여부를 확인하고 예외를 던집니다. ★★★
        if (!response_received)
        {
            throw std::runtime_error("Timeout: No response received for IsProgramPaused command.");
        }

        // 모든 과정이 성공했을 때만 최신 값을 반환합니다.
        return is_program_paused;
    }

    int MyCobot::ProgramResume()
    {
        LogTrace;

        // ProgramResume 명령어(0x28) 패킷을 직접 생성
        // [HEADER, HEADER, LEN(3), CMD(0x28), FOOTER]
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(2));                      // LEN
        command.append(char(Command::ProgramResume)); // 0x28
        command.append(FIRMATA_FOOTER);

        // 직접 전송
        SerialWrite(command);
        return 0;
    }

    bool rc::MyCobot::IsPowerOn()
    {
        // 응답 수신 여부를 확인하기 위한 플래그
        bool response_received = false;

        // 1. 전체 로직을 try-catch로 감쌉니다.
        try
        {
            // IsPoweredOn 명령어(0x12)를 직접 보냄
            QByteArray command;
            command.append(FIRMATA_HEADER);
            command.append(char(3));                    // LEN: CMD(1) + 자기자신(1) = 2. -> 프로토콜에 따라 3
            command.append(char(Command::IsPoweredOn)); // 0x12
            command.append(FIRMATA_FOOTER);
            SerialWrite(command);

            // 응답이 올 때까지 이벤트 루프를 돌며 대기
            QEventLoop loop;
            QTimer timer;
            timer.setSingleShot(true);

            // 람다를 사용하여 응답 수신 플래그를 설정
            connect(this, &MyCobot::isPoweredOnReceived, &loop, [&]()
                    {
            response_received = true;
            loop.quit(); });
            connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);

            timer.start(SERIAL_TIMEOUT);
            loop.exec();
        }
        // 2. SerialWrite에서 발생할 수 있는 예외를 잡습니다.
        catch (const std::exception &e)
        {
            // 예외 메시지에 어떤 명령에서 실패했는지 명확한 맥락을 추가합니다.
            throw std::runtime_error("Failed to send IsPoweredOn command: " + std::string(e.what()));
        }

        // 3. 타임아웃 발생 여부를 확인하고 예외를 던집니다.
        if (!response_received)
        {
            throw std::runtime_error("Timeout: No response received for IsPoweredOn command.");
        }

        // 모든 과정이 성공했을 때만 HandleReadyRead에 의해 업데이트된 최신 값을 반환합니다.
        return is_powered_on;
    }

    bool MyCobot::IsServoEnabled(Joint j)
    {
        // 1. IsServoEnabled 명령어(0x50) 패킷을 직접 생성하여 전송
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(3));                       // LEN
        command.append(char(Command::IsServoEnabled)); // 0x50

        // 파라미터로 관절 ID 추가 (원본 코드의 j+1 로직은 펌웨어 규칙에 따라 확인 필요)
        command.append(static_cast<char>(j));

        command.append(FIRMATA_FOOTER);
        SerialWrite(command);

        // 2. 응답이 올 때까지 이벤트 루프를 돌며 대기 (동기화)
        QEventLoop loop;
        QTimer timer;
        timer.setSingleShot(true);

        connect(this, &MyCobot::isServoEnabledReceived, &loop, &QEventLoop::quit); // 가상의 시그널
        connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);

        timer.start(SERIAL_TIMEOUT);
        loop.exec();

        // 3. HandleReadyRead에 의해 업데이트된 최신 값을 반환
        //    응답이 어떤 관절에 대한 것인지도 함께 처리해야 함
        return servo_enabled[j];
    }

    bool MyCobot::IsAllServoEnabled()
    {
        // 1. IsAllServoEnabled 명령어(0x51)를 직접 보냄
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(2));                          // LEN
        command.append(char(Command::IsAllServoEnabled)); // 0x51
        command.append(FIRMATA_FOOTER);
        SerialWrite(command);

        // 2. 응답이 올 때까지 이벤트 루프를 돌며 대기 (동기화)
        QEventLoop loop;
        QTimer timer;
        timer.setSingleShot(true);

        connect(this, &MyCobot::isAllServoEnabledReceived, &loop, &QEventLoop::quit); // 가상의 시그널
        connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);

        timer.start(SERIAL_TIMEOUT);
        loop.exec();

        // 3. HandleReadyRead에 의해 업데이트된 최신 값을 반환
        return is_all_servo_enabled;
    }

    void MyCobot::SetEncoders(const Angles &encoders, int speed)
    {
        // SetEncoders 명령어(0x3C) 패킷을 직접 생성
        // [HEADER, HEADER, LEN, CMD(0x3C), E1, E2, E3, E4, E5, E6, SPEED, FOOTER]
        // 데이터 길이: 엔코더(6*2=12) + 속도(1) = 13 바이트
        // LEN 필드 값: 13(데이터) + 1(CMD) + 1(LEN) = 15
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(15));                   // LEN
        command.append(char(Command::SetEncoders)); // 0x3C

        // 6개 엔코더 값을 Big-Endian 2바이트로 변환하여 추가
        for (const double &encoder_val : encoders)
        {
            signed short encoder = static_cast<signed short>(encoder_val);
            command.append(static_cast<char>((encoder >> 8) & 0xFF));
            command.append(static_cast<char>(encoder & 0xFF));
        }

        // 속도 추가
        command.append(static_cast<char>(speed));
        command.append(FIRMATA_FOOTER);

        // 직접 전송
        SerialWrite(command);
        LogInfo << "SerialWrite SetEncoders";
    }

    Angles rc::MyCobot::GetEncoders()
    {
        // 1. GetEncoders 명령어(0x3D)를 직접 보냄
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(2));                    // LEN
        command.append(char(Command::GetEncoders)); // 0x3D
        command.append(FIRMATA_FOOTER);
        SerialWrite(command);

        // 2. 응답이 올 때까지 이벤트 루프를 돌며 대기 (동기화)
        QEventLoop loop;
        QTimer timer;
        timer.setSingleShot(true);

        connect(this, &MyCobot::encodersReceived, &loop, &QEventLoop::quit); // 이전에 정의한 시그널 재사용
        connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);

        timer.start(SERIAL_TIMEOUT);
        loop.exec();

        // 3. HandleReadyRead에 의해 업데이트된 최신 값을 반환
        return cur_encoders;
    }

    void rc::MyCobot::SetEncoder(int joint, int val)
    {
        if (joint < 0 || joint >= rc::Joints)
            return;

        // SetEncoder 명령어(0x3A) 패킷을 직접 생성
        // [HEADER, HEADER, LEN, CMD(0x3A), JOINT_ID, VALUE, FOOTER]
        // 데이터 길이: 관절 ID(1) + 값(2) = 3 바이트
        // LEN 필드 값: 3(데이터) + 1(CMD) + 1(LEN) = 5
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(5));                   // LEN
        command.append(char(Command::SetEncoder)); // 0x3A

        // 관절 ID 추가
        command.append(static_cast<char>(joint));

        // 엔코더 값을 Big-Endian 2바이트로 변환하여 추가
        signed short encoder = static_cast<signed short>(val);
        command.append(static_cast<char>((encoder >> 8) & 0xFF));
        command.append(static_cast<char>(encoder & 0xFF));

        command.append(FIRMATA_FOOTER);

        // 직접 전송
        SerialWrite(command);
    }

    int MyCobot::SetGriper(int open)
    {
        // 'open' 값에 따라 적절한 명령을 직접 전송
        if (open == 1)
        {
            // 그리퍼 열기 명령 전송
            SerialWrite(CommandSetGriperOpen); // CommandSetGriperOpen이 QByteArray로 정의되어 있다고 가정
        }
        else
        {
            // 그리퍼 닫기 명령 전송
            SerialWrite(CommandSetGriperClose); // CommandSetGriperClose가 QByteArray로 정의되어 있다고 가정
        }

        return 0;
    }

    void MyCobot::SerialWrite(const QByteArray &data) const
    {
        // LogDebug << "--> SENDING: " << data.toHex(' ').toUpper();

        // 1. 쓰기 전에 포트가 열려 있는지 확인하는 방어 코드
        if (!serial_port || !serial_port->isOpen())
        {
            throw std::runtime_error("Serial write failed: Port is not open.");
        }

        // 2. 실제 쓰기 작업 수행
        const qint64 bytes_written = serial_port->write(data);

        // 3. 쓰기 작업 결과 확인 및 예외 처리
        if (bytes_written == -1)
        {
            // 쓰기 실패는 심각한 오류 (예: 연결 끊김)
            std::string error_message = serial_port->errorString().toStdString();
            LogError << "Could not write data: " << serial_port->errorString();
            throw std::runtime_error("Serial write failed: " + error_message);
        }
        else if (bytes_written != data.size())
        {
            // 모든 데이터를 보내지 못한 경우도 오류로 간주
            std::string error_message = "Wrote " + std::to_string(bytes_written) +
                                        " bytes, but expected to write " + std::to_string(data.size()) + " bytes.";
            LogError << "Failed to write all data. " << QString::fromStdString(error_message);
            throw std::runtime_error("Incomplete serial write: " + error_message);
        }

        // 4. 버퍼 비우기
        // flush()도 실패할 수 있지만, 여기서는 write() 실패가 더 중요하므로 생략 가능
        if (!serial_port->flush())
        {
            qWarning() << "Serial port flush failed.";
        }
    }

    int64_t MyCobot::GetCurrentTimeMs()
    {
        auto t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        return t.count();
    }

    void MyCobot::ResetInPositionFlag()
    {
        // 이 함수는 새로운 움직임 명령이 시작될 때 호출됩니다.
        // '제자리에 도착함' 상태를 나타내는 플래그를 false로 리셋합니다.
        is_in_position = false;
        // LogTrace는 디버깅 시에만 필요하므로, 제거하거나 그대로 둘 수 있습니다.
        // LogDebug << "Position state reset to 'not in position'.";
    }

    /**
     * @brief [신규] 특정 관절의 부하 데이터 요청을 보냅니다. (비동기)
     */
    void MyCobot::RequestJointLoad(Joint joint)
    {
        // 다음에 올 응답이 어떤 관절의 것인지 기억해둡니다.
        m_last_requested_load_joint = joint;

        // GetServoData(0x53) 명령어를 2바이트 읽기 모드로 보냅니다.
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(5));
        command.append(char(Command::GetServoData));
        command.append(static_cast<char>(joint));
        command.append(char(PRESENT_LOAD_ADDRESS));
        command.append(static_cast<char>(1)); // 2바이트 읽기 모드
        command.append(FIRMATA_FOOTER);
        SerialWrite(command);
    }

    /**
     * @brief [신규] 특정 관절의 캐시된 부하 값을 조회합니다.
     */
    int MyCobot::PeekJointLoad(Joint joint) const
    {
        // 배열 인덱스는 0부터 시작하므로, joint ID에서 1을 빼줍니다.
        if (joint >= J1 && joint <= J6)
        {
            return real_cur_loads[static_cast<int>(joint) - 1];
        }
        return -1; // 잘못된 관절 ID
    }

    void rc::MyCobot::HandleReadyRead()
    {
        // 단일 스레드 환경이므로 뮤텍스는 제거합니다.
        read_data.append(serial_port->readAll());
        // LogDebug << "<-- RECEIVED: " << read_data.toHex(' ').toUpper();

        // 강화된 파서로 유효한 명령어만 추출합니다.
        std::vector<std::pair<unsigned char, QByteArray>> parsed_commands = Parse(read_data);
        if (parsed_commands.empty())
        {
            return; // 처리할 명령이 없으면 종료
        }

        // 2바이트를 16비트 정수로 디코딩하는 헬퍼 함수
        auto decode_int16 = [](const QByteArray &data, int index) -> int16_t
        {
            if (index + 1 < data.size())
            {
                const uint8_t msb = static_cast<uint8_t>(data[index]);
                const uint8_t lsb = static_cast<uint8_t>(data[index + 1]);
                return static_cast<int16_t>((msb << 8) | lsb);
            }
            return 0;
        };

        // 파싱된 모든 명령어에 대해 처리
        for (const auto &content : parsed_commands)
        {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"
            switch (static_cast<Command>(content.first))
            {
            // ======================================================
            // Boolean (참/거짓) 상태 응답 처리
            // ======================================================
            case Command::IsPoweredOn:
            {
                is_powered_on = static_cast<bool>(content.second.at(0));
                emit isPoweredOnReceived(); // IsPowerOn()을 깨움
                break;
            }
            case Command::CheckRunning:
            {
                robot_is_moving = static_cast<bool>(content.second.at(0));
                emit checkRunningReceived(); // CheckRunning()을 깨움
                break;
            }
            case Command::IsInPosition:
            {
                is_in_position = static_cast<bool>(content.second.at(0));
                emit isInPositionReceived(); // IsInPosition()을 깨움
                break;
            }
            case Command::IsProgramPaused:
            {
                is_program_paused = static_cast<bool>(content.second.at(0));
                emit programPausedStatusReceived(); // IsProgramPaused()을 깨움
                break;
            }
            case Command::IsAllServoEnabled:
            {
                is_all_servo_enabled = static_cast<bool>(content.second.at(0));
                emit isAllServoEnabledReceived(); // IsAllServoEnabled()을 깨움
                break;
            }
            case Command::IsServoEnabled:
            {
                if (content.second.size() >= 2)
                { // 응답 형식: [joint_id, state]
                    servo_enabled[static_cast<uint8_t>(content.second.at(0)) - 1] = static_cast<bool>(content.second.at(1));
                }
                emit isServoEnabledReceived(); // IsServoEnabled()을 깨움
                break;
            }
            // ======================================================
            // 배열(Array) 형태의 데이터 응답 처리
            // ======================================================
            case Command::GetAngles:
            {
                if (content.second.size() >= 12)
                {
                    for (size_t i = 0; i < rc::Joints; ++i)
                    {
                        cur_angles[i] = static_cast<double>(decode_int16(content.second, i * 2)) / 100.0;
                    }
                }
                // emit anglesReceived(); // GetAngles()을 깨움
                break;
            }
            case Command::GetCoords:
            {
                if (content.second.size() >= 12)
                {
                    for (size_t i = 0; i < 3; ++i)
                        cur_coords[i] = static_cast<double>(decode_int16(content.second, i * 2)) / 10.0;
                    for (size_t i = 3; i < rc::Axes; ++i)
                        cur_coords[i] = static_cast<double>(decode_int16(content.second, i * 2)) / 100.0;
                }
                // emit coordsReceived(); // GetCoords()가 동기식이면 필요
                break;
            }
            case Command::GetEncoders:
            {
                if (content.second.size() >= 12)
                {
                    for (size_t i = 0; i < rc::Joints; ++i)
                    {
                        cur_encoders[i] = static_cast<double>(decode_int16(content.second, i * 2));
                    }
                }
                emit encodersReceived(); // GetEncoders()을 깨움
                break;
            }
            case Command::GetServoData: // 0x53에 대한 응답
            {
                // 가장 최근에 요청했던 관절의 부하 값으로 간주하고 저장합니다.
                int joint_index = static_cast<int>(m_last_requested_load_joint) - 1;

                if (joint_index >= 0 && joint_index < Joints)
                {
                    if (content.second.size() >= 2)
                    {
                        real_cur_loads[joint_index] = decode_int16(content.second, 0);
                    }
                    else if (content.second.size() == 1)
                    {
                        real_cur_loads[joint_index] = static_cast<uint8_t>(content.second.at(0));
                    }
                }
                break;
            }
            // ======================================================
            // 우리가 추가한 실시간 데이터 응답 처리
            // ======================================================
            case Command::GET_SERVO_SPEEDS:
            {
                if (content.second.size() >= 12)
                {
                    for (size_t i = 0; i < rc::Joints; ++i)
                    {
                        real_cur_speeds[i] = decode_int16(content.second, i * 2);
                    }
                }
                // emit speedsReceived(); // GetJointsRealSpeeds()을 깨움
                break;
            }
            // ★★★ 여기에 아래 case 블록을 추가합니다. ★★★
            case Command::GET_SERVO_VOLTAGES:
            {
                // 로봇은 6바이트의 데이터를 보냅니다.
                if (content.second.size() >= 6)
                {
                    for (size_t i = 0; i < rc::Joints; ++i)
                    {
                        // 각 관절당 1바이트 값을 읽습니다.
                        int raw_voltage = static_cast<uint8_t>(content.second.at(i));

                        // 10.0으로 나누어 실제 전압(Volt) 단위로 변환합니다.
                        real_cur_voltages[i] = static_cast<double>(raw_voltage) / 10.0;
                    }
                }
                // 비동기 방식이므로 emit은 필요 없습니다.
                // emit voltagesReceived(); // 만약 동기식 GetVoltages() 함수를 만든다면 필요합니다.
                break;
            }
            // ======================================================
            // 기타 단일 값 응답 처리
            // ======================================================
            case Command::GetSpeed:
            {
                if (!content.second.isEmpty())
                {
                    cur_speed = static_cast<double>(content.second.at(0));
                }
                emit speedReceived(); // GetSpeed()을 깨움
                break;
            }
            default:
            {
                LogDebug << "Unhandled command: <" << Qt::hex << content.first << ", " << content.second.toHex(' ').toUpper() << ">";
                break;
            }
            }
#pragma GCC diagnostic pop
        }
    }

    void MyCobot::HandleTimeout()
    {
        // serial port timeout
    }

    void MyCobot::HandleError(QSerialPort::SerialPortError error)
    {
        // WriteError는 통신 중 일시적으로 발생할 수 있음. 로그만 남겨도 충분.
        if (error == QSerialPort::SerialPortError::WriteError)
        {
            LogError << "A write error occurred on the serial port.";
        }
        // ResourceError는 보통 USB 연결이 물리적으로 끊기는 등의 심각한 문제.
        else if (error == QSerialPort::ResourceError)
        {
            LogError << "A resource error occurred. The device may have been disconnected.";

            // 연결이 끊어졌으므로, 내부 상태도 '연결 끊김'으로 확실히 변경.
            if (IsCncConnected()) // IsCncConnected는 serial_port->isOpen()을 확인
            {
                // StateOff() 대신 역할이 명확해진 Disconnect()를 호출
                Disconnect();

                // 에러 상태를 외부에 알리기 위한 처리 (예: 시그널 발생 또는 상태 변수 설정)
                // next_error 멤버 변수 대신, 더 명시적인 방법을 사용할 수 있음
                m_last_error_string = "Device removed or became unavailable. Please, check connection.";
                emit connectionLost(); // 연결 끊김을 알리는 시그널 (추천)
            }
        }
        // 기타 다른 에러에 대한 처리도 추가할 수 있음
        else if (error != QSerialPort::NoError)
        {
            LogError << "An unhandled serial port error occurred: " << error;
        }
    }

    void MyCobot::SetFreshMode(int mode)
    {
        // 명령어: [HEADER, HEADER, LEN(3), CMD(0x16), mode, FOOTER]
        QByteArray command;
        command.append(FIRMATA_HEADER);
        command.append(char(3)); // Length
        command.append(char(Command::SetFreshMode));
        command.append(char(mode));
        command.append(FIRMATA_FOOTER);
        SerialWrite(command);
    }

    std::vector<std::pair<unsigned char, QByteArray>> MyCobot::Parse(QByteArray &data)
    {
        std::vector<std::pair<unsigned char, QByteArray>> parsed_commands;
        while (data.size() >= 4)
        {
            int head_idx = data.indexOf(FIRMATA_HEADER);
            if (head_idx == -1)
            {
                data.clear();
                return parsed_commands;
            }

            if (head_idx > 0)
            {
                data.remove(0, head_idx);
            }

            if (data.size() < 3)
            { // 헤더(2) + 길이(1) 필드까지는 있어야 함
                break;
            }

            // 데이터 길이(LEN) 필드 읽기
            const int len_field = static_cast<unsigned char>(data[2]);
            // ★★★ 전체 패킷 길이는 (LEN 필드 값 + 3) 이 올바른 계산입니다. ★★★
            // 헤더(2) + 길이필드(1) + 명령어(1) + 데이터(N) + 푸터(1)
            // Python 소스 기준: L = N + 2 이므로, Total = 2+1+1+N+1 = 2+1+1+(L-2)+1 = L+3
            const int total_packet_len = len_field + 3;

            if (data.size() < total_packet_len)
            {
                // 아직 패킷이 다 도착하지 않음
                break;
            }

            if (static_cast<unsigned char>(data[total_packet_len - 1]) == 0xFA)
            {
                // 유효한 패킷 발견!
                unsigned char cmd_id = static_cast<unsigned char>(data[3]);
                // 실제 데이터 (명령어 ID 다음부터 푸터 전까지)
                QByteArray command_data = data.mid(4, len_field - 2);

                parsed_commands.push_back({cmd_id, command_data});

                // 처리된 패킷을 버퍼에서 제거
                data.remove(0, total_packet_len);
            }
            else
            {
                // 푸터가 일치하지 않으면 헤더가 잘못된 것으로 간주하고 한 바이트만 버림
                data.remove(0, 1);
            }
        }
        return parsed_commands;
    }

    // void MyCobot::SetFreeMove(bool on)
    // {
    //     LogTrace << "(" << on << ")";
    //     std::lock_guard<std::mutex> lock(command_mutex);
    //     if (!command_bits.test(Command::SetFreeMoveMode))
    //     {
    //         std::function<void()> f = [this, on]()
    //         {
    //             QByteArray command(CommandSetFreeMoveMode);
    //             command += static_cast<char>(on);
    //             command += FIRMATA_FOOTER;
    //             SerialWrite(command);
    //         };
    //         command_bits.set(Command::SetFreeMoveMode);
    //         commands.push_back(std::make_pair(Command::SetFreeMoveMode, f));
    //     }
    // }

    // bool MyCobot::IsSoftwareFreeMove() const
    // {
    //     // LogTrace;
    //     std::lock_guard<std::mutex> lock(command_mutex);
    //     if (!command_bits.test(Command::IsFreeMoveMode))
    //     {
    //         std::function<void()> f = [this]()
    //         {
    //             SerialWrite(CommandIsFreeMoveMode);
    //         };
    //         command_bits.set(Command::IsFreeMoveMode);
    //         commands.push_back(std::make_pair(Command::IsFreeMoveMode, f));
    //     }
    //     return is_free_move;
    // }
}
