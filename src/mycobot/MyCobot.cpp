#include "mycobot/MyCobot.hpp"

#include <system_error>
#include <QCoreApplication>
#include <QTime> // process_events_for 함수를 위해 포함

// 저수준 API의 헤더 파일을 포함합니다.
#include "MyCobot.hpp"

namespace mycobot
{
    /**
     * @brief PIMPL(Private Implementation) 패턴을 위한 내부 구현 클래스.
     * 현재는 비어있지만, 나중에 래퍼 클래스에 특화된 상태를 저장하는 데 사용할 수 있습니다.
     */
    class MYCOBOTCPP_LOCAL MyCobotImpl
    {
    };

    // ★★★ 네임스페이스 안에 이 함수 구현을 추가합니다. ★★★
    void wait(int milliseconds)
    {
        if (milliseconds <= 0)
            return;

        QTime dieTime = QTime::currentTime().addMSecs(milliseconds);
        while (QTime::currentTime() < dieTime)
        {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        }
    }

    /**
     * @brief MyCobot 래퍼 클래스의 싱글톤 인스턴스를 가져옵니다.
     * 최초 호출 시, 저수준 rc::MyCobot API를 초기화하고 연결합니다.
     */
    MyCobot MyCobot::I()
    {
        static MyCobot singleton{};
        if (!singleton.impl)
        {
            auto impl = std::make_shared<MyCobotImpl>();

            try
            {
                // 저수준 API의 Init()을 호출합니다.
                // Init()은 이제 성공 시 0을 반환하고, 실패 시 예외를 던집니다.
                // 따라서 반환 값을 체크하는 로직은 더 이상 필요 없습니다.
                rc::MyCobot::Instance().Init();
            }
            // ★★★ 저수준 API가 던지는 예외를 여기서 잡습니다 ★★★
            catch (const std::system_error &e)
            {
                // 사용자에게 더 친숙한 고수준 예외로 포장하여 다시 던진다.
                // e.what()에는 포트 이름과 구체적인 오류 메시지가 이미 포함되어 있습니다.
                throw InitializationException("Robot connection failed: " + std::string(e.what()));
            }
            catch (const std::exception &e) // 그 외 예상치 못한 예외 처리
            {
                throw InitializationException("An unexpected error occurred during robot initialization: " + std::string(e.what()));
            }

            // 예외 없이 여기까지 왔다면 초기화가 성공한 것.
            singleton.impl = impl;
        }

        return singleton;
    }

    // ==========================================================
    // 기본 제어 (명령 전송)
    // ==========================================================

    void MyCobot::PowerOn()
    {
        try
        {
            rc::MyCobot::Instance().PowerOn();
        }
        catch (const std::exception &e)
        {
            // "PowerOn command failed: " 라는 맥락을 추가하여 다시 던진다.
            throw CommandException("PowerOn command failed: " + std::string(e.what()));
        }
    }

    void MyCobot::PowerOff()
    {
        try
        {
            // PowerOff는 내부적으로 ReleaseAllServos를 호출
            rc::MyCobot::Instance().ReleaseAllServos();
        }
        catch (const std::exception &e)
        {
            throw CommandException("PowerOff (ReleaseAllServos) command failed: " + std::string(e.what()));
        }
    }

    void MyCobot::StopRobot()
    {
        try
        {
            rc::MyCobot::Instance().TaskStop();
        }
        catch (const std::exception &e)
        {
            throw CommandException("StopRobot (TaskStop) command failed: " + std::string(e.what()));
        }
    }

    void MyCobot::SetFreshMode(int mode)
    {
        try
        {
            rc::MyCobot::Instance().SetFreshMode(mode);
        }
        catch (const std::exception &e)
        {
            throw CommandException("SetFreshMode command failed: " + std::string(e.what()));
        }
    }

    void MyCobot::InitialPose(int speed)
    {
        try
        {
            Angles initial_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            rc::MyCobot::Instance().WriteAngles(initial_pose, speed);
        }
        catch (const std::exception &e)
        {
            throw CommandException("InitialPose command failed: " + std::string(e.what()));
        }
    }

    // ==========================================================
    // 위치/각도 제어 (명령 전송)
    // ==========================================================

    void MyCobot::WriteAngles(const Angles &angles, int speed)
    {
        try
        {
            rc::MyCobot::Instance().WriteAngles(angles, speed);
        }
        catch (const std::exception &e)
        {
            // "WriteAngles command failed: " 라는 맥락을 추가하여 다시 던진다.
            throw CommandException("WriteAngles command failed: " + std::string(e.what()));
        }
    }

    void MyCobot::WriteAngle(Joint joint, double value, int speed)
    {
        try
        {
            // enum 타입을 저수준 API에 맞게 캐스팅하는 부분은 그대로 유지합니다.
            rc::MyCobot::Instance().WriteAngle(static_cast<rc::Joint>(joint), value, speed);
        }
        catch (const std::exception &e)
        {
            throw CommandException("WriteAngle command failed: " + std::string(e.what()));
        }
    }

    void MyCobot::WriteCoords(const Coords &coords, int speed, int mode)
    {
        try
        {
            rc::MyCobot::Instance().WriteCoords(coords, speed, mode);
        }
        catch (const std::exception &e)
        {
            throw CommandException("WriteCoords command failed: " + std::string(e.what()));
        }
    }

    void MyCobot::WriteCoord(Axis axis, double value, int speed)
    {
        try
        {
            rc::MyCobot::Instance().WriteCoord(static_cast<rc::Axis>(axis), value, speed);
        }
        catch (const std::exception &e)
        {
            throw CommandException("WriteCoord command failed: " + std::string(e.what()));
        }
    }

    // ==========================================================
    // 실시간 데이터 요청 (비동기)
    // ==========================================================

    void MyCobot::RequestCoords()
    {
        try
        {
            rc::MyCobot::Instance().RequestCoords();
        }
        catch (const std::exception &e)
        {
            // "Request for angles failed: " 라는 명확한 맥락을 추가합니다.
            throw CommandException("Request for coords failed: " + std::string(e.what()));
        }
    }

    void MyCobot::RequestAngles()
    {
        try
        {
            rc::MyCobot::Instance().RequestAngles();
        }
        catch (const std::exception &e)
        {
            // "Request for angles failed: " 라는 명확한 맥락을 추가합니다.
            throw CommandException("Request for angles failed: " + std::string(e.what()));
        }
    }

    void MyCobot::RequestSpeeds()
    {
        try
        {
            rc::MyCobot::Instance().RequestSpeeds();
        }
        catch (const std::exception &e)
        {
            throw CommandException("Request for speeds failed: " + std::string(e.what()));
        }
    }

    void MyCobot::RequestJointLoad(Joint joint)
    {
        try
        {
            rc::MyCobot::Instance().RequestJointLoad(static_cast<rc::Joint>(joint));
        }
        catch (const std::exception &e)
        {
            throw CommandException("Request for load failed: " + std::string(e.what()));
        }
    }

    // ==========================================================
    // 캐시된 데이터 조회 (읽기)
    // ==========================================================

    Angles MyCobot::PeekAngles() const
    {
        return rc::MyCobot::Instance().PeekAngles();
    }

    Coords MyCobot::PeekCoords() const
    {
        return rc::MyCobot::Instance().PeekCoords();
    }

    IntAngles MyCobot::PeekSpeeds() const
    {
        return rc::MyCobot::Instance().PeekSpeeds();
    }

    int MyCobot::PeekJointLoad(Joint joint) const
    {
        return rc::MyCobot::Instance().PeekJointLoad(static_cast<rc::Joint>(joint));
    }

    // std::map<Joint, int> MyCobot::GetJointsLoads(const std::vector<Joint> &joints) const
    // {
    //     // 결과를 저장할 맵을 생성합니다.
    //     std::map<Joint, int> loads_map;

    //     try
    //     {
    //         // 사용자가 요청한 각 관절에 대해서만 순차적으로 부하를 읽어옵니다.
    //         for (const Joint &joint : joints)
    //         {
    //             // ★★★ 더 추상화된 저수준 함수를 호출합니다. ★★★
    //             int load_value = rc::MyCobot::Instance().GetJointLoad(
    //                 static_cast<rc::Joint>(joint));

    //             // 결과 맵에 저장합니다.
    //             loads_map[joint] = load_value;
    //         }
    //     }
    //     catch (const std::exception &e)
    //     {
    //         // 저수준 API에서 발생한 예외를 더 구체적인 정보와 함께 다시 던집니다.
    //         throw StateCheckException("Failed to get joint loads: " + std::string(e.what()));
    //     }

    //     return loads_map;
    // }

    // ==========================================================
    // 그리퍼 제어
    // ==========================================================

    void MyCobot::SetGriper(int open)
    {
        try
        {
            rc::MyCobot::Instance().SetGriper(open);
        }
        catch (const std::exception &e)
        {
            throw CommandException("SetGriper Command failed: " + std::string(e.what()));
        }
    }

} // namespace mycobot
