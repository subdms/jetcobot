#include "Firmata.hpp"

#define log_category ::rc::log::robot_controller
#include "log/Log.hpp"

namespace rc
{

    std::pair<int, int> RETURNED_COMMAND_SIZES[256] = {{0, 0}};

    void InitFirmata()
    {
        RETURNED_COMMAND_SIZES[Undefined] = {0, 0};
        RETURNED_COMMAND_SIZES[PowerOn] = {0, 0};
        RETURNED_COMMAND_SIZES[PowerOff] = {0, 0};
        RETURNED_COMMAND_SIZES[IsPoweredOn] = {1, 1};
        RETURNED_COMMAND_SIZES[ReleaseAllServos] = {0, 0};
        RETURNED_COMMAND_SIZES[IsControllerConnected] = {1, 1};
        RETURNED_COMMAND_SIZES[ReadNextError] = {0, 0};
        RETURNED_COMMAND_SIZES[SetFreeMoveMode] = {0, 0};
        RETURNED_COMMAND_SIZES[IsFreeMoveMode] = {1, 1};
        RETURNED_COMMAND_SIZES[GetAngles] = {12, 12};
        RETURNED_COMMAND_SIZES[GetEncoders] = {12, 12};
        RETURNED_COMMAND_SIZES[SetEncoders] = {0, 0};
        RETURNED_COMMAND_SIZES[WriteAngle] = {0, 0};
        RETURNED_COMMAND_SIZES[WriteAngles] = {0, 0};
        RETURNED_COMMAND_SIZES[GetCoords] = {12, 12};
        RETURNED_COMMAND_SIZES[WriteCoord] = {0, 0};
        RETURNED_COMMAND_SIZES[WriteCoords] = {0, 0};
        RETURNED_COMMAND_SIZES[ProgramPause] = {0, 0};
        RETURNED_COMMAND_SIZES[IsProgramPaused] = {1, 1};
        RETURNED_COMMAND_SIZES[ProgramResume] = {0, 0};
        RETURNED_COMMAND_SIZES[TaskStop] = {0, 0};
        RETURNED_COMMAND_SIZES[IsInPosition] = {1, 1};
        RETURNED_COMMAND_SIZES[CheckRunning] = {1, 1};
        RETURNED_COMMAND_SIZES[JogAngle] = {0, 0};
        RETURNED_COMMAND_SIZES[JogAbsolute] = {0, 0};
        RETURNED_COMMAND_SIZES[JogCoord] = {0, 0};
        RETURNED_COMMAND_SIZES[SendJogIncrement] = {0, 0};
        RETURNED_COMMAND_SIZES[JogStop] = {0, 0};
        RETURNED_COMMAND_SIZES[SetEncoder] = {0, 0};
        RETURNED_COMMAND_SIZES[GetSpeed] = {1, 1};
        RETURNED_COMMAND_SIZES[SetSpeed] = {0, 0};
        RETURNED_COMMAND_SIZES[GetFeedOverride] = {0, 0};
        RETURNED_COMMAND_SIZES[SendFeedOverride] = {0, 0};
        RETURNED_COMMAND_SIZES[GetAcceleration] = {0, 0};
        RETURNED_COMMAND_SIZES[SetAcceleration] = {0, 0};
        RETURNED_COMMAND_SIZES[GetJointMin] = {3, 3};
        RETURNED_COMMAND_SIZES[GetJointMax] = {3, 3};
        RETURNED_COMMAND_SIZES[SetJointMin] = {0, 0};
        RETURNED_COMMAND_SIZES[SetJointMax] = {0, 0};
        RETURNED_COMMAND_SIZES[IsServoEnabled] = {2, 2};
        RETURNED_COMMAND_SIZES[IsAllServoEnabled] = {1, 1};
        RETURNED_COMMAND_SIZES[SetServoData] = {0, 0};
        RETURNED_COMMAND_SIZES[GetServoData] = {1, 1};
        RETURNED_COMMAND_SIZES[SetServoCalibration] = {0, 0};
        RETURNED_COMMAND_SIZES[JointBrake] = {0, 0};
        RETURNED_COMMAND_SIZES[SetDigitalOut] = {2, 2};
        RETURNED_COMMAND_SIZES[GetDigitalIn] = {2, 2};
        RETURNED_COMMAND_SIZES[GripperMode] = {0, 0};
        RETURNED_COMMAND_SIZES[SetLedRgb] = {0, 0};
        RETURNED_COMMAND_SIZES[SetBasicOut] = {2, 2};
        RETURNED_COMMAND_SIZES[GetBasicIn] = {2, 2};

        // 속도, 전류, 전압 등은 6개 관절에 대한 2바이트 값, 총 12바이트를 반환합니다.
        RETURNED_COMMAND_SIZES[GET_SERVO_SPEEDS] = {12, 12};
        RETURNED_COMMAND_SIZES[GET_SERVO_CURRENTS] = {12, 12};
        RETURNED_COMMAND_SIZES[GET_SERVO_VOLTAGES] = {12, 12};
        RETURNED_COMMAND_SIZES[GET_SERVO_STATUS] = {12, 12};
        RETURNED_COMMAND_SIZES[GET_SERVO_TEMPS] = {12, 12};
    }

    /**
     * @brief Make command to be expected size, so that during parsing there is
     * no need for size checks.
     * If returned command parameters have size bigger than max
     * allowed size for given command, shrink extra parameters.
     * If returned command parameters size is smaller that min size for given
     * command, add missing parameters (initialized to zero).
     * @arg[in,out] command the argument is modified in place, if needed.
     */
    void FixupCommands(std::vector<std::pair<unsigned char, QByteArray>> &commands)
    {
        for (auto &command : commands)
        {
            if (command.second.size() < RETURNED_COMMAND_SIZES[command.first].first)
            {
                command.second.append(RETURNED_COMMAND_SIZES[command.first].first - command.second.size(), 0);
                LogTrace << ": not enough data, appending";
            }
            else if (command.second.size() > RETURNED_COMMAND_SIZES[command.first].first)
            {
                command.second.resize(RETURNED_COMMAND_SIZES[command.first].first);
                LogTrace << ": too much data, removing excess data";
            }
        }
    }

}
