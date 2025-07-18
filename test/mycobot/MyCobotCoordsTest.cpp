/**
 * @file high_level_api_test.cpp
 * @brief 고수준 MyCobot C++ API를 사용한 최종 제어 테스트
 *
 * 사용법:
 * ./high_level_api_test <J1> <J2> <J3> <J4> <J5> <J6>
 * 예시: ./high_level_api_test 0 0 0 0 0 90
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <QCoreApplication>

// 우리가 만든 고수준 API와 유틸리티 함수를 포함합니다.
#include "mycobot/MyCobot.hpp"

int main(int argc, char *argv[])
{
  // 1. 명령어 인자 확인
  if (argc != 7)
  {
    std::cerr << "사용법: " << argv[0] << " <J1> <J2> <J3> <J4> <J5> <J6>" << std::endl;
    std::cerr << "예시: " << argv[0] << " 0 0 0 0 0 90" << std::endl;
    return 1;
  }

  // Qt 애플리케이션 객체 생성
  QCoreApplication app(argc, argv);
  std::cout << "===== 고수준 API 최종 제어 테스트 시작 =====" << std::endl;

  try
  {
    // 2. 명령어 인자를 목표 각도로 변환
    mycobot::Coords target_coords;
    for (int i = 0; i < mycobot::Axes; ++i)
    {
      target_coords[i] = std::stod(argv[i + 1]);
    }

    // 3. 로봇 인스턴스 가져오기 및 초기화
    auto robot = mycobot::MyCobot::I();

    // 4. 로봇 전원 인가 및 설정
    std::cout << "로봇 전원 인가 및 모드 설정..." << std::endl;
    robot.PowerOn();
    mycobot::wait(2000); // 서보가 맞물릴 시간을 줍니다.
    robot.SetFreshMode(1);
    mycobot::wait(100);
    robot.InitialPose(30);
    mycobot::wait(5000);

    // 5. 목표 각도로 이동 명령 전송
    std::cout << "\n목표 각도로 이동 시작!" << std::endl;
    robot.WriteCoords(target_coords, 30);

    // 6. 로봇이 움직이는 동안 10Hz로 상태를 관찰 (최대 5초)
    std::cout << "--------------------------------------------------------" << std::endl;
    for (int i = 0; i < 50; ++i) // 5초 동안 10Hz (100ms) 루프
    {
      // a. 데이터 요청 (비동기)
      robot.RequestCoords();
      mycobot::wait(10); // ★★★ 요청 사이에 10ms 여유 시간 ★★★
      robot.RequestSpeeds();

      // b. 현재 캐시된 데이터 조회 (Peek)
      mycobot::Coords current_coords = robot.PeekCoords();
      mycobot::IntAngles current_speeds = robot.PeekSpeeds();

      // c. 현재 상태 출력
      std::cout << "Time: " << std::setw(4) << i * 100 << "ms | ";
      std::cout << "Coords: [";
      for (size_t j = 0; j < current_coords.size(); ++j)
      {
        std::cout << std::setw(6) << std::fixed << std::setprecision(1) << current_coords[j] << (j < 5 ? "," : "");
      }
      std::cout << "] | Speeds: [";
      for (size_t j = 0; j < current_speeds.size(); ++j)
      {
        std::cout << std::setw(5) << current_speeds[j] << (j < 5 ? "," : "");
      }
      std::cout << "]" << std::endl;

      // d. 로봇이 움직임을 멈췄는지 확인 (선택적)
      // if (!robot.PeekIsMoving()) {
      //     std::cout << "목표 지점 도착, 모니터링을 종료합니다." << std::endl;
      //     break;
      // }

      // e. 10Hz 주기 맞춤 및 이벤트 처리
      mycobot::wait(90);
    }

    std::cout << "\n테스트 완료." << std::endl;
  }
  catch (const std::exception &e)
  {
    std::cerr << "\n오류 발생: " << e.what() << std::endl;
    return -1;
  }

  std::cout << "\n===== 테스트가 성공적으로 완료되었습니다. ===== " << std::endl;
  return 0;
}
