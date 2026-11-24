# 기존 mpc_planner와 현재 DMPC의 차이

기준: 2026-10-28 KST. 비교 대상은 `mpc_planner_workspace/isaac_mpc/two`의 정식
`two_antbot_driving.launch.py`와 현재 `dmpc_workspace/dmpc`의
`two_antbot_dmpc.launch.py`다. 기존 워크스페이스에 남아 있는 Python 실험 노드나
3대 로봇 구성은 이번 비교에 포함하지 않는다.

## 변경된 부분

| 항목 | 기존 mpc_planner / two | 현재 DMPC |
|---|---|---|
| 계획기 | C++ `mpc_planner_antbot/antbot_planner` | Python `dmpc_runtime.ros_node.DmpcNode` |
| 최적화 | 생성된 acados solver, SQP_RTI | CasADi Opti + IPOPT, 로봇별 작업 스레드 |
| 상태 | x, y, yaw, 전진 속도, 스플라인 진행도 | x, y, yaw, 전진 속도 |
| 입력 | 가속도, yaw rate | 가속도, yaw rate |
| 목적함수 | 스플라인의 contour/lag 오차, 속도, 입력, 도착·회전 연속성 비용 | 참조 위치, 목표 위치, 속도, 입력, 회전 변화, 종단 방향 비용 |
| 참조 경로 | 스플라인 기반 참조와 복구 후보 | 목표 방향 진행 경로와 필요 시 우측 회피 참조 |
| 정체 복구 | 정체 감지, 여러 초기 경로 후보 재최적화·선택, 검증된 이전 계획 재사용 | 이전 자기 해를 초기값으로 사용. 실패 시 초기값 초기화. 기존 다중 후보 복구 기능은 미이식 |
| 도착 판정 | 목표 위치·방향·정지 상태 및 유지 시간 | 위치 오차 0.2 m 이하와 전진 속도 절댓값 0.05 m/s 이하. 방향은 비용에만 반영 |
| 목표 감속 | 기존 목표 접근·제동 여유 모듈 | 실측 응답을 반영한 목표 제동 거리 제한(0.25 m/s², 응답 여유 0.3초, 정지 여유 0.08 m) 추가 |
| 상대 예측 | 기본 CV + Gaussian, CV 및 Gaussian random walk 선택 가능 | 기본 CV + Gaussian, CV 선택 가능 |
| 발행 정보 | 계획, 상대 예측, Gaussian 분포, 참조·후보 경로, 복구 상태 | 계획, 상대 예측 경로, 계획 상태 |
| 설정 | `mpc_planner_antbot/config/settings.yaml` 등 | `isaac/controller_bringup/config/dmpc.yaml` |
| ROS 도메인 | 기본 30 | 기본 31 |
| 영상 뷰어 | HTTP 8210 / 신호 TCP 49100 / 미디어 UDP 47998 | HTTP 8212 / 신호 TCP 62002 / 미디어 UDP 62000 |
| 경로 뷰어 | 기존 peer_prediction_viewer | 현재 저장소의 `dmpc_runtime.path_viewer`, HTTP 8092 |

현재 Isaac에서 실행되는 계획기는 `src/decentralized.py`를 직접 실행하는 방식이
아니라, AntBot ROS 입력·출력에 맞춘 `dmpc_runtime`의 구현이다.

## 공통으로 유지한 부분

- 로봇마다 자기 MPC가 있고, 상대 위치·속도 관측으로 미래 움직임을 예측한다.
  기존 시스템도 이 구조였다. 두 시스템 모두 상대 목표나 최적화 해를 받아서
  자기 계획을 계산하는 구조는 아니다.
- AntBot USD 모델, Isaac 장면 실행 코드, native RPP, Swerve 및 관절 브리지를
  기존 `two`에서 복사했다. 시나리오 식별 파라미터 등 실행·검증용 변경이 있다.
- 제어 경로는 `Isaac odom → 상대 상태 변환 → 로봇별 MPC → MpcPlan → RPP →
  cmd_vel → Swerve/ros2_control → 관절 브리지 → Isaac joint_command`다.
- GT 위치·속도와 `/clock`은 Isaac에서 제공한다. ROS 통신은 Zenoh를 사용한다.
- 계획 주기 10 Hz, 예측 간격 0.2 s, 30구간/6초, RPP 출력 20 Hz,
  ros2_control 50 Hz를 사용한다.
- 현재 두 구성 모두 최대 속도 1.0 m/s, 가속도 0.5 m/s², yaw rate 0.8 rad/s다.
  속도는 상한이며 회피·회전·목표 접근에서 낮아진다.
- 최소 중심 간격 1.15 m, 추종 여유 0.05 m와 Gaussian 예측 여유를 사용한다.
  기본 예측은 `평균 = 관측 위치 + 관측 속도 × 경과시간`,
  `sigma = 0.3 × sqrt(0.2 × 경과시간)`, risk=0.05다.

## 테스트 해석

최적화 도구뿐 아니라 목적함수·참조 경로·복구·도착 판정도 다르므로, 주행 차이를
acados와 IPOPT의 성능 차이로만 해석할 수 없다. 동일 조건의 성능 우열은 아직
측정하지 않았다. 특히 C의 좁은 통로에서는 안전 정지·정체와 목표 도착을 별도로
기록한다. 현재 구성에 다중 후보 복구나 방향 도착 판정이 이식된 것으로 보아서는 안 된다.

기존 뷰어를 현재 저장소에 복사·적응시켜 `bash run_isaac.sh viewer --scenario A`로
실행한다. 이전에 안내한 외부 워크스페이스 파일의 Docker 마운트는 더 이상 필요하지 않다.
현재 화면은 실제 위치·계획·상대 예측 평균과 선택한 시나리오의 벽/고정 장애물을
표시한다. 현재 계획기가 발행하지 않는 Gaussian 분포·복구 후보를 측정값으로 표시하지 않는다.

## 근거 코드

- [현재 계획기](../dmpc_runtime/planner.py)
- [현재 ROS 노드 및 도착 판정](../dmpc_runtime/ros_node.py)
- [현재 주행 launch](../isaac/controller_bringup/launch/two_antbot_dmpc.launch.py)
- [현재 속도·예측 설정](../isaac/controller_bringup/config/dmpc.yaml)
- [기존 코드 출처 및 해시](../isaac/vendor/manifest.json)
- 기존 계획기: `mpc_planner_workspace/isaac_mpc/two/mpc_planning/mpc_planner_antbot/`

테스트 순서는 [실행 안내](SCENARIO_TEST_GUIDE.md), 결과는
[테스트 결과](SCENARIO_TEST_RESULTS.md)에 관리한다.
실제 주행에서 발견한 목표 통과 후 정지 문제의 분석과 수정은
[테스트 결과의 목표 정지 수정 내용](SCENARIO_TEST_RESULTS.md#goal-stop-fix)에 정리했다.
