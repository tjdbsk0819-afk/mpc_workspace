# Gazebo AntBot: 3·4대 A/B/C/D

Gazebo Harmonic + ROS 2 Jazzy. 기본은 headless 물리 시뮬레이션이며 기존
MPC/DMPC → native RPP → Swerve → topic hardware interface 구조를 사용한다.
`antbot_isaac_control/IsaacTopicSystem` 이름은 호환성을 위해 유지했지만,
관절 명령을 적용하고 상태·피드백을 발행하는 시뮬레이터는 Gazebo다.
Isaac Sim, USD, Isaac 컨테이너는 새 실행 경로에서 사용하지 않는다.

## 현재 예측 설정 — 2026-11-04

현재 Gazebo 실행 설정은 **N=30, dt=0.2초, 예측 시간 6초**다.
시나리오 A·3대의 2초·4초·기존 6초 측정 결과는 다음 파일에 통합했다.
`/home/tjdbsk/workspaces/mpc_planner_workspace/mpc_planner_timing_analysis.md`
보고서의 2초·4초·6초 수치는 각 실험 당시 이력이다. 2026-11-04에는 실행 설정만 6초로 변경했다.

## 실행

이 문서의 상위 디렉터리(`run_gazebo.sh`가 있는 곳)에서 실행한다.
Docker 접근 권한이 필요한 환경은 각 명령 앞에 `sudo`를 붙인다.

```bash
bash run_gazebo.sh check --scenario A --robots 3
bash run_gazebo.sh build

# 터미널 1: Gazebo 물리 시뮬레이션
bash run_gazebo.sh sim --scenario B --robots 3
# 터미널 2: 제어기와 플래너
bash run_gazebo.sh drive --scenario B --robots 3

# 종료 후 동일한 방법으로 4대 실행
bash run_gazebo.sh stop
bash run_gazebo.sh sim --scenario D --robots 4
# 다른 터미널
bash run_gazebo.sh drive --scenario D --robots 4
```

`build`는 Docker 저장소와 소스 파일시스템에 각각 최소 10 GiB 여유 공간이
있는지 확인한다. 두 경로가 같으면 검사도 한 번만 한다. Gazebo와 acados의
빌드 사용량은 환경에 따라 더 클 수 있다. 호스트 Humble 환경을 source할
필요가 없으며 Jazzy 의존성은 컨테이너 안에 설치한다.
MPC 이미지의 기존 기반 Dockerfile은 옆의 `dmpc_workspace/dmpc` 소스를 사용해
`antbot-dmpc:jazzy`를 먼저 만든다. 실행 때에는 MPC 자신의 소스만 마운트한다.

로컬 X11 디스플레이가 준비된 환경에서는 별도 터미널에서
`bash run_gazebo.sh gui`로 같은 Gazebo 서버에 접속할 수 있다.
기본 headless 실행에는 디스플레이나 NVIDIA 런타임이 필요 없다.
기존 Isaac WebRTC 페이지는 Gazebo GUI로 연결되지 않는다.

## 자동 테스트와 기록

`sim`을 해당 시나리오/대수로 먼저 시작하고, 별도 `drive`는 실행하지 않는다.
`test`가 직접 drive 프로세스를 시작·종료한다. 각 경우 sim을 종료하고 새로
시작해야 초기 위치가 재설정된다.

```bash
bash run_gazebo.sh sim --scenario A --robots 3
# 다른 터미널
bash run_gazebo.sh test --scenario A --robots 3
```

A/B/C/D × 3/4대의 8개 조합을 같은 방법으로 실행한다. JSON과 제어기 로그는
`gazebo/results/`에 기록된다. JSON은 전체 로봇의 도착·정지 여부, 모든 로봇
쌍(3대: 3쌍 / 4대: 6쌍)의 최소 중심 거리와 시각이 맞는 표본 수를 포함한다.
`not_passed`는 비정상 종료·미도착·안전 거리 미달·표본 누락 등을 포함하므로
로그와 `final_statuses`를 함께 확인한다. 물리 접촉 센서 검증을 대신하지 않는다.
테스트 시작 시 초기 위치·방향을 확인하므로 이미 주행한 장면은 재시작한다.

## 시나리오 정의

`scenarios.py`가 spawn, goal, world, planner, RPP 모두의 기준이다.
기존 robot_a·robot_b의 시작·목표, C의 벽 경계, D의 원기둥을 유지했다.
3대는 a/b/c, 4대는 a/b/c/d를 사용한다.

| 시나리오 | 유지한 조건 | 추가 로봇 |
|---|---|---|
| A | x축 정면 교행, a: -3→3, b: 3→-3 | c: -6→6, d: 6→-6 |
| B | a: 서→동, b: 남→북 교차 | c: 동→서, d: 북→남 |
| C | x=[-4,4], y=[-0.7,0.7] 좁은 복도 | c: -1.5→1.5, d: 1.5→-1.5 |
| D | A와 같은 a/b, 원점 반경 0.4m 원기둥 | c/d는 A와 동일 |

C는 로봇이 나란히 교행할 수 없는 원래 폭을 유지한다. 정지·교착은 가능한
예상 결과이며 통과 성공으로 분류하지 않는다. 통행 우선순위·후진 양보 정책은
이번 전환에 추가하지 않았다. 다른 시나리오도 3/4대 도착 성공은 실제 검증 전이다.

## 제어·관측 계약

- 관절 명령: `/robot_*/joint_command`의 조향 position 4개 + 바퀴 velocity 4개.
- Gazebo 플러그인이 물리 관절 속도를 설정한다. 조향은 gain 25/s의 위치 추종,
  최대 5rad/s이며 위치 순간이동을 사용하지 않는다. 바퀴는 최대 50rad/s다.
  실제 모터의 토크·전류 포화 모델은 포함하지 않은 속도 구동 모델이다.
- 200Hz 물리, 50Hz odom/joint state/feedback, 기존 50Hz Swerve를 유지한다.
- `JointCommandFeedback`: 물리 스텝 후 관절 상태, 원래 수신 명령 stamp,
  Gazebo에 적용한 시각을 함께 발행한다. 재생된 명령은 watchdog을 갱신하지 않는다.
- `/robot_*/odom`: world pose, body twist. 바퀴 적분 결과는 `wheel_odom`으로 분리한다.
- `/tf`의 `world → robot_*/base_link`는 Gazebo 플러그인만 발행한다.
- `/clock`은 Gazebo에서 ROS로 단방향 bridge한다. pause/reset/입력 단절 시
  기존 정지 감시를 유지하며 Gazebo 연결부 자체에도 0.5초 명령 watchdog이 있다.
- 각 로봇은 나머지 모든 로봇을 `peers/<id>/state`로 관측한다. 하나라도 없거나
  오래되면 유효 계획을 발행하지 않는다. DMPC는 모든 상대의 제약을 IPOPT에
  함께 넣고, 결과를 모든 상대에 대해 독립 재검사한다.
- MPC의 solver는 상대 3개 슬롯으로 다시 생성한다. 3대 실행의 남은 슬롯은
  작업 공간 밖의 고정 dummy이며 4대는 3개 모두 실제 상대다. 단일 상대만
  검사하던 이전 계획 fallback은 다중 로봇 경로에서 사용하지 않는다.
- 기본 ROS domain: MPC 30 / DMPC 31. 새 Gazebo 경로는 Fast DDS를 사용하고
  기존 Isaac의 Zenoh router가 필요 없다. Gazebo partition도 두 워크스페이스를 분리한다.

## 검증 상태

Python 구문·shell 구문·Compose 구성과 8개 시나리오 자산의 오프라인 생성,
실제 CasADi/IPOPT의 다중 상대 최적화·검증 및 MPC의 세 번째 상대·벽·정적
장애물 제약 회귀 테스트(새 테스트 총 7개)를 수행했다.
기존 DMPC 전체 테스트의 ROS 빌드 링크 검사 1건은 원본 워크스페이스에 빌드
산출물이 없어 실패했다. Gazebo 플러그인/C++ MPC 전체 빌드와 물리 주행은
디스크 공간 부족으로 아직 수행하지 못했다. `check`는 URDF→SDF 변환이나
Gazebo 실행 성공을 의미하지 않는다. 여유 공간 확보 후 `build`와 8개 조합의
물리 테스트가 남아 있다.

## 최대 속도와 경로 화면

기본 최대 속도는 1.5 m/s다. MPC의 목표 기준 속도와 solver 상한,
DMPC 최대 속도, native RPP/Nav2 목표 속도를 함께 맞췄다.
가속도·회피·목표 접근에 따라 실제 속도는 낮아질 수 있다.

`bash run_gazebo.sh view --scenario B --robots 3`으로 경로 화면을 시작한다.
MPC는 http://127.0.0.1:8091, DMPC는 http://127.0.0.1:8092다.
4대는 `--robots 4`를 사용한다. sim과 drive가 실행되면 실제 위치·계획·상대
예측 경로가 표시된다. view 자체는 시뮬레이터나 주행을 시작하지 않는다.

## 고정 시점 · 로봇별 실측 속도

Gazebo GUI는 시나리오 전체를 내려다보는 고정 탑뷰를 사용한다. 마우스
드래그·휠로 카메라가 움직이지 않으며, 좌측 상단의 낮은 가로 패널에 로봇별 현재 속도(m/s)를
표시한다. 로봇 몸체와 속도 카드의 색상을 일치시켰다. 속도는 각 로봇의
odom에서 계산한 평면 속력이며, 입력이 0.6초 이상 끊기면 —로 표시한다.

웹 경로 화면도 시점 고정을 기본으로 사용하며 계획·예측이 바뀌어도 화면
중심과 배율이 바뀌지 않는다. 시점 고정을 해제하면 자동 맞춤을 사용할 수
있고, 고정 탑뷰 초기화로 원래 시점으로 돌아갈 수 있다.

네이티브 속도 패널만 다시 빌드하려면 `bash run_gazebo.sh build-gui`를 사용한다.
기존 전체 build에도 패널이 포함된다.

## Gazebo 화면의 경로 · 상대 예측

고정 탑뷰에 각 로봇의 `mpc_path`를 본체와 같은 색의 실선으로 표시한다.
각 플래너의 `mpc_peer_prediction/<peer>`는 예측 대상 로봇의 색상으로 된
점선이다. 상대 로봇이 발행한 계획을 예측으로 대체해 표시하지 않는다.
속도 패널의 MPC 경로 / 상대 예측 체크박스로 각각 켜고 끌 수 있다.
입력이 비었거나 world 좌표계가 아니거나 오래되면 해당 경로를 지운다.
GUI에만 표시하는 마커이므로 물리 모델이나 충돌 계산에는 영향을 주지 않는다.

한글 네모 현상은 이미지에 Noto CJK 글꼴을 설치하고 HUD에
`Noto Sans CJK KR`를 지정해 해결했다. Qt SVG 지원도 이미지에 포함했다.
새 설치에서는 기존 `build` 명령이 글꼴과 GUI 패널까지 구성한다.

목적지는 로봇 색상의 원+십자와 방향선으로 표시한다. 상대 예측의 점선과 Gaussian 불확실성 원은 예측 대상의 색상이다(A가 B를 예측하면 B 색상). cv_gaussian은 CV 평균과 Gaussian 위치 불확실성의 결합이며 별도의 두 평균 궤적이 아니다. 원은 실제 예측 데이터의 첫·중간·마지막 시점 불확실성 반경이며 로봇 크기를 더한 충돌 반경은 아니다.

## 연산 시간 측정

기존 주행을 실행한 상태에서 별도 터미널에 아래 명령을 실행한다.
이 명령은 관측만 하며 시뮬레이터나 주행을 시작하지 않는다.

```bash
bash run_gazebo.sh timing --robots 3
```

각 로봇의 `/robot_a/planning_timing`(b/c도 동일) JSON 토픽을 사용한다.
`solver_ms`는 한 번의 재계획에서 호출한 solver API의 실제 벽시계 시간 합계다.
MPC는 여러 초기 추정을 시도할 수 있으므로 `attempts`도 함께 확인한다.
`planning_ms`는 입력 처리·예측·초기 추정·solver·검증·결과 발행을 포함한다.
DMPC는 worker 대기 및 완료 결과를 ROS 타이머에서 회수하는 지연도 포함한다.
시뮬레이션 시간이 아닌 monotonic/steady clock을 사용한다.

평균/최댓값과 목표 주기(10Hz의 100ms) 초과 횟수를 누적한다.
실패하거나 만료되어 버린 계산과 첫 실행 초기화 시간도 포함하며,
solver를 호출하지 않은 대기·도착 상태는 제외한다.
`accepted_samples`는 실행 가능한 계획 또는 검증된 fallback을 발행한 횟수다.
평균은 로봇별 프로세스 시작 이후 누적값이며 시간 초과는 측정상 판정이다.
이 기능이 solver를 100ms에서 강제로 중단하거나 제어 주기를 바꾸지는 않는다.

플래너 실행 터미널에도 5초마다 요약을 출력한다.
`test` 명령으로 실행하면 결과 JSON의 `planning_timing`에 각 로봇의 최종
누적 통계를 저장한다. 토픽을 받지 못한 로봇의 통계는 결과에 존재하지 않으므로
정지 상태의 0ms 계산으로 해석하지 않는다.
C++ MPC 변경은 재빌드 후 반영된다. 이번 시간 측정 변경은 빌드·실행 검증하지 않았다.

속도 HUD는 로봇 3대 340×108px, 4대 450×108px의 가로형이다. 로봇 A/B/C/D는 같은 색상의 문자와 실측 속도로 표시하며, 계획 경로·상대 예측 토글은 유지한다.

## 디렉토리 구성

기존 워크스페이스의 `run_gazebo.sh` 명령은 그대로 사용한다.

| 디렉토리 | 역할 |
|---|---|
| `scripts/` | 컨테이너 초기화, 빌드, 시뮬레이터 실행 |
| `scene/` | A~D 시나리오 정의 및 Gazebo 모델·월드 생성 |
| `launch/` | 로봇별 플래너·제어기 주행 launch |
| `gui/` | GUI 실행, 고정 시점 설정, 브라우저 경로 뷰어 |
| `tools/` | 주행 준비 확인, 자동 주행 시험, 계산 시간 모니터 |
| `tests/` | 시나리오 보존 및 MPC 제약 검증 |
| `plugin/` | Gazebo ROS 연동 및 속도·경로 표시 플러그인 소스 |
| `build/` | 플러그인 빌드 산출물 |
| `generated/` | 실행 시 생성되는 모델·월드·장면 정보 |

`results/`는 시험을 실행할 때 결과를 저장하는 경로다. Docker 설정과 `workspace.env`는 이 디렉토리 최상위에 유지한다.


## 단일 로봇 MPC 계산 시간 측정

저장한 A/B/D 궤적으로 나머지 3대를 움직이며 1대만 최적화하는 방법은
[single_mpc/README.md](single_mpc/README.md)를 참조하세요.

```bash
bash run_gazebo.sh benchmark --scenario A --ego robot_a
```
