# A → B → C → D 순차 실험

모든 명령은 서버 터미널에서 실행한다. 각 터미널에서 먼저 다음 경로로 이동한다.

```bash
cd /home/ubuntu/tjdbsk/dmpc_workspace/dmpc
```

한 번에 시나리오 하나만 실행한다. 시뮬레이터·경로 뷰어·테스트의 시나리오 이름은
모두 같아야 한다. `test`가 MPC/RPP/Swerve를 직접 실행하므로 별도로 `drive`를
함께 실행하지 않는다. 모든 실험 명령은 터미널에서 실행되며 백그라운드 주행을
자동으로 남기지 않는다.

## 공통 설정과 장면

| 시나리오 | 로봇 A 시작 → 목표 | 로봇 B 시작 → 목표 | 환경과 관찰점 |
|---|---|---|---|
| A | (-3,0,0) → (3,0,0) | (3,0,π) → (-3,0,π) | 정면 접근. 회피 방향·최소 간격·목표 도착 |
| B | (-3,0,0) → (3,0,0) | (0,-3,π/2) → (0,3,π/2) | 교차 주행. 교차점 회피·감속·목표 도착 |
| C | A와 같음 | A와 같음 | x=[-4,4], y=[-0.7,0.7] 벽. 통로 내 안전 정지·정체 |
| D | A와 같음 | A와 같음 | 원점에 반경 0.4 m 고정 원통. 상대 로봇과 원통 동시 회피 |

좌표는 (x[m],y[m],yaw[rad])다. 최대 속도 1.0 m/s, 가속도 0.5 m/s²,
최소 로봇 중심 간격 1.15 m, 위치 도착 허용 오차 0.2 m다.
목표 근처에서는 감속도 0.25 m/s²·응답 여유 0.3초·정지 여유 0.08 m로 계산한
제동 거리 제한도 적용한다. 최대 속도와 도착 기준은 유지하며 자세한 근거는
[테스트 결과의 목표 정지 수정 내용](SCENARIO_TEST_RESULTS.md#goal-stop-fix)에 있다.

C의 로봇 중심 허용 y 범위는 로봇 반경·벽 여유를 빼면 [-0.15,0.15] m다.
상호 간격 1.15 m를 유지한 채 옆으로 지나갈 공간이 없다. 현재 복구 기능으로
통과를 보장하지 않으며, 제한시간 종료는 `미완료`로 기록한다.

## 사전 확인

공유 Zenoh 라우터가 이미 실행 중이므로 중복 실행하지 않는다. 라우터가 없는
새 환경에서만 `bash run_isaac.sh router`를 사용한다.

```bash
bash run_isaac.sh check --scenario A
bash run_isaac.sh check --scenario B
bash run_isaac.sh check --scenario C
bash run_isaac.sh check --scenario D
```

이번 준비에서는 기존 Docker 이미지와 빌드 산출물을 사용한다. 소스가 설치 경로에
연결돼 있어 Python·YAML 변경에 ROS 재빌드나 이미지 재빌드는 필요하지 않다.
생성 메시지·C++ 구현이 바뀌거나 새 환경을 설치할 때는 `bash run_isaac.sh build`를 사용한다.

## A 실행

터미널 1 — Isaac을 시작하고 `[State]`와 `[WebRTC]` 출력까지 기다린다.

```bash
bash run_isaac.sh sim --scenario A
```

터미널 2 — 경로 뷰어를 실행한다.

```bash
bash run_isaac.sh viewer --scenario A
```

터미널 3 — 주행과 결과 기록을 시작한다.

```bash
bash run_isaac.sh test A --timeout 120
```

- 물리 화면: <http://118.216.110.114:8212>
- 경로 화면: <http://118.216.110.114:8092>
- 원격 PC에서 TCP 8212/8092/62002 및 WebRTC UDP 62000에 접근할 수 있어야 한다.
- 시나리오를 바꿀 때 `bash run_isaac.sh stop-sim`으로 현재 Isaac 종료가 끝난 뒤
  `bash run_isaac.sh sim --scenario C` 또는 `D`로 시작한다. 주행 시험도 먼저 종료한다.
  새 스트리밍 포트 설정을 받도록 기존 영상 탭을 닫고 8212 주소를 새로 연다.
- `test`는 현재 Isaac의 시나리오 파라미터와 정지 상태·시작 위치·방향을 확인한 뒤
  주행을 시작한다. 이미 이동했으면 Isaac을 종료하고 초기 위치로 재시작한다.

두 로봇 도착, 120초의 벽시계 제한시간 또는 `Ctrl+C`에서 기록을 마치고
해당 테스트가 띄운 주행 프로세스를 종료한다. 도착은 현재 계획기와 같은
위치·전진 속도 조건에 더해 실측 평면 속도 정지와 유효 계획 수신을 확인한다.
목표 방향 오차는 별도로 기록한다. `timeout`은 시뮬레이션 초가 아니다.

진행 로그의 배열 순서는 `[robot_a, robot_b]`다. `goal_error_m`은 현재 목표까지
거리, `speed_mps`는 실제 평면 속력, `odom_age_s`는 마지막 위치 입력 수신 후
경과시간이다. 도착하려면 같은 시점에 목표 거리 ≤0.2 m, 속력 ≤0.05 m/s,
입력 수신 나이 ≤0.35 s를 만족해야 한다. `odom_pairs` 증가는 입력 수신을 뜻하며
로봇이 움직이고 있거나 목표에 도착했다는 뜻은 아니다.

종료 후 결과를 확인하고 터미널 1과 2에서 `Ctrl+C`로 시뮬레이터·뷰어를 종료한다.

## B, C, D 실행

각 행의 세 명령을 별도 터미널에서 순서대로 실행한다. 직전 장면과 뷰어를 먼저 종료한다.

| 순서 | 터미널 1: Isaac | 터미널 2: 경로 뷰어 | 터미널 3: 주행+기록 |
|---|---|---|---|
| B | `bash run_isaac.sh sim --scenario B` | `bash run_isaac.sh viewer --scenario B` | `bash run_isaac.sh test B --timeout 120` |
| C | `bash run_isaac.sh sim --scenario C` | `bash run_isaac.sh viewer --scenario C` | `bash run_isaac.sh test C --timeout 120` |
| D | `bash run_isaac.sh sim --scenario D` | `bash run_isaac.sh viewer --scenario D` | `bash run_isaac.sh test D --timeout 120` |

## 결과 파일

결과는 **[docs/SCENARIO_TEST_RESULTS.md](SCENARIO_TEST_RESULTS.md) 하나에만** 저장한다.
매 실행이 종료되면 이 파일에 요약 행과 상세 결과가 추가된다. 시각은 KST이며
이전 실행과 수동 메모는 유지한다. 개별 실행 폴더·JSON·로그·telemetry 파일은 만들지 않는다.
주행 프로세스 출력은 메모리에서 처리하고, 오류와 필요한 마지막 메시지만 같은 MD에 남긴다.
도착 여부·최소 간격·실측 최대 속도·벽 및 원통 여유·방향 오차·상태 수신 횟수를 기록한다.
실제 접촉 센서는 수집하지 않으므로 관측 간격만으로 물리적 무접촉을 확정하지 않는다.
시뮬레이션 시간 초기화·입력 없음·장면 불일치도 성공으로 처리하지 않는다.

`test`의 종료코드는 두 로봇 도착 및 관측된 기준 위반이 없으면 0, 나머지는 1이다.
종료 정리 또는 자식 프로세스 오류가 있으면 도착했더라도 종료코드는 1이다.
이 종료코드는 접촉 안전성이나 최종 방향 정렬을 인증하지 않는다.
`SIGKILL`, 호스트 종료, Docker 강제 삭제처럼 정상 종료 절차가 실행되지 않으면
완료 결과가 MD에 추가되지 않을 수 있으므로 일반 중단에는 `Ctrl+C`를 사용한다.

## 수동 주행만 할 때

자동 기록 없이 실행할 때는 `test` 대신 다음 명령을 사용한다.

```bash
bash run_isaac.sh drive scenario:=A
```

이 경우 테스트 결과 문서는 자동으로 갱신되지 않는다.
