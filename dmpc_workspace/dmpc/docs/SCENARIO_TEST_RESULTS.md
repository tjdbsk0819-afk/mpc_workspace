# 시나리오 테스트 결과

모든 결과는 이 파일 하나에만 저장한다. `test A` 등의 명령이 끝나면 요약 표와
상세 기록이 추가된다. 날짜는 KST다. 기존 개별 테스트 폴더·JSON·로그·telemetry와
준비/디버그 산출물은 사용자 요청으로 삭제하며, 정량 결과와 분석을 이 문서에 통합했다.
앞으로도 별도 결과 폴더나 원본 로그 파일을 생성하지 않는다.

## 공통 설정과 판정

- 최대 속도 1.0 m/s, 모델 가속도 상한 0.5 m/s², 최소 중심 간격 1.15 m.
- 도착: 같은 시점에 목표 거리 ≤0.2 m, 실측 평면 속력 ≤0.05 m/s, 입력 수신 나이 ≤0.35초.
- 최종 방향 오차는 기록하지만 현재 도착 조건에는 포함하지 않는다.
- 거리·벽·원통 여유는 샘플링한 GT odom과 설정된 원형 근사로 계산한다.
  실제 접촉 센서를 수집한 결과나 샘플 사이의 안전성 보증은 아니다.
- 두 로봇 도착·제한시간·Ctrl+C에서 기록한다. 오류나 미완료를 성공으로 바꾸지 않는다.
- 종료 정리·자식 프로세스 오류도 이 MD에 남긴다. 주행 출력은 메모리에서만 처리한다.

<a id="webrtc-port-fix-20261029"></a>

## 스트리밍 시작 실패 점검 — 2026-10-29 KST

사용자 증상: C/D 전환 시 `49102 cannot be reserved`, B 재시작 시
`NVST_R_INVALID_OPERATION` / `Failed to start the primary stream server`가 반복됐지만
`[State]`는 계속 출력됐다. 물리 실행과 스트리밍 서버 시작이 별개여서 가능한 상태다.

09:12 점검에서 다른 Isaac 리스너는 없었고, 로컬 TCP 49102에서 외부
`104.18.32.47:443`으로 나갔던 HTTPS 연결의 `TIME_WAIT`가 발견됐다.
호스트 임시 송신 포트 범위는 32768–60999였고 예약 목록은 비어 있었다.
기존 신호 49102·영상 48000이 이 범위 안에 있어, 사전 포트 검사 통과 후에도
Kit 초기화 중 송신 연결이 해당 포트를 선점할 수 있는 구성이었다.
Linux의 예약 포트 동작은 [커널 문서](https://docs.kernel.org/networking/ip-sysctl.html#ip-variables)를 참고한다.

프로젝트 전용 포트를 **TCP 62002 / UDP 62000**으로 옮기고 웹 뷰어를 재빌드·재시작했다.
영상 페이지 주소는 `http://118.216.110.114:8212`다. 기존 탭을 닫고 다시 열어 새 번들을 받는다.
포트 변경은 모든 A/B/C/D 시나리오에 공통 적용된다.

확인 결과:

- 새 포트로 B를 시작해 Timeline playing·`[State]` 및 TCP 62002 리스닝 확인.
- 점검 시점 전체 실행 출력에서 `NVST Error`와 `StreamSdkException` 발생 0회.
- 배포된 웹 번들에 public IP와 신호 포트 62002 반영 확인.
- 사용자가 원격 PC에서 B 영상이 보인다고 확인했다.
- B 실행 중 D 중복 실행 요청이 기존 컨테이너 이름과 종료 안내를 표시하며 차단됨을 확인.
- 기존 자동 검사 31개 통과(1.330초), 변경한 셸 스크립트 문법 검사 통과.
- 이번 점검은 스트리밍 복구 확인이며 C/D의 주행 완주 결과를 추가한 것은 아니다.

시나리오 전환 명령을 추가했다. 실행 중인 주행 시험은 그 터미널에서 먼저 `Ctrl+C`로 종료한다.

```bash
bash run_isaac.sh stop-sim
bash run_isaac.sh sim --scenario C
# D를 실행하려면 위 마지막 명령의 C를 D로 바꾼다.
```

`stop-sim`은 Compose 프로젝트 `dmpc-two-antbots`의 `isaac-lab` 컨테이너만
SIGINT로 종료하고 완료를 기다린다. 같은 프로젝트의 시뮬레이터가 실행 중이면 새 `sim`은
시작을 차단한다. `Found orphan containers (...router...)` 경고는 이번 영상 실패와 구분하며,
공유 Zenoh 라우터와 별도 프로젝트 컨테이너는 유지했다.

점검 후 새 포트의 B 시뮬레이터 `dmpc-stream-check`는 사용자가 화면을 확인한 상태로 실행을 유지했다.

<a id="scenario-c-delayed-motion-report"></a>

## 시나리오 C 주행 지연 현상 보고 — 2026-10-29 KST

### 관찰 내용

기존 mpc_planner 사용 시에는 로봇이 서로 접근한 뒤 거리가 가까워지면 정지했다는
사용자 관찰이 있다. 현재 DMPC에서는 MPC 경로가 표시돼도 초기에는 움직이지 않았으며,
계속 실행하자 이후 이동한 경우도 관찰됐다. 따라서 현재 현상을 일률적으로
‘주행 불가’ 또는 ‘접근 후 정상 정지’로 규정하지 않고, **경로는 생성되지만 실행 승인에
따라 출발이 지연되거나 주행·정지가 반복되는 현상**으로 기술한다.
기존 시스템과 현재 시스템을 동일 조건으로 반복 비교한 정량 실험은 수행하지 않았다.

### 로그 근거

| 실행 시각 | MPC 유효 계획 수 A/B | 통로 밖 경로 거부 상태 관측 수 A/B | 최대 실측 속력 A/B(m/s) | 관측 결과 |
|---|---|---|---|---|
| [09:26:55](#run-20261029_092655_729520__C) | 601 / 541 | 2603 / 2551 | 0.773 / 0.338 | 일부 주행했으나 제한시간 내 두 로봇 미도착 |
| [09:30:05](#run-20261029_093005_184702__C) | 587 / 587 | 2969 / 2957 | 0.001 / 0.001 | 명령 속력 0, 실질적인 정지 지속, 두 로봇 미도착 |

두 실행은 각각 시뮬레이션 63.16초, 62.36초를 관측했다. 상태 집계는 서로 다른
계획·제어 주기의 메시지 수이며, 거부 상태 횟수를 서로 다른 경로의 개수나 실패율로
해석하지 않는다. 두 번째 실행의 미세 위치 변화는 주행 명령에 의한 이동으로 판단하지 않는다.
두 실행 모두 종료 정리 과정의 일부 Python 자식 프로세스에 종료 코드 -15가 기록됐다.
이는 주행 중 경계 검사 거부와 별도의 종료 처리 문제다.

### 원인 분석과 solver 차이의 관계

확인된 직접 정지 사유는 실행기의 `mpc_path_outside_workspace`다.
계획기는 유효 경로를 발행했지만 실행기는 벽 안전 범위 검사에서 이를 거부했다.
허용되는 경로가 발행되면 이동이 가능하므로, 이후 일부 이동했다는 관찰과 모순되지 않는다.

수정 전 코드상 MPC 출력 검증은 벽 경계에서 약 1 mm의 수치 오차를 허용하는 반면,
실행기는 약 1e-7 m 기준과 곡선 구간의 추가 여유 검사를 적용한다.
이 판정 기준 차이는 아래 09:36 재현·대조 실험에서 실행 거부 원인으로 확인했다.

기존 acados와 현재 IPOPT는 다른 solver이며, 생성 경로·수치 오차에 차이를 줄 수 있다.
solver 교체만을 원인으로 단정할 수 없으며, 동일 IPOPT에서도 계획 단계 벽 여유만
변경했을 때 실행기 검사가 통과했다. 확인된 직접 원인은 **실행기가 요구하는 추가
구간 여유를 계획 단계가 확보하지 않아, 생성된 경로의 실행 승인에 실패하는 것**이다.

### 원인 확정 재현·대조 실험 — 09:36 KST

현행 C 설정(통로 [-4,4]×[-0.7,0.7] m, 최대 속도 1 m/s, 30×0.2초 horizon,
계획기 벽 inset 0.55 m)을 사용했다. A/B의 기본 초기 위치·방향·정지 속도에서,
상대 로봇은 초기 위치에 정지한 관측 객체로 두고 IPOPT 경로를 생성했다.
생성 결과를 그대로 실제 C++ `Bounds::interval_safe` 검사에 입력했다.
이는 물리 시뮬레이터를 구동한 실험이나 이전 실행 경로의 원본 재생은 아니다.

- A/B 초기 상태에서 각 3회 생성한 경로 모두 계획기 검증은 통과했고 실행기 검사는 거부했다.
- 기본 A 경로의 최초 거부 지점은 0부터 센 29번 구간의 18번 내부 샘플,
  즉 약 5.99초 뒤 예측 지점이다. y=-0.14999579455 m에서 벽 안전 경계까지 남은
  여유는 약 **4.205 μm**, 실행기가 요구한 구간 보수 여유는 약 **6.251 μm**였다.
- 기본 경로의 모든 knot와 10 ms 구간 샘플은 벽 안전 범위 안에 있었다.
  최소 여유도 약 1.76 μm로 양수였다. 경로가 실제 경계를 넘어서 거부된 경우가 아니라,
  추가 구간 여유 요구를 만족하지 못해 거부된 사례다.
- 실행기는 현재 추종 구간만이 아니라 전달된 전체 6초 계획의 벽 적합성을 검사한다.
  따라서 먼 미래 구간의 작은 여유 부족도 현재 전체 계획 거부와 정지로 이어진다.

solver·속도·horizon·실행기 검사는 유지하고, 계획기 벽 inset만 0.55→0.551 m로
변경한 대조 실험을 수행했다. 동일 입력의 A/B 경로 모두 실행기 검사를 통과했다.

| 조건 | A 실행기 검사 | B 실행기 검사 | 실행기 안전 경계 기준 최소 knot 여유 |
|---|---|---|---|
| 기존 계획 inset 0.550 m | 거부 | 거부 | 약 0.000001761 m |
| 계획 inset 0.551 m, 추가 1 mm | 통과 | 통과 | 약 0.001001772 m |

이 결과로 **계획 단계와 실행 단계의 경계 여유 요구 불일치**를 재현·확인했다.
계획 재계산 후 여유가 충분해지면 실행이 가능하다는 점도 지연 후 이동 관찰과 일치한다.
다만 과거 모든 거부 메시지가 동일한 구간·수치에서 발생했음을 확인한 것은 아니다.
09:36의 1 mm 추가는 대조 실험의 파라미터 변경이었고, 이후 아래 운영 코드 수정에 반영했다.
실행 중 로봇·뷰어는 중지하거나 변경하지 않았고 임시 실험 파일은 자동 삭제했다.

### 운영 코드 수정 및 실제 C 재시험 — 09:43~09:51 KST

`dmpc_runtime/planner.py`, `dmpc_runtime/ros_node.py`,
`isaac/controller_bringup/config/dmpc.yaml`에 다음을 적용했다.

- `wall_planning_margin=0.001` m를 추가해 최적화 벽 제약과 참조 경로에 반영했다.
  물리 벽과 실행기의 기존 안전 범위는 유지한다.
- 발행 전 검증에서 실행기의 `Bounds::interval_safe`와 같은 10 ms 구간 검사,
  추가 곡선 여유와 1e-7 m 기준을 적용했다. 실행기가 거부할 경로는 유효 경로로 발행하지 않는다.
- 좁은 통로에서 새로 최적화를 시작할 때, 상대를 관통하는 참조 기반 초기값 대신
  현재 상태에서 속도·각속도 제한을 지키며 제동하는 초기 궤적을 사용한다.
  목적함수, 상대 예측, IPOPT, 80 ms 계산 한도 및 최대 속도 1 m/s는 유지했다.

초기값 보완의 근거: 첫 수정본의 실제 정지 위치를 수치 실험에 넣으면 기존 초기값은
80 ms에서 시간 초과했고, 한도를 0.2/0.5초로 늘려도 `Infeasible_Problem_Detected`가
발생했다. 동일 위치에서 정지 궤적 자체는 검증을 통과했으며, 이를 초기값으로 주면
기존 80 ms 한도에서도 두 로봇 모두 최적화에 성공했다. 이는 해당 정지 위치에서의
초기값 민감성을 확인한 실험이며, 이후 모든 위치의 수렴을 보장하지 않는다.

| 실제 C 실행 | 적용 내용 | 통로 밖 경로 거부 상태 A/B | 최대 실측 속력 A/B(m/s) | 최소 중심 간격(m) | 결과 |
|---|---|---|---|---|---|
| [09:44:29](#run-20261029_094429_424693__C) | 벽 여유·발행 전 구간 검사 | 0 / 0 | 0.433 / 0.631 | 3.069 | 두 로봇 출발, 이후 solver 실패로 정지, 제한시간 종료 |
| [09:49:50](#run-20261029_094950_281855__C) | 위 수정 + 좁은 통로 초기값 | 0 / 0 | 0.748 / 0.434 | 1.674 | 두 로봇 출발·접근, 이후 solver 실패로 정지, 제한시간 종료 |

두 실행 모두 벽·로봇 간격의 관측 기준 위반과 종료 정리·자식 프로세스 오류는 없었다.
최종 실행의 관측 시간은 시뮬레이션 31.74초, 벽 안전 범위 최소 여유는 0.027 m였다.
최종 제어 상태는 양쪽 모두 `mpc_solver_failed`이며, 계산시간 초과 경고도 계속 관찰됐다.
따라서 **기존의 벽 경로 거부로 출발하지 못하던 문제는 수정·재시험했지만,
C 전체 주행 및 접근 후 정지의 최적화 실패까지 해결한 것은 아니다.**

실제 IPOPT 결과를 vendored C++ 검사에 넣는 경계 회귀 검사와 기록된 정지 위치의
수렴 회귀 검사를 포함해 자동 검사 33개 통과(2.687초). C++ 실행기의 안전 검사를
완화하지 않았다. 이번 수정은 Python/YAML 변경으로 native 실행기 재빌드는 필요하지 않다.
실제 주행 결과는 이 MD에만 추가했고 시험용 제어기·시뮬레이터는 종료했다.

### 시나리오의 구조적 제약과 개선 방향

C의 통로 폭은 1.4 m이며, 원형 로봇 반경 0.5 m와 벽 안전 여유 0.05 m를 적용하면
로봇 중심의 횡방향 허용 범위는 [-0.15, 0.15] m다. 두 로봇이 나란히 설 수 있는 최대
횡방향 중심 간격 0.3 m는 요구 간격 1.15 m보다 작으므로, 현재 안전 모델에서는
서로의 순서를 바꾸며 통과할 수 없다. 상대 근접에 따른 정지와 실행기 경계 거부에 따른
정지는 서로 다른 현상이다.

계획 단계의 추가 내부 여유와 발행 전 검사 정합은 위와 같이 적용했다.
이후에는 접근 구간의 최적화 실패를 별도로 분석하고, 출발 지연·정상 추종 시간·
최초 정지 사유·위치·로봇 간격을 반복 측정해 기존 시스템과 비교할 필요가 있다.
C의 양방향 목표 도착은 현재 통로·안전 모델의 구조적 제약상 성공 기준으로 삼을 수 없다.

## 실제 주행 요약

| 시나리오 | 검증 결과 | 주요 수치·제한 |
|---|---|---|
| A | 목표 제동·저속 접근 수정 후 반복 도착 | 최신 18:25 회귀 검증 오차 0.082/0.085 m, 최소 간격 2.241 m, 종료 정상 |
| B | 저속 접근 정체 재현·수정 후 반복 도착 | 전체 주행 2회 도착, 정체 위치·방향 재현도 도착. 아래 B 분석 참고 |
| C | 벽 경로 거부 수정·출발 확인, 전체 시험 미완료 | 최신 실제 시험에서 경계 거부 0회, 최소 간격 1.674 m까지 접근. 이후 solver 실패가 남아 있어 추가 개선 필요 |
| D | 두 로봇 도착 | 오차 0.084/0.090 m, 최소 간격 2.269 m, 원통 안전 범위 여유 0.071 m |

## 준비·회귀 검증

- A/B/C/D 설정·Compose 검사 통과.
- 네 장면 모두 실제 Isaac에서 headless 0.1초 초기화·Timeline playing·State 출력 확인.
  이는 완주 검증과 구분한다.
- 최초 자동 검사 22개 통과, 목표 정지 및 native RPP 종료 수정 후 26개 통과.
- MD 단일 파일 저장 방식 변경 후 회귀 검사 27개 모두 통과(1.117초).

<a id="b-low-speed-approach-fix"></a>

## B: 목표 근처 재계획·저속 정체 분석 및 수정

2026-10-28 사용자가 관찰한 현상은 도착 판정이 거짓으로 고정된 문제가 아니었다.
[17:45 실행](#run-20261028_174552_294596__B)에서 A는 목표 오차 0.081 m로 도착했지만,
B는 0.586 m 밖에 남았다. [17:53 실행](#run-20261028_175334_690568__B)도 B 오차가 0.382 m였다.
목표 방향 오차도 컸으나 현재 도착 조건은 위치·정지·입력 신선도이며 방향 자체는 판정 조건이 아니다.
따라서 0.2 m 밖에 있는 B의 재계획은 계속되는 것이 맞고, 실제 문제는 재계획한 저속 동작을
실행해 목표 안으로 진입하지 못한 것이었다.

[수정 전 직접 재현](#run-20261028_180143_823405__B)에서도 벽시계 90초 후
B가 목표 0.239 m 앞에서 정체했다. 마지막 계획 속도는 약 0.007 m/s,
명령 각속도는 약 -0.075 rad/s였지만 실측 각속도는 약 -0.003 rad/s였다.
작은 전진·회전 혼합 명령에서 Swerve 조향이 반복적으로 바뀌며 진행이 거의 없었다.
또 MPC의 첫 각속도 변화 제한을 매번 실측 각속도에서 시작해 느린 구동 응답이
다음 명령 증가도 막았다. 반복 시험에서는 저속 회전 중 작은 음수 전진 속도가
기존 -0.001 m/s 모델 입력 기준에 걸려 `solver_failed`를 발생시키는 것도 확인했다.

수정 내용:

- 목표 1 m 이내·실측 전진 속도 절댓값 0.05 m/s 미만에서는 MPC 첫 각속도 변화 제한의
  기준을 최근 실제 발행한 `cmd_vel.angular.z`로 잡는다. 명령이 0.35초보다 오래되면
  실측값으로 복귀한다. 상태의 위치·방향 전파에는 계속 실측 속도와 각속도를 쓴다.
- MPC 계획 전진 속도와 실측 전진 속도의 절댓값이 모두 0.02 m/s 이하이고 계획 각속도의 절댓값이
  0.02 rad/s보다 크면 전진 명령을 0으로 두고 계획 각속도로 회전한다.
  가속도·각속도·상대 로봇·벽·장애물 검사에는 기존 제어 필터를 그대로 적용한다.
  출발 가속도 제한을 적용한 뒤의 속도로 판단하면 정상 출발도 회전으로 바뀌므로,
  반드시 제한 전의 MPC 계획 속도로 판단하도록 했다.
- -0.01 m/s 이상인 작은 음수 odom 전진 속도는 전진 전용 MPC의 초기 속도에서만
  0으로 취급한다. 그보다 큰 후진은 계속 거부한다. 원본 odom, 실제 명령 안전 검사,
  도착 판정에 사용하는 실측 속력은 바꾸지 않는다.
- 최적화 실패 원인을 5초 간격 경고로 표시해 수치 시간 초과와 모델 입력 오류를 구분한다.
  Python 노드의 SIGINT 처리도 명시해 시험 종료 시 정상 정리를 확인했다.

최대 속도 1.0 m/s, 도착 거리 0.2 m, 정지 속력 0.05 m/s는 유지했다.

| 검증 | 최종 목표 오차 A/B(m) | 최소 간격(m) | 시뮬레이션 관측 시간(s) | 결과 |
|---|---|---|---|---|
| [B 전체 주행 18:21](#run-20261028_182113_234011__B) | 0.087 / 0.092 | 1.804 | 23.20 | 두 로봇 도착·정상 종료 |
| [B 전체 주행 반복 18:23](#run-20261028_182302_278884__B) | 0.075 / 0.160 | 1.745 | 35.88 | B가 약 0.36 m에서 정체 후 회복해 도착 |
| [A 회귀 18:25](#run-20261028_182526_579211__A) | 0.082 / 0.085 | 2.241 | 19.20 | 두 로봇 도착·정상 종료 |
| [B 저속 접근 위치·방향 재현 18:27](#run-20261028_182707_425000__B) | 0.085 / 0.102 | 4.157 | 6.22 | 두 로봇 도착, solver 실패 없음 |

마지막 재현은 A를 `(2.75,0,0)`, B를 `(-0.2204444,2.9069488,1.301476)`에서
정지 상태로 시작했다. B의 위치·방향은 수정 전 정체 기록을 사용했고 바퀴·조향 상태는
새 장면 기본값이므로 전체 물리 상태를 그대로 재생한 실험은 아니다.
전체 주행 2회는 음수 미세 속도 허용 보완 전이며, 마지막 A와 B 접근 재현에는 그 보완도 적용했다.

수정 과정의 [18:15 실패](#run-20261028_181506_471976__B)는 회전 전환을 너무 넓게
적용한 중간 구현이다. 이후 위의 계획 속도 기준과 좁은 전환 범위로 고쳤다.
[18:09 기록](#run-20261028_180941_205035__B)은 초기 배치 불일치로 주행을 시작하지 못한
준비 실패이며 완주 시험에 포함하지 않는다. 실패 기록도 삭제하지 않았다.

native RPP 재빌드 성공. 저속 회전·음수 미세 속도·종료 후 집계 보존을 포함한 회귀 검사
31개 통과. 기존 결과의 상태 횟수·유효 계획 수에는 종료 정리 중 콜백이 더해질 수 있어,
향후 기록은 주행 종료 순간에 복사해 고정하도록 보완했다. 위치·거리·도착 판정은 이미
종료 정리 전에 측정한 값이다.

전체 주행 중 기존 80 ms IPOPT 계산 한도 초과는 간헐적으로 관찰됐다. 이때 정지·재시도하는
동작은 유지하며, 계산 한도 초과까지 제거했다고 판정하지 않는다.

<a id="goal-stop-fix"></a>

## 목표 정지 문제 재현·수정

수정 전 사용자의 16:40 A 기록에서는 목표를 지나친 뒤 0.595/0.336 m 오차로 정지했다.
직접 재현한 17:03 A에서도 B가 0.358 m를 지나쳐 정지했다.
MPC는 약 -0.5 m/s²로 계획했지만 실측 속력은 약 -0.29 m/s²로 줄었다.
목표 20cm 안을 통과할 때 아직 움직였고, 멈췄을 때는 범위 밖이라 도착 판정이 거짓이었다.

목표까지의 거리에 다음 제동 거리 제한을 추가했다.

```text
v × 0.3초 + v² / (2 × 0.25 m/s²) ≤ max(목표 거리 − 0.08m, 0)
```

최대 속도·모델 가속도 상한·도착 오차는 유지했다. 이미 제동 범위를 넘은 초기
속도에는 최대 감속으로 회복하는 과도 구간을 허용한다. 발행 전 독립 검증도 추가했다.

실험 준비 중 이전 장면 시간이 새 장면으로 초기화되면 기록기의 준비 상태도
초기화하도록 수정했다. 주행 시작 후 시간 초기화는 계속 중단 사유다.

native RPP 종료 오류는 기반 LifecycleNode가 publisher의 약한 참조를 정리하기 전에
플러그인 라이브러리가 해제되는 수명 문제였다. class loader를 노드보다 오래 유지해
수정했다. 단독 SIGINT 종료는 수정 전 139에서 수정 후 0으로 바뀌었고, 최종 A의
주행·정리·자식 프로세스 종료도 정상으로 확인했다. 초기 A/B/D 기록에 남은 종료 오류는
이 수정 이전의 결과다.

C는 중심 허용 y 범위가 [-0.15,0.15] m로 상호 간격 1.15 m를 유지하며 순서를
교환할 수 없다. 추가로 MPC 경계 경로를 RPP의 연속 구간 검사에서 거부한 문제가 남아 있다.
목표 제동 수정으로 C가 통과했다고 판정하지 않는다.

## 이전 수치 모의실험 요약

원본 수치 실험 JSON도 삭제하고 아래 요약만 통합했다. 실제 Isaac 주행이 아닌 수학 모델 결과이며, 당시 속도 설정은 원본에 기록되지 않아 현재 1 m/s 물리 주행의 증거로 사용하지 않는다. 모두 CV+Gaussian, 기준 간격 1.15 m다.

| 이전 실험 식별자 | 시나리오 | A/B 도착 | 목표 오차 A/B(m) | 최소 간격(m) | 모의 시간(s) | solver 실패 | 계산시간 중앙값(s) |
|---|---|---|---|---|---|---|---|
| numerical_A | A | 도착/도착 | 0.0137/0.0137 | 1.817 | 14.40 | 0 | 0.0087 |
| numerical_B | B | 도착/도착 | 0.0055/0.0153 | 1.826 | 15.20 | 0 | 0.0082 |
| numerical_B_budget | B | 도착/도착 | 0.0055/0.0153 | 1.826 | 15.20 | 0 | 0.0087 |
| numerical_D | D | 도착/도착 | 0.0129/0.0129 | 2.200 | 14.60 | 0 | 0.0116 |

## 실행 기록

<!-- ISAAC_TEST_RUNS_BEGIN -->

| 시작 시각(KST) | 시나리오 | 관측 판정 | 종료 사유 | A/B 도착 | 최소 간격(m) | 최대 실측 속도(m/s) | 기록 |
|---|---|---|---|---|---|---|---|
| 2026-10-28 16:38:37 | A | 미완료 | 사용자 중단 | 미도착/미도착 | 2.142 | 0.798 | [상세](#run-20261028_163837_685351__A) |
| 2026-10-28 16:40:28 | A | 미완료 | 사용자 중단 | 미도착/미도착 | 2.254 | 1.003 | [상세](#run-20261028_164028_075599__A) |
| 2026-10-28 17:03:31 | A | 미완료 | 제한시간 종료(미완료) | 도착/미도착 | 2.255 | 1.003 | [상세](#run-20261028_170331_530673__A) |
| 2026-10-28 17:09:11 | A | 관측 기준 위반 | 실행 오류: Isaac 입력 없음 또는 초기 위치·방향 불일치. 시뮬레이터를 같은 시나리오로 재시작하세요. | 미도착/미도착 | 6.367 | 0.000 | [상세](#run-20261028_170911_959620__A) |
| 2026-10-28 17:12:04 | A | 위치 도착(접촉·방향 별도) | 두 로봇 목표 도착 | 도착/도착 | 2.237 | 1.003 | [상세](#run-20261028_171204_138383__A) |
| 2026-10-28 17:15:01 | B | 위치 도착(접촉·방향 별도) | 두 로봇 목표 도착 | 도착/도착 | 2.371 | 1.000 | [상세](#run-20261028_171501_723888__B) |
| 2026-10-28 17:16:42 | D | 위치 도착(접촉·방향 별도) | 두 로봇 목표 도착 | 도착/도착 | 2.269 | 0.999 | [상세](#run-20261028_171642_551592__D) |
| 2026-10-28 17:20:06 | C | 미완료 | 제한시간 종료(미완료) | 미도착/미도착 | 5.958 | 0.001 | [상세](#run-20261028_172006_381525__C) |
| 2026-10-28 17:27:14 | A | 위치 도착(접촉·방향 별도) | 두 로봇 목표 도착 | 도착/도착 | 2.228 | 1.000 | [상세](#run-20261028_172714_443744__A) |
| 2026-10-28 17:34:02 | A | 위치 도착(접촉·방향 별도) | 두 로봇 목표 도착 | 도착/도착 | 2.218 | 1.000 | [상세](#run-20261028_173402_892455__A) |
| 2026-10-28 17:45:52 | B | 미완료 | 사용자 중단 | 도착/미도착 | 1.743 | 0.856 | [상세](#run-20261028_174552_294596__B) |
| 2026-10-28 17:53:34 | B | 미완료 | 사용자 중단 | 도착/미도착 | 1.619 | 0.890 | [상세](#run-20261028_175334_690568__B) |
| 2026-10-28 18:01:43 | B | 미완료 | 제한시간 종료(미완료) | 도착/미도착 | 1.679 | 0.862 | [상세](#run-20261028_180143_823405__B) |
| 2026-10-28 18:09:41 | B | 미완료 | 실행 오류: Isaac 입력 없음 또는 초기 위치·방향 불일치. 시뮬레이터를 같은 시나리오로 재시작하세요. | 미도착/미도착 | 5.967 | 0.001 | [상세](#run-20261028_180941_205035__B) |
| 2026-10-28 18:15:06 | B | 미완료 | 제한시간 종료(미완료) | 미도착/미도착 | 1.547 | 0.825 | [상세](#run-20261028_181506_471976__B) |
| 2026-10-28 18:21:13 | B | 위치 도착(접촉·방향 별도) | 두 로봇 목표 도착 | 도착/도착 | 1.804 | 0.834 | [상세](#run-20261028_182113_234011__B) |
| 2026-10-28 18:23:02 | B | 위치 도착(접촉·방향 별도) | 두 로봇 목표 도착 | 도착/도착 | 1.745 | 0.820 | [상세](#run-20261028_182302_278884__B) |
| 2026-10-28 18:25:26 | A | 위치 도착(접촉·방향 별도) | 두 로봇 목표 도착 | 도착/도착 | 2.241 | 1.004 | [상세](#run-20261028_182526_579211__A) |
| 2026-10-28 18:27:07 | B | 위치 도착(접촉·방향 별도) | 두 로봇 목표 도착 (저속 접근 위치·방향 재현, 바퀴 상태 초기화) | 도착/도착 | 4.157 | 0.138 | [상세](#run-20261028_182707_425000__B) |
| 2026-10-29 09:23:37 | B | 위치 도착(접촉·방향 별도) | 두 로봇 목표 도착 | 도착/도착 | 1.666 | 0.855 | [상세](#run-20261029_092337_090044__B) |
| 2026-10-29 09:26:55 | C | 미완료 | 제한시간 종료(미완료) | 미도착/미도착 | 2.421 | 0.773 | [상세](#run-20261029_092655_729520__C) |
| 2026-10-29 09:30:05 | C | 미완료 | 제한시간 종료(미완료) | 미도착/미도착 | 5.872 | 0.001 | [상세](#run-20261029_093005_184702__C) |
| 2026-10-29 09:40:09 | C | 미완료 | 제한시간 종료(미완료) | 미도착/미도착 | 5.688 | 0.215 | [상세](#run-20261029_094009_619716__C) |
| 2026-10-29 09:44:29 | C | 미완료 | 제한시간 종료(미완료) | 미도착/미도착 | 3.069 | 0.631 | [상세](#run-20261029_094429_424693__C) |
| 2026-10-29 09:49:50 | C | 미완료 | 제한시간 종료(미완료) | 미도착/미도착 | 1.674 | 0.748 | [상세](#run-20261029_094950_281855__C) |
| 2026-10-29 10:15:57 | C | 미완료 | 제한시간 종료(미완료) | 미도착/미도착 | 1.675 | 0.806 | [상세](#run-20261029_101557_525261__C) |
| 2026-10-29 10:25:29 | C | 미완료 | 사용자 중단 | 미도착/미도착 | 1.847 | 0.676 | [상세](#run-20261029_102529_266007__C) |
| 2026-10-29 10:27:34 | D | 위치 도착(접촉·방향 별도) | 두 로봇 목표 도착 | 도착/도착 | 2.292 | 1.000 | [상세](#run-20261029_102734_609728__D) |

<!-- ISAAC_TEST_RUNS_END -->

## 이전 관찰 메모


### A: 목표 통과 후 허용 범위 밖에서 정지 (2026-10-28 16:40:28 KST 실행)

- 근거: [실행 요약](#run-20261028_164028_075599__A),
  [실행 요약](#run-20261028_164028_075599__A),
  [실행 요약](#run-20261028_164028_075599__A).
- A 최종 위치 약 `(3.595, 0.031)`, 목표 `(3,0)`, 오차 `0.595 m`.
- B 최종 위치 약 `(-3.319,-0.106)`, 목표 `(-3,0)`, 오차 `0.336 m`.
- A는 시뮬레이션 약 19초에 목표 거리 `0.177 m`이지만 속력 `0.491 m/s`였고,
  약 21초에 정지했을 때는 목표 오차 `0.595 m`였다.
- B는 약 24초에 목표 거리 `0.163 m`이지만 속력 `0.322 m/s`였고,
  약 26초에 정지했을 때는 목표 오차 `0.336 m`였다.
- 따라서 `목표 거리 ≤ 0.2 m`와 `속력 ≤ 0.05 m/s`를 동시에 충족하지 않았다.
  화면에서 목표 근처에 보이는 것과 기록기의 도착 판정이 다른 이유다.
- 이후 두 로봇의 명령은 `v=0,w=0`, 제어 상태는 `mpc_stop`으로 남았다.
  현재 기록으로 확인된 현상은 목표 통과 뒤 정지이며, 제동 지연·계획 속도·추종 오차 중
  어느 요소를 얼마나 수정해야 하는지는 추가 제어 검증이 필요하다.
- 도착 허용 오차를 넓히거나 기존 결과를 성공으로 변경하지 않았다. 진행 로그에
  `goal_error_m`, `speed_mps`, `odom_age_s`를 추가해 남은 도착 조건을 확인할 수 있게 했다.
- `Ctrl+C` 종료 과정에서 ROS 중복 shutdown 및 RPP 프로세스 `-11` 오류도 확인됐다.
  위의 목표 통과·정지는 중단 이전에 발생했으며, 종료 오류는 별도로 수정·검증해야 한다.

후속 상태: 목표 제동 제한과 종료 절차·플러그인 수명을 수정했다.
[실행 요약](#run-20261028_172714_443744__A)은 두 로봇 도착,
원래 도착 기준 유지, 종료 오류 없음으로 검증됐다. 원인과 A/B/C/D별 제한은
[목표 정지 수정](#goal-stop-fix)에 정리했다. 이전 실행의 정량 결과는 아래 상세 기록으로 통합했다.


## 실행별 상세 기록

<a id="run-20261028_163837_685351__A"></a>

### 시나리오 A — 2026-10-28 16:38:37 KST

- 관측 판정: 미완료
- 종료 사유: 사용자 중단
- 설정 SHA-256: `799f60b4dab940fe19ce45f8ebfb82decd1dba20ac2f875946a04731cc0fdbb9`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 26.960 s
- 시각이 일치하는 odom 쌍: 1349
- 관측된 최소 중심 간격: 2.142 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 미도착 | 0.202 | 0.500 | 0.798 | 0.802 | 352 |
| robot_b | 미도착 | 0.308 | 0.269 | 0.493 | 0.508 | 456 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [peer_state_relay-7]: process has died [pid 53, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_shw42lwa']. / [ERROR] [peer_state_relay-10]: process has died [pid 56, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_e5a5ud3e']. / [ERROR] [dmpc_planner-8]: process has died [pid 54, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_bf0d3foj --params-file /tmp/launch_params_a90hgtob']. / [ERROR] [dmpc_planner-11]: process has died [pid 61, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_0bvouhu0 --params-file /tmp/launch_params_3ks4sens']. / [ERROR] [mpc_rpp_controller-9]: process has died [pid 55, exit code -11, cmd '/workspace/isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller --ros-args -r __ns:=/robot_a --params-file /tmp/dmpc_two/rpp_parameters/robot_a_kumcnob4.yaml -r peers/robot_b/state:=peer_state']. / [ERROR] [mpc_rpp_controller-12]: process has died [pid 62, exit code -11, cmd '/workspace/isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller --ros-args -r __ns:=/robot_b --params-file /tmp/dmpc_two/rpp_parameters/robot_b_736wdnhe.yaml -r peers/robot_a/state:=peer_state'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "tracking": 352,
        "waiting_for_fresh_states": 1,
        "goal_reached": 1124
      },
      "controller": {
        "waiting_for_fresh_gt": 6,
        "waiting_for_fresh_mpc_plan": 6,
        "tracking": 1360,
        "waiting_for_clock": 16,
        "mpc_waiting_for_fresh_states": 4,
        "mpc_goal_reached": 1743,
        "disabled": 1,
        "mpc_stop": 76
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 7,
        "tracking": 456,
        "solver_failed": 4,
        "discarded_stale_plan": 1
      },
      "controller": {
        "waiting_for_fresh_gt": 7,
        "waiting_for_fresh_mpc_plan": 3,
        "mpc_waiting_for_fresh_states": 14,
        "tracking": 1861,
        "mpc_solver_failed": 21,
        "waiting_for_clock": 23,
        "disabled": 1,
        "expired_mpc_plan": 2
      }
    }
  }
}
```

</details>

<a id="run-20261028_164028_075599__A"></a>

### 시나리오 A — 2026-10-28 16:40:28 KST

- 관측 판정: 미완료
- 종료 사유: 사용자 중단
- 설정 SHA-256: `799f60b4dab940fe19ce45f8ebfb82decd1dba20ac2f875946a04731cc0fdbb9`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 52.960 s
- 시각이 일치하는 odom 쌍: 2649
- 관측된 최소 중심 간격: 2.254 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 미도착 | 0.595 | 0.258 | 1.003 | 1.000 | 495 |
| robot_b | 미도착 | 0.336 | 0.313 | 0.917 | 0.925 | 494 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [peer_state_relay-7]: process has died [pid 53, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_i7wcbw4e']. / [ERROR] [peer_state_relay-10]: process has died [pid 56, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_y3vvrhyi']. / [ERROR] [dmpc_planner-11]: process has died [pid 57, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_cvpaoumt --params-file /tmp/launch_params_dayn7de9']. / [ERROR] [mpc_rpp_controller-12]: process has died [pid 58, exit code -11, cmd '/workspace/isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller --ros-args -r __ns:=/robot_b --params-file /tmp/dmpc_two/rpp_parameters/robot_b_7okfb4v8.yaml -r peers/robot_a/state:=peer_state']. / [ERROR] [mpc_rpp_controller-9]: process has died [pid 55, exit code -11, cmd '/workspace/isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller --ros-args -r __ns:=/robot_a --params-file /tmp/dmpc_two/rpp_parameters/robot_a_0i3vtuw3.yaml -r peers/robot_b/state:=peer_state'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 495
      },
      "controller": {
        "waiting_for_fresh_gt": 4,
        "waiting_for_fresh_mpc_plan": 1,
        "mpc_waiting_for_fresh_states": 5,
        "tracking": 493,
        "waiting_for_clock": 19,
        "waiting_for_mpc_heading": 3,
        "mpc_stop": 1409
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 494,
        "solver_failed": 2
      },
      "controller": {
        "waiting_for_fresh_gt": 5,
        "waiting_for_fresh_mpc_plan": 1,
        "mpc_waiting_for_fresh_states": 7,
        "tracking": 650,
        "mpc_solver_failed": 9,
        "waiting_for_clock": 12,
        "mpc_stop": 1253
      }
    }
  }
}
```

</details>

<a id="run-20261028_170331_530673__A"></a>

### 시나리오 A — 2026-10-28 17:03:31 KST

- 관측 판정: 미완료
- 종료 사유: 제한시간 종료(미완료)
- 설정 SHA-256: `799f60b4dab940fe19ce45f8ebfb82decd1dba20ac2f875946a04731cc0fdbb9`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 40.920 s
- 시각이 일치하는 odom 쌍: 2047
- 관측된 최소 중심 간격: 2.255 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.159 | 0.072 | 1.003 | 1.000 | 131 |
| robot_b | 미도착 | 0.358 | 0.269 | 0.872 | 0.887 | 382 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [dmpc_planner-8]: process has died [pid 53, exit code -2, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_2xq113s1 --params-file /tmp/launch_params_xh1emnjd']. / [ERROR] [peer_state_relay-7]: process has died [pid 52, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_lu8qj6uk']. / [ERROR] [dmpc_planner-11]: process has died [pid 56, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_3s5rnyb0 --params-file /tmp/launch_params_othtfkqn']. / [ERROR] [peer_state_relay-10]: process has died [pid 55, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_c5t4mrso']. / [ERROR] [mpc_rpp_controller-9]: process has died [pid 54, exit code -11, cmd '/workspace/isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller --ros-args -r __ns:=/robot_a --params-file /tmp/dmpc_two/rpp_parameters/robot_a_wrpu_pst.yaml -r peers/robot_b/state:=peer_state']. / [ERROR] [mpc_rpp_controller-12]: process has died [pid 57, exit code -11, cmd '/workspace/isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller --ros-args -r __ns:=/robot_b --params-file /tmp/dmpc_two/rpp_parameters/robot_b_r2gxakak.yaml -r peers/robot_a/state:=peer_state'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "1dffd3046f2bb5af037ef56bc4a2788b8b4d3025901928641ecb4a58c827a5c2",
    "dmpc_runtime/ros_node.py": "f57f2e2a82afe15a7fac0865115012b36a92f69415584ba44ba01f4efeabe101",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "940c57d2e7bb1460a89780a668b344e851566aba728c5083c0730f5bb545a1db"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 131,
        "goal_reached": 2457
      },
      "controller": {
        "waiting_for_fresh_gt": 5,
        "waiting_for_fresh_mpc_plan": 1,
        "mpc_waiting_for_fresh_states": 5,
        "tracking": 492,
        "waiting_for_clock": 9,
        "mpc_goal_reached": 3438
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 382,
        "solver_failed": 5
      },
      "controller": {
        "waiting_for_fresh_gt": 4,
        "waiting_for_fresh_mpc_plan": 1,
        "mpc_waiting_for_fresh_states": 5,
        "tracking": 678,
        "mpc_solver_failed": 21,
        "waiting_for_clock": 14,
        "mpc_stop": 776
      }
    }
  },
  "final_control_diagnostics": {
    "robot_a": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "mpc_goal_reached"
    },
    "robot_b": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "mpc_stop"
    }
  }
}
```

</details>

<a id="run-20261028_170911_959620__A"></a>

### 시나리오 A — 2026-10-28 17:09:11 KST

- 관측 판정: 관측 기준 위반
- 종료 사유: 실행 오류: Isaac 입력 없음 또는 초기 위치·방향 불일치. 시뮬레이터를 같은 시나리오로 재시작하세요.
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 2.680 s
- 시각이 일치하는 odom 쌍: 135
- 관측된 최소 중심 간격: 6.367 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 미도착 | 0.159 | 0.073 | 0.000 | 0.000 | 0 |
| robot_b | 미도착 | 0.358 | 0.269 | 0.000 | 0.000 | 0 |

관측된 기준 위반: 실험 중 시뮬레이션 시간 초기화

종료 정리 오류: 없음

자식 프로세스 오류: 없음

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "7d87376eea8ee0e8ac65ece1a05a9adfbc15a85f38e47998ce698b8bf48f3ccc",
    "dmpc_runtime/ros_node.py": "0a58b2824008aaef2960c89010c08351d8db8673e4a52a23e10ddbc5ccefa94b",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "940c57d2e7bb1460a89780a668b344e851566aba728c5083c0730f5bb545a1db"
  },
  "statuses": {
    "robot_a": {
      "planner": {},
      "controller": {}
    },
    "robot_b": {
      "planner": {},
      "controller": {}
    }
  }
}
```

</details>

<a id="run-20261028_171204_138383__A"></a>

### 시나리오 A — 2026-10-28 17:12:04 KST

- 관측 판정: 위치 도착(접촉·방향 별도)
- 종료 사유: 두 로봇 목표 도착
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 18.860 s
- 시각이 일치하는 odom 쌍: 944
- 관측된 최소 중심 간격: 2.237 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.086 | 0.291 | 1.003 | 0.999 | 138 |
| robot_b | 도착 | 0.087 | 0.320 | 0.934 | 0.949 | 177 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [robot_state_publisher-4]: process has died [pid 49, exit code -2, cmd '/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher --ros-args -r __node:=control_description -r __ns:=/robot_b --params-file /tmp/launch_params_2q3q_6jv -r /tf:=/robot_b/control_model/tf -r /tf_static:=/robot_b/control_model/tf_static']. / [ERROR] [ros2_control_node-5]: process has died [pid 50, exit code -2, cmd '/opt/ros/jazzy/lib/controller_manager/ros2_control_node --ros-args -r __ns:=/robot_b --params-file /tmp/dmpc_two/controller_parameters/robot_b_idfz2qcq.yaml --params-file /tmp/launch_params_hii4rkkk -r robot_description:=/robot_b/robot_description -r ~/robot_description:=/robot_b/robot_description -r odom:=/robot_b/wheel_odom']. / [ERROR] [dmpc_planner-11]: process has died [pid 56, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_jj3ydmkh --params-file /tmp/launch_params_poi6wwq9']. / [ERROR] [peer_state_relay-10]: process has died [pid 55, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_bj2tydds']. / [ERROR] [dmpc_planner-8]: process has died [pid 53, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_nru6ydjn --params-file /tmp/launch_params_8chcly91']. / [ERROR] [peer_state_relay-7]: process has died [pid 52, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_73r8p1ee']. / [ERROR] [mpc_rpp_controller-12]: process has died [pid 57, exit code -11, cmd '/workspace/isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller --ros-args -r __ns:=/robot_b --params-file /tmp/dmpc_two/rpp_parameters/robot_b_2fvlvqv7.yaml -r peers/robot_a/state:=peer_state']. / [ERROR] [mpc_rpp_controller-9]: process has died [pid 54, exit code -11, cmd '/workspace/isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller --ros-args -r __ns:=/robot_a --params-file /tmp/dmpc_two/rpp_parameters/robot_a_89h25ava.yaml -r peers/robot_b/state:=peer_state'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "7d87376eea8ee0e8ac65ece1a05a9adfbc15a85f38e47998ce698b8bf48f3ccc",
    "dmpc_runtime/ros_node.py": "0a58b2824008aaef2960c89010c08351d8db8673e4a52a23e10ddbc5ccefa94b",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "940c57d2e7bb1460a89780a668b344e851566aba728c5083c0730f5bb545a1db"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 138,
        "goal_reached": 423
      },
      "controller": {
        "waiting_for_fresh_gt": 4,
        "invalid_peer_state": 1,
        "waiting_for_fresh_mpc_plan": 1,
        "mpc_waiting_for_fresh_states": 6,
        "tracking": 516,
        "waiting_for_clock": 2,
        "mpc_goal_reached": 592
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 177,
        "solver_failed": 5
      },
      "controller": {
        "waiting_for_fresh_gt": 5,
        "waiting_for_fresh_mpc_plan": 1,
        "mpc_waiting_for_fresh_states": 7,
        "tracking": 657,
        "mpc_solver_failed": 27,
        "waiting_for_clock": 7
      }
    }
  },
  "final_control_diagnostics": {
    "robot_b": {
      "plan_age": 0.05,
      "ego_age": 0.0,
      "peer_age": 0.04,
      "profile_speed": 0.03980873137242985,
      "profile_yaw_rate": -0.007023361497885925,
      "position_error": 5.399010661390916e-05,
      "measured_speed": 0.05003615958159176,
      "measured_yaw_rate": -0.0005000046782602224,
      "rpp_speed": 0.03980873137242985,
      "rpp_yaw_rate": 0.0,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 5.834185737831783,
      "peer_checks": [
        {
          "robot_id": "robot_a",
          "age": 0.04,
          "separation": 5.834185737831783,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.06567993966134368
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "command_speed": 0.03980873137242985,
      "command_yaw_rate": 0.0,
      "predicted_clearance": 5.640237303850927,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.03980873137242985,
      "candidate_yaw_rate": 0.0,
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "tracking"
    },
    "robot_a": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "mpc_goal_reached"
    }
  }
}
```

</details>

<a id="run-20261028_171501_723888__B"></a>

### 시나리오 B — 2026-10-28 17:15:01 KST

- 관측 판정: 위치 도착(접촉·방향 별도)
- 종료 사유: 두 로봇 목표 도착
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 16.860 s
- 시각이 일치하는 odom 쌍: 844
- 관측된 최소 중심 간격: 2.371 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.086 | 0.008 | 0.966 | 0.969 | 161 |
| robot_b | 도착 | 0.082 | 0.001 | 1.000 | 0.999 | 113 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [dmpc_planner-11]: process has died [pid 56, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params__eqnt6m6 --params-file /tmp/launch_params_7xqg4nxx']. / [ERROR] [peer_state_relay-10]: process has died [pid 55, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_2azp434a']. / [ERROR] [peer_state_relay-7]: process has died [pid 52, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_yfsr7g0i']. / [ERROR] [dmpc_planner-8]: process has died [pid 53, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_44c1w17k --params-file /tmp/launch_params_tlx6vck4']. / [ERROR] [mpc_rpp_controller-12]: process has died [pid 57, exit code -11, cmd '/workspace/isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller --ros-args -r __ns:=/robot_b --params-file /tmp/dmpc_two/rpp_parameters/robot_b_qlkm351_.yaml -r peers/robot_a/state:=peer_state']. / [ERROR] [mpc_rpp_controller-9]: process has died [pid 54, exit code -11, cmd '/workspace/isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller --ros-args -r __ns:=/robot_a --params-file /tmp/dmpc_two/rpp_parameters/robot_a_gqwm7lvy.yaml -r peers/robot_b/state:=peer_state'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "7d87376eea8ee0e8ac65ece1a05a9adfbc15a85f38e47998ce698b8bf48f3ccc",
    "dmpc_runtime/ros_node.py": "34047c008c8cdeb0bf3e763d8fa666fd4c800a0121812202d157cc828e6018df",
    "dmpc_runtime/isaac_test.py": "e4ced6555d1561a1b975ac1f7b1cacd3f04479b5a1b79bca2069d73bce62afec",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "940c57d2e7bb1460a89780a668b344e851566aba728c5083c0730f5bb545a1db"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 161
      },
      "controller": {
        "waiting_for_fresh_gt": 5,
        "invalid_peer_state": 1,
        "mpc_waiting_for_fresh_states": 7,
        "tracking": 600,
        "waiting_for_clock": 2,
        "expired_mpc_plan": 1
      }
    },
    "robot_b": {
      "planner": {
        "tracking": 113,
        "goal_reached": 466
      },
      "controller": {
        "waiting_for_fresh_gt": 3,
        "invalid_peer_state": 1,
        "waiting_for_fresh_mpc_plan": 5,
        "waiting_for_clock": 2,
        "tracking": 414,
        "mpc_goal_reached": 653
      }
    }
  },
  "final_control_diagnostics": {
    "robot_b": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "mpc_goal_reached"
    },
    "robot_a": {
      "plan_age": 0.1,
      "ego_age": 0.01,
      "peer_age": 0.01,
      "profile_speed": 0.03138648931522925,
      "profile_yaw_rate": 0.00017766626197484824,
      "position_error": 0.0004279446069674539,
      "measured_speed": 0.05306274324152522,
      "measured_yaw_rate": -1.618208327945215e-05,
      "rpp_speed": 0.03138648931522925,
      "rpp_yaw_rate": 0.0,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 4.1222469480564445,
      "peer_checks": [
        {
          "robot_id": "robot_b",
          "age": 0.01,
          "separation": 4.1222469480564445,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.03283996983067184
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "command_speed": 0.03138648931522925,
      "command_yaw_rate": 0.0,
      "predicted_clearance": 3.9304260282681787,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.03138648931522925,
      "candidate_yaw_rate": 0.0,
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "tracking"
    }
  }
}
```

</details>

<a id="run-20261028_171642_551592__D"></a>

### 시나리오 D — 2026-10-28 17:16:42 KST

- 관측 판정: 위치 도착(접촉·방향 별도)
- 종료 사유: 두 로봇 목표 도착
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 18.400 s
- 시각이 일치하는 odom 쌍: 921
- 관측된 최소 중심 간격: 2.269 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: 0.071 m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.084 | 0.335 | 0.999 | 0.998 | 138 |
| robot_b | 도착 | 0.090 | 0.383 | 0.982 | 0.995 | 172 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [peer_state_relay-10]: process has died [pid 55, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_n1_nb40y']. / [ERROR] [dmpc_planner-11]: process has died [pid 56, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_usxi3vyy --params-file /tmp/launch_params_i7gbyifb']. / [ERROR] [peer_state_relay-7]: process has died [pid 52, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_xug0803d']. / [ERROR] [dmpc_planner-8]: process has died [pid 53, exit code 1, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_lgzoi2py --params-file /tmp/launch_params_iq1y_p98']. / [ERROR] [mpc_rpp_controller-9]: process has died [pid 54, exit code -11, cmd '/workspace/isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller --ros-args -r __ns:=/robot_a --params-file /tmp/dmpc_two/rpp_parameters/robot_a_ptxx3pqi.yaml -r peers/robot_b/state:=peer_state']. / [ERROR] [mpc_rpp_controller-12]: process has died [pid 57, exit code -11, cmd '/workspace/isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller --ros-args -r __ns:=/robot_b --params-file /tmp/dmpc_two/rpp_parameters/robot_b_iif_niz8.yaml -r peers/robot_a/state:=peer_state'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "7d87376eea8ee0e8ac65ece1a05a9adfbc15a85f38e47998ce698b8bf48f3ccc",
    "dmpc_runtime/ros_node.py": "34047c008c8cdeb0bf3e763d8fa666fd4c800a0121812202d157cc828e6018df",
    "dmpc_runtime/isaac_test.py": "e4ced6555d1561a1b975ac1f7b1cacd3f04479b5a1b79bca2069d73bce62afec",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "940c57d2e7bb1460a89780a668b344e851566aba728c5083c0730f5bb545a1db"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 138,
        "solver_failed": 1,
        "goal_reached": 350
      },
      "controller": {
        "mpc_waiting_for_fresh_states": 5,
        "tracking": 506,
        "mpc_solver_failed": 6,
        "waiting_for_clock": 5,
        "mpc_goal_reached": 489
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 172,
        "solver_failed": 5
      },
      "controller": {
        "waiting_for_fresh_gt": 4,
        "waiting_for_fresh_mpc_plan": 2,
        "mpc_waiting_for_fresh_states": 5,
        "tracking": 627,
        "mpc_solver_failed": 27,
        "waiting_for_clock": 3
      }
    }
  },
  "final_control_diagnostics": {
    "robot_b": {
      "plan_age": 0.04,
      "ego_age": 0.01,
      "peer_age": 0.030000000000000002,
      "profile_speed": 0.046719683493180314,
      "profile_yaw_rate": -0.010292516241231696,
      "position_error": 6.600955764978544e-05,
      "measured_speed": 0.05554808931173185,
      "measured_yaw_rate": -0.001623240204245979,
      "rpp_speed": 0.046719683493180314,
      "rpp_yaw_rate": 0.0,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 5.835198019225864,
      "peer_checks": [
        {
          "robot_id": "robot_a",
          "age": 0.030000000000000002,
          "separation": 5.835198019225864,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.05688049626575273
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "static_obstacle": {
        "x": 0.0,
        "y": 0.0,
        "radius": 0.4,
        "checked_separation": 1.055,
        "clearance": 1.8689952430080699
      },
      "command_speed": 0.046719683493180314,
      "command_yaw_rate": 0.0,
      "predicted_clearance": 5.642374015361894,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.046719683493180314,
      "candidate_yaw_rate": 0.0,
      "predicted_static_clearance": 1.86399524300807,
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": true,
      "status": "tracking"
    },
    "robot_a": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": true,
      "status": "mpc_goal_reached"
    }
  }
}
```

</details>

<a id="run-20261028_172006_381525__C"></a>

### 시나리오 C — 2026-10-28 17:20:06 KST

- 관측 판정: 미완료
- 종료 사유: 제한시간 종료(미완료)
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 24.540 s
- 시각이 일치하는 odom 쌍: 1228
- 관측된 최소 중심 간격: 5.958 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: 0.150 m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 미도착 | 5.979 | 0.000 | 0.001 | 0.000 | 232 |
| robot_b | 미도착 | 5.979 | 0.000 | 0.001 | 0.000 | 227 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [mpc_rpp_controller-12]: process has died [pid 58, exit code -11, cmd '/workspace/isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller --ros-args -r __ns:=/robot_b --params-file /tmp/dmpc_two/rpp_parameters/robot_b_6eg6b1xy.yaml -r peers/robot_a/state:=peer_state']. / [ERROR] [mpc_rpp_controller-9]: process has died [pid 55, exit code -11, cmd '/workspace/isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller --ros-args -r __ns:=/robot_a --params-file /tmp/dmpc_two/rpp_parameters/robot_a_aaxk59ag.yaml -r peers/robot_b/state:=peer_state'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "7d87376eea8ee0e8ac65ece1a05a9adfbc15a85f38e47998ce698b8bf48f3ccc",
    "dmpc_runtime/ros_node.py": "a9ef5f7bfef645ffd2d900e08a69a7693e58de4a5be00982bcb40abf120c040f",
    "dmpc_runtime/isaac_test.py": "7235264baeac131ef10fffb13eeae2036a33a20e76d91d24aa2593d60e030681",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "940c57d2e7bb1460a89780a668b344e851566aba728c5083c0730f5bb545a1db"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 232
      },
      "controller": {
        "waiting_for_fresh_mpc_plan": 2,
        "mpc_waiting_for_fresh_states": 6,
        "mpc_path_outside_workspace": 1107,
        "disabled": 1
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 227,
        "solver_failed": 6
      },
      "controller": {
        "waiting_for_fresh_gt": 1,
        "mpc_waiting_for_fresh_states": 6,
        "mpc_path_outside_workspace": 1088,
        "mpc_solver_failed": 27,
        "disabled": 1
      }
    }
  },
  "final_control_diagnostics": {
    "robot_a": {
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "disabled"
    },
    "robot_b": {
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "disabled"
    }
  }
}
```

</details>

<a id="run-20261028_172714_443744__A"></a>

### 시나리오 A — 2026-10-28 17:27:14 KST

- 관측 판정: 위치 도착(접촉·방향 별도)
- 종료 사유: 두 로봇 목표 도착
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 18.740 s
- 시각이 일치하는 odom 쌍: 938
- 관측된 최소 중심 간격: 2.228 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.081 | 0.308 | 1.000 | 0.999 | 137 |
| robot_b | 도착 | 0.088 | 0.357 | 0.964 | 0.974 | 175 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: 없음

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "7d87376eea8ee0e8ac65ece1a05a9adfbc15a85f38e47998ce698b8bf48f3ccc",
    "dmpc_runtime/ros_node.py": "a9ef5f7bfef645ffd2d900e08a69a7693e58de4a5be00982bcb40abf120c040f",
    "dmpc_runtime/isaac_test.py": "9b59eb9dfc8def41307854e6f58055e65b122331d506ed10d9d267ce3d9f6335",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "841bcd9b2553c23d0a4cace5a8726572921e7dd0095e12b6be9b29dd1f35fb4c"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 137,
        "goal_reached": 411
      },
      "controller": {
        "mpc_waiting_for_fresh_states": 3,
        "tracking": 500,
        "waiting_for_clock": 2,
        "mpc_goal_reached": 524,
        "disabled": 1
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 4,
        "tracking": 175,
        "solver_failed": 3,
        "goal_reached": 5
      },
      "controller": {
        "waiting_for_fresh_gt": 2,
        "mpc_waiting_for_fresh_states": 4,
        "tracking": 647,
        "mpc_solver_failed": 14,
        "waiting_for_clock": 6,
        "mpc_goal_reached": 4,
        "disabled": 1
      }
    }
  },
  "final_control_diagnostics": {
    "robot_b": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "disabled"
    },
    "robot_a": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "disabled"
    }
  }
}
```

</details>

<a id="run-20261028_173402_892455__A"></a>

### 시나리오 A — 2026-10-28 17:34:02 KST

- 관측 판정: 위치 도착(접촉·방향 별도)
- 종료 사유: 두 로봇 목표 도착
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 19.140 s
- 시각이 일치하는 odom 쌍: 958
- 관측된 최소 중심 간격: 2.218 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.083 | 0.418 | 1.000 | 0.998 | 136 |
| robot_b | 도착 | 0.089 | 0.319 | 0.923 | 0.935 | 176 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [peer_state_relay-7]: process has died [pid 52, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_4vlay6ok']. / [ERROR] [peer_state_relay-10]: process has died [pid 55, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_p_b54fos']. / [ERROR] [dmpc_planner-11]: process has died [pid 56, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_t4wmvc9b --params-file /tmp/launch_params_ky2uz967']. / [ERROR] [dmpc_planner-8]: process has died [pid 53, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_80dov9rg --params-file /tmp/launch_params_t34i78pa'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "7d87376eea8ee0e8ac65ece1a05a9adfbc15a85f38e47998ce698b8bf48f3ccc",
    "dmpc_runtime/ros_node.py": "a9ef5f7bfef645ffd2d900e08a69a7693e58de4a5be00982bcb40abf120c040f",
    "dmpc_runtime/isaac_test.py": "9b59eb9dfc8def41307854e6f58055e65b122331d506ed10d9d267ce3d9f6335",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "841bcd9b2553c23d0a4cace5a8726572921e7dd0095e12b6be9b29dd1f35fb4c"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 136,
        "goal_reached": 448
      },
      "controller": {
        "waiting_for_fresh_gt": 1,
        "mpc_waiting_for_fresh_states": 4,
        "tracking": 526,
        "waiting_for_clock": 5,
        "mpc_goal_reached": 619,
        "disabled": 1
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 176,
        "solver_failed": 4,
        "goal_reached": 6
      },
      "controller": {
        "mpc_waiting_for_fresh_states": 2,
        "tracking": 686,
        "mpc_solver_failed": 19,
        "waiting_for_clock": 6,
        "mpc_goal_reached": 4,
        "disabled": 1
      }
    }
  },
  "final_control_diagnostics": {
    "robot_a": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "disabled"
    },
    "robot_b": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "disabled"
    }
  }
}
```

</details>

<a id="run-20261028_174552_294596__B"></a>

### 시나리오 B — 2026-10-28 17:45:52 KST

- 관측 판정: 미완료
- 종료 사유: 사용자 중단
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 29.280 s
- 시각이 일치하는 odom 쌍: 1465
- 관측된 최소 중심 간격: 1.743 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.081 | 0.024 | 0.747 | 0.753 | 148 |
| robot_b | 미도착 | 0.586 | 1.564 | 0.856 | 0.866 | 269 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [peer_state_relay-7]: process has died [pid 62, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_nqf7rg1o']. / [ERROR] [peer_state_relay-10]: process has died [pid 65, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_na3h9isz']. / [ERROR] [dmpc_planner-8]: process has died [pid 63, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_l75z5k3u --params-file /tmp/launch_params_1c2x0jgz']. / [ERROR] [dmpc_planner-11]: process has died [pid 66, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_7kicu44p --params-file /tmp/launch_params_fhxlyh9b'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "7d87376eea8ee0e8ac65ece1a05a9adfbc15a85f38e47998ce698b8bf48f3ccc",
    "dmpc_runtime/ros_node.py": "a9ef5f7bfef645ffd2d900e08a69a7693e58de4a5be00982bcb40abf120c040f",
    "dmpc_runtime/isaac_test.py": "8a89ef8e214a4f9eed1597efd3c6252280cce6b347c676fc378b94546c0579c5",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "841bcd9b2553c23d0a4cace5a8726572921e7dd0095e12b6be9b29dd1f35fb4c"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 148,
        "solver_failed": 63,
        "goal_reached": 581
      },
      "controller": {
        "waiting_for_fresh_gt": 1,
        "mpc_waiting_for_fresh_states": 4,
        "tracking": 583,
        "waiting_for_clock": 7,
        "mpc_solver_failed": 323,
        "mpc_goal_reached": 795,
        "disabled": 1
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 3,
        "tracking": 269,
        "solver_failed": 5
      },
      "controller": {
        "waiting_for_fresh_gt": 1,
        "mpc_waiting_for_fresh_states": 2,
        "waiting_for_clock": 14,
        "tracking": 755,
        "tracking_speed_reduced": 9,
        "proximity_stop": 1,
        "proximity_recovery_wait": 6,
        "mpc_solver_failed": 20,
        "tracking_mpc_turn": 283,
        "disabled": 1
      }
    }
  },
  "final_control_diagnostics": {
    "robot_a": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "mpc_goal_reached"
    },
    "robot_b": {
      "plan_age": 0.060000000000000005,
      "ego_age": 0.0,
      "peer_age": 0.02,
      "profile_speed": 0.017226881375605633,
      "profile_yaw_rate": -0.08288916735490859,
      "position_error": 0.0005938961877874489,
      "measured_speed": -0.0005751691016726956,
      "measured_yaw_rate": 0.0009980098994112389,
      "rpp_speed": 0.017226881375605633,
      "rpp_yaw_rate": -0.08480726721682401,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 3.6536107159541533,
      "peer_checks": [
        {
          "robot_id": "robot_a",
          "age": 0.02,
          "separation": 3.6536107159541533,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.04644273072245939
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "command_speed": 0.015000000000000001,
      "command_yaw_rate": -0.015000000000000001,
      "predicted_clearance": 3.467948797549602,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.015000000000000001,
      "candidate_yaw_rate": -0.015000000000000001,
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "tracking"
    }
  }
}
```

</details>

<details><summary>마지막 실행 메시지</summary>

```text
[robot_state_publisher-4] [INFO] [1793177212.667017787] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793177212.667088743] [robot_a.controller_manager]: Deactivating controller 'antbot_swerve_controller'
[ros2_control_node-5] [INFO] [1793177212.667251739] [robot_b.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793177212.667359525] [robot_a.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-5] [INFO] [1793177212.667750837] [resource_manager]: 'deactivate' hardware 'robot_b_isaac_system'
[ros2_control_node-2] [INFO] [1793177212.667788250] [resource_manager]: 'deactivate' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793177212.668364476] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-2] [INFO] [1793177212.668410097] [resource_manager]: Successful 'deactivate' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793177212.668424165] [resource_manager]: 'shutdown' hardware 'robot_a_isaac_system'
[ros2_control_node-5] [INFO] [1793177212.668407337] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-2] [INFO] [1793177212.668442628] [resource_manager]: Successful 'shutdown' of hardware 'robot_a_isaac_system'
[ros2_control_node-5] [INFO] [1793177212.668447680] [resource_manager]: Successful 'deactivate' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793177212.668461438] [resource_manager]: 'shutdown' hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793177212.668476126] [resource_manager]: Successful 'shutdown' of hardware 'robot_b_isaac_system'
[ros2_control_node-2] [INFO] [1793177212.668527134] [robot_a.controller_manager]: Shutting down the controller manager.
[ros2_control_node-5] [INFO] [1793177212.668560839] [robot_b.controller_manager]: Shutting down the controller manager.
[ERROR] [peer_state_relay-7]: process has died [pid 62, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_nqf7rg1o'].
[mpc_rpp_controller-12] [INFO] [1793177212.671850485] [robot_b.mpc_rpp_controller]: Destroying
[ERROR] [peer_state_relay-10]: process has died [pid 65, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_na3h9isz'].
[ERROR] [dmpc_planner-8]: process has died [pid 63, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_l75z5k3u --params-file /tmp/launch_params_1c2x0jgz'].
[INFO] [mpc_rpp_controller-12]: process has finished cleanly [pid 67]
[ERROR] [dmpc_planner-11]: process has died [pid 66, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_7kicu44p --params-file /tmp/launch_params_fhxlyh9b'].
[mpc_rpp_controller-9] [INFO] [1793177212.680198828] [robot_a.mpc_rpp_controller]: Destroying
[INFO] [mpc_rpp_controller-9]: process has finished cleanly [pid 64]
[ros2_control_node-5] [ERROR] [1793177212.698644459] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [ERROR] [1793177212.698644996] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793177212.699256786] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793177212.699256979] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [INFO] [1793177212.699310748] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793177212.699321912] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-5] [INFO] [1793177212.699409306] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793177212.699412473] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793177212.699925148] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793177212.699931696] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793177212.699988217] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793177212.699990053] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 48]
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 55]
[INFO] [ros2_control_node-5]: process has finished cleanly [pid 56]
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 49]
```

</details>

<a id="run-20261028_175334_690568__B"></a>

### 시나리오 B — 2026-10-28 17:53:34 KST

- 관측 판정: 미완료
- 종료 사유: 사용자 중단
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 27.780 s
- 시각이 일치하는 odom 쌍: 1390
- 관측된 최소 중심 간격: 1.619 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.082 | 0.019 | 0.766 | 0.770 | 176 |
| robot_b | 미도착 | 0.382 | 1.785 | 0.890 | 0.894 | 255 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [dmpc_planner-11]: process has died [pid 64, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_36o3_ive --params-file /tmp/launch_params_4j5pji56']. / [ERROR] [peer_state_relay-7]: process has died [pid 54, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_zai9w_6s']. / [ERROR] [peer_state_relay-10]: process has died [pid 61, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_et9zrqi0']. / [ERROR] [dmpc_planner-8]: process has died [pid 55, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_07fa_lbx --params-file /tmp/launch_params_p6uigj6a'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "7d87376eea8ee0e8ac65ece1a05a9adfbc15a85f38e47998ce698b8bf48f3ccc",
    "dmpc_runtime/ros_node.py": "a9ef5f7bfef645ffd2d900e08a69a7693e58de4a5be00982bcb40abf120c040f",
    "dmpc_runtime/isaac_test.py": "8a89ef8e214a4f9eed1597efd3c6252280cce6b347c676fc378b94546c0579c5",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "841bcd9b2553c23d0a4cace5a8726572921e7dd0095e12b6be9b29dd1f35fb4c"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 176,
        "solver_failed": 32,
        "goal_reached": 509
      },
      "controller": {
        "mpc_waiting_for_fresh_states": 1,
        "tracking": 645,
        "waiting_for_clock": 8,
        "tracking_mpc_turn": 8,
        "mpc_solver_failed": 159,
        "mpc_goal_reached": 705,
        "disabled": 1
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 255,
        "solver_failed": 4
      },
      "controller": {
        "mpc_waiting_for_fresh_states": 4,
        "tracking": 642,
        "waiting_for_clock": 12,
        "tracking_speed_reduced": 7,
        "proximity_stop": 1,
        "proximity_recovery_wait": 6,
        "tracking_mpc_turn": 298,
        "mpc_solver_failed": 26,
        "disabled": 1
      }
    }
  },
  "final_control_diagnostics": {
    "robot_b": {
      "plan_age": 0.11,
      "ego_age": 0.01,
      "peer_age": 0.01,
      "profile_speed": 5.2629422237726164e-06,
      "profile_yaw_rate": -0.10360011493065482,
      "position_error": 0.00034988548193908486,
      "measured_speed": -0.00021246618350302974,
      "measured_yaw_rate": -0.056037213654202125,
      "rpp_speed": 0.0,
      "rpp_yaw_rate": -0.08103721365420213,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 3.8277555027834596,
      "peer_checks": [
        {
          "robot_id": "robot_a",
          "age": 0.01,
          "separation": 3.8277555027834596,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.03283996983067184
        }
      ],
      "yaw_source": "mpc_rotation_profile",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "command_speed": 0.0,
      "command_yaw_rate": -0.10360011493065482,
      "predicted_clearance": 3.6034605875139003,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.0,
      "candidate_yaw_rate": -0.10360011493065482,
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "tracking_mpc_turn"
    },
    "robot_a": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "mpc_goal_reached"
    }
  }
}
```

</details>

<details><summary>마지막 실행 메시지</summary>

```text
[ros2_control_node-2] [INFO] [1793177670.440518060] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793177670.440696037] [robot_a.controller_manager]: Shutdown request received....
[ros2_control_node-2] [INFO] [1793177670.440785048] [robot_a.controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-2] [INFO] [1793177670.440842536] [robot_a.controller_manager]: Deactivating controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793177670.441142567] [robot_a.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793177670.441537159] [resource_manager]: 'deactivate' hardware 'robot_a_isaac_system'
[ros2_control_node-5] [INFO] [1793177670.441818919] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-5] [INFO] [1793177670.441868116] [resource_manager]: Successful 'deactivate' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793177670.441882622] [resource_manager]: 'shutdown' hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793177670.441896633] [resource_manager]: Successful 'shutdown' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793177670.441997552] [robot_b.controller_manager]: Shutting down the controller manager.
[mpc_rpp_controller-12] [INFO] [1793177670.442111501] [robot_b.mpc_rpp_controller]: Destroying
[ros2_control_node-2] [INFO] [1793177670.442381595] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-2] [INFO] [1793177670.442397327] [resource_manager]: Successful 'deactivate' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793177670.442401288] [resource_manager]: 'shutdown' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793177670.442403672] [resource_manager]: Successful 'shutdown' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793177670.442436673] [robot_a.controller_manager]: Shutting down the controller manager.
[mpc_rpp_controller-9] [INFO] [1793177670.443226503] [robot_a.mpc_rpp_controller]: Destroying
[ERROR] [dmpc_planner-11]: process has died [pid 64, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_36o3_ive --params-file /tmp/launch_params_4j5pji56'].
[ERROR] [peer_state_relay-7]: process has died [pid 54, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_zai9w_6s'].
[ERROR] [peer_state_relay-10]: process has died [pid 61, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_et9zrqi0'].
[INFO] [mpc_rpp_controller-12]: process has finished cleanly [pid 66]
[ERROR] [dmpc_planner-8]: process has died [pid 55, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_07fa_lbx --params-file /tmp/launch_params_p6uigj6a'].
[INFO] [mpc_rpp_controller-9]: process has finished cleanly [pid 60]
[ros2_control_node-5] [ERROR] [1793177670.475778587] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [ERROR] [1793177670.475778561] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [INFO] [1793177670.476096307] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793177670.476107212] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-5] [INFO] [1793177670.476213870] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793177670.476217380] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [ERROR] [1793177670.477757642] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793177670.477760573] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [INFO] [1793177670.478163772] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793177670.478168871] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793177670.478234458] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793177670.478236047] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 48]
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 51]
[INFO] [ros2_control_node-5]: process has finished cleanly [pid 52]
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 49]
```

</details>

<a id="run-20261028_180143_823405__B"></a>

### 시나리오 B — 2026-10-28 18:01:43 KST

- 관측 판정: 미완료
- 종료 사유: 제한시간 종료(미완료)
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 49.820 s
- 시각이 일치하는 odom 쌍: 2489
- 관측된 최소 중심 간격: 1.679 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.063 | 0.028 | 0.733 | 0.735 | 191 |
| robot_b | 미도착 | 0.239 | 0.269 | 0.862 | 0.865 | 454 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: 없음

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "7d87376eea8ee0e8ac65ece1a05a9adfbc15a85f38e47998ce698b8bf48f3ccc",
    "dmpc_runtime/ros_node.py": "a9ef5f7bfef645ffd2d900e08a69a7693e58de4a5be00982bcb40abf120c040f",
    "dmpc_runtime/isaac_test.py": "3b72ff353251bf57b8c1a363ebe3266e190fd6d1b7ab616e3945bc3ff6d4215d",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "841bcd9b2553c23d0a4cace5a8726572921e7dd0095e12b6be9b29dd1f35fb4c"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 191,
        "solver_failed": 16,
        "goal_reached": 2536
      },
      "controller": {
        "mpc_waiting_for_fresh_states": 1,
        "tracking": 690,
        "waiting_for_clock": 3,
        "waiting_for_mpc_heading": 2,
        "Invalid MPC metadata": 5,
        "tracking_mpc_turn": 9,
        "mpc_solver_failed": 70,
        "mpc_goal_reached": 3191,
        "disabled": 1
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 454,
        "solver_failed": 13
      },
      "controller": {
        "waiting_for_fresh_mpc_plan": 1,
        "mpc_waiting_for_fresh_states": 3,
        "tracking": 954,
        "tracking_speed_reduced": 2,
        "proximity_stop": 1,
        "proximity_recovery_wait": 6,
        "tracking_mpc_turn": 749,
        "waiting_for_clock": 8,
        "mpc_solver_failed": 70,
        "disabled": 1
      }
    }
  },
  "final_positions": [
    [
      2.9368460178375244,
      0.0021036677062511444,
      -0.027803262470400086
    ],
    [
      -0.22044439613819122,
      2.9069488048553467,
      1.3014761149426748
    ]
  ],
  "final_control_diagnostics": {
    "robot_b": {
      "plan_age": 0.02,
      "ego_age": 0.01,
      "peer_age": 0.030000000000000002,
      "profile_speed": 0.006974785732808798,
      "profile_yaw_rate": -0.05444072529004137,
      "position_error": 4.54753707817772e-05,
      "measured_speed": 0.0016413178893695654,
      "measured_yaw_rate": -0.0025712837427976512,
      "rpp_speed": 0.006974785732808798,
      "rpp_yaw_rate": -0.04688457844698896,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 4.290274152129927,
      "peer_checks": [
        {
          "robot_id": "robot_a",
          "age": 0.030000000000000002,
          "separation": 4.290274152129927,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.05688049626575273
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "command_speed": 0.006974785732808798,
      "command_yaw_rate": -0.07468594798206454,
      "predicted_clearance": 4.072679201580242,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.006974785732808798,
      "candidate_yaw_rate": -0.07468594798206454,
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "tracking"
    },
    "robot_a": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "mpc_goal_reached"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        0.00017491739708930254,
        0.000157505928655155,
        -4.9367521569365636e-05,
        -8.966188761405647e-05,
        -3.255777359008789,
        -2.8172037601470947,
        -3.4489948749542236,
        -2.69675612449646
      ],
      "measured_velocities": [
        0.00584444310516119,
        -0.006049118936061859,
        0.004839892964810133,
        -0.0026833037845790386,
        0.0048961034044623375,
        0.003409154014661908,
        0.005072237923741341,
        0.0065996055491268635
      ],
      "commanded_positions": [
        0.0,
        0.0,
        0.0,
        0.0,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.46611300110816956,
        -0.560983419418335,
        0.46441909670829773,
        0.559077262878418,
        -0.12371911853551865,
        -3.165797472000122,
        -0.338703453540802,
        -2.868647336959839
      ],
      "measured_velocities": [
        -0.030690934509038925,
        -4.425965309143066,
        -0.0071708098985254765,
        3.76678204536438,
        0.062304362654685974,
        -1.5454175472259521,
        0.052187275141477585,
        1.6741459369659424
      ],
      "commanded_positions": [
        -0.46460999267392955,
        -0.5634364321237159,
        0.46460999267392955,
        0.5634364321237189,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        -1.530650247517315,
        0.0,
        1.530650247517315
      ]
    }
  },
  "final_plans": {
    "robot_a": {
      "valid": false,
      "status": "goal_reached",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    },
    "robot_b": {
      "valid": true,
      "status": "tracking",
      "positions": [
        [
          -0.22047125654361022,
          2.906903140019607
        ],
        [
          -0.21983942145021304,
          2.909133024504806
        ],
        [
          -0.2183426910767321,
          2.9140318990729606
        ],
        [
          -0.21600670321622428,
          2.920686239813788
        ],
        [
          -0.2123737258724566,
          2.9293030942757965
        ],
        [
          -0.20676781657892399,
          2.9401875981293095
        ]
      ],
      "velocities": [
        0.002361169661109974,
        0.020815633947905265,
        0.030410565950940854,
        0.0401215225826278,
        0.05341193304775215,
        0.06905544892162013
      ],
      "yaw_rates": [
        -0.05444072529004137,
        -0.15443500485506795,
        -0.2544234231733343,
        -0.35437658395013216,
        -0.4109641982002092,
        -0.4162727616419217
      ]
    }
  }
}
```

</details>

<details><summary>마지막 실행 메시지</summary>

```text
[ros2_control_node-5] [INFO] [1793178215.626489188] [resource_manager]: Successful 'shutdown' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793178215.626512604] [robot_b.controller_manager]: Shutting down the controller manager.
[ros2_control_node-2] [INFO] [1793178215.627180218] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793178215.627213448] [robot_a.controller_manager]: Shutdown request received....
[ros2_control_node-2] [INFO] [1793178215.627233220] [robot_a.controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-2] [INFO] [1793178215.627243776] [robot_a.controller_manager]: Deactivating controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793178215.627320014] [robot_a.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793178215.627389393] [resource_manager]: 'deactivate' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793178215.627584689] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-2] [INFO] [1793178215.627592521] [resource_manager]: Successful 'deactivate' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793178215.627594602] [resource_manager]: 'shutdown' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793178215.627596223] [resource_manager]: Successful 'shutdown' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793178215.627613135] [robot_a.controller_manager]: Shutting down the controller manager.
[robot_state_publisher-1] [INFO] [1793178215.627688170] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[mpc_rpp_controller-12] [INFO] [1793178215.628114930] [robot_b.mpc_rpp_controller]: Destroying
[dmpc_planner-8] [INFO] [1793178215.630563069] [robot_a.dmpc_planner]: robot_a: shutdown
[INFO] [mpc_rpp_controller-12]: process has finished cleanly [pid 59]
[dmpc_planner-11] [INFO] [1793178215.633815076] [robot_b.dmpc_planner]: robot_b: shutdown
[ros2_control_node-2] [ERROR] [1793178215.639098563] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793178215.639149825] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [INFO] [1793178215.639313931] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793178215.639321460] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793178215.639388225] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[mpc_rpp_controller-9] [INFO] [1793178215.639346809] [robot_a.mpc_rpp_controller]: Destroying
[ros2_control_node-2] [INFO] [1793178215.639390027] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [mpc_rpp_controller-9]: process has finished cleanly [pid 56]
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 49]
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 48]
[ros2_control_node-5] [ERROR] [1793178215.656414881] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [ERROR] [1793178215.656414887] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [INFO] [1793178215.657151803] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793178215.657159680] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-5] [INFO] [1793178215.657232441] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793178215.657234258] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 51]
[INFO] [ros2_control_node-5]: process has finished cleanly [pid 52]
[INFO] [peer_state_relay-7]: process has finished cleanly [pid 54]
[INFO] [peer_state_relay-10]: process has finished cleanly [pid 57]
[INFO] [dmpc_planner-8]: process has finished cleanly [pid 55]
[INFO] [dmpc_planner-11]: process has finished cleanly [pid 58]
```

</details>

<a id="run-20261028_180941_205035__B"></a>

### 시나리오 B — 2026-10-28 18:09:41 KST

- 관측 판정: 미완료
- 종료 사유: 실행 오류: Isaac 입력 없음 또는 초기 위치·방향 불일치. 시뮬레이터를 같은 시나리오로 재시작하세요.
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 19.460 s
- 시각이 일치하는 odom 쌍: 974
- 관측된 최소 중심 간격: 5.967 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 미도착 | 5.983 | 0.000 | 0.001 | 0.000 | 0 |
| robot_b | 미도착 | 4.231 | 1.571 | 0.001 | 0.000 | 0 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: 없음

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "7dbc4a8c21544ab35ac7c5251c49326bf809051130ed590ee06f40fb8d7e6b94",
    "dmpc_runtime/ros_node.py": "7876beecb2d048075c1ae9493c25dfe82165793799cf4adcf4615d5124601870",
    "dmpc_runtime/isaac_test.py": "a914bec4676f410aeca5b4c81563e85d3f3ee9976c13aa459404cbd76ce9fb76",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "70329ebd77bfd225e7cc12c4b577b1acfa36964259ad0596cb5a7c7678fdfb6e"
  },
  "statuses": {
    "robot_a": {
      "planner": {},
      "controller": {}
    },
    "robot_b": {
      "planner": {},
      "controller": {}
    }
  },
  "final_positions": [
    [
      -2.9834163188934326,
      2.7739872621168615e-08,
      1.2703030901492713e-06
    ],
    [
      2.983424186706543,
      -2.1962826224353194e-08,
      -3.1415915139170227
    ]
  ],
  "final_control_diagnostics": {},
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": false,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        2.1358535377657972e-05,
        7.6261803769739345e-06,
        -2.266061528644059e-05,
        -1.194273318105843e-05,
        0.160379558801651,
        0.16040953993797302,
        0.16040855646133423,
        0.16039274632930756
      ],
      "measured_velocities": [
        0.0037974112201482058,
        -0.0076613775454461575,
        0.005162721499800682,
        -0.0012515640119090676,
        0.005815432406961918,
        0.0029833950102329254,
        0.004799193236976862,
        0.007472946774214506
      ],
      "commanded_positions": [],
      "commanded_velocities": []
    },
    "robot_b": {
      "command_valid": false,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        1.828696440497879e-05,
        1.6230547771556303e-05,
        -8.932837772590574e-06,
        -2.585525726317428e-05,
        0.1604822427034378,
        0.16050848364830017,
        0.1604999154806137,
        0.16047818958759308
      ],
      "measured_velocities": [
        0.002583583351224661,
        -0.007201660890132189,
        0.005018546711653471,
        -0.0012857901165261865,
        0.0067607332020998,
        0.0033874232321977615,
        0.005012948531657457,
        0.007541237398982048
      ],
      "commanded_positions": [],
      "commanded_velocities": []
    }
  },
  "final_plans": {}
}
```

</details>

<a id="run-20261028_181506_471976__B"></a>

### 시나리오 B — 2026-10-28 18:15:06 KST

- 관측 판정: 미완료
- 종료 사유: 제한시간 종료(미완료)
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 64.560 s
- 시각이 일치하는 odom 쌍: 3229
- 관측된 최소 중심 간격: 1.547 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 미도착 | 1.669 | 0.580 | 0.825 | 0.829 | 112 |
| robot_b | 미도착 | 3.892 | 0.014 | 0.589 | 0.596 | 579 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: 없음

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "7dbc4a8c21544ab35ac7c5251c49326bf809051130ed590ee06f40fb8d7e6b94",
    "dmpc_runtime/ros_node.py": "7876beecb2d048075c1ae9493c25dfe82165793799cf4adcf4615d5124601870",
    "dmpc_runtime/isaac_test.py": "a914bec4676f410aeca5b4c81563e85d3f3ee9976c13aa459404cbd76ce9fb76",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "70329ebd77bfd225e7cc12c4b577b1acfa36964259ad0596cb5a7c7678fdfb6e"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 112,
        "solver_failed": 493
      },
      "controller": {
        "waiting_for_fresh_gt": 1,
        "waiting_for_fresh_mpc_plan": 3,
        "mpc_waiting_for_fresh_states": 6,
        "tracking": 366,
        "tracking_speed_reduced": 19,
        "tracking_mpc_turn": 39,
        "mpc_solver_failed": 2441,
        "disabled": 1
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 579,
        "solver_failed": 28
      },
      "controller": {
        "waiting_for_fresh_gt": 1,
        "mpc_waiting_for_fresh_states": 2,
        "tracking": 746,
        "tracking_speed_reduced": 50,
        "waiting_for_clock": 20,
        "proximity_stop": 1,
        "proximity_recovery_wait": 6,
        "tracking_mpc_turn": 1417,
        "mpc_solver_failed": 167,
        "disabled": 1
      }
    }
  },
  "final_positions": [
    [
      1.461421251296997,
      0.6476430296897888,
      0.5799401954552363
    ],
    [
      3.055466651916504,
      0.5886512398719788,
      1.5850379636994019
    ]
  ],
  "final_control_diagnostics": {
    "robot_a": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "mpc_solver_failed"
    },
    "robot_b": {
      "plan_age": 0.04,
      "ego_age": 0.01,
      "peer_age": 0.01,
      "profile_speed": 0.03499999448283214,
      "profile_yaw_rate": 0.014323101497238786,
      "position_error": 0.0003998057162366647,
      "measured_speed": -0.00017527433806564726,
      "measured_yaw_rate": 0.00032503544913069487,
      "rpp_speed": 0.03499999448283214,
      "rpp_yaw_rate": 0.0061469101356306476,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 1.5951373357055356,
      "peer_checks": [
        {
          "robot_id": "robot_a",
          "age": 0.01,
          "separation": 1.5951373357055356,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.03283996983067184
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "command_speed": 0.015000000000000001,
      "command_yaw_rate": 0.0026343904733952617,
      "predicted_clearance": 1.4165115432589985,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.015000000000000001,
      "candidate_yaw_rate": 0.0026343904733952617,
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "tracking"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.7085068821907043,
        0.8024387359619141,
        0.7078509330749512,
        -0.8024941682815552,
        5.558101654052734,
        -4.129077434539795,
        4.750583648681641,
        -5.0677571296691895
      ],
      "measured_velocities": [
        -0.017797332257032394,
        -0.011810289695858955,
        0.0140742938965559,
        0.002842763438820839,
        0.011937528848648071,
        -0.010744734667241573,
        -0.013752048835158348,
        0.002307258313521743
      ],
      "commanded_positions": [
        -0.7072540805259699,
        0.8026709566364212,
        0.7072540805259699,
        -0.8026709566364207,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.695414125919342,
        0.747761607170105,
        0.691878616809845,
        -0.7467167973518372,
        0.38238412141799927,
        0.41207486391067505,
        -0.6361535787582397,
        0.026715602725744247
      ],
      "measured_velocities": [
        -0.0544387511909008,
        0.024597035720944405,
        -0.00807714369148016,
        -0.008660513907670975,
        0.046535927802324295,
        0.018905766308307648,
        0.0029434235766530037,
        -0.006366944871842861
      ],
      "commanded_positions": [
        -0.6922868158534635,
        0.7461450102900837,
        0.692286815853464,
        -0.7461450102900837,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    }
  },
  "final_plans": {
    "robot_b": {
      "valid": true,
      "status": "tracking",
      "positions": [
        [
          3.0554666780008173,
          0.5886494683395993
        ],
        [
          3.0553051563664555,
          0.5986481599365134
        ],
        [
          3.0547570981081353,
          0.628643143146627
        ],
        [
          3.0537746389363267,
          0.6786334795608312
        ],
        [
          3.0522139856347383,
          0.7486160129250716
        ],
        [
          3.0490011991411183,
          0.8385568366289898
        ]
      ],
      "velocities": [
        0.0,
        0.09999998423666326,
        0.19999996595873593,
        0.29999994441287087,
        0.3999999184940206,
        0.49999988648684174
      ],
      "yaw_rates": [
        0.014323101497238786,
        0.010456970597430981,
        0.004231054361637467,
        0.021492215413633695,
        0.10956348857509371,
        0.20954724424961155
      ]
    },
    "robot_a": {
      "valid": false,
      "status": "solver_failed",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    }
  }
}
```

</details>

<details><summary>마지막 실행 메시지</summary>

```text
[ros2_control_node-5] [INFO] [1793179026.706293069] [resource_manager]: Successful 'shutdown' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793179026.706312886] [robot_b.controller_manager]: Shutting down the controller manager.
[dmpc_planner-8] [INFO] [1793179026.706441358] [robot_a.dmpc_planner]: robot_a: shutdown
[ros2_control_node-2] [INFO] [1793179026.706591741] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793179026.706623154] [robot_a.controller_manager]: Shutdown request received....
[ros2_control_node-2] [INFO] [1793179026.706642616] [robot_a.controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-2] [INFO] [1793179026.706652518] [robot_a.controller_manager]: Deactivating controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793179026.706733643] [robot_a.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793179026.706803567] [resource_manager]: 'deactivate' hardware 'robot_a_isaac_system'
[robot_state_publisher-1] [INFO] [1793179026.706949127] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[mpc_rpp_controller-12] [INFO] [1793179026.706959226] [robot_b.mpc_rpp_controller]: Destroying
[ros2_control_node-2] [INFO] [1793179026.707021573] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-2] [INFO] [1793179026.707027143] [resource_manager]: Successful 'deactivate' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793179026.707028850] [resource_manager]: 'shutdown' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793179026.707030412] [resource_manager]: Successful 'shutdown' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793179026.707047762] [robot_a.controller_manager]: Shutting down the controller manager.
[INFO] [mpc_rpp_controller-12]: process has finished cleanly [pid 59]
[ros2_control_node-5] [ERROR] [1793179026.715873499] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [ERROR] [1793179026.715873488] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [INFO] [1793179026.716224546] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793179026.716233206] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-5] [INFO] [1793179026.716299949] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793179026.716301648] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [ERROR] [1793179026.716969891] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793179026.717001323] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [INFO] [1793179026.717334837] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793179026.717342038] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793179026.717456129] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793179026.717458529] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[mpc_rpp_controller-9] [INFO] [1793179026.718321651] [robot_a.mpc_rpp_controller]: Destroying
[dmpc_planner-11] [INFO] [1793179026.719125054] [robot_b.dmpc_planner]: robot_b: shutdown
[INFO] [mpc_rpp_controller-9]: process has finished cleanly [pid 56]
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 49]
[INFO] [ros2_control_node-5]: process has finished cleanly [pid 52]
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 51]
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 48]
[INFO] [dmpc_planner-8]: process has finished cleanly [pid 55]
[INFO] [dmpc_planner-11]: process has finished cleanly [pid 58]
[INFO] [peer_state_relay-7]: process has finished cleanly [pid 54]
[INFO] [peer_state_relay-10]: process has finished cleanly [pid 57]
```

</details>

<a id="run-20261028_182113_234011__B"></a>

### 시나리오 B — 2026-10-28 18:21:13 KST

- 관측 판정: 위치 도착(접촉·방향 별도)
- 종료 사유: 두 로봇 목표 도착
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 23.200 s
- 시각이 일치하는 odom 쌍: 1160
- 관측된 최소 중심 간격: 1.804 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.087 | 0.049 | 0.756 | 0.763 | 157 |
| robot_b | 도착 | 0.092 | 0.771 | 0.834 | 0.840 | 188 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: 없음

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "7dbc4a8c21544ab35ac7c5251c49326bf809051130ed590ee06f40fb8d7e6b94",
    "dmpc_runtime/ros_node.py": "20a6b1845972531839c75b9a640d380fb7a10904eed9eb10aea7919c0e09a6d3",
    "dmpc_runtime/isaac_test.py": "a39af13fc425467065f1806703b52072c9a26d1c62f2a010f36dee4aa3a1ec94",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "ff7f177d3df754eeedf531c6c48e2e4ed13d72c26d80a47fb618d34b58551ead"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 4,
        "tracking": 157,
        "solver_failed": 62,
        "goal_reached": 5
      },
      "controller": {
        "waiting_for_fresh_gt": 2,
        "mpc_waiting_for_fresh_states": 2,
        "tracking": 598,
        "mpc_solver_failed": 295,
        "tracking_mpc_turn": 8,
        "waiting_for_clock": 2,
        "mpc_goal_reached": 1,
        "disabled": 1
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 4,
        "tracking": 187,
        "goal_reached": 328
      },
      "controller": {
        "waiting_for_fresh_gt": 1,
        "mpc_waiting_for_fresh_states": 2,
        "tracking": 694,
        "tracking_speed_reduced": 8,
        "proximity_stop": 2,
        "proximity_recovery_wait": 5,
        "waiting_for_clock": 4,
        "mpc_goal_reached": 456,
        "disabled": 1
      }
    }
  },
  "final_positions": [
    [
      2.913576602935791,
      0.005158283282071352,
      -0.04894752726497526
    ],
    [
      0.016343452036380768,
      2.909707546234131,
      2.3422724750124364
    ]
  ],
  "final_control_diagnostics": {
    "robot_b": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "mpc_goal_reached"
    },
    "robot_a": {
      "plan_age": 0.08,
      "ego_age": 0.01,
      "peer_age": 0.01,
      "profile_speed": 0.035787025565906135,
      "profile_yaw_rate": 0.001345962606238775,
      "position_error": 0.00013079534097605307,
      "measured_speed": 0.05121709544734738,
      "measured_yaw_rate": 0.00034490750039383724,
      "rpp_speed": 0.035787025565906135,
      "rpp_yaw_rate": 0.0,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 4.101376952694861,
      "peer_checks": [
        {
          "robot_id": "robot_b",
          "age": 0.01,
          "separation": 4.101376952694861,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.03283996983067184
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "command_speed": 0.035787025565906135,
      "command_yaw_rate": 0.0,
      "predicted_clearance": 3.9113371336968163,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.035787025565906135,
      "candidate_yaw_rate": 0.0,
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "tracking"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        0.0010076845064759254,
        0.0005295037408359349,
        -0.0015574343269690871,
        -0.0011343187652528286,
        -5.046985149383545,
        -5.1081929206848145,
        -5.022538661956787,
        -5.006553649902344
      ],
      "measured_velocities": [
        0.01601267419755459,
        0.000758676789700985,
        -0.012407669797539711,
        0.029175572097301483,
        0.482488751411438,
        0.4926843047142029,
        0.4968538284301758,
        0.5169000029563904
      ],
      "commanded_positions": [
        0.0,
        0.0,
        0.0,
        0.0,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.4854579890251663,
        0.4854579890251663,
        0.4854579890251663,
        0.4854579890251663
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.1005253866314888,
        -0.27095022797584534,
        0.10105935484170914,
        0.2698395550251007,
        4.053985595703125,
        -4.878537178039551,
        4.038909912109375,
        -4.518016815185547
      ],
      "measured_velocities": [
        0.007742961868643761,
        0.01579940877854824,
        0.004314656835049391,
        -0.03821853548288345,
        -0.007033612113445997,
        0.012255718931555748,
        -0.006030883640050888,
        -0.03479542210698128
      ],
      "commanded_positions": [
        -0.10095585368516763,
        -0.27209714566223864,
        0.10095585368516763,
        0.27209714566223864,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    }
  },
  "final_plans": {
    "robot_b": {
      "valid": false,
      "status": "goal_reached",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    },
    "robot_a": {
      "valid": true,
      "status": "tracking",
      "positions": [
        [
          2.9077680082451045,
          0.005443817316423532
        ],
        [
          2.9153901422340054,
          0.0050710595975340415
        ],
        [
          2.917628734276675,
          0.004962295786843519
        ],
        [
          2.91884227619408,
          0.004903774564823481
        ],
        [
          2.919478159462565,
          0.004873360575711239
        ],
        [
          2.91980397509965,
          0.004857910811057304
        ]
      ],
      "velocities": [
        0.06184812895599712,
        0.014464304610377145,
        0.007948022239680212,
        0.00420149929958243,
        0.0021646027042658902,
        0.0010972146738206995
      ],
      "yaw_rates": [
        0.001345962606238775,
        0.0017231918324108876,
        0.0019221638363899351,
        0.002027090050143138,
        0.002082401689513067,
        0.002111540610241716
      ]
    }
  }
}
```

</details>

<a id="run-20261028_182302_278884__B"></a>

### 시나리오 B — 2026-10-28 18:23:02 KST

- 관측 판정: 위치 도착(접촉·방향 별도)
- 종료 사유: 두 로봇 목표 도착
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 35.880 s
- 시각이 일치하는 odom 쌍: 1790
- 관측된 최소 중심 간격: 1.745 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.075 | 0.065 | 0.766 | 0.771 | 191 |
| robot_b | 도착 | 0.160 | 0.228 | 0.820 | 0.828 | 324 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: 없음

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "7dbc4a8c21544ab35ac7c5251c49326bf809051130ed590ee06f40fb8d7e6b94",
    "dmpc_runtime/ros_node.py": "20a6b1845972531839c75b9a640d380fb7a10904eed9eb10aea7919c0e09a6d3",
    "dmpc_runtime/isaac_test.py": "a39af13fc425467065f1806703b52072c9a26d1c62f2a010f36dee4aa3a1ec94",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "ff7f177d3df754eeedf531c6c48e2e4ed13d72c26d80a47fb618d34b58551ead"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 191,
        "solver_failed": 23,
        "goal_reached": 1220
      },
      "controller": {
        "waiting_for_fresh_gt": 3,
        "mpc_waiting_for_fresh_states": 2,
        "tracking": 701,
        "waiting_for_clock": 7,
        "tracking_mpc_turn": 14,
        "mpc_solver_failed": 114,
        "mpc_goal_reached": 1676,
        "disabled": 1
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 7,
        "tracking": 324,
        "solver_failed": 13,
        "goal_reached": 6
      },
      "controller": {
        "waiting_for_fresh_gt": 1,
        "mpc_waiting_for_fresh_states": 1,
        "tracking": 890,
        "waiting_for_clock": 13,
        "tracking_speed_reduced": 3,
        "proximity_stop": 1,
        "proximity_recovery_wait": 5,
        "tracking_mpc_turn": 325,
        "mpc_solver_failed": 77,
        "mpc_goal_reached": 4,
        "disabled": 1
      }
    }
  },
  "final_positions": [
    [
      2.925471782684326,
      0.006217967718839645,
      -0.06481458414440494
    ],
    [
      -0.1599106341600418,
      3.010575532913208,
      1.342698865549925
    ]
  ],
  "final_control_diagnostics": {
    "robot_b": {
      "plan_age": 0.07,
      "ego_age": 0.01,
      "peer_age": 0.030000000000000002,
      "profile_speed": 0.026532107756474802,
      "profile_yaw_rate": -0.18208123154642916,
      "position_error": 0.0004408844481800198,
      "measured_speed": 0.0500853467837038,
      "measured_yaw_rate": -0.1243483945502349,
      "rpp_speed": 0.026532107756474802,
      "rpp_yaw_rate": 0.0,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 4.306310853299526,
      "peer_checks": [
        {
          "robot_id": "robot_a",
          "age": 0.030000000000000002,
          "separation": 4.306310853299526,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.05688049626575273
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "command_speed": 0.026532107756474802,
      "command_yaw_rate": -0.19186145572637658,
      "predicted_clearance": 4.023766923745979,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.026532107756474802,
      "candidate_yaw_rate": -0.19186145572637658,
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "tracking"
    },
    "robot_a": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "mpc_goal_reached"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        0.0003347518213558942,
        0.000349171255948022,
        -0.000673791510052979,
        -0.0009582615457475185,
        -3.8835527896881104,
        -3.635066032409668,
        -3.940850019454956,
        -3.5843703746795654
      ],
      "measured_velocities": [
        0.00762573629617691,
        -0.003875548020005226,
        -0.0012755179777741432,
        -0.013970151543617249,
        0.0033168725203722715,
        0.004143327474594116,
        0.009131519123911858,
        -0.0006868867203593254
      ],
      "commanded_positions": [
        0.0,
        0.0,
        0.0,
        0.0,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.6381557583808899,
        -0.9301490187644958,
        0.6342398524284363,
        0.9318119883537292,
        -2.306504249572754,
        -4.37772798538208,
        -2.6162843704223633,
        -3.6606290340423584
      ],
      "measured_velocities": [
        0.07620643079280853,
        -0.4706481695175171,
        0.051864128559827805,
        -0.08694195747375488,
        0.7792503833770752,
        0.24205592274665833,
        0.8860353827476501,
        0.2580365240573883
      ],
      "commanded_positions": [
        -0.6365804383324312,
        -0.930999,
        0.6365804383324312,
        0.930999,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        1.0118030061602727,
        0.13716235200242552,
        1.0370128122039706,
        0.4604633228762238
      ]
    }
  },
  "final_plans": {
    "robot_b": {
      "valid": true,
      "status": "tracking",
      "positions": [
        [
          -0.1609054207801818,
          3.0067591667175297
        ],
        [
          -0.15969525352932698,
          3.011925551299746
        ],
        [
          -0.15969482040326083,
          3.0119270784729464
        ],
        [
          -0.1596943806503023,
          3.0119283610061363
        ],
        [
          -0.15969370971463595,
          3.0119300113268275
        ],
        [
          -0.15969262218172578,
          3.0119322956960457
        ]
      ],
      "velocities": [
        0.053054318734425364,
        9.896778524243746e-06,
        5.978764432208095e-06,
        7.581167788661802e-06,
        1.0236144675486497e-05,
        1.5067753646412972e-05
      ],
      "yaw_rates": [
        -0.18208123154642916,
        -0.23961132863416862,
        -0.26967957020916267,
        -0.28505179549062937,
        -0.2922603207482858,
        -0.29437122464034626
      ]
    },
    "robot_a": {
      "valid": false,
      "status": "goal_reached",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    }
  }
}
```

</details>

<a id="run-20261028_182526_579211__A"></a>

### 시나리오 A — 2026-10-28 18:25:26 KST

- 관측 판정: 위치 도착(접촉·방향 별도)
- 종료 사유: 두 로봇 목표 도착
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 19.200 s
- 시각이 일치하는 odom 쌍: 961
- 관측된 최소 중심 간격: 2.241 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.082 | 0.373 | 1.004 | 1.000 | 138 |
| robot_b | 도착 | 0.085 | 0.334 | 0.954 | 0.962 | 178 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: 없음

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "d152cebb683c1e62a08c1822fe5daea1c8203aa668f27bef7dc995a3a643ba0b",
    "dmpc_runtime/ros_node.py": "20a6b1845972531839c75b9a640d380fb7a10904eed9eb10aea7919c0e09a6d3",
    "dmpc_runtime/isaac_test.py": "a39af13fc425467065f1806703b52072c9a26d1c62f2a010f36dee4aa3a1ec94",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "ff7f177d3df754eeedf531c6c48e2e4ed13d72c26d80a47fb618d34b58551ead"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 4,
        "tracking": 138,
        "goal_reached": 417
      },
      "controller": {
        "waiting_for_fresh_mpc_plan": 1,
        "mpc_waiting_for_fresh_states": 4,
        "tracking": 508,
        "waiting_for_clock": 4,
        "tracking_mpc_turn": 1,
        "mpc_goal_reached": 576,
        "disabled": 1
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 177,
        "solver_failed": 4,
        "goal_reached": 7
      },
      "controller": {
        "mpc_solver_failed": 13,
        "tracking": 666,
        "waiting_for_clock": 4,
        "tracking_mpc_turn": 1,
        "mpc_goal_reached": 5,
        "disabled": 1
      }
    }
  },
  "final_positions": [
    [
      2.921499252319336,
      -0.025206971913576126,
      0.37336069723987775
    ],
    [
      -2.921893358230591,
      0.034699730575084686,
      -2.808044003731466
    ]
  ],
  "final_control_diagnostics": {
    "robot_a": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "mpc_goal_reached"
    },
    "robot_b": {
      "plan_age": 0.11,
      "ego_age": 0.01,
      "peer_age": 0.030000000000000002,
      "profile_speed": 0.028778125067640943,
      "profile_yaw_rate": -0.007959122191675418,
      "position_error": 0.000654226599888203,
      "measured_speed": 0.05016450074892357,
      "measured_yaw_rate": -0.0005228843705868762,
      "rpp_speed": 0.028778125067640943,
      "rpp_yaw_rate": 0.0,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 5.843245172293684,
      "peer_checks": [
        {
          "robot_id": "robot_a",
          "age": 0.030000000000000002,
          "separation": 5.843245172293684,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.05688049626575273
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "command_speed": 0.028778125067640943,
      "command_yaw_rate": 0.0,
      "predicted_clearance": 5.64877079402886,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.028778125067640943,
      "candidate_yaw_rate": 0.0,
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "tracking"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.00038919097278267145,
        -0.002208151388913393,
        0.0013634397182613611,
        0.0024394907522946596,
        -1.0086199045181274,
        1.2106192111968994,
        -0.9325355291366577,
        1.2608224153518677
      ],
      "measured_velocities": [
        -0.005587768740952015,
        -0.028787842020392418,
        0.02448473684489727,
        0.02910744771361351,
        0.014072848483920097,
        -0.017303641885519028,
        -0.01285128016024828,
        0.03290949761867523
      ],
      "commanded_positions": [
        0.0,
        -0.00047004947476898123,
        0.0,
        0.00047004947476898123,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.0002490299812052399,
        -0.0019228346645832062,
        0.002253815531730652,
        0.0023539781104773283,
        -2.4654760360717773,
        -0.8940911293029785,
        -2.530444622039795,
        -0.8724485039710999
      ],
      "measured_velocities": [
        -0.017585307359695435,
        -0.05974695086479187,
        0.03494727239012718,
        0.041277043521404266,
        0.5030365586280823,
        0.4504673480987549,
        0.47168058156967163,
        0.511195182800293
      ],
      "commanded_positions": [
        0.0,
        0.0,
        0.0,
        0.0,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.4293590125973211,
        0.4293590125973211,
        0.4293590125973211,
        0.4293590125973211
      ]
    }
  },
  "final_plans": {
    "robot_b": {
      "valid": true,
      "status": "tracking",
      "positions": [
        [
          -2.9155635833740234,
          0.03689993545413017
        ],
        [
          -2.9229494057788354,
          0.03434553367063534
        ],
        [
          -2.924900067206309,
          0.03367519014955651
        ],
        [
          -2.9259548810460143,
          0.033315476359329625
        ],
        [
          -2.9265077406048676,
          0.03312854974980635
        ],
        [
          -2.9267916226254145,
          0.03303343546482455
        ]
      ],
      "velocities": [
        0.06481844927070157,
        0.013332271837757821,
        0.007294027214335678,
        0.003850598814479726,
        0.0019854567943416424,
        0.0010084668079480533
      ],
      "yaw_rates": [
        -0.007959122191675418,
        -0.011059352724618521,
        -0.01269414943784493,
        -0.013556039412180925,
        -0.014010296982859001,
        -0.01424958223360122
      ]
    },
    "robot_a": {
      "valid": false,
      "status": "goal_reached",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    }
  }
}
```

</details>

<a id="run-20261028_182707_425000__B"></a>

### 시나리오 B — 2026-10-28 18:27:07 KST

- 관측 판정: 위치 도착(접촉·방향 별도)
- 종료 사유: 두 로봇 목표 도착 (저속 접근 위치·방향 재현, 바퀴 상태 초기화)
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 6.220 s
- 시각이 일치하는 odom 쌍: 312
- 관측된 최소 중심 간격: 4.157 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.085 | 0.000 | 0.138 | 0.140 | 22 |
| robot_b | 도착 | 0.102 | 1.031 | 0.131 | 0.100 | 56 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: 없음

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "d152cebb683c1e62a08c1822fe5daea1c8203aa668f27bef7dc995a3a643ba0b",
    "dmpc_runtime/ros_node.py": "20a6b1845972531839c75b9a640d380fb7a10904eed9eb10aea7919c0e09a6d3",
    "dmpc_runtime/isaac_test.py": "a39af13fc425467065f1806703b52072c9a26d1c62f2a010f36dee4aa3a1ec94",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "ff7f177d3df754eeedf531c6c48e2e4ed13d72c26d80a47fb618d34b58551ead"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 3,
        "tracking": 22,
        "goal_reached": 320
      },
      "controller": {
        "mpc_waiting_for_fresh_states": 1,
        "tracking": 75,
        "mpc_goal_reached": 436,
        "disabled": 1
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 5,
        "tracking": 56,
        "goal_reached": 8
      },
      "controller": {
        "invalid_peer_state": 1,
        "waiting_for_fresh_gt": 2,
        "mpc_waiting_for_fresh_states": 5,
        "waiting_for_clock": 4,
        "tracking_mpc_turn": 64,
        "tracking": 131,
        "mpc_goal_reached": 6,
        "disabled": 1
      }
    }
  },
  "final_positions": [
    [
      2.914619207382202,
      3.154572368657682e-06,
      6.991087996809278e-06
    ],
    [
      -0.05664561688899994,
      3.084552049636841,
      0.5400968768862974
    ]
  ],
  "final_control_diagnostics": {
    "robot_b": {
      "plan_age": 0.030000000000000002,
      "ego_age": 0.01,
      "peer_age": 0.01,
      "profile_speed": 0.0478996893354119,
      "profile_yaw_rate": -0.08477421813437011,
      "position_error": 2.1880597739237085e-05,
      "measured_speed": 0.05452987352874117,
      "measured_yaw_rate": -0.11874321429533137,
      "rpp_speed": 0.0478996893354119,
      "rpp_yaw_rate": 0.0,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 4.282964564659846,
      "peer_checks": [
        {
          "robot_id": "robot_a",
          "age": 0.01,
          "separation": 4.282964564659846,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.03283996983067184
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "command_speed": 0.0478996893354119,
      "command_yaw_rate": -0.04281971064539569,
      "predicted_clearance": 4.024161879136755,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.0478996893354119,
      "candidate_yaw_rate": -0.04281971064539569,
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "tracking"
    },
    "robot_a": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "mpc_goal_reached"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        3.892636868840782e-06,
        -2.40314839174971e-05,
        2.8851876777480356e-05,
        -6.103641112531477e-07,
        1.598328948020935,
        1.598305344581604,
        1.5982909202575684,
        1.5983412265777588
      ],
      "measured_velocities": [
        0.003594004549086094,
        -0.00881127081811428,
        0.006213545333594084,
        -0.001790382550098002,
        0.006006733980029821,
        0.0023792048450559378,
        0.0042390706948935986,
        0.007131054066121578
      ],
      "commanded_positions": [
        0.0,
        0.0,
        0.0,
        0.0,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.3036949932575226,
        -0.8351508378982544,
        0.30362820625305176,
        0.8303367495536804,
        4.986288070678711,
        0.6313377618789673,
        4.762084484100342,
        1.4995789527893066
      ],
      "measured_velocities": [
        0.3948947787284851,
        0.2841614782810211,
        -0.3055870831012726,
        -0.8073630332946777,
        0.45769864320755005,
        0.6749410629272461,
        0.9776347279548645,
        -0.6709543466567993
      ],
      "commanded_positions": [
        -0.30827062703561264,
        -0.8409989999999999,
        0.30827062703561264,
        0.8409989999999999,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.41280865698683905,
        1.0508410956511922,
        1.0621746636428993,
        -0.350129778135205
      ]
    }
  },
  "final_plans": {
    "robot_a": {
      "valid": false,
      "status": "goal_reached",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    },
    "robot_b": {
      "valid": true,
      "status": "tracking",
      "positions": [
        [
          -0.05848131701350212,
          3.0834755897521973
        ],
        [
          -0.05129288069143657,
          3.0877581997362546
        ],
        [
          -0.04803713249043946,
          3.0896454800472797
        ],
        [
          -0.04683109010714139,
          3.0903398609389177
        ],
        [
          -0.0465472352566772,
          3.090504075227542
        ],
        [
          -0.04651245989424356,
          3.090524471784157
        ]
      ],
      "velocities": [
        0.05699256431739631,
        0.02668298104411494,
        0.010949132271881015,
        0.0029674158200754096,
        0.0003119140369019979,
        9.124217651258935e-05
      ],
      "yaw_rates": [
        -0.08477421813437011,
        -0.02852901734884224,
        0.0038937645310529066,
        0.021776424485143164,
        0.031345606148371986,
        0.03641912319747296
      ]
    }
  }
}
```

</details>

<details><summary>마지막 실행 메시지</summary>

```text
[ros2_control_node-5] [INFO] [1793179639.065715386] [resource_manager]: Successful 'shutdown' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793179639.065746257] [robot_b.controller_manager]: Shutting down the controller manager.
[ros2_control_node-2] [INFO] [1793179639.065910946] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793179639.065943466] [robot_a.controller_manager]: Shutdown request received....
[ros2_control_node-2] [INFO] [1793179639.065963829] [robot_a.controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-2] [INFO] [1793179639.065973225] [robot_a.controller_manager]: Deactivating controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793179639.066131246] [robot_a.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793179639.066227056] [resource_manager]: 'deactivate' hardware 'robot_a_isaac_system'
[robot_state_publisher-1] [INFO] [1793179639.066313472] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793179639.066496140] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-2] [INFO] [1793179639.066500923] [resource_manager]: Successful 'deactivate' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793179639.066502538] [resource_manager]: 'shutdown' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793179639.066503987] [resource_manager]: Successful 'shutdown' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793179639.066518392] [robot_a.controller_manager]: Shutting down the controller manager.
[mpc_rpp_controller-12] [INFO] [1793179639.067137023] [robot_b.mpc_rpp_controller]: Destroying
[dmpc_planner-8] [INFO] [1793179639.068567866] [robot_a.dmpc_planner]: robot_a: shutdown
[INFO] [mpc_rpp_controller-12]: process has finished cleanly [pid 61]
[dmpc_planner-11] [INFO] [1793179639.070417136] [robot_b.dmpc_planner]: robot_b: shutdown
[ros2_control_node-5] [ERROR] [1793179639.072678626] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [ERROR] [1793179639.072678617] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [INFO] [1793179639.073090337] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793179639.073097331] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-5] [INFO] [1793179639.073158795] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793179639.073160496] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[mpc_rpp_controller-9] [INFO] [1793179639.076765881] [robot_a.mpc_rpp_controller]: Destroying
[INFO] [ros2_control_node-5]: process has finished cleanly [pid 52]
[INFO] [mpc_rpp_controller-9]: process has finished cleanly [pid 56]
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 48]
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 51]
[ros2_control_node-2] [ERROR] [1793179639.092925357] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793179639.092921374] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [INFO] [1793179639.093444918] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793179639.093452753] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793179639.093519784] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793179639.093521524] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 49]
[INFO] [dmpc_planner-11]: process has finished cleanly [pid 60]
[INFO] [dmpc_planner-8]: process has finished cleanly [pid 55]
[INFO] [peer_state_relay-10]: process has finished cleanly [pid 57]
[INFO] [peer_state_relay-7]: process has finished cleanly [pid 54]
```

</details>

<a id="run-20261029_092337_090044__B"></a>

### 시나리오 B — 2026-10-29 09:23:37 KST

- 관측 판정: 위치 도착(접촉·방향 별도)
- 종료 사유: 두 로봇 목표 도착
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 23.740 s
- 시각이 일치하는 odom 쌍: 1188
- 관측된 최소 중심 간격: 1.666 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.080 | 0.005 | 0.718 | 0.723 | 171 |
| robot_b | 도착 | 0.097 | 0.351 | 0.855 | 0.860 | 220 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [dmpc_planner-11]: process has died [pid 57, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_iko05dmp --params-file /tmp/launch_params_7ciii0b5']. / [ERROR] [peer_state_relay-10]: process has died [pid 56, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_nzrd8bbm']. / [ERROR] [peer_state_relay-7]: process has died [pid 53, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_jcql4spz']. / [ERROR] [dmpc_planner-8]: process has died [pid 54, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_lyuc2prd --params-file /tmp/launch_params_ys0936fe'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "d152cebb683c1e62a08c1822fe5daea1c8203aa668f27bef7dc995a3a643ba0b",
    "dmpc_runtime/ros_node.py": "20a6b1845972531839c75b9a640d380fb7a10904eed9eb10aea7919c0e09a6d3",
    "dmpc_runtime/isaac_test.py": "c42c93a750f4127d92f5f26798c79dc50fcdbe021372670d1b4588e5c4dd4c2d",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "ff7f177d3df754eeedf531c6c48e2e4ed13d72c26d80a47fb618d34b58551ead"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 171,
        "solver_failed": 22,
        "goal_reached": 253
      },
      "controller": {
        "waiting_for_fresh_gt": 5,
        "waiting_for_fresh_mpc_plan": 1,
        "mpc_waiting_for_fresh_states": 5,
        "tracking": 666,
        "waiting_for_clock": 7,
        "tracking_mpc_turn": 2,
        "mpc_solver_failed": 106,
        "expired_mpc_plan": 1,
        "mpc_goal_reached": 354
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 220
      },
      "controller": {
        "waiting_for_fresh_gt": 5,
        "waiting_for_fresh_mpc_plan": 1,
        "mpc_waiting_for_fresh_states": 5,
        "tracking": 754,
        "waiting_for_clock": 7,
        "proximity_stop": 2,
        "proximity_recovery_wait": 6,
        "tracking_mpc_turn": 90,
        "expired_mpc_plan": 1
      }
    }
  },
  "final_positions": [
    [
      2.9197518825531006,
      -3.80803867301438e-05,
      0.005457801966596938
    ],
    [
      -0.06203621253371239,
      2.9257428646087646,
      1.922120101406596
    ]
  ],
  "final_control_diagnostics": {
    "robot_a": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "mpc_goal_reached"
    },
    "robot_b": {
      "plan_age": 0.060000000000000005,
      "ego_age": 0.0,
      "peer_age": 0.02,
      "profile_speed": 0.04779272973270112,
      "profile_yaw_rate": -0.0778986610985985,
      "position_error": 5.839386746580947e-05,
      "measured_speed": 0.050275405436238166,
      "measured_yaw_rate": -0.09858997171002776,
      "rpp_speed": 0.04779272973270112,
      "rpp_yaw_rate": -0.08975796575739402,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 4.176555535866061,
      "peer_checks": [
        {
          "robot_id": "robot_a",
          "age": 0.02,
          "separation": 4.176555535866061,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.04644273072245939
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "command_speed": 0.04779272973270112,
      "command_yaw_rate": -0.08975796575739402,
      "predicted_clearance": 3.942388327433445,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.04779272973270112,
      "candidate_yaw_rate": -0.08975796575739402,
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": false,
      "status": "tracking"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        2.001756183744874e-05,
        -0.00021134206326678395,
        0.000478020723676309,
        -0.00010541973460931331,
        -3.605733871459961,
        -2.8417835235595703,
        -3.6317617893218994,
        -2.7236990928649902
      ],
      "measured_velocities": [
        0.0030902817379683256,
        -0.009857340715825558,
        0.011614028364419937,
        -0.002997622126713395,
        0.006634179037064314,
        0.0020768435206264257,
        0.0015044347383081913,
        0.006477050483226776
      ],
      "commanded_positions": [
        0.0,
        0.0,
        0.0,
        0.0,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.3117735981941223,
        -0.7023193836212158,
        0.31186774373054504,
        0.6985757946968079,
        -3.1403417587280273,
        -3.6076395511627197,
        -2.9497814178466797,
        -3.0970728397369385
      ],
      "measured_velocities": [
        0.11453912407159805,
        -0.6830459833145142,
        -0.14789746701717377,
        0.23209869861602783,
        0.7217178344726562,
        0.210352823138237,
        0.796341061592102,
        0.6581951975822449
      ],
      "commanded_positions": [
        -0.31166375642854666,
        -0.7045848153216101,
        0.31166375642854666,
        0.7045848153216101,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.7237702796212595,
        0.08753194681056206,
        0.7722777856228857,
        0.5756574421351857
      ]
    }
  },
  "final_plans": {
    "robot_b": {
      "valid": true,
      "status": "tracking",
      "positions": [
        [
          -0.060534480959177156,
          2.9218802452087407
        ],
        [
          -0.06371401794727474,
          2.9305438563126924
        ],
        [
          -0.0659924482979885,
          2.9370388998596786
        ],
        [
          -0.06779893956644341,
          2.942386945718072
        ],
        [
          -0.06925311984680922,
          2.9468356115817302
        ],
        [
          -0.07044107487785962,
          2.950577028930377
        ]
      ],
      "velocities": [
        0.0543892006696521,
        0.03789802332727465,
        0.03093326503992814,
        0.025516091909324422,
        0.021287130038078583,
        0.01796783115805369
      ],
      "yaw_rates": [
        -0.0778986610985985,
        -0.06313777866900085,
        -0.05273117398205442,
        -0.04514693226903924,
        -0.039450459683984536,
        -0.03505571795832121
      ]
    },
    "robot_a": {
      "valid": false,
      "status": "goal_reached",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    }
  }
}
```

</details>

<details><summary>마지막 실행 메시지</summary>

```text
[ros2_control_node-5] [INFO] [1793233466.195438825] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-5] [INFO] [1793233466.195448745] [resource_manager]: Successful 'deactivate' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793233466.195451732] [resource_manager]: 'shutdown' hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793233466.195454067] [resource_manager]: Successful 'shutdown' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793233466.195480717] [robot_b.controller_manager]: Shutting down the controller manager.
[ros2_control_node-2] [INFO] [1793233466.195596254] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793233466.195633419] [robot_a.controller_manager]: Shutdown request received....
[ros2_control_node-2] [INFO] [1793233466.195656643] [robot_a.controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-2] [INFO] [1793233466.195668125] [robot_a.controller_manager]: Deactivating controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793233466.195740456] [robot_a.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793233466.195821778] [resource_manager]: 'deactivate' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793233466.195912859] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-2] [INFO] [1793233466.195919310] [resource_manager]: Successful 'deactivate' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793233466.195921264] [resource_manager]: 'shutdown' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793233466.195923237] [resource_manager]: Successful 'shutdown' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793233466.195941171] [robot_a.controller_manager]: Shutting down the controller manager.
[ERROR] [dmpc_planner-11]: process has died [pid 57, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_iko05dmp --params-file /tmp/launch_params_7ciii0b5'].
[ERROR] [peer_state_relay-10]: process has died [pid 56, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_nzrd8bbm'].
[mpc_rpp_controller-12] [INFO] [1793233466.197840134] [robot_b.mpc_rpp_controller]: Destroying
[ERROR] [peer_state_relay-7]: process has died [pid 53, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_jcql4spz'].
[ERROR] [dmpc_planner-8]: process has died [pid 54, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_lyuc2prd --params-file /tmp/launch_params_ys0936fe'].
[INFO] [mpc_rpp_controller-12]: process has finished cleanly [pid 58]
[mpc_rpp_controller-9] [INFO] [1793233466.205335579] [robot_a.mpc_rpp_controller]: Destroying
[INFO] [mpc_rpp_controller-9]: process has finished cleanly [pid 55]
[ros2_control_node-2] [ERROR] [1793233466.210031169] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793233466.210031165] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [INFO] [1793233466.210624815] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793233466.210633677] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793233466.210760096] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793233466.210762267] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-5] [ERROR] [1793233466.211626212] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [ERROR] [1793233466.211626238] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [INFO] [1793233466.212246795] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793233466.212252734] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 50]
[ros2_control_node-5] [INFO] [1793233466.212313425] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793233466.212315143] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 47]
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 48]
[INFO] [ros2_control_node-5]: process has finished cleanly [pid 51]
```

</details>

<a id="run-20261029_092655_729520__C"></a>

### 시나리오 C — 2026-10-29 09:26:55 KST

- 관측 판정: 미완료
- 종료 사유: 제한시간 종료(미완료)
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 63.160 s
- 시각이 일치하는 odom 쌍: 3159
- 관측된 최소 중심 간격: 2.421 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: 0.003 m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 미도착 | 2.901 | 0.008 | 0.773 | 0.775 | 601 |
| robot_b | 미도착 | 5.517 | 0.122 | 0.338 | 0.347 | 541 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [peer_state_relay-7]: process has died [pid 54, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_w4gny2nv']. / [ERROR] [dmpc_planner-8]: process has died [pid 55, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_bo03uo_r --params-file /tmp/launch_params_a4u0yp2e']. / [ERROR] [peer_state_relay-10]: process has died [pid 57, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_okfzp3po']. / [ERROR] [dmpc_planner-11]: process has died [pid 58, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_to7u3ygh --params-file /tmp/launch_params_ym0gmodc'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "d152cebb683c1e62a08c1822fe5daea1c8203aa668f27bef7dc995a3a643ba0b",
    "dmpc_runtime/ros_node.py": "20a6b1845972531839c75b9a640d380fb7a10904eed9eb10aea7919c0e09a6d3",
    "dmpc_runtime/isaac_test.py": "c42c93a750f4127d92f5f26798c79dc50fcdbe021372670d1b4588e5c4dd4c2d",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "ff7f177d3df754eeedf531c6c48e2e4ed13d72c26d80a47fb618d34b58551ead"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 601
      },
      "controller": {
        "waiting_for_fresh_gt": 4,
        "waiting_for_fresh_mpc_plan": 2,
        "mpc_waiting_for_fresh_states": 8,
        "mpc_path_outside_workspace": 2603,
        "tracking": 286,
        "tracking_speed_reduced": 16
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 541,
        "solver_failed": 58
      },
      "controller": {
        "waiting_for_fresh_gt": 4,
        "waiting_for_fresh_mpc_plan": 2,
        "mpc_waiting_for_fresh_states": 7,
        "mpc_path_outside_workspace": 2551,
        "tracking": 112,
        "mpc_solver_failed": 283
      }
    }
  },
  "final_positions": [
    [
      0.10292232036590576,
      -0.14701630175113678,
      -0.007974312974624559
    ],
    [
      2.517409563064575,
      0.030194416642189026,
      3.0196047477917305
    ]
  ],
  "final_control_diagnostics": {
    "robot_a": {
      "plan_age": 0.07,
      "ego_age": 0.0,
      "peer_age": 0.02,
      "profile_speed": 0.5358785053463914,
      "profile_yaw_rate": 0.005503167283388081,
      "position_error": 0.0002656029206164617,
      "measured_speed": 0.5403478377440822,
      "measured_yaw_rate": 0.008013617238464055,
      "rpp_speed": 0.5358785053463914,
      "rpp_yaw_rate": 0.004042195440265003,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 2.420980644386497,
      "peer_checks": [
        {
          "robot_id": "robot_b",
          "age": 0.02,
          "separation": 2.420980644386497,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.04644273072245939
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "wall_clearance": 0.0029836982488631314,
      "command_speed": 0.40190887900979355,
      "command_yaw_rate": 0.0030316465801987517,
      "predicted_clearance": 1.745062875383805,
      "speed_scale": 0.75,
      "rotation_only": false,
      "candidate_speed": 0.40190887900979355,
      "candidate_yaw_rate": 0.0030316465801987517,
      "predicted_wall_clearance": 0.0008391396276980922,
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "tracking_speed_reduced"
    },
    "robot_b": {
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "mpc_solver_failed"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        0.0034626007545739412,
        0.004146505147218704,
        -0.0043424884788692,
        -0.0036358176730573177,
        5.02517032623291,
        5.042874813079834,
        5.0492167472839355,
        4.973344326019287
      ],
      "measured_velocities": [
        -0.0011874475749209523,
        -0.014798560179769993,
        0.0008787035476416349,
        -0.00615269597619772,
        5.221201419830322,
        5.252902030944824,
        5.221038341522217,
        5.259062767028809
      ],
      "commanded_positions": [
        0.0038447333244961968,
        0.003816384159638364,
        -0.0038447333244961968,
        -0.003816384159638364,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        5.308376196110277,
        5.342870134140952,
        5.294809511362178,
        5.356231038043163
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.07923489809036255,
        -0.09587723761796951,
        0.08231787383556366,
        0.09488219022750854,
        5.026862621307373,
        4.336655616760254,
        4.943329334259033,
        4.405856132507324
      ],
      "measured_velocities": [
        0.02319714054465294,
        -0.00944787822663784,
        0.027708400040864944,
        -0.008180116303265095,
        -0.005848823115229607,
        -0.0003106142394244671,
        -0.0069162193685770035,
        0.001383382361382246
      ],
      "commanded_positions": [
        -0.08063693562934393,
        -0.0954732088701955,
        0.08063693562934393,
        0.0954732088701955,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    }
  },
  "final_plans": {
    "robot_a": {
      "valid": true,
      "status": "tracking",
      "positions": [
        [
          0.06543660979369932,
          -0.14670577729585335
        ],
        [
          0.17260894724147824,
          -0.14755420310927714
        ],
        [
          0.2757116253825506,
          -0.14829245341366126
        ],
        [
          0.3634733668009468,
          -0.14889934619308512
        ],
        [
          0.4312404904663955,
          -0.14936912537455305
        ],
        [
          0.4790095059187004,
          -0.14970600119330088
        ]
      ],
      "velocities": [
        0.5295026548819856,
        0.5422543558107972,
        0.4887988635282855,
        0.3888395345404124,
        0.28884798536739015,
        0.18885404790668345
      ],
      "yaw_rates": [
        0.005503167283388081,
        0.002116016757300037,
        0.0003101582229533118,
        -0.0005176466428779322,
        -0.0007052931346075856,
        -0.0005049897286260374
      ]
    },
    "robot_b": {
      "valid": false,
      "status": "solver_failed",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    }
  }
}
```

</details>

<details><summary>마지막 실행 메시지</summary>

```text
[ros2_control_node-2] [INFO] [1793233740.989620171] [resource_manager]: Successful 'deactivate' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793233740.989621997] [resource_manager]: 'shutdown' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793233740.989623560] [resource_manager]: Successful 'shutdown' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793233740.989642611] [robot_a.controller_manager]: Shutting down the controller manager.
[robot_state_publisher-1] [INFO] [1793233740.989725271] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-5] [INFO] [1793233740.989967822] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-5] [INFO] [1793233740.989997103] [robot_b.controller_manager]: Shutdown request received....
[ros2_control_node-5] [INFO] [1793233740.990018706] [robot_b.controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-5] [INFO] [1793233740.990026716] [robot_b.controller_manager]: Deactivating controller 'antbot_swerve_controller'
[ros2_control_node-5] [INFO] [1793233740.990103954] [robot_b.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-5] [INFO] [1793233740.990161468] [resource_manager]: 'deactivate' hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793233740.990252054] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-5] [INFO] [1793233740.990255074] [resource_manager]: Successful 'deactivate' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793233740.990256449] [resource_manager]: 'shutdown' hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793233740.990257918] [resource_manager]: Successful 'shutdown' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793233740.990273541] [robot_b.controller_manager]: Shutting down the controller manager.
[mpc_rpp_controller-12] [INFO] [1793233740.991067446] [robot_b.mpc_rpp_controller]: Destroying
[ERROR] [peer_state_relay-7]: process has died [pid 54, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_w4gny2nv'].
[ERROR] [dmpc_planner-8]: process has died [pid 55, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_bo03uo_r --params-file /tmp/launch_params_a4u0yp2e'].
[ERROR] [peer_state_relay-10]: process has died [pid 57, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_okfzp3po'].
[INFO] [mpc_rpp_controller-12]: process has finished cleanly [pid 59]
[ERROR] [dmpc_planner-11]: process has died [pid 58, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_to7u3ygh --params-file /tmp/launch_params_ym0gmodc'].
[ros2_control_node-5] [ERROR] [1793233740.998775171] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [ERROR] [1793233740.998778323] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [INFO] [1793233740.999226385] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793233740.999232663] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-5] [INFO] [1793233740.999283068] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793233740.999284228] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[mpc_rpp_controller-9] [INFO] [1793233741.005750899] [robot_a.mpc_rpp_controller]: Destroying
[INFO] [ros2_control_node-5]: process has finished cleanly [pid 52]
[ros2_control_node-2] [ERROR] [1793233741.006839328] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793233741.006940496] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 48]
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 51]
[ros2_control_node-2] [INFO] [1793233741.008183852] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793233741.008191450] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793233741.008249993] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793233741.008251721] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [mpc_rpp_controller-9]: process has finished cleanly [pid 56]
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 49]
```

</details>

<a id="run-20261029_093005_184702__C"></a>

### 시나리오 C — 2026-10-29 09:30:05 KST

- 관측 판정: 미완료
- 종료 사유: 제한시간 종료(미완료)
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 62.360 s
- 시각이 일치하는 odom 쌍: 3119
- 관측된 최소 중심 간격: 5.872 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: 0.150 m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 미도착 | 5.936 | 0.000 | 0.001 | 0.000 | 587 |
| robot_b | 미도착 | 5.936 | 0.000 | 0.001 | 0.000 | 587 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [peer_state_relay-10]: process has died [pid 58, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_2vqvv4td']. / [ERROR] [dmpc_planner-8]: process has died [pid 54, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_64s4m5i8 --params-file /tmp/launch_params_i8xqesjj']. / [ERROR] [peer_state_relay-7]: process has died [pid 53, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_lf0737z5']. / [ERROR] [dmpc_planner-11]: process has died [pid 59, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_ucmshii2 --params-file /tmp/launch_params_ztlmzx7o'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "d152cebb683c1e62a08c1822fe5daea1c8203aa668f27bef7dc995a3a643ba0b",
    "dmpc_runtime/ros_node.py": "20a6b1845972531839c75b9a640d380fb7a10904eed9eb10aea7919c0e09a6d3",
    "dmpc_runtime/isaac_test.py": "c42c93a750f4127d92f5f26798c79dc50fcdbe021372670d1b4588e5c4dd4c2d",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "ff7f177d3df754eeedf531c6c48e2e4ed13d72c26d80a47fb618d34b58551ead"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 587
      },
      "controller": {
        "waiting_for_fresh_gt": 5,
        "waiting_for_fresh_mpc_plan": 2,
        "mpc_waiting_for_fresh_states": 6,
        "mpc_path_outside_workspace": 2969
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 587
      },
      "controller": {
        "waiting_for_fresh_gt": 5,
        "waiting_for_fresh_mpc_plan": 1,
        "mpc_waiting_for_fresh_states": 6,
        "mpc_path_outside_workspace": 2957
      }
    }
  },
  "final_positions": [
    [
      -2.9359970092773438,
      2.1785319859191077e-07,
      9.702204221857823e-06
    ],
    [
      2.936067819595337,
      -5.700616156900651e-07,
      -3.1415837016210384
    ]
  ],
  "final_control_diagnostics": {
    "robot_b": {
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "mpc_path_outside_workspace"
    },
    "robot_a": {
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "mpc_path_outside_workspace"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        4.358217483968474e-05,
        3.723802728927694e-05,
        -3.872920569847338e-05,
        -6.054793993826024e-05,
        0.6210325360298157,
        0.6211351752281189,
        0.6210989356040955,
        0.6210817098617554
      ],
      "measured_velocities": [
        0.0022673700004816055,
        -0.006253606639802456,
        0.00517632532864809,
        -0.0013922977959737182,
        0.006821244489401579,
        0.003798204008489847,
        0.004724038764834404,
        0.0072971065528690815
      ],
      "commanded_positions": [
        1.7743577700457536e-05,
        1.5720994269941002e-05,
        -1.9289596821181476e-05,
        -2.571936056483537e-05,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        4.901214924757369e-05,
        3.21577608701773e-05,
        -2.7568057703319937e-05,
        -5.323361619957723e-05,
        0.6203356385231018,
        0.6204301118850708,
        0.6203851699829102,
        0.6203690767288208
      ],
      "measured_velocities": [
        0.003743136301636696,
        -0.007809843868017197,
        0.006648462265729904,
        -0.003105724696069956,
        0.005737348925322294,
        0.002667604014277458,
        0.0036235027946531773,
        0.006020647939294577
      ],
      "commanded_positions": [
        2.6338628231314942e-05,
        1.530966801510658e-05,
        -6.99868360243272e-06,
        -2.859173036995344e-05,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    }
  },
  "final_plans": {
    "robot_b": {
      "valid": true,
      "status": "tracking",
      "positions": [
        [
          2.9360929184385918,
          -5.527410528128188e-07
        ],
        [
          2.925927577634413,
          6.682711060026526e-05
        ],
        [
          2.895767691124908,
          0.0006369952438236121
        ],
        [
          2.845631504338008,
          0.002340710203133357
        ],
        [
          2.775541850595291,
          0.005596941198031368
        ],
        [
          2.68550978916433,
          0.010499511013929912
        ]
      ],
      "velocities": [
        0.0008279757819713138,
        0.10082795204540074,
        0.2008279245091148,
        0.30082789197701026,
        0.4008278526512352,
        0.5008278036507844
      ],
      "yaw_rates": [
        -0.04997999223355766,
        -0.08028055735475026,
        -0.07431338665678727,
        -0.05268205437602618,
        -0.028518114511010058,
        -0.008533070030061015
      ]
    },
    "robot_a": {
      "valid": true,
      "status": "tracking",
      "positions": [
        [
          -2.9360233011197905,
          2.0617931920673446e-07
        ],
        [
          -2.9258579783738634,
          -6.716377410287733e-05
        ],
        [
          -2.895698109373744,
          -0.0006372990425237022
        ],
        [
          -2.845561939103009,
          -0.002340965945483337
        ],
        [
          -2.7754723011116083,
          -0.005597145106299118
        ],
        [
          -2.6854402552220464,
          -0.010499666823034375
        ]
      ],
      "velocities": [
        0.0008278851523422428,
        0.10082786141579954,
        0.2008278338795464,
        0.3008278013474813,
        0.4008277620217548,
        0.5008277130213656
      ],
      "yaw_rates": [
        -0.04997793069147884,
        -0.0802807782168407,
        -0.07431444442567048,
        -0.05268320058150923,
        -0.02851903164320061,
        -0.008533684887531784
      ]
    }
  }
}
```

</details>

<details><summary>마지막 실행 메시지</summary>

```text
[ros2_control_node-5] [INFO] [1793233930.383728315] [resource_manager]: 'deactivate' hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793233930.383843977] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-5] [INFO] [1793233930.383848622] [resource_manager]: Successful 'deactivate' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793233930.383850359] [resource_manager]: 'shutdown' hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793233930.383851974] [resource_manager]: Successful 'shutdown' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793233930.383870870] [robot_b.controller_manager]: Shutting down the controller manager.
[robot_state_publisher-4] [INFO] [1793233930.384029129] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ERROR] [peer_state_relay-7]: process has died [pid 53, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_lf0737z5'].
[ros2_control_node-2] [INFO] [1793233930.384487616] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793233930.384518276] [robot_a.controller_manager]: Shutdown request received....
[ros2_control_node-2] [INFO] [1793233930.384541222] [robot_a.controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-2] [INFO] [1793233930.384551499] [robot_a.controller_manager]: Deactivating controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793233930.384627971] [robot_a.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793233930.384701380] [resource_manager]: 'deactivate' hardware 'robot_a_isaac_system'
[robot_state_publisher-1] [INFO] [1793233930.384908711] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793233930.385283176] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-2] [INFO] [1793233930.385290072] [resource_manager]: Successful 'deactivate' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793233930.385291919] [resource_manager]: 'shutdown' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793233930.385293507] [resource_manager]: Successful 'shutdown' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793233930.385310538] [robot_a.controller_manager]: Shutting down the controller manager.
[INFO] [mpc_rpp_controller-12]: process has finished cleanly [pid 60]
[ERROR] [dmpc_planner-11]: process has died [pid 59, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_ucmshii2 --params-file /tmp/launch_params_ztlmzx7o'].
[mpc_rpp_controller-9] [INFO] [1793233930.392922266] [robot_a.mpc_rpp_controller]: Destroying
[ros2_control_node-5] [ERROR] [1793233930.396444811] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [ERROR] [1793233930.396604986] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [INFO] [1793233930.396741871] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793233930.396746646] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [mpc_rpp_controller-9]: process has finished cleanly [pid 55]
[ros2_control_node-5] [INFO] [1793233930.396822307] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793233930.396824238] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [ERROR] [1793233930.400350589] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793233930.400413317] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [INFO] [1793233930.400540826] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793233930.400548144] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793233930.400615594] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793233930.400617604] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 47]
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 50]
[INFO] [ros2_control_node-5]: process has finished cleanly [pid 51]
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 48]
```

</details>

<a id="run-20261029_094009_619716__C"></a>

### 시나리오 C — 2026-10-29 09:40:09 KST

- 관측 판정: 미완료
- 종료 사유: 제한시간 종료(미완료)
- 설정 SHA-256: `dad472eccd082ae359630f69cf5e54fa27c4e095b5cc4442f541f6ec0fd0f991`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 65.860 s
- 시각이 일치하는 odom 쌍: 3294
- 관측된 최소 중심 간격: 5.688 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: 0.144 m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 미도착 | 5.782 | 0.064 | 0.215 | 0.228 | 621 |
| robot_b | 미도착 | 5.906 | 0.020 | 0.079 | 0.094 | 619 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [peer_state_relay-10]: process has died [pid 57, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_5u3u43y_']. / [ERROR] [dmpc_planner-8]: process has died [pid 55, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params__4dexeb4 --params-file /tmp/launch_params_bltlgmyn']. / [ERROR] [peer_state_relay-7]: process has died [pid 54, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_zc5pb7o3']. / [ERROR] [dmpc_planner-11]: process has died [pid 58, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_u7rxklvn --params-file /tmp/launch_params_6ch2e_iu'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "d152cebb683c1e62a08c1822fe5daea1c8203aa668f27bef7dc995a3a643ba0b",
    "dmpc_runtime/ros_node.py": "20a6b1845972531839c75b9a640d380fb7a10904eed9eb10aea7919c0e09a6d3",
    "dmpc_runtime/isaac_test.py": "c42c93a750f4127d92f5f26798c79dc50fcdbe021372670d1b4588e5c4dd4c2d",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "ff7f177d3df754eeedf531c6c48e2e4ed13d72c26d80a47fb618d34b58551ead"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "tracking": 621
      },
      "controller": {
        "waiting_for_fresh_gt": 4,
        "invalid_peer_state": 1,
        "waiting_for_fresh_mpc_plan": 6,
        "mpc_path_outside_workspace": 2833,
        "tracking_mpc_turn": 2,
        "tracking": 63
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 619,
        "solver_failed": 2
      },
      "controller": {
        "waiting_for_fresh_gt": 5,
        "waiting_for_fresh_mpc_plan": 2,
        "mpc_waiting_for_fresh_states": 5,
        "mpc_path_outside_workspace": 2943,
        "tracking": 39,
        "mpc_solver_failed": 9
      }
    }
  },
  "final_positions": [
    [
      -2.782135248184204,
      -0.005887518636882305,
      -0.06350859509163923
    ],
    [
      2.905900001525879,
      0.00020775805751327425,
      3.1218635552206906
    ]
  ],
  "final_control_diagnostics": {
    "robot_b": {
      "plan_age": 0.05,
      "ego_age": 0.0,
      "peer_age": 0.02,
      "profile_speed": 0.10935300595178783,
      "profile_yaw_rate": -0.08212185276046051,
      "position_error": 0.0002772541029762479,
      "measured_speed": 0.07689247767674223,
      "measured_yaw_rate": -0.056324016378767224,
      "rpp_speed": 0.10935300595178783,
      "rpp_yaw_rate": -0.0730919647932756,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 5.693903974760253,
      "peer_checks": [
        {
          "robot_id": "robot_a",
          "age": 0.02,
          "separation": 5.693903974760253,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.04644273072245939
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "wall_clearance": 0.14982251215260467,
      "command_speed": 0.09189247767674223,
      "command_yaw_rate": -0.06142128133246341,
      "predicted_clearance": 5.29661452856974,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.09189247767674223,
      "candidate_yaw_rate": -0.06142128133246341,
      "predicted_wall_clearance": 0.14890347162970946,
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "tracking"
    },
    "robot_a": {
      "plan_age": 0.13,
      "ego_age": 0.0,
      "peer_age": 0.02,
      "profile_speed": 0.26949674164068455,
      "profile_yaw_rate": -0.04354875496097321,
      "position_error": 0.0024344421494437974,
      "measured_speed": 0.2125964972616766,
      "measured_yaw_rate": -0.03649398702933134,
      "rpp_speed": 0.26949674164068455,
      "rpp_yaw_rate": -0.0339317853077571,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 5.693868910532845,
      "peer_checks": [
        {
          "robot_id": "robot_b",
          "age": 0.02,
          "separation": 5.693868910532845,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.04644273072245939
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "wall_clearance": 0.1443828908726572,
      "command_speed": 0.2275964972616766,
      "command_yaw_rate": -0.028656210961457027,
      "predicted_clearance": 5.23154395332908,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.2275964972616766,
      "candidate_yaw_rate": -0.028656210961457027,
      "predicted_wall_clearance": 0.13687720756985794,
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "tracking"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.04170232266187668,
        -0.045548733323812485,
        0.041427139192819595,
        0.045491430908441544,
        2.3180699348449707,
        1.953099012374878,
        2.287551164627075,
        1.9799412488937378
      ],
      "measured_velocities": [
        -0.001034855842590332,
        -0.01097874715924263,
        -0.004312492907047272,
        0.005129575729370117,
        2.1759960651397705,
        1.9936771392822266,
        2.1868057250976562,
        1.9960954189300537
      ],
      "commanded_positions": [
        -0.039837697026809206,
        -0.043295600865159004,
        0.039837697026809206,
        0.043295600865159004,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        2.2129942814360253,
        2.09259269039987,
        2.296834532087542,
        2.0013204729915217
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.15510132908821106,
        -0.22044791281223297,
        0.1542351245880127,
        0.22248798608779907,
        1.0672754049301147,
        0.7551993131637573,
        0.8848342895507812,
        0.9936776757240295
      ],
      "measured_velocities": [
        0.007732240483164787,
        0.1031990572810173,
        -0.11625111848115921,
        0.028625842183828354,
        0.9663370847702026,
        0.6623954772949219,
        1.022476315498352,
        0.6051079034805298
      ],
      "commanded_positions": [
        -0.1542274514579809,
        -0.21869449725802986,
        0.1542274514579809,
        0.21869449725802986,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        1.047364702045575,
        0.7604052975419348,
        1.0945553993901869,
        0.6510221881644259
      ]
    }
  },
  "final_plans": {
    "robot_b": {
      "valid": true,
      "status": "tracking",
      "positions": [
        [
          2.911280983590456,
          0.0001278079296905104
        ],
        [
          2.887418202411786,
          0.000728782256409149
        ],
        [
          2.843584337962969,
          0.0025126783170954336
        ],
        [
          2.7798078370651966,
          0.005971123197954362
        ],
        [
          2.6961068449827224,
          0.011298859556412027
        ],
        [
          2.5924800557850696,
          0.01841018342033647
        ]
      ],
      "velocities": [
        0.06935302510147194,
        0.1693529772272617,
        0.2693529209793619,
        0.3693528534569223,
        0.469352770218717,
        0.5693526640029538
      ],
      "yaw_rates": [
        -0.08212185276046051,
        -0.07834359476023957,
        -0.05950553017864835,
        -0.036083325568542955,
        -0.014395800393015181,
        0.002560414251949558
      ]
    },
    "robot_a": {
      "valid": true,
      "status": "tracking",
      "positions": [
        [
          -2.7927177249939352,
          -0.0052099058214170055
        ],
        [
          -2.7415577148197476,
          -0.008587421779824621
        ],
        [
          -2.6704719193129494,
          -0.013727946708146877
        ],
        [
          -2.5794600000972743,
          -0.020606252941981953
        ],
        [
          -2.468506216886323,
          -0.0290082197364252
        ],
        [
          -2.3375862880767957,
          -0.038607298223553285
        ]
      ],
      "velocities": [
        0.20635756762531204,
        0.30635750547278273,
        0.40635743038585614,
        0.5063573366874448,
        0.6063572143969846,
        0.7063570434659299
      ],
      "yaw_rates": [
        -0.03907096184170546,
        -0.02495893240731678,
        -0.008341433033401652,
        0.006364223704589856,
        0.017301281949860314,
        0.024250235159236286
      ]
    }
  }
}
```

</details>

<details><summary>마지막 실행 메시지</summary>

```text
[ros2_control_node-5] [INFO] [1793234534.894987061] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-5] [INFO] [1793234534.894991673] [resource_manager]: Successful 'deactivate' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793234534.894993669] [resource_manager]: 'shutdown' hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793234534.894995258] [resource_manager]: Successful 'shutdown' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793234534.895016463] [robot_b.controller_manager]: Shutting down the controller manager.
[mpc_rpp_controller-12] [INFO] [1793234534.895145502] [robot_b.mpc_rpp_controller]: Destroying
[ros2_control_node-2] [INFO] [1793234534.895302782] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793234534.895331563] [robot_a.controller_manager]: Shutdown request received....
[ros2_control_node-2] [INFO] [1793234534.895351312] [robot_a.controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-2] [INFO] [1793234534.895361350] [robot_a.controller_manager]: Deactivating controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793234534.895433078] [robot_a.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793234534.895507958] [resource_manager]: 'deactivate' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793234534.895623013] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-2] [INFO] [1793234534.895627882] [resource_manager]: Successful 'deactivate' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793234534.895629525] [resource_manager]: 'shutdown' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793234534.895630955] [resource_manager]: Successful 'shutdown' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793234534.895646113] [robot_a.controller_manager]: Shutting down the controller manager.
[robot_state_publisher-1] [INFO] [1793234534.896397717] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ERROR] [dmpc_planner-8]: process has died [pid 55, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params__4dexeb4 --params-file /tmp/launch_params_bltlgmyn'].
[INFO] [mpc_rpp_controller-12]: process has finished cleanly [pid 59]
[ERROR] [peer_state_relay-7]: process has died [pid 54, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_zc5pb7o3'].
[ERROR] [dmpc_planner-11]: process has died [pid 58, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_u7rxklvn --params-file /tmp/launch_params_6ch2e_iu'].
[mpc_rpp_controller-9] [INFO] [1793234534.912396368] [robot_a.mpc_rpp_controller]: Destroying
[ros2_control_node-2] [ERROR] [1793234534.912866564] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793234534.912866515] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [INFO] [1793234534.913067806] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793234534.913074624] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793234534.913136498] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793234534.913138125] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-5] [ERROR] [1793234534.913392710] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [ERROR] [1793234534.913392834] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [INFO] [1793234534.913940770] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793234534.913948000] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-5] [INFO] [1793234534.914250655] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793234534.914254495] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [mpc_rpp_controller-9]: process has finished cleanly [pid 56]
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 48]
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 51]
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 49]
[INFO] [ros2_control_node-5]: process has finished cleanly [pid 52]
```

</details>

<a id="run-20261029_094429_424693__C"></a>

### 시나리오 C — 2026-10-29 09:44:29 KST

- 관측 판정: 미완료
- 종료 사유: 제한시간 종료(미완료)
- 설정 SHA-256: `0f19654e6ba92e42ee15d28379d8eb58aa8d1801cda35372cdffb30e813e65db`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 30.020 s
- 시각이 일치하는 odom 쌍: 1502
- 관측된 최소 중심 간격: 3.069 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: 0.001 m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 미도착 | 5.186 | 0.153 | 0.433 | 0.438 | 49 |
| robot_b | 미도착 | 3.878 | 0.005 | 0.631 | 0.637 | 64 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: 없음

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "wall_planning_margin": 0.001,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "b4636e93b7f016e9d5f86d3529270f2bffaecc71a660fdb51c61b0e1218231e3",
    "dmpc_runtime/ros_node.py": "75fa9e23b99cc0b6ed3557fdc59ba9adc1d48988ec1ca6e4c48c143a74feddcd",
    "dmpc_runtime/isaac_test.py": "c42c93a750f4127d92f5f26798c79dc50fcdbe021372670d1b4588e5c4dd4c2d",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "ff7f177d3df754eeedf531c6c48e2e4ed13d72c26d80a47fb618d34b58551ead"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 49,
        "solver_failed": 234
      },
      "controller": {
        "waiting_for_fresh_gt": 3,
        "waiting_for_fresh_mpc_plan": 2,
        "mpc_waiting_for_fresh_states": 7,
        "tracking": 199,
        "waiting_for_clock": 4,
        "expired_mpc_plan": 2,
        "tracking_mpc_turn": 1,
        "mpc_solver_failed": 1200
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 64,
        "solver_failed": 219
      },
      "controller": {
        "waiting_for_fresh_gt": 4,
        "invalid_peer_state": 1,
        "waiting_for_fresh_mpc_plan": 2,
        "mpc_waiting_for_fresh_states": 7,
        "tracking": 267,
        "mpc_solver_failed": 1097,
        "waiting_for_clock": 4,
        "tracking_mpc_turn": 1
      }
    }
  },
  "final_positions": [
    [
      -2.184929132461548,
      -0.08305229246616364,
      -0.15307809179643267
    ],
    [
      0.8748116493225098,
      0.14882749319076538,
      -3.1363439861921396
    ]
  ],
  "final_control_diagnostics": {
    "robot_b": {
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "mpc_solver_failed"
    },
    "robot_a": {
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "mpc_solver_failed"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.07325296103954315,
        -0.08350714296102524,
        0.07699445635080338,
        0.08742000162601471,
        -4.230618476867676,
        -5.038278579711914,
        -4.282673358917236,
        -5.0104804039001465
      ],
      "measured_velocities": [
        0.035941943526268005,
        0.027879826724529266,
        0.026371199637651443,
        0.023251831531524658,
        -0.01930341124534607,
        0.026648566126823425,
        -0.012931564822793007,
        0.013748742640018463
      ],
      "commanded_positions": [
        -0.07548781239769925,
        -0.0855749082561816,
        0.07548781239769925,
        0.0855749082561816,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        0.013736730441451073,
        0.013325237669050694,
        -0.014319994486868382,
        -0.014110086485743523,
        -4.626721382141113,
        -4.5641889572143555,
        -4.608558654785156,
        -4.615882873535156
      ],
      "measured_velocities": [
        -0.00251627154648304,
        -0.014789571985602379,
        0.0050324201583862305,
        -0.004934646654874086,
        0.008714873343706131,
        -0.000280547421425581,
        0.004352733492851257,
        0.005508932285010815
      ],
      "commanded_positions": [
        0.014238650913632966,
        0.013857455336337754,
        -0.014238650913632966,
        -0.013857455336337754,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    }
  },
  "final_plans": {
    "robot_b": {
      "valid": false,
      "status": "solver_failed",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    },
    "robot_a": {
      "valid": false,
      "status": "solver_failed",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    }
  }
}
```

</details>

<details><summary>마지막 실행 메시지</summary>

```text
[ros2_control_node-5] [INFO] [1793234729.596102057] [resource_manager]: 'shutdown' hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793234729.596103815] [resource_manager]: Successful 'shutdown' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793234729.596131225] [robot_b.controller_manager]: Shutting down the controller manager.
[robot_state_publisher-4] [INFO] [1793234729.596060591] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793234729.596491484] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793234729.596523455] [robot_a.controller_manager]: Shutdown request received....
[ros2_control_node-2] [INFO] [1793234729.596543175] [robot_a.controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-2] [INFO] [1793234729.596553105] [robot_a.controller_manager]: Deactivating controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793234729.596641586] [robot_a.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793234729.596718144] [resource_manager]: 'deactivate' hardware 'robot_a_isaac_system'
[mpc_rpp_controller-12] [INFO] [1793234729.596854139] [robot_b.mpc_rpp_controller]: Destroying
[ros2_control_node-2] [INFO] [1793234729.597873864] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-2] [INFO] [1793234729.597883938] [resource_manager]: Successful 'deactivate' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793234729.597886630] [resource_manager]: 'shutdown' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793234729.597888446] [resource_manager]: Successful 'shutdown' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793234729.597908323] [robot_a.controller_manager]: Shutting down the controller manager.
[robot_state_publisher-1] [INFO] [1793234729.597954427] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[INFO] [mpc_rpp_controller-12]: process has finished cleanly [pid 61]
[ros2_control_node-5] [ERROR] [1793234729.603891172] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [ERROR] [1793234729.603891183] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [INFO] [1793234729.604230401] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793234729.604236964] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-5] [INFO] [1793234729.604326553] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793234729.604328479] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [ros2_control_node-5]: process has finished cleanly [pid 52]
[mpc_rpp_controller-9] [INFO] [1793234729.614459325] [robot_a.mpc_rpp_controller]: Destroying
[ros2_control_node-2] [ERROR] [1793234729.615252285] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793234729.615304422] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [INFO] [1793234729.615468086] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793234729.615474967] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793234729.615543634] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793234729.615545467] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [mpc_rpp_controller-9]: process has finished cleanly [pid 56]
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 48]
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 51]
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 49]
[INFO] [peer_state_relay-10]: process has finished cleanly [pid 57]
[INFO] [dmpc_planner-8]: process has finished cleanly [pid 55]
[INFO] [dmpc_planner-11]: process has finished cleanly [pid 60]
[INFO] [peer_state_relay-7]: process has finished cleanly [pid 54]
```

</details>

<a id="run-20261029_094950_281855__C"></a>

### 시나리오 C — 2026-10-29 09:49:50 KST

- 관측 판정: 미완료
- 종료 사유: 제한시간 종료(미완료)
- 설정 SHA-256: `0f19654e6ba92e42ee15d28379d8eb58aa8d1801cda35372cdffb30e813e65db`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 31.740 s
- 시각이 일치하는 odom 쌍: 1588
- 관측된 최소 중심 간격: 1.674 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: 0.027 m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 미도착 | 2.453 | 0.559 | 0.748 | 0.753 | 90 |
| robot_b | 미도착 | 5.220 | 0.188 | 0.434 | 0.436 | 40 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: 없음

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "wall_planning_margin": 0.001,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "871877ed5aa01e262a132e3c187e19afb5ef0d488e58be3ea24c337002c6a04d",
    "dmpc_runtime/ros_node.py": "75fa9e23b99cc0b6ed3557fdc59ba9adc1d48988ec1ca6e4c48c143a74feddcd",
    "dmpc_runtime/isaac_test.py": "c42c93a750f4127d92f5f26798c79dc50fcdbe021372670d1b4588e5c4dd4c2d",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "ff7f177d3df754eeedf531c6c48e2e4ed13d72c26d80a47fb618d34b58551ead"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 1,
        "tracking": 90,
        "solver_failed": 210
      },
      "controller": {
        "waiting_for_fresh_gt": 2,
        "mpc_waiting_for_fresh_states": 5,
        "tracking": 335,
        "waiting_for_clock": 3,
        "tracking_speed_reduced": 16,
        "mpc_solver_failed": 1039
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 40,
        "solver_failed": 260
      },
      "controller": {
        "waiting_for_fresh_gt": 1,
        "waiting_for_fresh_mpc_plan": 1,
        "mpc_waiting_for_fresh_states": 7,
        "tracking": 156,
        "mpc_solver_failed": 1282,
        "waiting_for_clock": 2
      }
    }
  },
  "final_positions": [
    [
      0.5473557114601135,
      0.010595482774078846,
      -0.5586135718549925
    ],
    [
      2.2192375659942627,
      0.08921109139919281,
      2.953391977949955
    ]
  ],
  "final_control_diagnostics": {
    "robot_a": {
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "mpc_solver_failed"
    },
    "robot_b": {
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "mpc_solver_failed"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.5078420042991638,
        -0.938982367515564,
        0.5088160037994385,
        0.9415406584739685,
        -1.050299882888794,
        -4.413314342498779,
        -1.6559895277023315,
        -3.33992600440979
      ],
      "measured_velocities": [
        0.010042584501206875,
        -0.002667076187208295,
        0.012390309944748878,
        0.0360509492456913,
        -0.008725469931960106,
        -0.0026977481320500374,
        -0.0070405034348368645,
        0.02754417061805725
      ],
      "commanded_positions": [
        -0.5084545855583594,
        -0.938999,
        0.5084545855583594,
        0.938999,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.08007188141345978,
        -0.09198306500911713,
        0.08398918807506561,
        0.09622772783041,
        -4.43794584274292,
        -5.444190502166748,
        -4.506833553314209,
        -5.3986945152282715
      ],
      "measured_velocities": [
        0.039032865315675735,
        0.035466842353343964,
        0.02760263904929161,
        0.022031623870134354,
        -0.025013510137796402,
        0.03104409947991371,
        -0.016476400196552277,
        0.013462521135807037
      ],
      "commanded_positions": [
        -0.08244879387539754,
        -0.09450132623228813,
        0.08244879387539754,
        0.09450132623228813,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    }
  },
  "final_plans": {
    "robot_a": {
      "valid": false,
      "status": "solver_failed",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    },
    "robot_b": {
      "valid": false,
      "status": "solver_failed",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    }
  }
}
```

</details>

<details><summary>마지막 실행 메시지</summary>

```text
[ros2_control_node-5] [INFO] [1793235050.497781533] [robot_b.controller_manager]: Shutting down the controller manager.
[robot_state_publisher-4] [INFO] [1793235050.497758559] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793235050.498084243] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793235050.498115566] [robot_a.controller_manager]: Shutdown request received....
[ros2_control_node-2] [INFO] [1793235050.498135869] [robot_a.controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-2] [INFO] [1793235050.498145714] [robot_a.controller_manager]: Deactivating controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793235050.498218870] [robot_a.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793235050.498289220] [resource_manager]: 'deactivate' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793235050.498443170] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-2] [INFO] [1793235050.498447745] [resource_manager]: Successful 'deactivate' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793235050.498449485] [resource_manager]: 'shutdown' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793235050.498451755] [resource_manager]: Successful 'shutdown' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793235050.498468894] [robot_a.controller_manager]: Shutting down the controller manager.
[robot_state_publisher-1] [INFO] [1793235050.498436489] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[mpc_rpp_controller-12] [INFO] [1793235050.500221254] [robot_b.mpc_rpp_controller]: Destroying
[dmpc_planner-11] [INFO] [1793235050.502583093] [robot_b.dmpc_planner]: robot_b: shutdown
[INFO] [mpc_rpp_controller-12]: process has finished cleanly [pid 58]
[ros2_control_node-5] [ERROR] [1793235050.505771670] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [ERROR] [1793235050.505771685] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [INFO] [1793235050.505962185] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793235050.505970892] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-5] [INFO] [1793235050.506032655] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793235050.506034322] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [ros2_control_node-5]: process has finished cleanly [pid 51]
[dmpc_planner-8] [INFO] [1793235050.512424491] [robot_a.dmpc_planner]: robot_a: shutdown
[ros2_control_node-2] [ERROR] [1793235050.514016053] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793235050.514245799] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [INFO] [1793235050.514322692] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793235050.514329167] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793235050.514394319] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793235050.514396190] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[mpc_rpp_controller-9] [INFO] [1793235050.514545220] [robot_a.mpc_rpp_controller]: Destroying
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 47]
[INFO] [mpc_rpp_controller-9]: process has finished cleanly [pid 55]
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 48]
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 50]
[INFO] [dmpc_planner-11]: process has finished cleanly [pid 57]
[INFO] [peer_state_relay-7]: process has finished cleanly [pid 53]
[INFO] [peer_state_relay-10]: process has finished cleanly [pid 56]
[INFO] [dmpc_planner-8]: process has finished cleanly [pid 54]
```

</details>

<a id="run-20261029_101557_525261__C"></a>

### 시나리오 C — 2026-10-29 10:15:57 KST

- 관측 판정: 미완료
- 종료 사유: 제한시간 종료(미완료)
- 설정 SHA-256: `0f19654e6ba92e42ee15d28379d8eb58aa8d1801cda35372cdffb30e813e65db`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 63.100 s
- 시각이 일치하는 odom 쌍: 3155
- 관측된 최소 중심 간격: 1.675 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: 0.002 m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 미도착 | 5.099 | 0.179 | 0.495 | 0.505 | 67 |
| robot_b | 미도착 | 2.577 | 0.848 | 0.806 | 0.811 | 126 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [peer_state_relay-10]: process has died [pid 64, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_rcxb3o_n']. / [ERROR] [peer_state_relay-7]: process has died [pid 55, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_qvm1wtc9']. / [ERROR] [dmpc_planner-8]: process has died [pid 57, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_fo7pm2s1 --params-file /tmp/launch_params_cmfeido1']. / [ERROR] [dmpc_planner-11]: process has died [pid 65, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_0k6lofy9 --params-file /tmp/launch_params_8jqdzk00'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "wall_planning_margin": 0.001,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "871877ed5aa01e262a132e3c187e19afb5ef0d488e58be3ea24c337002c6a04d",
    "dmpc_runtime/ros_node.py": "75fa9e23b99cc0b6ed3557fdc59ba9adc1d48988ec1ca6e4c48c143a74feddcd",
    "dmpc_runtime/isaac_test.py": "c42c93a750f4127d92f5f26798c79dc50fcdbe021372670d1b4588e5c4dd4c2d",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "ff7f177d3df754eeedf531c6c48e2e4ed13d72c26d80a47fb618d34b58551ead"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 67,
        "solver_failed": 527
      },
      "controller": {
        "mpc_waiting_for_fresh_states": 5,
        "tracking": 259,
        "waiting_for_clock": 2,
        "mpc_solver_failed": 2557,
        "waiting_for_mpc_heading": 2
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 126,
        "solver_failed": 468
      },
      "controller": {
        "waiting_for_fresh_gt": 1,
        "mpc_waiting_for_fresh_states": 5,
        "tracking": 492,
        "tracking_mpc_turn": 2,
        "mpc_solver_failed": 2338,
        "waiting_for_clock": 4
      }
    }
  },
  "final_positions": [
    [
      -2.0988142490386963,
      -0.07421132177114487,
      -0.17866761921335012
    ],
    [
      -0.42401811480522156,
      -0.0579463467001915,
      -2.2933381720106385
    ]
  ],
  "final_control_diagnostics": {
    "robot_b": {
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "mpc_solver_failed"
    },
    "robot_a": {
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "mpc_solver_failed"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.35309386253356934,
        -0.693678081035614,
        0.35671892762184143,
        0.6933227181434631,
        -3.1383960247039795,
        -4.578104019165039,
        -3.4958431720733643,
        -3.8684747219085693
      ],
      "measured_velocities": [
        0.04140563681721687,
        0.008098220452666283,
        0.024627480655908585,
        -0.01698831468820572,
        -0.03505302965641022,
        0.00680460873991251,
        -0.018627554178237915,
        -0.01678645797073841
      ],
      "commanded_positions": [
        -0.35552189340102736,
        -0.6941958011860956,
        0.35552189340102736,
        0.6941958011860956,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        0.9382050633430481,
        0.5092320442199707,
        -0.9403224587440491,
        -0.5104233026504517,
        -6.041528224945068,
        -1.34666907787323,
        -5.1450371742248535,
        -1.9586443901062012
      ],
      "measured_velocities": [
        -0.007298216223716736,
        -0.016239577904343605,
        -0.015943314880132675,
        -0.006180593278259039,
        0.0046213651075959206,
        -0.015919135883450508,
        0.009617896750569344,
        -0.005644399207085371
      ],
      "commanded_positions": [
        0.938999,
        0.5100244597812069,
        -0.938999,
        -0.5100244597812069,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    }
  },
  "final_plans": {
    "robot_a": {
      "valid": false,
      "status": "solver_failed",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    },
    "robot_b": {
      "valid": false,
      "status": "solver_failed",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    }
  }
}
```

</details>

<details><summary>마지막 실행 메시지</summary>

```text
[ros2_control_node-2] [INFO] [1793236682.796161332] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793236682.796193317] [robot_a.controller_manager]: Shutdown request received....
[ros2_control_node-2] [INFO] [1793236682.796217295] [robot_a.controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-2] [INFO] [1793236682.796227342] [robot_a.controller_manager]: Deactivating controller 'antbot_swerve_controller'
[robot_state_publisher-1] [INFO] [1793236682.796269411] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793236682.796311506] [robot_a.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793236682.796391071] [resource_manager]: 'deactivate' hardware 'robot_a_isaac_system'
[mpc_rpp_controller-12] [INFO] [1793236682.796493579] [robot_b.mpc_rpp_controller]: Destroying
[ros2_control_node-5] [INFO] [1793236682.796620684] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-5] [INFO] [1793236682.796632170] [resource_manager]: Successful 'deactivate' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793236682.796635096] [resource_manager]: 'shutdown' hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793236682.796637145] [resource_manager]: Successful 'shutdown' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793236682.796664699] [robot_b.controller_manager]: Shutting down the controller manager.
[ros2_control_node-2] [INFO] [1793236682.796966019] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-2] [INFO] [1793236682.796973538] [resource_manager]: Successful 'deactivate' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793236682.796975453] [resource_manager]: 'shutdown' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793236682.796976997] [resource_manager]: Successful 'shutdown' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793236682.796996106] [robot_a.controller_manager]: Shutting down the controller manager.
[ERROR] [peer_state_relay-7]: process has died [pid 55, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_qvm1wtc9'].
[INFO] [mpc_rpp_controller-12]: process has finished cleanly [pid 68]
[ERROR] [dmpc_planner-8]: process has died [pid 57, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_fo7pm2s1 --params-file /tmp/launch_params_cmfeido1'].
[ERROR] [dmpc_planner-11]: process has died [pid 65, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_0k6lofy9 --params-file /tmp/launch_params_8jqdzk00'].
[ros2_control_node-5] [ERROR] [1793236682.804752221] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [ERROR] [1793236682.804760518] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [INFO] [1793236682.805061209] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793236682.805068533] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-5] [INFO] [1793236682.805144404] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793236682.805146453] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [ros2_control_node-5]: process has finished cleanly [pid 51]
[mpc_rpp_controller-9] [INFO] [1793236682.812228161] [robot_a.mpc_rpp_controller]: Destroying
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 50]
[INFO] [mpc_rpp_controller-9]: process has finished cleanly [pid 61]
[ros2_control_node-2] [ERROR] [1793236682.818678889] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793236682.818678851] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [INFO] [1793236682.819082144] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793236682.819089953] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793236682.819151646] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793236682.819153126] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 47]
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 48]
```

</details>

<a id="run-20261029_102529_266007__C"></a>

### 시나리오 C — 2026-10-29 10:25:29 KST

- 관측 판정: 미완료
- 종료 사유: 사용자 중단
- 설정 SHA-256: `0f19654e6ba92e42ee15d28379d8eb58aa8d1801cda35372cdffb30e813e65db`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 25.400 s
- 시각이 일치하는 odom 쌍: 1271
- 관측된 최소 중심 간격: 1.847 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: 0.017 m
- 고정 장애물 안전 범위 최소 여유: — m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 미도착 | 2.738 | 0.245 | 0.676 | 0.677 | 83 |
| robot_b | 미도착 | 5.110 | 0.310 | 0.393 | 0.397 | 48 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [peer_state_relay-10]: process has died [pid 77, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_p523enhi']. / [ERROR] [peer_state_relay-7]: process has died [pid 61, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_as5zk2sg']. / [ERROR] [dmpc_planner-11]: process has died [pid 78, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_kx8aqugy --params-file /tmp/launch_params_4_2h1x9h']. / [ERROR] [dmpc_planner-8]: process has died [pid 71, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_jaqfwxal --params-file /tmp/launch_params_lzkom5s_'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "wall_planning_margin": 0.001,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "871877ed5aa01e262a132e3c187e19afb5ef0d488e58be3ea24c337002c6a04d",
    "dmpc_runtime/ros_node.py": "75fa9e23b99cc0b6ed3557fdc59ba9adc1d48988ec1ca6e4c48c143a74feddcd",
    "dmpc_runtime/isaac_test.py": "c42c93a750f4127d92f5f26798c79dc50fcdbe021372670d1b4588e5c4dd4c2d",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "ff7f177d3df754eeedf531c6c48e2e4ed13d72c26d80a47fb618d34b58551ead"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 2,
        "tracking": 83,
        "solver_failed": 156
      },
      "controller": {
        "waiting_for_fresh_gt": 1,
        "mpc_waiting_for_fresh_states": 6,
        "tracking": 307,
        "waiting_for_clock": 2,
        "tracking_speed_reduced": 13,
        "mpc_solver_failed": 764
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 3,
        "tracking": 48,
        "solver_failed": 190
      },
      "controller": {
        "mpc_waiting_for_fresh_states": 5,
        "tracking": 173,
        "mpc_solver_failed": 909,
        "tracking_mpc_turn": 1,
        "waiting_for_clock": 1,
        "tracking_speed_reduced": 13
      }
    }
  },
  "final_positions": [
    [
      0.26251456141471863,
      0.0639227107167244,
      -0.2453828045158009
    ],
    [
      2.108015775680542,
      0.1325790137052536,
      2.8316172017348236
    ]
  ],
  "final_control_diagnostics": {
    "robot_a": {
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "mpc_solver_failed"
    },
    "robot_b": {
      "workspace_bounds_enabled": true,
      "static_obstacle_enabled": false,
      "workspace_center_bounds": [
        -3.45,
        3.45,
        -0.1499999999999999,
        0.1499999999999999
      ],
      "status": "mpc_solver_failed"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.15119668841362,
        -0.21200348436832428,
        0.1546226143836975,
        0.21370387077331543,
        -4.97006368637085,
        6.242480278015137,
        -5.095986366271973,
        -6.074763298034668
      ],
      "measured_velocities": [
        0.04949488490819931,
        0.02237071841955185,
        0.014864886179566383,
        0.0004906784743070602,
        -0.04148173704743385,
        0.017011549323797226,
        -0.010966591536998749,
        -0.000965178245678544
      ],
      "commanded_positions": [
        -0.1541565244208644,
        -0.21352273793188967,
        0.1541565244208644,
        0.21352273793188967,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.15538983047008514,
        -0.17399157583713531,
        0.15775269269943237,
        0.17274971306324005,
        -2.880373239517212,
        -4.614236831665039,
        -3.0591962337493896,
        -4.428624629974365
      ],
      "measured_velocities": [
        0.02905026637017727,
        -0.01583973318338394,
        0.017555637285113335,
        -0.01012248732149601,
        -0.025181539356708527,
        -0.015438813716173172,
        -0.014428747817873955,
        -0.007083861157298088
      ],
      "commanded_positions": [
        -0.15706018324533544,
        -0.17319037032193677,
        0.15706018324533544,
        0.17319037032193677,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    }
  },
  "final_plans": {
    "robot_b": {
      "valid": false,
      "status": "solver_failed",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    },
    "robot_a": {
      "valid": false,
      "status": "solver_failed",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    }
  }
}
```

</details>

<details><summary>마지막 실행 메시지</summary>

```text
[ros2_control_node-2] [INFO] [1793237182.649635356] [robot_a.controller_manager]: Shutdown request received....
[ros2_control_node-2] [INFO] [1793237182.649729970] [robot_a.controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-2] [INFO] [1793237182.649787913] [robot_a.controller_manager]: Deactivating controller 'antbot_swerve_controller'
[robot_state_publisher-1] [INFO] [1793237182.649858078] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793237182.650122531] [robot_a.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793237182.650562716] [resource_manager]: 'deactivate' hardware 'robot_a_isaac_system'
[ERROR] [peer_state_relay-10]: process has died [pid 77, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_p523enhi'].
[ros2_control_node-5] [INFO] [1793237182.650250024] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-5] [INFO] [1793237182.650310110] [resource_manager]: Successful 'deactivate' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793237182.650326509] [resource_manager]: 'shutdown' hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793237182.650341483] [resource_manager]: Successful 'shutdown' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793237182.650450837] [robot_b.controller_manager]: Shutting down the controller manager.
[ERROR] [peer_state_relay-7]: process has died [pid 61, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_as5zk2sg'].
[ros2_control_node-2] [INFO] [1793237182.651338917] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-2] [INFO] [1793237182.651399430] [resource_manager]: Successful 'deactivate' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793237182.651415915] [resource_manager]: 'shutdown' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793237182.651430583] [resource_manager]: Successful 'shutdown' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793237182.651533006] [robot_a.controller_manager]: Shutting down the controller manager.
[ERROR] [dmpc_planner-11]: process has died [pid 78, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_kx8aqugy --params-file /tmp/launch_params_4_2h1x9h'].
[ERROR] [dmpc_planner-8]: process has died [pid 71, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_jaqfwxal --params-file /tmp/launch_params_lzkom5s_'].
[mpc_rpp_controller-12] [INFO] [1793237182.657021719] [robot_b.mpc_rpp_controller]: Destroying
[INFO] [mpc_rpp_controller-12]: process has finished cleanly [pid 80]
[mpc_rpp_controller-9] [INFO] [1793237182.667595860] [robot_a.mpc_rpp_controller]: Destroying
[INFO] [mpc_rpp_controller-9]: process has finished cleanly [pid 74]
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 48]
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 51]
[ros2_control_node-2] [ERROR] [1793237182.684533873] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793237182.684533888] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [INFO] [1793237182.685323444] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793237182.685329369] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793237182.685451757] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793237182.685453229] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 49]
[ros2_control_node-5] [ERROR] [1793237182.693288095] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [ERROR] [1793237182.693288136] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [INFO] [1793237182.693918894] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793237182.693926346] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-5] [INFO] [1793237182.694183078] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793237182.694190242] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [ros2_control_node-5]: process has finished cleanly [pid 54]
```

</details>

<a id="run-20261029_102734_609728__D"></a>

### 시나리오 D — 2026-10-29 10:27:34 KST

- 관측 판정: 위치 도착(접촉·방향 별도)
- 종료 사유: 두 로봇 목표 도착
- 설정 SHA-256: `0f19654e6ba92e42ee15d28379d8eb58aa8d1801cda35372cdffb30e813e65db`
- 속도 제한: 1.0 m/s, 가속도: 0.5 m/s²
- 시뮬레이션 관측 시간: 18.960 s
- 시각이 일치하는 odom 쌍: 949
- 관측된 최소 중심 간격: 2.292 m (기준 1.15 m)
- 벽 안전 범위 최소 여유: — m
- 고정 장애물 안전 범위 최소 여유: 0.076 m

| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |
|---|---|---|---|---|---|---|
| robot_a | 도착 | 0.081 | 0.396 | 1.000 | 0.998 | 133 |
| robot_b | 도착 | 0.089 | 0.374 | 0.956 | 0.966 | 171 |

관측된 기준 위반: 없음

종료 정리 오류: 없음

자식 프로세스 오류: [ERROR] [peer_state_relay-10]: process has died [pid 56, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_pk7c0s71']. / [ERROR] [peer_state_relay-7]: process has died [pid 53, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_61ykjlkz']. / [ERROR] [dmpc_planner-11]: process has died [pid 57, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_gzmv43vh --params-file /tmp/launch_params_dpslifvh']. / [ERROR] [dmpc_planner-8]: process has died [pid 54, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_jnvd_17k --params-file /tmp/launch_params_a2mhyzcc'].

<details><summary>실행 설정·상태 집계</summary>

```json
{
  "config": {
    "use_sim_time": true,
    "horizon": 30,
    "dt": 0.2,
    "control_frequency": 10.0,
    "state_timeout": 0.35,
    "plan_timeout": 0.35,
    "max_speed": 1.0,
    "max_acceleration": 0.5,
    "max_yaw_rate": 0.8,
    "max_yaw_acceleration": 0.5,
    "separation": 1.15,
    "tracking_margin": 0.05,
    "wall_planning_margin": 0.001,
    "solve_seconds": 0.08,
    "goal_tolerance": 0.2,
    "goal_braking_deceleration": 0.25,
    "goal_reaction_time": 0.3,
    "goal_stop_reserve": 0.08,
    "peer_prediction_model": "cv_gaussian",
    "peer_prediction_noise": 0.3,
    "peer_prediction_dt": 0.2,
    "peer_prediction_risk": 0.05
  },
  "source_sha256": {
    "dmpc_runtime/planner.py": "871877ed5aa01e262a132e3c187e19afb5ef0d488e58be3ea24c337002c6a04d",
    "dmpc_runtime/ros_node.py": "75fa9e23b99cc0b6ed3557fdc59ba9adc1d48988ec1ca6e4c48c143a74feddcd",
    "dmpc_runtime/isaac_test.py": "c42c93a750f4127d92f5f26798c79dc50fcdbe021372670d1b4588e5c4dd4c2d",
    "isaac/path_following_controller/src/mpc_rpp_controller.cpp": "ff7f177d3df754eeedf531c6c48e2e4ed13d72c26d80a47fb618d34b58551ead"
  },
  "statuses": {
    "robot_a": {
      "planner": {
        "waiting_for_fresh_states": 7,
        "tracking": 133,
        "goal_reached": 404
      },
      "controller": {
        "waiting_for_fresh_gt": 2,
        "mpc_waiting_for_fresh_states": 10,
        "invalid_peer_state": 1,
        "tracking": 520,
        "tracking_mpc_turn": 1,
        "waiting_for_clock": 7,
        "mpc_goal_reached": 565
      }
    },
    "robot_b": {
      "planner": {
        "waiting_for_fresh_states": 3,
        "tracking": 171,
        "solver_failed": 3
      },
      "controller": {
        "mpc_waiting_for_fresh_states": 6,
        "tracking": 675,
        "mpc_solver_failed": 12,
        "waiting_for_clock": 6
      }
    }
  },
  "final_positions": [
    [
      2.9222257137298584,
      -0.021423213183879852,
      0.3963520716101043
    ],
    [
      -2.919987916946411,
      0.03990670666098595,
      -2.767237487332293
    ]
  ],
  "final_control_diagnostics": {
    "robot_a": {
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": true,
      "status": "mpc_goal_reached"
    },
    "robot_b": {
      "plan_age": 0.030000000000000002,
      "ego_age": 0.0,
      "peer_age": 0.02,
      "profile_speed": 0.04631288700332718,
      "profile_yaw_rate": -0.00834714858094184,
      "position_error": 6.750350147691889e-06,
      "measured_speed": 0.05479657438196696,
      "measured_yaw_rate": -0.00018916301601738604,
      "rpp_speed": 0.04631288700332718,
      "rpp_yaw_rate": 0.0,
      "nominal_separation": 1.15,
      "checked_separation": 1.1549999999999998,
      "separation": 5.841545594378381,
      "peer_checks": [
        {
          "robot_id": "robot_a",
          "age": 0.02,
          "separation": 5.841545594378381,
          "checked_separation": 1.1549999999999998,
          "uncertainty_radius_now": 0.04644273072245939
        }
      ],
      "yaw_source": "rpp",
      "peer_prediction_model": "cv_gaussian",
      "clearance_includes_gaussian": true,
      "static_obstacle": {
        "x": 0.0,
        "y": 0.0,
        "radius": 0.4,
        "checked_separation": 1.055,
        "clearance": 1.8692886995335745
      },
      "command_speed": 0.04631288700332718,
      "command_yaw_rate": 0.0,
      "predicted_clearance": 5.652457283689509,
      "speed_scale": 1.0,
      "rotation_only": false,
      "candidate_speed": 0.04631288700332718,
      "candidate_yaw_rate": 0.0,
      "predicted_static_clearance": 1.8642886995335746,
      "workspace_bounds_enabled": false,
      "static_obstacle_enabled": true,
      "status": "tracking"
    }
  },
  "final_joint_feedback": {
    "robot_a": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.0008710665279068053,
        -0.0023423731327056885,
        0.0017844594549387693,
        0.0002303608343936503,
        -0.8165590763092041,
        1.5462746620178223,
        -0.7547721266746521,
        1.568876028060913
      ],
      "measured_velocities": [
        -0.012039760127663612,
        -0.03989071771502495,
        0.0313480868935585,
        0.00019782455638051033,
        0.0207377802580595,
        -0.024780530482530594,
        -0.018866755068302155,
        0.00883412268012762
      ],
      "commanded_positions": [
        0.0,
        0.0,
        0.0,
        0.0,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.0,
        0.0,
        0.0,
        0.0
      ]
    },
    "robot_b": {
      "command_valid": true,
      "names": [
        "steering_front_left_joint",
        "steering_front_right_joint",
        "steering_rear_left_joint",
        "steering_rear_right_joint",
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
      ],
      "measured_positions": [
        -0.0006405820604413748,
        -0.004733393434435129,
        0.00105468335095793,
        0.004539343994110823,
        -2.022045612335205,
        -0.21780379116535187,
        -2.043152093887329,
        -0.23202116787433624
      ],
      "measured_velocities": [
        -0.042939577251672745,
        0.06273701786994934,
        0.004749741405248642,
        0.07704675197601318,
        0.5640938878059387,
        0.685469925403595,
        0.534546434879303,
        0.3731251358985901
      ],
      "commanded_positions": [
        0.0,
        -0.003263171795140995,
        0.0,
        0.003263171795140995,
        null,
        null,
        null,
        null
      ],
      "commanded_velocities": [
        null,
        null,
        null,
        null,
        0.4646868400576037,
        0.6797079288978738,
        0.5116528295099001,
        0.29663174066962994
      ]
    }
  },
  "final_plans": {
    "robot_a": {
      "valid": false,
      "status": "goal_reached",
      "positions": [],
      "velocities": [],
      "yaw_rates": []
    },
    "robot_b": {
      "valid": true,
      "status": "tracking",
      "positions": [
        [
          -2.917427442460007,
          0.040917446172920105
        ],
        [
          -2.924353868171737,
          0.038201323084672896
        ],
        [
          -2.9264373198396614,
          0.03738935828743847
        ],
        [
          -2.9275712983346684,
          0.03695080112700483
        ],
        [
          -2.9281683431108783,
          0.03672188776354102
        ],
        [
          -2.9284759044263438,
          0.03660504613856746
        ]
      ],
      "velocities": [
        0.0599826665933356,
        0.014416734626640868,
        0.007944079431638927,
        0.004214212202303539,
        0.002180035942394015,
        0.0011100402486455122
      ],
      "yaw_rates": [
        -0.00834714858094184,
        -0.012092153603325153,
        -0.014066940481147545,
        -0.015108079643898471,
        -0.01565681746125375,
        -0.015945879309891466
      ]
    }
  }
}
```

</details>

<details><summary>마지막 실행 메시지</summary>

```text
[ros2_control_node-5] [INFO] [1793237295.366753664] [resource_manager]: Successful 'deactivate' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793237295.366757738] [resource_manager]: 'shutdown' hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793237295.366761704] [resource_manager]: Successful 'shutdown' of hardware 'robot_b_isaac_system'
[ros2_control_node-5] [INFO] [1793237295.366794360] [robot_b.controller_manager]: Shutting down the controller manager.
[robot_state_publisher-1] [INFO] [1793237295.367208242] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793237295.368083200] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[ros2_control_node-2] [INFO] [1793237295.368109676] [robot_a.controller_manager]: Shutdown request received....
[ros2_control_node-2] [INFO] [1793237295.368124803] [robot_a.controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-2] [INFO] [1793237295.368131705] [robot_a.controller_manager]: Deactivating controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793237295.368181953] [robot_a.controller_manager]: Shutting down controller 'antbot_swerve_controller'
[ros2_control_node-2] [INFO] [1793237295.368246139] [resource_manager]: 'deactivate' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793237295.368574633] [antbot_isaac_topic_system]: Deactivated Isaac topic system with zero wheel command.
[ros2_control_node-2] [INFO] [1793237295.368579576] [resource_manager]: Successful 'deactivate' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793237295.368581190] [resource_manager]: 'shutdown' hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793237295.368582715] [resource_manager]: Successful 'shutdown' of hardware 'robot_a_isaac_system'
[ros2_control_node-2] [INFO] [1793237295.368599900] [robot_a.controller_manager]: Shutting down the controller manager.
[mpc_rpp_controller-12] [INFO] [1793237295.369067379] [robot_b.mpc_rpp_controller]: Destroying
[ERROR] [peer_state_relay-10]: process has died [pid 56, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_b --params-file /tmp/launch_params_pk7c0s71'].
[ERROR] [peer_state_relay-7]: process has died [pid 53, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/peer_state_relay --ros-args -r __ns:=/robot_a --params-file /tmp/launch_params_61ykjlkz'].
[INFO] [mpc_rpp_controller-12]: process has finished cleanly [pid 58]
[ERROR] [dmpc_planner-11]: process has died [pid 57, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_b --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_gzmv43vh --params-file /tmp/launch_params_dpslifvh'].
[ERROR] [dmpc_planner-8]: process has died [pid 54, exit code -15, cmd '/workspace/isaac/control_build/install/antbot_dmpc_ros/lib/antbot_dmpc_ros/dmpc_planner --ros-args -r __ns:=/robot_a --params-file /workspace/isaac/control_build/install/antbot_dmpc_ros/share/antbot_dmpc_ros/config/dmpc.yaml --params-file /tmp/launch_params_jnvd_17k --params-file /tmp/launch_params_a2mhyzcc'].
[ros2_control_node-5] [ERROR] [1793237295.377143560] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [ERROR] [1793237295.377195233] [robot_b.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-5] [INFO] [1793237295.377380672] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793237295.377414752] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-5] [INFO] [1793237295.377522981] [robot_b.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-5] [INFO] [1793237295.377526410] [robot_b.controller_manager.pal_statistics]: publish_async_failures_ 0
[mpc_rpp_controller-9] [INFO] [1793237295.380686335] [robot_a.mpc_rpp_controller]: Destroying
[INFO] [ros2_control_node-5]: process has finished cleanly [pid 51]
[ros2_control_node-2] [ERROR] [1793237295.384136247] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [ERROR] [1793237295.384136299] [robot_a.controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
[ros2_control_node-2] [INFO] [1793237295.384444650] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793237295.384450763] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[ros2_control_node-2] [INFO] [1793237295.384510893] [robot_a.controller_manager.pal_statistics]: Async messages lost 0
[ros2_control_node-2] [INFO] [1793237295.384512646] [robot_a.controller_manager.pal_statistics]: publish_async_failures_ 0
[INFO] [mpc_rpp_controller-9]: process has finished cleanly [pid 55]
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 50]
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 48]
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 47]
```

</details>
