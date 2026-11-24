# Isaac 3대 워크스페이스 구성

기본 실행은 워크스페이스 최상위 `run.sh`를 그대로 사용한다.
Gazebo 및 Isaac 2대 워크스페이스와 별도로 유지한다.

| 경로 | 역할 |
|---|---|
| `docker/build/` | ROS 메시지·제어기·MPC 빌드 및 solver 준비 |
| `docker/legacy/` | 이전 2대 실행 도구와 legacy 시뮬레이션 설정 |
| `simulation/` | 현재 3대 Isaac runner와 배치 정의 |
| `simulation/tools/` | 준비 상태 확인·상태 모니터링 |
| `simulation/legacy/` | 이전 2대 runner |
| `tests/mpc/` | 예측·MPC·재계획 검증 |
| `tests/rpp/` | 경로 추종·명령 검사·native RPP 검증 |
| `tests/swerve/` | 조향·구동 프로파일·관절 추종 검증 |
| `tests/isaac/` | 3대 배치·Isaac 연결·물리 주행 검증 |
| `tests/legacy/` | 이전 Python MPC·Adapter·2대 회귀 검증 |
| `docs/history/` | 복사 당시 파일 목록 등 이력 자료 |

단위 검사: 워크스페이스 루트에서 `python -m unittest discover -s tests -v`.
ROS·Isaac 연결 검사는 각 하위 폴더의 `check_*.py`를 명시적으로 실행한다.
Docker Compose는 기존과 같이 워크스페이스 루트를 `--project-directory`로 지정한다.
`docker/legacy/compose.legacy_mpc.yaml`의 ros 서비스는 현재 MPC Compose도 상속하므로 삭제하지 않는다.
ROS 패키지·설치 경로와 로봇 USD·메시는 유지한다.

## 정리 후 확인 (2026-11-04)

- 단위 검사 74개 실행: 73개 통과, 명시적 활성화가 필요한 ROS 검사 1개 건너뜀.
- staggered/simultaneous 배치 설정 확인 및 MPC/Isaac Compose 구성 검사 통과.
- 실제 C++ 조향 프로파일 600개, 연속 재계획 250회, 저속 회전 150회와 native/Python 추종 비교 261개 통과. C++ 검사는 `bash docker/ros-entrypoint.sh`로 ROS 설치 환경을 로드한 뒤 실행한다.
- 기존 설치 산출물에 일부 local_setup.bash 및 패키지 등록 누락이 확인됐다. 관련 ROS 패키지와 build/install 디렉토리는 이번에 이동하지 않았다. 실제 Isaac 주행 전 `sudo bash run.sh build`로 재빌드하고 패키지 인식을 재확인해야 한다.
- 실제 Isaac 물리 주행은 이번 정리 과정에서 실행하지 않았다.
