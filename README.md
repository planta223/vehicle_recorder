# vehicle_recorder — 별도 기록과 정지 관측

이 패키지는 `monitor_1021`의 기록 기능과 이전 정지 관측 구현을 모은다.
기존 `vehicle_stop` 소스 패키지는 삭제하고 메시지 import/launch를 이 이름으로 이관했다.
관측용 CommandEvent/StopStatus의 필드와 토픽, 실제 제어 명령의 형식은 유지했다.

## 프로세스와 제어 경계

- `record`: 시험 세션을 관리하고 C++ `record_backend`가 토픽 전체를 rosbag2에 저장한다. 제어 명령·운용 상태를 발행하지 않는다.
- `stop_observer`: GNSS와 명령 시각을 사용해 정지 상태를 계산·발행한다. 디스크/웹 I/O가 없다.
- C++ 제어기와 manual/trans는 메시지 정의만 참조한다. 기록 노드의 실행 여부와 무관하게
  기존 명령을 발행/송신한다. 정지 시 마지막 조향을 유지하는 로직은 바꾸지 않았다.
- control_console의 안전 종료는 기록 노드가 아닌 정지 관측 결과를 확인한다.
  기록 프로세스를 종료해도 정지 판정은 유지된다.

C++ generic subscription은 메시지를 Python 객체로 풀지 않고 수신한 CDR 버퍼 전체를
별도 writer 스레드에 넘긴다. writer는 `rosbag2_cpp::Writer`와 SQLite3 플러그인을 사용한다.
실시간 CSV/JSONL 변환, 배열 문자열화, 경로 XY 추출·해시 계산·중복 구독은 제거했다.
전체 경로를 포함한 모든 지정 메시지를 매번 보존하므로 경로 변경 이력도 사후에 재구성할 수 있다.
`stop_observer`의 실시간 정지 판정과 외부 제어기의 계산·발행 기능은 유지한다.

저장 큐는 기본 4096건/256 MiB 중 먼저 도달하는 한도, DDS 수신 depth는 4096이다.
큐 초과 시 덮어쓰기나 조용한 샘플 생략을 하지 않고 실패를 표시하고 기록을 종료한다.
기존 큐는 모두 비운 뒤 종료하며 임의의 시간 제한으로 writer를 강제 종료하지 않는다.
rosbag2의 별도 캐시는 비활성화하고 SQLite `resilient` 프리셋을 사용한다.
QoS는 rosbag2의 publisher 적응 방식을 사용하며 실제 offered/requested QoS도 보존한다.
감지 가능한 DDS loss와 QoS 불일치는 오류와 실패로 기록한다. 지원하지 않는 DDS loss
이벤트는 `dds_lost_events_supported: false`로 표시한다.

`complete`는 수신 콜백 이후 저장 완료 및 감지된 오류 여부에 대한 값이다. 센서·드라이버·DDS
전체의 무손실 인증이 아니며 `acquisition_lossless_verified`는 기본 false다.
구독 준비 전·종료 후 발행, best-effort 전송 중 손실, DDS가 보고하지 않는 손실은 이 값만으로
판정할 수 없다. 먼저 `Recording serialized ...`를 확인하고 시험하며, 시험 종료 후 입력을 멈추고
기록을 닫는다. `undiscovered_topics`와 토픽별 수신/저장 건수도 확인한다.
프로세스 강제 종료·전원 차단 시 메모리 큐의 데이터는 보장하지 않는다.

## 독립 저장소 빌드와 실행

이 저장소의 루트가 ROS 2 `vehicle_recorder` 패키지다. `control_manager`,
`manual_control`, `trans`, 실차 로그와 지도 파일 자체는 이 저장소에 포함하지 않는다.
ROS 2 Humble 환경에서 colcon 워크스페이스의 `src/vehicle_recorder`로 배치한다.

```bash
cd ~/vehicle_recorder_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select vehicle_recorder --symlink-install
source install/setup.bash
ros2 run vehicle_recorder record --ros-args \
  --params-file src/vehicle_recorder/config/record.yaml \
  -p log_root:=./log/vehicle_recorder
```

위 예시는 저장소를 `~/vehicle_recorder_ws/src/vehicle_recorder`에 clone한 뒤 실행한다.
기본 설정의 `/workspace/log/...`는 기존 컨테이너용 경로이므로 독립 실행에서는
위와 같이 `log_root`를 지정한다. 시험별 토픽과 `snapshot_files`는 자체 YAML로 설정한다.
기록기가 구독하는 사용자 정의 메시지는 해당 메시지 패키지가 설치되어 있어야 한다.
예를 들어 `/control/*`의 제어 메시지에는 `control_manager`, 피드백 메시지에는
`encoder_motor`의 타입 정의가 필요하며, 이 저장소가 그 패키지를 자동 실행하지 않는다.

아래 실행 절차의 `main/IF_YOU_WANT_TO_RUN/...` 경로는 기존 2026WS 워크스페이스의
통합 실행 스크립트다. 독립 clone에는 포함되지 않으며 직접 `ros2 run` 또는
이 패키지의 `record.xml` / `stop_observer.xml`을 사용한다.

## 보존한 편집 이력

기존 `.git` 이력은 없었으며, 2026-10-21~22에 보존된 소스 스냅샷에서 확인 가능한
변경을 복구 커밋으로 재구성했다. 커밋 시각은 복구한 시각이며 원래 편집 시각이나
작성자를 의미하지 않는다. 부분 백업은 실제 남아 있는 파일만 반영했고 동일한
백업은 중복 커밋하지 않았다. 각 백업 경로, 파일 SHA-256, 대응 커밋은
[history/recovery.json](history/recovery.json)에 기록했다.

초기화 기준 태그 `baseline-20261024`까지는 Python JSONL/CSV 방식이다.
이후 C++ rosbag2 방식으로 전환했으며 이전 기록 파일과 과거 Git 이력은 변경하지 않는다.

## 실행

기존 main 컨테이너에 빌드한 뒤 사용한다.

```bash
bash main/IF_YOU_WANT_TO_RUN/vehicle_recorder.sh
```

또는 컨테이너에서 `ros2 launch vehicle_recorder record.xml`을 실행한다.
`control_manager`는 파일을 쓰지 않는 정지 관측 노드만 자동으로 실행한다.
표시·기록 프로세스는 자동으로 시작하지 않는다.

수동시험에서 정지 관측 노드가 없다면 하나만 별도로 실행한다.

```bash
ros2 launch vehicle_recorder stop_observer.xml command_topic:=/cmd
ros2 launch vehicle_recorder record.xml
```

standalone 결과 토픽은 `/vehicle/stop_status`, control_manager 결과는 `/control/stop_status`다.
두 토픽은 기록/표시 기본 목록에 포함된다. 기존 `stop_monitor.sh`는 standalone 정지 관측 진입점이다.

## 저장

대화형 터미널(`docker exec -it main bash`)에서는 `q` 또는 `Q` 한 번으로 기록을
종료한다. 대기 중인 파일 쓰기를 마친 뒤 `정상적인 기록입니까? y/n:`을 묻는다.
`y`를 입력하면 `기록 이름을 입력해주세요.`가 표시되고, 입력한 이름을 기존 실행
폴더명 뒤에 붙인다. 한글과 공백을 사용할 수 있으며 빈 이름·경로 구분자·제어문자는
허용하지 않는다(UTF-8 120바이트 이내). `n`이면 이름을 묻지 않고 원본 폴더를 보존한다.
`metadata.json`의 `review_status`는 `pending`/`accepted`/`rejected`, `record_name`은
입력한 이름이다. 이는 writer의 누락·오류를 나타내는 `complete`와 별개의 운영자 평가다.
`Ctrl+C`/SIGTERM이나 터미널 없는 자동 실행은 기존처럼 파일을 닫고 미검토로 남긴다.
자동 실행에서 터미널 입력을 사용하지 않으려면 `-p interactive_review:=false`를 지정한다.
launch 자식 노드도 `/dev/tty`를 통해 같은 대화형 터미널에서 조작할 수 있다.
제동 시험 지원 launch에서는 시험 조작 터미널의 정지·종료를 확인한 뒤 기록을 종료한다.
기록 검토가 끝나면 지원 launch의 정지 관측·trans도 자동 종료된다.

기본 `/workspace/log/vehicle_recorder/<실행별 폴더>/`는 호스트
`main/log/vehicle_recorder/<실행별 폴더>/`에 보존된다.

- `bag/`: ROS 메시지 전체를 CDR로 저장한 rosbag2 SQLite3 `.db3`와 `metadata.yaml`.
  source header stamp, MCU tick, 모든 배열/공분산/유효성 필드 및 NaN/Inf 비트도 그대로 남는다.
- `receipts.csv`: 전체 삽입 순서 `bag_message_index`, 토픽, 토픽 내 수신 순서,
  `received_ros_ns`, `received_monotonic_ns`, 직렬화 크기. bag timestamp는 같은 수신 ROS 시각이다.
  원본 header stamp는 메시지 내부에서 읽으며 수신 시각으로 대체하지 않는다.
- `metadata.json`: 기록 설정의 실제 적용값, 실행 파일/스크립트 해시, Docker 정보,
  설정·인터페이스 해시, backend 결과, 미발견 토픽, 운영자 평가와 기록 이름.
- `backend.yaml`: 토픽별 타입·QoS·수신/저장 건수, 감지된 DDS 누락·큐 초과·오류,
  큐 최대 사용량, CPU 시간. 기록 중 1초 간격으로 갱신한다.
- `backend_parameters.yaml`: backend에 실제 전달한 파라미터.
- `interfaces/`: 사용한 패키지 및 메시지 의존 패키지의 설치된 msg/idl/srv/action 정의와 package.xml.
  발견 시 한 번 복사하며 메시지를 사후에 해석할 때 사용한다.
- `config/`: `snapshot_files`에 지정한 설정·지도 사본.

`reference_path_topic`은 `topics`에 없더라도 전체 메시지를 기록한다. 목록에 중복되면
하나만 구독한다. 전체 메시지의 구독·저장 주기를 줄이거나 경로를 한 번만 저장하지 않는다.
메시지 기반 속도/가속도, 경로 해시, CSV/JSONL, 그래프는 사후 변환으로 만든다.
기존 JSONL/CSV 분석 스크립트는 새 bag을 읽거나 오프라인 변환하도록 바꿔야 한다.
기존 로그와 그래프 파일을 자동 변환하거나 삭제하지 않는다.

```bash
ros2 bag info <시험폴더>/bag
# 사후 도구의 rosbag2_py.SequentialReader로 CDR 읽기 가능.
```

ROS 원본은 센서 통신 원본(CAN/UDP/NMEA)과 구분한다. 드라이버가 ROS 발행 전에 수행한
환산·반올림·필드 선택은 이 기록기가 복원할 수 없으며 별도의 통신 원본 수집이 필요하다.
사용한 지도·설정·버전은 기존 control_console 실행 폴더와 함께 보존하거나
정확한 사본을 `snapshot_files`에 지정한다. 실행 중 외부 노드의 파라미터 override는
그 노드의 실행 기록과 함께 보존한다.

## 검증

일반 단위 검사는 `colcon test --packages-select vehicle_recorder`로 실행한다.
같은 Docker 이미지의 `--rm --network none` 컨테이너에서 실제 주행과 분리된 ROS domain을
설정하고 다음 통합 검사를 실행한다. 증거 경로는 호스트에 마운트한다.

```bash
export ROS_DOMAIN_ID=196 ROS_LOCALHOST_ONLY=1 VEHICLE_RECORDER_ISOLATED_TEST=1
export CDR_TEST_OUT=/evidence/cdr
python3 src/vehicle_recorder/test/cdr_recording_smoke.py
```

240개 원문 CDR 바이트와 저장 건수, 1217점 전체 경로, NaN payload/Inf/-0/subnormal,
header·공분산, ROS/monotonic 시각 연결, 인터페이스 정의를 검사한다.
별도로 1바이트 큐 한도와 `/dev/full` 쓰기 실패가 `complete: false` 및 오류 종료로
보고되는지도 확인한다. 이 검사는 시험 구간의 일치 확인이며 모든 실차 조건의 전달 보장은 아니다.

## 부하 재현 결과 (2026-10-24)

10월 23일의 17개 토픽 크기·발행 주기와 1217점 경로를 격리 환경에서 재현했다.
8초 측정 구간에서 기록기 전체 프로세스의 CPU는 1코어 기준 **101.25% → 6.63%**로
약 **93.46%** 줄었고, GNSS는 발행 160개 중 기존 68개/새 방식 160개를 기록했다.
새 방식은 전체 6346개를 모두 저장했으며 모든 필드·배열·부동소수점 비트가 일치했다.
별도 240개 메시지 검사는 원문 CDR 바이트도 일치했고 NaN payload/Inf/-0 등을 포함했다.
미들웨어의 CDR 정렬 padding은 발행측과 수신측에서 달라질 수 있어 전체 토픽 비교에서는
모든 데이터 필드를 비트 기준으로 검사했다. 기록기는 수신한 직렬화 버퍼를 변환하지 않는다.

실차 CPU 측정값이나 모든 조건의 무손실 보장은 아니다. 같은 규모에서는 기록기 CPU가
약 90% 이상 감소할 것으로 예상하지만 토픽 양·디스크·DDS 상태에 따라 달라진다.
검증 요약은 [test/results/rosbag2_20261024.json](test/results/rosbag2_20261024.json)에 남겼다.

## 정지 판정 계약

유효하고 시각이 증가하는 GNSS 샘플의 `|v| ≤ 0.1 m/s`가 0.5초 지속되면 확정한다.
`stopped_since`는 그 구간 첫 샘플 시각, `confirmed_at`은 확정 샘플 시각이다.
반복된 동일 샘플은 지속 시간을 채우지 않는다. 속도 초과·불량·시각 역행·0.3초 넘는
수신/샘플 공백은 판정을 초기화한다. 현재 정지 여부와 정지 명령 유무는 별개다.

CommandEvent는 발행 직전 COMMAND와 UDP 첫 sendto 직전 TRANSMITTED를 구분한다.
TRANSMITTED는 세 목적지 전송이 모두 성공한 뒤 발행하며 보드 수신/물리 제동 시작은 뜻하지 않는다.
DRY_RUN에는 실제 전송 시각을 부여하지 않는다. 같은 정지 명령의 반복은 최초 시각을 덮어쓰지 않는다.
정지거리 distance_m은 전송 시점 위치에서 최초 정지 샘플 위치까지의 평면 직선거리다.
위치는 양쪽 유효 GNSS 샘플로만 보간하고 외삽하지 않으며, 미확보 값은 valid=false다.
