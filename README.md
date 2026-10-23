# vehicle_recorder — 별도 기록과 정지 관측

이 패키지는 `monitor_1021`의 기록 기능과 이전 정지 관측 구현을 모은다.
기존 `vehicle_stop` 소스 패키지는 삭제하고 메시지 import/launch를 이 이름으로 이관했다.
관측용 CommandEvent/StopStatus의 필드와 토픽, 실제 제어 명령의 형식은 유지했다.

## 프로세스와 제어 경계

- `record`: 토픽을 구독하여 CSV/JSONL에 저장한다. 제어 명령·운용 상태를 발행하지 않는다.
- `stop_observer`: GNSS와 명령 시각을 사용해 정지 상태를 계산·발행한다. 디스크/웹 I/O가 없다.
- C++ 제어기와 manual/trans는 메시지 정의만 참조한다. 기록 노드의 실행 여부와 무관하게
  기존 명령을 발행/송신한다. 정지 시 마지막 조향을 유지하는 로직은 바꾸지 않았다.
- control_console의 안전 종료는 기록 노드가 아닌 정지 관측 결과를 확인한다.
  기록 프로세스를 종료해도 정지 판정은 유지된다.

디스크 쓰기는 기록 프로세스 내부의 별도 스레드·유한 큐를 사용한다. ROS 콜백은 큐가 꽉 차도
파일 쓰기를 기다리지 않으며 누락/쓰기 오류는 metadata와 오류 로그에 표시한다.
큐는 기본 4096건, flush는 1초 간격이다. best-effort 구독이므로 DDS 전달 전체를 보장하지 않는다.
기록을 시작하고 구독 준비를 확인한 뒤 시험한다. 메모리 큐와 버퍼는 전원 차단 시 손실될 수 있다.
CPU·메모리·디스크는 같은 컴퓨터에서 공유하므로 물리적 부하가 0이라는 뜻은 아니다.
실차에서는 기록 전후 제어 주기·지터를 비교하고, 필요하면 CPU/디스크 자원을 추가 분리한다.

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

기본 `/workspace/log/vehicle_recorder/<실행별 폴더>/`는 호스트
`main/log/vehicle_recorder/<실행별 폴더>/`에 보존된다.

- `messages.jsonl`: 토픽·타입·원본 timestamp·수신 ROS/monotonic 시각과 메시지 값.
- `topic-<hash>.csv`: 토픽/타입별 표. 중첩 필드는 점 표기, 배열은 JSON 문자열.
- reference_path 이벤트: 실제 제어 경로의 좌표계·XY·해시를 내용 변경 시 보존.
- `metadata.json`: 토픽/CSV 매핑, 코드 해시, Docker 정보, 누락·오류·완료 여부.
- `config/`: `snapshot_files`에 지정한 실제 설정·지도 사본.

GNSS 원본, 제어 상태, PP 진단, 자동/선택/액추에이터 명령, 명령 발행/전송 시각,
정지 관측 결과, 모터·조향 피드백을 기본으로 기록한다. 헤더 없는 피드백의 MCU tick은
ROS 시각으로 바꾸지 않고 그대로 보존하며 원본 timestamp는 null로 둔다.
NaN/Inf는 null로 기록하고 원본 유효성 필드는 유지한다.
사용한 지도·설정·버전은 기존 control_console의 실행 폴더와 함께 보존하거나
그 실행 폴더의 정확한 설정 사본을 `config/record.yaml`의 snapshot_files에 지정한다.

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
