# Mecanum Control PC

ROS가 설치되지 않은 외부 PC에서 실행하는 Python 3 / Tkinter 제어 프로그램입니다.
Gateway의 TCP/NDJSON 인터페이스만 사용하며 ROS 또는 pip 패키지가 필요하지 않습니다.
Windows/Linux의 Tkinter가 포함된 Python 3 환경에서 실행할 수 있습니다.
Linux 배포판에 따라 Tkinter는 OS의 `python3-tk` 패키지를 별도로 설치해야 합니다.

```bash
cd control_pc
python3 mecanum_control.py
```

Windows에서는 `py mecanum_control.py`도 사용할 수 있습니다.
`python3 -m tkinter`로 Tkinter 설치와 데스크톱 표시 환경을 확인할 수 있습니다.

1. Robot IP에 로봇의 숫자 IP 주소, Port에 Gateway port(기본 `8765`)를 입력합니다.
   같은 PC에서 시험할 때는 `127.0.0.1` 또는 `localhost`를 사용합니다.
2. Connect를 누릅니다. 최근 1.5초 이내 정상 `robot_state`를 받아야 CONNECTED가 됩니다.
   TCP 연결만 열렸거나 command_result만 수신한 경우에는 CONNECTED가 아닙니다.
3. Base Ready, Mode, Pose(m/rad), Velocity(m/s, rad/s), Motion, Error를 확인합니다.
4. BASE에서 Start Mapping 또는 Start Navigation을 사용할 수 있습니다.
   MAPPING에서는 Save Map과 Stop Mode, NAVIGATION에서는 Stop Mode를 사용할 수 있습니다.
5. Map Name에는 `room_01`처럼 확장자 없는 이름을 입력합니다. 실제 경로는 로봇 Gateway의
   `map_dir` 설정으로 결정되며, PC의 경로나 지도 파일을 전송하지 않습니다.
6. 하단 로그에서 증가하는 request_id와 요청별 OK/FAILED 결과를 확인합니다.
   저장/종료는 완료까지 시간이 걸릴 수 있습니다. 최종 허용 여부는 Gateway가 판단합니다.

연결이 끊기거나 telemetry가 1.5초 이상 오지 않으면 DISCONNECTED와 오래된 데이터 표시가
나타나고 운용 버튼이 비활성화됩니다. 마지막 수치는 유지됩니다. Connect로 수동 재연결할 수
있으며 heartbeat나 자동 재연결은 없습니다. 연결이 끊긴 요청은 결과를 알 수 없다고 표시합니다.

Disconnect 또는 창 닫기는 수동 주행 zero 명령을 가능한 한 전송한 뒤 PC 통신을 종료합니다. 로봇에서 실행 중인 Mapping/Navigation은
계속 실행됩니다. 필요하면 먼저 Stop Mode의 완료 응답을 확인하세요.

`robot_client.py`는 nonblocking socket을 사용하는 별도 worker와 thread-safe queue를 담당하고,
`mecanum_control.py`는 Tkinter main thread에서만 UI를 갱신합니다.
연결 판정 상수는 `robot_client.py`의 `ROBOT_STATE_TIMEOUT = 1.5`입니다.
창 종료 시 socket을 shutdown/close하고 worker를 join합니다.

## Mapping 수동 주행

MAPPING에서만 수동 주행 버튼이 활성화됩니다. Linear 기본 0.10 m/s, Angular 기본
0.40 rad/s를 변경할 수 있습니다. 버튼을 누르는 동안 100 ms마다 전송하고 놓으면 zero를
전송합니다. Left/Right는 좌우 평행 이동, Rotate L/R은 좌우 회전입니다. Stop은 즉시 zero를
보냅니다. 포커스 이탈·최소화·창 종료에서도 가능한 한 zero를 전송합니다.

Gateway가 최종 모드 검사와 속도 제한(기본 vx/vy 각각 ±0.30 m/s, wz ±1.00 rad/s)을 적용합니다.
수동 명령이 끊기면 Gateway의 steady-clock timeout(기본 0.30초)이 한 번 zero를 발행합니다.
전송되지 않은 수동 속도는 최신 값으로 교체하여 오래된 명령이 쌓이지 않게 합니다.

## 실시간 지도

오른쪽 Canvas에 최대 1 Hz의 변경된 지도를 표시합니다. Unknown은 회색, free는 흰색,
occupied는 검정이며 중간 점유 값은 grayscale입니다. 이미지 Y축을 뒤집고 Canvas 크기에
맞춰 nearest-neighbor로 축소/확대합니다. 고급 zoom/pan은 없습니다.

`map_view.py`가 지도 압축 해제 검증과 표시를 담당합니다. base64/zlib는 Python 표준 라이브러리며
추가 pip dependency는 없습니다. 지도는 최대 1,048,576 cells를 지원합니다.

기존 Pose 숫자는 odom 좌표를 유지합니다. 파란 점/방향선은 Gateway가 TF로 변환한 `map_pose`를
사용하고 map origin의 회전·이동·resolution을 적용합니다. TF/odometry가 없거나 오래됐거나
연결이 끊기면 overlay를 숨깁니다. 원본 지도 metadata는 유지됩니다.

현재 지원 범위는 연결, robot_state, Mapping/Navigation 모드, map 저장, 지도 표시 및 Mapping
수동 주행, Navigation goal 및 Initial Pose입니다. Supervisor는 포함하지 않습니다.

## Navigation goal

NAVIGATION에서 지도 위 왼쪽 버튼을 누르면 위치를 선택하고, 드래그하면 방향을 지정합니다.
놓으면 주황색 원/방향선과 Goal X/Y/Yaw가 표시됩니다. 짧은 클릭은 현재 map_pose yaw,
없으면 이전 goal yaw를 사용합니다(둘 다 없을 때만 0). 지도 바깥 여백 클릭은 무시합니다.

선택만으로 이동하지 않습니다. **Send Goal**을 눌러야 전송합니다. Navigation 상태에
PENDING → EXECUTING → SUCCEEDED/ABORTED/CANCELED 또는 REJECTED가 표시됩니다.
Send Goal 응답의 성공은 목표 수락이며 도착을 뜻하지 않습니다. 실행 중 새 goal은 Gateway가
거부합니다. Cancel Goal은 실행 중 취소를 요청하고, 최종 CANCELED는 telemetry로 확인합니다.
BASE/MAPPING 및 연결 끊김에서는 goal 조작과 전송/취소가 비활성화됩니다.
Supervisor는 포함하지 않습니다.


## AMCL Initial Pose

NAVIGATION에서 **Select Initial Pose**를 누른 뒤 지도를 press-drag-release하여 초기
위치와 방향 후보를 선택합니다. 보라색 사각형/방향선이 후보이며 파란 로봇 위치 및 주황색
Goal과 구분됩니다. **Apply Initial Pose**를 눌러야 전송합니다. 성공 응답은 발행 확인이며
AMCL 수렴을 뜻하지 않습니다. 이후 로봇의 map_pose 갱신을 확인하세요.

**Select Goal**을 누르면 Goal 선택으로 돌아갑니다. 두 후보는 독립적으로 유지되고,
현재 선택 모드는 지도 아래에 표시됩니다. 짧은 클릭의 초기 방향은 현재 로봇 map yaw,
없으면 이전 Initial Pose 후보 yaw, 둘 다 없으면 0입니다. BASE/MAPPING 또는 연결 끊김에서는
선택 버튼과 Apply가 비활성화됩니다. 기존 지도 좌표 변환을 그대로 사용합니다.
