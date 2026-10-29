#!/usr/bin/env bash
# start.sh
# [REVIEW #25] shebang은 반드시 1행에 있어야 함 (주석은 2행으로 이동)
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/logs"
mkdir -p "${LOG_DIR}"

# [REVIEW #3] 종료 요청 여부는 종료 코드가 아니라 플래그 파일로 판단
STOP_FLAG="${LOG_DIR}/.stop_requested"

# ==========================================================
# 🔁 이전에 실행 중인 start.sh가 있으면 정상 종료시킨 뒤 새로 실행
# ==========================================================
# - 감시 루프(서브셸)도 이름이 'bash ./start.sh'라서, 부모가 start.sh가 아닌 '본체'만 골라 종료
# - 본체는 마지막에 tail -f를 포그라운드로 기다리고 있어 TERM만 보내면 trap(cleanup)이 실행되지 않음
#   -> tail도 함께 종료해야 cleanup(노드 정리)이 돈다
stop_previous_instances() {
  local pid ppid args pcmd cwd
  local prev=()
  for pid in $(pgrep -f "start\.sh" 2>/dev/null || true); do
    [ "${pid}" = "$$" ] && continue
    ppid="$(ps -o ppid= -p "${pid}" 2>/dev/null | tr -d ' ')"
    [ -z "${ppid}" ] && continue
    [ "${ppid}" = "$$" ] && continue
    args="$(ps -o args= -p "${pid}" 2>/dev/null || true)"
    [[ "${args}" == bash*start.sh* ]] || continue
    pcmd="$(ps -o args= -p "${ppid}" 2>/dev/null || true)"
    [[ "${pcmd}" == *start.sh* ]] && continue
    # 이 프로젝트의 start.sh인지 확인 (절대경로 실행 또는 이 폴더에서 ./start.sh 실행)
    cwd="$(readlink "/proc/${pid}/cwd" 2>/dev/null || true)"
    [[ "${args}" == *"${SCRIPT_DIR}"* || "${cwd}" == "${SCRIPT_DIR}" ]] || continue
    prev+=("${pid}")
  done
  [ "${#prev[@]}" -eq 0 ] && return 0

  echo "🔁 이전에 실행 중인 start.sh 발견 (PID: ${prev[*]}) -> 종료 후 새로 시작합니다..."
  for pid in "${prev[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
    pkill -TERM -P "${pid}" -x tail 2>/dev/null || true
  done

  # 이전 cleanup은 최대 약 6초 소요 -> 15초까지 대기
  for _ in $(seq 1 150); do
    local alive=0
    for pid in "${prev[@]}"; do
      kill -0 "${pid}" 2>/dev/null && alive=1
    done
    [ "${alive}" -eq 0 ] && break
    sleep 0.1
  done

  # 그래도 남아 있으면 본체와 감시 루프를 강제 종료 (노드는 아래 kill_leftovers가 정리)
  for pid in "${prev[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      echo "⚠️  PID ${pid}가 응답하지 않아 강제 종료합니다."
      pkill -KILL -P "${pid}" 2>/dev/null || true
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  done
}
stop_previous_instances

rm -f "${STOP_FLAG}"

# 옵션 (환경변수로 변경 가능)
#   ENABLE_SLAM=0 ./start.sh  -> SLAM 끄고 주행 (제어에는 SLAM 결과를 쓰지 않음, REVIEW #20)
#   USE_RVIZ=1    ./start.sh  -> rviz2 함께 실행
ENABLE_SLAM="${ENABLE_SLAM:-1}"
USE_RVIZ="${USE_RVIZ:-0}"

# 모든 노드 프로세스 패턴 (시작 전 정리 + 종료 시 잔여 정리 공용)
NODE_PATTERNS=(
  "python3.*main_fusion.py"
  "python3.*esp32_bridge.py"
  "python3.*imu.py"
  "python3.*lidar.py"
  "python3.*yolo26n.py"
  "python3.*gps_tmap_nav.py"
  "python3.*gps_imu_odom.py"
  "python3.*web_dashboard.py"
  "python3.*tmap.py"
  "python3.*sensor_monitor.py"
  "slam_toolbox"
  "ldlidar"
  "rviz2"
)

kill_leftovers() {
  local sig="$1"
  fuser -k "-${sig}" 5000/tcp 2>/dev/null || true
  for pat in "${NODE_PATTERNS[@]}"; do
    pkill "-${sig}" -f "${pat}" 2>/dev/null || true
  done
}

echo "🚀 자율주행 및 통합 관제 시스템 백그라운드 구동"

# ROS 2 환경변수 로드
if [ -f "/opt/ros/humble/setup.bash" ]; then
  set +u
  source "/opt/ros/humble/setup.bash"
  set -u
fi

# 워크스페이스 오버레이 로드
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
  set +u
  source "${SCRIPT_DIR}/install/setup.bash"
  set -u
fi

# 하드웨어 권한 자동 개방
# [REVIEW #19] 코드가 여는 이름(/dev/gps, /dev/robot_cam)도 포함. udev 규칙과 이름을 하나로 통일 권장
sudo chmod 666 /dev/esp32 /dev/rplidar /dev/imu /dev/gps /dev/ublox_gps /dev/robot_cam /dev/camera /dev/video* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true

echo "🧹 기존 잔여 프로세스 강제 정리..."
kill_leftovers 9
sleep 2

# ==========================================================
# 🛑 [REVIEW #2] 종료 처리: 노드를 띄우기 '전에' 등록
# ==========================================================
SUP_PIDS=()
PID_MAIN=""

stop_supervisor() {
  local pid="$1"
  [ -n "${pid}" ] && kill -TERM "${pid}" 2>/dev/null || true
}

cleanup() {
  trap - SIGINT SIGTERM
  echo
  echo "🛑 종료 요청 감지. 모든 센서와 프로세스를 안전하게 종료합니다..."
  touch "${STOP_FLAG}"

  # 1) 제어 코어를 먼저 내림 -> 명령이 끊기면 ESP32 워치독(0.5초)이 모터 정지
  stop_supervisor "${PID_MAIN}"
  sleep 0.5

  # 2) 나머지 감시 엔진 종료 (각 감시 엔진이 자기 프로세스 그룹 전체를 TERM -> KILL)
  for pid in ${SUP_PIDS[@]+"${SUP_PIDS[@]}"}; do
    stop_supervisor "${pid}"
  done

  # 3) 최대 5초 대기
  for _ in $(seq 1 50); do
    local alive=0
    for pid in ${SUP_PIDS[@]+"${SUP_PIDS[@]}"}; do
      kill -0 "${pid}" 2>/dev/null && alive=1
    done
    [ "${alive}" -eq 0 ] && break
    sleep 0.1
  done

  # 4) 그래도 남은 프로세스 강제 정리 (모든 노드 포함)
  kill_leftovers 9
  rm -f "${STOP_FLAG}"
  echo "✅ 모든 프로세스 정리 완료."
  exit 0
}
trap cleanup SIGINT SIGTERM

# ==========================================================
# 🛡️ [Auto-Respawn Supervisor Engine] 자가 치유 감시 엔진
# ==========================================================
# 노드가 예외나 USB 노이즈 등으로 비정상 종료 시 1.5초 내에 자동 재실행합니다.
# [REVIEW #2] 각 노드는 setsid로 독립 프로세스 그룹에서 실행 -> 종료 시 그룹 전체(ros2 launch 자식 포함)를 정리
# [REVIEW #3] segfault(139), OOM(137) 등 시그널 크래시도 재시작. 종료 요청은 STOP_FLAG로만 판단
run_supervised() {
  local name="$1"
  local log_file="$2"
  shift 2

  (
    child_pid=""
    stop_child() {
      if [ -n "${child_pid}" ]; then
        kill -TERM -- "-${child_pid}" 2>/dev/null || kill -TERM "${child_pid}" 2>/dev/null
        for _ in $(seq 1 30); do
          kill -0 -- "-${child_pid}" 2>/dev/null || break
          sleep 0.1
        done
        kill -KILL -- "-${child_pid}" 2>/dev/null || true
      fi
      exit 0
    }
    trap stop_child SIGTERM SIGHUP

    while true; do
      [ -f "${STOP_FLAG}" ] && exit 0
      # (백그라운드 자식은 그룹 리더가 아니므로 setsid가 fork 없이 같은 PID로 새 그룹을 만듦 -> PGID == child_pid)
      setsid -w "$@" >> "${log_file}" 2>&1 < /dev/null &
      child_pid=$!
      wait "${child_pid}" 2>/dev/null
      exit_code=$?
      child_pid=""

      # 종료 요청에 의한 종료면 재시작 중단
      [ -f "${STOP_FLAG}" ] && exit 0

      echo "[$(date '+%Y-%m-%d %H:%M:%S')] ⚠️ [Supervisor] '${name}' 비정상 종료 (코드: ${exit_code})! 1.5초 후 자동 복구(Respawn)합니다..." | tee -a "${log_file}"
      sleep 1.5
    done
  ) &
  SUP_PIDS+=($!)
}

# ==========================================================
# [Layer 1] 하드웨어 드라이버 & 1차 센서/액추에이터 시리얼 I/O
# ==========================================================
# [1/11] LDLiDAR 하드웨어 드라이버 구동
# [REVIEW #17] ld19.launch.py의 port_name 기본값(/dev/ttyUSB0)이 실제 LiDAR인지 반드시 확인
echo "🔌 [1/11] STL-19P (D500) 라이다 하드웨어 구동 (감시 모드)"
run_supervised "LiDAR_HW" "${LOG_DIR}/ldlidar_hw.log" ros2 launch ldlidar_stl_ros2 ld19.launch.py
PID_LD19=$!
sleep 2

# [2/11] IMU 센서 노드
echo "⚖️  [2/11] IMU 방위각 노드 구동 (감시 모드)"
run_supervised "IMU_Node" "${LOG_DIR}/imu.log" python3 -u "${SCRIPT_DIR}/imu.py"
PID_IMU=$!

# [3/11] GPS & T-Map 전역 경로 계획 노드 (GPS 원천 노드)
echo "🛰️  [3/11] GPS & T-Map 전역 경로 계획 노드 구동 (감시 모드)"
run_supervised "GPS_Nav" "${LOG_DIR}/gps_tmap_nav.log" python3 -u "${SCRIPT_DIR}/gps_tmap_nav.py"
PID_GPS_NAV=$!

# [4/11] ESP32 모터 제어 브릿지 (하위 제어 통로 사전 확보)
echo "🔌 [4/11] ESP32 모터 제어 브릿지 구동 (감시 모드)"
run_supervised "ESP32_Bridge" "${LOG_DIR}/esp32_bridge.log" python3 -u "${SCRIPT_DIR}/esp32_bridge.py"
PID_ESP32=$!

# [5/11] YOLO AI 비전 노드
echo "📷 [5/11] YOLO 비전 노드 구동 (감시 모드)"
run_supervised "YOLO_Vision" "${LOG_DIR}/vision.log" python3 -u "${SCRIPT_DIR}/yolo26n.py"
PID_VISION=$!

sleep 1

# ==========================================================
# [Layer 2] 센서 전처리 및 위치 추정 (Localization)
# ==========================================================
# [6/11] LiDAR 180도 필터 노드
echo "👁️  [6/11] LiDAR 180도 필터 노드 구동 (감시 모드)"
run_supervised "LiDAR_Filter" "${LOG_DIR}/lidar.log" python3 -u "${SCRIPT_DIR}/lidar.py"
PID_LIDAR=$!

# [7/11] GPS/IMU 오도메트리 (GPS+IMU 준비 후 Odom 및 TF 발행)
echo "🗺️  [7/11] GPS/IMU Odom (Always-ON SLAM) 구동 (감시 모드)"
run_supervised "GPS_IMU_Odom" "${LOG_DIR}/odom.log" python3 -u "${SCRIPT_DIR}/gps_imu_odom.py"
PID_ODOM=$!

sleep 1

# ==========================================================
# [Layer 3] 실시간 환경 맵핑 (Mapping)
# ==========================================================
# [8/11] SLAM Toolbox 맵핑 코어 구동 (LiDAR 필터 + Odom TF 확보 후 구동)
if [ "${ENABLE_SLAM}" = "1" ]; then
  echo "🗺️  [8/11] SLAM Toolbox 실시간 맵핑 구동 (감시 모드, rviz2=${USE_RVIZ})"
  run_supervised "SLAM_Toolbox" "${LOG_DIR}/slam.log" ros2 launch "${SCRIPT_DIR}/slam_mapping.launch.py" slam_params_file:="${SCRIPT_DIR}/slam_toolbox_params.yaml" use_sim_time:=false use_rviz:="$([ "${USE_RVIZ}" = "1" ] && echo true || echo false)"
  PID_SLAM=$!
else
  echo "⏭️  [8/11] SLAM 비활성화 (ENABLE_SLAM=0)"
  PID_SLAM=""
fi

# ==========================================================
# [Layer 4] 의사결정 및 중앙 제어 코어 (Decision & Control)
# ==========================================================
# [9/11] 메인 융합 제어 코어 (모든 센서 및 통신 준비 완료 후 구동 -> Fault 제로)
echo "🧠 [9/11] Main Fusion 제어 코어 구동 (감시 모드)"
run_supervised "Main_Fusion" "${LOG_DIR}/main_fusion.log" python3 -u "${SCRIPT_DIR}/main_fusion.py"
PID_MAIN=$!

# ==========================================================
# [Layer 5] 통합 관제 및 진단 모니터 (GCS & Monitoring)
# ==========================================================
# [10/11] 통합 관제탑 웹 대시보드
echo "📡 [10/11] 통합 관제탑 웹 대시보드 구동 (감시 모드)"
run_supervised "Web_Dashboard" "${LOG_DIR}/web_dashboard.log" python3 -u "${SCRIPT_DIR}/web_dashboard.py"
PID_DASHBOARD=$!

# [11/11] 센서 진단 모니터
echo "📊 [11/11] 센서 상태 모니터링 데몬 구동 (감시 모드)"
run_supervised "Sensor_Monitor" "${LOG_DIR}/sensor_monitor.log" python3 -u "${SCRIPT_DIR}/sensor_monitor.py"
PID_MONITOR=$!

# [버그 수정] Snap 브라우저 권한 충돌 우회 실행
echo "🌐 웹 서버 부팅 완료 대기 중..."
for i in {1..30}; do
  if curl -s http://127.0.0.1:5000 > /dev/null; then
    echo "✅ 웹 서버 응답 확인! 권한 충돌 없는 네이티브 방식으로 브라우저를 엽니다."
    export DISPLAY=:0
    # python 내장 웹브라우저 모듈을 사용하면 snap-confine 권한 에러를 완벽히 우회합니다.
    python3 -m webbrowser "http://127.0.0.1:5000" &
    break
  fi
  sleep 0.5
done

echo
echo "=========================================================="
echo "✅ 시스템 전체 백그라운드 가동 성공"
echo "🌐 로컬 관제 UI : http://127.0.0.1:5000"
echo "🛑 프로세스 완전 종료 : Ctrl + C"
echo "⚠️  비상시에는 하드웨어 킬스위치로 모터 전원을 차단하세요"
echo "=========================================================="

tail -f "${LOG_DIR}/main_fusion.log"
cleanup