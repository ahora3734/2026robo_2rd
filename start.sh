# start.sh
#!/usr/bin/env bash
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/logs"
mkdir -p "${LOG_DIR}"

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
sudo chmod 666 /dev/esp32 /dev/rplidar /dev/imu /dev/ublox_gps /dev/camera /dev/video* /dev/ttyUSB* 2>/dev/null || true

echo "🧹 기존 잔여 프로세스 강제 정리..."
fuser -k -9 5000/tcp 2>/dev/null || true
pkill -9 -f "python3.*lidar.py" 2>/dev/null || true
pkill -9 -f "python3.*imu.py" 2>/dev/null || true
pkill -9 -f "python3.*yolo26n.py" 2>/dev/null || true
pkill -9 -f "python3.*tmap.py" 2>/dev/null || true
pkill -9 -f "python3.*main_fusion.py" 2>/dev/null || true
pkill -9 -f "python3.*gps_imu_odom.py" 2>/dev/null || true
pkill -9 -f "python3.*esp32_bridge.py" 2>/dev/null || true
pkill -9 -f "python3.*sensor_monitor.py" 2>/dev/null || true
pkill -9 -f "slam_toolbox" 2>/dev/null || true
pkill -9 -f "ldlidar" 2>/dev/null || true
sleep 2

# [1/9] LDLiDAR 하드웨어 드라이버 구동
echo "🔌 [1/9] STL-19P (D500) 라이다 하드웨어 구동"
ros2 launch ldlidar_stl_ros2 ld19.launch.py > "${LOG_DIR}/ldlidar_hw.log" 2>&1 &
PID_LD19=$!
sleep 2

# [2/9] LiDAR 필터 노드
echo "👁️  [2/9] LiDAR 180도 필터 노드 구동"
python3 -u "${SCRIPT_DIR}/lidar.py" > "${LOG_DIR}/lidar.log" 2>&1 &
PID_LIDAR=$!

# [3/9] IMU 노드
echo "⚖️  [3/9] IMU 방위각 노드 구동"
python3 -u "${SCRIPT_DIR}/imu.py" > "${LOG_DIR}/imu.log" 2>&1 &
PID_IMU=$!

# [4/9] YOLO AI 비전
echo "📷 [4/9] YOLO 비전 노드 구동"
python3 -u "${SCRIPT_DIR}/yolo26n.py" > "${LOG_DIR}/vision.log" 2>&1 &
PID_VISION=$!

sleep 1

# [5/9] GPS/IMU 오도메트리
echo "🗺️  [5/9] GPS/IMU Odom (Always-ON SLAM) 구동"
python3 -u "${SCRIPT_DIR}/gps_imu_odom.py" > "${LOG_DIR}/odom.log" 2>&1 &
PID_ODOM=$!

# [6/9] 메인 융합 제어 코어
echo "🧠 [6/9] Main Fusion 제어 코어 구동"
python3 -u "${SCRIPT_DIR}/main_fusion.py" > "${LOG_DIR}/main_fusion.log" 2>&1 &
PID_MAIN=$!

# [7/9] TMAP 웹 관제 서버
echo "📡 [7/9] TMAP 통합 관제탑 구동"
python3 -u "${SCRIPT_DIR}/tmap.py" > "${LOG_DIR}/tmap.log" 2>&1 &
PID_TMAP=$!

# [8/9] ESP32 모터 제어 브릿지
echo "🔌 [8/9] ESP32 모터 제어 브릿지 구동"
python3 -u "${SCRIPT_DIR}/esp32_bridge.py" > "${LOG_DIR}/esp32_bridge.log" 2>&1 &
PID_ESP32=$!

# [9/9] 센서 진단 모니터
echo "📊 [9/9] 센서 상태 모니터링 데몬 구동"
python3 -u "${SCRIPT_DIR}/sensor_monitor.py" > "${LOG_DIR}/sensor_monitor.log" 2>&1 &
PID_MONITOR=$!

# SLAM Toolbox 맵핑 코어 구동
echo "🗺️  SLAM Toolbox 실시간 맵핑 구동"
(
  cd "${SCRIPT_DIR}"
  ros2 launch "${SCRIPT_DIR}/slam_mapping.launch.py" slam_params_file:="${SCRIPT_DIR}/slam_toolbox_params.yaml" use_sim_time:=false
) > "${LOG_DIR}/slam.log" 2>&1 &
PID_SLAM=$!

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
echo "=========================================================="

cleanup() {
  echo
  echo "🛑 Ctrl+C 입력 감지. 모든 센서와 프로세스를 안전하게 종료합니다..."
  kill -9 "${PID_LD19}" "${PID_LIDAR}" "${PID_IMU}" "${PID_VISION}" \
          "${PID_ODOM}" "${PID_MAIN}" "${PID_TMAP}" "${PID_ESP32}" \
          "${PID_MONITOR}" "${PID_SLAM}" 2>/dev/null || true
  fuser -k -9 5000/tcp 2>/dev/null || true
  pkill -9 -f "python3.*tmap.py" 2>/dev/null || true
  pkill -9 -f "slam_toolbox" 2>/dev/null || true
  pkill -9 -f "ldlidar" 2>/dev/null || true
  wait 2>/dev/null || true
  echo "✅ 모든 프로세스 정리 완료."
  exit 0
}
trap cleanup SIGINT SIGTERM

tail -f "${LOG_DIR}/main_fusion.log"