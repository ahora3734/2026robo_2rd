# esp32_bridge.py
#!/usr/bin/env python3
import glob
import json
import os
import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def find_esp32_port(preferred_port="/dev/esp32", baudrate=115200):
    """
    ESP32 시리얼 포트를 자동 탐색합니다.
    1) preferred_port(/dev/esp32)가 존재하면 최우선 사용
    2) /dev/ttyUSB*, /dev/ttyACM* 중 다른 센서(/dev/rplidar, /dev/imu, /dev/ublox_gps, /dev/gps)에
       할당된 포트를 제외한 후보들을 검사하여 ESP32 텔레메트리/부팅 응답이 오는 포트를 반환
    """
    if preferred_port and os.path.exists(preferred_port):
        return preferred_port

    # 다른 센서가 사용 중인 실제 디바이스 경로 수집 (충돌 방지)
    excluded_realpaths = set()
    for symlink in ("/dev/rplidar", "/dev/imu", "/dev/ublox_gps", "/dev/gps"):
        if os.path.exists(symlink):
            excluded_realpaths.add(os.path.realpath(symlink))

    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    filtered = [c for c in candidates if os.path.realpath(c) not in excluded_realpaths]

    # 제외 후 남은 후보가 딱 1개뿐이면 프로빙 지연 없이 즉시 선택
    if len(filtered) == 1:
        return filtered[0]

    # 후보가 여러 개면 115200bps로 짧게 읽어 ESP32 텔레메트리(JSON) 확인
    for port in filtered:
        s = None
        try:
            s = serial.Serial()
            s.port = port
            s.baudrate = baudrate
            s.timeout = 0.25
            s.write_timeout = 0.1
            s.dtr = False
            s.rts = False
            s.open()
            # DTR 리셋이 걸리는 보드도 고려하여 최대 0.6초 동안 확인
            deadline = time.monotonic() + 0.6
            buf = ""
            while time.monotonic() < deadline:
                chunk = s.read(s.in_waiting or 64).decode("utf-8", errors="ignore")
                if chunk:
                    buf += chunk
                    if any(k in buf for k in ('"pot"', '"steer_cur"', "[ESP32]", "ets Jul", "rst:0x")):
                        s.close()
                        return port
            s.close()
        except Exception:
            if s and s.is_open:
                try:
                    s.close()
                except Exception:
                    pass

    return filtered[0] if filtered else preferred_port


class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.declare_parameter('port', '/dev/esp32')
        self.declare_parameter('baudrate', 115200)

        self.preferred_port = self.get_parameter('port').get_parameter_value().string_value
        self.port = self.preferred_port
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.ser = None
        self.sub_cmd = self.create_subscription(String, '/vehicle_cmd', self.cmd_callback, 10)
        self.pub_status = self.create_publisher(String, '/esp32_raw_status', 10)

        self.create_timer(1.0, self.check_connection)
        self.create_timer(0.05, self.read_serial) # 0.1 -> 0.05로 응답성 상향
        self.get_logger().info(f"ESP32 Bridge 노드 가동 (기본 포트: {self.preferred_port}, 자동 탐색 지원)")

    def connect_serial(self):
        try:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass

            target_port = find_esp32_port(self.preferred_port, self.baudrate)
            self.port = target_port

            # 블로킹 방지를 위해 timeout, write_timeout 명시
            # [BUG FIX] write_timeout 0.05 → 0.1 : ESP32 처리 지연 시 SerialTimeoutException 방지
            s = serial.Serial()
            s.port = self.port
            s.baudrate = self.baudrate
            s.timeout = 0.05
            s.write_timeout = 0.1
            s.dtr = False
            s.rts = False
            s.open()
            self.ser = s
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"✅ ESP32 시리얼 연결 성공: {self.port}")
            return True
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f"⚠️ ESP32 시리얼 연결 대기 중 ({self.port}): {e}")
            return False

    def check_connection(self):
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()

    def cmd_callback(self, msg):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            data = json.loads(msg.data)
            pwm = int(round(float(data.get('pwm', 0))))
            steer = float(data.get('steer', 0.0))
            steer_hold = bool(data.get('steer_hold', False))

            pwm = max(-255, min(255, pwm))
            # [BUG FIX] 브릿지 조향 클램프를 펌웨어 MAX_STEER_ANGLE_DEG(35°)와 일치시킴
            # 기존 ±100 → ±35 : main_fusion에서 이미 클램프하지만, 방어적으로 여기서도 동일하게 맞춤
            steer = max(-35.0, min(35.0, steer))

            # 하드웨어는 음수 PWM이 전진 (drive.py와 동일하게 부호 반전, 양수 = 전진은 main_fusion 규약)
            pwm = -pwm

            # 정지 조향 요청 시에만 세 번째 필드(HOLD=1) 추가 (펌웨어는 두 형식 모두 지원)
            if steer_hold:
                command_str = f"<{pwm},{steer:.2f},1>\n"
            else:
                command_str = f"<{pwm},{steer:.2f}>\n"
            self.ser.write(command_str.encode('utf-8'))
        except json.JSONDecodeError:
            self.get_logger().error(f"JSON 파싱 에러: {msg.data[:80]}")
        except serial.SerialTimeoutException:
            self.get_logger().error("❌ ESP32 송신 타임아웃")
        except Exception as e:
            self.get_logger().error(f"❌ ESP32 데이터 전송 실패 (재연결 시도): {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def read_serial(self):
        if self.ser and self.ser.is_open:
            try:
                # 쌓인 줄을 모두 읽어 텔레메트리 지연 누적 방지 (steer_cur는 정지 조향 완료 판정에 쓰임)
                for _ in range(20):
                    if self.ser.in_waiting <= 0:
                        break
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        msg = String()
                        msg.data = line
                        self.pub_status.publish(msg)
            except Exception as e:
                self.get_logger().error(f"❌ ESP32 읽기 에러: {e}")
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                self.ser = None

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            try:
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()