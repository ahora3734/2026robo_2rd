# esp32_bridge.py
#!/usr/bin/env python3
import json
import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.declare_parameter('port', '/dev/esp32')
        self.declare_parameter('baudrate', 115200)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.ser = None
        self.sub_cmd = self.create_subscription(String, '/vehicle_cmd', self.cmd_callback, 10)
        self.pub_status = self.create_publisher(String, '/esp32_raw_status', 10)

        self.create_timer(1.0, self.check_connection)
        self.create_timer(0.05, self.read_serial) # 0.1 -> 0.05로 응답성 상향
        self.get_logger().info(f"ESP32 Bridge 노드 가동 (포트: {self.port})")

    def connect_serial(self):
        try:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            # 블로킹 방지를 위해 timeout, write_timeout 명시
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05, write_timeout=0.05)
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
            steer = max(-100.0, min(100.0, steer))

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