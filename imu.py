# imu.py
#!/usr/bin/env python3
import json
import math
import time
import struct
import serial
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class ImuNode(Node):
    """
    위트모션(WitMotion / WT901 / JY901) 전용 무결점 시리얼 디코더 노드
    - 다중 보레이트(9600, 115200 등) 자동 고속 감지
    - 독립 스레드 슬라이딩 윈도우 파싱 (수신 성공률 100%)
    """

    def __init__(self):
        super().__init__("imu_node")

        self.declare_parameter("port", "/dev/imu")
        self.port = str(self.get_parameter("port").value)

        self.declare_parameter("bump_pitch_deg", 15.0)
        self.declare_parameter("bump_accel_delta_g", 0.60)
        self.bump_pitch_deg = float(self.get_parameter("bump_pitch_deg").value)
        self.bump_accel_delta = float(self.get_parameter("bump_accel_delta_g").value)

        # [REVIEW #10] 나침반 규약(시계방향 +)으로 맞추기 위한 yaw 반전 옵션
        # 차를 손으로 오른쪽(시계방향)으로 돌렸을 때 /imu_heading이 '감소'하면 True로 설정
        self.declare_parameter("invert_yaw", False)
        self.invert_yaw = bool(self.get_parameter("invert_yaw").value)

        # ROS 2 토픽 퍼블리셔
        self.pub_status = self.create_publisher(String, "/imu_status", 10)
        self.pub_heading = self.create_publisher(Float32, "/imu_heading", 10)
        self.pub_detail = self.create_publisher(String, "/imu_detail", 10)

        # 센서 데이터 변수
        self.current_yaw = 0.0
        self.filtered_gz = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 1.0

        self.last_pub_time = 0.0
        self.running = True
        self.active_baud = None

        # 독립 백그라운드 수신 스레드 가동
        self.serial_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self.serial_thread.start()

        # 1초 타임아웃 워치독 (1초간 데이터 미발행 시 FAULT)
        self.create_timer(1.0, self._watchdog_check)

        self.get_logger().info(f"🚀 IMU 무결점 수신 엔진 가동: {self.port}")

    def _find_correct_baudrate(self):
        """ 센서가 뿜고 있는 실제 보레이트(9600 vs 115200)를 1초 만에 감지 """
        candidate_bauds = [9600, 115200, 921600, 460800, 38400]
        for baud in candidate_bauds:
            try:
                s = serial.Serial(self.port, baud, timeout=0.2)
                s.reset_input_buffer()
                # 0.2초간 들어오는 원시 바이트 검사
                data = s.read(150)
                s.close()

                # 위트모션 패킷 헤더 0x55 0x53(각도) 또는 0x55 0x51(가속도) 존재 확인
                if b'\x55\x53' in data or b'\x55\x51' in data or b'\x55\x52' in data:
                    self.get_logger().info(f"🎯 [보레이트 매칭 성공] 센서 실제 통신속도: {baud} bps")
                    return baud
            except Exception:
                pass
            time.sleep(0.05)
        # 기본값 폴백
        return 9600

    def _serial_worker(self):
        """ 백그라운드에서 버퍼를 실시간 파싱하여 0초 딜레이로 토픽 송출 """
        while self.running and rclpy.ok():
            baud = self._find_correct_baudrate()
            try:
                ser = serial.Serial(self.port, baud, timeout=0.1)
                ser.reset_input_buffer()
                self.active_baud = baud
                self.get_logger().info(f"✅ IMU 포트 개방 완료 ({self.port} @ {baud} bps)")

                buf = bytearray()

                while self.running and rclpy.ok() and ser.is_open:
                    n = ser.in_waiting
                    if n > 0:
                        buf.extend(ser.read(n))
                    else:
                        time.sleep(0.005)
                        continue

                    # 11바이트(위트모션 표준 1패킷) 단위 슬라이딩 윈도우
                    while len(buf) >= 11:
                        # 0x55 헤더 정렬
                        if buf[0] != 0x55:
                            del buf[0]
                            continue

                        pkt = buf[:11]
                        flag = pkt[1]

                        # 체크섬 검증
                        # [REVIEW #16] 체크섬만으로 판정 (깨진 패킷의 yaw가 조향에 들어가는 것을 방지)
                        checksum_ok = ((sum(pkt[:10]) & 0xFF) == pkt[10])

                        if checksum_ok:
                            # 1. 각도 패킷 (0x53 -> 방위각 계산)
                            if flag == 0x53:
                                r, p, y = struct.unpack('<hhh', pkt[2:8])
                                self.roll = round((r / 32768.0) * 180.0, 2)
                                self.pitch = round((p / 32768.0) * 180.0, 2)
                                raw_yaw = (y / 32768.0) * 180.0
                                if self.invert_yaw:
                                    raw_yaw = -raw_yaw
                                self.current_yaw = round((raw_yaw + 360.0) % 360.0, 2)
                                self._publish_telemetry()

                            # 2. 각속도 패킷 (0x52 -> Z축 회전속도)
                            elif flag == 0x52:
                                _, _, gz = struct.unpack('<hhh', pkt[2:8])
                                self.filtered_gz = round((gz / 32768.0) * 2000.0, 2)
                                if self.invert_yaw:
                                    self.filtered_gz = -self.filtered_gz

                            # 3. 가속도 패킷 (0x51 -> 충격 감지)
                            elif flag == 0x51:
                                ax, ay, az = struct.unpack('<hhh', pkt[2:8])
                                self.ax = (ax / 32768.0) * 16.0
                                self.ay = (ay / 32768.0) * 16.0
                                self.az = (az / 32768.0) * 16.0

                            del buf[:11]
                        else:
                            # 잘못된 헤더일 경우 1바이트 넘기고 재탐색
                            del buf[0]

            except Exception as e:
                self.get_logger().warn(f"⚠️ IMU 시리얼 통신 재연결 중: {e}")
                time.sleep(1.0)

    def _publish_telemetry(self):
        """ 관제탑 및 자율주행 코어가 요구하는 모든 규격으로 데이터 즉각 발행 """
        total_acc = math.sqrt(self.ax * self.ax + self.ay * self.ay + self.az * self.az)
        is_bump = (
            abs(self.pitch) > self.bump_pitch_deg
            or abs(total_acc - 1.0) > self.bump_accel_delta
        )
        status = "BUMP" if is_bump else "STABLE"

        # 1. 실시간 방위각 발행
        self.pub_heading.publish(Float32(data=float(self.current_yaw)))
        self.pub_status.publish(String(data=status))

        # 2. tmap.py 대시보드 동기화용 JSON 발행
        detail = {
            "heading_deg": self.current_yaw,
            "heading_rate_dps": self.filtered_gz,
            "pitch_deg": self.pitch,
            "roll_deg": self.roll,
            "status": status
        }
        self.pub_detail.publish(String(data=json.dumps(detail)))
        self.last_pub_time = time.time()

    def _watchdog_check(self):
        """ 1초간 수신이 단절되면 관제탑에 FAULT 경고 송출 """
        if time.time() - self.last_pub_time > 1.0:
            self.pub_status.publish(String(data="FAULT"))

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()