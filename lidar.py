# lidar.py
#!/usr/bin/env python3
import math
import statistics
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class LidarNode(Node):
    def __init__(self):
        super().__init__("lidar_node")
        self.declare_parameter("safe_dist_center", 1.2)
        self.declare_parameter("danger_dist_center", 0.5)
        self.declare_parameter("safe_dist_side", 0.8)
        self.declare_parameter("curb_dist_threshold", 0.4)
        self.declare_parameter("min_valid_dist", 0.18)
        self.declare_parameter("max_valid_dist", 12.0)
        self.declare_parameter("center_half_angle_deg", 20.0)
        self.declare_parameter("publish_filtered_scan", True)

        self.safe_center = float(self.get_parameter("safe_dist_center").value)
        self.danger_center = float(self.get_parameter("danger_dist_center").value)
        self.safe_side = float(self.get_parameter("safe_dist_side").value)
        self.curb_thresh = float(self.get_parameter("curb_dist_threshold").value)
        self.min_valid = float(self.get_parameter("min_valid_dist").value)
        self.max_valid = float(self.get_parameter("max_valid_dist").value)
        self.center_half = float(self.get_parameter("center_half_angle_deg").value)
        self.publish_filtered = bool(self.get_parameter("publish_filtered_scan").value)

        # [차량 실측 치수] (m)
        self.declare_parameter("vehicle_width_m", 0.90)
        self.declare_parameter("vehicle_length_m", 1.54)
        self.declare_parameter("wheelbase_m", 0.96)
        self.declare_parameter("front_overhang_m", 0.20)        # 앞 범퍼 ~ 앞바퀴 축 (실측 200mm)
        self.declare_parameter("lidar_to_front_bumper_m", 0.08) # 라이다가 앞 범퍼보다 뒤에 있는 거리
        self.declare_parameter("lidar_left_offset_m", 0.0)      # 라이다가 중심선보다 왼쪽에 있는 거리
        self.declare_parameter("max_steer_deg", 35.0)
        width = float(self.get_parameter("vehicle_width_m").value)
        length = float(self.get_parameter("vehicle_length_m").value)
        wheelbase = float(self.get_parameter("wheelbase_m").value)
        front_overhang = float(self.get_parameter("front_overhang_m").value)
        self.lidar_to_bumper = float(self.get_parameter("lidar_to_front_bumper_m").value)
        self.lidar_left_offset = float(self.get_parameter("lidar_left_offset_m").value)
        max_steer = math.radians(float(self.get_parameter("max_steer_deg").value))

        # [REVIEW #14] 전방 판정을 부채꼴 대신 차폭 직사각형 통로로 수행
        # front 값은 라이다가 아니라 '앞 범퍼' 기준 거리로 발행 (main_fusion 정지 거리도 범퍼 기준)
        self.declare_parameter("corridor_margin_m", 0.2)
        self.declare_parameter("corridor_length_m", 6.0)
        self.declare_parameter("front_min_rank", 3)   # n번째로 가까운 값 사용 (단일 노이즈 점 무시)
        self.corridor_half = width / 2.0 + float(self.get_parameter("corridor_margin_m").value)
        self.corridor_len = float(self.get_parameter("corridor_length_m").value)
        self.front_rank = max(1, int(self.get_parameter("front_min_rank").value))

        # [회피 궤적 검사] 핸들을 최대로 꺾고 전진할 때 차체(직사각형)가 쓸고 지나가는 영역에
        # 라이다 점이 있는지 검사 -> 좌/우 각각 '부딪히기 전까지 갈 수 있는 거리(arc_left/right)' 발행
        self.declare_parameter("arc_check_length_m", 2.5)
        self.declare_parameter("arc_check_step_m", 0.1)
        self.declare_parameter("arc_margin_m", 0.10)
        self.arc_len = float(self.get_parameter("arc_check_length_m").value)
        self.arc_step = max(0.02, float(self.get_parameter("arc_check_step_m").value))
        margin = float(self.get_parameter("arc_margin_m").value)

        # 좌표계: 뒷바퀴 축 중심 원점, x 전방, y 좌측
        bumper_x = wheelbase + front_overhang
        self.lidar_x = bumper_x - self.lidar_to_bumper
        self.fp_x_min = -(length - bumper_x) - margin   # 뒤 범퍼
        self.fp_x_max = bumper_x + margin               # 앞 범퍼
        self.fp_y_half = width / 2.0 + margin
        self.arc_point_range = self.arc_len + length    # 검사에 쓰는 라이다 점 최대 거리

        turn_radius = wheelbase / math.tan(max_steer)
        self.arc_poses = {1: [], -1: []}  # 1: 좌회전, -1: 우회전
        n_steps = int(round(self.arc_len / self.arc_step))
        for k in range(1, n_steps + 1):
            s = k * self.arc_step
            th = s / turn_radius
            for d in (1, -1):
                psi = d * th
                self.arc_poses[d].append((s, turn_radius * math.sin(th), d * turn_radius * (1.0 - math.cos(th)),
                                          math.cos(psi), math.sin(psi)))
        self.get_logger().info(f"🚗 최소 회전반경 {turn_radius:.2f} m (휠베이스 {wheelbase:.2f} m, 최대 조향 {math.degrees(max_steer):.0f}°)")

        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
        self.pub_obstacle = self.create_publisher(String, "/obstacle_info", 10)

        self.pub_filtered = None
        if self.publish_filtered:
            self.pub_filtered = self.create_publisher(LaserScan, "/scan_filtered", 10)

        self.get_logger().info("✅ 180도 대응 LiDAR 프로세서 가동 (/scan -> /scan_filtered)")

    def robust_distance(self, values):
        valid = [float(v) for v in values if math.isfinite(v) and self.min_valid <= float(v) <= self.max_valid]
        if not valid:
            return self.max_valid
        if len(valid) < 5:
            return min(valid)
        valid.sort()
        lo = int(len(valid) * 0.10)
        hi = max(lo + 1, int(len(valid) * 0.35))
        return float(statistics.median(valid[lo:hi]))

    def near_min_distance(self, values):
        """[REVIEW #14] 가는 물체(다리, 기둥)를 지우지 않도록 최소값 계열 사용"""
        if not values:
            return self.max_valid
        valid = sorted(values)
        return float(valid[min(self.front_rank, len(valid)) - 1])

    def arc_free_distance(self, px, py, direction):
        """최대 조향으로 전진할 때 차체가 처음 라이다 점에 닿기 전까지의 주행 거리 (m)"""
        if px.size == 0:
            return self.arc_len
        for s, x0, y0, c, sn in self.arc_poses[direction]:
            dx = px - x0
            dy = py - y0
            lx = c * dx + sn * dy
            ly = -sn * dx + c * dy
            hit = (lx >= self.fp_x_min) & (lx <= self.fp_x_max) & (np.abs(ly) <= self.fp_y_half)
            if hit.any():
                return max(0.0, s - self.arc_step)
        return self.arc_len

    def make_filtered_scan(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = max(msg.range_min, self.min_valid)
        out.range_max = min(msg.range_max, self.max_valid)

        out.ranges = []
        for raw in msg.ranges:
            if math.isfinite(raw) and self.min_valid <= raw <= self.max_valid:
                out.ranges.append(float(raw))
            else:
                out.ranges.append(float("nan"))

        if hasattr(msg, "intensities"):
            out.intensities = list(msg.intensities)
        return out

    def scan_callback(self, msg: LaserScan):
        left, center, right = [], [], []
        corridor = []
        arc_x, arc_y = [], []
        angle = msg.angle_min
        inc = msg.angle_increment if msg.angle_increment > 0 else 0.005

        for raw in msg.ranges:
            if math.isfinite(raw) and self.min_valid <= raw <= self.max_valid:
                deg = math.degrees(angle)
                deg = (deg + 180.0) % 360.0 - 180.0

                if -90.0 <= deg <= 90.0:
                    if -self.center_half <= deg <= self.center_half:
                        center.append(raw)
                    elif self.center_half < deg <= 90.0:
                        left.append(raw)
                    else:
                        right.append(raw)

                    # 차량 기준 x(전방), y(좌측, 차 중심선 기준) 좌표
                    x = raw * math.cos(angle)
                    y = raw * math.sin(angle) + self.lidar_left_offset
                    # 통로 내부 점만 수집 (x는 앞 범퍼 기준 거리)
                    x_bumper = x - self.lidar_to_bumper
                    if 0.0 < x_bumper <= self.corridor_len and abs(y) <= self.corridor_half:
                        corridor.append(x_bumper)
                    # 회피 궤적 검사용 점 (뒷바퀴 축 원점 좌표로 변환)
                    if raw <= self.arc_point_range:
                        arc_x.append(x + self.lidar_x)
                        arc_y.append(y)
            angle += inc

        # 전방 거리 = 차폭 통로 안의 가장 가까운 장애물까지 앞 범퍼로부터의 거리
        front = self.near_min_distance(corridor) if corridor else self.corridor_len
        left_d = self.robust_distance(left)
        right_d = self.robust_distance(right)

        # 회피 궤적 검사: 지금 차체와 이미 겹친 점(차체/브래킷 반사 등)은 제외
        px = np.asarray(arc_x, dtype=np.float64)
        py = np.asarray(arc_y, dtype=np.float64)
        if px.size:
            inside = (px >= self.fp_x_min) & (px <= self.fp_x_max) & (np.abs(py) <= self.fp_y_half)
            px, py = px[~inside], py[~inside]
        arc_left = self.arc_free_distance(px, py, 1)
        arc_right = self.arc_free_distance(px, py, -1)

        if front <= self.danger_center:
            status = "danger"
        elif front < self.safe_center or left_d < self.safe_side or right_d < self.safe_side:
            status = "warning"
        else:
            status = "clear"

        if left_d <= self.curb_thresh:
            status += "_curb_left"
        if right_d <= self.curb_thresh:
            status += "_curb_right"

        self.pub_obstacle.publish(String(data=(
            f"status:{status},front:{front:.2f},left:{left_d:.2f},right:{right_d:.2f},"
            f"arc_left:{arc_left:.2f},arc_right:{arc_right:.2f}")))

        if self.pub_filtered is not None:
            self.pub_filtered.publish(self.make_filtered_scan(msg))

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
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