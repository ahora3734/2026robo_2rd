# lidar.py
#!/usr/bin/env python3
import math
import statistics
import time
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

        # [REVIEW #14] 전방 판정을 부채꼴 대신 차폭 직사각형 통로로 수행
        self.declare_parameter("vehicle_width_m", 0.9)
        self.declare_parameter("corridor_margin_m", 0.2)
        self.declare_parameter("corridor_length_m", 6.0)
        self.declare_parameter("front_min_rank", 3)   # n번째로 가까운 값 사용 (단일 노이즈 점 무시)
        self.corridor_half = (float(self.get_parameter("vehicle_width_m").value) / 2.0
                              + float(self.get_parameter("corridor_margin_m").value))
        self.corridor_len = float(self.get_parameter("corridor_length_m").value)
        self.front_rank = max(1, int(self.get_parameter("front_min_rank").value))

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

                    # 차량 기준 x(전방), y(좌측) 좌표로 통로 내부 점만 수집
                    x = raw * math.cos(angle)
                    y = raw * math.sin(angle)
                    if 0.0 < x <= self.corridor_len and abs(y) <= self.corridor_half:
                        corridor.append(x)
            angle += inc

        # 전방 거리 = 차폭 통로 안의 가장 가까운 장애물까지의 전방 거리
        front = self.near_min_distance(corridor) if corridor else self.max_valid
        left_d = self.robust_distance(left)
        right_d = self.robust_distance(right)

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

        self.pub_obstacle.publish(String(data=(f"status:{status},front:{front:.2f},left:{left_d:.2f},right:{right_d:.2f}")))

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