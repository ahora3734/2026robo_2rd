# main_fusion.py
#!/usr/bin/env python3
import json
import math
import time
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

@dataclass
class GeoPoint:
    lat: float
    lon: float

class MainFusionNode(Node):
    def __init__(self):
        super().__init__("main_fusion_node")
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("max_pwm", 180)
        self.declare_parameter("corner_pwm", 110)
        self.declare_parameter("near_object_pwm", 80)
        self.declare_parameter("slow_obstacle_pwm", 60)
        self.declare_parameter("slow_obstacle_dist", 2.2)
        self.declare_parameter("stop_obstacle_dist", 0.8)
        self.declare_parameter("emergency_obstacle_dist", 0.5)
        self.declare_parameter("lookahead_min_m", 2.0)
        self.declare_parameter("lookahead_max_m", 5.0)
        self.declare_parameter("lookahead_speed_gain", 0.015)
        self.declare_parameter("heading_gain", 0.9)
        self.declare_parameter("cross_track_gain", 5.0)
        self.declare_parameter("max_steer", 100.0)
        self.declare_parameter("steer_filter_alpha", 0.18)
        self.declare_parameter("avoid_steer", 35.0)
        self.declare_parameter("avoid_min_duration_sec", 1.2)
        self.declare_parameter("rejoin_distance_m", 1.5)
        self.declare_parameter("arrival_distance_m", 2.5)
        self.declare_parameter("lidar_timeout_sec", 1.0)
        self.declare_parameter("imu_timeout_sec", 1.0)
        self.declare_parameter("gps_timeout_sec", 2.0)
        self.declare_parameter("vision_timeout_sec", 1.5)
        self.declare_parameter("gps_course_min_move_m", 2.0)
        self.declare_parameter("gps_course_alpha", 0.15)

        self.control_hz = float(self.get_parameter("control_hz").value)
        self.max_pwm = self.clamp_pwm(self.get_parameter("max_pwm").value)
        self.corner_pwm = self.clamp_pwm(self.get_parameter("corner_pwm").value)
        self.near_object_pwm = self.clamp_pwm(self.get_parameter("near_object_pwm").value)
        self.slow_obstacle_pwm = self.clamp_pwm(self.get_parameter("slow_obstacle_pwm").value)
        self.slow_obstacle_dist = float(self.get_parameter("slow_obstacle_dist").value)
        self.stop_obstacle_dist = float(self.get_parameter("stop_obstacle_dist").value)
        self.emergency_obstacle_dist = float(self.get_parameter("emergency_obstacle_dist").value)
        self.lookahead_min = float(self.get_parameter("lookahead_min_m").value)
        self.lookahead_max = float(self.get_parameter("lookahead_max_m").value)
        self.lookahead_speed_gain = float(self.get_parameter("lookahead_speed_gain").value)
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.cross_track_gain = float(self.get_parameter("cross_track_gain").value)
        self.max_steer = float(self.get_parameter("max_steer").value)
        self.steer_alpha = float(self.get_parameter("steer_filter_alpha").value)
        self.avoid_steer = float(self.get_parameter("avoid_steer").value)
        self.avoid_min_duration = float(self.get_parameter("avoid_min_duration_sec").value)
        self.rejoin_distance = float(self.get_parameter("rejoin_distance_m").value)
        self.arrival_distance = float(self.get_parameter("arrival_distance_m").value)
        self.lidar_timeout = float(self.get_parameter("lidar_timeout_sec").value)
        self.imu_timeout = float(self.get_parameter("imu_timeout_sec").value)
        self.gps_timeout = float(self.get_parameter("gps_timeout_sec").value)
        self.vision_timeout = float(self.get_parameter("vision_timeout_sec").value)
        self.gps_course_min_move = float(self.get_parameter("gps_course_min_move_m").value)
        self.gps_course_alpha = float(self.get_parameter("gps_course_alpha").value)

        self.create_subscription(String, "/obstacle_info", self.lidar_callback, 10)
        self.create_subscription(String, "/imu_status", self.imu_status_callback, 10)
        self.create_subscription(Float32, "/imu_heading", self.heading_callback, 10)
        self.create_subscription(String, "/imu_detail", self.imu_detail_callback, 10)
        self.create_subscription(String, "/gps_pos", self.gps_callback, 10)
        self.create_subscription(String, "/nav_path_data", self.nav_path_callback, 10)
        self.create_subscription(String, "/vision_info", self.vision_callback, 10)

        self.pub_cmd = self.create_publisher(String, "/vehicle_cmd", 10)
        self.pub_status = self.create_publisher(String, "/vehicle_status", 10)

        now = time.monotonic()
        self.last_lidar = 0.0
        self.last_imu = 0.0
        self.last_gps = 0.0
        self.last_vision = now

        self.imu_heading = 0.0
        self.imu_rate = 0.0
        self.imu_status = "STABLE"

        self.cur_lat = None
        self.cur_lon = None
        self.prev_gps = None
        self.gps_course = None
        self.heading_offset = 0.0

        self.path = []
        self.target_idx = 0

        self.lidar_front = 10.0
        self.lidar_left = 10.0
        self.lidar_right = 10.0
        self.lidar_status = "clear"

        self.vision_status = "CLEAR"
        self.state = "WAITING_FOR_NAV"
        self.avoid_direction = 0
        self.avoid_start = 0.0

        self.current_pwm = 0.0
        self.filtered_steer = 0.0

        self.create_timer(max(0.01, 1.0 / self.control_hz), self.control_loop)

    @staticmethod
    def clamp_pwm(value):
        try:
            return max(0, min(255, int(round(float(value)))))
        except Exception:
            return 0

    @staticmethod
    def wrap180(angle):
        return (angle + 180.0) % 360.0 - 180.0

    @staticmethod
    def distance(a: GeoPoint, b: GeoPoint):
        R = 6371000.0
        p1 = math.radians(a.lat)
        p2 = math.radians(b.lat)
        dp = math.radians(b.lat - a.lat)
        dl = math.radians(b.lon - a.lon)
        h = (math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2)
        return (R * 2.0 * math.atan2(math.sqrt(h), math.sqrt(1.0 - h)))

    @staticmethod
    def bearing(a: GeoPoint, b: GeoPoint):
        r1 = math.radians(a.lat)
        r2 = math.radians(b.lat)
        dl = math.radians(b.lon - a.lon)
        x = math.sin(dl) * math.cos(r2)
        y = (math.cos(r1) * math.sin(r2) - math.sin(r1) * math.cos(r2) * math.cos(dl))
        return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0

    @staticmethod
    def local_xy(ref: GeoPoint, p: GeoPoint):
        lat_m = 111320.0
        lon_m = 111320.0 * math.cos(math.radians(ref.lat))
        return ((p.lon - ref.lon) * lon_m, (p.lat - ref.lat) * lat_m)

    def segment_cte(self, p, a, b):
        ax, ay = self.local_xy(p, a)
        bx, by = self.local_xy(p, b)
        denom = bx * bx + by * by
        if denom < 1e-9:
            return math.hypot(ax, ay), 0.0, 0.0
        vx, vy = bx - ax, by - ay
        wx, wy = -ax, -ay
        t = (wx * vx + wy * vy) / denom
        t = max(0.0, min(1.0, t))
        cx = ax + t * vx
        cy = ay + t * vy
        cte = math.hypot(cx, cy)
        cross = vx * (-ay) - vy * (-ax)
        sign = 1.0 if cross >= 0 else -1.0
        return cte, sign, t

    def closest_path(self, p):
        if len(self.path) < 2:
            return float("inf"), 0, 0.0, 0.0
        start = max(0, min(self.target_idx, len(self.path) - 2))
        best = (float("inf"), start, 0.0, 0.0)
        for i in range(start, len(self.path) - 1):
            a = GeoPoint(self.path[i][0], self.path[i][1])
            b = GeoPoint(self.path[i + 1][0], self.path[i + 1][1])
            d, sign, t = self.segment_cte(p, a, b)
            if d < best[0]:
                best = (d, i, sign, t)
        return best

    def update_target_progress(self, p):
        while self.target_idx < len(self.path) - 1:
            wp = GeoPoint(self.path[self.target_idx][0], self.path[self.target_idx][1])
            if self.distance(p, wp) < 3.0:
                self.target_idx += 1
            else:
                break

    def lookahead_target(self):
        if not self.path or self.cur_lat is None:
            return None
        p = GeoPoint(self.cur_lat, self.cur_lon)
        lookahead = max(self.lookahead_min, min(self.lookahead_max, self.lookahead_min + self.current_pwm * self.lookahead_speed_gain))
        total = 0.0
        prev = p
        for i in range(self.target_idx, len(self.path)):
            wp = GeoPoint(self.path[i][0], self.path[i][1])
            total += self.distance(prev, wp)
            if total >= lookahead:
                self.target_idx = min(i, len(self.path) - 1)
                return wp
            prev = wp
        final = self.path[-1]
        return GeoPoint(final[0], final[1])

    def lidar_callback(self, msg):
        self.last_lidar = time.monotonic()
        try:
            parts = {}
            for item in msg.data.split(","):
                if ":" in item:
                    k, v = item.split(":", 1)
                    parts[k.strip()] = v.strip()
            self.lidar_status = parts.get("status", "clear")
            def num(key, default):
                try:
                    x = float(parts.get(key, default))
                    return x if math.isfinite(x) else default
                except Exception:
                    return default
            self.lidar_front = num("front", 10.0)
            self.lidar_left = num("left", 10.0)
            self.lidar_right = num("right", 10.0)
        except Exception:
            self.lidar_status = "invalid"

    def imu_status_callback(self, msg):
        self.last_imu = time.monotonic()
        self.imu_status = msg.data.strip() or "STABLE"

    def heading_callback(self, msg):
        self.last_imu = time.monotonic()
        try:
            x = float(msg.data)
            if math.isfinite(x):
                self.imu_heading = x % 360.0
        except Exception:
            pass

    def imu_detail_callback(self, msg):
        try:
            text = msg.data.replace("'", '"')
            obj = json.loads(text)
            rate = float(obj.get("heading_rate_dps", 0.0))
            if math.isfinite(rate):
                self.imu_rate = rate
        except Exception:
            pass

    def gps_callback(self, msg):
        self.last_gps = time.monotonic()
        try:
            lat, lon = map(float, msg.data.split(","))
            if not (math.isfinite(lat) and math.isfinite(lon)):
                return
            current = GeoPoint(lat, lon)
            if self.prev_gps is not None:
                moved = self.distance(self.prev_gps, current)
                if moved >= self.gps_course_min_move:
                    course = self.bearing(self.prev_gps, current)
                    if self.gps_course is None:
                        self.gps_course = course
                    else:
                        error = self.wrap180(course - self.gps_course)
                        self.gps_course = (self.gps_course + self.gps_course_alpha * error) % 360.0
                    error = self.wrap180(self.gps_course - self.imu_heading)
                    self.heading_offset = (self.heading_offset + 0.05 * error) % 360.0
            self.prev_gps = current
            self.cur_lat = lat
            self.cur_lon = lon
        except Exception:
            pass

    @property
    def navigation_heading(self):
        return (self.imu_heading + self.heading_offset) % 360.0

    def nav_path_callback(self, msg):
        try:
            data = json.loads(msg.data)
            raw = data.get("path", [])
            path = []
            for p in raw:
                if len(p) < 2: continue
                lat, lon = float(p[0]), float(p[1])
                if math.isfinite(lat) and math.isfinite(lon):
                    path.append([lat, lon])
            self.path = path
            requested_idx = int(data.get("target_idx", 0))
            self.target_idx = max(0, min(requested_idx, max(0, len(self.path) - 1)))
            if self.path:
                self.state = "NAVIGATE"
            else:
                self.state = "WAITING_FOR_NAV"
        except Exception:
            pass

    def vision_callback(self, msg):
        self.last_vision = time.monotonic()
        self.vision_status = (msg.data.strip() or "CLEAR")

    def sensor_faults(self):
        now = time.monotonic()
        faults = []
        if self.last_lidar <= 0.0 or now - self.last_lidar > self.lidar_timeout: faults.append("LiDAR")
        if self.last_imu <= 0.0 or now - self.last_imu > self.imu_timeout: faults.append("IMU")
        if self.last_gps <= 0.0 or now - self.last_gps > self.gps_timeout: faults.append("GPS")
        if now - self.last_vision > self.vision_timeout: faults.append("CAM")
        return faults

    def choose_avoid_direction(self):
        return 1 if self.lidar_right >= self.lidar_left else -1

    def path_steer(self):
        if not self.path or self.cur_lat is None:
            return 0.0
        p = GeoPoint(self.cur_lat, self.cur_lon)
        self.update_target_progress(p)
        target = self.lookahead_target()
        if target is None:
            return 0.0
        target_bearing = self.bearing(p, target)
        heading_error = self.wrap180(target_bearing - self.navigation_heading)
        cte, side_sign, _, = self.closest_path(p)
        signed_cte = cte * side_sign
        steer = (self.heading_gain * heading_error - self.cross_track_gain * signed_cte)
        return max(-self.max_steer, min(self.max_steer, steer))

    def ramp_pwm(self, target):
        target = float(self.clamp_pwm(target))
        step = 4.0
        if abs(self.current_pwm - target) <= step:
            self.current_pwm = target
        elif self.current_pwm < target:
            self.current_pwm += step
        else:
            self.current_pwm -= step
        self.current_pwm = max(0.0, min(255.0, self.current_pwm))
        return int(round(self.current_pwm))

    def obstacle_override(self, now):
        if self.lidar_front <= self.emergency_obstacle_dist:
            self.state = "E_STOP"
            return 0, 0.0, "E-STOP"
        if self.vision_status == "PERSON_FRONT":
            self.state = "STOP"
            return 0, 0.0, "보행자 정지"
        if self.lidar_front <= self.stop_obstacle_dist:
            self.state = "STOP"
            return 0, 0.0, "근접 장애물 정지"
        if self.state in ("STOP", "E_STOP"):
            if self.lidar_front > self.stop_obstacle_dist + 0.25 and self.vision_status != "PERSON_FRONT":
                self.state = "NAVIGATE"
            else:
                return 0, 0.0, "정지 유지"
        if self.lidar_front <= self.slow_obstacle_dist and self.state == "NAVIGATE":
            self.avoid_direction = self.avoid_direction or self.choose_avoid_direction()
            self.avoid_start = now
            self.state = "AVOID_RIGHT" if self.avoid_direction > 0 else "AVOID_LEFT"
        if self.state in ("AVOID_LEFT", "AVOID_RIGHT"):
            elapsed = now - self.avoid_start
            steer = self.avoid_steer if self.state == "AVOID_RIGHT" else -self.avoid_steer
            if elapsed >= self.avoid_min_duration and self.lidar_front > self.slow_obstacle_dist + 0.4:
                self.state = "REJOIN_PATH"
            return self.slow_obstacle_pwm, steer, "장애물 회피"
        if self.state == "REJOIN_PATH":
            p = GeoPoint(self.cur_lat, self.cur_lon)
            cte, _, _, = self.closest_path(p)
            if cte <= self.rejoin_distance and self.lidar_front > self.slow_obstacle_dist + 0.4:
                self.avoid_direction = 0
                self.state = "NAVIGATE"
            return self.corner_pwm, self.path_steer(), "경로 복귀"
        return None

    def publish_state(self, speed_pwm, steer, mode):
        self.filtered_steer = (self.steer_alpha * steer + (1.0 - self.steer_alpha) * self.filtered_steer)
        cmd_pwm = int(max(0, min(255, round(speed_pwm))))
        
        cmd = {
            "pwm": cmd_pwm,
            "steer": round(float(self.filtered_steer), 2),
            "mode": mode,
            "state": self.state,
        }
        self.pub_cmd.publish(String(data=json.dumps(cmd, ensure_ascii=False)))

        cte = None
        target_bearing = None
        if self.cur_lat is not None and self.cur_lon is not None and self.path:
            p = GeoPoint(self.cur_lat, self.cur_lon)
            cte, _, _, = self.closest_path(p)
            target = self.lookahead_target()
            if target:
                target_bearing = self.bearing(p, target)

        # UI와 브릿지 모두 만족하도록 key 병합 적용
        status = {
            "speed_pwm": cmd_pwm,
            "pwm": cmd_pwm,
            "max_pwm": 255,
            "steer": round(float(self.filtered_steer), 1),
            "state": self.state,
            "fusion_state": self.state,
            "status_msg": mode,
            "imu_heading": round(self.navigation_heading, 1),
            "imu_rate_dps": round(self.imu_rate, 2),
            "gps_course": None if self.gps_course is None else round(self.gps_course, 1),
            "heading_offset": round(self.heading_offset, 1),
            "cross_track_m": None if cte is None else round(cte, 2),
            "target_bearing": None if target_bearing is None else round(target_bearing, 1),
            "lidar_front": round(self.lidar_front, 2),
            "lidar_left": round(self.lidar_left, 2),
            "lidar_right": round(self.lidar_right, 2),
            "vision": self.vision_status,
        }
        self.pub_status.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def control_loop(self):
        now = time.monotonic()
        if not self.path:
            self.state = "WAITING_FOR_NAV"
            pwm = self.ramp_pwm(0)
            self.publish_state(pwm, 0.0, "내비게이션 경로 대기")
            return
        faults = self.sensor_faults()
        if faults:
            self.state = "SENSOR_FAULT"
            pwm = self.ramp_pwm(0)
            self.publish_state(pwm, 0.0, "센서 데이터 오류: " + ", ".join(faults))
            return
        if self.cur_lat is None:
            self.state = "WAITING_FOR_GPS"
            pwm = self.ramp_pwm(0)
            self.publish_state(pwm, 0.0, "GPS 위치 대기")
            return
        p = GeoPoint(self.cur_lat, self.cur_lon)
        final = GeoPoint(self.path[-1][0], self.path[-1][1])
        if self.distance(p, final) <= self.arrival_distance:
            self.state = "ARRIVED"
            pwm = self.ramp_pwm(0)
            self.publish_state(pwm, 0.0, "목적지 도착")
            return

        override = self.obstacle_override(now)
        if override is not None:
            target_pwm, steer, mode = override
            pwm = self.ramp_pwm(target_pwm)
            self.publish_state(pwm, steer, mode)
            return

        self.state = "NAVIGATE"
        steer = self.path_steer()
        target_pwm = self.max_pwm

        if self.vision_status == "PERSON_NEAR":
            target_pwm = min(target_pwm, self.near_object_pwm)
        if self.vision_status == "CAR":
            target_pwm = min(target_pwm, self.corner_pwm)
        if self.imu_status == "BUMP":
            target_pwm = min(target_pwm, self.corner_pwm)
        if abs(steer) > 35.0:
            target_pwm = min(target_pwm, self.corner_pwm)

        pwm = self.ramp_pwm(target_pwm)
        self.publish_state(pwm, steer, "정상 경로 추종")

def main(args=None):
    rclpy.init(args=args)
    node = MainFusionNode()
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