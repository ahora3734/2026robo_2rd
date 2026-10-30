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

AVOID_STATES = ("AVOID_LEFT", "AVOID_RIGHT")

class MainFusionNode(Node):
    def __init__(self):
        super().__init__("main_fusion_node")
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("max_pwm", 180)
        self.declare_parameter("corner_pwm", 110)
        self.declare_parameter("near_object_pwm", 80)
        self.declare_parameter("slow_obstacle_pwm", 60)
        # 장애물 거리는 모두 '앞 범퍼' 기준 (lidar.py가 범퍼 기준으로 발행)
        self.declare_parameter("slow_obstacle_dist", 3.5)       # 이 거리부터 저속 접근
        self.declare_parameter("stop_obstacle_dist", 0.8)
        self.declare_parameter("emergency_obstacle_dist", 0.5)
        self.declare_parameter("lookahead_min_m", 2.0)
        self.declare_parameter("lookahead_max_m", 5.0)
        self.declare_parameter("lookahead_speed_gain", 0.015)
        self.declare_parameter("heading_gain", 0.9)
        self.declare_parameter("cross_track_gain", 5.0)
        # [REVIEW #8] ESP32 펌웨어 최대 조향각(±35°)과 일치시킴 (기존 100)
        self.declare_parameter("max_steer", 35.0)
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

        # --- [REVIEW 반영] 신규 파라미터 ---
        # 경로 추종: Pure Pursuit (False면 heading + CTE 방식, 부호 수정본 사용)
        self.declare_parameter("use_pure_pursuit", True)
        self.declare_parameter("wheelbase_m", 0.96)           # 고카트 휠베이스 (실측 960mm)
        self.declare_parameter("gps_to_rear_axle_m", 0.45)    # GPS 안테나가 뒷바퀴 축보다 앞에 있는 거리 (실측 450mm, 뒤에 있으면 음수)
        self.declare_parameter("progress_search_window", 8)   # 진행 세그먼트 검색 범위
        self.declare_parameter("corner_steer_deg", 20.0)      # 이 조향각 이상이면 코너 감속
        # 가감속: 가속은 천천히, 감속은 빠르게, 정지는 즉시
        self.declare_parameter("accel_step", 4.0)
        self.declare_parameter("decel_step", 10.0)
        # heading 오프셋 보정
        self.declare_parameter("heading_offset_gain", 0.05)
        self.declare_parameter("heading_correction_max_rate_dps", 10.0)
        self.declare_parameter("heading_correction_max_steer", 8.0)
        self.declare_parameter("require_align", True)         # 출발 시 직진 정렬 단계 사용
        self.declare_parameter("align_distance_m", 2.5)
        self.declare_parameter("align_pwm", 90)
        # 고정 장애물 회피: 판단 정지 -> 대기 -> (필요 시 후진) -> 정지 조향 -> 회피 -> 경로 복귀
        self.declare_parameter("obstacle_decision_dist", 2.0)   # 이 거리에서 정지 후 판단
        self.declare_parameter("obstacle_wait_sec", 2.5)        # 이 시간 동안 그대로면 고정 장애물로 판단
        self.declare_parameter("avoid_arc_required_m", 1.2)     # 최대 조향 궤적으로 최소 이만큼 갈 수 있어야 회피
        self.declare_parameter("avoid_arc_abort_m", 0.3)        # 회피 중 궤적 여유가 이보다 작으면 정지
        self.declare_parameter("steer_prep_tolerance_deg", 3.0) # 정지 조향 완료 판정 오차
        self.declare_parameter("steer_prep_timeout_sec", 3.0)   # 이 시간 안에 못 꺾으면 출발하지 않음
        # 후진 (후방 센서 없음: 조향 0으로 방금 지나온 길을 따라 짧게만 후진)
        self.declare_parameter("allow_reverse", True)
        self.declare_parameter("reverse_pwm", 40)
        self.declare_parameter("reverse_target_front_m", 1.8)   # 앞 범퍼~장애물 거리가 이만큼 될 때까지
        self.declare_parameter("reverse_max_sec", 3.0)
        self.declare_parameter("reverse_max_count", 2)          # 경로 복귀 전까지 최대 후진 횟수
        # ESP32 텔레메트리 감시 (조향 스톨/추종 실패)
        self.declare_parameter("require_esp32_telemetry", True)
        self.declare_parameter("esp32_timeout_sec", 1.0)
        self.declare_parameter("steer_track_error_deg", 10.0)
        self.declare_parameter("steer_track_timeout_sec", 2.0)
        self.declare_parameter("steer_fault_hold_sec", 3.0)
        # GPS 품질
        self.declare_parameter("require_gps_fix", True)
        self.declare_parameter("non_rtk_max_pwm", 110)        # RTK FIX/Float가 아니면 속도 제한

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

        self.use_pure_pursuit = bool(self.get_parameter("use_pure_pursuit").value)
        self.wheelbase = float(self.get_parameter("wheelbase_m").value)
        self.gps_to_rear_axle = float(self.get_parameter("gps_to_rear_axle_m").value)
        self.progress_window = max(2, int(self.get_parameter("progress_search_window").value))
        self.corner_steer = float(self.get_parameter("corner_steer_deg").value)
        self.accel_step = float(self.get_parameter("accel_step").value)
        self.decel_step = float(self.get_parameter("decel_step").value)
        self.heading_offset_gain = float(self.get_parameter("heading_offset_gain").value)
        self.heading_corr_max_rate = float(self.get_parameter("heading_correction_max_rate_dps").value)
        self.heading_corr_max_steer = float(self.get_parameter("heading_correction_max_steer").value)
        self.require_align = bool(self.get_parameter("require_align").value)
        self.align_distance = float(self.get_parameter("align_distance_m").value)
        self.align_pwm = self.clamp_pwm(self.get_parameter("align_pwm").value)
        self.obstacle_decision_dist = float(self.get_parameter("obstacle_decision_dist").value)
        self.obstacle_wait = float(self.get_parameter("obstacle_wait_sec").value)
        self.avoid_arc_required = float(self.get_parameter("avoid_arc_required_m").value)
        self.avoid_arc_abort = float(self.get_parameter("avoid_arc_abort_m").value)
        self.steer_prep_tol = float(self.get_parameter("steer_prep_tolerance_deg").value)
        self.steer_prep_timeout = float(self.get_parameter("steer_prep_timeout_sec").value)
        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)
        self.reverse_pwm = self.clamp_pwm(self.get_parameter("reverse_pwm").value)
        self.reverse_target_front = float(self.get_parameter("reverse_target_front_m").value)
        self.reverse_max = float(self.get_parameter("reverse_max_sec").value)
        self.reverse_max_count = int(self.get_parameter("reverse_max_count").value)
        self.require_esp32 = bool(self.get_parameter("require_esp32_telemetry").value)
        self.esp32_timeout = float(self.get_parameter("esp32_timeout_sec").value)
        self.steer_track_err = float(self.get_parameter("steer_track_error_deg").value)
        self.steer_track_timeout = float(self.get_parameter("steer_track_timeout_sec").value)
        self.steer_fault_hold = float(self.get_parameter("steer_fault_hold_sec").value)
        self.require_gps_fix = bool(self.get_parameter("require_gps_fix").value)
        self.non_rtk_max_pwm = self.clamp_pwm(self.get_parameter("non_rtk_max_pwm").value)

        self.create_subscription(String, "/obstacle_info", self.lidar_callback, 10)
        self.create_subscription(String, "/imu_status", self.imu_status_callback, 10)
        self.create_subscription(Float32, "/imu_heading", self.heading_callback, 10)
        self.create_subscription(String, "/imu_detail", self.imu_detail_callback, 10)
        self.create_subscription(String, "/gps_pos", self.gps_callback, 10)
        self.create_subscription(String, "/gps_status", self.gps_status_callback, 10)
        self.create_subscription(String, "/nav_path_data", self.nav_path_callback, 10)
        self.create_subscription(String, "/vision_info", self.vision_callback, 10)
        # [REVIEW #5] ESP32 조향 텔레메트리 감시
        self.create_subscription(String, "/esp32_raw_status", self.esp32_status_callback, 10)
        # [REVIEW #7] 대시보드 원격 비상정지 (보조 수단, 하드웨어 킬스위치 필수)
        self.create_subscription(String, "/estop", self.estop_callback, 10)

        self.pub_cmd = self.create_publisher(String, "/vehicle_cmd", 10)
        self.pub_status = self.create_publisher(String, "/vehicle_status", 10)

        now = time.monotonic()
        self.last_lidar = 0.0
        self.last_imu = 0.0
        self.last_gps = 0.0
        self.last_vision = now
        self.last_esp32 = 0.0

        self.imu_heading = 0.0
        self.imu_rate = 0.0
        self.imu_status = "STABLE"

        self.cur_lat = None
        self.cur_lon = None
        self.prev_gps = None
        self.gps_course = None
        self.heading_offset = 0.0
        self.heading_aligned = False
        self.align_start = None
        self.gps_real = True
        self.gps_fix_quality = None

        self.path = []
        # [REVIEW #8] 진행 세그먼트(차가 지금 있는 구간)와 lookahead 목표점을 분리
        self.progress_idx = 0
        self.progress_t = 0.0
        self.progress_initialized = False
        self.last_cte = None
        self.last_cte_sign = 0.0
        self.last_target_bearing = None

        self.lidar_front = 10.0
        self.lidar_left = 10.0
        self.lidar_right = 10.0
        self.lidar_arc_left = 0.0
        self.lidar_arc_right = 0.0
        self.lidar_status = "clear"

        self.vision_status = "CLEAR"
        self.state = "WAITING_FOR_NAV"
        self.avoid_direction = 0
        self.avoid_start = 0.0
        self.wait_start = 0.0
        self.steer_prep_start = 0.0
        self.reverse_start = 0.0
        self.reverse_count = 0

        self.esp32_steer_cur = 0.0
        self.esp32_steer_time = 0.0

        self.steer_err_since = None
        self.steer_fault_until = 0.0
        self.steer_fault_text = ""
        self.remote_estop = False

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

    def vehicle_point(self):
        """Pure Pursuit 기준점(뒷바퀴 축 중심). 방위 정렬 전에는 GPS 안테나 위치를 그대로 사용"""
        p = GeoPoint(self.cur_lat, self.cur_lon)
        if not self.heading_aligned or self.gps_to_rear_axle == 0.0:
            return p
        h = math.radians(self.navigation_heading)
        back = -self.gps_to_rear_axle
        lat = p.lat + (back * math.cos(h)) / 111320.0
        lon = p.lon + (back * math.sin(h)) / (111320.0 * math.cos(math.radians(p.lat)))
        return GeoPoint(lat, lon)

    @staticmethod
    def local_xy(ref: GeoPoint, p: GeoPoint):
        lat_m = 111320.0
        lon_m = 111320.0 * math.cos(math.radians(ref.lat))
        return ((p.lon - ref.lon) * lon_m, (p.lat - ref.lat) * lat_m)

    def segment_cte(self, p, a, b):
        # a, b: 차량(p) 기준 상대좌표 (x=동, y=북)
        ax, ay = self.local_xy(p, a)
        bx, by = self.local_xy(p, b)
        vx, vy = bx - ax, by - ay
        # [REVIEW #8-1] 분모는 세그먼트 길이의 제곱 |b-a|^2 (기존: |b-p|^2)
        denom = vx * vx + vy * vy
        if denom < 1e-9:
            return math.hypot(ax, ay), 0.0, 0.0
        wx, wy = -ax, -ay
        t = (wx * vx + wy * vy) / denom
        t = max(0.0, min(1.0, t))
        cx = ax + t * vx
        cy = ay + t * vy
        cte = math.hypot(cx, cy)
        # cross > 0 : 진행 방향 기준 차량이 경로 '왼쪽'
        cross = vx * (-ay) - vy * (-ax)
        sign = 1.0 if cross >= 0 else -1.0
        return cte, sign, t

    def closest_path(self, p):
        """반환: (cte, 세그먼트 인덱스, 좌우 부호, 세그먼트 내 비율 t)"""
        if len(self.path) < 2:
            return float("inf"), 0, 0.0, 0.0
        if self.progress_initialized:
            start = max(0, min(self.progress_idx, len(self.path) - 2))
            end = min(len(self.path) - 1, start + self.progress_window)
        else:
            # 새 경로를 받은 직후에는 전체 구간에서 현재 위치를 찾음
            start, end = 0, len(self.path) - 1
        best = (float("inf"), start, 0.0, 0.0)
        for i in range(start, end):
            a = GeoPoint(self.path[i][0], self.path[i][1])
            b = GeoPoint(self.path[i + 1][0], self.path[i + 1][1])
            d, sign, t = self.segment_cte(p, a, b)
            if d < best[0]:
                best = (d, i, sign, t)
        return best

    def update_progress(self, p):
        """[REVIEW #8-3] 제어 루프에서 한 번만 호출하여 진행 상태를 갱신"""
        if len(self.path) < 2:
            self.progress_idx = 0
            self.progress_t = 0.0
            self.last_cte = None
            self.last_cte_sign = 0.0
            return
        cte, seg, sign, t = self.closest_path(p)
        self.progress_idx = seg
        self.progress_t = t
        self.progress_initialized = True
        self.last_cte = cte
        self.last_cte_sign = sign

    def lookahead_distance(self):
        return max(self.lookahead_min, min(self.lookahead_max, self.lookahead_min + self.current_pwm * self.lookahead_speed_gain))

    def lookahead_target(self, p):
        """진행 세그먼트 위 투영점에서 경로를 따라 lookahead 거리만큼 앞의 점 (상태 변경 없음)"""
        if not self.path:
            return None
        if len(self.path) == 1:
            return GeoPoint(self.path[0][0], self.path[0][1])
        seg = max(0, min(self.progress_idx, len(self.path) - 2))
        a = GeoPoint(self.path[seg][0], self.path[seg][1])
        b = GeoPoint(self.path[seg + 1][0], self.path[seg + 1][1])
        t = self.progress_t
        prev = GeoPoint(a.lat + (b.lat - a.lat) * t, a.lon + (b.lon - a.lon) * t)
        remaining = self.lookahead_distance()
        for i in range(seg + 1, len(self.path)):
            wp = GeoPoint(self.path[i][0], self.path[i][1])
            d = self.distance(prev, wp)
            if d >= remaining and d > 1e-6:
                r = remaining / d
                return GeoPoint(prev.lat + (wp.lat - prev.lat) * r, prev.lon + (wp.lon - prev.lon) * r)
            remaining -= d
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
            # 최대 조향 궤적으로 부딪히기 전까지 갈 수 있는 거리 (없으면 0 = 회피 불가로 간주)
            self.lidar_arc_left = num("arc_left", 0.0)
            self.lidar_arc_right = num("arc_right", 0.0)
        except Exception:
            self.lidar_status = "invalid"

    def imu_status_callback(self, msg):
        # [REVIEW #6] 상태 메시지는 생존 신호로 쓰지 않음 (IMU 워치독의 "FAULT"도 이 토픽으로 옴)
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

    def gps_status_callback(self, msg):
        try:
            obj = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(obj, dict):
            return
        self.gps_real = bool(obj.get("is_real_gps", True))
        q = obj.get("fix_quality")
        try:
            self.gps_fix_quality = int(q) if q is not None else None
        except Exception:
            self.gps_fix_quality = None

    def gps_is_rtk(self):
        return self.gps_fix_quality in (4, 5)

    def gps_callback(self, msg):
        try:
            lat, lon = map(float, msg.data.split(",")[:2])
        except Exception:
            return
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return
        self.last_gps = time.monotonic()
        current = GeoPoint(lat, lon)

        # 1. 초기 GPS 기준점 설정
        if self.prev_gps is None:
            self.prev_gps = current
        if self.state != "ALIGN":
            self.align_start = None

        # 2. 정지/저속 시 GPS 헤딩 갱신 동결 (GPS 드리프트로 인한 헤딩 오프셋 왜곡 원천 차단)
        is_driving = (self.current_pwm > 30 and self.state in ("NAVIGATE", "CRUISE", "ALIGN", "AVOID_LEFT", "AVOID_RIGHT", "REJOIN_PATH"))

        if is_driving:
            if self.state == "ALIGN" and not self.heading_aligned:
                # [REVIEW #9] 정렬 단계: 조향 0으로 직진한 구간의 GPS course로 오프셋을 바로 대입
                if self.align_start is None:
                    self.align_start = current
                elif self.distance(self.align_start, current) >= self.align_distance:
                    course = self.bearing(self.align_start, current)
                    self.gps_course = course
                    self.heading_offset = self.wrap180(course - self.imu_heading) % 360.0
                    self.heading_aligned = True
                    self.align_start = None
                    self.prev_gps = current
                    self.get_logger().info(f"🧭 초기 방위 정렬 완료: course={course:.1f}°, offset={self.heading_offset:.1f}°")
            else:
                moved = self.distance(self.prev_gps, current)
                # 실제로 2m 이상 주행 이동했을 때만 방위각 및 오프셋 보정
                if moved >= self.gps_course_min_move:
                    course = self.bearing(self.prev_gps, current)
                    if self.gps_course is None:
                        self.gps_course = course
                    else:
                        error = self.wrap180(course - self.gps_course)
                        self.gps_course = (self.gps_course + self.gps_course_alpha * error) % 360.0
                    # [REVIEW #9] 회전 중에는 GPS course가 지연되므로 보정 중단
                    turning = (abs(self.imu_rate) > self.heading_corr_max_rate or abs(self.filtered_steer) > self.heading_corr_max_steer)
                    if not turning:
                        if not self.heading_aligned:
                            # 정렬 단계를 쓰지 않는 경우: 첫 course로 바로 대입
                            self.heading_offset = self.wrap180(self.gps_course - self.imu_heading) % 360.0
                            self.heading_aligned = True
                        else:
                            # [REVIEW #9] 현재 오프셋을 반영한 오차로 보정해야 수렴함
                            error = self.wrap180(self.gps_course - (self.imu_heading + self.heading_offset))
                            self.heading_offset = (self.heading_offset + self.heading_offset_gain * error) % 360.0
                    self.prev_gps = current
        else:
            # 정지 상태에서는 기준점을 현재 위치로 유지하여 오차 누적 방지
            self.prev_gps = current
            self.align_start = None

        self.cur_lat = lat
        self.cur_lon = lon

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
            # [REVIEW #13] 웨이포인트 통과 시 같은 경로가 재발행되므로, 경로가 바뀐 경우에만 상태를 초기화
            if path == self.path:
                return
            self.path = path
            requested_idx = int(data.get("target_idx", 0))
            self.progress_idx = max(0, min(requested_idx, max(0, len(self.path) - 2)))
            self.progress_t = 0.0
            self.progress_initialized = False
            self.avoid_direction = 0
            self.reverse_count = 0
            if not self.path:
                self.state = "WAITING_FOR_NAV"
            elif self.state in ("WAITING_FOR_NAV", "ARRIVED"):
                self.state = "NAVIGATE"
            self.get_logger().info(f"🗺️ 새 경로 수신: {len(self.path)} 포인트")
        except Exception:
            pass

    def vision_callback(self, msg):
        self.last_vision = time.monotonic()
        self.vision_status = (msg.data.strip() or "CLEAR")

    def esp32_status_callback(self, msg):
        """[REVIEW #5] ESP32 텔레메트리로 조향 스톨/추종 실패 감지"""
        try:
            obj = json.loads(msg.data)
        except Exception:
            return  # 부팅 메시지 등 JSON이 아닌 줄은 무시
        if not isinstance(obj, dict):
            return
        now = time.monotonic()
        self.last_esp32 = now
        try:
            cur = float(obj.get("steer_cur", 0.0))
            tgt = float(obj.get("steer_target", 0.0))
            pwm = int(float(obj.get("pwm", 0)))
            armed = int(obj.get("armed", 0))
        except Exception:
            return
        # 정지 조향(STEER_PREP) 완료 판정용 실제 조향각
        self.esp32_steer_cur = cur
        self.esp32_steer_time = now
        fault = str(obj.get("fault", "") or "")
        if fault:
            self.steer_fault_until = now + self.steer_fault_hold
            self.steer_fault_text = fault
        # 구동 중인데 조향이 목표를 계속 따라가지 못하면 고장으로 판단
        if armed and pwm != 0 and abs(tgt - cur) > self.steer_track_err:
            if self.steer_err_since is None:
                self.steer_err_since = now
            elif now - self.steer_err_since > self.steer_track_timeout:
                self.steer_fault_until = now + self.steer_fault_hold
                self.steer_fault_text = "STEER_TRACK"
                self.steer_err_since = None
        else:
            self.steer_err_since = None

    def estop_callback(self, msg):
        cmd = msg.data.strip().upper()
        if cmd in ("STOP", "ESTOP", "E_STOP", "1", "TRUE"):
            if not self.remote_estop:
                self.get_logger().warn("🛑 원격 비상정지 수신")
            self.remote_estop = True
        elif cmd in ("RELEASE", "RESUME", "0", "FALSE"):
            if self.remote_estop:
                self.get_logger().info("✅ 원격 비상정지 해제")
            self.remote_estop = False

    def sensor_faults(self):
        now = time.monotonic()
        faults = []
        if self.last_lidar <= 0.0 or now - self.last_lidar > self.lidar_timeout: faults.append("LiDAR")
        if self.last_imu <= 0.0 or now - self.last_imu > self.imu_timeout or self.imu_status == "FAULT": faults.append("IMU")
        if self.last_gps <= 0.0 or now - self.last_gps > self.gps_timeout: faults.append("GPS")
        if self.require_gps_fix and not self.gps_real: faults.append("GPS_FIX")
        if now - self.last_vision > self.vision_timeout: faults.append("CAM")
        if self.require_esp32 and (self.last_esp32 <= 0.0 or now - self.last_esp32 > self.esp32_timeout): faults.append("ESP32")
        if now < self.steer_fault_until: faults.append(f"STEER({self.steer_fault_text})")
        return faults

    def arc_free(self, direction):
        """direction: +1 오른쪽, -1 왼쪽. 최대 조향으로 부딪히기 전까지 갈 수 있는 거리 (lidar.py 계산)"""
        return self.lidar_arc_right if direction > 0 else self.lidar_arc_left

    def choose_avoid_direction(self):
        """회전 궤적에 여유가 있는 쪽으로만 회피, 양쪽 모두 막혔으면 0"""
        right, left = self.arc_free(1), self.arc_free(-1)
        right_ok = right >= self.avoid_arc_required
        left_ok = left >= self.avoid_arc_required
        if right_ok and left_ok and abs(right - left) < 0.2:
            # 양쪽 여유가 비슷하면 경로가 있는 쪽으로 (cte 부호 +1 = 차가 경로 왼쪽 -> 오른쪽으로)
            return 1 if self.last_cte_sign >= 0 else -1
        if right_ok and (not left_ok or right >= left):
            return 1
        if left_ok:
            return -1
        return 0

    def obstacle_speed_cap(self, target_pwm):
        """장애물에 가까워지면 저속 접근"""
        if self.lidar_front <= self.slow_obstacle_dist:
            return min(target_pwm, self.slow_obstacle_pwm)
        return target_pwm

    def enter_obstacle_wait(self, now, settle_only=False):
        """정지 후 대기. settle_only면 차가 멈출 시간(0.5초)만 기다리고 바로 다시 판단"""
        self.state = "OBSTACLE_WAIT"
        self.wait_start = now - max(0.0, self.obstacle_wait - 0.5) if settle_only else now
        self.avoid_direction = 0

    def path_steer(self, p=None):
        if not self.path or self.cur_lat is None:
            return 0.0
        if p is None:
            p = self.vehicle_point()
        target = self.lookahead_target(p)
        if target is None:
            return 0.0
        target_bearing = self.bearing(p, target)
        self.last_target_bearing = target_bearing
        heading_error = self.wrap180(target_bearing - self.navigation_heading)
        if self.use_pure_pursuit:
            # [REVIEW #8] Pure Pursuit: δ = atan(2·L·sin α / Ld)
            if abs(heading_error) >= 90.0:
                steer = math.copysign(self.max_steer, heading_error)
            else:
                ld = max(self.lookahead_min, self.distance(p, target))
                steer = math.degrees(math.atan2(2.0 * self.wheelbase * math.sin(math.radians(heading_error)), ld))
        else:
            signed_cte = (self.last_cte or 0.0) * self.last_cte_sign
            # [REVIEW #8-2] 부호 수정: 경로 왼쪽(sign=+1)이면 오른쪽(+)으로 보정
            steer = (self.heading_gain * heading_error + self.cross_track_gain * signed_cte)
        return max(-self.max_steer, min(self.max_steer, steer))

    def ramp_pwm(self, target):
        target = float(self.clamp_pwm(target))
        if self.current_pwm < target:
            self.current_pwm = min(target, self.current_pwm + self.accel_step)
        else:
            self.current_pwm = max(target, self.current_pwm - self.decel_step)
        self.current_pwm = max(0.0, min(255.0, self.current_pwm))
        return int(round(self.current_pwm))

    def hard_stop(self):
        """[REVIEW #1] 정지 상태에서는 램프 없이 즉시 0"""
        self.current_pwm = 0.0
        return 0

    def obstacle_wait_step(self, now, person):
        """정지 대기: 사라지면 재출발, 그대로면 고정 장애물 -> 회피 방향 조향 또는 후진"""
        front = self.lidar_front
        if person:
            # 사람은 회피하지 않고 비킬 때까지 대기
            self.wait_start = now
            return 0, 0.0, "보행자 대기"
        if front > self.obstacle_decision_dist + 0.3:
            # 장애물이 사라짐 (움직이는 물체였음) -> 주행 재개
            self.reverse_count = 0
            self.state = "NAVIGATE"
            return None
        elapsed = now - self.wait_start
        if elapsed < self.obstacle_wait:
            return 0, 0.0, f"장애물 대기 ({self.obstacle_wait - elapsed:.1f}s)"

        # 대기 후에도 그대로 -> 고정 장애물
        direction = self.choose_avoid_direction()
        if direction != 0:
            self.avoid_direction = direction
            self.state = "STEER_PREP"
            self.steer_prep_start = now
            return 0, direction * self.avoid_steer, "회피 방향 조향 준비"
        if (self.allow_reverse and self.reverse_count < self.reverse_max_count
                and front < self.reverse_target_front):
            self.state = "REVERSE"
            self.reverse_start = now
            self.reverse_count += 1
            return self.reverse_pwm, 0.0, "후진하여 회피 공간 확보"
        return 0, 0.0, "회피 경로 없음 - 대기"

    def steer_prep_step(self, now, person):
        """정지 상태에서 핸들을 회피 방향 최대로 꺾고, 실제 조향각(steer_cur)이 도달하면 출발"""
        target = self.avoid_direction * self.avoid_steer
        if person:
            self.enter_obstacle_wait(now)
            return 0, 0.0, "보행자 감지 - 조향 준비 중단"
        fresh = (now - self.esp32_steer_time) <= 0.3
        if fresh and abs(self.esp32_steer_cur - target) <= self.steer_prep_tol:
            self.state = "AVOID_RIGHT" if self.avoid_direction > 0 else "AVOID_LEFT"
            self.avoid_start = now
            return self.slow_obstacle_pwm, target, "장애물 회피 출발"
        if now - self.steer_prep_start > self.steer_prep_timeout:
            self.enter_obstacle_wait(now)
            return 0, 0.0, "조향 준비 실패 - 재시도 대기"
        return 0, target, "정지 상태에서 회피 방향 조향"

    def reverse_step(self, now):
        """조향 0으로 저속 직진 후진. 앞 장애물과의 거리가 확보되거나 시간 초과 시 정지 후 재판단"""
        if self.lidar_front >= self.reverse_target_front or now - self.reverse_start >= self.reverse_max:
            self.enter_obstacle_wait(now, settle_only=True)
            return 0, 0.0, "후진 완료 - 회피 방향 판단"
        return self.reverse_pwm, 0.0, f"후진 중 ({self.reverse_count}/{self.reverse_max_count})"

    def obstacle_override(self, now):
        front = self.lidar_front
        person = self.vision_status == "PERSON_FRONT"

        if self.state == "REVERSE":
            return self.reverse_step(now)
        if self.state == "STEER_PREP":
            return self.steer_prep_step(now, person)
        if self.state == "OBSTACLE_WAIT":
            return self.obstacle_wait_step(now, person)

        # 주행 중: 근접 장애물/보행자 -> 즉시 정지 후 대기
        if front <= self.emergency_obstacle_dist:
            self.enter_obstacle_wait(now)
            return 0, 0.0, "E-STOP (초근접 장애물)"
        if person:
            self.enter_obstacle_wait(now)
            return 0, 0.0, "보행자 정지"
        if front <= self.stop_obstacle_dist:
            self.enter_obstacle_wait(now)
            return 0, 0.0, "근접 장애물 정지"
        # 정상 주행/경로 복귀 중 판단 거리 진입 -> 정지 후 고정 장애물인지 확인 (경로 복귀 중 재회피 포함)
        if front <= self.obstacle_decision_dist and self.state not in AVOID_STATES:
            self.enter_obstacle_wait(now)
            return 0, 0.0, "장애물 감지 - 정지 후 판단"

        if self.state in AVOID_STATES:
            direction = 1 if self.state == "AVOID_RIGHT" else -1
            if self.arc_free(direction) < self.avoid_arc_abort:
                self.enter_obstacle_wait(now, settle_only=True)
                return 0, 0.0, "회피 경로 근접 - 정지"
            if now - self.avoid_start >= self.avoid_min_duration and front > self.obstacle_decision_dist + 0.3:
                self.state = "REJOIN_PATH"
            return self.slow_obstacle_pwm, direction * self.avoid_steer, "장애물 회피"
        if self.state == "REJOIN_PATH":
            cte = self.last_cte if self.last_cte is not None else float("inf")
            if cte <= self.rejoin_distance:
                self.avoid_direction = 0
                self.reverse_count = 0
                self.state = "NAVIGATE"
            return self.corner_pwm, self.path_steer(), "경로 복귀"
        return None

    def publish_state(self, speed_pwm, steer, mode, steer_hold=False, raw_steer=False):
        # raw_steer: 필터 없이 목표 조향을 그대로 전송 (정지 조향, 후진)
        if raw_steer:
            self.filtered_steer = float(steer)
        else:
            self.filtered_steer = (self.steer_alpha * steer + (1.0 - self.steer_alpha) * self.filtered_steer)
        # 음수 = 후진 (REVERSE 상태에서만 음수가 들어옴)
        cmd_pwm = int(max(-255, min(255, round(speed_pwm))))

        cmd = {
            "pwm": cmd_pwm,
            "steer": round(float(self.filtered_steer), 2),
            "steer_hold": bool(steer_hold),
            "mode": mode,
            "state": self.state,
        }
        self.pub_cmd.publish(String(data=json.dumps(cmd, ensure_ascii=False)))

        # [REVIEW #8] 상태 표시용 값은 제어 루프에서 계산한 캐시를 사용 (여기서 상태를 바꾸지 않음)
        cte = self.last_cte if (self.last_cte is not None and math.isfinite(self.last_cte)) else None
        target_bearing = self.last_target_bearing

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
            "heading_aligned": self.heading_aligned,
            "gps_fix_quality": self.gps_fix_quality,
            "progress_idx": self.progress_idx,
            "cross_track_m": None if cte is None else round(cte, 2),
            "target_bearing": None if target_bearing is None else round(target_bearing, 1),
            "lidar_front": round(self.lidar_front, 2),
            "lidar_left": round(self.lidar_left, 2),
            "lidar_right": round(self.lidar_right, 2),
            "lidar_arc_left": round(self.lidar_arc_left, 2),
            "lidar_arc_right": round(self.lidar_arc_right, 2),
            "vision": self.vision_status,
            "remote_estop": self.remote_estop,
        }
        self.pub_status.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def control_loop(self):
        now = time.monotonic()
        self.last_target_bearing = None
        if self.remote_estop:
            self.state = "REMOTE_ESTOP"
            self.publish_state(self.hard_stop(), 0.0, "원격 비상정지 (대시보드)")
            return
        if not self.path:
            self.state = "WAITING_FOR_NAV"
            self.publish_state(self.hard_stop(), 0.0, "내비게이션 경로 대기")
            return
        faults = self.sensor_faults()
        if faults:
            self.state = "SENSOR_FAULT"
            self.publish_state(self.hard_stop(), 0.0, "센서 데이터 오류: " + ", ".join(faults))
            return
        if self.cur_lat is None:
            self.state = "WAITING_FOR_GPS"
            self.publish_state(self.hard_stop(), 0.0, "GPS 위치 대기")
            return
        p = self.vehicle_point()
        self.update_progress(p)
        final = GeoPoint(self.path[-1][0], self.path[-1][1])
        if self.distance(p, final) <= self.arrival_distance:
            self.state = "ARRIVED"
            self.publish_state(self.hard_stop(), 0.0, "목적지 도착")
            return

        override = self.obstacle_override(now)
        if override is not None:
            target_pwm, steer, mode = override
            if self.state == "REVERSE":
                # 후진: 램프 없이 저속 고정, 조향은 직진 유지
                self.current_pwm = 0.0
                self.publish_state(-abs(target_pwm), 0.0, mode, raw_steer=True)
                return
            if self.state == "STEER_PREP":
                # 정지 조향: 구동 PWM 0 + ESP32에 정지 조향 허용, 목표각은 필터 없이 바로 전송
                self.publish_state(self.hard_stop(), steer, mode, steer_hold=True, raw_steer=True)
                return
            pwm = self.hard_stop() if target_pwm <= 0 else self.ramp_pwm(self.obstacle_speed_cap(target_pwm))
            self.publish_state(pwm, steer, mode)
            return

        speed_cap = self.max_pwm
        if not self.gps_is_rtk():
            # [REVIEW #12] RTK가 아니면(오차 1~3m) 속도 제한
            speed_cap = min(speed_cap, self.non_rtk_max_pwm)

        if self.require_align and not self.heading_aligned:
            # [REVIEW #9] 출발 직후 진짜 방위를 모르므로 조향 0으로 직진하며 GPS course 측정
            self.state = "ALIGN"
            pwm = self.ramp_pwm(self.obstacle_speed_cap(min(self.align_pwm, speed_cap)))
            self.publish_state(pwm, 0.0, "초기 방위 정렬 직진")
            return

        self.state = "NAVIGATE"
        steer = self.path_steer(p)
        target_pwm = speed_cap

        if self.vision_status == "PERSON_NEAR":
            target_pwm = min(target_pwm, self.near_object_pwm)
        if self.vision_status == "CAR":
            target_pwm = min(target_pwm, self.corner_pwm)
        if self.imu_status == "BUMP":
            target_pwm = min(target_pwm, self.corner_pwm)
        if abs(steer) > self.corner_steer:
            target_pwm = min(target_pwm, self.corner_pwm)
        target_pwm = self.obstacle_speed_cap(target_pwm)

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