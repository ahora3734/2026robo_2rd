#!/usr/bin/env python3
"""
gps_tmap_nav.py
============================================================
GPS 수신 및 T-Map/교내 실측 전역 경로 계획 전용 노드
- u-blox ZED-F9P RTK GNSS (/dev/ublox_gps) NMEA 데이터 파싱
- /gps_pos 및 /gps_status 토픽 실시간 발행
- 교내 실측 정밀 경로 및 SK T-Map 보행자 API 기반 경로 생성
- 웹 관제탑(/set_destination)으로부터 목적지 요청 수신
- /nav_path_data 발행 (main_fusion 및 web_dashboard 연동)
============================================================
"""

import json
import math
import os
import threading
import time
import requests
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# [교내 정밀 실측 GPS 데이터 100% 보존]
CAMPUS_ROUTES = {
    "MAIN_GATE_TO_INTERSECTION": [
        [36.0563257, 129.3705626], [36.0563495, 129.3706043], [36.0563718, 129.3706468],
        [36.0563821, 129.3706893], [36.0563975, 129.3707233], [36.0564147, 129.3707594],
        [36.0564233, 129.3708167], [36.0564508, 129.3708719], [36.0564611, 129.3709207],
        [36.0564662, 129.3709589], [36.0564799, 129.3710226], [36.0564885, 129.3710735],
        [36.0564936, 129.3711266], [36.0564936, 129.3711669], [36.0564936, 129.3712094],
        [36.0564867, 129.3712391], [36.0564816, 129.3712964], [36.0564782, 129.3713495],
        [36.0564696, 129.3714196], [36.0564679, 129.3714727], [36.0564628, 129.3715385],
        [36.0564559, 129.3715958], [36.0564525, 129.3716574], [36.0564459, 129.3716738],
        [36.0564382, 129.3717111]
    ],
    "GYM_TO_INTERSECTION": [
        [36.0572170, 129.3708713], [36.0571453, 129.3708474], [36.0570736, 129.3708338],
        [36.0569992, 129.3708270], [36.0569193, 129.3708065], [36.0568559, 129.3707997],
        [36.0567649, 129.3707963], [36.0566960, 129.3707929], [36.0566298, 129.3708202],
        [36.0565885, 129.3708509], [36.0565251, 129.3708679], [36.0564479, 129.3709020],
        [36.0564644, 129.3709293], [36.0564754, 129.3709975], [36.0564919, 129.3710623],
        [36.0564919, 129.3711169], [36.0564974, 129.3711646], [36.0564836, 129.3712328],
        [36.0564836, 129.3713044], [36.0564808, 129.3713931], [36.0564670, 129.3714852],
        [36.0564560, 129.3715500], [36.0564450, 129.3716114], [36.0564478, 129.3716387],
        [36.0564382, 129.3717111]
    ],
    "START_TO_CAFETERIA": [
        [36.0566126, 129.3723359], [36.0565643, 129.3723299], [36.0565369, 129.3723120],
        [36.0564918, 129.3723040], [36.0564483, 129.3722861], [36.0564161, 129.3722602],
        [36.0563935, 129.3722164], [36.0563919, 129.3721925], [36.0563951, 129.3721586],
        [36.0563999, 129.3721227], [36.0564015, 129.3720749], [36.0564112, 129.3720370],
        [36.0564112, 129.3719952], [36.0564193, 129.3719494], [36.0564257, 129.3718876],
        [36.0564225, 129.3718318], [36.0564289, 129.3718019], [36.0564402, 129.3717581],
        [36.0564466, 129.3717003], [36.0564047, 129.3717003], [36.0563821, 129.3717142],
        [36.0563531, 129.3717301], [36.0563112, 129.3717540], [36.0562548, 129.3717939],
        [36.0561904, 129.3718258], [36.0561421, 129.3718676], [36.0560777, 129.3719035],
        [36.0560052, 129.3719553], [36.0559134, 129.3720191], [36.0558490, 129.3720530],
        [36.0557810, 129.3720968], [36.0556847, 129.3721586], [36.0556154, 129.3721945],
        [36.0555655, 129.3722284], [36.0555204, 129.3722603], [36.0554978, 129.3722742],
        [36.0554428, 129.3723069]
    ],
    "START_TO_INTERSECTION": [
        [36.0566059, 129.3723184], [36.0565730, 129.3723165], [36.0565198, 129.3723088],
        [36.0564807, 129.3722933], [36.0564356, 129.3722740], [36.0564027, 129.3722418],
        [36.0563923, 129.3721989], [36.0563940, 129.3721431], [36.0564009, 129.3721088],
        [36.0564009, 129.3720681], [36.0564044, 129.3720231], [36.0564148, 129.3719888],
        [36.0564183, 129.3719352], [36.0564287, 129.3718987], [36.0564270, 129.3718537],
        [36.0564339, 129.3718108], [36.0564391, 129.3717572], [36.0564382, 129.3717111]
    ]
}

KEYWORD_TO_ROUTE = {
    "정문": "MAIN_GATE_TO_INTERSECTION",
    "체육관": "GYM_TO_INTERSECTION",
    "급식소": "START_TO_CAFETERIA",
    "삼거리": "START_TO_INTERSECTION"
}

SERIAL_PORT = "/dev/ublox_gps"
BAUDRATE = 115200
TMAP_APP_KEY = os.getenv("TMAP_APP_KEY", "")
MOCK_START_LAT = 36.056400
MOCK_START_LON = 129.371500

class GpsTmapNavNode(Node):
    def __init__(self):
        super().__init__("gps_tmap_nav_node")

        self.declare_parameter("gps_port", SERIAL_PORT)
        self.declare_parameter("gps_baudrate", BAUDRATE)
        self.declare_parameter("tmap_app_key", TMAP_APP_KEY)

        self.gps_port = self.get_parameter("gps_port").value
        self.gps_baudrate = int(self.get_parameter("gps_baudrate").value)
        self.tmap_app_key = self.get_parameter("tmap_app_key").value

        # 퍼블리셔
        self.pub_gps_pos = self.create_publisher(String, "/gps_pos", 10)
        self.pub_gps_status = self.create_publisher(String, "/gps_status", 10)
        self.pub_path = self.create_publisher(String, "/nav_path_data", 10)

        # 서브스크라이버 (웹 관제탑 등 외부에서의 목적지 설정 명령 수신)
        self.sub_dest = self.create_subscription(String, "/set_destination", self.on_set_destination_cmd, 10)

        # 상태 변수
        self.gps_lock = threading.Lock()
        self.route_lock = threading.Lock()

        self.gps_data = {
            "lat": MOCK_START_LAT,
            "lon": MOCK_START_LON,
            "quality": "GPS 연결 대기 중",
            "is_real_gps": False,
        }

        self.dest_lat = None
        self.dest_lon = None
        self.has_destination = False

        self.route_data = {
            "path": [],
            "waypoints": [],
            "current_wp_idx": 0,
            "next_wp_desc": "목적지를 설정하세요.",
            "dist_to_next_wp": 0.0,
        }

        self.running = True

        # GPS 수신 백그라운드 스레드 가동
        self.gps_thread = threading.Thread(target=self.gps_reader_worker, daemon=True)
        self.gps_thread.start()

        self.get_logger().info(f"✅ GPS & T-Map 전역 경로 계획 노드 가동 (GPS 포트: {self.gps_port})")

    @staticmethod
    def haversine_m(lat1, lon1, lat2, lon2):
        R = 6371000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    @staticmethod
    def nmea_to_decimal(raw, direction):
        try:
            if not raw or not direction: return None
            if direction in ("N", "S"): deg, minutes = float(raw[:2]), float(raw[2:])
            else: deg, minutes = float(raw[:3]), float(raw[3:])
            value = deg + minutes / 60.0
            return -value if direction in ("S", "W") else value
        except Exception:
            return None

    def publish_path_message(self, path, target_idx):
        with self.route_lock:
            payload = {
                "path": path,
                "target_idx": int(target_idx),
                "waypoints": self.route_data.get("waypoints", []),
                "next_wp_desc": self.route_data.get("next_wp_desc", ""),
                "dist_to_next_wp": self.route_data.get("dist_to_next_wp", 0.0),
                "has_dest": self.has_destination,
            }
        self.pub_path.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def update_navigation(self, lat, lon):
        if not self.has_destination: return
        with self.route_lock:
            wps = list(self.route_data["waypoints"])
            idx = int(self.route_data["current_wp_idx"])
            path = list(self.route_data["path"])
        if not wps or idx >= len(wps): return

        target = wps[idx]
        d = self.haversine_m(lat, lon, target["lat"], target["lon"])

        advanced = False
        with self.route_lock:
            self.route_data["dist_to_next_wp"] = round(d, 1)
            if d < 3.0 and idx < len(wps) - 1:
                idx += 1
                self.route_data["current_wp_idx"] = idx
                target = wps[idx]
                advanced = True
            self.route_data["next_wp_desc"] = f"[{target['name']}] {target['desc']}"
        # publish_path_message()가 route_lock을 다시 잡으므로 락 밖에서 호출 (데드락 방지)
        if advanced:
            self.publish_path_message(path, idx)

    def fetch_tmap_route(self, start_lat, start_lon):
        if not (self.tmap_app_key and self.dest_lat is not None and self.dest_lon is not None):
            self.get_logger().warn("⚠️ T-Map API 키 또는 목적지 좌표가 유효하지 않습니다.")
            return False

        url = "https://apis.openapi.sk.com/tmap/routes/pedestrian?version=1"
        headers = {"appKey": self.tmap_app_key, "Content-Type": "application/x-www-form-urlencoded"}
        payload = {
            "startX": str(start_lon), "startY": str(start_lat),
            "endX": str(self.dest_lon), "endY": str(self.dest_lat),
            "startName": "현재위치", "endName": "목적지",
            "reqCoordType": "WGS84GEO", "resCoordType": "WGS84GEO", "searchCoordType": "WGS84GEO"
        }
        try:
            r = requests.post(url, headers=headers, data=payload, timeout=5)
            r.raise_for_status()
            data = r.json()
            path, waypoints = [], []
            for feat in data.get("features", []):
                geom = feat.get("geometry", {})
                prop = feat.get("properties", {})
                typ = geom.get("type")
                coords = geom.get("coordinates")
                if typ == "LineString" and coords:
                    path.extend([[c[1], c[0]] for c in coords])
                elif typ == "Point" and coords:
                    waypoints.append({
                        "lat": coords[1], "lon": coords[0],
                        "name": prop.get("name", "안내점"),
                        "desc": prop.get("description", "")
                    })
            if len(path) < 2: return False
            if not waypoints:
                waypoints = [{"lat": p[0], "lon": p[1], "name": "경로점", "desc": ""} for p in path]

            with self.route_lock:
                self.route_data["path"] = path
                self.route_data["waypoints"] = waypoints
                self.route_data["current_wp_idx"] = 0
                self.route_data["next_wp_desc"] = waypoints[0]["desc"]
            self.publish_path_message(path, 0)
            self.get_logger().info(f"✅ T-Map OpenAPI 보행자 경로 수신 완료 (포인트 수: {len(path)})")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ T-Map 경로 요청 실패: {e}")
            return False

    def on_set_destination_cmd(self, msg: String):
        try:
            data = json.loads(msg.data)
            name = data.get("name", "")
            lat = float(data.get("lat", 0.0))
            lng = float(data.get("lng", 0.0))

            # 1. 교내 프리셋 경로 매칭
            if name in KEYWORD_TO_ROUTE:
                self.has_destination = True
                route_key = KEYWORD_TO_ROUTE[name]
                hardcoded_path = CAMPUS_ROUTES[route_key]
                waypoints = [
                    {"lat": p[0], "lon": p[1], "name": "교내 정밀 지점", "desc": f"목적지: {name}"}
                    for p in hardcoded_path
                ]
                with self.route_lock:
                    self.route_data["path"] = hardcoded_path
                    self.route_data["waypoints"] = waypoints
                    self.route_data["current_wp_idx"] = 0
                    self.route_data["next_wp_desc"] = waypoints[0]["desc"] if waypoints else ""
                self.publish_path_message(hardcoded_path, 0)
                self.get_logger().info(f"🎯 교내 정밀 실측 경로 설정: {name} (총 {len(hardcoded_path)} 포인트)")
                return

            # 2. 좌표 기반 T-Map API 호출
            if lat != 0.0 and lng != 0.0:
                self.dest_lat = lat
                self.dest_lon = lng
                self.has_destination = True
                with self.gps_lock:
                    cur_lat, cur_lon = self.gps_data["lat"], self.gps_data["lon"]
                ok = self.fetch_tmap_route(cur_lat, cur_lon)
                if not ok:
                    self.get_logger().warn("⚠️ T-Map 경로 생성에 실패했습니다.")
        except Exception as e:
            self.get_logger().error(f"목적지 설정 명령 파싱 오류: {e}")

    def gps_reader_worker(self):
        q_map = {
            "0": "유효하지 않음",
            "1": "Single GPS",
            "2": "DGPS",
            "4": "RTK FIX",
            "5": "RTK Float"
        }
        warned = False  # 연결 실패 경고는 끊길 때마다 1회만 출력 (1초마다 로그 폭주 방지)
        while self.running and rclpy.ok():
            ser = None
            try:
                ser = serial.Serial(self.gps_port, self.gps_baudrate, timeout=0.1)
                ser.reset_input_buffer()
                warned = False
                self.get_logger().info(f"✅ GPS 시리얼 연결 성공: {self.gps_port} @ {self.gps_baudrate} bps")

                while self.running and rclpy.ok() and ser.is_open:
                    line = ser.readline().decode("ascii", errors="ignore").strip()
                    if line.startswith(("$GNGGA", "$GPGGA")):
                        parts = line.split(",")
                        if len(parts) > 6:
                            lat = self.nmea_to_decimal(parts[2], parts[3])
                            lon = self.nmea_to_decimal(parts[4], parts[5])
                            quality = parts[6]
                            if lat is not None and lon is not None:
                                with self.gps_lock:
                                    self.gps_data.update({
                                        "lat": lat,
                                        "lon": lon,
                                        "quality": q_map.get(quality, "일반 GPS"),
                                        # main_fusion의 RTK 판정(4=FIX, 5=Float)용 숫자 품질값
                                        "fix_quality": int(quality) if quality.isdigit() else None,
                                        "is_real_gps": quality != "0",
                                    })
                                
                                # 토픽 발행
                                self.pub_gps_pos.publish(String(data=f"{lat},{lon}"))
                                self.pub_gps_status.publish(String(data=json.dumps(self.gps_data, ensure_ascii=False)))
                                
                                # 내비게이션 진행률 갱신
                                self.update_navigation(lat, lon)
            except Exception as e:
                if not warned:
                    self.get_logger().warn(f"⚠️ GPS 시리얼 연결 실패 ({self.gps_port}): {e}")
                    warned = True
                with self.gps_lock:
                    self.gps_data["is_real_gps"] = False
                    self.gps_data["fix_quality"] = None
                    self.gps_data["quality"] = "GPS 연결 끊김 (수신 대기)"
                if ser:
                    try: ser.close()
                    except Exception: pass
                time.sleep(1.0)

    def destroy_node(self):
        self.running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GpsTmapNavNode()
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
