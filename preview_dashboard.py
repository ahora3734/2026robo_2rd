#!/usr/bin/env python3
"""
대시보드 로컬 미리보기 서버 (ROS 2 / Flask 종속성 없음)
- tmap.py의 Cyberpunk UI와 완전 동일한 화면 제공
- 시뮬레이션 센서 데이터 (GPS, LiDAR 180도 스캔, IMU, YOLO, PWM) 실시간 스트리밍
"""
import base64
import http.server
import json
import math
import os
import socketserver
import threading
import time
import urllib.parse
import webbrowser

PORT = 5000

# 교내 정밀 실측 GPS 데이터
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
        [36.0555655, 129.3722284], [36.0552040, 129.3722603], [36.0554978, 129.3722742],
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

# 현재 선택된 경로
active_route_name = "삼거리"
active_path = CAMPUS_ROUTES["START_TO_INTERSECTION"]
active_idx = 0
start_time = time.time()

# 카메라 이미지 로드 (가상 대시캠)
CAM_IMAGE_PATH = r"C:\Users\포항고\.gemini\antigravity-ide\brain\763a48c7-85b5-4e7b-b55f-cfba444ea1cc\gokart_dash_cam_1793005374184.jpg"
cam_image_bytes = b""
if os.path.exists(CAM_IMAGE_PATH):
    with open(CAM_IMAGE_PATH, "rb") as f:
        cam_image_bytes = f.read()

# web_dashboard.py (또는 tmap.py)의 INDEX_HTML 직접 로드
INDEX_HTML = ""
for fname, end_m in [("web_dashboard.py", '"""\n\nclass WebDashboardNode'), ("tmap.py", '"""\n\nclass TmapRosBridge')]:
    fpath = os.path.join(os.path.dirname(__file__), fname)
    if os.path.exists(fpath):
        try:
            with open(fpath, "r", encoding="utf-8") as f:
                content = f.read()
                start_marker = 'INDEX_HTML = r"""'
                s = content.find(start_marker)
                e = content.find(end_m)
                if s != -1 and e != -1:
                    INDEX_HTML = content[s + len(start_marker):e]
                    break
        except Exception:
            pass

class DashboardHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # 콘솔 로그 간소화
        pass

    def do_GET(self):
        global active_route_name, active_path, active_idx
        parsed = urllib.parse.urlparse(self.path)

        if parsed.path == "/" or parsed.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(INDEX_HTML.encode("utf-8"))
            return

        elif parsed.path == "/video_feed":
            # MJPEG 스트림 형태로 카메라 프레임 전송
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                for _ in range(50):
                    if cam_image_bytes:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + cam_image_bytes + b"\r\n")
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        elif parsed.path == "/api/data":
            # 실시간 자율주행 시뮬레이션 데이터 반환
            elapsed = time.time() - start_time
            
            # 경로 상에서 차량이 천천히 이동하는 효과
            if active_path:
                idx = int((elapsed * 0.8) % len(active_path))
                current_lat, current_lon = active_path[idx]
            else:
                current_lat, current_lon = 36.0564382, 129.3717111

            # 180도 가상 라이다 포인트 클라우드 생성 (전방 도로/장애물 패턴)
            scan_pts = []
            for deg in range(-90, 91, 2):
                rad = math.radians(deg)
                # 전방은 뻥 뚫려 있고(5~8m), 좌우 양쪽은 연석/벽(1.5~3m) 형태 시뮬레이션
                base_dist = 6.5 - 3.5 * (abs(deg) / 90.0)
                noise = 0.15 * math.sin(deg * 0.2 + elapsed * 2.0)
                dist = max(0.5, round(base_dist + noise, 3))
                scan_pts.append([dist, round(rad, 3)])

            # 방위각 및 조향각 계산
            heading = (135.0 + 8.0 * math.sin(elapsed * 0.5)) % 360.0
            steer = round(5.2 * math.sin(elapsed * 0.8), 1)
            pwm = int(140 + 15 * math.sin(elapsed * 0.3))

            response_data = {
                "gps": {
                    "lat": current_lat,
                    "lon": current_lon,
                    "quality": "RTK FIX",
                    "is_real_gps": True
                },
                "route": {
                    "path": active_path,
                    "waypoints": [{"lat": p[0], "lon": p[1], "name": f"WP", "desc": f"목적지: {active_route_name}"} for p in active_path],
                    "current_wp_idx": min(len(active_path) - 1, int(elapsed * 0.8) % len(active_path)),
                    "next_wp_desc": f"[{active_route_name}] 정밀 자율주행 경로 추종 중",
                    "dist_to_next_wp": round(max(0.5, 15.0 - (elapsed * 0.8) % 15.0), 1)
                },
                "has_dest": True,
                "telemetry": {
                    "speed": 18.5,
                    "gear": "D",
                    "pwm": pwm,
                    "steer": steer,
                    "steer_cur": steer,
                    "pot_raw": int(2048 + (steer / 35.0) * 1400),
                    "state": "NAVIGATE",
                    "fusion_state": "NAVIGATE",
                    "status_msg": "정상 경로 추종 (가변저항 피드백 정렬 완료)",
                    "imu_heading": round(heading, 1),
                    "imu_raw_heading": round((heading - 40.0) % 360.0, 1),
                    "imu_gz": round(steer * 0.5, 2),
                    "imu_active": True,
                    "gps_course": round(heading, 1),
                    "cross_track_m": round(0.06 * math.sin(elapsed), 2),
                    "lidar_front": 5.45,
                    "vision": "CLEAR"
                },
                "slam_map": None,
                "scan": scan_pts
            }

            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(response_data, ensure_ascii=False).encode("utf-8"))
            return

        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        global active_route_name, active_path, start_time
        parsed = urllib.parse.urlparse(self.path)

        if parsed.path == "/api/set_destination":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length).decode("utf-8")
            try:
                data = json.loads(body)
                name = data.get("name", "")
                if name in KEYWORD_TO_ROUTE:
                    active_route_name = name
                    active_path = CAMPUS_ROUTES[KEYWORD_TO_ROUTE[name]]
                    start_time = time.time()
                    resp = {"status": "ok", "type": "hardcoded", "name": name}
                else:
                    active_route_name = "사용자 지정"
                    start_time = time.time()
                    resp = {"status": "ok", "type": "custom"}
            except Exception as e:
                resp = {"status": "error", "msg": str(e)}

            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.end_headers()
            self.wfile.write(json.dumps(resp, ensure_ascii=False).encode("utf-8"))
            return

        elif parsed.path == "/api/search_poi":
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.end_headers()
            self.wfile.write(json.dumps({"results": []}).encode("utf-8"))
            return

        self.send_response(404)
        self.end_headers()

class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

import sys
if hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

def create_server(preferred_port=5000):
    for p in range(preferred_port, preferred_port + 10):
        try:
            srv = ThreadedHTTPServer(("127.0.0.1", p), DashboardHandler)
            return srv, p
        except OSError:
            continue
    raise RuntimeError("5000~5009 포트를 모두 사용할 수 없습니다.")

def open_browser(url):
    time.sleep(0.4)
    opened = False
    try:
        opened = webbrowser.open(url, new=2)
    except Exception:
        pass
    if not opened and os.name == 'nt':
        try:
            os.system(f'start "" "{url}"')
        except Exception:
            pass

if __name__ == "__main__":
    try:
        server, actual_port = create_server(PORT)
    except Exception as e:
        print(f"❌ 서버 시작 실패: {e}")
        sys.exit(1)

    url = f"http://127.0.0.1:{actual_port}"
    print(f"\n========================================================")
    print(f"🚀 [자율주행 대시보드 미리보기 서버 가동]")
    print(f"🌐 접속 URL: {url}")
    print(f"👉 브라우저가 자동으로 뜨지 않으면 위 링크를 클릭하세요.")
    print(f"🛑 종료하려면 터미널에서 Ctrl + C 를 누르세요.")
    print(f"========================================================\n")

    threading.Thread(target=open_browser, args=(url,), daemon=True).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n서버가 안전하게 종료되었습니다.")


