#!/usr/bin/env python3
import glob
import os
import threading
import time
import cv2
import numpy as np
import rclpy
import torch
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from ultralytics import YOLO

class YoloVisionNode(Node):
    def __init__(self):
        super().__init__("yolo_vision_node")
        # ---- 파라미터 선언 ----
        self.declare_parameter("model_path", "yolo26n.pt")
        self.declare_parameter("camera_id", "/dev/robot_cam")
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("camera_fps", 30.0)
        self.declare_parameter("conf_thresh", 0.45)
        self.declare_parameter("stop_ratio", 0.40)
        self.declare_parameter("near_ratio", 0.15)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("jpeg_quality", 70)  # 품질 70% 로 낮춤
        self.declare_parameter("center_roi_half_width", 0.30)
        self.declare_parameter("status_confirm_frames", 2)

        # ---- 파라미터 로드 ----
        self.model_path = str(self.get_parameter("model_path").value)
        cam = self.get_parameter("camera_id").value
        cam_text = str(cam)
        self.camera_id = int(cam_text) if cam_text.isdigit() else cam_text

        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.camera_fps = float(self.get_parameter("camera_fps").value)
        self.conf_thresh = float(self.get_parameter("conf_thresh").value)
        self.stop_ratio = float(self.get_parameter("stop_ratio").value)
        self.near_ratio = float(self.get_parameter("near_ratio").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.roi_half = float(self.get_parameter("center_roi_half_width").value)
        self.confirm_frames = int(self.get_parameter("status_confirm_frames").value)

        # ---- 퍼블리셔 ----
        self.pub_status = self.create_publisher(String, "/vision_info", 10)
        self.pub_img = self.create_publisher(CompressedImage, "/yolo_stream", 10)

        # ---- 디바이스 설정 ----
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.use_half = self.device == "cuda"
        self.get_logger().info(f"🚀 YOLO 로딩: {self.model_path} | device={self.device}")

        # ---- 모델 로드 ----
        self.model = YOLO(self.model_path)
        self.model.to(self.device)

        # ---- 카메라 초기화 ----
        self.cap = None
        self.running = True
        self.status_candidate = "CLEAR"
        self.status_count = 0
        self.stable_status = "CLEAR"

        # 비동기 카메라 프레임 버퍼
        self.latest_raw_frame = None
        self.frame_lock = threading.Lock()

        self.init_camera()

        # 최신 프레임만 유지하는 독립 스레드
        self.reader_thread = threading.Thread(target=self.camera_reader_loop, daemon=True)
        self.reader_thread.start()

        # YOLO 추론 및 스트리밍 타이머
        self.timer = self.create_timer(1.0 / publish_rate, self.detect_and_stream)

    def test_camera_port(self, port):
        cap = None
        try:
            cap = cv2.VideoCapture(port, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                cap = cv2.VideoCapture(port)
            if not cap.isOpened():
                return False, None
            # ---- 오래된 프레임 스택 방지 ----
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            try:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            except:
                pass
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
            for _ in range(3):
                ret, frame = cap.read()
                if ret and frame is not None:
                    return True, cap
                time.sleep(0.05)
            cap.release()
            return False, None
        except Exception:
            if cap is not None:
                cap.release()
            return False, None

    def find_available_camera(self):
        scan_ports = [self.camera_id]
        for i in range(11):
            if f"/dev/video{i}" not in scan_ports:
                scan_ports.append(f"/dev/video{i}")
        for port in scan_ports:
            if isinstance(port, str) and not os.path.exists(port):
                continue
            ok, cap = self.test_camera_port(port)
            if ok:
                self.camera_id = port
                return cap
        return None

    def init_camera(self):
        if self.cap is not None:
            self.cap.release()
        cap = self.find_available_camera()
        if cap is not None:
            self.cap = cap
            self.get_logger().info(f"✅ 카메라 연결 됨: {self.camera_id}")
        else:
            self.get_logger().error("❌ 카메라를 찾을 수 없습니다.")

    def camera_reader_loop(self):
        """V4L2 큐에 쌓인 과거 프레임을 버리고 최신 프레임 1장만 유지한다."""
        while self.running and rclpy.ok():
            if self.cap is None or not self.cap.isOpened():
                time.sleep(1.0)
                self.init_camera()
                continue
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.frame_lock:
                    self.latest_raw_frame = frame
            else:
                time.sleep(0.01)

    def stable_status_update(self, candidate):
        if candidate == self.stable_status:
            self.status_candidate = candidate
            self.status_count = 0
            return self.stable_status
        if candidate != self.status_candidate:
            self.status_candidate = candidate
            self.status_count = 1
            return self.stable_status
        self.status_count += 1
        if self.status_count >= self.confirm_frames:
            self.stable_status = candidate
            self.status_count = 0
        return self.stable_status

    def detect_and_stream(self):
        # 최신 프레임 확보 (락 최소화)
        with self.frame_lock:
            if self.latest_raw_frame is None:
                return
            frame = self.latest_raw_frame.copy()
            self.latest_raw_frame = None

        # YOLO 추론
        try:
            results = self.model(
                frame,
                classes=[0, 2],
                conf=self.conf_thresh,
                half=self.use_half,
                verbose=False,
                device=self.device,
            )
        except Exception:
            return

        result = results[0]
        annotated = result.plot()

        # 상태 판단 (원본 로직 그대로)
        frame_h, frame_w = frame.shape[:2]
        roi_left = int(frame_w * (0.5 - self.roi_half))
        roi_right = int(frame_w * (0.5 + self.roi_half))
        max_person_ratio = 0.0
        has_car_center = False
        for box in result.boxes:
            cls_id = int(box.cls[0].item())
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
            cx = 0.5 * (x1 + x2)
            ratio = max(0.0, y2 - y1) / max(1, frame_h)
            in_center = roi_left <= cx <= roi_right
            if cls_id == 0 and in_center:
                max_person_ratio = max(max_person_ratio, ratio)
            elif cls_id == 2 and in_center:
                has_car_center = True
        candidate = "CLEAR"
        if max_person_ratio >= self.stop_ratio:
            candidate = "PERSON_FRONT"
        elif max_person_ratio >= self.near_ratio:
            candidate = "PERSON_NEAR"
        elif has_car_center:
            candidate = "CAR"
        status = self.stable_status_update(candidate)
        self.pub_status.publish(String(data=status))

        # JPEG 인코딩 (품질 70)
        try:
            ok, buffer = cv2.imencode(
                ".jpg",
                annotated,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if ok and buffer is not None:
                msg = CompressedImage()
                msg.format = "jpeg"
                msg.data = np.asarray(buffer).tobytes()
                self.pub_img.publish(msg)
        except Exception:
            pass

    def destroy_node(self):
        self.running = False
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloVisionNode()
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
