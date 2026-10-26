#!/usr/bin/env python3
"""
web_dashboard.py
============================================================
통합 관제탑 웹 서버 및 실시간 텔레메트리/스트리밍 전용 노드
- Flask 웹 서버 (Port 5000)
- 4분할 사이버펑크 관제 대시보드 (Leaflet 지도, YOLO 카메라, 10m 레이더/SLAM, 디지털 클러스터)
- 실시간 카메라 비동기 Zero-Latency 스트리밍 (/video_feed)
- SLAM OccupancyGrid 맵의 실시간 Base64 PNG 변환
- 목적지 설정 명령을 ROS 2 토픽(/set_destination)으로 송출
============================================================
"""

import base64
import json
import math
import os
import threading
import time
import requests
import cv2
import numpy as np

import rclpy
from flask import Flask, Response, jsonify, render_template_string, request
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String

TMAP_APP_KEY = os.getenv("TMAP_APP_KEY", "")

# 전역 공유 상태 및 락
latest_yolo_frame = None
yolo_lock = threading.Lock()
new_frame_event = threading.Event()

latest_map = None
latest_scan = []
map_lock = threading.Lock()

gps_lock = threading.Lock()
route_lock = threading.Lock()
telemetry_lock = threading.Lock()

ros_node = None

gps_data = {
    "lat": 36.056400,
    "lon": 129.371500,
    "quality": "GPS 연결 대기 중",
    "is_real_gps": False,
}

route_data = {
    "path": [],
    "waypoints": [],
    "current_wp_idx": 0,
    "next_wp_desc": "지도 클릭 또는 버튼으로 목적지를 설정하세요.",
    "dist_to_next_wp": 0.0,
}

has_destination = False

telemetry_data = {
    "speed": 0.0,
    "gear": "STOP",
    "status_msg": "자율주행 대기",
    "pwm": 0.0,
    "steer": 0.0,
    "fusion_state": "WAITING_FOR_NAV",
    "imu_heading": 0.0,
    "imu_gz": 0.0,
    "imu_active": False,
    "gps_course": None,
    "cross_track_m": None,
    "lidar_front": 0.0,
    "vision": "CLEAR",
}

app = Flask(__name__)

INDEX_HTML = r"""
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>학교 자율주행 통합 관제탑</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#08080a;color:#fff;font-family:Segoe UI,sans-serif;overflow:hidden}
.grid{display:grid;grid-template-columns:1fr 1fr;grid-template-rows:1fr 1fr;width:100vw;height:100vh;gap:6px;padding:6px}
.panel{background:#0f0f13;border:1px solid #222;border-radius:8px;position:relative;overflow:hidden;box-shadow:0 0 15px rgba(0,255,255,0.05); display:flex; flex-direction:column;}
.header{height:28px;padding:6px 12px;background:linear-gradient(90deg, #111, #1a1a24);color:#00ffff;font-weight:bold;font-size:13px;position:relative;z-index:10;border-bottom:1px solid #333;text-shadow:0 0 5px rgba(0,255,255,0.5);flex-shrink:0;}
.content{flex:1;position:relative;overflow:hidden;}
#map{width:100%;height:100%; background:#0a0a0a;}
.leaflet-tile { filter: invert(100%) hue-rotate(180deg) brightness(95%) contrast(90%); }
.cam{width:100%;height:100%;object-fit:cover;background:#000}
#slam{width:100%;height:100%;display:block;background:#050508}

/* Cyberpunk Dashboard Styles */
.dash-container { width:100%; height:100%; display:flex; flex-direction:column; padding:15px; gap:15px; background: radial-gradient(circle at center, #15151a, #08080a); }
.dash-top { display:flex; align-items:center; justify-content:center; gap:30px; margin-bottom:5px; }

/* Ring Gauge for PWM */
.gauge-container { position:relative; width:120px; height:120px; display:flex; align-items:center; justify-content:center; border-radius:50%; background:#111; box-shadow: 0 0 20px rgba(0, 255, 102, 0.2); }
.gauge-ring { position:absolute; width:100%; height:100%; border-radius:50%; background: conic-gradient(#00ff66 0%, #222 0%); transition: background 0.2s ease-out; }
.gauge-center { position:absolute; width:100px; height:100px; border-radius:50%; background:#0f0f13; display:flex; flex-direction:column; align-items:center; justify-content:center; z-index:2; box-shadow: inset 0 0 12px rgba(0,0,0,0.9); }
.pwm-val { font-size:40px; font-weight:900; color:#00ff66; text-shadow:0 0 10px rgba(0,255,102,0.7); line-height:1; }
.pwm-unit { font-size:12px; color:#888; letter-spacing:1px; margin-top:4px; }

/* Status Badge & Mode */
.dash-status { display:flex; flex-direction:column; gap:10px; align-items:flex-start; }
.state-badge { font-size:18px; font-weight:bold; background:linear-gradient(90deg,#0055ff,#00aaff); padding:8px 24px; border-radius:4px; box-shadow:0 0 15px rgba(0,170,255,0.5); letter-spacing:1px; text-transform:uppercase; border: 1px solid #00ffff; }
.mode-text { font-size:14px; color:#0ff; font-weight:bold; text-shadow:0 0 5px rgba(0,255,255,0.5); }

/* Sensor LEDs */
.sensor-leds { display:flex; justify-content:center; gap:20px; background:rgba(0,0,0,0.5); padding:10px; border-radius:6px; border:1px solid #333; }
.led-item { display:flex; align-items:center; gap:8px; font-size:12px; color:#ccc; font-weight:bold; letter-spacing:1px; }
.led { width:12px; height:12px; border-radius:50%; background:#ff003c; box-shadow:0 0 10px #ff003c; transition: 0.3s; }

/* 2x2 Data Grid */
.data-grid { display:grid; grid-template-columns:1fr 1fr; grid-template-rows:1fr 1fr; gap:10px; flex:1; }
.data-card { background:rgba(255,255,255,0.03); border:1px solid #2a2a35; border-radius:6px; padding:12px; display:flex; flex-direction:column; justify-content:center; align-items:center; transition:0.2s; box-shadow: inset 0 0 20px rgba(0,0,0,0.5); }
.data-card:hover { border-color:#00ffff; background:rgba(0,255,255,0.05); }
.data-label { font-size:11px; color:#999; text-transform:uppercase; letter-spacing:1px; margin-bottom:6px; }
.data-value { font-size:24px; font-weight:bold; color:#fff; text-shadow:0 0 8px rgba(255,255,255,0.4); }

.nav-card{position:absolute;top:12px;left:50%;transform:translateX(-50%);z-index:1000;background:rgba(15,15,20,0.9);border:1px solid #00ffff;color:#eee;width:90%;max-width:390px;padding:10px;border-radius:8px;text-align:center;box-shadow:0 0 15px rgba(0,255,255,0.2)}
.nav-title{font-weight:bold;font-size:13px;color:#fff}
.nav-sub{font-size:12px;color:#00ffff;margin:5px}
.fast{display:flex;gap:5px}.fast button{flex:1;padding:7px;border:1px solid #00aaff;border-radius:4px;background:rgba(0,136,255,0.2);color:#00ffff;font-weight:bold;cursor:pointer;transition:0.2s}
.fast button:hover{background:#00aaff;color:#fff;box-shadow:0 0 10px #00aaff}
.search{display:flex;gap:5px;margin-top:5px}
.search input{flex:1;padding:6px;background:#111;border:1px solid #444;color:#fff;border-radius:4px}
.search button{padding:6px 10px;background:#00aa00;border:0;color:white;border-radius:4px;cursor:pointer}
.gps{position:absolute;bottom:7px;left:7px;z-index:1000;background:rgba(0,0,0,.8);padding:5px 8px;border-radius:4px;font-size:10px;color:#00ff00}
.slam-label{position:absolute;right:8px;top:8px;z-index:5;background:rgba(0,0,0,.7);padding:5px 7px;border-radius:4px;font-size:10px;color:#0ff;border:1px solid #0ff}
.legend{position:absolute;left:8px;bottom:8px;z-index:5;background:rgba(0,0,0,.7);padding:6px 8px;border-radius:4px;font-size:10px;color:#aaa}
</style>
</head>
<body>
<div class="grid">
<div class="panel">
<div class="header">내비게이션 / 하이브리드 경로 추적</div>
<div class="content">
<div class="nav-card">
<div class="nav-title" id="nav-desc">목적지 설정</div>
<div class="nav-sub" id="nav-dist">대기 중...</div>
<div class="fast">
<button onclick="setHybridDestination('정문')">정문</button>
<button onclick="setHybridDestination('체육관')">체육관</button>
<button onclick="setHybridDestination('급식소')">급식소</button>
</div>
<div class="search">
<input id="keyword" placeholder="외부 장소 검색 (교내는 자동 매칭)">
<button onclick="searchPoi()">검색</button>
</div>
</div>
<div class="gps" id="gps-info">GPS 대기</div>
<div id="map"></div>
</div>
</div>
<div class="panel">
<div class="header">YOLO 실시간 AI 비전</div>
<div class="content"><img src="/video_feed" class="cam"></div>
</div>
<div class="panel">
<div class="header">Cyberpunk Radar / SLAM Map (10.0m Scale)</div>
<div class="content">
<canvas id="slam"></canvas>
<div class="slam-label">HD RADAR SYS</div>
<div class="legend">자동 고정 | 반경 10m | 거리별 위험도 색상</div>
</div>
</div>
<div class="panel">
<div class="header">차량 상태 디지털 클러스터</div>
<div class="content">
    <div class="dash-container">
        <div class="dash-top">
            <div class="gauge-container">
                <div class="gauge-ring" id="pwm-ring"></div>
                <div class="gauge-center">
                    <span class="pwm-val" id="pwm">0</span>
                    <span class="pwm-unit">PWM</span>
                </div>
            </div>
            <div class="dash-status">
                <div class="state-badge" id="state">WAITING</div>
                <div class="mode-text" id="mode">자율주행 대기 중</div>
            </div>
        </div>
        
        <div class="sensor-leds">
            <div class="led-item"><span class="led" id="led-gps"></span> GPS</div>
            <div class="led-item"><span class="led" id="led-lidar"></span> LiDAR</div>
            <div class="led-item"><span class="led" id="led-imu"></span> IMU</div>
            <div class="led-item"><span class="led" id="led-cam"></span> YOLO</div>
        </div>

        <div class="data-grid">
            <div class="data-card">
                <div class="data-label">방위각 (Heading)</div>
                <div class="data-value" id="imu-val">--°</div>
            </div>
            <div class="data-card">
                <div class="data-label">경로 편차 (CTE)</div>
                <div class="data-value" id="cte-val">-- m</div>
            </div>
            <div class="data-card">
                <div class="data-label">전방 라이다 (0°)</div>
                <div class="data-value" id="lidar-val">-- m</div>
            </div>
            <div class="data-card">
                <div class="data-label">YOLO 인식</div>
                <div class="data-value" id="vision-val">--</div>
            </div>
        </div>
    </div>
</div>
</div>
</div>
<script>
const map=L.map('map').setView([36.0564, 129.3715], 18);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19, attribution:'&copy; OpenStreetMap'}).addTo(map);

let interacting=false;
map.on('mousedown dragstart zoomstart',()=>interacting=true);
map.on('mouseup zoomend',()=>setTimeout(()=>interacting=false,3000));
const icon=L.divIcon({html:'<div style="font-size:24px;color:#00ff00;text-shadow:0 0 10px #00ff00;">▲</div>',className:'',iconSize:[24,24],iconAnchor:[12,12]});
const robot=L.marker([36.0564, 129.3715],{icon}).addTo(map);
const route=L.polyline([],{color:'#00ffff',weight:4,opacity:.8}).addTo(map);
let dest=null;

function setHybridDestination(name) {
    if(dest) map.removeLayer(dest);
    fetch('/api/set_destination',{
        method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({name: name, lat: 0, lng: 0})
    });
}

function setDestination(lat,lng,name){
  if(dest)map.removeLayer(dest);
  dest=L.marker([lat,lng]).addTo(map).bindPopup('목적지: '+name).openPopup();
  fetch('/api/set_destination',{
    method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({lat:lat,lng:lng,name:name})
  });
}
map.on('click',e=>setDestination(e.latlng.lat,e.latlng.lng,'사용자 지정'));

function searchPoi(){
  const kw=document.getElementById('keyword').value.trim(); if(!kw)return;
  if(["정문", "체육관", "급식소", "삼거리"].includes(kw)) {
      setHybridDestination(kw);
      return;
  }
  fetch('/api/search_poi',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({keyword:kw})}).then(r=>r.json()).then(d=>{
    const result=d.results||[];
    if(result.length) setDestination(result[0].lat,result[0].lon,result[0].name);
  });
}

const canvas=document.getElementById('slam');
function resizeSlam(){canvas.width=canvas.clientWidth;canvas.height=canvas.clientHeight;}
window.addEventListener('resize',resizeSlam);resizeSlam();

// --- 60FPS Cyberpunk Radar Rendering Engine (10.0m Scale) ---
let currentScan = [];
let currentMapImage = null;
let currentMapInfo = null;
let currentMapB64 = "";
let radarAngle = 0;

function renderRadar() {
    const ctx = canvas.getContext('2d');
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    
    // 최대 반경 10.0m 설정
    const maxDist = 10.0; 
    const maxRadius = Math.min(cx, cy) - 20;
    const ppm = maxRadius / maxDist; 

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    if (currentMapImage && currentMapInfo) {
        ctx.save();
        const w = currentMapInfo.height * currentMapInfo.resolution * ppm; 
        const h = currentMapInfo.width * currentMapInfo.resolution * ppm;
        const top_left_x = cx - (currentMapInfo.origin.position.y + currentMapInfo.height * currentMapInfo.resolution) * ppm;
        const top_left_y = cy - (currentMapInfo.origin.position.x + currentMapInfo.width * currentMapInfo.resolution) * ppm;
        
        ctx.imageSmoothingEnabled = false;
        ctx.globalAlpha = 0.8; 
        ctx.drawImage(currentMapImage, top_left_x, top_left_y, w, h);
        ctx.restore();
    }

    // 180도 화각 부채꼴 & 후방 음영
    ctx.save();
    ctx.translate(cx, cy);
    ctx.beginPath();
    ctx.arc(0, 0, maxRadius + 15, Math.PI, 2 * Math.PI);
    ctx.fillStyle = 'rgba(0, 255, 180, 0.03)';
    ctx.fill();
    ctx.beginPath();
    ctx.arc(0, 0, maxRadius + 15, 0, Math.PI);
    ctx.fillStyle = 'rgba(0, 0, 0, 0.6)';
    ctx.fill();
    ctx.restore();

    // 동심원 (2.5m, 5.0m, 7.5m, 10.0m)
    ctx.save();
    ctx.translate(cx, cy);
    ctx.strokeStyle = 'rgba(0, 255, 255, 0.2)';
    ctx.fillStyle = 'rgba(0, 255, 255, 0.7)';
    ctx.font = '10px "Segoe UI"';
    ctx.textAlign = 'center';
    
    [2.5, 5.0, 7.5, 10.0].forEach(r => {
        let rad = (r / maxDist) * maxRadius;
        ctx.beginPath();
        ctx.arc(0, 0, rad, 0, 2 * Math.PI);
        ctx.setLineDash(r === 10.0 ? [] : [3, 4]); 
        ctx.stroke();
        ctx.fillText(r + 'M', 0, -rad - 4);
    });
    ctx.restore();

    // 방위각 가이드선
    ctx.save();
    ctx.translate(cx, cy);
    ctx.strokeStyle = 'rgba(0, 255, 255, 0.15)';
    ctx.setLineDash([2, 5]);
    ctx.beginPath();
    const angles = [-Math.PI/2, -Math.PI/4, 0, Math.PI/4, Math.PI/2];
    angles.forEach(a => {
        let ca = a - Math.PI/2; 
        ctx.moveTo(0, 0);
        ctx.lineTo(Math.cos(ca) * (maxRadius + 15), Math.sin(ca) * (maxRadius + 15));
    });
    ctx.stroke();
    ctx.restore();

    // 스위프 애니메이션
    radarAngle -= 0.04; 
    if(radarAngle < 0) radarAngle = 2 * Math.PI;
    
    ctx.save();
    ctx.translate(cx, cy);
    for(let i=0; i<30; i++) {
        let a = radarAngle + (i * 0.015);
        ctx.beginPath();
        ctx.moveTo(0,0);
        ctx.arc(0,0, maxRadius+15, a, a+0.02);
        ctx.fillStyle = `rgba(0, 255, 150, ${0.15 - (i*0.005)})`;
        ctx.fill();
    }
    ctx.beginPath();
    ctx.moveTo(0,0);
    ctx.lineTo(Math.cos(radarAngle)*(maxRadius+15), Math.sin(radarAngle)*(maxRadius+15));
    ctx.strokeStyle = 'rgba(0, 255, 200, 0.8)';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.restore();

    // LiDAR 스캔 포인트 (10.0m 스케일 변환 및 네온 효과)
    ctx.save();
    ctx.translate(cx, cy);
    for(const p of currentScan) {
        const dist = p[0]; 
        const angle = p[1]; 
        
        if(dist <= 0.0 || dist > maxDist || !isFinite(dist)) continue;

        const x = dist * Math.cos(angle); 
        const y = dist * Math.sin(angle); 

        const ratio = maxRadius / maxDist;
        const px = -y * ratio;
        const py = -x * ratio;

        let color = '#00ffff'; 
        if(dist <= 1.5) color = '#ff003c'; 
        else if (dist <= 3.0) color = '#ffea00'; 

        ctx.shadowBlur = 10; 
        ctx.shadowColor = color;
        ctx.fillStyle = color;
        
        ctx.beginPath();
        ctx.arc(px, py, 2.5, 0, 2 * Math.PI);
        ctx.fill();
    }
    ctx.restore();

    // 로봇 아이콘 (Aviation Arrow)
    ctx.save();
    ctx.translate(cx, cy);
    ctx.shadowBlur = 15;
    ctx.shadowColor = '#00ff66';
    ctx.fillStyle = '#00ff66';
    ctx.beginPath();
    ctx.moveTo(0, -14);   
    ctx.lineTo(-9, 10);   
    ctx.lineTo(0, 4);     
    ctx.lineTo(9, 10);    
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    requestAnimationFrame(renderRadar);
}
requestAnimationFrame(renderRadar);

function setLedStatus(id, isActive) {
    const led = document.getElementById(id);
    if(isActive) {
        led.style.background = '#00ff66';
        led.style.boxShadow = '0 0 10px #00ff66';
    } else {
        led.style.background = '#ff003c';
        led.style.boxShadow = '0 0 10px #ff003c';
    }
}

function update(){
  fetch('/api/data').then(r=>r.json()).then(d=>{
    const g=d.gps,r=d.route,t=d.telemetry;
    robot.setLatLng([g.lat,g.lon]);
    if(!interacting)map.panTo([g.lat,g.lon]);
    
    if(r.path && r.path.length > 0) { route.setLatLngs(r.path); } else { route.setLatLngs([]); }
    
    document.getElementById('nav-desc').innerText=r.next_wp_desc;
    document.getElementById('nav-dist').innerText=d.has_dest?('다음 지점까지 '+r.dist_to_next_wp+'m'):'대기 중...';
    document.getElementById('gps-info').innerText=g.quality+' | '+Number(g.lat).toFixed(6)+', '+Number(g.lon).toFixed(6);
    
    // PWM Gauge
    const pwmVal = Number(t.pwm||t.speed_pwm||0).toFixed(0);
    document.getElementById('pwm').innerText = pwmVal;
    const pwmPercent = Math.min(100, Math.max(0, (pwmVal / 255) * 100));
    document.getElementById('pwm-ring').style.background = `conic-gradient(#00ff66 ${pwmPercent}%, #222 ${pwmPercent}%)`;

    document.getElementById('state').innerText=t.state||t.fusion_state||'WAITING';
    document.getElementById('mode').innerText=t.status_msg||'--';
    
    // IMU Data & FAULT UI Sync
    const imuEl = document.getElementById('imu-val');
    if(!t.imu_active) {
        imuEl.innerText = "FAULT";
        imuEl.style.color = "#ff003c";
        imuEl.style.textShadow = "0 0 8px rgba(255,0,60,0.6)";
    } else {
        imuEl.innerText = (t.imu_heading==null?'--':Number(t.imu_heading).toFixed(1)+'°');
        imuEl.style.color = "#fff";
        imuEl.style.textShadow = "0 0 8px rgba(255,255,255,0.4)";
    }

    document.getElementById('cte-val').innerText=(t.cross_track_m==null?'--':Number(t.cross_track_m).toFixed(2)+' m');
    
    const lidarFront = Number(t.lidar_front||0);
    const lidarEl = document.getElementById('lidar-val');
    lidarEl.innerText = lidarFront > 0 ? lidarFront.toFixed(2)+' m' : '--';
    if(lidarFront > 0 && lidarFront <= 1.5) { lidarEl.style.color = '#ff003c'; lidarEl.style.textShadow = '0 0 8px rgba(255,0,60,0.6)'; }
    else if(lidarFront > 1.5 && lidarFront <= 3.0) { lidarEl.style.color = '#ffea00'; lidarEl.style.textShadow = '0 0 8px rgba(255,234,0,0.6)'; }
    else { lidarEl.style.color = '#fff'; lidarEl.style.textShadow = '0 0 5px rgba(255,255,255,0.3)'; }

    document.getElementById('vision-val').innerText=(t.vision||'CLEAR');

    // Sensor LEDs Update
    setLedStatus('led-gps', g.is_real_gps);
    setLedStatus('led-lidar', lidarFront > 0 && lidarFront < 12.0);
    setLedStatus('led-imu', t.imu_active); 
    setLedStatus('led-cam', t.vision !== null && t.vision !== '');
    
    currentScan = d.scan || [];
    if(d.slam_map && d.slam_map.image_b64) {
        if(currentMapB64 !== d.slam_map.image_b64) {
            currentMapB64 = d.slam_map.image_b64;
            const img = new Image();
            img.onload = () => { currentMapImage = img; currentMapInfo = d.slam_map.info; };
            img.src = "data:image/png;base64," + currentMapB64;
        }
    }
  }).catch(()=>{});
}
setInterval(update,500);
update();
</script>
</body>
</html>
"""

class WebDashboardNode(Node):
    def __init__(self):
        super().__init__("web_dashboard_node")

        # 퍼블리셔 (사용자가 웹에서 선택한 목적지 전달)
        self.pub_dest = self.create_publisher(String, "/set_destination", 10)

        # 서브스크라이버
        self.create_subscription(String, "/gps_status", self.gps_status_callback, 10)
        self.create_subscription(String, "/gps_pos", self.gps_pos_callback, 10)
        self.create_subscription(String, "/nav_path_data", self.nav_path_callback, 10)
        self.create_subscription(CompressedImage, "/yolo_stream", self.cam_callback, 10)
        self.create_subscription(LaserScan, "/scan_filtered", self.scan_callback, qos_profile_sensor_data)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.create_subscription(String, "/vehicle_status", self.status_callback, 10)
        self.create_subscription(String, "/imu_detail", self.imu_detail_callback, 10)

        self.last_imu_time = 0.0
        self.get_logger().info("✅ Web Dashboard 관제탑 ROS 2 노드 구동 완료")

    def gps_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            with gps_lock:
                gps_data.update(data)
        except Exception:
            pass

    def gps_pos_callback(self, msg: String):
        try:
            parts = msg.data.split(",")
            lat, lon = float(parts[0]), float(parts[1])
            with gps_lock:
                gps_data["lat"] = lat
                gps_data["lon"] = lon
        except Exception:
            pass

    def nav_path_callback(self, msg: String):
        global has_destination
        try:
            data = json.loads(msg.data)
            with route_lock:
                route_data["path"] = data.get("path", [])
                route_data["waypoints"] = data.get("waypoints", [])
                route_data["current_wp_idx"] = data.get("target_idx", 0)
                route_data["next_wp_desc"] = data.get("next_wp_desc", "")
                route_data["dist_to_next_wp"] = data.get("dist_to_next_wp", 0.0)
                has_destination = bool(data.get("has_dest", len(route_data["path"]) > 0))
        except Exception:
            pass

    def cam_callback(self, msg: CompressedImage):
        global latest_yolo_frame
        with yolo_lock:
            latest_yolo_frame = bytes(msg.data)
        new_frame_event.set()

    def scan_callback(self, msg: LaserScan):
        global latest_scan
        pts = []
        angle = msg.angle_min
        inc = msg.angle_increment if msg.angle_increment > 0 else 0.005 
        num_points = len(msg.ranges)
        step = max(1, num_points // 150) if num_points > 0 else 1

        for i, dist in enumerate(msg.ranges):
            if i % step == 0 and math.isfinite(dist) and 0.1 <= dist <= 10.0:
                pts.append([round(dist, 3), round(angle, 3)])
            angle += inc
        latest_scan = pts

    def map_callback(self, msg: OccupancyGrid):
        global latest_map
        w = msg.info.width
        h = msg.info.height
        if w == 0 or h == 0: return

        try:
            arr = np.array(msg.data, dtype=np.int8).reshape((h, w))
            img_rgba = np.zeros((h, w, 4), dtype=np.uint8)

            free_mask = (arr >= 0) & (arr <= 20)
            img_rgba[free_mask] = [30, 40, 50, 100]

            occ_mask = arr > 65
            img_rgba[occ_mask] = [255, 255, 0, 255]

            img_rot = np.rot90(img_rgba, k=1)
            img_final = np.fliplr(img_rot)

            _, buffer = cv2.imencode('.png', img_final)
            b64_str = base64.b64encode(buffer).decode('utf-8')

            with map_lock:
                latest_map = {
                    "info": {
                        "width": w,
                        "height": h,
                        "resolution": float(msg.info.resolution),
                        "origin": {
                            "position": {
                                "x": float(msg.info.origin.position.x),
                                "y": float(msg.info.origin.position.y),
                            }
                        },
                    },
                    "image_b64": b64_str,
                }
        except Exception as e:
            self.get_logger().error(f"SLAM Map 렌더링 오류: {e}")

    def status_callback(self, msg: String):
        try:
            obj = json.loads(msg.data)
            with telemetry_lock:
                telemetry_data.update(obj)
                if "speed_pwm" in obj: telemetry_data["pwm"] = obj["speed_pwm"]
                if "state" in obj: telemetry_data["fusion_state"] = obj["state"]
                if "lidar_front" in obj: telemetry_data["lidar_front"] = obj["lidar_front"]
        except Exception:
            pass

    def imu_detail_callback(self, msg: String):
        try:
            self.last_imu_time = time.time()
            obj = json.loads(msg.data.replace("'", '"'))
            with telemetry_lock:
                if "heading_deg" in obj: telemetry_data["imu_heading"] = obj["heading_deg"]
                if "heading_rate_dps" in obj: telemetry_data["imu_gz"] = obj["heading_rate_dps"]
        except Exception:
            pass


# Zero-Latency 비동기 제너레이터
def generate_camera_stream():
    global latest_yolo_frame
    while True:
        if new_frame_event.wait(timeout=0.5):
            new_frame_event.clear()
            with yolo_lock:
                frame = latest_yolo_frame
            if frame is not None:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")

@app.route("/")
def index():
    return render_template_string(INDEX_HTML)

@app.route("/video_feed")
def video_feed():
    return Response(generate_camera_stream(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/api/data")
def api_data():
    with gps_lock: gps = dict(gps_data)
    with route_lock: route = dict(route_data)
    with telemetry_lock:
        tele = dict(telemetry_data)
        if ros_node is not None:
            tele["imu_active"] = (time.time() - ros_node.last_imu_time) <= 1.0
        else:
            tele["imu_active"] = False

    with map_lock: slam_map = latest_map
    return jsonify({
        "gps": gps,
        "route": route,
        "has_dest": has_destination,
        "telemetry": tele,
        "slam_map": slam_map,
        "scan": latest_scan,
    })

@app.route("/api/set_destination", methods=["POST"])
def set_destination_api():
    data = request.json or {}
    if ros_node is not None:
        ros_node.pub_dest.publish(String(data=json.dumps(data, ensure_ascii=False)))
        return jsonify({"status": "ok"})
    return jsonify({"status": "error", "msg": "ROS 노드 미구동"})

@app.route("/api/search_poi", methods=["POST"])
def search_poi():
    keyword = (request.json or {}).get("keyword", "").strip()
    if not keyword or not TMAP_APP_KEY: return jsonify({"results": []})
    
    params = {
        "version": 1, "searchKeyword": keyword, "count": 5,
        "reqCoordType": "WGS84GEO", "resCoordType": "WGS84GEO", "searchCoordType": "WGS84GEO"
    }
    try:
        r = requests.get(
            "https://apis.openapi.sk.com/tmap/pois",
            headers={"appKey": TMAP_APP_KEY},
            params=params,
            timeout=5,
        )
        r.raise_for_status()
        pois = r.json().get("searchPoiInfo", {}).get("pois", {}).get("poi", [])
        results = []
        for p in pois:
            try:
                lat = float(p.get("frontLat") or p.get("noorLat"))
                lon = float(p.get("frontLon") or p.get("noorLon"))
            except Exception: continue
            results.append({"name": p.get("name"), "lat": lat, "lon": lon})
        return jsonify({"results": results})
    except Exception:
        return jsonify({"results": []})

def run_ros2():
    global ros_node
    rclpy.init()
    ros_node = WebDashboardNode()
    try:
        rclpy.spin(ros_node)
    finally:
        ros_node.destroy_node()
        ros_node = None
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    threading.Thread(target=run_ros2, daemon=True).start()
    threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False, threaded=True),
        daemon=True
    ).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
