#!/usr/bin/env python3
import glob
import json
import os
import sys
import time
import select
import termios
import tty
import serial

# ==========================================
# 설정 및 파라미터
# ==========================================
SERIAL_PORT = "/dev/esp32"
BAUDRATE = 115200


def resolve_esp32_port(port=SERIAL_PORT):
    if port and os.path.exists(port):
        return port
    excluded = {os.path.realpath(s) for s in ("/dev/rplidar", "/dev/imu", "/dev/ublox_gps", "/dev/gps") if os.path.exists(s)}
    candidates = [c for c in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")) if os.path.realpath(c) not in excluded]
    return candidates[0] if candidates else port

# 기어(속도) 설정
GEARS = {'1': 80, '2': 130, '3': 180, '4': 255}

# 조향 설정 (단위: 도, ESP32 MAX_STEER_ANGLE_DEG = 35.0과 일치)
MAX_STEER = 35.0
STEER_ACTIVE_RATE = 1.0   # A/D 누를 때 루프 1회당 조향 변화량 (20Hz 기준 20°/s, 짧게 한 번 누르면 약 13°)
STEER_RETURN_RATE = 3.0   # 손을 뗐을 때 원점 복귀 속도

# 가감속 (Ramp) 설정
PWM_ACCEL_STEP = 15.0
PWM_DECEL_STEP = 30.0

# 통신 타이머
LOOP_RATE_HZ = 20
STEER_TIMEOUT = 0.65  # 키보드 첫 반복 지연(0.5초)보다 길게 설정해 누르고 있을 때 끊김 방지


class TeleopManual:
    def __init__(self):
        self.ser = None
        self.running = True
        self.is_armed = False

        self.current_gear = '1'
        self.max_pwm = GEARS[self.current_gear]
        
        self.drive_mode = 'STOP'  
        self.target_pwm = 0.0
        self.current_pwm = 0.0
        
        self.target_steer = 0.0
        self.current_steer = 0.0

        self.esp32_pot = None
        self.esp32_steer_cur = None

        self.last_steer_input_time = 0.0
        self.last_steer_active_time = 0.0
        self.latest_key = ""
        
        self.old_settings = termios.tcgetattr(sys.stdin)
        self.connect_serial()

    def connect_serial(self):
        try:
            port = resolve_esp32_port(SERIAL_PORT)
            s = serial.Serial()
            s.port = port
            s.baudrate = BAUDRATE
            s.timeout = 0.02
            s.write_timeout = 0.05
            s.dtr = False
            s.rts = False
            s.open()
            self.ser = s
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"\r\n[SYSTEM] ✅ ESP32 시리얼 연결 성공: {port}\r\n")
            time.sleep(0.3)
        except Exception as e:
            print(f"\r\n[SYSTEM] ❌ ESP32 시리얼 연결 실패: {e}\r\n")
            time.sleep(1)
            self.ser = None

    def read_esp32_telemetry(self):
        if not self.ser or not self.ser.is_open:
            return
        try:
            for _ in range(10):
                if self.ser.in_waiting <= 0:
                    break
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('{') and line.endswith('}'):
                    obj = json.loads(line)
                    self.esp32_pot = obj.get("pot", self.esp32_pot)
                    self.esp32_steer_cur = obj.get("steer_cur", self.esp32_steer_cur)
        except Exception:
            pass

    def print_dashboard(self):
        bar_length = 25
        
        pwm_ratio = abs(self.current_pwm) / 255.0
        filled_pwm = int(bar_length * pwm_ratio)
        pwm_bar = "█" * filled_pwm + "-" * (bar_length - filled_pwm)
        
        steer_ratio = (self.current_steer + MAX_STEER) / (MAX_STEER * 2.0)
        steer_pos = int(bar_length * steer_ratio)
        steer_bar_list = ["-"] * bar_length
        if 0 <= steer_pos < bar_length:
            steer_bar_list[steer_pos] = "O"
        steer_bar = "".join(steer_bar_list)

        arm_status = "\033[92m[🟢 ENGINE ARMED]\033[0m" if self.is_armed else "\033[91m[🔴 DISARMED (E 키를 누르세요)]\033[0m"
        actual_steer_str = f"{self.esp32_steer_cur:5.1f}° (ADC: {self.esp32_pot})" if self.esp32_steer_cur is not None else "수신 대기 중..."

        dashboard = f"\033[H\033[J======================================================\r\n" \
                    f"🚀 ESP32 Teleop (Cruise Control Mode)\r\n" \
                    f"======================================================\r\n" \
                    f"[Controls]\r\n" \
                    f"  \033[93m[E] 시동 켜기\033[0m\r\n" \
                    f"  [W] / [S] 전진 / 후진 크루즈 주행 (한 번만 누르기)\r\n" \
                    f"  [SPACE]   전체 즉시 정지 (브레이크)\r\n" \
                    f"  [A] / [D] 누르는 동안 좌/우 회전 (떼면 원점 복귀)\r\n" \
                    f"  \033[96m[C]       핸들 즉시 중앙(0°) 정렬 (속도 유지)\033[0m\r\n" \
                    f"------------------------------------------------------\r\n" \
                    f"[Status] {arm_status}\r\n" \
                    f"  Latest Key  : [{self.latest_key.upper()}]\r\n" \
                    f"  Gear        : {self.current_gear} 단 (Target Max: {self.max_pwm})\r\n" \
                    f"  Drive Mode  : {self.drive_mode}\r\n" \
                    f"  \r\n" \
                    f"  Current PWM : {int(self.current_pwm):4d} [{pwm_bar}]\r\n" \
                    f"  Target Steer: {self.target_steer:5.1f}° (Ramp: {self.current_steer:5.1f}°) [{steer_bar}]\r\n" \
                    f"  Actual Steer: {actual_steer_str}\r\n" \
                    f"======================================================\r\n"
        
        sys.stdout.write(dashboard)
        sys.stdout.flush()

    def run(self):
        tty.setcbreak(sys.stdin.fileno())

        try:
            while self.running:
                now = time.time()
                self.read_esp32_telemetry()
                
                # 입력 버퍼 비우기 및 최신 키 획득
                self.latest_key = ""
                while sys.stdin in select.select([sys.stdin], [], [], 0.0)[0]:
                    k = sys.stdin.read(1).lower()
                    self.latest_key = k

                    if k == '\x1b':  # ESC
                        self.running = False
                        break
                    elif k == 'e':
                        self.is_armed = True
                    elif k == ' ':
                        # 스페이스바: 모터 및 조향 즉시 전체 정지
                        self.drive_mode = 'STOP'
                        self.target_pwm = 0.0
                        self.current_pwm = 0.0  # 램프 없이 즉시 정지
                        self.target_steer = 0.0
                        self.last_steer_input_time = 0.0
                        self.last_steer_active_time = now
                    elif k == 'c':
                        # C 키: 주행 상태는 유지하면서 핸들만 즉시 중앙으로
                        self.target_steer = 0.0
                        self.last_steer_input_time = 0.0
                        self.last_steer_active_time = now
                    elif k in ['1', '2', '3', '4']:
                        self.current_gear = k
                        self.max_pwm = GEARS[self.current_gear]
                        # 기어 변속 시 주행 상태면 타겟 PWM 즉시 반영
                        if self.drive_mode == 'FORWARD':
                            self.target_pwm = float(self.max_pwm)
                        elif self.drive_mode == 'REVERSE':
                            self.target_pwm = -float(self.max_pwm)

                    if not self.is_armed:
                        continue

                    # 크루즈 주행 제어
                    if k == 'w':
                        self.drive_mode = 'FORWARD'
                        self.target_pwm = float(self.max_pwm)
                    elif k == 's':
                        self.drive_mode = 'REVERSE'
                        self.target_pwm = -float(self.max_pwm)

                    # 조향 제어: 키 반복마다 계단식으로 끊기지 않도록 누르면 목표를 끝단으로 두고 STEER_ACTIVE_RATE로 부드럽게 이동
                    if k == 'a':
                        self.target_steer = -MAX_STEER
                        self.last_steer_input_time = now
                        self.last_steer_active_time = now
                    elif k == 'd':
                        self.target_steer = MAX_STEER
                        self.last_steer_input_time = now
                        self.last_steer_active_time = now

                if not self.running:
                    break

                if not self.is_armed:
                    self.print_dashboard()
                    time.sleep(1.0 / LOOP_RATE_HZ)
                    continue

                # A/D 키에서 손을 떼면 부드럽게 중앙(0.0°)으로 복귀
                if now - self.last_steer_input_time > STEER_TIMEOUT:
                    if self.target_steer != 0.0:
                        self.last_steer_active_time = now
                    self.target_steer = 0.0

                # 부드러운 가감속(Ramping) 및 조향 원점 복귀 연산
                if abs(self.current_pwm - self.target_pwm) < PWM_DECEL_STEP:
                    self.current_pwm = self.target_pwm
                elif self.target_pwm > self.current_pwm:
                    step = PWM_ACCEL_STEP if self.current_pwm >= 0 else PWM_DECEL_STEP
                    self.current_pwm += step
                elif self.target_pwm < self.current_pwm:
                    step = PWM_ACCEL_STEP if self.current_pwm <= 0 else PWM_DECEL_STEP
                    self.current_pwm -= step

                steer_rate = STEER_ACTIVE_RATE if self.target_steer != 0.0 else STEER_RETURN_RATE
                if abs(self.current_steer - self.target_steer) < steer_rate:
                    self.current_steer = self.target_steer
                elif self.current_steer < self.target_steer:
                    self.current_steer += steer_rate
                elif self.current_steer > self.target_steer:
                    self.current_steer -= steer_rate

                # W 누를 때 전진, S 누를 때 후진 정상 방향 적용 (- 부호)
                cmd_pwm = -int(round(self.current_pwm))
                cmd_steer = round(self.current_steer, 2)

                # 정지 상태(PWM=0)에서도 조향 테스트 및 원점 복귀가 가능하도록 hold 플래그 전달
                need_hold = (
                    abs(self.target_steer) > 0.1
                    or abs(self.current_steer) > 0.1
                    or (now - self.last_steer_active_time < 2.0)
                    or (self.esp32_steer_cur is not None and abs(cmd_steer - self.esp32_steer_cur) > 1.5 and (now - self.last_steer_active_time < 3.0))
                )
                hold_flag = 1 if need_hold else 0
                cmd_str = f"w,{cmd_pwm},{cmd_steer},{hold_flag}\n"

                if self.ser and self.ser.is_open:
                    try:
                        self.ser.write(cmd_str.encode('utf-8'))
                    except Exception:
                        pass 

                self.print_dashboard()
                time.sleep(1.0 / LOOP_RATE_HZ)

        except Exception as e:
            print(f"\r\n[SYSTEM] 오류 발생: {e}\r\n")
        finally:
            self.shutdown()

    def shutdown(self):
        print("\r\n[SYSTEM] 🛑 시스템 종료...\r\n")
        if self.ser and self.ser.is_open:
            try:
                for _ in range(3):
                    self.ser.write(b"w,0,0.0,0\n")
                    self.ser.flush()
                    time.sleep(0.05)
                self.ser.close()
            except Exception:
                pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


if __name__ == '__main__':
    teleop = TeleopManual()
    teleop.run()