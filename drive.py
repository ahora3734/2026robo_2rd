#!/usr/bin/env python3
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

# 기어(속도) 설정
GEARS = {'1': 80, '2': 130, '3': 180, '4': 255}

# 조향 설정 (단위: 도, ESP32 MAX_STEER_ANGLE_DEG = 35.0과 일치)
MAX_STEER = 35.0
STEER_STEP = 7.0          # A/D 한 번 누를 때 7°
STEER_RETURN_RATE = 5.0   # 루프 1회당 조향 변화량 (20Hz 기준 100°/s)

# 가감속 (Ramp) 설정
PWM_ACCEL_STEP = 15.0
PWM_DECEL_STEP = 30.0

# 통신 타이머
LOOP_RATE_HZ = 20
STEER_TIMEOUT = 0.6  # A/D 키에서 손 떼고 중앙 복귀까지 대기하는 시간 (터미널 키 반복 지연보다 길게)


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

        self.last_steer_input_time = 0.0
        self.latest_key = ""
        
        self.old_settings = termios.tcgetattr(sys.stdin)
        self.connect_serial()

    def connect_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.05, write_timeout=0.05)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"\r\n[SYSTEM] ✅ ESP32 시리얼 연결 성공: {SERIAL_PORT}\r\n")
            time.sleep(0.5)
        except Exception as e:
            print(f"\r\n[SYSTEM] ❌ ESP32 시리얼 연결 실패: {e}\r\n")
            time.sleep(1)
            self.ser = None

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

        dashboard = f"\033[H\033[J======================================================\r\n" \
                    f"🚀 ESP32 Teleop (Cruise Control Mode)\r\n" \
                    f"======================================================\r\n" \
                    f"[Controls]\r\n" \
                    f"  \033[93m[E] 시동 켜기\033[0m\r\n" \
                    f"  [W] / [S] 전진 / 후진 크루즈 주행 (한 번만 누르기)\r\n" \
                    f"  [SPACE]   전체 즉시 정지 (브레이크)\r\n" \
                    f"  [A] / [D] 누르는 동안 좌/우 회전 (떼면 원점 복귀)\r\n" \
                    f"  \033[96m[C]       핸들 즉시 중앙 정렬 (속도 유지)\033[0m\r\n" \
                    f"------------------------------------------------------\r\n" \
                    f"[Status] {arm_status}\r\n" \
                    f"  Latest Key  : [{self.latest_key.upper()}]\r\n" \
                    f"  Gear        : {self.current_gear} 단 (Target Max: {self.max_pwm})\r\n" \
                    f"  Drive Mode  : {self.drive_mode}\r\n" \
                    f"  \r\n" \
                    f"  Current PWM : {int(self.current_pwm):4d} [{pwm_bar}]\r\n" \
                    f"  Steer Angle : {self.current_steer:5.1f}  [{steer_bar}]\r\n" \
                    f"======================================================\r\n"
        
        sys.stdout.write(dashboard)
        sys.stdout.flush()

    def run(self):
        tty.setcbreak(sys.stdin.fileno())

        try:
            while self.running:
                now = time.time()
                
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
                    elif k == 'c':
                        # C 키: 주행 상태는 유지하면서 핸들만 즉시 중앙으로
                        self.target_steer = 0.0
                        self.last_steer_input_time = 0.0
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

                    # 조향 제어
                    if k == 'a':
                        self.target_steer = max(-MAX_STEER, self.target_steer - STEER_STEP)
                        self.last_steer_input_time = now
                    elif k == 'd':
                        self.target_steer = min(MAX_STEER, self.target_steer + STEER_STEP)
                        self.last_steer_input_time = now

                if not self.running:
                    break

                if not self.is_armed:
                    self.print_dashboard()
                    time.sleep(1.0 / LOOP_RATE_HZ)
                    continue

                # 조향: A/D 키를 0.25초 이상 누르지 않으면 자동으로 중앙(0.0) 복귀
                if now - self.last_steer_input_time > STEER_TIMEOUT:
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

                if abs(self.current_steer - self.target_steer) < STEER_RETURN_RATE:
                    self.current_steer = self.target_steer
                elif self.current_steer < self.target_steer:
                    self.current_steer += STEER_RETURN_RATE
                elif self.current_steer > self.target_steer:
                    self.current_steer -= STEER_RETURN_RATE

                # W 누를 때 전진, S 누를 때 후진 정상 방향 적용 (- 부호)
                cmd_pwm = -int(round(self.current_pwm))
                cmd_steer = round(self.current_steer, 2)
                cmd_str = f"w,{cmd_pwm},{cmd_steer}\n"

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
                    self.ser.write(b"w,0,0.0\n")
                    self.ser.flush()
                    time.sleep(0.05)
                self.ser.close()
            except Exception:
                pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


if __name__ == '__main__':
    teleop = TeleopManual()
    teleop.run()