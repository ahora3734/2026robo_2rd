#!/usr/bin/env python3
# esp32_steer_test.py
# ESP32 단독 조향 점검 (start.sh를 끈 상태에서 Jetson에서 실행, 반드시 바퀴를 들고 할 것)
#
#   가변저항 값 보기 (모터 OFF, 손으로 핸들 돌리며 중앙/양끝 ADC 확인):
#     python3 esp32_steer_test.py pot
#   조향 시험 (목표각으로 돌리고 도달 시간 측정, 구동 모터도 PWM만큼 돎):
#     python3 esp32_steer_test.py steer 20          # 오른쪽 20도
#     python3 esp32_steer_test.py steer -20         # 왼쪽 20도
#     python3 esp32_steer_test.py steer 20 --pwm 40 --sec 4
#   정지 조향 시험 (구동 모터 정지 상태에서 핸들만 꺾기, 회피 STEER_PREP과 동일):
#     python3 esp32_steer_test.py steer 35 --pwm 0 --hold
#   후진 방향 확인 (바퀴가 뒤로 도는지):
#     python3 esp32_steer_test.py steer 0 --pwm -30 --sec 2
import argparse
import glob
import json
import os
import sys
import time
import serial

def resolve_esp32_port(port):
    if port and os.path.exists(port):
        return port
    excluded = {os.path.realpath(s) for s in ("/dev/rplidar", "/dev/imu", "/dev/ublox_gps", "/dev/gps") if os.path.exists(s)}
    candidates = [c for c in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")) if os.path.realpath(c) not in excluded]
    return candidates[0] if candidates else port

ap = argparse.ArgumentParser()
ap.add_argument("mode", choices=["pot", "steer"])
ap.add_argument("angle", nargs="?", type=float, default=20.0)
ap.add_argument("--pwm", type=int, default=30, help="구동 PWM (0이면 펌웨어가 조향 모터를 끔)")
ap.add_argument("--sec", type=float, default=4.0, help="조향 시험 시간")
ap.add_argument("--hold", action="store_true", help="정지 조향 허용 (PWM 0에서도 조향 모터 구동)")
ap.add_argument("--port", default="/dev/esp32")
args = ap.parse_args()

target_port = resolve_esp32_port(args.port)
if target_port != args.port:
    print(f"🔍 {args.port} 미발견 -> 자동 탐색된 포트({target_port})로 연결합니다.")
ser = serial.Serial(target_port, 115200, timeout=0.02)
time.sleep(2.0)  # 포트를 열면 ESP32가 리셋되므로 부팅 대기
ser.reset_input_buffer()


def read_telemetry():
    """버퍼에 쌓인 줄을 모두 읽고 마지막 JSON만 반환"""
    last = None
    while ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line.startswith("{"):
            try:
                last = json.loads(line)
            except Exception:
                pass
    return last


def stop():
    for _ in range(5):
        ser.write(b"<0,0.00>\n")
        time.sleep(0.05)


try:
    if args.mode == "pot":
        import os
        # Windows ANSI 지원 활성화
        if os.name == "nt":
            os.system("")

        def check_enter_pressed():
            """크로스 플랫폼 Enter 키 입력 감지 (Non-blocking)"""
            if os.name == "nt":
                import msvcrt
                if msvcrt.kbhit():
                    ch = msvcrt.getch()
                    if ch in (b"\r", b"\n"):
                        return True
            else:
                import select
                rlist, _, _ = select.select([sys.stdin], [], [], 0)
                if rlist:
                    sys.stdin.readline()
                    return True
            return False

        def flush_input():
            if os.name == "nt":
                import msvcrt
                while msvcrt.kbhit():
                    msvcrt.getch()
            else:
                import select
                while select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()

        def run_step(step_num, title, instruction, target_var, line_num):
            print("\n" + "=" * 68)
            print(f" 🎯 [단계 {step_num}/3] {title}")
            print(f" 👉 행동: {instruction}")
            print(f" 📝 반영 위치: esp32_auto.ino 파일의 {line_num} '{target_var}'")
            print("=" * 68)
            print(" ⏳ 위치를 맞춘 뒤 키보드의 [Enter]를 누르면 해당 값이 저장됩니다.\n")

            flush_input()
            samples = []
            last_pot = 2048

            while True:
                t = read_telemetry()
                if t and "pot" in t:
                    last_pot = int(t["pot"])
                    samples.append(last_pot)
                    if len(samples) > 10:
                        samples.pop(0)
                    avg_pot = int(sum(samples) / len(samples))
                    print(
                        f"\r  ▶ 실시간 pot 값: \033[1;33m{last_pot:4d}\033[0m (평균 {avg_pot:4d}) | 위치 맞추고 [Enter] 누르기...   ",
                        end="",
                        flush=True,
                    )

                if check_enter_pressed():
                    break
                time.sleep(0.04)

            captured_val = int(sum(samples) / len(samples)) if samples else last_pot
            print(f"\n\n  ✅ [{title} 완료] 측정된 ADC 값: {captured_val}")
            print(f"     👉 esp32_auto.ino {line_num}에 다음과 같이 입력하세요:")
            print(f"        {target_var} = {captured_val};")
            time.sleep(0.8)
            return captured_val

        print("\n====================================================================")
        print(" 🚗 [ESP32 조향 가변저항 캘리브레이션 위저드]")
        print("  단계별 안내에 따라 핸들을 맞춘 후 [Enter]를 눌러주세요.")
        print("  (모든 단계를 마치면 esp32_auto.ino에 바로 붙여넣을 코드가 출력됩니다)")
        print("====================================================================")

        # 1단계: 중앙
        center_pot = run_step(
            1,
            "핸들 및 앞바퀴 중앙(직진) 정렬",
            "손으로 핸들을 돌려 앞바퀴를 완전한 11자(직진) 중앙에 놓으세요.",
            "int POT_CENTER",
            "26번 줄",
        )

        # 2단계: 좌측 끝
        left_pot = run_step(
            2,
            "핸들 최대 좌회전 (왼쪽 끝단)",
            "손으로 핸들을 왼쪽 끝까지 천천히 최대한 돌리세요.",
            "int POT_MIN_LIMIT 또는 MAX_LIMIT",
            "29~30번 줄",
        )

        # 3단계: 우측 끝
        right_pot = run_step(
            3,
            "핸들 최대 우회전 (오른쪽 끝단)",
            "손으로 핸들을 오른쪽 끝까지 천천히 최대한 돌리세요.",
            "int POT_MAX_LIMIT 또는 MIN_LIMIT",
            "29~30번 줄",
        )

        pot_min = min(left_pot, right_pot)
        pot_max = max(left_pot, right_pot)
        is_reversed = right_pot < left_pot  # 우회전 시 ADC가 감소하면 센서 방향 반전

        print("\n\n" + "=" * 68)
        print(" 🎉 [축하합니다! 캘리브레이션 측정이 모두 완료되었습니다]")
        print("====================================================================")
        print(" 📄 수정 대상 파일: esp32_auto/esp32_auto.ino (26번 ~ 49번 줄)")
        print(" 아래 코드 블록을 그대로 복사하여 esp32_auto.ino의 해당 변수에 붙여넣으세요:\n")
        print("--------------------------------------------------------------------")
        print(f"// --- [26~30번 줄 수정] ---")
        print(f"int POT_CENTER = {center_pot};      // 핸들 직진 중앙 ADC (측정값: {center_pot})")
        print(f"int POT_MIN_LIMIT = {pot_min};    // 최소 끝단 ADC (측정값: {pot_min})")
        print(f"int POT_MAX_LIMIT = {pot_max};   // 최대 끝단 ADC (측정값: {pot_max})")
        print()
        print(f"// --- [49번 줄 수정] ---")
        print(f"#define REVERSE_POT_SENSOR   {'true' if is_reversed else 'false'}  // 우회전 시 ADC {'감소' if is_reversed else '증가'} 감지")
        print("--------------------------------------------------------------------")
        print("\n 💡 안내:")
        print(f"  - 중앙 직진값 : {center_pot} (POT_CENTER에 입력)")
        print(f"  - 왼쪽 끝단   : {left_pot}")
        print(f"  - 오른쪽 끝단 : {right_pot}")
        print(f"  - 안전 마진(POT_LIMIT_MARGIN) 100이 기본 적용되어 끝단 파손을 방지합니다.")
        print("====================================================================\n")

        print("실시간 모니터링을 계속 진행합니다. (종료하려면 Ctrl+C를 누르세요)")
        lo, hi = pot_min, pot_max
        while True:
            t = read_telemetry()
            if t:
                pot = int(t["pot"])
                lo, hi = min(lo, pot), max(hi, pot)
                print(f"\r  현재 pot={pot:4d}  조향각={t['steer_cur']:6.1f}°   (최소 {lo}, 최대 {hi})   ", end="", flush=True)
            time.sleep(0.05)

    else:
        target = args.angle
        print(f"목표 {target:+.1f}° ({'오른쪽' if target > 0 else '왼쪽'}), 구동 PWM {args.pwm}, {args.sec}초")
        if args.pwm == 0 and not args.hold:
            print("  주의: PWM 0이면 펌웨어가 조향 모터를 끕니다 (--hold 를 붙이거나 --pwm 을 1 이상으로)")
        hold_field = ",1" if args.hold else ""
        cmd = f"<{args.pwm},{target:.2f}{hold_field}>\n".encode()
        t0 = time.monotonic()
        start_pot = None
        reached = None
        while time.monotonic() - t0 < args.sec:
            ser.write(cmd)  # 워치독(0.5초) 때문에 계속 반복 전송
            t = read_telemetry()
            if t:
                el = time.monotonic() - t0
                if start_pot is None:
                    start_pot = int(t["pot"])
                if reached is None and abs(t["steer_cur"] - target) <= 1.0:
                    reached = el
                fault = t.get("fault", "")
                print(f"  {el:4.2f}s  pot={int(t['pot']):4d}  현재={t['steer_cur']:6.1f}°  목표={t['steer_target']:6.1f}°  armed={t['armed']} {fault}")
            time.sleep(0.05)
        stop()
        print()
        end = read_telemetry()
        if start_pot is not None and end:
            d = int(end["pot"]) - start_pot
            print(f"pot 변화: {start_pot} -> {int(end['pot'])} ({'+' if d >= 0 else ''}{d})")
        if reached is not None:
            print(f"✅ 목표 도달 시간: {reached:.2f}초 (도중에 0.3초 이상 멈추면 펌웨어 STALL 판정에 걸림)")
        else:
            print("❌ 목표에 도달하지 못함 -> 방향이 반대거나(REVERSE_*), 모터 힘/배선 문제")
        print("확인: 핸들이 실제로 오른쪽(+) / 왼쪽(-)으로 돌았나요?")
except KeyboardInterrupt:
    pass
finally:
    stop()
    ser.close()
    print("\n정지 명령 전송 후 종료")
