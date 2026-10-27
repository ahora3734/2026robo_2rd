// esp32_control.ino
// ==========================================
// ROS 2 - ESP32 Go-Kart Firmware (가변저항 미연결 직접 제어 모드)
// A 누르면 좌회전, D 누르면 우회전, 손 떼면 즉시 정지
// ==========================================

#include <Arduino.h>

// --- [1. 하드웨어 핀 맵] ---
#define LEFT_PWM_PIN  18
#define LEFT_DIR_PIN  19
#define RIGHT_PWM_PIN 21
#define RIGHT_DIR_PIN 22

#define STEER_PWM_PIN 25
#define STEER_DIR_PIN 26

// [방향 반전] A를 눌렀는데 우측으로 돌면 이 값을 true <-> false 로 변경
#define REVERSE_STEER_MOTOR false 

// 조향 모터 속도 설정 (0 ~ 255 중 적당한 속도)
#define MANUAL_STEER_PWM  70

// --- [2. 제어 변수 및 상태] ---
bool is_armed = false;          
int target_drive_pwm = 0;       
float target_steer_angle = 0.0; 

unsigned long last_comm_time = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 500; 

void parseSerialDataSafe();
void controlDriveMotors();
void controlSteerMotor();
void stopAllMotors();

void setup() {
  Serial.begin(115200);
  
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(STEER_PWM_PIN, OUTPUT);
  pinMode(STEER_DIR_PIN, OUTPUT);

  stopAllMotors();
  is_armed = false;

  delay(500); 
}

void loop() {
  parseSerialDataSafe();

  // 0.5초간 통신 없으면 안전 정지
  if (millis() - last_comm_time > WATCHDOG_TIMEOUT_MS) {
    is_armed = false;
    stopAllMotors();
  } else {
    if (is_armed) {
      controlDriveMotors();
      controlSteerMotor();
    } else {
      stopAllMotors();
    }
  }

  delay(10); 
}

// ==========================================
// 1. 시리얼 통신 안전 파싱 (C-스타일 고정 버퍼로 힙 단편화 방지)
// ==========================================
static char rx_buf[64];
static uint8_t rx_idx = 0;

void parseSerialDataSafe() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx_idx == 0) continue;
      rx_buf[rx_idx] = '\0';

      int pwm_in = 0;
      float steer_in = 0.0f;
      bool valid = false;

      // 포맷 A: esp32_bridge.py 표준 규격 <PWM,STEER> (예: <120,-15.20>)
      if (rx_buf[0] == '<') {
        if (sscanf(rx_buf, "<%d,%f>", &pwm_in, &steer_in) == 2) {
          valid = true;
        }
      }
      // 포맷 B: 수동/키보드 명령 호환 w,PWM,STEER / s,PWM,STEER
      else if (rx_buf[0] == 'w' || rx_buf[0] == 's') {
        if (sscanf(rx_buf + 2, "%d,%f", &pwm_in, &steer_in) == 2) {
          valid = true;
        }
      }

      if (valid) {
        target_drive_pwm = pwm_in;
        target_steer_angle = steer_in;
        last_comm_time = millis();
        is_armed = true;
      }

      rx_idx = 0;
    } else {
      if (rx_idx < sizeof(rx_buf) - 1) {
        rx_buf[rx_idx++] = c;
      } else {
        rx_idx = 0;
      }
    }
  }
}

// ==========================================
// 2. 주행 모터 제어
// ==========================================
void controlDriveMotors() {
  int pwm_out = abs(target_drive_pwm);
  if (pwm_out > 255) pwm_out = 255;

  if (target_drive_pwm > 0) { 
    digitalWrite(LEFT_DIR_PIN, HIGH);
    digitalWrite(RIGHT_DIR_PIN, HIGH);
    analogWrite(LEFT_PWM_PIN, pwm_out);
    analogWrite(RIGHT_PWM_PIN, pwm_out);
  } 
  else if (target_drive_pwm < 0) { 
    digitalWrite(LEFT_DIR_PIN, LOW);
    digitalWrite(RIGHT_DIR_PIN, LOW);
    analogWrite(LEFT_PWM_PIN, pwm_out);
    analogWrite(RIGHT_PWM_PIN, pwm_out);
  } 
  else {
    analogWrite(LEFT_PWM_PIN, 0);
    analogWrite(RIGHT_PWM_PIN, 0);
  }
}

// ==========================================
// 3. 조향 모터 직접 제어 (가변저항 없이 동작)
// ==========================================
void controlSteerMotor() {
  // A키를 누르면 음수(-), D키를 누르면 양수(+), 손 떼면 0.0
  if (target_steer_angle > 10.0) {
    // [우회전]
    bool dir = true;
    if (REVERSE_STEER_MOTOR) dir = !dir;
    digitalWrite(STEER_DIR_PIN, dir ? HIGH : LOW);
    analogWrite(STEER_PWM_PIN, MANUAL_STEER_PWM);
  } 
  else if (target_steer_angle < -10.0) {
    // [좌회전]
    bool dir = false;
    if (REVERSE_STEER_MOTOR) dir = !dir;
    digitalWrite(STEER_DIR_PIN, dir ? HIGH : LOW);
    analogWrite(STEER_PWM_PIN, MANUAL_STEER_PWM);
  } 
  else {
    // [손 뗌 -> 즉시 정지]
    analogWrite(STEER_PWM_PIN, 0);
  }
}

// ==========================================
// 4. 모터 전원 완전 차단
// ==========================================
void stopAllMotors() {
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
  analogWrite(STEER_PWM_PIN, 0); 
}