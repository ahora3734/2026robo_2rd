// esp32_auto.ino
// ==============================================================================
// ROS 2 - ESP32 Go-Kart Firmware (32번 핀 가변저항 피드백 폐루프 조향 제어)
// ==============================================================================
// - 32번 핀(ADC1)으로 핸들 가변저항 값을 읽어 현재 조향 각도를 산출
// - 젯슨(Jetson)으로부터 수신한 목표 조향각과 비교하여 실시간 P/PID 조향 모터 제어
// - 젯슨으로 현재 가변저항 ADC 및 각도 텔레메트리를 시리얼 JSON으로 실시간 피드백 전송
// - 통신 두절(0.5초 초과) 시 안전 워치독 작동으로 전 모터 즉각 정지
// ==============================================================================

#include <Arduino.h>

// --- [1. 하드웨어 핀 맵] ---
#define LEFT_PWM_PIN   18
#define LEFT_DIR_PIN   19
#define RIGHT_PWM_PIN  21
#define RIGHT_DIR_PIN  22

#define STEER_PWM_PIN  25
#define STEER_DIR_PIN  26

#define POT_PIN        32  // 핸들 가변저항 아날로그 핀 (ESP32 ADC1_CH4)

// --- [2. 캘리브레이션 및 제어 파라미터 (차량 환경에 맞춰 조정)] ---
// 핸들이 정중앙일 때의 가변저항 ADC 값 (0 ~ 4095 중 대략 중앙 2048 전후)
int POT_CENTER = 2048;

// 핸들 물리적 한계점 (과회전으로 인한 기구물 파손 방지)
int POT_MIN_LIMIT = 600;   // 최대 좌회전 또는 우회전 끝단 ADC
int POT_MAX_LIMIT = 3500;  // 최대 우회전 또는 좌회전 끝단 ADC

// 조향 불감대 (Deadband): 목표 각도와 오차가 이 범위 이내면 모터 정지 (모터 발열/떨림 방지)
const float STEER_DEADBAND_DEG = 1.0; 

// 최대 허용 조향각 (도, deg)
const float MAX_STEER_ANGLE_DEG = 35.0;

// 조향 모터 P 게인 (오차에 따른 모터 속도 비례 상수)
const float STEER_KP = 5.0; 
const int STEER_MIN_PWM = 65;   // 조향 모터가 기어를 돌리기 시작하는 최소 PWM
const int STEER_MAX_PWM = 180;  // 조향 모터 안전 최대 PWM

// 방향 반전 옵션 (테스트 후 핸들이 반대로 돌면 true <-> false 전환)
#define REVERSE_STEER_MOTOR  false
#define REVERSE_POT_SENSOR   false

// --- [3. 통신 및 상태 변수] ---
bool is_armed = false;
int target_drive_pwm = 0;
float target_steer_angle = 0.0;

float current_steer_angle = 0.0;
float filtered_pot_adc = 2048.0;

unsigned long last_comm_time = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 500;

unsigned long last_telemetry_time = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 50; // 20Hz 주기로 젯슨에 상태 전송

// 기어 파손 방지: 조향 모터 연속 구동 한계 시간 (스톨 차단)
const unsigned long STEER_STALL_TIMEOUT_MS = 1500;
unsigned long steer_motor_start_time = 0;

// --- [함수 프로토타입] ---
void parseSerialDataSafe();
void updatePotentiometer();
void controlDriveMotors();
void controlSteerClosedLoop();
void stopAllMotors();
void sendTelemetryToJetson();

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(STEER_PWM_PIN, OUTPUT);
  pinMode(STEER_DIR_PIN, OUTPUT);

  pinMode(POT_PIN, INPUT);

  stopAllMotors();
  is_armed = false;

  // 부팅 시 초기 가변저항 값 안정화
  delay(100);
  filtered_pot_adc = analogRead(POT_PIN);
  updatePotentiometer();

  Serial.println(F("[ESP32] Ready - Closed Loop Steering with Potentiometer (Pin 32)"));
}

void loop() {
  // 1. 젯슨 명령 수신 및 파싱
  parseSerialDataSafe();

  // 2. 가변저항 ADC 필터링 및 현재 각도 계산
  updatePotentiometer();

  // 3. 안전 워치독 검사 (0.5초 통신 단절 시 정지)
  if (millis() - last_comm_time > WATCHDOG_TIMEOUT_MS) {
    is_armed = false;
    stopAllMotors();
  } else {
    if (is_armed) {
      controlDriveMotors();
      controlSteerClosedLoop();
    } else {
      stopAllMotors();
    }
  }

  // 4. 젯슨으로 현재 가변저항 및 조향각 피드백 텔레메트리 전송 (20Hz)
  if (millis() - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_time = millis();
    sendTelemetryToJetson();
  }

  delay(5);
}

// ==============================================================================
// 1. 시리얼 수신 파서: <PWM,STEER> 및 w,PWM,STEER 모두 지원
// ==============================================================================
void parseSerialDataSafe() {
  while (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    if (data.length() == 0) continue;

    // 포맷 A: esp32_bridge.py 표준 규격 <PWM,STEER> (예: <120,-15.20>)
    if (data.startsWith("<") && data.endsWith(">")) {
      String payload = data.substring(1, data.length() - 1);
      int comma = payload.indexOf(',');
      if (comma > 0) {
        String pwm_str = payload.substring(0, comma);
        String steer_str = payload.substring(comma + 1);

        target_drive_pwm = pwm_str.toInt();
        target_steer_angle = steer_str.toFloat();

        // 조향 목표각 안전 클램핑
        target_steer_angle = constrain(target_steer_angle, -MAX_STEER_ANGLE_DEG, MAX_STEER_ANGLE_DEG);

        last_comm_time = millis();
        is_armed = true;
      }
    }
    // 포맷 B: 수동/키보드 명령 호환 w,PWM,STEER / s,PWM,STEER
    else if (data.startsWith("w,") || data.startsWith("s,")) {
      int firstComma = data.indexOf(',');
      int secondComma = data.indexOf(',', firstComma + 1);
      if (firstComma > 0 && secondComma > firstComma) {
        String pwm_str = data.substring(firstComma + 1, secondComma);
        String steer_str = data.substring(secondComma + 1);

        target_drive_pwm = pwm_str.toInt();
        target_steer_angle = steer_str.toFloat();
        target_steer_angle = constrain(target_steer_angle, -MAX_STEER_ANGLE_DEG, MAX_STEER_ANGLE_DEG);

        last_comm_time = millis();
        is_armed = true;
      }
    }
  }
}

// ==============================================================================
// 2. 가변저항(32번 핀) 필터링 및 현재 조향각(deg) 변환
// ==============================================================================
void updatePotentiometer() {
  int raw_adc = analogRead(POT_PIN);

  // EMA 저주파 통과 필터 (노이즈 및 채터링 제거)
  filtered_pot_adc = (filtered_pot_adc * 0.8f) + (raw_adc * 0.2f);

  // ADC 값 -> 조향 각도(-MAX_STEER ~ +MAX_STEER) 매핑
  float angle = 0.0f;
  if (filtered_pot_adc >= POT_CENTER) {
    angle = (float)(filtered_pot_adc - POT_CENTER) / (float)(POT_MAX_LIMIT - POT_CENTER) * MAX_STEER_ANGLE_DEG;
  } else {
    angle = -(float)(POT_CENTER - filtered_pot_adc) / (float)(POT_CENTER - POT_MIN_LIMIT) * MAX_STEER_ANGLE_DEG;
  }

  if (REVERSE_POT_SENSOR) {
    angle = -angle;
  }

  current_steer_angle = constrain(angle, -MAX_STEER_ANGLE_DEG, MAX_STEER_ANGLE_DEG);
}

// ==============================================================================
// 3. 조향 모터 폐루프(Closed-Loop) 비례 제어 (가변저항 피드백)
// ==============================================================================
void controlSteerClosedLoop() {
  // [기어 보호 1] 차량 정지 시(PWM == 0) 조향 모터 완전 OFF
  // - 정지 마찰력으로 인한 기어 파손/발열 방지
  // - 사람이 손으로 핸들을 자유롭게 돌려 대시보드 화면을 보며 영점(Center) 튜닝 가능!
  if (abs(target_drive_pwm) == 0) {
    analogWrite(STEER_PWM_PIN, 0);
    steer_motor_start_time = 0;
    return;
  }

  // [기어 보호 2] 기구적 하드웨어 한계 초과 시 즉시 보호 차단
  if (filtered_pot_adc <= POT_MIN_LIMIT || filtered_pot_adc >= POT_MAX_LIMIT) {
    analogWrite(STEER_PWM_PIN, 0);
    steer_motor_start_time = 0;
    return;
  }

  float error = target_steer_angle - current_steer_angle;

  // [기어 보호 3] 불감대(Deadband) 내에 도달하면 조향 모터 정지
  if (abs(error) <= STEER_DEADBAND_DEG) {
    analogWrite(STEER_PWM_PIN, 0);
    steer_motor_start_time = 0;
    return;
  }

  // [기어 보호 4] 스톨(Stall) / 기어 걸림 방지 안전 타이머
  if (steer_motor_start_time == 0) {
    steer_motor_start_time = millis();
  } else if (millis() - steer_motor_start_time > STEER_STALL_TIMEOUT_MS) {
    // 1.5초 이상 목표 도달 실패 시 기어 파손 방지 차단
    analogWrite(STEER_PWM_PIN, 0);
    return;
  }

  // P 비례 제어로 필요 모터 속도 산출
  int steer_pwm = (int)(abs(error) * STEER_KP * 10.0f);
  steer_pwm = constrain(steer_pwm, STEER_MIN_PWM, STEER_MAX_PWM);

  // 회전 방향 결정
  bool turn_right = (error > 0);
  if (REVERSE_STEER_MOTOR) {
    turn_right = !turn_right;
  }

  digitalWrite(STEER_DIR_PIN, turn_right ? HIGH : LOW);
  analogWrite(STEER_PWM_PIN, steer_pwm);
}

// ==============================================================================
// 4. 좌/우 주행 모터 구동
// ==============================================================================
void controlDriveMotors() {
  int pwm_out = abs(target_drive_pwm);
  pwm_out = constrain(pwm_out, 0, 255);

  if (target_drive_pwm > 0) {
    digitalWrite(LEFT_DIR_PIN, HIGH);
    digitalWrite(RIGHT_DIR_PIN, HIGH);
    analogWrite(LEFT_PWM_PIN, pwm_out);
    analogWrite(RIGHT_PWM_PIN, pwm_out);
  } else if (target_drive_pwm < 0) {
    digitalWrite(LEFT_DIR_PIN, LOW);
    digitalWrite(RIGHT_DIR_PIN, LOW);
    analogWrite(LEFT_PWM_PIN, pwm_out);
    analogWrite(RIGHT_PWM_PIN, pwm_out);
  } else {
    analogWrite(LEFT_PWM_PIN, 0);
    analogWrite(RIGHT_PWM_PIN, 0);
  }
}

// ==============================================================================
// 5. 모터 전원 완전 차단
// ==============================================================================
void stopAllMotors() {
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
  analogWrite(STEER_PWM_PIN, 0);
  steer_motor_start_time = 0;
}

// ==============================================================================
// 6. 젯슨으로 텔레메트리 회신 (JSON 문자열 -> /esp32_raw_status)
// ==============================================================================
void sendTelemetryToJetson() {
  // 젯슨의 esp32_bridge.py 및 관제 화면에서 파싱하기 쉽도록 JSON 한 줄로 전송
  Serial.print(F("{\"pot\":"));
  Serial.print((int)filtered_pot_adc);
  Serial.print(F(",\"steer_cur\":"));
  Serial.print(current_steer_angle, 1);
  Serial.print(F(",\"steer_target\":"));
  Serial.print(target_steer_angle, 1);
  Serial.print(F(",\"pwm\":"));
  Serial.print(target_drive_pwm);
  Serial.print(F(",\"armed\":"));
  Serial.print(is_armed ? 1 : 0);
  Serial.println(F("}"));
}
