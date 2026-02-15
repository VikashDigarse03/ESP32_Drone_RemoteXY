/*
DRONE CONTROLLER - ESP32 (High Performance V3.0)
- Resolution: 11-Bit (0-2047)
- Control: Angle + Rate (PD)
- Safety: Angle cutoff & connection failsafe
- Loop: 250Hz

- need to implement Kalman filter for MPU6050
-Tune PID for the drone
*/

#define REMOTEXY_MODE__WIFI_POINT

#include <WiFi.h>            // REQUIRED for RemoteXY WiFi
#include <RemoteXY.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

// --- WiFi Configuration ---
#define REMOTEXY_WIFI_SSID "Thunderbolts"
#define REMOTEXY_WIFI_PASSWORD "DSL_Thunderbolts"
#define REMOTEXY_SERVER_PORT 6377

// --- RemoteXY UI Configuration ---
#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] = {
  255,5,0,0,0,39,0,19,0,0,0,0,31,1,200,84,1,1,3,0,
  5,16,33,40,40,0,2,26,31,5,141,31,42,42,32,2,26,31,
  4,68,10,56,8,128,2,26
};
struct {
  int8_t joystick_01_x; // Yaw
  int8_t joystick_01_y; // Throttle
  int8_t joystick_02_x; // Roll
  int8_t joystick_02_y; // Pitch
  int8_t slider_01;     // PID gain
  uint8_t connect_flag;
} RemoteXY;
#pragma pack(pop)

// --- Motor Pins ---
const int motorFL = 25;
const int motorFR = 33;
const int motorRL = 26;
const int motorRR = 27;

// --- PWM Settings ---
// 🔹 BRUSHED MOTORS
#define PWM_FREQ 10000

// 🔹 IF USING BRUSHLESS ESCs, USE THIS INSTEAD:
// #define PWM_FREQ 50

#define PWM_RES 11
#define MAX_PWM 2047
#define MIN_START_PWM 150
#define MAX_TILT 45.0

MPU6050 mpu(Wire);
unsigned long last_loop_time = 0;
bool isArmed = false;

// PID values
float pid_p_gain = 0;
float pid_roll_kp = 2.5, pid_roll_kd = 0.8;
float pid_pitch_kp = 2.5, pid_pitch_kd = 0.8;
float pid_yaw_kp = 3.0;

void setup() {
  Serial.begin(115200);

  // PWM attach (ESP32 core v3.x)
  ledcAttach(motorFL, PWM_FREQ, PWM_RES);
  ledcAttach(motorFR, PWM_FREQ, PWM_RES);
  ledcAttach(motorRL, PWM_FREQ, PWM_RES);
  ledcAttach(motorRR, PWM_FREQ, PWM_RES);

  stopMotors();

  // I2C
  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(500);

  // MPU6050
  Serial.println("Initializing MPU...");
  mpu.begin();

  Serial.println("Calibrating gyro...");
  delay(1000);
  mpu.calcGyroOffsets(true);

  Serial.println("Ready.");
  RemoteXY_Init();
}

void loop() {
  RemoteXYEngine.handler();

  // Failsafe
  if (RemoteXY.connect_flag == 0) {
    stopMotors();
    isArmed = false;
    return;
  }

  if (millis() - last_loop_time >= 4) {
    last_loop_time = millis();
    mpu.update();

    // --- Axis remap (90° rotated mount) ---
    float actualRoll  = mpu.getAngleY();
    float actualPitch = -mpu.getAngleX();
    float gyroRoll    = mpu.getGyroY();
    float gyroPitch   = -mpu.getGyroX();

    // Throttle
    int throttle = map(RemoteXY.joystick_01_y, -100, 100, 0, MAX_PWM);
    throttle = constrain(throttle, 0, MAX_PWM);

    if (throttle < 50) isArmed = true;
    if (!isArmed) throttle = 0;

    // Commands
    int cmd_roll  = abs(RemoteXY.joystick_02_x) > 5 ? RemoteXY.joystick_02_x : 0;
    int cmd_pitch = abs(RemoteXY.joystick_02_y) > 5 ? RemoteXY.joystick_02_y : 0;
    int cmd_yaw   = abs(RemoteXY.joystick_01_x) > 5 ? RemoteXY.joystick_01_x : 0;

    pid_p_gain = RemoteXY.slider_01 * 0.1;

    // Safety cutoff
    if (abs(actualRoll) > MAX_TILT || abs(actualPitch) > MAX_TILT) {
      stopMotors();
      return;
    }

    float roll_target  = cmd_roll * 0.2;
    float pitch_target = cmd_pitch * 0.2;
    float roll_trim = -2.5; 

    float roll_error = (roll_target + roll_trim) - actualRoll;
    float pitch_error = pitch_target - actualPitch;

    float roll_pid  = (roll_error * pid_roll_kp * pid_p_gain) - (gyroRoll * pid_roll_kd);
    float pitch_pid = (pitch_error * pid_pitch_kp * pid_p_gain) - (gyroPitch * pid_pitch_kd);
    float yaw_pid   = cmd_yaw * pid_yaw_kp;

    // Motor mix (X quad)
    float pitch_trim = -20;  // negative = reduce rear thrust
 
// + value  → drone rolls RIGHT
// - value  → drone rolls LEFT

int fl = throttle + pitch_pid + roll_pid + yaw_pid -40;
int fr = throttle + pitch_pid - roll_pid - yaw_pid +20;
int rl = throttle - pitch_pid + roll_pid - yaw_pid + pitch_trim -40;
int rr = throttle - pitch_pid - roll_pid + yaw_pid + pitch_trim +20;


    if (throttle < MIN_START_PWM) {
      stopMotors();
    } else {
      ledcWrite(motorFL, constrain(fl, 0, MAX_PWM));
      ledcWrite(motorFR, constrain(fr, 0, MAX_PWM));
      ledcWrite(motorRL, constrain(rl, 0, MAX_PWM));
      ledcWrite(motorRR, constrain(rr, 0, MAX_PWM));
    }
  }
}

void stopMotors() {
  ledcWrite(motorFL, 0);
  ledcWrite(motorFR, 0);
  ledcWrite(motorRL, 0);
  ledcWrite(motorRR, 0);
}