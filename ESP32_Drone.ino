/*
DRONE CONTROLLER - ESP32 (High Performance V3.0)
- Resolution: 11-Bit (0-2047)
- Control: Angle + Rate (PD)
- Safety: Angle cutoff & connection failsafe
- Loop: 250Hz
*/

#define REMOTEXY_MODE__WIFI_POINT

#include <WiFi.h>
#include <RemoteXY.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

// ================= WIFI =================
#define REMOTEXY_WIFI_SSID "Drone_ESP32_Pro"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

// ================= REMOTEXY =================
#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] = {
  255,5,0,0,0,39,0,19,0,0,0,0,31,1,200,84,1,1,3,0,
  5,16,33,40,40,0,2,26,31,5,141,31,42,42,32,2,26,31,
  4,68,10,56,8,128,2,26
};
struct {
  int8_t joystick_01_x;
  int8_t joystick_01_y;
  int8_t joystick_02_x;
  int8_t joystick_02_y;
  int8_t slider_01;
  uint8_t connect_flag;
} RemoteXY;
#pragma pack(pop)

// ================= MOTORS =================
const int motorFL = 26;
const int motorFR = 27;
const int motorRL = 25;
const int motorRR = 33;

// ================= PWM =================
#define PWM_FREQ 10000
#define PWM_RES 11
#define MAX_PWM 2047
#define MIN_START_PWM 150
#define MAX_TILT 45.0

MPU6050 mpu(Wire);
unsigned long last_loop_time = 0;
bool isArmed = false;

// ================= PID =================
float pid_p_gain = 0;
float pid_roll_kp = 2.5, pid_roll_kd = 0.8;
float pid_pitch_kp = 2.5, pid_pitch_kd = 0.8;
float pid_yaw_kp = 3.0;

// ================= KALMAN =================
typedef struct {
  float angle;
  float bias;
  float rate;
  float P[2][2];
} Kalman_t;

Kalman_t kalmanRoll, kalmanPitch;

// ================= KALMAN FUNCTIONS =================
void initKalman(Kalman_t &k) {
  k.angle = 0;
  k.bias = 0;
  k.P[0][0] = 1; k.P[0][1] = 0;
  k.P[1][0] = 0; k.P[1][1] = 1;
}

float kalmanUpdate(Kalman_t &k, float newAngle, float newRate, float dt) {
  k.rate = newRate - k.bias;
  k.angle += dt * k.rate;

  k.P[0][0] += dt * (dt*k.P[1][1] - k.P[0][1] - k.P[1][0] + 0.001);
  k.P[0][1] -= dt * k.P[1][1];
  k.P[1][0] -= dt * k.P[1][1];
  k.P[1][1] += 0.003 * dt;

  float S = k.P[0][0] + 0.03;
  float K0 = k.P[0][0] / S;
  float K1 = k.P[1][0] / S;

  float y = newAngle - k.angle;
  k.angle += K0 * y;
  k.bias  += K1 * y;

  float P00 = k.P[0][0];
  float P01 = k.P[0][1];

  k.P[0][0] -= K0 * P00;
  k.P[0][1] -= K0 * P01;
  k.P[1][0] -= K1 * P00;
  k.P[1][1] -= K1 * P01;

  return k.angle;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  ledcAttach(motorFL, PWM_FREQ, PWM_RES);
  ledcAttach(motorFR, PWM_FREQ, PWM_RES);
  ledcAttach(motorRL, PWM_FREQ, PWM_RES);
  ledcAttach(motorRR, PWM_FREQ, PWM_RES);

  stopMotors();

  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(500);

  mpu.begin();
  delay(1000);
  mpu.calcGyroOffsets(true);

  initKalman(kalmanRoll);
  initKalman(kalmanPitch);

  RemoteXY_Init();
}

// ================= LOOP =================
void loop() {
  RemoteXYEngine.handler();

  if (RemoteXY.connect_flag == 0) {
    stopMotors();
    isArmed = false;
    return;
  }

  if (millis() - last_loop_time >= 4) {
    last_loop_time = millis();
    mpu.update();

    float dt = 0.004;

    // ----- RAW ACC ANGLES -----
    float accRoll  = atan2(mpu.getAccY(), mpu.getAccZ()) * RAD_TO_DEG;
    float accPitch = atan2(-mpu.getAccX(),
                      sqrt(mpu.getAccY()*mpu.getAccY() + mpu.getAccZ()*mpu.getAccZ()))
                      * RAD_TO_DEG;

    // ----- GYRO -----
    float gyroRoll  = mpu.getGyroX();
    float gyroPitch = mpu.getGyroY();

    // ----- KALMAN FILTERED ANGLES -----
float actualRoll  =  kalmanUpdate(kalmanRoll,  accPitch, gyroPitch, dt);
float actualPitch = -kalmanUpdate(kalmanPitch, accRoll,  gyroRoll,  dt);

    Serial.print("Roll: ");
Serial.print(actualRoll);
Serial.print("  Pitch: ");
Serial.println(actualPitch);


    // ----- THROTTLE -----
    int throttle = map(RemoteXY.joystick_01_y, -100, 100, 0, MAX_PWM);
    throttle = constrain(throttle, 0, MAX_PWM);

    if (throttle < 50) isArmed = true;
    if (!isArmed) throttle = 0;

    int cmd_roll  = abs(RemoteXY.joystick_02_x) > 5 ? RemoteXY.joystick_02_x : 0;
    int cmd_pitch = abs(RemoteXY.joystick_02_y) > 5 ? RemoteXY.joystick_02_y : 0;
    int cmd_yaw   = abs(RemoteXY.joystick_01_x) > 5 ? RemoteXY.joystick_01_x : 0;

    pid_p_gain = RemoteXY.slider_01 * 0.1;

    if (abs(actualRoll) > MAX_TILT || abs(actualPitch) > MAX_TILT) {
      stopMotors();
      return;
    }

    float roll_target  = cmd_roll * 0.2;
    float pitch_target = cmd_pitch * 0.2;

    float roll_error  = roll_target  - actualRoll;
    float pitch_error = pitch_target - actualPitch;

    float roll_pid  = (roll_error  * pid_roll_kp  * pid_p_gain) - (gyroRoll  * pid_roll_kd);
    float pitch_pid = (pitch_error * pid_pitch_kp * pid_p_gain) - (gyroPitch * pid_pitch_kd);
    float yaw_pid   = cmd_yaw * pid_yaw_kp;

    int fl = throttle + pitch_pid + roll_pid + yaw_pid;
    int fr = throttle + pitch_pid - roll_pid - yaw_pid;
    int rl = throttle - pitch_pid + roll_pid - yaw_pid;
    int rr = throttle - pitch_pid - roll_pid + yaw_pid;

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

// ================= STOP MOTORS =================
void stopMotors() {
  ledcWrite(motorFL, 0);
  ledcWrite(motorFR, 0);
  ledcWrite(motorRL, 0);
  ledcWrite(motorRR, 0);
}
