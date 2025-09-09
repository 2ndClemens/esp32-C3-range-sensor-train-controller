#include <Wire.h>
#include <VL53L0X.h>

// ===================== Sensors =====================
VL53L0X sensor1;
VL53L0X sensor2;

#define XSHUT1 4
#define XSHUT2 5

// ===================== Motor Driver =====================
#define mot1 10
#define mot2 8

#define PWM_CHANNEL1 0
#define PWM_CHANNEL2 1
#define PWM_FREQ 5000
#define PWM_RES 8    // 0–255

// ===================== Track Parameters =====================
const float trackLength = 100.0; // cm
const float stopOffset  = 5.0;   // stop before end
int maxSpeedPWM = 200;           // PWM value for cruising speed
int accelStep   = 3;             // PWM increment per step
unsigned long accelInterval = 40; // ms between speed updates
unsigned long dwellTime     = 5000; // ms standing still

// ===================== Train State Machine =====================
enum TrainState {
  RUN_FORWARD,
  BRAKING_FORWARD,
  DWELL_AT_B,
  RUN_BACKWARD,
  BRAKING_BACKWARD,
  DWELL_AT_A
};

TrainState state = RUN_FORWARD;

int currentPWM = 0;    // current speed (PWM)
int targetPWM  = 0;
int direction  = 1;    // +1 forward, -1 backward

// Timing
unsigned long lastUpdate = 0;
unsigned long dwellStart = 0;

// ===================== Position / Speed Estimation =====================
float position = 0;         // cm along track
float prevPosition = 0;
float speed = 0;            // cm/s
unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);

  // Setup PWM
  ledcSetup(PWM_CHANNEL1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RES);
  ledcAttachPin(mot1, PWM_CHANNEL1);
  ledcAttachPin(mot2, PWM_CHANNEL2);

  // Init I2C
  Wire.begin(18, 19);

  // Shutdown pins
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  delay(10);

  // Sensor 1
  digitalWrite(XSHUT1, HIGH);
  delay(10);
  if (!sensor1.init()) {
    Serial.println("Failed to init sensor 1");
    while (1);
  }
  sensor1.setAddress(0x30);

  // Sensor 2
  digitalWrite(XSHUT2, HIGH);
  delay(10);
  if (!sensor2.init()) {
    Serial.println("Failed to init sensor 2");
    while (1);
  }
  sensor2.setAddress(0x31);

  sensor1.setMeasurementTimingBudget(50000);
  sensor2.setMeasurementTimingBudget(50000);

  prevTime = millis();
}

// ===================== Motor Control =====================
void setMotor(int dir, int pwm) {
  pwm = constrain(pwm, 0, 255);
  if (dir == 1) { // forward
    ledcWrite(PWM_CHANNEL1, pwm);
    ledcWrite(PWM_CHANNEL2, 0);
  } else if (dir == -1) { // backward
    ledcWrite(PWM_CHANNEL1, 0);
    ledcWrite(PWM_CHANNEL2, pwm);
  } else { // stop
    ledcWrite(PWM_CHANNEL1, 0);
    ledcWrite(PWM_CHANNEL2, 0);
  }
}

// ===================== Update Position & Speed =====================
void updatePosition() {
  uint16_t d1 = sensor1.readRangeSingleMillimeters();
  uint16_t d2 = sensor2.readRangeSingleMillimeters();

  // crude filtering (ignore out of range values)
  if (d1 > 2000) d1 = 2000;
  if (d2 > 2000) d2 = 2000;

  // Position estimate: closer sensor dominates
  position = (float)d1 / 10.0;                   // cm from sensor1 side
  if (d2 < d1) {
    position = trackLength - (float)d2 / 10.0;   // cm from sensor2 side
  }

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0; // seconds
  if (dt > 0.05) {
    speed = (position - prevPosition) / dt;
    prevPosition = position;
    prevTime = now;
  }

  Serial.print("Pos: ");
  Serial.print(position, 1);
  Serial.print(" cm  Speed: ");
  Serial.print(speed, 1);
  Serial.print(" cm/s  State: ");
  Serial.println(state);
}

// ===================== Smooth Speed Ramping =====================
void updateSpeed() {
  unsigned long now = millis();
  if (now - lastUpdate >= accelInterval) {
    if (currentPWM < targetPWM) currentPWM += accelStep;
    if (currentPWM > targetPWM) currentPWM -= accelStep;

    setMotor(direction, currentPWM);
    lastUpdate = now;
  }
}

// ===================== Main Loop =====================
void loop() {
  updatePosition();   // read sensors, estimate pos/speed
  updateSpeed();      // ramp toward target speed

  switch (state) {
    case RUN_FORWARD:
      direction = 1;
      targetPWM = maxSpeedPWM;
      if (position >= trackLength - stopOffset) {
        state = BRAKING_FORWARD;
      }
      break;

    case BRAKING_FORWARD:
      direction = 1;
      targetPWM = 0;
      if (currentPWM == 0) {
        dwellStart = millis();
        state = DWELL_AT_B;
        Serial.println("Arrived at Station B");
      }
      break;

    case DWELL_AT_B:
      if (millis() - dwellStart >= dwellTime) {
        direction = -1;
        targetPWM = maxSpeedPWM;
        state = RUN_BACKWARD;
        Serial.println("Departing Station B");
      }
      break;

    case RUN_BACKWARD:
      direction = -1;
      targetPWM = maxSpeedPWM;
      if (position <= stopOffset) {
        state = BRAKING_BACKWARD;
      }
      break;

    case BRAKING_BACKWARD:
      direction = -1;
      targetPWM = 0;
      if (currentPWM == 0) {
        dwellStart = millis();
        state = DWELL_AT_A;
        Serial.println("Arrived at Station A");
      }
      break;

    case DWELL_AT_A:
      if (millis() - dwellStart >= dwellTime) {
        direction = 1;
        targetPWM = maxSpeedPWM;
        state = RUN_FORWARD;
        Serial.println("Departing Station A");
      }
      break;
  }
}
