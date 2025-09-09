#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;

#define mot1 10
#define mot2 8

#define XSHUT1 4
#define XSHUT2 5

// PWM settings
#define PWM_CHANNEL1 0
#define PWM_CHANNEL2 1
#define PWM_FREQ 5000
#define PWM_RES 8    // 8-bit: 0–255

int currentSpeed = 0;
int maxSpeed = 200;     // adjust for top speed (0–255)
int accelStep = 5;      // speed increment
int accelDelay = 30;    // ms between speed steps
int dwellTime = 5000;   // stop time in ms

void setup() {
  Serial.begin(115200);

  // Attach PWM channels
  ledcSetup(PWM_CHANNEL1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RES);

  ledcAttachPin(mot1, PWM_CHANNEL1);
  ledcAttachPin(mot2, PWM_CHANNEL2);

  // Sensors
  Wire.begin(18, 19); // SDA, SCL

  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);

  // Keep both sensors off
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  delay(10);

  // Init sensor 1
  digitalWrite(XSHUT1, HIGH);
  delay(10);
  if (!sensor1.init()) {
    Serial.println("Failed to init sensor 1");
    while (1);
  }
  sensor1.setAddress(0x30);
  Serial.println("Sensor 1 at 0x30");

  // Init sensor 2
  digitalWrite(XSHUT2, HIGH);
  delay(10);
  if (!sensor2.init()) {
    Serial.println("Failed to init sensor 2");
    while (1);
  }
  sensor2.setAddress(0x31);
  Serial.println("Sensor 2 at 0x31");

  sensor1.setMeasurementTimingBudget(50000);
  sensor2.setMeasurementTimingBudget(50000);
}

void setMotor(int dir, int speed) {
  if (dir == 1) { // forward
    ledcWrite(PWM_CHANNEL1, speed);
    ledcWrite(PWM_CHANNEL2, 0);
  } else if (dir == -1) { // backward
    ledcWrite(PWM_CHANNEL1, 0);
    ledcWrite(PWM_CHANNEL2, speed);
  } else { // stop
    ledcWrite(PWM_CHANNEL1, 0);
    ledcWrite(PWM_CHANNEL2, 0);
  }
}

void rampMotor(int dir, int targetSpeed) {
  while (currentSpeed != targetSpeed) {
    if (currentSpeed < targetSpeed) currentSpeed += accelStep;
    else if (currentSpeed > targetSpeed) currentSpeed -= accelStep;

    setMotor(dir, abs(currentSpeed));
    delay(accelDelay);
  }
}

void loop() {
  uint16_t dist1 = sensor1.readRangeSingleMillimeters();
  uint16_t dist2 = sensor2.readRangeSingleMillimeters();

  Serial.print("S1: ");
  Serial.print(dist1);
  Serial.print(" mm\tS2: ");
  Serial.print(dist2);
  Serial.println(" mm");

  delay(100);

  // If close to sensor 2 → stop, wait, then reverse
  if (dist2 < 60) {
    rampMotor(1, 0);         // decelerate to stop
    Serial.println("Arrived at Station B - waiting...");
    delay(dwellTime);        // stand still for passengers
    rampMotor(-1, maxSpeed); // accelerate backward
    Serial.println("Departing Station B");
  }

  // If close to sensor 1 → stop, wait, then forward
  if (dist1 < 60) {
    rampMotor(-1, 0);        // decelerate to stop
    Serial.println("Arrived at Station A - waiting...");
    delay(dwellTime);        // stand still for passengers
    rampMotor(1, maxSpeed);  // accelerate forward
    Serial.println("Departing Station A");
  }
}
