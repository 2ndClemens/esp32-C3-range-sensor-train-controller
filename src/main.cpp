#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;

#define XSHUT1 4
#define XSHUT2 5

void setup() {
  Serial.begin(115200);
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
  sensor1.setAddress(0x30); // New address
  Serial.println("Sensor 1 at 0x30");

  // Init sensor 2
  digitalWrite(XSHUT2, HIGH);
  delay(10);
  if (!sensor2.init()) {
    Serial.println("Failed to init sensor 2");
    while (1);
  }
  sensor2.setAddress(0x31); // New address
  Serial.println("Sensor 2 at 0x31");

  // Optional tuning
  sensor1.setMeasurementTimingBudget(50000);
  sensor2.setMeasurementTimingBudget(50000);
}

void loop() {
  uint16_t dist1 = sensor1.readRangeSingleMillimeters();
  uint16_t dist2 = sensor2.readRangeSingleMillimeters();

  Serial.print("S1: ");
  Serial.print(dist1);
  Serial.print(" mm\tS2: ");
  Serial.print(dist2);
  Serial.println(" mm");

  delay(200);
}
