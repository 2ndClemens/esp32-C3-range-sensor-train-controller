#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

void setup() {
  Serial.begin(115200);
  Wire.begin(18, 19); // or your chosen pins

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  // Optional: better short-range stability
  sensor.setSignalRateLimit(0.1);            // Lower limit = better close range accuracy
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  // Increase timing budget for accuracy (default = 33 ms)
  sensor.setMeasurementTimingBudget(50000);  // 50 ms
   sensor.setSignalRateLimit(.25);
   //sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
//sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
}

void loop() {
  uint16_t distance = sensor.readRangeSingleMillimeters();

  if (sensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  } else {
    Serial.print("Distance (mm): ");
    Serial.println(distance);
  }

  delay(100);
}
