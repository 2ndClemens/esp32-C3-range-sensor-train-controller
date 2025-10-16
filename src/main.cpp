#include <Wire.h>
#include <VL53L0X.h>
#include <Arduino.h>
#include "Freenove_WS2812_Lib_for_ESP32.h"

typedef uint16_t u16;
typedef uint8_t u8;

#define LEDS_COUNT  1
#define LEDS_PIN	  2
#define CHANNEL		  0

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

u8 m_color[5][3] = {
  {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}, {0, 0, 0}
};

VL53L0X sensor1;
VL53L0X sensor2;

#define mot1 10
#define mot2 8

#define XSHUT1 4
#define XSHUT2 5

void setup() {
  Serial.begin(115200);

    strip.begin();
  strip.setBrightness(10);

  strip.setLedColorData(0, m_color[0][0], m_color[0][1], m_color[0][2]);
      strip.show();

  pinMode(mot1, OUTPUT);
  pinMode(mot2, OUTPUT);

  // Keep both sensors off
  digitalWrite(mot1, HIGH);
  digitalWrite(mot2, LOW);

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

  delay(60);

  if(dist2 < 100 ){
      digitalWrite(mot1, HIGH);
  digitalWrite(mot2, LOW);
  Serial.println("fw");
    strip.setLedColorData(0, m_color[1][0], m_color[1][1], m_color[1][2]);
      strip.show();
  }
    if(dist1 < 100 ){
      digitalWrite(mot2, HIGH);
  digitalWrite(mot1, LOW);
  Serial.println("bw");
    strip.setLedColorData(0, m_color[2][0], m_color[2][1], m_color[2][2]);
      strip.show();
  }
}


