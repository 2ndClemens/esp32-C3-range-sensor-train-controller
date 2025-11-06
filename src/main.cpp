#include <Wire.h>
#include <VL53L0X.h>
#include <Arduino.h>
#include "Freenove_WS2812_Lib_for_ESP32.h"

typedef uint16_t u16;
typedef uint8_t u8;

#define LEDS_COUNT 1
#define LEDS_PIN 2
#define CHANNEL 0

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

u8 m_color[5][3] = {
    {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}, {0, 0, 0}};

VL53L0X sensor1;
VL53L0X sensor2;

// Motor pins
#define mot1 5
#define mot2 4

// PWM channels for motor control
#define PWM_CH1 1
#define PWM_CH2 2
#define PWM_FREQ 300
#define PWM_RES 8 // 8-bit resolution (0–255)

#define XSHUT1 8
#define XSHUT2 10

int TARGET_MOTOR_SPEED = 235;
int MOTOR_SPEED = 0; // out of 255 → ~60% speed
// Distance thresholds (mm)
const uint16_t SPEED_DIST = 400;  // when to decelerate
const uint16_t TRIGGER_DIST = 50; // when to flip direction
const uint16_t EXIT_DIST = 80;    // hysteresis: must move past this before allowing another flip

uint16_t dist1 = 0;
uint16_t dist2 = 0;

uint16_t dist1prev = 0;
uint16_t dist2prev = 0;

// direction state: 1 = forward (towards sensor2), -1 = backward (towards sensor1)
int8_t direction = 1;
bool canSwitch = true;
int speed = 0;

void applyMotor(int8_t dir)
{
  // dir: 1 = forward -> mot1 = PWM, mot2 = 0
  //     -1 = backward -> mot2 = PWM, mot1 = 0
  if (dir > 0)
  {
    ledcWrite(PWM_CH1, MOTOR_SPEED);
    ledcWrite(PWM_CH2, 0);
  }
  else if (dir < 0)
  {
    ledcWrite(PWM_CH1, 0);
    ledcWrite(PWM_CH2, MOTOR_SPEED);
  }
  else
  {
    ledcWrite(PWM_CH1, 0);
    ledcWrite(PWM_CH2, 0);
  }
}
void setup()
{
  Serial.begin(115200);

  strip.begin();
  strip.setBrightness(10);
  strip.setLedColorData(0, m_color[0][0], m_color[0][1], m_color[0][2]);
  strip.show();

  // Set up PWM for both motor pins
  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(mot1, PWM_CH1);
  ledcAttachPin(mot2, PWM_CH2);

  // Initially stop
  ledcWrite(PWM_CH1, 0);
  ledcWrite(PWM_CH2, 0);

  Wire.begin(18, 19); // SDA, SCL
  // Wire.setClock(10000);

  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);

  // Keep both sensors off initially
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  delay(10);

  // Init sensor 1
  digitalWrite(XSHUT1, HIGH);
  delay(10);
  if (!sensor1.init())
  {
    Serial.println("Failed to init sensor 1");
    while (1)
      ;
  }
  sensor1.setAddress(0x30); // New address
  Serial.println("Sensor 1 at 0x30");

  // Init sensor 2
  digitalWrite(XSHUT2, HIGH);
  delay(10);
  if (!sensor2.init())
  {
    Serial.println("Failed to init sensor 2");
    while (1)
      ;
  }
  sensor2.setAddress(0x31); // New address
  Serial.println("Sensor 2 at 0x31");

  sensor1.setMeasurementTimingBudget(50000);
  sensor2.setMeasurementTimingBudget(50000);

  // start moving in initial direction
  applyMotor(direction);
}

void loop()
{

  dist1 = sensor1.readRangeSingleMillimeters();
  dist2 = sensor2.readRangeSingleMillimeters();

  Serial.print("\tS1: ");
  Serial.print(dist1);
  Serial.print(" mm\tS2: ");
  Serial.print(dist2);
  Serial.print(" mm\tDir: ");
  Serial.println(direction > 0 ? "FWD" : "BWD");

  if (dist2 < SPEED_DIST)
  {
    speed = dist2prev - dist2;
    if (speed > 10)
    {
      MOTOR_SPEED -= (40 * speed)/dist2 ;
      if (MOTOR_SPEED < 0)
      {
        MOTOR_SPEED = 0;
      }
    }
  }

  if (dist1 < SPEED_DIST)
  {
    speed = dist1prev - dist1;
    if (speed > 10)
    {
      MOTOR_SPEED -= (40 * speed)/dist1;
      if (MOTOR_SPEED < 0)
      {
        MOTOR_SPEED = 0;
      }
    }
  }

  // If moving forward (towards sensor2)
  if (direction > 0)
  {
    applyMotor(1); // keep driving forward
    // if reached sensor2 and switching allowed -> reverse
    if (dist2 < TRIGGER_DIST)
    {

      if (canSwitch)
      {

        direction = -1;
        MOTOR_SPEED = 0;
        canSwitch = false;
        Serial.println("Reached sensor2: reversing -> BACKWARD");
        strip.setLedColorData(0, m_color[2][0], m_color[2][1], m_color[2][2]); // blue for backward
        strip.show();
        // delay(100); // small pause to ensure motor direction change registers
      }
    }
  }
  // If moving backward (towards sensor1)
  else if (direction < 0)
  {
    applyMotor(-1); // keep driving backward
    // if reached sensor1 and switching allowed -> forward
    if (dist1 < TRIGGER_DIST)
    {

      if (canSwitch)
      {
        direction = 1;
        MOTOR_SPEED = 0;
        canSwitch = false;
        Serial.println("Reached sensor1: reversing -> FORWARD");
        strip.setLedColorData(0, m_color[1][0], m_color[1][1], m_color[1][2]); // green for forward
        strip.show();
        // delay(100);
      }
    }
  }

  // Re-enable switching only after both sensors read beyond EXIT_DIST (train has left the trigger zone)
  if (!canSwitch)
  {
    if (MOTOR_SPEED < TARGET_MOTOR_SPEED)
    {
      MOTOR_SPEED += 5;
    }
    if (MOTOR_SPEED > TARGET_MOTOR_SPEED)
    {
      MOTOR_SPEED -= 5;
    }
    if (dist1 > EXIT_DIST && dist2 > EXIT_DIST)
    {
      canSwitch = true;
      Serial.println("Switch re-enabled (train left both trigger zones)");
    }
  }
  Serial.print(" speed: ");
  Serial.print(speed);
  dist1prev = dist1;
  dist2prev = dist2;

  delay(10);
}
