#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define MT6701_ADDR 0x06  // scan if unsure

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
}

uint16_t readAngleRaw() {
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(0x03); // angle register
  Wire.endTransmission(false);

  Wire.requestFrom(MT6701_ADDR, 2);

  if (Wire.available() < 2) return 0;

  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();

  // Combine 14-bit value
  uint16_t angle = ((highByte << 6) | (lowByte & 0x3F));

  return angle;
}

float readAngleDegrees() {
  return (readAngleRaw() * 360.0) / 16384.0;
}

void loop() {
  float angle = readAngleDegrees();
  Serial.printf("Angle: %.2f°\n", angle);
  delay(100);
}