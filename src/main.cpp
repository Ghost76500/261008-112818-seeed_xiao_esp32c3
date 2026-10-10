#include <Arduino.h>
#include <Wire.h>

#define LED_YELLOW_PIN 8
#define LED_GREEN_PIN 1

constexpr uint8_t MPU6050_ADDRESSES[] = {0x68, 0x69};
constexpr uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU6050_REG_WHO_AM_I = 0x75;
constexpr int SDA_PIN = 6;
constexpr int SCL_PIN = 5;

uint8_t mpu6050Address = 0;
bool mpu6050Ready = false;
unsigned long lastSampleTime = 0;
unsigned long lastLedToggleTime = 0;
bool ledState = false;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpu6050Address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t firstReg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(mpu6050Address);
  Wire.write(firstReg);
  if (Wire.endTransmission(true) != 0) {
    return false;
  }

  if (Wire.requestFrom(mpu6050Address, length, true) != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    if (!Wire.available()) {
      return false;
    }
    buffer[i] = Wire.read();
  }
  return true;
}

bool initializeMpu6050() {
  mpu6050Address = 0;
  for (uint8_t address : MPU6050_ADDRESSES) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      mpu6050Address = address;
      break;
    }
  }
  if (mpu6050Address == 0) {
    Serial.println(
        "No I2C device ACK at 0x68 or 0x69. Check GPIO6/7, VCC, GND, and wiring.");
    return false;
  }

  uint8_t identity = 0;
  if (!readRegisters(MPU6050_REG_WHO_AM_I, &identity, 1)) {
    Serial.printf("I2C device at 0x%02X ACKed, but WHO_AM_I read failed.\n",
                  mpu6050Address);
    return false;
  }
  if (identity != 0x68) {
    Serial.printf("Device at 0x%02X has unexpected WHO_AM_I: 0x%02X\n",
                  mpu6050Address, identity);
    return false;
  }
  if (!writeRegister(MPU6050_REG_PWR_MGMT_1, 0x00)) {
    Serial.println("MPU6050 wake-up command failed.");
    return false;
  }

  Serial.println("MPU6050 initialized.");
  return true;
}

int16_t readInt16(const uint8_t *buffer, size_t index) {
  return static_cast<int16_t>(
      (static_cast<uint16_t>(buffer[index]) << 8) | buffer[index + 1]);
}

void printMpu6050Data() {
  uint8_t data[14];
  if (!readRegisters(MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data))) {
    Serial.println("MPU6050 data read failed.");
    mpu6050Ready = false;
    return;
  }

  const float accelX = readInt16(data, 0) / 16384.0f;
  const float accelY = readInt16(data, 2) / 16384.0f;
  const float accelZ = readInt16(data, 4) / 16384.0f;
  const float temperature = readInt16(data, 6) / 340.0f + 36.53f;
  const float gyroX = readInt16(data, 8) / 131.0f;
  const float gyroY = readInt16(data, 10) / 131.0f;
  const float gyroZ = readInt16(data, 12) / 131.0f;

  Serial.printf("Accel (g): X=%.2f Y=%.2f Z=%.2f | "
                "Gyro (deg/s): X=%.2f Y=%.2f Z=%.2f | Temp: %.2f C\n",
                accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temperature);
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  mpu6050Ready = initializeMpu6050();
}

void loop() {
  const unsigned long now = millis();

  if (!mpu6050Ready && now - lastSampleTime >= 2000) {
    lastSampleTime = now;
    mpu6050Ready = initializeMpu6050();
  }
  if (mpu6050Ready && now - lastSampleTime >= 500) {
    lastSampleTime = now;
    printMpu6050Data();
  }

  if (now - lastLedToggleTime >= 2000) {
    lastLedToggleTime = now;
    ledState = !ledState;
    digitalWrite(LED_YELLOW_PIN, ledState ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, ledState ? LOW : HIGH);
  }
}