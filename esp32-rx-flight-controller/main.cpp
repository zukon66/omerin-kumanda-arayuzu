#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

#define NRF_CE_PIN 4
#define NRF_CSN_PIN 5
#define NRF_SCK_PIN 18
#define NRF_MOSI_PIN 23
#define NRF_MISO_PIN 19

#define ESC_PIN 26
#define SERVO_ROLL_PIN 13
#define SERVO_PITCH_PIN 14
#define SERVO_YAW_PIN 27

#define BUZZER_PIN 32
#define BATTERY_ADC_PIN 34

#define PWM_MIN_US 1000
#define PWM_CENTER_US 1500
#define PWM_MAX_US 2000

#define MODE_TURTLE 0
#define MODE_NORMAL 1
#define MODE_SPORT 2

#define TOGGLE_NONE 0
#define TOGGLE_BUZZER 1

#define FAILSAFE_TIMEOUT_MS 1000UL
#define IMU_UPDATE_MS 10UL
#define TELEMETRY_UPDATE_MS 50UL
#define BUZZER_TOGGLE_MS 150UL

#define ADC_REFERENCE_VOLTAGE 3.3f
#define ADC_MAX_COUNT 4095.0f
#define BATTERY_DIVIDER_RATIO 2.0f
#define BATTERY_ADC_SAMPLES 8

struct __attribute__((packed)) TX_Payload {
  uint16_t throttlePwm;
  uint16_t yawPwm;
  uint16_t pitchPwm;
  uint16_t rollPwm;
  int16_t encoderValue;
  uint8_t activeKeyId;
  uint8_t packetId;
};

struct __attribute__((packed)) RX_Payload {
  uint16_t aircraftBatteryMv;
  uint8_t satellites;
  uint16_t speedKmh;
  int16_t altitudeM;
  uint16_t headingDeg;
  int32_t latitudeE7;
  int32_t longitudeE7;
  int16_t rollDeg10;
  int16_t pitchDeg10;
  uint8_t signalPercent;
  uint32_t distanceM;
  uint16_t pressureHpa;
  int16_t cameraAngleDeg;
};

static_assert(sizeof(TX_Payload) <= 32, "TX payload too large");
static_assert(sizeof(RX_Payload) <= 32, "RX payload too large");

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
Servo esc;
Servo servoRoll;
Servo servoPitch;
Servo servoYaw;
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
HardwareSerial GPSSerial(2);

const byte radioAddress[6] = "UAV01";

TX_Payload rxCommand = {
  PWM_MIN_US,
  PWM_CENTER_US,
  PWM_CENTER_US,
  PWM_CENTER_US,
  0,
  0,
  0
};

RX_Payload telemetry = {
  3700,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  1013,
  0
};

unsigned long lastPacketMs = 0;
unsigned long lastImuUpdateMs = 0;
unsigned long lastTelemetryUpdateMs = 0;
unsigned long lastBuzzerToggleMs = 0;
bool failsafeActive = true;
bool buzzerRequested = false;
bool buzzerState = false;

float rollAngleDeg = 0.0f;
float pitchAngleDeg = 0.0f;
bool imuReady = false;

uint16_t clampPwm(uint16_t value) {
  if (value < PWM_MIN_US) {
    return PWM_MIN_US;
  }
  if (value > PWM_MAX_US) {
    return PWM_MAX_US;
  }
  return value;
}

uint16_t escLimitForMode(uint8_t mode) {
  if (mode == MODE_TURTLE) {
    return 1500;
  }
  if (mode == MODE_SPORT) {
    return 2000;
  }
  return 1750;
}

void writeOutputs(uint16_t throttle, uint16_t roll, uint16_t pitch, uint16_t yaw) {
  esc.writeMicroseconds(clampPwm(throttle));
  servoRoll.writeMicroseconds(clampPwm(roll));
  servoPitch.writeMicroseconds(clampPwm(pitch));
  servoYaw.writeMicroseconds(clampPwm(yaw));
}

void applyFailsafe() {
  failsafeActive = true;
  buzzerRequested = false;
  writeOutputs(PWM_MIN_US, PWM_CENTER_US, PWM_CENTER_US, PWM_CENTER_US);
}

void applyCommand() {
  uint8_t mode = MODE_NORMAL;
  if (rxCommand.activeKeyId == 1) {
    mode = MODE_TURTLE;
  } else if (rxCommand.activeKeyId == 3) {
    mode = MODE_SPORT;
  }

  uint16_t throttle = clampPwm(rxCommand.throttlePwm);
  uint16_t maxThrottle = escLimitForMode(mode);

  if (throttle > maxThrottle) {
    throttle = maxThrottle;
  }

  buzzerRequested = (rxCommand.activeKeyId == 9);

  writeOutputs(
    throttle,
    clampPwm(rxCommand.rollPwm),
    clampPwm(rxCommand.pitchPwm),
    clampPwm(rxCommand.yawPwm)
  );
}

void updateBuzzer() {
  if (!buzzerRequested) {
    buzzerState = false;
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  unsigned long now = millis();
  if (now - lastBuzzerToggleMs >= BUZZER_TOGGLE_MS) {
    lastBuzzerToggleMs = now;
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
  }
}

float readBatteryVoltage() {
  uint32_t total = 0;

  for (uint8_t i = 0; i < BATTERY_ADC_SAMPLES; i++) {
    total += analogRead(BATTERY_ADC_PIN);
  }

  float adcValue = (float)total / (float)BATTERY_ADC_SAMPLES;
  float adcVoltage = (adcValue / ADC_MAX_COUNT) * ADC_REFERENCE_VOLTAGE;
  return adcVoltage * BATTERY_DIVIDER_RATIO;
}

void updateGps() {
  while (GPSSerial.available() > 0) {
    gps.encode((char)GPSSerial.read());
  }
}

void updateImu() {
  if (!imuReady) {
    return;
  }

  unsigned long now = millis();
  if (now - lastImuUpdateMs < IMU_UPDATE_MS) {
    return;
  }

  lastImuUpdateMs = now;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float accelRoll = atan2f(accel.acceleration.y, accel.acceleration.z) * 180.0f / PI;
  float accelPitch = atan2f(
    -accel.acceleration.x,
    sqrtf((accel.acceleration.y * accel.acceleration.y) +
          (accel.acceleration.z * accel.acceleration.z))
  ) * 180.0f / PI;

  rollAngleDeg = (0.96f * rollAngleDeg) + (0.04f * accelRoll);
  pitchAngleDeg = (0.96f * pitchAngleDeg) + (0.04f * accelPitch);
}

void updateTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemetryUpdateMs < TELEMETRY_UPDATE_MS) {
    return;
  }

  lastTelemetryUpdateMs = now;

  telemetry.aircraftBatteryMv = (uint16_t)constrain((int)(readBatteryVoltage() * 1000.0f), 0, 65535);
  telemetry.satellites = gps.satellites.isValid() ? (uint8_t)constrain((int)gps.satellites.value(), 0, 255) : 0;
  telemetry.speedKmh = gps.speed.isValid() ? (uint16_t)constrain((int)gps.speed.kmph(), 0, 65535) : 0;
  telemetry.altitudeM = gps.altitude.isValid() ? (int16_t)constrain((int)gps.altitude.meters(), -32768, 32767) : 0;
  telemetry.headingDeg = gps.course.isValid() ? (uint16_t)constrain((int)gps.course.deg(), 0, 359) : 0;
  telemetry.latitudeE7 = gps.location.isValid() ? (int32_t)(gps.location.lat() * 10000000.0) : 0;
  telemetry.longitudeE7 = gps.location.isValid() ? (int32_t)(gps.location.lng() * 10000000.0) : 0;
  telemetry.rollDeg10 = (int16_t)constrain((int)(rollAngleDeg * 10.0f), -32768, 32767);
  telemetry.pitchDeg10 = (int16_t)constrain((int)(pitchAngleDeg * 10.0f), -32768, 32767);
  telemetry.signalPercent = 100;
  telemetry.distanceM = 0;
  telemetry.pressureHpa = 1013;
  telemetry.cameraAngleDeg = 0;

  radio.writeAckPayload(1, &telemetry, sizeof(telemetry));
}

void handleRadio() {
  while (radio.available()) {
    radio.read(&rxCommand, sizeof(rxCommand));
    lastPacketMs = millis();
    failsafeActive = false;
    applyCommand();
    radio.writeAckPayload(1, &telemetry, sizeof(telemetry));
  }
}

void setupRadio() {
  SPI.begin(NRF_SCK_PIN, NRF_MISO_PIN, NRF_MOSI_PIN, NRF_CSN_PIN);
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openReadingPipe(1, radioAddress);
  radio.startListening();
  radio.writeAckPayload(1, &telemetry, sizeof(telemetry));
}

void setupImu() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  imuReady = mpu.begin();

  if (imuReady) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
}

void setupOutputs() {
  esc.setPeriodHertz(50);
  servoRoll.setPeriodHertz(50);
  servoPitch.setPeriodHertz(50);
  servoYaw.setPeriodHertz(50);

  esc.attach(ESC_PIN, PWM_MIN_US, PWM_MAX_US);
  servoRoll.attach(SERVO_ROLL_PIN, PWM_MIN_US, PWM_MAX_US);
  servoPitch.attach(SERVO_PITCH_PIN, PWM_MIN_US, PWM_MAX_US);
  servoYaw.attach(SERVO_YAW_PIN, PWM_MIN_US, PWM_MAX_US);

  writeOutputs(PWM_MIN_US, PWM_CENTER_US, PWM_CENTER_US, PWM_CENTER_US);
}

void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

  setupOutputs();
  setupImu();
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  setupRadio();

  lastPacketMs = millis();
  lastImuUpdateMs = millis();
  lastTelemetryUpdateMs = millis();
  lastBuzzerToggleMs = millis();
  applyFailsafe();

  Serial.println("RX flight controller ready");
}

void loop() {
  updateGps();
  updateImu();
  updateTelemetry();
  handleRadio();

  if (!failsafeActive && (millis() - lastPacketMs > FAILSAFE_TIMEOUT_MS)) {
    applyFailsafe();
    Serial.println("FAILSAFE active");
  }

  updateBuzzer();
}
