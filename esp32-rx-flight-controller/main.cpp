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

struct RX_Command {
  uint16_t throttle;
  uint16_t roll;
  uint16_t pitch;
  uint16_t yaw;
  uint8_t flightMode;
  uint8_t toggleCommand;
};

struct TX_Telemetry {
  float batteryVoltage;
  uint8_t satelliteCount;
  float speedKmh;
  float altitudeM;
  float gpsCourseDeg;
  float rollDeg;
  float pitchDeg;
};

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
Servo esc;
Servo servoRoll;
Servo servoPitch;
Servo servoYaw;
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
HardwareSerial GPSSerial(2);

const byte radioAddress[6] = "PLANE";

RX_Command rxCommand = {
  PWM_MIN_US,
  PWM_CENTER_US,
  PWM_CENTER_US,
  PWM_CENTER_US,
  MODE_NORMAL,
  TOGGLE_NONE
};

TX_Telemetry telemetry = {
  0.0f,
  0,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
  0.0f
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
  uint16_t throttle = clampPwm(rxCommand.throttle);
  uint16_t maxThrottle = escLimitForMode(rxCommand.flightMode);

  if (throttle > maxThrottle) {
    throttle = maxThrottle;
  }

  buzzerRequested = (rxCommand.toggleCommand == TOGGLE_BUZZER);

  writeOutputs(
    throttle,
    clampPwm(rxCommand.roll),
    clampPwm(rxCommand.pitch),
    clampPwm(rxCommand.yaw)
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

  telemetry.batteryVoltage = readBatteryVoltage();
  telemetry.satelliteCount = gps.satellites.isValid() ? gps.satellites.value() : 0;
  telemetry.speedKmh = gps.speed.isValid() ? gps.speed.kmph() : 0.0f;
  telemetry.altitudeM = gps.altitude.isValid() ? gps.altitude.meters() : 0.0f;
  telemetry.gpsCourseDeg = gps.course.isValid() ? gps.course.deg() : 0.0f;
  telemetry.rollDeg = rollAngleDeg;
  telemetry.pitchDeg = pitchAngleDeg;

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
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

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
