#pragma once

#include <Arduino.h>

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
