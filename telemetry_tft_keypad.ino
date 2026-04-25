#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Keypad.h>
#include <RF24.h>

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

#define POWDER_BLUE tft.color565(176, 224, 230)

static const int SCREEN_W = 320;
static const int SCREEN_H = 240;
static const unsigned long POPUP_MS = 1500;
static const int UI_BACKLIGHT_PIN = -1;
static const int NRF_CE_PIN = 22;
static const int NRF_CSN_PIN = 15;
static const int JOY_THROTTLE_PIN = 34;
static const int JOY_YAW_PIN = 35;
static const int JOY_PITCH_PIN = 36;
static const int JOY_ROLL_PIN = 39;
static const int ENCODER_A_PIN = 16;
static const int ENCODER_B_PIN = 17;
static const int JOY_CENTER = 2048;
static const int JOY_DEADZONE = 150;
static const unsigned long RADIO_TX_MS = 40;
static const unsigned long LINK_TIMEOUT_MS = 1000;
static const unsigned long CONNECTED_BANNER_MS = 1000;

const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {13, 14, 27, 26};
byte colPins[COLS] = {25, 33, 32, 21};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

const byte RADIO_ADDRESS[6] = "UAV01";

enum LinkState {
  STATE_CONNECTING,
  STATE_CONNECTED,
  STATE_DISCONNECTED
};

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

TX_Payload txPayload = {1500, 1500, 1500, 1500, 0, 0, 0};
RX_Payload rxPayload = {3700, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1013, 0};

LinkState linkState = STATE_CONNECTING;
bool radioReady = false;
bool telemetryValid = false;
uint8_t activeKeyId = 0;
unsigned long lastRadioTxAt = 0;
unsigned long lastAckAt = 0;
unsigned long connectedBannerUntil = 0;
volatile int16_t encoderTicks = 0;
volatile uint8_t encoderLastState = 0;

void IRAM_ATTR encoderIsr();
void setupRadio();
void readControlInputs();
uint16_t readJoystickPwm(int pin);
void serviceRadio();
void updateLinkState();
void drawStatusScreen(const String &message);
void drawStatusOverlay(const String &message);

struct TelemetryData {
  int satellites;
  int aircraftBattery;
  int controllerBattery;
  int signal;
  int throttle;
  int altitude;
  int pressure;
  int speed;
  int distance;
  int cameraAngle;
  int heading;
  float latitude;
  float longitude;
  float roll;
  float pitch;
};

TelemetryData data;

bool popupActive = false;
unsigned long popupStartedAt = 0;
char popupText[24] = "";

unsigned long lastTelemetryAt = 0;
unsigned long lastFrameAt = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("ESP32 TFT UI boot");

  if (UI_BACKLIGHT_PIN >= 0) {
    pinMode(UI_BACKLIGHT_PIN, OUTPUT);
    digitalWrite(UI_BACKLIGHT_PIN, HIGH);
  }

  pinMode(JOY_THROTTLE_PIN, INPUT);
  pinMode(JOY_YAW_PIN, INPUT);
  pinMode(JOY_PITCH_PIN, INPUT);
  pinMode(JOY_ROLL_PIN, INPUT);
  analogReadResolution(12);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  encoderLastState = (digitalRead(ENCODER_A_PIN) << 1) | digitalRead(ENCODER_B_PIN);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderIsr, CHANGE);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  Serial.printf("Free heap before sprite: %u\n", ESP.getFreeHeap());
  spr.setColorDepth(8);
  void *spriteBuffer = spr.createSprite(SCREEN_W, SCREEN_H);
  if (!spriteBuffer) {
    Serial.println("Sprite allocation failed");
    while (true) {
      tft.fillScreen(TFT_BLACK);
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("SPRITE FAIL", SCREEN_W / 2, SCREEN_H / 2, 2);
      delay(1000);
    }
  }
  Serial.printf("Free heap after sprite: %u\n", ESP.getFreeHeap());

  spr.setTextDatum(MC_DATUM);
  spr.setSwapBytes(true);

  setupRadio();
  updateTelemetry();
  drawFrame();
  Serial.println("UI ready");
}

void loop() {
  handleKeypad();
  readControlInputs();
  serviceRadio();
  updateLinkState();
  handlePopupTimer();

  unsigned long now = millis();
  if (now - lastTelemetryAt >= 100) {
    updateTelemetry();
    lastTelemetryAt = now;
  }

  if (now - lastFrameAt >= 50) {
    drawFrame();
    lastFrameAt = now;
  }
}

void IRAM_ATTR encoderIsr() {
  uint8_t state = (digitalRead(ENCODER_A_PIN) << 1) | digitalRead(ENCODER_B_PIN);
  uint8_t transition = (encoderLastState << 2) | state;

  if (transition == 0b0001 || transition == 0b0111 || transition == 0b1110 || transition == 0b1000) {
    encoderTicks++;
  } else if (transition == 0b0010 || transition == 0b0100 || transition == 0b1101 || transition == 0b1011) {
    encoderTicks--;
  }

  encoderLastState = state;
}

void setupRadio() {
  radioReady = radio.begin();
  if (!radioReady) {
    Serial.println("RF24 begin failed");
    return;
  }

  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(3, 5);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108);
  radio.openWritingPipe(RADIO_ADDRESS);
  radio.stopListening();
  Serial.println("RF24 ready");
}

void readControlInputs() {
  txPayload.throttlePwm = readJoystickPwm(JOY_THROTTLE_PIN);
  txPayload.yawPwm = readJoystickPwm(JOY_YAW_PIN);
  txPayload.pitchPwm = readJoystickPwm(JOY_PITCH_PIN);
  txPayload.rollPwm = readJoystickPwm(JOY_ROLL_PIN);

  noInterrupts();
  int16_t ticks = encoderTicks;
  interrupts();

  txPayload.encoderValue = ticks;
  txPayload.activeKeyId = activeKeyId;
}

uint16_t readJoystickPwm(int pin) {
  int raw = analogRead(pin);
  raw = constrain(raw, 0, 4095);

  if (abs(raw - JOY_CENTER) <= JOY_DEADZONE) {
    return 1500;
  }

  return constrain(map(raw, 0, 4095, 1000, 2000), 1000, 2000);
}

void serviceRadio() {
  unsigned long now = millis();
  if (!radioReady || now - lastRadioTxAt < RADIO_TX_MS) {
    return;
  }

  lastRadioTxAt = now;
  txPayload.packetId++;

  bool sent = radio.write(&txPayload, sizeof(txPayload));
  if (sent && radio.isAckPayloadAvailable()) {
    uint8_t payloadSize = radio.getDynamicPayloadSize();
    if (payloadSize == sizeof(rxPayload)) {
      radio.read(&rxPayload, sizeof(rxPayload));
      telemetryValid = true;
      lastAckAt = now;
    } else {
      radio.flush_rx();
    }
  }
}

void updateLinkState() {
  unsigned long now = millis();

  if (telemetryValid && linkState == STATE_CONNECTING) {
    linkState = STATE_CONNECTED;
    connectedBannerUntil = now + CONNECTED_BANNER_MS;
    Serial.println("Link connected");
  }

  if (telemetryValid && linkState == STATE_DISCONNECTED && now - lastAckAt <= LINK_TIMEOUT_MS) {
    linkState = STATE_CONNECTED;
    connectedBannerUntil = now + CONNECTED_BANNER_MS;
    Serial.println("Link restored");
  }

  if (linkState == STATE_CONNECTED && now - lastAckAt > LINK_TIMEOUT_MS) {
    linkState = STATE_DISCONNECTED;
    Serial.println("Link lost failsafe");
  }
}

void handleKeypad() {
  char key = keypad.getKey();
  if (!key) {
    return;
  }

  int index = keyToIndex(key);
  if (index < 1 || index > 16) {
    return;
  }

  setPopupText(index);
  activeKeyId = index;
  popupStartedAt = millis();
  popupActive = true;
}

void handlePopupTimer() {
  if (popupActive && millis() - popupStartedAt >= POPUP_MS) {
    popupActive = false;
  }
}

int keyToIndex(char key) {
  switch (key) {
    case '1': return 1;
    case '2': return 2;
    case '3': return 3;
    case 'A': return 4;
    case '4': return 5;
    case '5': return 6;
    case '6': return 7;
    case 'B': return 8;
    case '7': return 9;
    case '8': return 10;
    case '9': return 11;
    case 'C': return 12;
    case '*': return 13;
    case '0': return 14;
    case '#': return 15;
    case 'D': return 16;
    default: return 0;
  }
}

void setPopupText(int index) {
  const char *text = "";
  switch (index) {
    case 1: text = "TURTLE MOD"; break;
    case 2: text = "NORMAL MOD"; break;
    case 3: text = "SPORT MOD"; break;
    case 4: text = "ECO MOD"; break;
    case 5: text = "ISIKLAR KAPALI"; break;
    case 6: text = "ISIKLAR ACIK"; break;
    case 7: text = "FLASOR MOD"; break;
    case 8: text = "KAMERA YUKARI"; break;
    case 9: text = "HOMEPOINT OK"; break;
    case 10: text = "KALIBRASYON"; break;
    case 11: text = "UCAGI BUL"; break;
    case 12: text = "KAMERA ASAGI"; break;
    case 13: text = "RAKIM SIFIRLA"; break;
    case 14: text = "EMNIRET AKTIF"; break;
    case 15: text = "DUZ TUT AKTIF"; break;
    case 16: text = "UYKU MODU"; break;
  }

  strncpy(popupText, text, sizeof(popupText) - 1);
  popupText[sizeof(popupText) - 1] = '\0';
}

void updateTelemetry() {
  data.satellites = rxPayload.satellites;
  data.aircraftBattery = constrain(map(rxPayload.aircraftBatteryMv, 3300, 4200, 0, 100), 0, 100);
  data.controllerBattery = 100;
  data.signal = constrain(rxPayload.signalPercent, 0, 100);
  data.throttle = constrain(map(txPayload.throttlePwm, 1000, 2000, 0, 100), 0, 100);
  data.altitude = rxPayload.altitudeM;
  data.pressure = rxPayload.pressureHpa;
  data.speed = rxPayload.speedKmh;
  data.distance = rxPayload.distanceM;
  data.cameraAngle = rxPayload.cameraAngleDeg;
  data.heading = rxPayload.headingDeg;
  data.latitude = (float)rxPayload.latitudeE7 / 10000000.0f;
  data.longitude = (float)rxPayload.longitudeE7 / 10000000.0f;
  data.roll = (float)rxPayload.rollDeg10 / 10.0f;
  data.pitch = (float)rxPayload.pitchDeg10 / 10.0f;
}

void drawFrame() {
  spr.fillSprite(TFT_BLACK);

  drawTopBar();
  drawUpperMiddle();
  drawSpeedRow();
  drawBottomBar();

  if (linkState == STATE_CONNECTED && millis() < connectedBannerUntil) {
    drawStatusOverlay("Baglanti Basarili");
  }

  if (linkState == STATE_DISCONNECTED) {
    drawStatusOverlay("BAGLANTI KOPTU - FAILSAFE");
  }

  if (popupActive) {
    drawPopup();
  }

  spr.pushSprite(0, 0);
}

void drawStatusScreen(const String &message) {
  int x = 45;
  int y = 80;
  int w = 230;
  int h = 80;

  spr.fillSprite(TFT_BLACK);
  spr.drawRect(x, y, w, h, POWDER_BLUE);
  spr.drawRect(x + 3, y + 3, w - 6, h - 6, POWDER_BLUE);
  drawCenteredText(message, SCREEN_W / 2, SCREEN_H / 2, 2, POWDER_BLUE);
}

void drawStatusOverlay(const String &message) {
  int x = 25;
  int y = 70;
  int w = 270;
  int h = 100;

  spr.fillRect(x, y, w, h, TFT_BLACK);
  spr.drawRect(x, y, w, h, POWDER_BLUE);
  spr.drawRect(x + 3, y + 3, w - 6, h - 6, POWDER_BLUE);
  drawCenteredText(message, SCREEN_W / 2, SCREEN_H / 2, 2, TFT_WHITE);
}

void drawTopBar() {
  drawBox(0, 0, 40, 30);
  drawBox(40, 0, 60, 30);
  drawBox(100, 0, 60, 30);
  drawBox(160, 0, 60, 30);
  drawBox(220, 0, 100, 30);

  drawCenteredText("SAT", 20, 8, 1, POWDER_BLUE);
  drawCenteredNumber(data.satellites, 20, 21, 1, TFT_WHITE);

  drawCenteredText("U PIL", 70, 8, 1, POWDER_BLUE);
  drawPercent(data.aircraftBattery, 70, 21);

  drawCenteredText("K PIL", 130, 8, 1, POWDER_BLUE);
  drawPercent(data.controllerBattery, 130, 21);

  drawCenteredText("MOD", 190, 8, 1, POWDER_BLUE);
  drawCenteredText(modeName(), 190, 21, 1, TFT_WHITE);

  drawCenteredText("SIGNAL", 270, 8, 1, POWDER_BLUE);
  if (linkState == STATE_CONNECTED) {
    drawSignalBars(236, 16, data.signal);
    drawRightText(String(data.signal) + "%", 315, 21, 1, TFT_WHITE);
  } else {
    drawCenteredText("SINYAL YOK", 270, 21, 1, TFT_WHITE);
  }
}

void drawUpperMiddle() {
  drawBox(0, 30, 40, 90);
  drawBox(40, 30, 70, 90);
  drawBox(110, 30, 60, 90);
  drawBox(170, 30, 110, 90);
  drawBox(280, 30, 40, 90);

  drawThrottleBar(0, 30, 40, 90, data.throttle);
  drawDirectionIcon(40, 30, 70, 90);
  drawSafetyHint(110, 30, 60, 90);
  drawLevelGauge(170, 30, 110, 90);
  drawCameraGauge(280, 30, 40, 90, data.cameraAngle);
}

void drawSpeedRow() {
  drawBox(0, 120, 100, 50);
  drawBox(100, 120, 120, 50);
  drawBox(220, 120, 100, 50);

  drawCenteredText("BASINC", 50, 130, 1, POWDER_BLUE);
  drawCenteredText(String(data.pressure) + " HPA", 50, 145, 1, TFT_WHITE);
  drawCenteredText(String(data.altitude) + " M", 50, 160, 1, TFT_WHITE);

  drawCenteredText("KMH", 160, 130, 1, POWDER_BLUE);
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  spr.drawString(String(data.speed), 160, 151, 4);

  drawCenteredText("TOPLAM YOL", 270, 133, 1, POWDER_BLUE);
  drawCenteredText(String(data.distance) + " M", 270, 154, 2, TFT_WHITE);
}

void drawBottomBar() {
  drawBox(0, 170, 60, 70);
  drawBox(60, 170, 200, 35);
  drawBox(60, 205, 200, 35);
  drawBox(260, 170, 60, 70);

  drawJoystick(0, 170, 60, 70, true);
  drawCoordinates(60, 170, 200, 35);
  drawCompassStrip(60, 205, 200, 35);
  drawJoystick(260, 170, 60, 70, false);
}

void drawBox(int x, int y, int w, int h) {
  spr.drawRect(x, y, w, h, POWDER_BLUE);
}

void drawCenteredText(const String &text, int x, int y, int font, uint16_t color) {
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(color, TFT_BLACK);
  spr.drawString(text, x, y, font);
}

void drawRightText(const String &text, int x, int y, int font, uint16_t color) {
  spr.setTextDatum(MR_DATUM);
  spr.setTextColor(color, TFT_BLACK);
  spr.drawString(text, x, y, font);
}

void drawCenteredNumber(int value, int x, int y, int font, uint16_t color) {
  drawCenteredText(String(value), x, y, font, color);
}

void drawPercent(int value, int x, int y) {
  drawCenteredText(String(value) + "%", x, y, 1, TFT_WHITE);
}

const char *modeName() {
  if (data.speed < 25) {
    return "ECO";
  }
  if (data.speed < 80) {
    return "NORM";
  }
  return "SPORT";
}

void drawSignalBars(int x, int y, int value) {
  int activeBars = map(value, 0, 100, 0, 5);
  for (int i = 0; i < 5; i++) {
    int barH = 4 + i * 3;
    int barX = x + i * 7;
    int barY = y + 12 - barH;
    uint16_t color = (i < activeBars) ? POWDER_BLUE : TFT_DARKGREY;
    spr.fillRect(barX, barY, 5, barH, color);
  }
}

void drawThrottleBar(int x, int y, int w, int h, int value) {
  drawCenteredText("THR", x + w / 2, y + 9, 1, POWDER_BLUE);
  int barX = x + 15;
  int barY = y + 20;
  int barW = 10;
  int barH = h - 30;
  spr.drawRect(barX, barY, barW, barH, POWDER_BLUE);
  int fillH = map(value, 0, 100, 0, barH - 2);
  spr.fillRect(barX + 2, barY + barH - 1 - fillH, barW - 4, fillH, POWDER_BLUE);
  drawCenteredText(String(value), x + w / 2, y + h - 8, 1, TFT_WHITE);
}

void drawDirectionIcon(int x, int y, int w, int h) {
  drawCenteredText("YON", x + w / 2, y + 10, 1, POWDER_BLUE);
  int cx = x + w / 2;
  int cy = y + h / 2 + 4;
  spr.drawCircle(cx, cy, 25, POWDER_BLUE);
  spr.fillTriangle(cx, cy - 26, cx - 8, cy - 5, cx + 8, cy - 5, POWDER_BLUE);
  spr.drawLine(cx, cy - 18, cx, cy + 18, POWDER_BLUE);
  spr.drawLine(cx - 18, cy, cx + 18, cy, POWDER_BLUE);
}

void drawSafetyHint(int x, int y, int w, int h) {
  drawCenteredText("EMN", x + w / 2, y + 10, 1, POWDER_BLUE);
  spr.drawRoundRect(x + 14, y + 25, 32, 32, 4, POWDER_BLUE);
  spr.drawLine(x + 30, y + 31, x + 30, y + 48, POWDER_BLUE);
  spr.drawLine(x + 22, y + 39, x + 38, y + 39, POWDER_BLUE);
  drawCenteredText(data.signal > 60 ? "OK" : "WAIT", x + w / 2, y + 70, 1, TFT_WHITE);
}

void drawLevelGauge(int x, int y, int w, int h) {
  drawCenteredText("SU TERAZISI", x + w / 2, y + 10, 1, POWDER_BLUE);
  int cx = x + w / 2;
  int cy = y + h / 2 + 5;
  spr.drawCircle(cx, cy, 32, POWDER_BLUE);
  spr.drawLine(cx - 38, cy, cx + 38, cy, TFT_DARKGREY);
  spr.drawLine(cx, cy - 28, cx, cy + 28, TFT_DARKGREY);

  int bx = cx + constrain((int)data.roll, -28, 28);
  int by = cy + constrain((int)data.pitch, -24, 24);
  spr.fillCircle(bx, by, 5, POWDER_BLUE);
  drawCenteredText("R " + String(data.roll, 1), x + 35, y + 78, 1, TFT_WHITE);
  drawCenteredText("P " + String(data.pitch, 1), x + 78, y + 78, 1, TFT_WHITE);
}

void drawCameraGauge(int x, int y, int w, int h, int angle) {
  drawCenteredText("CAM", x + w / 2, y + 9, 1, POWDER_BLUE);
  int cx = x + w / 2;
  int top = y + 22;
  int bottom = y + h - 15;
  spr.drawLine(cx, top, cx, bottom, POWDER_BLUE);
  spr.drawLine(cx - 8, top, cx + 8, top, POWDER_BLUE);
  spr.drawLine(cx - 8, bottom, cx + 8, bottom, POWDER_BLUE);
  int markerY = map(angle, -45, 45, bottom, top);
  spr.fillTriangle(cx - 10, markerY, cx, markerY - 5, cx, markerY + 5, POWDER_BLUE);
  spr.fillTriangle(cx + 10, markerY, cx, markerY - 5, cx, markerY + 5, POWDER_BLUE);
  drawCenteredText(String(angle), cx, y + h - 6, 1, TFT_WHITE);
}

void drawJoystick(int x, int y, int w, int h, bool leftSide) {
  int cx = x + w / 2;
  int cy = y + h / 2;
  drawCenteredText(leftSide ? "L JOY" : "R JOY", cx, y + 9, 1, POWDER_BLUE);
  spr.drawCircle(cx, cy + 5, 23, POWDER_BLUE);
  spr.drawCircle(cx, cy + 5, 6, POWDER_BLUE);
  spr.drawLine(cx - 23, cy + 5, cx + 23, cy + 5, TFT_DARKGREY);
  spr.drawLine(cx, cy - 18, cx, cy + 28, TFT_DARKGREY);
}

void drawCoordinates(int x, int y, int w, int h) {
  drawCenteredText("LAT " + String(data.latitude, 5), x + w / 4, y + 11, 1, POWDER_BLUE);
  drawCenteredText("LON " + String(data.longitude, 5), x + (w * 3) / 4, y + 11, 1, POWDER_BLUE);
  drawCenteredText("GPS LOCK", x + w / 2, y + 26, 1, TFT_WHITE);
}

void drawCompassStrip(int x, int y, int w, int h) {
  drawCenteredText("W", x + 25, y + 11, 1, TFT_WHITE);
  drawCenteredText("N", x + 75, y + 11, 1, POWDER_BLUE);
  drawCenteredText("E", x + 125, y + 11, 1, TFT_WHITE);
  drawCenteredText("S", x + 175, y + 11, 1, TFT_WHITE);
  spr.drawLine(x + 10, y + 22, x + w - 10, y + 22, POWDER_BLUE);
  for (int i = 0; i < 9; i++) {
    int tx = x + 20 + i * 20;
    int th = (i % 2 == 0) ? 8 : 5;
    spr.drawLine(tx, y + 18, tx, y + 18 + th, POWDER_BLUE);
  }
  spr.fillTriangle(x + w / 2, y + 25, x + w / 2 - 6, y + 34, x + w / 2 + 6, y + 34, POWDER_BLUE);
}

void drawPopup() {
  int x = 80;
  int y = 60;
  int w = 160;
  int h = 120;

  spr.fillRect(x, y, w, h, TFT_BLACK);
  spr.drawRect(x, y, w, h, POWDER_BLUE);
  spr.drawRect(x + 3, y + 3, w - 6, h - 6, POWDER_BLUE);
  spr.drawLine(x + 20, y + 35, x + w - 20, y + 35, POWDER_BLUE);

  drawCenteredText("KEYPAD", x + w / 2, y + 20, 2, POWDER_BLUE);
  drawCenteredText(popupText, x + w / 2, y + 68, 2, TFT_WHITE);
  drawCenteredText("ACTIVE", x + w / 2, y + 100, 1, POWDER_BLUE);
}
