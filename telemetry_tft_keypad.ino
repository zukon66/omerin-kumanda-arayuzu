#include <TFT_eSPI.h>
#include <SPI.h>
#include <Keypad.h>

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

#define POWDER_BLUE tft.color565(176, 224, 230)

static const int SCREEN_W = 320;
static const int SCREEN_H = 240;
static const unsigned long POPUP_MS = 1500;

const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {13, 12, 14, 27};
byte colPins[COLS] = {26, 25, 33, 32};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

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
  randomSeed(analogRead(34));

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  spr.setColorDepth(16);
  spr.createSprite(SCREEN_W, SCREEN_H);
  spr.setTextDatum(MC_DATUM);
  spr.setSwapBytes(true);

  updateTelemetry();
  drawFrame();
}

void loop() {
  handleKeypad();
  handlePopupTimer();

  unsigned long now = millis();
  if (now - lastTelemetryAt >= 500) {
    updateTelemetry();
    lastTelemetryAt = now;
  }

  if (now - lastFrameAt >= 50) {
    drawFrame();
    lastFrameAt = now;
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
  data.satellites = random(7, 18);
  data.aircraftBattery = random(42, 101);
  data.controllerBattery = random(55, 101);
  data.signal = random(45, 101);
  data.throttle = random(0, 101);
  data.altitude = random(0, 500);
  data.pressure = random(980, 1035);
  data.speed = random(0, 146);
  data.distance = random(0, 12000);
  data.cameraAngle = random(-45, 46);
  data.latitude = 41.0000f + (float)random(-5000, 5000) / 100000.0f;
  data.longitude = 29.0000f + (float)random(-5000, 5000) / 100000.0f;
  data.roll = (float)random(-300, 301) / 10.0f;
  data.pitch = (float)random(-200, 201) / 10.0f;
}

void drawFrame() {
  spr.fillSprite(TFT_BLACK);

  drawTopBar();
  drawUpperMiddle();
  drawSpeedRow();
  drawBottomBar();

  if (popupActive) {
    drawPopup();
  }

  spr.pushSprite(0, 0);
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
  drawSignalBars(236, 16, data.signal);
  drawRightText(String(data.signal) + "%", 315, 21, 1, TFT_WHITE);
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
