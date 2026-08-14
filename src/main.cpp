/*
 * ESP32 + GC9A01 - M-style Round Gauge UI  (LIVE OBD data)
 * --------------------------------------------------------
 * Final integration: the motorsport gauge UI + B10K screen select + real OBD
 * reads through the SparkFun OBD-II UART board (ISO 15765-4 CAN, protocol 6).
 *
 * Any PID the car doesn't answer shows "--" (no fake values). EST HP is an
 * estimate: MAF-based when available, else load x peak-torque.
 *
 * Wiring:
 *   Display (7-pin): VCC->3V3 GND->GND SCL->18 SDA->23 RST->4 DC->2 CS->5
 *   B10K pot: pin1->3V3  wiper->GPIO34  pin3->GND
 *   OBD board: TX-O -> divider -> GPIO16 ; GPIO17 -> RX-I ; GND common
 *              board powered from the car via the OBD-DB9 cable.
 *
 * FLASHING: unplug GPIO17 -> RX-I while uploading, reconnect after.
 * Libraries: Adafruit GFX, Adafruit GC9A01A, ELMduino (PowerBroker2).
 */

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <ELMduino.h>

// ---------- Pins ----------
#define TFT_CS   5
#define TFT_DC   2
#define TFT_RST  4
#define POT_PIN  34
#define OBD_RX   16
#define OBD_TX   17
#define OBD_BAUD 9600
#define OBD_PROTOCOL '6'     // ISO 15765-4 CAN (E90 / Venza). '7'=29-bit, '0'=auto

// ---------- Screen model ----------
#define NUM_SCREENS 5
#define CX 120
#define CY 120

// ---------- Arc (tach band) geometry ----------
#define ARC_RI       104
#define ARC_RO       114
#define ARC_STEPS    120
#define ARC_START    150.0
#define ARC_SWEEP    240.0
#define ARC_REDSTEP  102
#define DEG2RAD      0.0174533

// ---------- HP estimate tuning ----------
#define MAF_HP_DIVISOR 0.80f
#define MAX_TORQUE_NM  270.0f

// ---------- Colours (RGB565) ----------
#define C_BG      0x0000
#define C_WHITE   0xFFFF
#define C_LABEL   0x7C11
#define C_UNIT    0x8410
#define C_FRAME   0x2104
#define C_TRACK   0x2966
#define C_REDZONE 0x5800
#define M_BLUE    0x2D3C
#define M_DBLUE   0x11D1
#define M_RED     0xE005
#define C_WARN    0xFD20
#define C_RED     0xF800
#define C_DOT_OFF 0x39C7

Adafruit_GC9A01A tft(TFT_CS, TFT_DC, TFT_RST);
ELM327 myELM327;
bool obdReady = false;

// ---------- Live values + "have we ever read this?" flags ----------
float vRPM = 0, vCoolant = 0, vIntake = 0, vVolts = 0, vMAF = 0, vThrottle = 0, vLoad = 0, vEstHp = 0;
bool  sRPM = false, sCoolant = false, sIntake = false, sVolts = false,
      sMAF = false, sThrottle = false, sLoad = false, sHp = false;

enum PidState { P_RPM, P_COOLANT, P_INTAKE, P_VOLTS, P_MAF, P_THROTTLE, P_LOAD };
PidState pidState = P_RPM;

// ---------- Screen-switch + redraw caches ----------
int currentScreen = 0, lastScreen = -1;
int lastArcStep = -1;
uint16_t lastAccent = 0;
char lastNumStr[12] = "";
uint16_t lastNumColor = 0;
char lastSecStr[16] = "";
int nbX = 0, nbY = 0, nbW = 0, nbH = 0;
bool haveNb = false;

// =====================================================================
// UI helpers
// =====================================================================
void centerText(const char* s, int y, uint8_t size, uint16_t color) {
  tft.setFont();
  tft.setTextSize(size);
  tft.setTextColor(color);
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(CX - w / 2, y);
  tft.print(s);
}

void drawBigNumber(const char* s, uint16_t color) {
  tft.setFont(&FreeSansBold18pt7b);
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  int drawX = CX - w / 2 - x1;
  int boxX  = CX - w / 2;
  int boxY  = 136 + y1;
  if (haveNb) tft.fillRect(nbX - 3, nbY - 3, nbW + 6, nbH + 6, C_BG);
  tft.setTextColor(color);
  tft.setCursor(drawX, 136);
  tft.print(s);
  tft.setFont();
  nbX = boxX; nbY = boxY; nbW = w; nbH = h; haveNb = true;
}

static inline void polar(float r, float aDeg, int &x, int &y) {
  float a = aDeg * DEG2RAD;
  x = CX + (int)lround(r * cos(a));
  y = CY + (int)lround(r * sin(a));
}

void arcWedge(int i, uint16_t color) {
  float a0 = ARC_START + ARC_SWEEP * (float)i / ARC_STEPS;
  float a1 = ARC_START + ARC_SWEEP * (float)(i + 1) / ARC_STEPS;
  int ix0, iy0, ox0, oy0, ix1, iy1, ox1, oy1;
  polar(ARC_RI, a0, ix0, iy0); polar(ARC_RO, a0, ox0, oy0);
  polar(ARC_RI, a1, ix1, iy1); polar(ARC_RO, a1, ox1, oy1);
  tft.fillTriangle(ix0, iy0, ox0, oy0, ix1, iy1, color);
  tft.fillTriangle(ox0, oy0, ox1, oy1, ix1, iy1, color);
}

uint16_t wedgeBg(int i) { return (i >= ARC_REDSTEP) ? C_REDZONE : C_TRACK; }

void drawCap(float aDeg, uint16_t color) {
  int x, y; polar((ARC_RI + ARC_RO) / 2.0, aDeg, x, y);
  tft.fillCircle(x, y, (ARC_RO - ARC_RI) / 2, color);
}

void drawTricolor() {
  for (int k = 0; k < 4; k++) {
    tft.drawLine(104 + k, 74, 116 + k, 62, M_BLUE);
    tft.drawLine(110 + k, 74, 122 + k, 62, M_DBLUE);
    tft.drawLine(116 + k, 74, 128 + k, 62, M_RED);
  }
}

void drawPageDots(int screen) {
  int y = 196;
  int startX = CX - (NUM_SCREENS * 12) / 2 + 6;
  for (int i = 0; i < NUM_SCREENS; i++)
    tft.fillCircle(startX + i * 12, y, 3, (i == screen) ? M_BLUE : C_DOT_OFF);
}

void drawStaticFrame(int screen, const char* title, const char* unit) {
  tft.fillScreen(C_BG);
  tft.drawCircle(CX, CY, 118, C_FRAME);
  for (int i = 0; i < ARC_STEPS; i++) arcWedge(i, wedgeBg(i));
  drawCap(ARC_START, C_TRACK);
  drawCap(ARC_START + ARC_SWEEP, C_REDZONE);
  centerText(title, 44, 2, C_LABEL);
  drawTricolor();
  centerText(unit, 150, 2, C_UNIT);
  drawPageDots(screen);
}

// =====================================================================
// OBD
// =====================================================================
void computeHp() {
  float est = 0;
  if (sMAF && vMAF > 1.0f)            est = vMAF / MAF_HP_DIVISOR;         // airflow-based
  else if (sLoad && sRPM) {                                              // load-based fallback
    float torque = MAX_TORQUE_NM * (vLoad / 100.0f);
    est = (torque * vRPM / 9549.0f) * 1.341f;
  }
  if (!sRPM || vRPM < 1200 || (sThrottle && vThrottle < 10)) est = 0;    // gate
  vEstHp = 0.85f * vEstHp + 0.15f * est;                                 // smooth
  sHp = (sMAF || (sLoad && sRPM));
}

// One non-blocking PID step per call
void pollOnePid() {
  float v;
  switch (pidState) {
    case P_RPM:
      v = myELM327.rpm();
      if (myELM327.nb_rx_state == ELM_SUCCESS)          { vRPM = v; sRPM = true; pidState = P_COOLANT; }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG) { pidState = P_COOLANT; }
      break;
    case P_COOLANT:
      v = myELM327.engineCoolantTemp();
      if (myELM327.nb_rx_state == ELM_SUCCESS)          { vCoolant = v; sCoolant = true; pidState = P_INTAKE; }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG) { pidState = P_INTAKE; }
      break;
    case P_INTAKE:
      v = myELM327.intakeAirTemp();
      if (myELM327.nb_rx_state == ELM_SUCCESS)          { vIntake = v; sIntake = true; pidState = P_VOLTS; }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG) { pidState = P_VOLTS; }
      break;
    case P_VOLTS:
      v = myELM327.batteryVoltage();
      if (myELM327.nb_rx_state == ELM_SUCCESS)          { vVolts = v; sVolts = true; pidState = P_MAF; }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG) { pidState = P_MAF; }
      break;
    case P_MAF:
      v = myELM327.mafRate();
      if (myELM327.nb_rx_state == ELM_SUCCESS)          { vMAF = v; sMAF = true; computeHp(); pidState = P_THROTTLE; }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG) { pidState = P_THROTTLE; }
      break;
    case P_THROTTLE:
      v = myELM327.throttle();
      if (myELM327.nb_rx_state == ELM_SUCCESS)          { vThrottle = v; sThrottle = true; pidState = P_LOAD; }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG) { pidState = P_LOAD; }
      break;
    case P_LOAD:
      v = myELM327.engineLoad();
      if (myELM327.nb_rx_state == ELM_SUCCESS)          { vLoad = v; sLoad = true; computeHp(); pidState = P_RPM; }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG) { pidState = P_RPM; }
      break;
  }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(C_BG);
  centerText("CONNECTING", 104, 2, C_LABEL);
  centerText("OBD-II", 128, 2, C_UNIT);

  Serial2.begin(OBD_BAUD, SERIAL_8N1, OBD_RX, OBD_TX);
  delay(300);
  obdReady = myELM327.begin(Serial2, false, 2000, OBD_PROTOCOL);
  lastScreen = -1;   // force first frame
}

void readModeDial() {
  int raw  = analogRead(POT_PIN);
  int zone = constrain(map(raw, 0, 4095, 0, NUM_SCREENS - 1), 0, NUM_SCREENS - 1);
  static int candidate = 0, count = 0;
  if (zone == candidate) { if (count < 5) count++; }
  else { candidate = zone; count = 0; }
  if (count >= 5 && candidate != currentScreen) currentScreen = candidate;
}

void loop() {
  readModeDial();

  if (obdReady) pollOnePid();
  else {
    static uint32_t last = 0;      // periodic reconnect if the link dropped
    if (millis() - last > 5000) { last = millis(); obdReady = myELM327.begin(Serial2, false, 2000, OBD_PROTOCOL); }
  }

  // ---- Per-screen config from live values ----
  const char* title = ""; const char* unit = "";
  char numStr[12] = ""; char secStr[16] = "";
  float value = 0, vmin = 0, vmax = 1;
  uint16_t numColor = C_WHITE, accent = M_BLUE;
  bool ok = false;

  switch (currentScreen) {
    case 0: title = "COOLANT"; unit = "C"; vmin = 40; vmax = 130; value = vCoolant; ok = sCoolant;
      if (ok) { snprintf(numStr, sizeof(numStr), "%d", (int)lround(vCoolant));
        if (vCoolant > 115) { numColor = C_RED; accent = C_RED; }
        else if (vCoolant > 105) { numColor = C_WARN; accent = C_WARN; } }
      break;
    case 1: title = "INTAKE"; unit = "C"; vmin = 0; vmax = 80; value = vIntake; ok = sIntake;
      if (ok) snprintf(numStr, sizeof(numStr), "%d", (int)lround(vIntake));
      break;
    case 2: title = "VOLTAGE"; unit = "V"; vmin = 10; vmax = 15; value = vVolts; ok = sVolts;
      if (ok) { snprintf(numStr, sizeof(numStr), "%.1f", vVolts);
        if (vVolts < 11.5) { numColor = C_RED; accent = C_RED; }
        else if (vVolts < 12.0) { numColor = C_WARN; accent = C_WARN; } }
      break;
    case 3: title = "EST HP"; unit = "HP"; vmin = 0; vmax = 400; value = vEstHp; ok = sHp;
      if (ok) snprintf(numStr, sizeof(numStr), "%d", (int)lround(vEstHp));
      if (sMAF) snprintf(secStr, sizeof(secStr), "MAF %d g/s", (int)lround(vMAF));
      else      snprintf(secStr, sizeof(secStr), "MAF --");
      break;
    case 4: title = "LOAD"; unit = "%"; vmin = 0; vmax = 100; value = vLoad; ok = sLoad;
      if (ok) snprintf(numStr, sizeof(numStr), "%d", (int)lround(vLoad));
      if (sThrottle) snprintf(secStr, sizeof(secStr), "THR %d%%", (int)lround(vThrottle));
      else           snprintf(secStr, sizeof(secStr), "THR --");
      break;
  }

  if (!ok) { strcpy(numStr, "--"); numColor = C_UNIT; accent = C_TRACK; value = vmin; }

  // ---- Static frame on screen change ----
  if (currentScreen != lastScreen) {
    drawStaticFrame(currentScreen, title, unit);
    lastScreen = currentScreen;
    lastArcStep = -1; lastAccent = accent;
    lastNumStr[0] = '\0'; lastSecStr[0] = '\0';
    lastNumColor = numColor; haveNb = false;
  }

  // ---- Band fill ----
  float frac = (value - vmin) / (vmax - vmin);
  if (frac < 0) frac = 0; if (frac > 1) frac = 1;
  int step = (int)lround(frac * ARC_STEPS);

  if (accent != lastAccent) {
    for (int i = 0; i < step; i++) arcWedge(i, accent);
    lastAccent = accent; lastArcStep = step;
  } else if (step != lastArcStep) {
    if (lastArcStep < 0)            { for (int i = 0; i < step; i++) arcWedge(i, accent); }
    else if (step > lastArcStep)    { for (int i = lastArcStep; i < step; i++) arcWedge(i, accent); }
    else                            { for (int i = step; i < lastArcStep; i++) arcWedge(i, wedgeBg(i)); }
    lastArcStep = step;
  }
  drawCap(ARC_START, (step > 0) ? accent : C_TRACK);

  // ---- Central number ----
  if (strcmp(numStr, lastNumStr) != 0 || numColor != lastNumColor) {
    drawBigNumber(numStr, numColor);
    strcpy(lastNumStr, numStr);
    lastNumColor = numColor;
  }

  // ---- Secondary line ----
  if (strcmp(secStr, lastSecStr) != 0) {
    tft.fillRect(55, 162, 130, 20, C_BG);
    if (secStr[0] != '\0') centerText(secStr, 164, 2, C_LABEL);
    strcpy(lastSecStr, secStr);
  }
}
