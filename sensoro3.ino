#include <U8g2lib.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <math.h>

// ================== PINOS / HARDWARE ==================
#define SDA_PIN      5
#define SCL_PIN      6
#define PIN_AO       2      // GPIO analógico (ESP32-C3)
#define PIN_BTN_CAL  9      // Botão (BOOT no ESP32-C3)

// ================== SENSOR / ADC ==================
const float VC_SENSOR  = 5.0f;
const float VREF_PIN   = 3.3f;
const int   ADC_BITS   = 12;
const int   N_SAMPLES  = 16;

// ================== DISPLAY (rotacionado 180°) ==================
const uint8_t OLED_W  = 72;
const uint8_t OLED_H  = 40;
int16_t X_OFF = 28;
int16_t Y_OFF = 0;

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
  U8G2_R2, SCL_PIN, SDA_PIN, U8X8_PIN_NONE
);

struct FontChoice { const uint8_t* font; uint8_t height; };
FontChoice FONT_CANDIDATES[] = {
  { u8g2_font_logisoso24_tf, 24 },
  { u8g2_font_logisoso20_tf, 20 },
  { u8g2_font_logisoso18_tf, 18 },
  { u8g2_font_logisoso16_tf, 16 },
  { u8g2_font_6x12_tf,       12 },
  { u8g2_font_5x8_tr,         8 }
};

// ================== PÁGINAS ==================
const uint32_t PAGE_MS = 1200;
uint32_t lastPageFlip  = 0;
int page = 0;               // 0 = V, 1 = ppb
float vsens_disp = 0.0f;
float ppb_disp   = 0.0f;

// ================== CALIBRAÇÃO ==================
Preferences prefs;
float V_AIR_CLEAN = 0.16f;
float V_REF_OZONE = 0.00f;
float CONC_AIR    = 30.0f;
float CONC_OZONE  = 1000.0f;

// Pausa display/ble live durante seleção/calibração/reset
bool calibActive = false;

// ================== BLE ==================
#define SERVICE_UUID_O3   "8b2fa2c8-6b2f-4e64-8f6f-8a3b8a4d10a1"
#define CHAR_UUID_TEXT    "f3e1b8a2-2b3c-4a5d-9e7f-1a2b3c4d5e6f" // live
#define CHAR_UUID_CAL     "9a7c1b2e-88f0-45b1-b3a9-7f0f0c3b1e99" // calib

NimBLEServer*         pServer  = nullptr;
NimBLEService*        pService = nullptr;
NimBLECharacteristic* pCharTxt = nullptr;
NimBLECharacteristic* pCharCal = nullptr;
NimBLEAdvertising*    pAdv     = nullptr;

volatile bool g_connected = false;
uint32_t g_lastAdvKick = 0;

class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s) { g_connected = true; }
  void onDisconnect(NimBLEServer* s) {
    g_connected = false;
    NimBLEDevice::startAdvertising();
  }
};

void bleInit() {
  NimBLEDevice::init("O3 sensor");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  pService = pServer->createService(SERVICE_UUID_O3);

  pCharTxt = pService->createCharacteristic(
      CHAR_UUID_TEXT,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  pCharTxt->createDescriptor("2902");
  pCharTxt->setValue("V=0.00,P=0.0");

  pCharCal = pService->createCharacteristic(
      CHAR_UUID_CAL,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  pCharCal->createDescriptor("2902");
  pCharCal->setValue("AIR=0.000,O3=0.000");

  pService->start();
  pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID_O3);
  NimBLEDevice::startAdvertising();
  g_lastAdvKick = millis();
}

void bleUpdateLive(float vsens_live, float ppb_live) {
  char buf[24];
  snprintf(buf, sizeof(buf), "V=%.2f,P=%.1f", vsens_live, ppb_live);
  pCharTxt->setValue((uint8_t*)buf, strlen(buf));
  pCharTxt->notify();
}

void bleUpdateCal() {
  char buf[40];
  snprintf(buf, sizeof(buf), "AIR=%.3f,O3=%.3f", V_AIR_CLEAN, V_REF_OZONE);
  pCharCal->setValue((uint8_t*)buf, strlen(buf));
  pCharCal->notify();
}

void bleTick(uint32_t now) {
  if (!g_connected && (now - g_lastAdvKick >= 3000)) {
    NimBLEDevice::startAdvertising();
    g_lastAdvKick = now;
  }
}

// ================== HELPERS ==================
String fmt_compacto(float v) {
  if (v < 10.0f)   return String(v, 2);
  if (v < 100.0f)  return String(v, 1);
  if (v < 1000.0f) return String((int)roundf(v));
  char buf[8]; snprintf(buf, sizeof(buf), "%.1fk", v/1000.0f);
  return String(buf);
}

void drawBigCenteredText(const String& text) {
  for (auto &fc : FONT_CANDIDATES) {
    u8g2.setFont(fc.font);
    int w = u8g2.getStrWidth(text.c_str());
    if (w <= (int)(OLED_W - 4)) {
      int x = X_OFF + (OLED_W - w)/2;
      int baseline = Y_OFF + (OLED_H + fc.height)/2 - 2;
      u8g2.setCursor(x, baseline);
      u8g2.print(text);
      return;
    }
  }
  u8g2.setFont(u8g2_font_5x8_tr);
  int w = u8g2.getStrWidth(text.c_str());
  int x = X_OFF + (OLED_W - w)/2;
  int baseline = Y_OFF + (OLED_H + 8)/2 - 2;
  u8g2.setCursor(x, baseline);
  u8g2.print(text);
}

int readAdcAveraged() {
  uint32_t acc = 0;
  for (int i = 0; i < N_SAMPLES; i++) {
    acc += analogRead(PIN_AO);
    delayMicroseconds(250);
  }
  return acc / N_SAMPLES;
}

float readVsens() {
  int raw = readAdcAveraged();
  return (raw / float((1 << ADC_BITS) - 1)) * VREF_PIN;
}

// ================== CALIBRAÇÕES ==================
void saveCal() {
  prefs.begin("o3cal", false);
  prefs.putFloat("vair", V_AIR_CLEAN);
  prefs.putFloat("voz", V_REF_OZONE);
  prefs.putBool("has", true);
  prefs.end();
  bleUpdateCal();
}

void doCalibrateCleanAir(uint16_t ms_window = 5000) {
  calibActive = true;
  u8g2.clearBuffer(); drawBigCenteredText("CAL AIR"); u8g2.sendBuffer();
  const uint32_t t0 = millis();
  double acc = 0.0; uint32_t n = 0;
  while (millis() - t0 < ms_window) { acc += readVsens(); n++; delay(10); }
  V_AIR_CLEAN = (n > 0) ? (float)(acc / n) : 0.2f;
  saveCal();
  u8g2.clearBuffer(); drawBigCenteredText("OK AIR"); u8g2.sendBuffer();
  delay(500);
  calibActive = false;
}

void doCalibrateOzone(uint16_t ms_window = 5000) {
  calibActive = true;
  u8g2.clearBuffer(); drawBigCenteredText("CAL O3"); u8g2.sendBuffer();
  const uint32_t t0 = millis();
  double acc = 0.0; uint32_t n = 0;
  while (millis() - t0 < ms_window) { acc += readVsens(); n++; delay(10); }
  V_REF_OZONE = (n > 0) ? (float)(acc / n) : 0.05f;
  saveCal();
  u8g2.clearBuffer(); drawBigCenteredText("OK O3"); u8g2.sendBuffer();
  delay(500);
  calibActive = false;
}

void doResetOzoneCal() {
  V_REF_OZONE = 0.0f;
  saveCal();
  u8g2.clearBuffer(); drawBigCenteredText("O3=0 OK"); u8g2.sendBuffer();
  delay(700);
  calibActive = false;
}

// ================== CONVERSÃO ==================
float vsens_to_ppb(float vsens) {
  const float EPS = 1e-6f;
  float v0 = fmaxf(V_AIR_CLEAN, EPS);
  float v1 = fmaxf(V_REF_OZONE, EPS);
  float v  = fmaxf(vsens, EPS);

  if (V_REF_OZONE <= 0.0f || fabsf(v1 - v0) < 1e-6f) {
    float ratio = v0 / v;
    if (ratio < 1.0f) ratio = 1.0f;
    return 200.0f * powf(ratio, 1.25f);
  }

  float logC0 = logf(CONC_AIR);
  float logC1 = logf(CONC_OZONE);
  float logV0 = logf(v0);
  float logV1 = logf(v1);
  float logV  = logf(v);

  float slope = (logC1 - logC0) / (logV1 - logV0);
  float logC  = logC0 + slope * (logV - logV0);
  return expf(logC);
}

// ================== BOTÃO (FSM) ==================
const uint16_t TRESH_O3         = 2000;   // 2s  -> O3
const uint16_t TRESH_AIR        = 5000;   // 5s  -> AIR
const uint16_t AIR_ABORT        = 8000;   // >8s -> ABORT (calibração)
const uint16_t RESET_O3_HOLD    = 12000;  // 12s -> entra em confirmação O3=0
const uint16_t RESET_CONFIRM_MS = 3000;   // soltar até 3s para confirmar

enum BtnState { IDLE, SELECT, RESET_CONFIRM };
BtnState btnState = IDLE;

bool     btnPrev      = false;
uint32_t pressStartAt = 0;
uint32_t resetStartAt = 0;

inline bool isPressed() { return digitalRead(PIN_BTN_CAL) == LOW; }

void handleCalButton(uint32_t now) {
  bool pressed = isPressed();

  // borda de descida
  if (pressed && !btnPrev) {
    pressStartAt = now;
    calibActive  = true;   // pausa telas/ble live
    btnState     = SELECT;
  }

  if (pressed && btnState == SELECT) {
    uint32_t held = now - pressStartAt;
    if (held >= TRESH_O3 && held < TRESH_AIR) {
      u8g2.clearBuffer(); drawBigCenteredText("O3");  u8g2.sendBuffer();
    } else if (held >= TRESH_AIR && held < AIR_ABORT) {
      u8g2.clearBuffer(); drawBigCenteredText("AIR"); u8g2.sendBuffer();
    } else if (held >= AIR_ABORT && held < RESET_O3_HOLD) {
      u8g2.clearBuffer(); drawBigCenteredText("ABORT"); u8g2.sendBuffer();
    } else if (held >= RESET_O3_HOLD) {
      // entra no modo de confirmação de reset
      btnState     = RESET_CONFIRM;
      resetStartAt = now;
      u8g2.clearBuffer(); drawBigCenteredText("O3=0"); u8g2.sendBuffer();
    }
  }

  if (pressed && btnState == RESET_CONFIRM) {
    // mantendo pressionado na confirmação: se passar de 3s, aborta o reset
    if ((now - resetStartAt) > RESET_CONFIRM_MS) {
      u8g2.clearBuffer(); drawBigCenteredText("ABORT"); u8g2.sendBuffer();
      // volta para SELECT mas sem executar nada; aguarda soltar
    }
  }

  // borda de subida (soltou)
  if (!pressed && btnPrev) {
    if (btnState == RESET_CONFIRM) {
      // confirma reset se soltou dentro da janela
      if ((now - resetStartAt) <= RESET_CONFIRM_MS) {
        doResetOzoneCal();
      } else {
        calibActive = false; // abortou reset
      }
    } else if (btnState == SELECT) {
      uint32_t held = now - pressStartAt;
      if (held >= TRESH_O3 && held < TRESH_AIR) {
        doCalibrateOzone(5000);
      } else if (held >= TRESH_AIR && held < AIR_ABORT) {
        doCalibrateCleanAir(5000);
      } else {
        // soltou em ABORT ou antes de 2s: não faz nada
        calibActive = false;
      }
    }
    // reset estado
    btnState = IDLE;
  }

  btnPrev = pressed;
}

// ================== SETUP / LOOP ==================
void setup() {
  delay(200);
  u8g2.begin();
  u8g2.setContrast(255);

  pinMode(PIN_BTN_CAL, INPUT_PULLUP);
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(PIN_AO, ADC_11db);

  bleInit();

  prefs.begin("o3cal", true);
  bool has = prefs.getBool("has", false);
  if (has) {
    V_AIR_CLEAN = prefs.getFloat("vair", V_AIR_CLEAN);
    V_REF_OZONE = prefs.getFloat("voz", V_REF_OZONE);
  }
  prefs.end();
  bleUpdateCal();

  float v0 = readVsens();
  vsens_disp = v0;
  ppb_disp   = vsens_to_ppb(v0);
}

void loop() {
  uint32_t now = millis();

  float v_sens = readVsens();
  float ppb    = vsens_to_ppb(v_sens);

  handleCalButton(now);

  if (!calibActive) {
    if (now - lastPageFlip >= PAGE_MS) {
      lastPageFlip = now;
      page ^= 1;
      if (page == 0) vsens_disp = v_sens;
      else           ppb_disp   = ppb;
    }

    u8g2.clearBuffer();
    if (page == 0) drawBigCenteredText(String(vsens_disp, 2) + " V");
    else           drawBigCenteredText(fmt_compacto(ppb_disp));
    u8g2.sendBuffer();

    static uint32_t lastBle = 0;
    if (now - lastBle >= 1000) {
      lastBle = now;
      bleUpdateLive(v_sens, ppb);
    }
  }

  bleTick(now);
  delay(60);
}
