#include <U8g2lib.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <math.h>

// ================== PINOS / HARDWARE ==================
#define SDA_PIN      5
#define SCL_PIN      6
#define PIN_AO       2      // GPIO analógico (ESP32-C3)
#define PIN_BTN_CAL  9      // Botão para calibrar (ex.: BOOT no ESP32-C3)

// ================== SENSOR / ADC ==================
const float VC_SENSOR  = 5.0f;   // meça e ajuste (ex.: 5.02f)
const float VREF_PIN   = 3.3f;
const float K_DIV      = 1.0f;   // ligado direto

const int   ADC_BITS   = 12;
const int   N_SAMPLES  = 16;

// ================== DISPLAY (OLED 72x40 na área útil) ==================
const uint8_t OLED_W  = 72;
const uint8_t OLED_H  = 40;
int16_t X_OFF = 28;
int16_t Y_OFF = 24;

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
  U8G2_R0, SCL_PIN, SDA_PIN, U8X8_PIN_NONE
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

// ================== TROCA DE PÁGINA / SNAPSHOT ==================
const uint32_t PAGE_MS = 1200;
uint32_t lastPageFlip  = 0;
int page = 0;               // 0 = V, 1 = ppb
float vsens_disp = 0.0f;    // valores congelados
float ppb_disp   = 0.0f;

// ================== CALIBRAÇÃO (própria) ==================
Preferences prefs;
float V_AIR_CLEAN = 0.16f;  // fallback
float R0_DEN      = 0.0f;   // (VC/Vair) - 1
const bool FORCE_CAL_AT_BOOT = false; // não recalibra no boot

// Ajuste empírico: ppb ≈ A * (Rs/R0)^B
const float PPB_A = 200.0f;
const float PPB_B = 1.25f;

// ================== BLE ==================
#define SERVICE_UUID_O3   "8b2fa2c8-6b2f-4e64-8f6f-8a3b8a4d10a1"
#define CHAR_UUID_TEXT    "f3e1b8a2-2b3c-4a5d-9e7f-1a2b3c4d5e6f"

NimBLEServer*         pServer  = nullptr;
NimBLEService*        pService = nullptr;
NimBLECharacteristic* pCharTxt = nullptr;
NimBLEAdvertising*    pAdv     = nullptr;

volatile bool g_connected = false;
uint32_t g_lastAdvKick = 0;

// === Callbacks BLE: mantém status e volta a anunciar ao desconectar ===
class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s) {
    g_connected = true;
  }
  void onDisconnect(NimBLEServer* s) {
    g_connected = false;
    NimBLEDevice::startAdvertising(); // tenta anunciar imediatamente
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
  // Descriptor 0x2902 (CCCD) ajuda alguns apps a habilitar "Notify"
  pCharTxt->createDescriptor("2902");

  pCharTxt->setValue("V=0.00,P=0.0");
  pService->start();

  pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID_O3);
  NimBLEDevice::startAdvertising();
  g_lastAdvKick = millis();
}

// Atualiza SEMPRE o valor (READ pega o último) e pede notify (se inscrito, recebe)
void bleUpdate(float vsens_live, float ppb_live) {
  char buf[20];
  int n = snprintf(buf, sizeof(buf), "V=%.2f,P=%.1f", vsens_live, ppb_live);
  if (n < 0) return;
  if (n >= (int)sizeof(buf)) buf[sizeof(buf)-1] = '\0';

  pCharTxt->setValue((uint8_t*)buf, strlen(buf));
  pCharTxt->notify(); // se não houver assinante, a stack ignora
}

// “Kick” periódico no advertising caso algo pare de anunciar
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
  float v_pin  = (raw / float((1 << ADC_BITS) - 1)) * VREF_PIN;
  float v_sens = v_pin * K_DIV;  // Vsens antes do divisor (se houver)
  return v_sens;
}

// ===== Calibração em ar limpo (média de janela), salva em NVS =====
void doCalibrateCleanAir(uint16_t ms_window = 5000) {
  u8g2.clearBuffer(); drawBigCenteredText("CAL"); u8g2.sendBuffer();

  const uint32_t t0 = millis();
  double acc = 0.0; uint32_t n = 0;
  while (millis() - t0 < ms_window) {
    acc += readVsens();
    n++;
    delay(10);
  }
  if (n == 0) n = 1;
  V_AIR_CLEAN = (float)(acc / n);
  R0_DEN      = (VC_SENSOR / fmaxf(V_AIR_CLEAN, 1e-4f)) - 1.0f;

  prefs.begin("o3cal", false);
  prefs.putFloat("vair", V_AIR_CLEAN);
  prefs.putFloat("r0den", R0_DEN);
  prefs.putBool("has", true);
  prefs.end();

  u8g2.clearBuffer(); drawBigCenteredText("OK"); u8g2.sendBuffer();
  delay(500);
}

// ===== Vsens -> Rs/R0 -> ppb =====
float vsens_to_ppb(float vsens) {
  const float EPS = 1e-4f;
  if (R0_DEN <= 0.0f) R0_DEN = (VC_SENSOR / fmaxf(V_AIR_CLEAN, EPS)) - 1.0f;
  float num   = (VC_SENSOR / fmaxf(vsens, EPS)) - 1.0f;
  float rsr0  = num / fmaxf(R0_DEN, EPS);
  if (rsr0 < 1.0f) rsr0 = 1.0f;
  return PPB_A * powf(rsr0, PPB_B);
}

// ===== Páginas =====
void drawPageVsens(float v_sens_snap) {
  u8g2.clearBuffer();
  drawBigCenteredText(fmt_compacto(v_sens_snap) + " V");
  u8g2.sendBuffer();
}
void drawPagePPB(float ppb_snap) {
  u8g2.clearBuffer();
  drawBigCenteredText(fmt_compacto(ppb_snap)); // sem "ppb" para caber grande
  u8g2.sendBuffer();
}

// ===== Botão (long-press) =====
const uint16_t CAL_LONG_MS = 2000;
bool     btnCalConsumed = false;
uint32_t btnCalDownAt   = 0;

void handleCalButton(uint32_t now) {
  bool pressed = (digitalRead(PIN_BTN_CAL) == LOW); // INPUT_PULLUP

  if (pressed) {
    if (btnCalDownAt == 0) {
      btnCalDownAt = now;
      btnCalConsumed = false;
    } else if (!btnCalConsumed && (now - btnCalDownAt >= CAL_LONG_MS)) {
      doCalibrateCleanAir(5000);
      float v0 = readVsens();
      vsens_disp = v0;
      ppb_disp   = vsens_to_ppb(v0);
      btnCalConsumed = true;
    }
  } else {
    btnCalDownAt = 0;
    btnCalConsumed = false;
  }
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

  // Carrega calibração se existir
  prefs.begin("o3cal", true);
  bool has = prefs.getBool("has", false);
  if (has) {
    V_AIR_CLEAN = prefs.getFloat("vair", V_AIR_CLEAN);
    R0_DEN      = prefs.getFloat("r0den", 0.0f);
  }
  prefs.end();

  if (FORCE_CAL_AT_BOOT || !has) {
    doCalibrateCleanAir(5000);
  } else if (R0_DEN <= 0.0f) {
    R0_DEN = (VC_SENSOR / fmaxf(V_AIR_CLEAN, 1e-4f)) - 1.0f;
  }

  float v0 = readVsens();
  vsens_disp = v0;
  ppb_disp   = vsens_to_ppb(v0);
}

void loop() {
  uint32_t now = millis();

  // Leituras "ao vivo"
  float v_sens = readVsens();
  float ppb    = vsens_to_ppb(v_sens);

  // Botão de calibração (long-press)
  handleCalButton(now);

  // Troca de página + snapshot na troca
  if (now - lastPageFlip >= PAGE_MS) {
    lastPageFlip = now;
    page ^= 1;
    if (page == 0) vsens_disp = v_sens;
    else           ppb_disp   = ppb;
  }

  // Render congelado
  if (page == 0) drawPageVsens(vsens_disp);
  else           drawPagePPB(ppb_disp);

  // BLE: atualiza valor e notifica a cada 1 s
  static uint32_t lastBle = 0;
  if (now - lastBle >= 1000) {
    lastBle = now;
    bleUpdate(v_sens, ppb);
  }

  // Mantém advertising vivo caso desconecte e a stack pare de anunciar
  bleTick(now);

  delay(60);
}
