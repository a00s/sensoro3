#include <U8g2lib.h>
#include <NimBLEDevice.h>

// ====== PINOS DO SEU BOARD (ESP32-C3) ======
#define SDA_PIN 5
#define SCL_PIN 6

// ====== OLED EMBUTIDO (área visível 72x40) ======
const uint8_t OLED_W  = 72;
const uint8_t OLED_H  = 40;

// ====== OFFSET DENTRO DO BUFFER 128x64 ======
int16_t X_OFF = 28;   // ajuste fino
int16_t Y_OFF = 24;   // ajuste fino

// I2C por software (pinos personalizados no C3)
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
  U8G2_R0,
  /* scl=*/ SCL_PIN,
  /* sda=*/ SDA_PIN,
  /* reset=*/ U8X8_PIN_NONE
);

// ====== SENSOR O3 (analógico com divisor Opção A) ======
#define PIN_AO       2          // GPIO2 (ADC1_CH2) no ESP32-C3
const float K_DIV     = 1.532f; // (R1+R2)/R2 para Vsens (antes do divisor)
const int   ADC_BITS  = 12;     // 0..4095
const float VREF_PIN  = 3.3f;
const int   N_SAMPLES = 16;     // média para suavizar

// ====== TROCA DE PÁGINA (0 = Vsens, 1 = ppm) ======
const uint32_t PAGE_MS = 1200;
uint32_t lastPageFlip = 0;
int page = 0;

// ====== ESTIMATIVA DE PPM (heurística; ajuste depois com seu datasheet) ======
float estimate_ppm_from_vsens(float vsens) {
  const float v0 = 0.10f; // offset ~ruído/ambiente limpo
  const float a  = 0.20f; // ganho
  const float b  = 1.40f; // curvatura
  float x = vsens - v0;
  if (x < 0) x = 0;
  return a * powf(x, b);
}

// ====== UI ======
void drawFrame72x40() {
  u8g2.drawFrame(X_OFF, Y_OFF, OLED_W, OLED_H);
  u8g2.drawFrame(X_OFF + 1, Y_OFF + 1, OLED_W - 2, OLED_H - 2);
}

struct FontChoice {
  const uint8_t* font;
  uint8_t height;
};

FontChoice FONT_CANDIDATES[] = {
  { u8g2_font_logisoso24_tf, 24 },
  { u8g2_font_logisoso20_tf, 20 },
  { u8g2_font_logisoso18_tf, 18 },
  { u8g2_font_logisoso16_tf, 16 },
  { u8g2_font_6x12_tf,       12 },
  { u8g2_font_5x8_tr,         8 }
};

void drawBigCenteredNumber(const char* text) {
  for (auto &fc : FONT_CANDIDATES) {
    u8g2.setFont(fc.font);
    int w = u8g2.getStrWidth(text);
    if (w <= (int)(OLED_W - 4)) {
      int x = X_OFF + (OLED_W - w)/2;
      int baseline = Y_OFF + (OLED_H + fc.height)/2 - 2;
      u8g2.setCursor(x, baseline);
      u8g2.print(text);
      return;
    }
  }
  u8g2.setFont(u8g2_font_5x8_tr);
  int w = u8g2.getStrWidth(text);
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

void drawPageVsens(float v_sens) {
  u8g2.clearBuffer();
  drawFrame72x40();
  char txt[16];
  snprintf(txt, sizeof(txt), "%.2fV", v_sens);
  drawBigCenteredNumber(txt);
  u8g2.sendBuffer();
}

void drawPagePPM(float ppm) {
  u8g2.clearBuffer();
  drawFrame72x40();
  char txt[16];
  snprintf(txt, sizeof(txt), "%.3f", ppm);
  drawBigCenteredNumber(txt);
  // unidade pequena no canto (opcional)
  u8g2.setFont(u8g2_font_5x8_tr);
  const char* unit = "ppm";
  int uw = u8g2.getStrWidth(unit);
  u8g2.setCursor(X_OFF + OLED_W - uw - 2, Y_OFF + OLED_H - 2);
  u8g2.print(unit);
  u8g2.sendBuffer();
}

// ====== BLE (NimBLE) ======
// 1 serviço + 1 characteristic textual legível
#define SERVICE_UUID_O3   "8b2fa2c8-6b2f-4e64-8f6f-8a3b8a4d10a1"
#define CHAR_UUID_TEXT    "f3e1b8a2-2b3c-4a5d-9e7f-1a2b3c4d5e6f"

NimBLEServer*         pServer  = nullptr;
NimBLEService*        pService = nullptr;
NimBLECharacteristic* pCharTxt = nullptr;

void bleInit() {
  NimBLEDevice::init("O3 sensor");           // nome que aparece no celular
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);    // opcional: potência de TX

  pServer  = NimBLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID_O3);

  // characteristic em texto (READ + NOTIFY)
  pCharTxt = pService->createCharacteristic(
      CHAR_UUID_TEXT,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  pCharTxt->setValue("Vsens: 0.00 V\nPPM:   0.000");

  pService->start();

  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID_O3);
  NimBLEDevice::startAdvertising();
}

// envia texto formatado (duas linhas) — ótimo no nRF Connect
void bleUpdate(float vsens, float ppm) {
  char buf[40];
  // alinhado em colunas para ficar bonito no app
  snprintf(buf, sizeof(buf), "Vsens: %.2f V\nPPM:   %.3f", vsens, ppm);
  pCharTxt->setValue(buf);
  pCharTxt->notify();
}

// ====== SETUP/LOOP ======
void setup() {
  delay(200);
  u8g2.begin();
  u8g2.setContrast(255);

  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(PIN_AO, ADC_11db); // ~3.6 V faixa no pino

  bleInit();
}

void loop() {
  // Leitura
  int   raw    = readAdcAveraged();
  float v_pin  = (raw / float((1 << ADC_BITS) - 1)) * VREF_PIN; // tensão no GPIO
  float v_sens = v_pin * K_DIV;                                  // tensão original do sensor
  float ppm    = estimate_ppm_from_vsens(v_sens);                // heurística

  // Alterna Vsens <-> ppm
  uint32_t now = millis();
  if (now - lastPageFlip >= PAGE_MS) {
    lastPageFlip = now;
    page ^= 1;
  }

  if (page == 0) drawPageVsens(v_sens);
  else           drawPagePPM(ppm);

  // Atualiza BLE ~1x/s
  static uint32_t lastBle = 0;
  if (now - lastBle >= 1000) {
    lastBle = now;
    bleUpdate(v_sens, ppm);
  }

  delay(60);
}
