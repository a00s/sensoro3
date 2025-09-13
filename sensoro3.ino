#include <U8g2lib.h>

// ====== PINOS DO SEU BOARD (ESP32-C3) ======
#define SDA_PIN 5
#define SCL_PIN 6

// ====== TAMANHO FÍSICO DO OLED EMBUTIDO ======
const uint8_t OLED_W  = 72;
const uint8_t OLED_H  = 40;

// ====== OFFSET DENTRO DO BUFFER 128x64 ======
// Centro teórico: X=28, Y=12
// Ajuste estes dois até a moldura encostar exatamente nas bordas visíveis
int16_t X_OFF = 28;   // ajuste fino: 26..32
int16_t Y_OFF = 24;   // ajuste fino: 10..16

// Usa I2C por software para garantir pinos personalizados no C3
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
  U8G2_R0,
  /* scl=*/ SCL_PIN,
  /* sda=*/ SDA_PIN,
  /* reset=*/ U8X8_PIN_NONE
);

void drawCalibrationFrame() {
  // Moldura exata do display 72x40
  u8g2.drawFrame(X_OFF, Y_OFF, OLED_W, OLED_H);

  // Marcadores dos cantos (2x2 pixels) para ver corte/clipping
  u8g2.drawBox(X_OFF,               Y_OFF,                2, 2); // canto sup-esq
  u8g2.drawBox(X_OFF + OLED_W - 2,  Y_OFF,                2, 2); // sup-dir
  u8g2.drawBox(X_OFF,               Y_OFF + OLED_H - 2,   2, 2); // inf-esq
  u8g2.drawBox(X_OFF + OLED_W - 2,  Y_OFF + OLED_H - 2,   2, 2); // inf-dir

  // Linha um pixel para dentro (para conferir easily over/underflow)
  u8g2.drawFrame(X_OFF + 1, Y_OFF + 1, OLED_W - 2, OLED_H - 2);
}

void drawCentered(const char* s, int y) {
  int w = u8g2.getStrWidth(s);
  int x = X_OFF + (OLED_W - w) / 2;
  u8g2.setCursor(x, y);
  u8g2.print(s);
}

void setup() {
  delay(300);
  u8g2.begin();
  u8g2.setContrast(255);
  u8g2.setFont(u8g2_font_5x8_tr); // cabe bem em 72x40

  u8g2.clearBuffer();
  drawCalibrationFrame();                // desenha moldura de calibração
  drawCentered("epigenetica", Y_OFF+14); // texto linha 1
  drawCentered("emcasa",      Y_OFF+28); // texto linha 2
  u8g2.sendBuffer();
}

void loop() {
  // se quiser, dá pra piscar a moldura ou alternar telas aqui
}
