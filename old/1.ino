#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>

// Конструктор для дисплея 128x64 на базе SSD1306 и аппаратного SPI.
// Передаем пины: (вращение, CS, DC, [RESET])
// Замените пины 10, 8, 9 на те, к которым вы припаяли CS, DC и RES.
// Конструктор для SH1106 на аппаратном SPI (сдвиг исчезнет автоматически)
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=/ 9, / dc=/ 8, / reset=*/ 10);


void setup() {
  u8g2.begin();
}

void loop() {
  u8g2.clearBuffer();          // Очистка внутреннего буфера
  u8g2.setFont(u8g2_font_ncenB08_tr); // Выбор шрифта
  u8g2.drawStr(0, 24, "Hello World!"); // Пишем текст (X, Y, текст)
  u8g2.sendBuffer();           // Выводим буфер на экран
  delay(1000);
}