#include <Arduino.h>
#include <U8x8lib.h>
#include "voltage.h"

//For TTGO LoRa32 V2 use:
#define MY_OLED_SDA (21)
#define MY_OLED_SCL (22)
#define MY_OLED_RST (16)

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(MY_OLED_RST, MY_OLED_SCL, MY_OLED_SDA);

void initDisplay()
{
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.clear();
    u8x8.draw2x2String(0, 0, "Water");
    u8x8.draw2x2String(0, 3, "Level");
    u8x8.draw2x2String(0, 6, "Meter");
    delay(1000);
    u8x8.clear();
    // vTaskDelete(NULL);
}

void printValues()
{
  char buff[16];
  u8x8.inverse();
  snprintf(buff, sizeof(buff), "U:%.2fV", voltage);
  u8x8.draw2x2String(0, 0, buff);

  u8x8.noInverse();
  // u8x8.setCursor(0, 3);
  // u8x8.printf("ADC:%-4d", adc_reading);

  snprintf(buff, sizeof(buff), "ADC:%-4d", adc_reading);
  u8x8.draw2x2String(0, 3, buff);

  snprintf(buff, sizeof(buff), "%.1fcm", height);
  u8x8.draw2x2String(0, 5, buff);

  Serial.printf("U=%.2fV\t\tADC=%-4d\t\tHeight=:%.1fcm\n", voltage, adc_reading, height);
}
