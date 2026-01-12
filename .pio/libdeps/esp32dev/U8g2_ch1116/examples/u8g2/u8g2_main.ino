#include <Arduino.h>
#include <U8g2lib_CH1116.h>

U8G2_CH1116_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

void setup()
{
    display.begin();
    display.setFont(u8g2_font_8bitclassic_tr);
}

void loop()
{
    display.clearBuffer();
    display.drawFrame(0, 0, 128, 64);
    display.setCursor(10, 32);
    display.print("Hello");
    display.sendBuffer();
    delay(1000);

    display.clearBuffer();
    display.drawFrame(0, 0, 128, 64);
    display.setCursor(10, 32);
    display.print("Word");
    display.sendBuffer();
    delay(1000);
}