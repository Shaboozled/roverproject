#include <Arduino.h>
#include <U8x8lib_CH1116.h>

U8X8_CH1116_128X64_NONAME_HW_I2C display(/* reset=*/ U8X8_PIN_NONE);

void setup()
{
    display.begin();

}

void loop(void)
{
  display.setFont(u8x8_font_chroma48medium8_r);
  display.drawString(0,0,"Hello World!");
  display.drawString(0,1,"Hello World!");
  display.setCursor(0, 2);
  display.printf("Hello World! %d", 123);
  display.refreshDisplay();
  delay(2000);

}