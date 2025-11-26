#include "U8g2lib_CH1116.h"

U8G2_CH1116_128X64_NONAME_F_HW_I2C::U8G2_CH1116_128X64_NONAME_F_HW_I2C(const u8g2_cb_t *rotation, uint8_t reset, uint8_t clock, uint8_t data) : U8G2()
{
    u8g2_SetupCH1116_128X64_F(&u8g2, rotation, u8x8_byte_arduino_hw_i2c, u8x8_gpio_and_delay_arduino);
    u8x8_SetPin_HW_I2C(getU8x8(), reset, clock, data);
}
