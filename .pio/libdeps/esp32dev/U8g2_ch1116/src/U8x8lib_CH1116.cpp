#include "U8x8lib_CH1116.h"

U8X8_CH1116_128X64_NONAME_HW_I2C::U8X8_CH1116_128X64_NONAME_HW_I2C(uint8_t reset, uint8_t clock, uint8_t data) : U8X8()
{
    u8x8_Setup(getU8x8(), u8x8_d_SH1116_128X64_I2C_FullBuffer, u8x8_cad_ssd13xx_fast_i2c, u8x8_byte_arduino_hw_i2c, u8x8_gpio_and_delay_arduino);
    u8x8_SetPin_HW_I2C(getU8x8(), reset, clock, data);
}
