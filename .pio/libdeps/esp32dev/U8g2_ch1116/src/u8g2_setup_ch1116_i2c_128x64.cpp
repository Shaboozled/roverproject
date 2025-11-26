#include <U8g2lib.h>
#include "u8g2_setup_ch1116_i2c_128x64.h"


void u8g2_SetupCH1116_128X64_F(u8g2_t* display, const u8g2_cb_t* rotation, u8x8_msg_cb byteCb, u8x8_msg_cb gpioDelayCb)
{
    uint8_t bufferTileHeight;
    uint8_t* buffer = u8g2_m_16_8_f(&bufferTileHeight);
    u8g2_SetupDisplay(display, u8x8_d_SH1116_128X64_I2C_FullBuffer, u8x8_cad_ssd13xx_fast_i2c, byteCb, gpioDelayCb);
    u8g2_SetupBuffer(display, buffer, bufferTileHeight, u8g2_ll_hvline_vertical_top_lsb, rotation);
}
