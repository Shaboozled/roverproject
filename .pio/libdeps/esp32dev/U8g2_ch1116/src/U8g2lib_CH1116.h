#pragma once

#include <U8g2lib.h>
#include "u8g2_setup_ch1116_i2c_128x64.h"
#include "u8x8_d_ch1116_128x64.h"

class U8G2_CH1116_128X64_NONAME_F_HW_I2C : public U8G2 {
  public:
    U8G2_CH1116_128X64_NONAME_F_HW_I2C(const u8g2_cb_t *rotation, uint8_t reset = U8X8_PIN_NONE, uint8_t clock = U8X8_PIN_NONE, uint8_t data = U8X8_PIN_NONE);
};