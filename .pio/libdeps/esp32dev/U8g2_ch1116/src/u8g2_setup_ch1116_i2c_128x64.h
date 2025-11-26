#pragma once

#include <U8g2lib.h>
#include "u8x8_d_ch1116_128x64.h"

void u8g2_SetupCH1116_128X64_F(u8g2_t* display, const u8g2_cb_t* rotation, u8x8_msg_cb byteCb, u8x8_msg_cb gpioDelayCb);
