#pragma once

#include "clib/u8x8.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t u8x8_d_SH1116_128X64_I2C_FullBuffer(u8x8_t* display, uint8_t msg, uint8_t arg_int, void* arg_ptr);

#ifdef __cplusplus
}
#endif
