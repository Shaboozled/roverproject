#include "u8x8_d_ch1116_128x64.h"

static const uint8_t u8x8_d_ssd1306_128x64_noname_powersave0_seq[] = {
  U8X8_START_TRANSFER(),
  U8X8_C(0x0af),
  U8X8_END_TRANSFER(),
  U8X8_END()
};

static const uint8_t u8x8_d_ssd1306_128x64_noname_powersave1_seq[] = {
  U8X8_START_TRANSFER(),
  U8X8_C(0x0ae),
  U8X8_END_TRANSFER(),
  U8X8_END()
};

static const uint8_t u8x8_d_ssd1306_128x64_noname_flip0_seq[] = {
  U8X8_START_TRANSFER(),
  U8X8_C(0x0a1),
  U8X8_C(0x0c8),
  U8X8_END_TRANSFER(),
  U8X8_END()
};

static const uint8_t u8x8_d_ssd1306_128x64_noname_flip1_seq[] = {
  U8X8_START_TRANSFER(),
  U8X8_C(0x0a0),
  U8X8_C(0x0c0),
  U8X8_END_TRANSFER(),
  U8X8_END()
};

static const uint8_t u8x8_d_ssd1312_128x64_noname_flip0_seq[] = {
  U8X8_START_TRANSFER(),
  U8X8_C(0x0a1),
  U8X8_C(0x0c0),
  U8X8_END_TRANSFER(),
  U8X8_END()
};

static const uint8_t u8x8_d_ssd1312_128x64_noname_flip1_seq[] = {
  U8X8_START_TRANSFER(),
  U8X8_C(0x0a0),
  U8X8_C(0x0c8),
  U8X8_END_TRANSFER(),
  U8X8_END()
};

static uint8_t u8x8_d_ssd1306_sh1106_generic(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  uint8_t x, c;
  uint8_t *ptr;
  switch(msg)
  {
    case U8X8_MSG_DISPLAY_SET_POWER_SAVE:
      if ( arg_int == 0 )
        u8x8_cad_SendSequence(u8x8, u8x8_d_ssd1306_128x64_noname_powersave0_seq);
      else
        u8x8_cad_SendSequence(u8x8, u8x8_d_ssd1306_128x64_noname_powersave1_seq);
      break;
    case U8X8_MSG_DISPLAY_SET_FLIP_MODE:
      if ( arg_int == 0 )
      {
        u8x8_cad_SendSequence(u8x8, u8x8_d_ssd1306_128x64_noname_flip0_seq);
        u8x8->x_offset = u8x8->display_info->default_x_offset;
      }
      else
      {
        u8x8_cad_SendSequence(u8x8, u8x8_d_ssd1306_128x64_noname_flip1_seq);
        u8x8->x_offset = u8x8->display_info->flipmode_x_offset;
      }
      break;
#ifdef U8X8_WITH_SET_CONTRAST
    case U8X8_MSG_DISPLAY_SET_CONTRAST:
      u8x8_cad_StartTransfer(u8x8);
      u8x8_cad_SendCmd(u8x8, 0x081 );
      u8x8_cad_SendArg(u8x8, arg_int );
      u8x8_cad_EndTransfer(u8x8);
      break;
#endif
    case U8X8_MSG_DISPLAY_DRAW_TILE:
      u8x8_cad_StartTransfer(u8x8);
      x = ((u8x8_tile_t *)arg_ptr)->x_pos;
      x *= 8;
      x += u8x8->x_offset;

      u8x8_cad_SendCmd(u8x8, 0x040);

      u8x8_cad_SendCmd(u8x8, 0x010 | (x>>4));
      u8x8_cad_SendArg(u8x8, 0x000 | ((x&15)));
      u8x8_cad_SendArg(u8x8, 0x0b0 | (((u8x8_tile_t *)arg_ptr)->y_pos));

      do
      {
        c = ((u8x8_tile_t *)arg_ptr)->cnt;
        ptr = ((u8x8_tile_t *)arg_ptr)->tile_ptr;
        u8x8_cad_SendData(u8x8, c*8, ptr);
        arg_int--;
      } while( arg_int > 0 );

      u8x8_cad_EndTransfer(u8x8);
      break;
    default:
      return 0;
  }
  return 1;
}

static const u8x8_display_info_t u8x8_ch1116_128x64_display_info = {
    .chip_enable_level = 0,
    .chip_disable_level = 1,
    .post_chip_enable_wait_ns = 20,
    .pre_chip_disable_wait_ns = 10,
    .reset_pulse_width_ms = 3,
    .post_reset_wait_ms = 30,
    .sda_setup_time_ns = 15,
    .sck_pulse_width_ns = 20,
    .sck_clock_hz = 4000000UL,
    .spi_mode = 0,
    .i2c_bus_clock_100kHz = 4,
    .data_setup_time_ns = 40,
    .write_pulse_width_ns = 150,
    .tile_width = 16,
    .tile_height = 8,
    .default_x_offset = 0,
    .flipmode_x_offset = 0,
    .pixel_width = 128,
    .pixel_height = 64
};

static const uint8_t u8x8_d_ch1116_init_seq[] = {
    U8X8_START_TRANSFER(),
    U8X8_C(0xAE),        // Turn display off
    U8X8_CA(0xD5, 0x80), // Set display clock divide ratio & oscillator frequency
    U8X8_CA(0xA8, 0x3F), // Set multiplex ratio
    U8X8_CA(0xD3, 0x00), // Set display offset
    U8X8_C(0x40),        // Set start line
    U8X8_C(0xA1),        // Segment remap
    U8X8_C(0xC8),        // COM output scan direction
    U8X8_CA(0xDA, 0x12), // COM pins config
    U8X8_CA(0x81, 0x7F), // Contrast
    U8X8_CA(0xD9, 0x22), // Pre-charge
    U8X8_CA(0xDB, 0x20), // VCOMH
    U8X8_C(0xA4),        // Resume display from RAM
    U8X8_C(0xA6),        // Normal display
    U8X8_C(0x23),        // Set Breathing Light
    U8X8_C(0x00),        // Disable breathing
    U8X8_C(0xAD),        // DC-DC control mode
    U8X8_C(0x8B),        // Enable DC-DC regulator
    U8X8_C(0x00),        // Lower column start address
    U8X8_C(0x10),        // Higher column start address
    U8X8_END_TRANSFER(),
    U8X8_END()
};

static const uint8_t u8x8_d_ch1116_seq_on[] = {
    U8X8_START_TRANSFER(),
    U8X8_C(0xAF),
    U8X8_END_TRANSFER(),
    U8X8_END()};


extern "C" uint8_t u8x8_d_SH1116_128X64_I2C_FullBuffer(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  if (u8x8_d_ssd1306_sh1106_generic(u8x8, msg, arg_int, arg_ptr))
    return 1;
  switch (msg)
  {
  case U8X8_MSG_DISPLAY_INIT:
    u8x8_d_helper_display_init(u8x8);
    u8x8_cad_SendSequence(u8x8, u8x8_d_ch1116_init_seq);
    {
      static uint8_t zeros[132] = {0};
      for (uint8_t page = 0; page < 8; page++)
      {
        u8x8_cad_SendCmd(u8x8, 0xB0 | page);
        u8x8_cad_SendCmd(u8x8, 0x00);
        u8x8_cad_SendCmd(u8x8, 0x10);
        u8x8_cad_SendData(u8x8, sizeof(zeros), zeros);
      }
      u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_MILLI, 100, NULL);
    }
    u8x8_cad_SendSequence(u8x8, u8x8_d_ch1116_seq_on);
    break;
  case U8X8_MSG_DISPLAY_SETUP_MEMORY:
    u8x8_d_helper_display_setup_memory(u8x8, &u8x8_ch1116_128x64_display_info);
    break;
  default:
    return 0;
  }
  return 1;
}