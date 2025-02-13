#ifndef lcd_h
#define lcd_h

#include <Arduino.h>
#include "U8glib.h"

#define LCD_WIDTH 128
#define LCD_HEIGHT 64
#define LCD_ROWS 8 // 64 pix high, 8 bits, 64/8=8, thus 8 rows
#define FONT_WIDTH 6
#define FONT_TAB_SIZE 4

/* TODO: maybe use u8g.getheight etc. instead of above fixed numbers. But they are also used to determine array sizes... So should be a constant */

extern U8GLIB_ST7920_128X64_1X u8g;

#endif