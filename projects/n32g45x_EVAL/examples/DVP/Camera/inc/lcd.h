/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file lcd.h
 * @author Nations
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
#ifndef __LCD_H__
#define __LCD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef uint16_t color_t;

#define RGB565(r, g, b) ((color_t)((((r)&0xF8) << 8) | (((g)&0xFC) << 3) | (((b)&0xF8) >> 3)))
#define COLOR_WHITE     RGB565(255, 255, 255)
#define COLOR_BLACK     RGB565(0, 0, 0)
#define COLOR_GREY      RGB565(128, 128, 128)
#define COLOR_BLUE      RGB565(0, 0, 255)
#define COLOR_RED       RGB565(255, 0, 0)
#define COLOR_GREEN     RGB565(0, 255, 0)
#define COLOR_CYAN      RGB565(0, 255, 255)
#define COLOR_YELLOW    RGB565(255, 255, 0)
#define COLOR_PURPLE    RGB565(255, 0, 255)

#define LCD_WIDTH       (240)
#define LCD_HEIGHT      (320)

enum
{
    LCD_FONT_8X16,
    LCD_FONT_16X24,
    LCD_FONT_24X32,
};

enum
{
    FORMAT_GREY,
    FORMAT_RGB565,
    FORMAT_RGB24,
};

enum
{
    ROTATE_0,
    ROTATE_90,
    ROTATE_180,
    ROTATE_270,
};

typedef struct
{
    uint16_t width;
    uint16_t height;
    void* img_data;
    uint8_t format;
} image_t;

void lcd_init(void);
void lcd_fill(color_t color);
void lcd_set_font(uint8_t font);
void lcd_get_font_size(uint16_t* width, uint16_t* height);
void lcd_calc_string_size(const char* str, uint16_t* width, uint16_t* height);
void lcd_set_pixel(uint16_t x, uint16_t y, color_t color);
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, color_t color);
void lcd_draw_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, bool fill, color_t color);
void lcd_draw_circle(uint16_t x, uint16_t y, uint16_t radius, bool fill, color_t color);
void lcd_draw_image(uint16_t x, uint16_t y, const image_t* image);

typedef void (*lcd_feed_pixel)(void* param, color_t* line_buf);
void lcd_draw_image_ex(uint16_t x, uint16_t y, uint16_t width, uint16_t height, void* param, lcd_feed_pixel func);
void lcd_draw_string(uint16_t x, uint16_t y, const char* str, color_t font_color, color_t back_color);
void lcd_draw_image_rotate(uint16_t x, uint16_t y, uint16_t width, uint16_t height,uint8_t *pImage);
void lcd_draw_rgbimage_rotate(uint16_t x, uint16_t y, uint16_t width, uint16_t height,uint16_t *pImage);

#ifdef __cplusplus
}
#endif

#endif // __LCD_H__
