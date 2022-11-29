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
 * @file lcd.c
 * @author Nations
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
#include "ili9341_lcd.h"
#include "lcd.h"

#define write_cmd(x)    ILI9341_Write_Cmd(x)
#define write_data(x)   ILI9341_Write_Data(x)


/**
 * @brief   Init LCD
 * @param   None
 * @retval: None
 */
void lcd_init(void)
{
    ILI9341_Init(); /* Init the LCD driver ILI9341 */
    LCD_SetFont(&Font16x24);    /*Set the font as Font16x24 */
    LCD_SetColors(COLOR_GREEN, COLOR_BLACK);    /*Set the text and background colors */

    lcd_fill(COLOR_BLACK);
    ILI9341_BackLed_Control(ENABLE);
}

/**
 * @brief   Fill all the LCD with some color
 * @param   color specifies the color to be filled
 * @retval: None
 */
void lcd_fill(color_t color)
{
    lcd_draw_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, true, color);
}

/**
 * @brief   Draw a pixel for some color
 * @param   x  specfies the position in x axis
 * @param   y  specfies the position in y axis
 * @param   color  specfies the color to be used
 * @retval: None
 */
void lcd_set_pixel(uint16_t x, uint16_t y, color_t color)
{
    lcd_draw_rect(x, y, x + 1, y + 1, true, color);
}

/**
 * @brief   Draw a line between two point by some color on LCD
 * @param   x1  specfies the position in x axis of first point
 * @param   y1  specfies the position in y axis of first point
 * @param   x2  specfies the position in x axis of second point
 * @param   y2  specfies the position in y axis of second point
 * @param   color  specfies the color to be used
 * @retval: None
 */
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, color_t color)
{
    LCD_SetTextColor(color);
    ILI9341_DrawLine(x1, y1, x2, y2);
}

/**
 * @brief   Draw a rectangle on LCD, and fill it optionally.
 * @param   x  specfies the start position in x axis
 * @param   y  specfies the start position in y axis
 * @param   width  specfies the size in x axis
 * @param   height specfies the size in y axis
 * @param   fill specifies the fill option
 * @param   color  specfies the color to be used
 * @retval: None
 */
void lcd_draw_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, bool fill, color_t color)
{
    LCD_SetTextColor(color);
    ILI9341_DrawRectangle(x, y, width, height, (uint8_t)fill);
}

/**
 * @brief   Draw a circle on LCD, and fill it optionally.
 * @param   x  specfies the center position in x axis
 * @param   y  specfies the center position in y axis
 * @param   radius  specfies the radius of circle
 * @param   fill specifies the fill option
 * @param   color  specfies the color to be used
 * @retval: None
 */
void lcd_draw_circle(uint16_t x, uint16_t y, uint16_t radius, bool fill, color_t color)
{
    LCD_SetTextColor(color);
    ILI9341_DrawCircle(x, y, radius, (uint8_t)fill);
}

/**
 * @brief   Show a grey image on LCD
 * @param   image  specfies the pointer of image data structure
 *                   8bit grey format used (8bit/pixel)
 * @retval: None
 */
static void lcd_draw_grey(const image_t* image)
{
    const uint8_t* pcolor = (const uint8_t*)(image->img_data);
    const uint8_t* pend   = pcolor + image->width * image->height;
    while (pcolor != pend)
    {
        uint8_t grey = *pcolor++;
        write_data(RGB565(grey, grey, grey));
    }
}

/**
 * @brief   Show a colorful image on LCD
 * @param   image  specfies the pointer of image data structure
 *                   RGB565 format used (16bit/pixel)
 * @retval: None
 */
static void lcd_draw_rgb565(const image_t* image)
{
    const color_t* pcolor = (const color_t*)(image->img_data);
    const color_t* pend   = pcolor + image->width * image->height;
    while (pcolor != pend)
    {
        write_data(*pcolor++);
    }
}

/**
 * @brief   Show a colorful image on LCD
 * @param   image  specfies the pointer of image data structure
 *                   RGB888 format used (32bit/pixel)
 * @retval: None
 */
static void lcd_draw_rgb24(const image_t* image)
{
    const uint32_t* pcolor = (const uint32_t*)(image->img_data);
    const uint32_t* pend   = pcolor + image->width * image->height;
    while (pcolor != pend)
    {
        uint32_t pixel = *pcolor++;
        uint16_t r     = (pixel >> 16) & 0xff;
        uint16_t g     = (pixel >> 8) & 0xff;
        uint16_t b     = pixel & 0xff;
        write_data(RGB565(r, g, b));
    }
}

/**
 * @brief   Show a image on LCD
 * @param   x  specfies the start position in x axis
 * @param   y  specfies the start position in y axis
 * @param   image  specfies the pointer of image data structure
 * @retval: None
 */
void lcd_draw_image(uint16_t x, uint16_t y, const image_t* image)
{
    ILI9341_OpenWindow(x, y, image->width, image->height);

    write_cmd(CMD_SetPixel);

    switch (image->format)
    {
    case FORMAT_GREY:
        lcd_draw_grey(image);
        break;
    case FORMAT_RGB565:
        lcd_draw_rgb565(image);
        break;
    case FORMAT_RGB24:
        lcd_draw_rgb24(image);
        break;
    default:
        break;
    }
}

/**
 * @brief   Show a image on LCD with some pre_transform function
 * @param   x  specfies the start position in x axis
 * @param   y  specfies the start position in y axis
 * @param   width  specfies the size in x axis
 * @param   height specfies the size in y axis
 * @param   param  specfies the pointer of param used in pre_transform function
 * @param   func   specfies the pre_transform function
 * @retval: None
 */
void lcd_draw_image_ex(uint16_t x, uint16_t y, uint16_t width, uint16_t height, void* param, lcd_feed_pixel func)
{
    color_t line_buf[LCD_WIDTH];
    ILI9341_OpenWindow(x, y, width, height);
    write_cmd(CMD_SetPixel);

    for (uint16_t i = 0; i < height; ++i)
    {
        func(param, line_buf);
        for (uint16_t j = 0; j < width; ++j)
        {
            write_data(line_buf[j]);
        }
    }
}

/**
 * @brief   Show a grey image on LCD,and rotate it(exchange x and y axis)
 * @param   x  specfies the start position in x axis for LCD
 * @param   y  specfies the start position in y axis for LCD
 * @param   width  specfies the size in x axis for LCD
 * @param   height specfies the size in y axis for LCD
 * @param   pImage  specfies the pointer of image data(8bit/pixel)
 * @retval: None
 */
void lcd_draw_image_rotate(uint16_t x, uint16_t y, uint16_t width, uint16_t height,uint8_t *pImage)
{
    uint32_t i,j;
    uint8_t  tgray,*pData;
    
    ILI9341_OpenWindow(x, y, width, height);
    write_cmd(CMD_SetPixel);

    for(i=0;i<height;i++)
    {
        pData = pImage+i;
        for(j=0;j<width;j++)
        {
            tgray = *pData;
            write_data(RGB565(tgray,tgray,tgray));
            pData += height;
        }
    }
}

/**
 * @brief   Show a colorful image on LCD,and rotate it(exchange x and y axis)
 * @param   x  specfies the start position in x axis for LCD
 * @param   y  specfies the start position in y axis for LCD
 * @param   width  specfies the size in x axis for LCD
 * @param   height specfies the size in y axis for LCD
 * @param   pImage  specfies the pointer of image data(16bit/pixel,MSB first)
 * @retval: None
 */
void lcd_draw_rgbimage_rotate(uint16_t x, uint16_t y, uint16_t width, uint16_t height,uint16_t *pImage)
{
    uint32_t i,j;
    uint16_t  *pData,tdata;
    
    ILI9341_OpenWindow(x, y, width, height);
    write_cmd(CMD_SetPixel);

    for(i=0;i<height;i++)
    {
        pData = pImage+i;
        for(j=0;j<width;j++)
        {
            tdata = ((*pData &0x00FF)<<8) | ((*pData >>8)&0x00FF);
            write_data(tdata);
            pData += height;
        }
    }
}

/**
 * @brief   Show a ascii string on LCD by some color
 * @param   x  specfies the start position in x axis for LCD
 * @param   y  specfies the start position in y axis for LCD
 * @param   str  specfies the pointer of string to be display
 * @param   font_color specfies the text color of string
 * @param   back_color specfies the background color of string
 * @retval: None
 */
void lcd_draw_string(uint16_t x, uint16_t y, const char* str, color_t font_color, color_t back_color)
{
    LCD_SetColors(font_color, back_color);
    ILI9341_DispString_EN(x, y, (char*)str);
}

/**
 * @brief   Config the fonts used by LCD display
 * @param   font specfies the new font
 * @retval: None
 */
void lcd_set_font(uint8_t font)
{
    switch (font)
    {
    case LCD_FONT_8X16:
        LCD_SetFont(&Font8x16);
        break;
    case LCD_FONT_16X24:
        LCD_SetFont(&Font16x24);
        break;
    case LCD_FONT_24X32:
        LCD_SetFont(&Font24x32);
        break;
    default:
        break;
    }
}

/**
 * @brief   Get the current font config
 * @param   width  specfies the buf pointer used to store x size of font 
 * @param   height specfies the buf pointer used to store y size of font 
 * @retval: None
 */
void lcd_get_font_size(uint16_t* width, uint16_t* height)
{
    sFONT* f = LCD_GetFont();
    *width   = f->Width;
    *height  = f->Height;
}


/**
 * @brief   Calculate the total pixels used by a ascii string
 * @param   str  specifies the pointer of string
 * @param   width  specfies the buf pointer used to store x size of font 
 * @param   height specfies the buf pointer used to store y size of font 
 * @retval: None
 */
void lcd_calc_string_size(const char* str, uint16_t* width, uint16_t* height)
{
    // NOTE: multi-line string is not processed
    size_t len = strlen(str);
    lcd_get_font_size(width, height);
    *width *= len;
}

