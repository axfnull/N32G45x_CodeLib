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
 * @file ili9341_lcd.h
 * @author Nations
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
#ifndef __ILI9341_LCD_H
#define __ILI9341_LCD_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include "main.h"
#include "fonts.h"

#define ILI9341_CS_CLK  RCC_APB2_PERIPH_GPIOD
#define ILI9341_CS_PORT GPIOD
#define ILI9341_CS_PIN  GPIO_PIN_7

#define ILI9341_DC_CLK  RCC_APB2_PERIPH_GPIOG
#define ILI9341_DC_PORT GPIOG
#define ILI9341_DC_PIN  GPIO_PIN_0

#define ILI9341_WR_CLK  RCC_APB2_PERIPH_GPIOD
#define ILI9341_WR_PORT GPIOD
#define ILI9341_WR_PIN  GPIO_PIN_5

#define ILI9341_RD_CLK  RCC_APB2_PERIPH_GPIOD
#define ILI9341_RD_PORT GPIOD
#define ILI9341_RD_PIN  GPIO_PIN_4

#if     ((BOARD_TYPE == BOARD_V1_0)||(BOARD_TYPE == BOARD_V1_1))
#define ILI9341_BK_CLK  RCC_APB2_PERIPH_GPIOD
#define ILI9341_BK_PORT GPIOD
#define ILI9341_BK_PIN  GPIO_PIN_6

#elif   (BOARD_TYPE == BOARD_V1_2)
#define ILI9341_BK_CLK  RCC_APB2_PERIPH_GPIOB
#define ILI9341_BK_PORT GPIOB
#define ILI9341_BK_PIN  GPIO_PIN_3

#else
#error  "Invalid BOARD_TYPE"
#endif

#define ILI9341_D0_CLK  RCC_APB2_PERIPH_GPIOD
#define ILI9341_D0_PORT GPIOD
#define ILI9341_D0_PIN  GPIO_PIN_14

#define ILI9341_D1_CLK  RCC_APB2_PERIPH_GPIOD
#define ILI9341_D1_PORT GPIOD
#define ILI9341_D1_PIN  GPIO_PIN_15

#define ILI9341_D2_CLK  RCC_APB2_PERIPH_GPIOD
#define ILI9341_D2_PORT GPIOD
#define ILI9341_D2_PIN  GPIO_PIN_0

#define ILI9341_D3_CLK  RCC_APB2_PERIPH_GPIOD
#define ILI9341_D3_PORT GPIOD
#define ILI9341_D3_PIN  GPIO_PIN_1

#define ILI9341_D4_CLK  RCC_APB2_PERIPH_GPIOE
#define ILI9341_D4_PORT GPIOE
#define ILI9341_D4_PIN  GPIO_PIN_7

#define ILI9341_D5_CLK  RCC_APB2_PERIPH_GPIOE
#define ILI9341_D5_PORT GPIOE
#define ILI9341_D5_PIN  GPIO_PIN_8

#define ILI9341_D6_CLK  RCC_APB2_PERIPH_GPIOE
#define ILI9341_D6_PORT GPIOE
#define ILI9341_D6_PIN  GPIO_PIN_9

#define ILI9341_D7_CLK  RCC_APB2_PERIPH_GPIOE
#define ILI9341_D7_PORT GPIOE
#define ILI9341_D7_PIN  GPIO_PIN_10

#define ILI9341_D8_CLK  RCC_APB2_PERIPH_GPIOE
#define ILI9341_D8_PORT GPIOE
#define ILI9341_D8_PIN  GPIO_PIN_11

#define ILI9341_D9_CLK  RCC_APB2_PERIPH_GPIOE
#define ILI9341_D9_PORT GPIOE
#define ILI9341_D9_PIN  GPIO_PIN_12

#define ILI9341_D10_CLK  RCC_APB2_PERIPH_GPIOE
#define ILI9341_D10_PORT GPIOE
#define ILI9341_D10_PIN  GPIO_PIN_13

#define ILI9341_D11_CLK  RCC_APB2_PERIPH_GPIOE
#define ILI9341_D11_PORT GPIOE
#define ILI9341_D11_PIN  GPIO_PIN_14

#define ILI9341_D12_CLK  RCC_APB2_PERIPH_GPIOE
#define ILI9341_D12_PORT GPIOE
#define ILI9341_D12_PIN  GPIO_PIN_15

#define ILI9341_D13_CLK  RCC_APB2_PERIPH_GPIOD
#define ILI9341_D13_PORT GPIOD
#define ILI9341_D13_PIN  GPIO_PIN_8

#define ILI9341_D14_CLK  RCC_APB2_PERIPH_GPIOD
#define ILI9341_D14_PORT GPIOD
#define ILI9341_D14_PIN  GPIO_PIN_9

#define ILI9341_D15_CLK  RCC_APB2_PERIPH_GPIOD
#define ILI9341_D15_PORT GPIOD
#define ILI9341_D15_PIN  GPIO_PIN_10

#if     ((BOARD_TYPE == BOARD_V1_1)||(BOARD_TYPE == BOARD_V1_2))
#define PSRAM_CE_CLK    RCC_APB2_PERIPH_GPIOG
#define PSRAM_CE_PORT   GPIOG
#define PSRAM_CE_PIN    GPIO_PIN_9
#endif

#define DEBUG_DELAY()

#define CMD_SetCoordinateX 0x2A
#define CMD_SetCoordinateY 0x2B
#define CMD_SetPixel       0x2C

#define ILI9341_DispWindow_X_Star 0
#define ILI9341_DispWindow_Y_Star 0

#define ILI9341_LESS_PIXEL 240
#define ILI9341_MORE_PIXEL 320

extern uint16_t LCD_X_LENGTH, LCD_Y_LENGTH;

#define LCD_SCAN_MODE   0

#define BACKGROUND      BLACK

#define WHITE   0xFFFF
#define BLACK   0x0000
#define GREY    0xF7DE
#define BLUE    0x001F
#define BLUE2   0x051F
#define RED     0xF800
#define MAGENTA 0xF81F
#define GREEN   0x07E0
#define CYAN    0x7FFF
#define YELLOW  0xFFE0
#define BRED    0xF81F
#define GRED    0xFFE0
#define GBLUE   0x07FF

void ILI9341_Write_Cmd(uint16_t cmd);
void ILI9341_Write_Data(uint16_t data);

void ILI9341_Init(void);
void ILI9341_BackLed_Control(FunctionalState enumState);
void ILI9341_GramScan(uint8_t ucOtion);
void ILI9341_OpenWindow(uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight);
void ILI9341_Clear(uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight);
void ILI9341_SetPointPixel(uint16_t usX, uint16_t usY);
uint16_t ILI9341_GetPointPixel(uint16_t usX, uint16_t usY);
void ILI9341_DrawLine(uint16_t usX1, uint16_t usY1, uint16_t usX2, uint16_t usY2);
void ILI9341_DrawRectangle(uint16_t usX_Start,
                           uint16_t usY_Start,
                           uint16_t usWidth,
                           uint16_t usHeight,
                           uint8_t ucFilled);
void ILI9341_DrawCircle(uint16_t usX_Center, uint16_t usY_Center, uint16_t usRadius, uint8_t ucFilled);
void ILI9341_DispChar_EN(uint16_t usX, uint16_t usY, const char cChar);
void ILI9341_DispStringLine_EN(uint16_t line, char* pStr);
void ILI9341_DispString_EN(uint16_t usX, uint16_t usY, char* pStr);
void ILI9341_DispString_EN_YDir(uint16_t usX, uint16_t usY, char* pStr);

void LCD_SetFont(sFONT* fonts);
sFONT* LCD_GetFont(void);
void LCD_ClearLine(uint16_t Line);
void LCD_SetBackColor(uint16_t Color);
void LCD_SetTextColor(uint16_t Color);
void LCD_SetColors(uint16_t TextColor, uint16_t BackColor);
void LCD_GetColors(uint16_t* TextColor, uint16_t* BackColor);

#ifdef __cplusplus
}
#endif

#endif /* __ILI9341_ILI9341_H */

