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
 * @file gc0308.h
 * @author Nations
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __GC0308_H__
#define __GC0308_H__

#ifdef __cplusplus
extern "C" {
#endif
    
#include "main.h"

#define GC0308_I2C_ADDR     ((uint8_t)0x42)
#define GC0308_CHIP_ID      ((uint8_t)0x9B)
#define GC0308_INVALIE_REG  ((uint8_t)0xFF)

// GC0308 Registers Definition
#define GC0308_ID                       0x00
#define GC0308_Hb                       0x01
#define GC0308_Vb                       0x02
#define GC0308_ExposureH                0x03
#define GC0308_ExposureL                0x04
#define GC0308_RowStaH                  0x05
#define GC0308_RowStaL                  0x06
#define GC0308_ColumnStaH               0x07
#define GC0308_ColumnStaL               0x08
#define GC0308_WindowHeightH            0x09
#define GC0308_WindowHeightL            0x0A
#define GC0308_WindowWidthH             0x0B
#define GC0308_WindowWidthL             0x0C
#define GC0308_vs_st                    0x0D
#define GC0308_vs_et                    0x0E
#define GC0308_VbHb11_8                 0x0F
#define GC0308_RshWidth                 0x10
#define GC0308_TspWidth                 0x11
#define GC0308_Sh_Delay                 0x12
#define GC0308_RowTailWidth             0x13
#define GC0308_CISCTL_Mode1             0x14
#define GC0308_CISCTL_mode2             0x15
#define GC0308_CISCTL_mode3             0x16
#define GC0308_CISCTL_mode4             0x17
#define GC0308_RESERVED1                0x18
#define GC0308_RESERVED2                0x19
#define GC0308_Analog_mode1             0x1A
#define GC0308_Analog_mode2             0x1B
#define GC0308_HDTD                     0x1C
#define GC0308_Vref_v25                 0x1D
#define GC0308_ADC_R                    0x1E
#define GC0308_PAD_drv                  0x1F
#define GC0308_Rest_related             0xFE
#define GC0308_BlockEn1                 0x20
#define GC0308_BlockEn2                 0x21
#define GC0308_AAAAEn                   0x22
#define GC0308_SpecialEffect            0x23
#define GC0308_OutputFormat             0x24
#define GC0308_OutputEn                 0x25
#define GC0308_Sync_mode                0x26
#define GC0308_RESERVED3                0x27
#define GC0308_ClkDiv_mode              0x28
#define GC0308_Bypass_mode              0x29
#define GC0308_ClockGatingEn            0x2A
#define GC0308_Dither_mode              0x2B
#define GC0308_DitherBit                0x2C
#define GC0308_Debug_mode1              0x2D
#define GC0308_Debug_mode2              0x2E
#define GC0308_Debug_mode3              0x2F
#define GC0308_Blk_mode                 0x30
#define GC0308_BlkLimitValue            0x31
#define GC0308_Global_offset            0x32
#define GC0308_Current_R_offset         0x33
#define GC0308_Current_G_offset         0x34
#define GC0308_Current_B_offset         0x35
#define GC0308_Current_R_dark_current   0x36
#define GC0308_Current_G_dark_current   0x37
#define GC0308_Current_B_dark_current   0x38
#define GC0308_ExpRateDarkc             0x39
#define GC0308_OffsetSubmodeRatio       0x3A
#define GC0308_DarkSubmodecurrent_ratio 0x3B
#define GC0308_Manual_G1_offset         0x3C
#define GC0308_Manual_R1_offset         0x3D
#define GC0308_Manual_B2_offset         0x3E
#define GC0308_Manual_G2_offset         0x3F
#define GC0308_CropWin_mode             0x46
#define GC0308_CropWin_y1               0x47
#define GC0308_CropWin_x1               0x48
#define GC0308_CropWinHeiH              0x49
#define GC0308_CropWinHeiL              0x4A
#define GC0308_CropWinWidH              0x4B
#define GC0308_CropWinWidL              0x4C
#define GC0308_GlobalGain               0x50
#define GC0308_AutoPregain              0x51
#define GC0308_AutoPostgain             0x52
#define GC0308_ChannelGain_G1           0x53
#define GC0308_ChannelGain_R            0x54
#define GC0308_ChannelGain_B            0x55
#define GC0308_ChannelGain_G2           0x56
#define GC0308_R_ratio                  0x57
#define GC0308_G_ratio                  0x58
#define GC0308_B_ratio                  0x59
#define GC0308_AWB_R_gain               0x5A
#define GC0308_AWB_G_gain               0x5B
#define GC0308_AWB_B_gain               0x5C
#define GC0308_LscDecLev1               0x5D
#define GC0308_LscDecLev2               0x5E
#define GC0308_LscDecLev3               0x5F
#define GC0308_DN_modeEn                0x60
#define GC0308_DN_modeRatio             0x61
#define GC0308_DN_bil_b_base            0x62
#define GC0308_DN_b_incr                0x63
#define GC0308_DN_bil_n_base            0x64
#define GC0308_DN_n_incr                0x65
#define GC0308_DD_dark_bright_TH        0x66
#define GC0308_DD_flat_TH               0x67
#define GC0308_DD_limit                 0x68
#define GC0308_ASDE_GainHighTH          0x69
#define GC0308_ASDE_DN_c_slope          0x6A
#define GC0308_ASDE_DN_b_n_slope        0x6B
#define GC0308_ASDE_DD_briTH            0x6C
#define GC0308_ASDE_DD_limit            0x6D
#define GC0308_ASDE_auto_EE1_effect     0x6E
#define GC0308_ASDE_auto_EE2_effect     0x6F
#define GC0308_ASDE_auto_saturation_dec 0x70
#define GC0308_ASDE_auto_saturation_low 0x71
#define GC0308_EEINTP_mode1             0x72
#define GC0308_EEINTP_mode2             0x73
#define GC0308_DirectionTH1             0x74
#define GC0308_DirectionTH2             0x75
#define GC0308_Diff_HV_TI_TH            0x76
#define GC0308_Edge1_2effect            0x77
#define GC0308_EdgeRatio                0X78
#define GC0308_Edge1_max_min            0x79
#define GC0308_Edge2_max_min            0x7A
#define GC0308_Edge1_2th                0x7B
#define GC0308_EdgePN_max               0x7C
#define GC0308_ABB_mode                 0x80
#define GC0308_ABBTarAve                0x81
#define GC0308_ABBRange                 0x82
#define GC0308_ABBLimVal                0x83
#define GC0308_ABBSpeed                 0x84
#define GC0308_Cur_R_blaLev             0x85
#define GC0308_Cur_G_blaLev             0x86
#define GC0308_Cur_B_blaLev             0x87
#define GC0308_Cur_R_blaFac             0x88
#define GC0308_Cur_G_blaFac             0x89
#define GC0308_Cur_B_blaFac             0x8A
#define GC0308_CC_MatrixC11             0x93
#define GC0308_CC_MatrixC12             0x94
#define GC0308_CC_MatrixC13             0x95
#define GC0308_CC_MatrixC21             0x96
#define GC0308_CC_MatrixC22             0x97
#define GC0308_CC_MatrixC23             0x98
#define GC0308_CC_MatrixC41             0x9C
#define GC0308_CC_MatrixC42             0x9D
#define GC0308_CC_MatrixC43             0x9E
#define GC0308_Gamma_out0               0x9F
#define GC0308_Gamma_out1               0xA0
#define GC0308_Gamma_out2               0xA1
#define GC0308_Gamma_out3               0xA2
#define GC0308_Gamma_out4               0xA3
#define GC0308_Gamma_out5               0xA4
#define GC0308_Gamma_out6               0xA5
#define GC0308_Gamma_out7               0xA6
#define GC0308_Gamma_out8               0xA7
#define GC0308_Gamma_out9               0xA8
#define GC0308_Gamma_out10              0xA9
#define GC0308_Gamma_out11              0xAA
#define GC0308_Gamma_out12              0xAB
#define GC0308_Gamma_out13              0xAC
#define GC0308_Gamma_out14              0xAD
#define GC0308_Gamma_out15              0xAE
#define GC0308_Gamma_out16              0xAF
#define GC0308_GlobalSat                0xB0
#define GC0308_Saturation_Cb            0xB1
#define GC0308_Saturation_Cr            0xB2
#define GC0308_LumaContr                0xB3
#define GC0308_ContrCen                 0xB4
#define GC0308_Luma_offset              0xB5
#define GC0308_Skin_Cb_cen              0xB6
#define GC0308_Skin_Cr_cen              0xB7
#define GC0308_SkinRadiSqu              0xB8
#define GC0308_SkinBri                  0xB9
#define GC0308_Fixed_Cb                 0xBA
#define GC0308_Fixed_Cr                 0xBB
#define GC0308_RESERVED4                0xBC
#define GC0308_Edge_dec_sa              0xBD
#define GC0308_AutoGray_mode            0xBE
#define GC0308_SaturSubStrength         0xBF
#define GC0308_Y_Gamma_out0             0xC0
#define GC0308_Y_Gamma_out1             0xC1
#define GC0308_Y_Gamma_out2             0xC2
#define GC0308_Y_Gamma_out3             0xC3
#define GC0308_Y_Gamma_out4             0xC4
#define GC0308_Y_Gamma_out5             0xC5
#define GC0308_Y_Gamma_out6             0xC6
#define GC0308_Y_Gamma_out7             0xC7
#define GC0308_Y_Gamma_out8             0xC8
#define GC0308_Y_Gamma_out9             0xC9
#define GC0308_Y_Gamma_out10            0xCA
#define GC0308_Y_Gamma_out11            0xCB
#define GC0308_Y_Gamma_out12            0xCC
#define GC0308_AEC_mode1                0xD0
#define GC0308_AEC_mode2                0xD1
#define GC0308_AEC_mode3                0xD2
#define GC0308_AEC_tar_Y                0xD3
#define GC0308_Y_ave                    0xD4
#define GC0308_AEC_H_LRange             0xD5
#define GC0308_AEC_ign                  0xD6
#define GC0308_AEC_NumLim_HRange        0xD7
#define GC0308_RESERVED5                0xD8
#define GC0308_AEC_offset1              0xD9
#define GC0308_AEC_offset2              0xDA
#define GC0308_AEC_slow                 0xDB
#define GC0308_AEC_fast                 0xDC
#define GC0308_AEC_exp                  0xDD
#define GC0308_AEC_step2                0xDE
#define GC0308_AEC_I_D                  0xDF
#define GC0308_AEC_I_stop_L_margin      0xE0
#define GC0308_AEC_I_stop_margin        0xE1
#define GC0308_AntiFlicSteH             0xE2
#define GC0308_AntiFlicSteL             0xE3
#define GC0308_ExpLev1H                 0xE4
#define GC0308_ExpLev1L                 0xE5
#define GC0308_ExpLev2H                 0xE6
#define GC0308_ExpLev2L                 0xE7
#define GC0308_ExpLev3H                 0xE8
#define GC0308_ExpLev3L                 0xE9
#define GC0308_ExpLev4H                 0xEA
#define GC0308_ExpLev4L                 0xEB
#define GC0308_MaxExpLev                0xEC
#define GC0308_ExpMinL                  0xED
#define GC0308_MaxPosGain               0xEE
#define GC0308_MaxPreGain               0xEF
#define GC0308_ABSRange                 0xF0
#define GC0308_ABSStopMar               0xF1
#define GC0308_Y_S_compe                0xF2
#define GC0308_Y_StreLim                0xF3
#define GC0308_Y_tilt                   0xF4
#define GC0308_Y_stretch_K              0xF5
#define GC0308_Big_win_x0               0xF7
#define GC0308_Big_win_y0               0xF8
#define GC0308_Big_win_x1               0xF9
#define GC0308_Big_win_y1               0xFA
#define GC0308_Diff_Y_big_thd           0xFB


#define GC0308_MCLK_PORT_CLK    (RCC_APB2_PERIPH_GPIOA)
#define GC0308_MCLK_PORT        (GPIOA)
#define GC0308_MCLK_PIN         (GPIO_PIN_8)

#define GC0308_PWDN_PORT_CLK    (RCC_APB2_PERIPH_GPIOB)
#define GC0308_PWDN_PORT        (GPIOB)
#define GC0308_PWDN_PIN         (GPIO_PIN_7)
#define GC0308_EXIT_PWDN        (GC0308_PWDN_PORT->PBC = GC0308_PWDN_PIN)
#define GC0308_ENTER_PWDN       (GC0308_PWDN_PORT->PBSC = GC0308_PWDN_PIN)

#if     ((BOARD_TYPE == BOARD_V1_1)||(BOARD_TYPE == BOARD_V1_2))
#define GC0308_RESET_PORT_CLK   (RCC_APB2_PERIPH_GPIOA)
#define GC0308_RESET_PORT       (GPIOA)
#define GC0308_RESET_PIN        (GPIO_PIN_15)

#elif   (BOARD_TYPE == BOARD_V1_0)
#define GC0308_RESET_PORT_CLK   (RCC_APB2_PERIPH_GPIOG)
#define GC0308_RESET_PORT       (GPIOG)
#define GC0308_RESET_PIN        (GPIO_PIN_9)

#else
    #error  "Invalid BOARD_TYPE!"
#endif

#define GC0308_EXIT_RESET       (GC0308_RESET_PORT->PBSC = GC0308_RESET_PIN)
#define GC0308_ENTER_RESET      (GC0308_RESET_PORT->PBC = GC0308_RESET_PIN)

void GC0308_Init(void);

#ifdef __cplusplus
}
#endif

#endif

