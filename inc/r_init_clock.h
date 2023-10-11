/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
* other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
* EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
* SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
* SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
* this software. By using this software, you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2021 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/******************************************************************************
* File Name   : r_init_clock.h
* Description :Initialize the clock
******************************************************************************/
/******************************************************************************
* History : 28.05.2020  Ver.1.00
*         : 21.04.2021  Ver.1.10
******************************************************************************/

/* Guards against multiple inclusion */
#ifndef R_INIT_CLOCK_H
#define R_INIT_CLOCK_H

/*******************************************************************************
 Macro definitions
 *******************************************************************************/

/* The value to select whether to use */
    #define B_NOT_USE       (0)
    #define B_USE           (1)
    #define B_USE_PLL_MAIN  (2)
    #define B_USE_PLL_HOCO  (3)

/* ==== Please set the main clock ==== */
/* This sample code uses the main clock with the following settings.
 Please change the following settings as necessary. */
    #define SEL_MAIN        (B_NOT_USE)
/* Frequency : 8 MHz */
    #define MAIN_CLOCK_Hz   (8000000L)
/* Drive capability : 8 MHz crystal resonator */
    #define REG_MOFCR       (0x00)
/* Wait control register : 1.024ms(Clock stabilization wait time) */
    #define REG_MOSCWTCR    (0x03)

/* ==== Please set the PLL clock ==== */
/* This sample code uses the PLL clock with the following settings.
 Please change the following settings as necessary. */
    #define SEL_PLL         (B_USE_PLL_HOCO)
/* Division ratio and multiplication factor and clock source.
 */
    #define REG_PLLCR   (0x0700)        /* no division, multiply-by-4 main clock */

/* ==== Please select the source of the system clock ==== */
    #if (SEL_HOCO == B_USE) || (SEL_PLL == B_USE_PLL_HOCO)
        #define CLK_HOCO    (0x0100)        /* HOCO */
    #endif
    #if (SEL_MAIN == B_USE)
        #define CLK_MAIN    (0x0200)        /* Main clock */
    #endif
    #if (SEL_PLL == B_USE_PLL_MAIN)
        #define CLK_PLL     (0x0400)        /* PLL */
    #endif

/* This sample code uses PLL as the system clock.
 Please change the following settings as necessary. */
    #define SEL_SYSCLK      (CLK_HOCO)

/* ==== Please select the internal clock division ratio ==== */
/* This sample code uses the following internal clock division ratio.
 Please change the following settings as necessary. */
    #define REG_SCKCR   (0x00000000)    /* ICLK,FCLK,PCLKB,PCLKD : no division */

/*******************************************************************************
 Exported global variables and functions (to be accessed by other files)
 *******************************************************************************/
void R_INIT_Clock (void);
void CGC_oscillation_main (void);
void CGC_oscillation_PLL (void);
void CGC_oscillation_HOCO (void);

#endif /* R_INIT_CLOCK_H */

/* End of File */

