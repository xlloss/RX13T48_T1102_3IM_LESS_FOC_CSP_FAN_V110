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
/***********************************************************************************************************************
* File Name   : r_init_clock.c
* Description : Initialize the clock.
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : 28.05.2020  Ver.1.00
*         : 21.04.2021  Ver.1.10
***********************************************************************************************************************/

/***********************************************************************************************************************
* Includes <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include <stdint.h>
#include "iodefine.h"
#include "r_init_clock.h"

/*******************************************************************************
 Macro definitions
 *******************************************************************************/

/*******************************************************************************
 Private variables and functions
 *******************************************************************************/

/*******************************************************************************
 * Outline      : Initial clock settings
 * Header       : r_init_clock.h
 * Function Name: R_INIT_Clock
 * Description  : Makes initial clock settings and specifies the number of memory wait cycles.
 * Arguments    : none
 * Return Value : none
 * Note         : This function should be called when the PSW.I bit is 0 (interrupt disabled).
 *                In order to guarantee normal operation, set appropriate values to the VOLSR and
 *                the ADPGADCR0 registers.
 *                The value to set VOLSR is specify with the definition VOLSR_REG in the file r_init_clock.h.
 *                The value to set ADPGADCR0 is specified directly in this file.
 *******************************************************************************/
void R_INIT_Clock (void)
{
    /* ---- Enable write protection ---- */
    /* PRCR - Protect Register
     b15:b8  PRKEY - PRC Key Code - A5h
     (The write value should be A5h to permission writing PRCi bit)
     b7:b4   Reserved - The write value should be 0.
     b3      PRC3 - Protect Bit 3 - Write disabled
     b2      Reserved - The write value should be 0.
     b1      PRC1 - Protect Bit 1 - Write enabled
     b0      PRC0 - Protect Bit 0 - Write enabled */
    SYSTEM.PRCR.WORD = 0xA503;

    /* ---- Turn off the PLL power supply ---- */
#if (SEL_PLL == B_NOT_USE)
    /* PLLCR2 - PLL Control Register 2
     b7:b1   Reserved - The write value should be 0.
     b0      PLLEN  - PLL Control Register - Powered off */
    SYSTEM.PLLCR2.BYTE = 0x01;
#endif

    /* ---- Set the main clock ---- */
#if ((SEL_MAIN == B_USE) || (SEL_PLL == B_USE_PLL_MAIN))
    CGC_oscillation_main();
#endif

    /* ---- Set the HOCO clock ---- */
#if ((SEL_HOCO == B_USE) || (SEL_PLL == B_USE_PLL_HOCO))
    CGC_oscillation_HOCO();
#endif

    /* ---- Set the PLL clock ---- */
#if (SEL_PLL == B_USE_PLL_MAIN) || (SEL_PLL == B_USE_PLL_HOCO)
    CGC_oscillation_PLL();
#endif

    /* ---- Set the internal clock division ratio ---- */
    /* SCKCR - System Clock Control Register
     b31:b28 FCK      - FlashIF Clock(FCLK) Select - divide-by-4
     b27:b24 ICK      - System Clock (ICLK) Select - no division
     b23:b12 Reserved - The write value should be 0.
     b11:b8  PCLKB    - Peripheral Module Clock B(PCLKB) Select - divide-by-4
     b7:b4   Reserved - The write value should be 0.
     b3:b0   PCLKD    - Peripheral Module Clock D(PCLKD) Select - divide-by-4 */
    SYSTEM.SCKCR.LONG = REG_SCKCR;

    while (REG_SCKCR != SYSTEM.SCKCR.LONG)
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Set the internal clock source ---- */
    /* SCKCR3 - System Clock Control Register 3
     b15:b11 Reserved - The write value should be 0.
     b10:b8  CKSEL    - Clock Source Select - PLL circuit is selected.
     b7:b1   Reserved - The write value should be 0. */
    SYSTEM.SCKCR3.WORD = SEL_SYSCLK;

    while (SEL_SYSCLK != SYSTEM.SCKCR3.WORD)
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Disable write protection ---- */
    /* PRCR - Protect Register
     b15:b8 PRKEY - PRC Key Code - A5h
     (The write value should be A5h to permission writing PRCi bit)
     b2     Reserved - The write value should be 0.
     b1     PRC1 - Protect Bit 1 - Write disabled
     b0     PRC0 - Protect Bit 0 - Write disabled */
    SYSTEM.PRCR.WORD = 0xA500;
}
/*******************************************************************************
 End of function R_INIT_Clock
 *******************************************************************************/

/*******************************************************************************
 * Outline      : Main clock oscillation enable
 * Header       : r_init_clock.h
 * Function Name: CGC_oscillation_main
 * Description  : Sets the drive capacity of the main clock and sets the MOSCWTCR register, 
 *                then starts oscillation of the main clock. 
 *                After this, waits for the main clock oscillation stabilization waiting time using hardware.
 * Arguments    : none
 * Return Value : none
 *******************************************************************************/
void CGC_oscillation_main (void)
{
    /* ----  Set the main clock oscillator drive capability ---- */
    /* MOFCR - Main Clock Oscillator Forced Oscillation Control Register
     b7       Reserved - The write value should be 0.
     b6       MOSEL    - Main Clock Oscillator Switch - Resonator
     b5:b4    MODRV2   - Main Clock Oscillator Drive Capability 2 Switching - 8MHz
     b3:b0    Reserved - The write value should be 0. */
    SYSTEM.MOFCR.BYTE = REG_MOFCR;

    while (REG_MOFCR != SYSTEM.MOFCR.BYTE)
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Set wait time until the main clock oscillator stabilizes ---- */
    /* MOSCWTCR - Main Clock Oscillator Wait Control Register
     b7:b0    MSTS     - Main Clock Oscillator Waiting Time
                       - Wait time is 2635 cycles (approx. 9.98 ms). */
    SYSTEM.MOSCWTCR.BYTE = REG_MOSCWTCR;

    /* ---- Operate the main clock oscillator ---- */
    /* MOSCCR   - Main Clock Oscillator Control Register
     b7:b1    Reserved - The write value should be 0.
     b0       MOSTP    - Main Clock Oscillator Stop - Main clock oscillator is operating. */
    SYSTEM.MOSCCR.BYTE = 0x00;

    while (0x00 != SYSTEM.MOSCCR.BYTE)
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Wait processing for the clock oscillation stabilization ---- */
    /* OSCOVFSR - Oscillation Stabilization Flag Register
     b7:b5  Reserved    - The write value should be 0.
     b4     ILCOVF      - IWDT Low-Speed Clock Oscillation Stabilization Flag
     b3     HCOVF       - HOCO Clock Oscillation Stabilization Flag
     b2     PLOVF       - PLL Clock Oscillation Stabilization Flag
     b1     Reserved    - The write value should be 0.
     b0     MOOVF       - Main Clock Oscillation Stabilization Flag */
    while (1 != SYSTEM.OSCOVFSR.BIT.MOOVF)
    {
        /* Confirm that the Oscillation of the main clock is stable so the clock is
         available for use as the system clock.*/
    }
}
/*******************************************************************************
 End of function CGC_oscillation_main
 *******************************************************************************/

/*******************************************************************************
 * Outline      : PLL clock oscillation enable
 * Header       : r_init_clock.h
 * Function Name: CGC_oscillation_PLL
 * Description  : Sets the PLL input division ratio and frequency multiplication factor, 
 *                then starts oscillation of the PLL clock. 
 *                After this, waits for the PLL clock oscillation stabilization waiting time using hardware.
 * Arguments    : none
 * Return Value : none
 *******************************************************************************/
void CGC_oscillation_PLL (void)
{
    /* ---- Set the PLL division ratio and multiplication factor ---- */
    /* PLLCR - PLL Control Register
     b15:b14  Reserved  - The write value should be 0.
     b13:b8   STC       - Frequency Multiplication Factor Select
                        - Frequency multiplication factor is multiply-by-4.
     b7:b2    Reserved  - The write value should be 0.
     b1:b0    PLIDIV    - PLL Input Frequency Division Ratio Select
                        - PLL input division ratio is no division. */
    SYSTEM.PLLCR.WORD = REG_PLLCR;

    /* ---- Operate the PLL clock oscillator ---- */
    /* PLLCR2 - PLL Control Register 2
     b7:b1    Reserved - The write value should be 0.
     b0       PLLEN    - PLL Stop Control - PLL is operating. */
    SYSTEM.PLLCR2.BYTE = 0x00;

    /* ---- Wait processing for the clock oscillation stabilization ---- */
    while (1 != SYSTEM.OSCOVFSR.BIT.PLOVF)
    {
        /* Confirm that the Oscillation of the pll clock is stable so the clock is
         available for use as the system clock.*/
    }
}
/*******************************************************************************
 End of function CGC_oscillation_PLL
 *******************************************************************************/

/*******************************************************************************
 * Outline      : HOCO clock oscillation enable
 * Header       : r_init_clock.h
 * Function Name: CGC_oscillation_HOCO
 * Description  : Sets the HOCO frequency, then starts oscillation of the HOCO.
 *                After this, waits for the HOCO oscillation stabilization waiting time using hardware.
 * Arguments    : none
 * Return Value : none
 *******************************************************************************/
void CGC_oscillation_HOCO (void)
{
    /* ---- Operate the HOCO clock ---- */
    /* HOCOCR - High-Speed On-Chip Oscillator Control Register
     b7:b1    Reserved   - The write value should be 0.
     b0       HCSTP      - the HOCO is operating. */
    SYSTEM.HOCOCR.BYTE = 0x00;

    /* ---- Wait processing for the clock oscillation stabilization ---- */
    while (1 != SYSTEM.OSCOVFSR.BIT.HCOVF)
    {
        /* Confirm that the Oscillation of the pll clock is stable so the clock is
         available for use as the system clock.*/
    }
}
/*******************************************************************************
 End of function CGC_oscillation_HOCO
 *******************************************************************************/

/* End of File */

