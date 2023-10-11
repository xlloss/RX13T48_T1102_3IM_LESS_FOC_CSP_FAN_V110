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
* File Name   : mtr_ctrl_rx13t48.c
* Description : The processing of a MCU control layer
******************************************************************************/
/******************************************************************************
* History : 28.05.2020  Ver.1.00
*         : 21.04.2021  Ver.1.10
******************************************************************************/

/******************************************************************************
* Includes <System Includes> , "Project Includes"
******************************************************************************/
#include "iodefine.h"
#include "mtr_common.h"
#include "mtr_3im_less_foc.h"
#include "mtr_ctrl_rx13t48.h"
#include "mtr_ctrl_rx13t48_t1102.h"

#include "r_init_clock.h"

/******************************************************************************
* Private global variables and functions
******************************************************************************/
void    mtr_init_module(void);                      /* initialize module */
void    mtr_init_cmt(void);                             /* initialize CMT */
void    mtr_init_poe3(void);                            /* initialize POE3 */
void    init_wdt(void);                                 /* initialize WDT */

/******************************************************************************
* Function Name : R_MTR_InitHardware
* Description   : Initialize peripheral function
* Arguments     : none
* Return Value  : none
******************************************************************************/
void R_MTR_InitHardware(void)
{
    /*==================================*/
    /*     Module stop state release    */
    /*==================================*/
    mtr_init_module();

    /*=============================*/
    /*     initialize I/O port     */
    /*=============================*/
    mtr_init_io_port();

    /*==========================*/
    /*     initialize clock     */
    /*==========================*/
    R_INIT_Clock();

    /*========================*/
    /*     initialize CMT0    */
    /*========================*/
    mtr_init_cmt();

    /*===================================*/
    /*     initialize MTU3_3,MTU3_4      */
    /*===================================*/
    mtr_init_mtu();

    /*=========================*/
    /*     initialize POE3     */
    /*=========================*/
    mtr_init_poe3();

    /*==================================*/
    /*     initialize A/D converter     */
    /*==================================*/
    mtr_init_ad_converter();

    /*========================*/
    /*     initialize WDT     */
    /*========================*/
    init_wdt();
}

/******************************************************************************
* Function Name : mtr_init_module
* Description   : Initialize module
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_init_module(void)
{
    /* ---- Enable write protection ---- */
    /* PRCR - Protect Register 
     b15:b8 PRKEY - PRC Key Code - A5h 
     (The write value should be A5h to permission writing PRCi bit)
     b7:b4  Reserved - The write value should be 0.
     b3     PRC3 - Protect Bit 3 - Write disabled
     b2     Reserved - The write value should be 0.
     b1     PRC1 - Protect Bit 1 - Write enabled
     b0     PRC0 - Protect Bit 0 - Write disabled */
    SYSTEM.PRCR.WORD = 0xA502;

    /* Module stop state release */
    MSTP_SCI1 = 0;                                      /* the module stop state is canceled */
    MSTP_SCI5 = 0;                                      /* the module stop state is canceled */
    MSTP_DTC  = 0;                                      /* the module stop state is canceled */
    MSTP_RAM0 = 0;                                      /* the module stop state is canceled */

    /* ---- Disable write protection ---- */
    /* PRCR - Protect Register 
     b15:b8 PRKEY - PRC Key Code - A5h 
     (The write value should be A5h to permission writing PRCi bit)
     b1     PRC1 - Protect Bit 1 - Write disabled */
    SYSTEM.PRCR.WORD = 0xA500;
}

/******************************************************************************
* Function Name : mtr_init_cmt
* Description   : Initialize CMT
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_init_cmt(void)
{
    SYSTEM.PRCR.WORD = 0xA502;                          /* Enable write protection */
    MSTP_CMT0 = 0;                                      /* the module stop state is canceled */
    SYSTEM.PRCR.WORD = 0xA500;                          /* Disable write protection */

    CMT.CMSTR0.BIT.STR0 = 0;                            /* stop CMT0 */

    CMT0.CMCR.BIT.CKS = 0;                              /* clock : PCLK/8 */
    CMT0.CMCOR = 3999;                                  /* set interval period : 1 [ms] */

    CMT0.CMCR.BIT.CMIE = 1;                             /* compare match interrupt (CMI0) enabled */
    ICU.IPR[IPR_CMT0_CMI0].BIT.IPR = 13;
    ICU.IR[IR_CMT0_CMI0].BIT.IR = 0;
    ICU.IER[IER_CMT0_CMI0].BIT.IEN4 = 1;

    CMT.CMSTR0.BIT.STR0 = 1;                            /* start CMT0 */
}

/******************************************************************************
* Function Name : mtr_init_poe3
* Description   : Initialize POE3
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_init_poe3(void)
{
    PORTE.PMR.BIT.B2 = 0;                               /* set POE3 for over current  */
    MPC.PWPR.BIT.B0WI = 0;
    MPC.PWPR.BIT.PFSWE = 1;

    MPC.PE2PFS.BIT.PSEL = 0x07;

    MPC.PWPR.BIT.PFSWE = 0;
    MPC.PWPR.BIT.B0WI = 1;
    PORTE.PMR.BIT.B2 = 1;

    POE.ICSR4.BIT.POE10M = 0x00;
    POE.ICSR4.BIT.POE10E = 1;
    POE.ICSR4.BIT.PIE4 = 1;
    POE.POECR2.BIT.MTU3BDZE = 1;
    POE.POECR2.BIT.MTU4ACZE = 1;
    POE.POECR2.BIT.MTU4BDZE = 1;
	
    ICU.IER[IER_POE_OEI4].BIT.IEN3 = 1;
    ICU.IPR[IPR_POE_OEI4].BIT.IPR = 15;
}

/******************************************************************************
* Function Name : init_wdt
* Description   : Initialize WDT
* Arguments     : none
* Return Value  : none
******************************************************************************/
void init_wdt(void)
{
    IWDT.IWDTCR.WORD = 0x3323;
    IWDT.IWDTSR.BIT.UNDFF = 0;
    IWDT.IWDTSR.BIT.REFEF = 0;
    IWDT.IWDTRCR.BIT.RSTIRQS = 1;

    IWDT.IWDTRR = 0x00;
    IWDT.IWDTRR = 0xFF;
}

/******************************************************************************
* Function Name : mtr_clear_wdt
* Description   : Clear Watch Dog Timer
* Arguments     : none
* Return Value  : none
******************************************************************************/
void clear_wdt(void)
{
    IWDT.IWDTRR = 0x00;
    IWDT.IWDTRR = 0xFF;                           /* clear watch dog timer */
}

/******************************************************************************
* Function Name : mtr_clear_oc_flag
* Description   : Clear forced cutoff flag (detect over current)
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_clear_oc_flag(void)
{
    POE.ICSR4.BIT.POE10F = 0;                   /* release from high-impedance state */
}

/******************************************************************************
* Function Name : mtr_clear_cmt0_flag
* Description   : Clear CMT0 interrupt flag 
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_clear_cmt0_flag(void)
{
    ICU.IR[IR_CMT0_CMI0].BIT.IR = 0;                    /* clear interrupt flag */
}

