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
* File Name   : mtr_ctrl_rx13t48.h
* Description : A definitions of the processing of a MCU control layer
******************************************************************************/
/******************************************************************************
* History : 28.05.2020  Ver.1.00
*         : 21.04.2021  Ver.1.10
******************************************************************************/

/* guard against multiple inclusion */
#ifndef MTR_CTRL_RX13T48_H
#define MTR_CTRL_RX13T48_H

/******************************************************************************
* Global function definitions
******************************************************************************/
void    R_MTR_InitHardware(void);                              /* initialize peripheral function */
void    clear_wdt(void);                                       /* clear watch dog timer */
void    mtr_clear_oc_flag(void);                               /* clear forced cutoff flag (detect over current) */
void    mtr_clear_cmt0_flag(void);                             /* clear CMT0 interrupt flag */

#endif /* MTR_CTRL_RX13T48_H */
