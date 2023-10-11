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
* File Name   : mtr_ctrl_t1102.h
* Description : A definitions of the processing of a H/W control layer
******************************************************************************/
/******************************************************************************
* History : 28.05.2020  Ver.1.00
*         : 21.04.2021  Ver.1.10
******************************************************************************/

/* guard against multiple inclusion */
#ifndef MTR_CTRL_T1102_H
#define MTR_CTRL_T1102_H

/******************************************************************************
* Global function definitions
******************************************************************************/
void        R_MTR_ChargeCapacitor(void);/* wait for charging capacitor */
void        ic_gate_on(void);           /* turn on gate for preventing the inrush current */
void        led1_on(void);              /* turn on LED1 */
void        led2_on(void);              /* turn on LED2 */
void        led1_off(void);             /* turn off LED1 */
void        led2_off(void);             /* turn off LED2 */
uint8       get_sw1(void);              /* get sw1 */
uint8       get_sw2(void);              /* get sw2 */
uint8       get_sw3(void);              /* get sw3 */
uint16      get_vr1(void);              /* get vr1 */
#endif /* MTR_CTRL_T1102_H */
