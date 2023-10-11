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
* File Name   : mtr_ctrl_t1102.c
* Description : The processing of a H/W control layer(Inverter board dependance)
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : 28.05.2020  Ver.1.00
*         : 21.04.2021  Ver.1.10
***********************************************************************************************************************/

/***********************************************************************************************************************
* Includes <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include <machine.h>
#include "mtr_common.h"
#include "mtr_ctrl_t1102.h"
#include "iodefine.h"
#include "mtr_ctrl_rx13t48_t1102.h"
#include "mtr_ctrl_rx13t48.h"

/******************************************************************************
* Exported global variables
******************************************************************************/
extern volatile float32  g_f4_iu_ad;
extern volatile float32  g_f4_iv_ad;
extern volatile float32  g_f4_iw_ad;
extern volatile float32  g_f4_vdc_ad;

/******************************************************************************
* Function Name : R_MTR_ChargeCapacitor
* Description   : Wait for charging capacitor
* Arguments     : none
* Return Value  : none
******************************************************************************/
void R_MTR_ChargeCapacitor(void)
{
    while (g_f4_vdc_ad < MTR_IC_GATE_ON_V)
    {
        mtr_get_iuiviwvdc(&g_f4_iu_ad, &g_f4_iv_ad, &g_f4_iw_ad, &g_f4_vdc_ad);
        g_f4_vdc_ad = g_f4_vdc_ad * MTR_VDC_SCALING;
    }
    ic_gate_on();
}

/******************************************************************************
* Function Name : ic_gate_on
* Description   : Turn on gate for preventing the inrush current
* Arguments     : none
* Return Value  : none
******************************************************************************/
void ic_gate_on(void)
{
    MTR_PORT_IC_GATE = MTR_IC_GATE_ON;
}

/******************************************************************************
* Function Name : led1_on
* Description   : Turn on LED1
* Arguments     : none
* Return Value  : none
******************************************************************************/
void led1_on(void)
{
    MTR_PORT_LED1 = MTR_LED_ON;
}

/******************************************************************************
* Function Name : led2_on
* Description   : Turn on LED2
* Arguments     : none
* Return Value  : none
******************************************************************************/
void led2_on(void)
{
    MTR_PORT_LED2 = MTR_LED_ON;
}

/******************************************************************************
* Function Name : led1_off
* Description   : Turn off LED1
* Arguments     : none
* Return Value  : none
******************************************************************************/
void led1_off(void)
{
    MTR_PORT_LED1 = MTR_LED_OFF;
}

/******************************************************************************
* Function Name : led2_off
* Description   : Turn off LED2
* Arguments     : none
* Return Value  : none
******************************************************************************/
void led2_off(void)
{
    MTR_PORT_LED2 = MTR_LED_OFF;
}

/******************************************************************************
* Function Name : get_sw1
* Description   : get drive switch status
* Arguments     : none
* Return Value  : drive switch status
******************************************************************************/
uint8 get_sw1(void)
{
    uint8 u1_temp;
    
    u1_temp = MTR_PORT_SW1;
    return (u1_temp);
}

/******************************************************************************
* Function Name : get_sw2
* Description   : get error reset switch status
* Arguments     : none
* Return Value  : error reset switch status
******************************************************************************/
uint8 get_sw2(void)
{
    uint8 u1_temp;
    
    u1_temp = MTR_PORT_SW2;
    return (u1_temp);
}

/******************************************************************************
* Function Name : get_sw3
* Description   : get timer mode switch status
* Arguments     : none
* Return Value  : timer mode switch status
******************************************************************************/
uint8 get_sw3(void)
{
    uint8 u1_temp;
    
    u1_temp = MTR_PORT_SW3;
    return (u1_temp);
}

/******************************************************************************
* Function Name : get_vr1 
* Description   : get ref speed value
* Arguments     : none
* Return Value  : ref speed value
******************************************************************************/
uint16 get_vr1(void)
{
    uint16 u2_ref_speed_ad;

    u2_ref_speed_ad = S12AD.ADDR7;

    return (u2_ref_speed_ad);
}