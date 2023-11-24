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
* File Name   : mtr_ctrl_rx13t48_t1102.c
* Description : The processing of MCU & inverter control layer
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

/******************************************************************************
* Function Name : mtr_init_mtu
* Description   : Initialize MTU3
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_init_mtu(void)
{
    MPC.PWPR.BIT.B0WI = 0;
    MPC.PWPR.BIT.PFSWE = 1;
    MPC.P71PFS.BIT.PSEL = 0x01;
    MPC.P72PFS.BIT.PSEL = 0x01;
    MPC.P73PFS.BIT.PSEL = 0x01;
    MPC.P74PFS.BIT.PSEL = 0x01;
    MPC.P75PFS.BIT.PSEL = 0x01;
    MPC.P76PFS.BIT.PSEL = 0x01;
    MPC.PWPR.BIT.PFSWE = 0;
    MPC.PWPR.BIT.B0WI = 1;

    SYSTEM.PRCR.WORD = 0xA502;                          /* Enable write protection */
    MSTP_MTU = 0;                                       /* the module stop state is canceled */
    SYSTEM.PRCR.WORD = 0xA500;                          /* Disable write protection */

    MTU.TSTRA.BYTE = 0x00;                              /* stop MTU3 */

    MTU3.TCR.BIT.TPSC = 0;                              /* MTU3_3 clock : ICLK */
    MTU3.TCR.BIT.CKEG = 0;
    MTU4.TCR.BIT.TPSC = 0;                              /* MTU3_4 clock : ICLK */
    MTU4.TCR.BIT.CKEG = 0;

    MTU.TGCRA.BIT.BDC = 0;

    MTU3.TCNT = MTR_DEADTIME_SET;                       /* set deadtime */
    MTU4.TCNT = 0;

    MTU3.TGRB = (uint16)(MTR_HALF_CARRIER_SET + (MTR_DEADTIME_SET + MTR_AD_TIME_SET) / 2);
    MTU4.TGRA = (uint16)(MTR_HALF_CARRIER_SET + (MTR_DEADTIME_SET + MTR_AD_TIME_SET) / 2);
    MTU4.TGRB = (uint16)(MTR_HALF_CARRIER_SET + (MTR_DEADTIME_SET + MTR_AD_TIME_SET) / 2);
                                                        /* set duty */
    MTU3.TGRD = (uint16)(MTR_HALF_CARRIER_SET + (MTR_DEADTIME_SET + MTR_AD_TIME_SET) / 2);
    MTU4.TGRC = (uint16)(MTR_HALF_CARRIER_SET + (MTR_DEADTIME_SET + MTR_AD_TIME_SET) / 2);
    MTU4.TGRD = (uint16)(MTR_HALF_CARRIER_SET + (MTR_DEADTIME_SET + MTR_AD_TIME_SET) / 2);
                                                        /* set duty */

    MTU.TDDRA = MTR_DEADTIME_SET;

    MTU.TCDRA = (uint16)(MTR_CARRIER_SET - MTR_DEADTIME_SET);
    MTU.TCBRA = (uint16)(MTR_CARRIER_SET - MTR_DEADTIME_SET);

    MTU3.TGRA = MTR_CARRIER_SET;                        /* set carrier period */
    MTU3.TGRC = MTR_CARRIER_SET;

    MTU.TOCR1A.BIT.PSYE = 0;
    MTU.TOCR1A.BIT.OLSP = 1;                            /* positive phase : initial output"L",active level"H" */
    MTU.TOCR1A.BIT.OLSN = 1;                            /* negative phase : initial output"L",active level"H" */

    MTU3.TMDR1.BIT.MD = 14;                             /* complementary PWM mode (transfer at trough) */
    MTU3.TMDR1.BIT.BFA = 1;                             /* TGRA and TGRC used together for buffer operation */
    MTU3.TMDR1.BIT.BFB = 1;                             /* TGRB and TGRD used together for buffer operation */
    MTU4.TMDR1.BYTE = 0x00;

    MTU.TOERA.BYTE = 0xC0;                              /* PWM output disable  */

    MTU4.TIER.BIT.TCIEV = 1;
    MTU4.TIER.BIT.TTGE2 = 1;
    MTU4.TIER.BIT.TTGE = 0;

    ICU.IPR[IPR_MTU4_TCIV4].BIT.IPR = 14;
    ICU.IR[IR_MTU4_TCIV4].BIT.IR = 0;
    ICU.IER[IER_MTU4_TCIV4].BIT.IEN2 = 1;

    MTU.TITMRA.BIT.TITM = 0;                            /* set interrupt skipping function */
    MTU.TITCR1A.BIT.T4VCOR = MTR_INT_DECIMATION;
    MTU.TITCR1A.BIT.T4VEN = 1;

    MTU.TSTRA.BYTE = 0xC0;                              /* start MTU3_3 and MTU3_4 */
}

/******************************************************************************
* Function Name : mtr_init_io_port
* Description   : Initialize I/O port
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_init_io_port(void)
{
    /*************** inverter ***************/
    PORT7.PODR.BIT.B1 = 0;                              /* Up = "L" */
    PORT7.PODR.BIT.B2 = 0;                              /* Vp = "L" */
    PORT7.PODR.BIT.B3 = 0;                              /* Wp = "L" */
    PORT7.PODR.BIT.B4 = 0;                              /* Un = "L" */
    PORT7.PODR.BIT.B5 = 0;                              /* Vn = "L" */
    PORT7.PODR.BIT.B6 = 0;                              /* Wn = "L" */
    PORT7.PDR.BIT.B1 = 1;                               /* output mode for Up */
    PORT7.PDR.BIT.B2 = 1;                               /* output mode for Vp */
    PORT7.PDR.BIT.B3 = 1;                               /* output mode for Wp */
    PORT7.PDR.BIT.B4 = 1;                               /* output mode for Un */
    PORT7.PDR.BIT.B5 = 1;                               /* output mode for Vn */
    PORT7.PDR.BIT.B6 = 1;                               /* output mode for Wn */

    PORTB.PODR.BIT.B3 = 0;                              /* PFC_G1 = "L" */
    PORTB.PDR.BIT.B2 = 0;                               /* input  mode motor enable 0: enable, 1: disable */
    PORTB.PDR.BIT.B3 = 1;                               /* output mode */
    PORTB.PDR.BIT.B5 = 0;                               /* DIR input  mode */

    /*************** unused ***************/
    PORT1.PODR.BIT.B0 = 1;
    PORT1.PDR.BIT.B1 = 0;
    PORT1.PDR.BIT.B0 = 1;

    PORT2.PODR.BIT.B3 = 1;
    PORT2.PODR.BIT.B2 = 1;
    PORT2.PDR.BIT.B4 = 0;
    PORT2.PDR.BIT.B3 = 1;
    PORT2.PDR.BIT.B2 = 1;

    PORT7.PDR.BIT.B0 = 0;

    PORT9.PODR.BIT.B4 = 1;
    PORT9.PDR.BIT.B4 = 1;

    PORTA.PODR.BIT.B3 = 1;
    /* PORTA.PODR.BIT.B2 = 1; */
    PORTA.PDR.BIT.B3 = 1;
    /* PORTA.PDR.BIT.B2 = 1; */

    PORTB.PODR.BIT.B6 = 1;
    PORTB.PODR.BIT.B1 = 1;
    PORTB.PODR.BIT.B0 = 1;
    PORTB.PDR.BIT.B7 = 0;
    PORTB.PDR.BIT.B6 = 1;
    PORTB.PDR.BIT.B1 = 1;
    PORTB.PDR.BIT.B0 = 1;
}

/******************************************************************************
* Function Name : mtr_init_ad_converter
* Description   : Initialize A/D converter
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_init_ad_converter(void)
{

    MPC.PWPR.BIT.B0WI = 0;
    MPC.PWPR.BIT.PFSWE = 1;
    MPC.P40PFS.BIT.ASEL = 1;
    MPC.P41PFS.BIT.ASEL = 1;
    MPC.P42PFS.BIT.ASEL = 1;
    MPC.P43PFS.BIT.ASEL = 1;
    MPC.P44PFS.BIT.ASEL = 1;
    MPC.P45PFS.BIT.ASEL = 1;
    MPC.P46PFS.BIT.ASEL = 1;
    MPC.P47PFS.BIT.ASEL = 1;
    MPC.PWPR.BIT.PFSWE = 0;
    MPC.PWPR.BIT.B0WI = 1;

    SYSTEM.PRCR.WORD = 0xA502;                          /* Enable write protection */
    MSTP_S12AD = 0;                                     /* the module stop state is canceled */
    SYSTEM.PRCR.WORD = 0xA500;                          /* Disable write protection */

    /*************** 12bit A/D converter Unit0 (Iu,Iw,Vdc) ***************/
    S12AD.ADCSR.BIT.EXTRG = 0;                         /* use single-cycle scan mode with sample-and-hold function */
    S12AD.ADCSR.BIT.TRGE = 1;
    S12AD.ADCSR.BIT.ADIE = 0;
    S12AD.ADCSR.BIT.ADCS = 1;

    S12AD.ADANSA0.WORD = 0x00FF;                       /* select chanel AN000...AN007 */

    S12AD.ADSHCR.BIT.SHANS = 7;
    S12AD.ADCER.BIT.ADRFMT = 0;

    /* PGA disable */
    S12AD.ADPGACR.BIT.P000SEL1 = 0;
    S12AD.ADPGACR.BIT.P001SEL1 = 0;
    S12AD.ADPGACR.BIT.P002SEL1 = 0;
    S12AD.ADPGACR.BIT.P000ENAMP = 0;
    S12AD.ADPGACR.BIT.P001ENAMP = 0;
    S12AD.ADPGACR.BIT.P002ENAMP = 0;

    S12AD.ADSTRGR.BIT.TRSA = 5;    /* trigger is TRGA4N */

}

/******************************************************************************
* Function Name : init_ui
* Description   : Initialize peripheral function for user interface
* Arguments     : none
* Return Value  : none
******************************************************************************/
void init_ui(void)
{
    /*************** LED1,2 ***************/
    PORTD.PODR.BIT.B6 = 1;
    PORTD.PODR.BIT.B4 = 1;
    PORTD.PDR.BIT.B6 = 1;
    PORTD.PDR.BIT.B4 = 1;

    /************* SW1,2 **************/
    PORTB.PDR.BIT.B5 = 0;
    PORTB.PDR.BIT.B4 = 0;

    /************* ICS **************/
    MPC.PWPR.BIT.B0WI = 0;
    MPC.PWPR.BIT.PFSWE = 1;
    MPC.PD5PFS.BIT.PSEL = 0x0A;
    MPC.PD3PFS.BIT.PSEL = 0x0A;
    MPC.PWPR.BIT.PFSWE = 0;
    MPC.PWPR.BIT.B0WI = 1;

    PORTD.PDR.BIT.B5 = 0;
    PORTD.PDR.BIT.B3 = 1;
    PORTD.PMR.BIT.B5 = 1;
    PORTD.PMR.BIT.B3 = 1;
}

/******************************************************************************
* Function Name : mtr_ctrl_start
* Description   : PWM output start for motor control start
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_ctrl_start(void)
{
    MTR_PORT_UP = 0;                                    /* Up = "L" */
    MTR_PORT_UN = 0;                                    /* Un = "L" */
    MTR_PORT_VP = 0;                                    /* Vp = "L" */
    MTR_PORT_VN = 0;                                    /* Vn = "L" */
    MTR_PORT_WP = 0;                                    /* Wp = "L" */
    MTR_PORT_WN = 0;                                    /* Wn = "L" */

    PORT7.PMR.BIT.B1 = 1;
    PORT7.PMR.BIT.B2 = 1;
    PORT7.PMR.BIT.B3 = 1;
    PORT7.PMR.BIT.B4 = 1;
    PORT7.PMR.BIT.B5 = 1;
    PORT7.PMR.BIT.B6 = 1;

    MTU.TOERA.BYTE = 0xFF;                              /* PWM output enable */
}

/******************************************************************************
* Function Name : mtr_ctrl_stop
* Description   : PWM output disable for motor control stop
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_ctrl_stop(void)
{
    MTU.TOERA.BYTE = 0xC0;                              /* PWM output disable */

    MTR_PORT_UP = 0;                                    /* Up = "L" */
    MTR_PORT_UN = 0;                                    /* Un = "L" */
    MTR_PORT_VP = 0;                                    /* Vp = "L" */
    MTR_PORT_VN = 0;                                    /* Vn = "L" */
    MTR_PORT_WP = 0;                                    /* Wp = "L" */
    MTR_PORT_WN = 0;                                    /* Wn = "L" */

    PORT7.PMR.BIT.B1 = 0;
    PORT7.PMR.BIT.B2 = 0;
    PORT7.PMR.BIT.B3 = 0;
    PORT7.PMR.BIT.B4 = 0;
    PORT7.PMR.BIT.B5 = 0;
    PORT7.PMR.BIT.B6 = 0;
}

/******************************************************************************
* Function Name : mtr_get_iuiviwvdc
* Description   : A/D conversion for Iu,Iv,Iw,Vdc
* Arguments     : f4_iu_ad
                  f4_iv_ad
                  f4_iw_ad
                  f4_vdc_ad
* Return Value  : A/D converted value
******************************************************************************/
void mtr_get_iuiviwvdc(volatile float32 *f4_iu_ad, volatile float32 *f4_iv_ad, volatile float32 *f4_iw_ad, volatile float32 *f4_vdc_ad)
{
    while (S12AD.ADCSR.BIT.ADST == 1);
    *f4_iu_ad = (float32)S12AD.ADDR0;
    *f4_iv_ad = (float32)S12AD.ADDR1;
    *f4_iw_ad = (float32)S12AD.ADDR2;
    *f4_vdc_ad = (float32)S12AD.ADDR6;
}

/******************************************************************************
* Function Name : mtr_get_ipm_temperature
* Description   : A/D conversion for IPM temperature
* Arguments     : none
* Return Value  : A/D converted value
******************************************************************************/
int16 mtr_get_ipm_temperature(void)
{
    int16 s2_temp;

    s2_temp = S12AD.ADDR5;

    return (s2_temp);
}

/******************************************************************************
* Function Name : mtr_clear_mtu4_flag
* Description   : Clear MTU4 interrupt flag
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_clear_mtu4_flag(void)
{
    ICU.IR[IR_MTU4_TCIV4].BIT.IR = 0;
}

/******************************************************************************
* Function Name : mtr_inv_set_uvw
* Description   : PWM duty setting
* Arguments     : float32 f4_modu, float32 f4_modv, float32 f4_modw
* Return Value  : none
******************************************************************************/
void mtr_inv_set_uvw(float32 f4_modu, float32 f4_modv, float32 f4_modw)
{
    float32 f4_temp0, f4_temp1;

    f4_temp0 = (float32)(MTR_HALF_CARRIER_SET + (MTR_DEADTIME_SET + MTR_AD_TIME_SET) / 2);
    f4_temp1 = (float32)(MTR_CARRIER_SET);

    f4_modu = -f4_modu;
    MTU3.TGRD = (uint16)(((f4_temp1 - f4_temp0) * f4_modu) + f4_temp0);
                                                        /* set U-phase duty */
    f4_modv = -f4_modv;
    MTU4.TGRC = (uint16)(((f4_temp1 - f4_temp0) * f4_modv) + f4_temp0);
                                                        /* set V-phase duty */
    f4_modw = -f4_modw;
    MTU4.TGRD = (uint16)(((f4_temp1 - f4_temp0) * f4_modw) + f4_temp0);
                                                        /* set W-phase duty */
}

/******************************************************************************
* Function Name : mtr_init_register
* Description   : Remove unintentional first pulse
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_init_register(void)
{
    MTU3.TGRD = (uint16)(MTR_HALF_CARRIER_SET + (MTR_DEADTIME_SET + MTR_AD_TIME_SET) / 2);
    MTU4.TGRC = (uint16)(MTR_HALF_CARRIER_SET + (MTR_DEADTIME_SET + MTR_AD_TIME_SET) / 2);
    MTU4.TGRD = (uint16)(MTR_HALF_CARRIER_SET + (MTR_DEADTIME_SET + MTR_AD_TIME_SET) / 2);
                                                        /* set duty */
}
