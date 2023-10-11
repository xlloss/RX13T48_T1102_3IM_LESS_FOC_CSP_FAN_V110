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
* File Name   : mtr_ctrl_rx13t48_t1102.h
* Description : A definitions of the processing of MCU & inverter control layer
******************************************************************************/
/******************************************************************************
* History : 28.05.2020  Ver.1.00
*         : 21.04.2021  Ver.1.10
******************************************************************************/

/* guard against multiple inclusion */
#ifndef MTR_CTRL_RX13T48_T1102_H
#define MTR_CTRL_RX13T48_T1102_H

/******************************************************************************
* Macro definitions
******************************************************************************/
#define     MTR_PWM_TIMER_FREQ      (32.0f)                   /* PWM timer frequency [MHz] */
#define     MTR_CARRIER_FREQ        (16.0f)                    /* carrier wave frequency [kHz] */

#define     MTR_DEADTIME            (2.5f)                     /* deadtime [us] */
#define     MTR_DEADTIME_SET        ((uint16)(MTR_DEADTIME * MTR_PWM_TIMER_FREQ))
                                                               /* setting value of deadtime */

#define     MTR_AD_FREQ             (32.0f)                    /* A/D frequency [MHz] */
#define     MTR_AD_SAMPLING_CYCLE   (45.0f)                    /* start-of-A/D-conversion delay time + sampling time [cycle] */
#define     MTR_AD_SAMPLING_TIME    (MTR_AD_SAMPLING_CYCLE / MTR_AD_FREQ)
                                                               /* A/D sampling time [us] */
#define     MTR_AD_TIME_SET         ((uint16)(MTR_PWM_TIMER_FREQ * MTR_AD_SAMPLING_TIME))
                                                               /* A/D sampling time for duty calculation */
#define     MTR_CARRIER_SET         ((uint16)((MTR_PWM_TIMER_FREQ * 1000 / MTR_CARRIER_FREQ / 2)+ MTR_DEADTIME_SET))
                                                               /* max setting value of carrier period */
#define     MTR_HALF_CARRIER_SET    ((uint16)(MTR_CARRIER_SET / 2))
                                                               /* half of duty value */

#define     MTR_PORT_UP             (PORT7.PODR.BIT.B1)        /* output port of PWM timer(Up) */
#define     MTR_PORT_UN             (PORT7.PODR.BIT.B4)        /* output port of PWM timer(Un) */
#define     MTR_PORT_VP             (PORT7.PODR.BIT.B2)        /* output port of PWM timer(Vp) */
#define     MTR_PORT_VN             (PORT7.PODR.BIT.B5)        /* output port of PWM timer(Vn) */
#define     MTR_PORT_WP             (PORT7.PODR.BIT.B3)        /* output port of PWM timer(Wp) */
#define     MTR_PORT_WN             (PORT7.PODR.BIT.B6)        /* output port of PWM timer(Wn) */

#define     MTR_PORT_LED1           (PORTD.PODR.BIT.B6)        /* output port of LED1 */
#define     MTR_PORT_LED2           (PORTD.PODR.BIT.B4)        /* output port of LED2 */

#define     MTR_LED_ON              (0)                        /* active level of LED */
#define     MTR_LED_OFF             (1)                        /* inactive level of LED */

#define     MTR_INPUT_V             (110*1.41421356)           /* input voltage [V] */
#define     MTR_IC_GATE_ON_V        ((int32)(MTR_INPUT_V*0.8f))
#define     MTR_HALF_VDC            (MTR_INPUT_V/2.0f)         /* vdc/2 */
#define     MTR_ADC_SCALING         (0x7FF)                    /* A/D offset */
#define     MTR_CURRENT_SCALING     (100.0f/4095.0f)           /* for current scaling */
#define     MTR_VDC_SCALING         (686.0f/4095.0f)           /* for vdc scaling */
#define     MTR_IPMTEMPERATURE_SCALING (5.0f/4095.0f)          /* for IPM temperature scaling */

#define     MTR_OVERCURRENT_LIMIT   (5.0f)                     /* over current limit [A] */
#define     MTR_OVERVOLTAGE_LIMIT   (420.0f)                   /* over voltage limit [V] */
#define     MTR_UNDERVOLTAGE_LIMIT  (0.0f)                     /* under voltage limit [V] */
#define     MTR_OVERIPMTEMPERATURE_LIMIT (3)                   /* over temperature limit [V] (3[V]=60Å}10[Åé]) */

#define     MTR_PORT_IC_GATE        (PORTB.PODR.BIT.B2)        /* output port of gate for preventing the inrush current */
#define     MTR_IC_GATE_ON          (1)                        /* active level of gate for preventing the inrush current */

#define     MTR_PORT_SW1            (PORTB.PIDR.BIT.B5)        /* input port of SW1 */
#define     MTR_PORT_SW2            (PORTB.PIDR.BIT.B4)        /* input port of SW2 */
#define     MTR_PORT_SW3            (PORT7.PIDR.BIT.B0)        /* input port of SW3 */

/******************************************************************************
* Global function definitions
******************************************************************************/
void    mtr_init_mtu(void);                                    /* initialize MTU3 */
void    mtr_init_io_port(void);                                /* initialize I/O port */
void    mtr_init_ad_converter(void);                           /* initialize A/D converter */
void    init_ui(void);                                         /* initialize peripheral function for user interface */
void    mtr_ctrl_start(void);                                  /* PWM output start for motor control start */
void    mtr_ctrl_stop(void);                                   /* PWM output disable for motor control stop */
uint16  mtr_get_vr1(void);                                     /* A/D conversion for VR1 */
void    mtr_get_iuiviwvdc(volatile float32 * , volatile float32 * , volatile float32 *, volatile float32 *);
                                                               /* A/D conversion for Iu,Iw,Vdc */
int16   mtr_get_ipm_temperature(void);                         /* A/D conversion for IPM temperature */
void    mtr_clear_mtu4_flag(void);                             /* clear MTU4 interrupt flag */
void    mtr_inv_set_uvw(float32, float32, float32);            /* PWM duty setting */
void    mtr_init_register(void);                               /* remove unintentional first pulse */

#endif /* MTR_CTRL_RX13T48_T1102_H */
