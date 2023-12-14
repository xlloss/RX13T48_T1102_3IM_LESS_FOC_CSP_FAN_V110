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

/*******************************************************************************
* File Name   : main.h
* Description : A definitions of the processing of an application layer
******************************************************************************/
/******************************************************************************
* History : 28.05.2020  Ver.1.00
*         : 21.04.2021  Ver.1.10
******************************************************************************/

/* guard against multiple inclusion */
#ifndef MAIN_H
#define MAIN_H

#include "motor_parameter.h"
#include "control_parameter.h"

/******************************************************************************
* Macro definitions
******************************************************************************/
#define MAX_SPEED                 (CP_MAX_SPEED_RPM)             /* max speed [rpm] (mechanical angle) */
#define MIN_SPEED                 (CP_MIN_SPEED_RPM)             /* min speed [rpm] (mechanical angle) */

#define IQ_PI_KP                  (CP_IQ_PI_KP)                  /* proportional gain for delta-axis current PI control */
#define IQ_PI_KI                  (CP_IQ_PI_KI)                  /* integral gain for delta-axis current PI control */
#define SPEED_PI_KP               (CP_SPEED_PI_KP)               /* proportional gain for speed PI control */
#define SPEED_PI_KI               (CP_SPEED_PI_KI)               /* integral gain for speed PI control */
#define SPEED_LPF_K               (CP_SPEED_LPF_K)               /* speed LPF gain */
#define CURRENT_LPF_K             (CP_CURRENT_LPF_K)             /* current LPF gain */
#define STATOR_RESISTANCE         (MP_STATOR_RESISTANCE)         /* stator resistance[ohm] */
#define ROTOR_RESISTANCE          (MP_ROTOR_RESISTANCE)          /* rotor resistance [ohm] */
#define MUTUAL_INDUCTANCE         (MP_MUTUAL_INDUCTANCE)         /* mutual inductance [H] */
#define STATOR_LEAKAGE_INDUCTANCE (MP_STATOR_LEAKAGE_INDUCTANCE) /* stator leakage inductance [H] */
#define ROTOR_LEAKAGE_INDUCTANCE  (MP_ROTOR_LEAKAGE_INDUCTANCE)  /* rotor leakage inductance [H] */
#define OFFSET_LPF_K              (CP_OFFSET_LPF_K)              /* current offset LPF gain */
#define CTRL_REF_ID               (CP_CTRL_REF_ID)               /* gamma-axis current reference [A] */
#define BOOT_ID_UP_TIME           (CP_BOOT_ID_UP_TIME)           /* rise time at gamma-axis current startup [ms] */
#define ID_CONST_TIME             (CP_ID_CONST_TIME)             /* gamma-axis current/flux stabilization wait time [ms] */
#define ACCEL_MODE0               (CP_ACCEL_MODE0)               /* speed command acceleration step [rad/s] */
#define FLUCTUATION_LIMIT         (CP_FLUCTUATION_LIMIT)         /* speed fluctuation limit */
#define DELAY                     (CP_DELAY)                     /* g_f4_angle_compensation = DELAY * MTR_CTRL_PERIOD */
#define OFFSET_CALC_TIME          (CP_OFFSET_CALC_TIME)          /* current offset calculation time */
#define VOLTAGE_DROP              (CP_VOLTAGE_DROP)              /* voltage drop caused by dead time and Vce [V] */
#define VOLTAGE_DROP_K            (CP_VOLTAGE_DROP_K)            /* voltage drop corrective gain [V/A] */
#define POLE_PAIRS                (MP_POLE_PAIRS)                /* pole pairs */

#define M_CW                      (0)                            /* CW */
#define M_CCW                     (1)                            /* CCW */
#define ICS_INT_LEVEL             (6)                            /* interrupt priority level */

#define SW_ON                     (1)                            /* switch ON  state */
#define SW_OFF                    (0)                            /* switch OFF state */

#define VR1_SCALING               (2500.0f/4095.0f)              /* for ref speed scaling */

#define ADC_WORKAROUND            1
#define VDC_AD_K_0 0.13           /* current 2.10 AMP */
#define VDC_AD_K_1 0.08           /* current 1.5 AMP */
#define VDC_AD_K_2 0.05           /* current 0.5 AMP */
#define SLOW_START_TIME           6000
#define BOOT_DELAY_TIME           1000
/******************************************************************************
* Private global functions
******************************************************************************/
void       main(void);                                     /* main function */

extern uint8    mp_pole_pairs;
extern uint8    mtr_pole_pairs;
extern float32  mtr_max_speed_rad;
extern float32  mtr_min_speed_rad;
extern float32  mtr_speed_limit;
extern float32  mtr_limit_rotor_speed_rad;
extern float32  mtr_i_limit_rotor_speed_rad;


#endif /* MAIN_H */
