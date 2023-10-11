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
* File Name   : mtr_3im_less_foc.h
* Description : A definitions of the processing of a motor control layer
******************************************************************************/
/******************************************************************************
* History : 28.05.2020  Ver.1.00
*         : 21.04.2021  Ver.1.10
******************************************************************************/

/* guard against multiple inclusion */
#ifndef MTR_3IM_LESS_FOC_H
#define MTR_3IM_LESS_FOC_H

#include "motor_parameter.h"
#include "control_parameter.h"

/******************************************************************************
* Macro definitions
******************************************************************************/
#define     MTR_INT_DECIMATION           (1)                             /* interrupt skipping number */
#define     MTR_CTRL_PERIOD              ((MTR_INT_DECIMATION + 1)/(MTR_CARRIER_FREQ*1000))
                                                                         /* control period */
#define     MTR_CONTROL_FREQ             ((MTR_CARRIER_FREQ*1000)/(MTR_INT_DECIMATION + 1))
                                                                         /* control frequency [Hz] */

#define     MTR_POLE_PAIRS               (MP_POLE_PAIRS)                 /* number of pole pairs */
#define     MTR_RS                       (MP_STATOR_RESISTANCE)          /* stator resistance[ohm] */
#define     MTR_RR                       (MP_ROTOR_RESISTANCE)           /* rotor resistance [ohm] */
#define     MTR_M                        (MP_MUTUAL_INDUCTANCE)          /* mutual inductance [H] */
#define     MTR_LLS                      (MP_STATOR_LEAKAGE_INDUCTANCE)  /* stator leakage inductance [H] */
#define     MTR_LLR                      (MP_ROTOR_LEAKAGE_INDUCTANCE)   /* rotor leakage inductance [H] */
#define     MTR_LS                       (MTR_M + MTR_LLS)               /* stator inductance [H] */
#define     MTR_LR                       (MTR_M + MTR_LLR)               /* rotor inductance [H] */
#define     MTR_M_LR                     (MTR_M / MTR_LR)                /* M / LR */
#define     MTR_RR_LR                    (MTR_RR / MTR_LR)               /* RR / LR */
#define     MTR_SIGMA                    (1.0f - MTR_M / MTR_LS * MTR_M_LR)
                                                                         /* 1 - M * M / (LS * LR) */
#define     MTR_LS_SIGMA                 (MTR_LS * MTR_SIGMA)            /* LS * SIGMA */

#define     MTR_TWOPI                    (2*3.14159265f)                 /* 2*pi */
#define     MTR_TWOPI_3                  (MTR_TWOPI/3)                   /* 2*pi/3 */
#define     MTR_SQRT_2                   (1.41421356f)                   /* sqrt(2) */
#define     MTR_SQRT_3                   (1.7320508f)                    /* sqrt(3) */
#define     MTR_SQRT_2_3                 (0.81649658f)                   /* sqrt(2/3) */

#define     MTR_RPM_RAD                  (MTR_TWOPI/60)                  /* [rpm]->[rad/s] */

#define     MTR_ID_PI_KP                 (CP_ID_PI_KP)                   /* proportional gain for delta-axis current PI */
#define     MTR_ID_PI_KI                 (CP_ID_PI_KI)                   /* integral gain for delta-axiscurrent PI */
#define     MTR_IQ_PI_KP                 (CP_IQ_PI_KP)                   /* proportional gain for delta-axis current PI */
#define     MTR_IQ_PI_KI                 (CP_IQ_PI_KI)                   /* integral gain for delta-axiscurrent PI */
#define     MTR_SPEED_PI_KP              (CP_SPEED_PI_KP)                /* proportional gain for speed PI */
#define     MTR_SPEED_PI_KI              (CP_SPEED_PI_KI)                /* integral gain for speed PI */
#define     MTR_SPEED_LPF_K              (CP_SPEED_LPF_K)                /* speed LPF gain */
#define     MTR_CURRENT_LPF_K            (CP_CURRENT_LPF_K)              /* current LPF gain */
#define     MTR_OFFSET_LPF_K             (CP_OFFSET_LPF_K)               /* current offset LPF gain */

#define     MTR_LIMIT_ID                 (5.0)                           /* gamma-axis current limit */
#define     MTR_I_LIMIT_ID               (5.0)                           /* gamma-axis current PI integral term limit */

#define     MTR_LIMIT_IQ                 (5.0)                           /* delta-axis current limit */
#define     MTR_I_LIMIT_IQ               (5.0)                           /* speed PI integral term limit for delta-axis current */

#define     MTR_MAX_SPEED_RPM            (CP_MAX_SPEED_RPM)              /* max speed [rpm] (mechanical angle) */
#define     MTR_MAX_SPEED_RAD            (MTR_MAX_SPEED_RPM*MTR_POLE_PAIRS*MTR_TWOPI/60)
                                                                         /* max speed [rad/s] */
#define     MTR_MIN_SPEED_RPM            (CP_MIN_SPEED_RPM)              /* min speed [rpm] (mechanical angle) */
#define     MTR_MIN_SPEED_RAD            (MTR_MIN_SPEED_RPM*MTR_POLE_PAIRS*MTR_TWOPI/60)
                                                                         /* min speed [rad/s] */

#define     MTR_SPEED_LIMIT              (MTR_MAX_SPEED_RAD*1.3)         /* over speed limit [rad/s] */

#define     MTR_LIMIT_ROTOR_SPEED_RAD    (MTR_MAX_SPEED_RAD*1.2)         /* rotor_speed limit [rad/s] */
#define     MTR_I_LIMIT_ROTOR_SPEED_RAD  (MTR_MAX_SPEED_RAD*1.2)         /* current PI integral term limit for rotor_speed */

#define     MTR_CTRL_REF_ID              (CP_CTRL_REF_ID)                /* gamma-axis current reference [A] */
#define     MTR_BOOT_ID_UP_TIME          (CP_BOOT_ID_UP_TIME)            /* rise time at gamma-axis current startup [ms] */
#define     MTR_BOOT_ID_UP_STEP          (CP_CTRL_REF_ID/MTR_BOOT_ID_UP_TIME)
                                                                         /* addition value of gamma-axis current */
#define     MTR_ID_CONST_TIME            (CP_ID_CONST_TIME)              /* gamma-axis current/flux stabilization wait time [ms] */

#define     MTR_ACCEL_MODE0              (CP_ACCEL_MODE0)                /* speed command acceleration step [rad/s] */

#define     MTR_FLUCTUATION_LIMIT        (CP_FLUCTUATION_LIMIT)          /* speed fluctuation limit */

#define     MTR_DELAY                    (CP_DELAY)                      /* g_f4_angle_compensation = MTR_DELAY * MTR_CTRL_PERIOD */
#define     MTR_ANGLE_COMPENSATION       (MTR_DELAY * MTR_CTRL_PERIOD)

#define     MTR_OFFSET_CALC_TIME         (CP_OFFSET_CALC_TIME)           /* current offset calculation time */

#define     MTR_VOLTAGE_DROP             (CP_VOLTAGE_DROP)               /* voltage drop caused by dead time and Vce [V] */
#define     MTR_VOLTAGE_DROP_K           (CP_VOLTAGE_DROP_K)             /* voltage drop corrective gain [V/A] */

#define     MTR_EVERY_TIME               (0)                             /* calculate current offset every time */
#define     MTR_ONE_TIME                 (1)                             /* calculate current offset only one time */

#define     MTR_CW                       (0)
#define     MTR_CCW                      (1)

#define     MTR_FLG_CLR                  (0)                             /* for flag clear */
#define     MTR_FLG_SET                  (1)                             /* for flag set */

/* Id reference status */
#define     MTR_ID_UP                    (0)
#define     MTR_ID_CONST                 (1)
#define     MTR_ID_CONST_CTRL            (2)

/* Iq reference status */
#define     MTR_IQ_ZERO                  (0)
#define     MTR_IQ_SPEED_PI_OUTPUT       (1)

/* speed reference status */
#define     MTR_SPEED_ZERO               (0)
#define     MTR_SPEED_CHANGE             (1)

/******************************************************************************
* Macro definitions for sequence control
******************************************************************************/
/* run mode */
#define     MTR_BOOT_MODE                   (0x00)
#define     MTR_START_MODE                  (0x01)
#define     MTR_CTRL_MODE                   (0x02)

/* control mode */
#define     MTR_ZERO_PEC_MODE               (0x00)
#define     MTR_OPENLOOP_MODE               (0x01)
#define     MTR_HALL_120_MODE               (0x02)
#define     MTR_LESS_120_MODE               (0x03)
#define     MTR_ENCD_FOC_MODE               (0x04)
#define     MTR_LESS_FOC_MODE               (0x05)

/* error status */
#define     MTR_OVER_CURRENT_ERROR          (0x01)
#define     MTR_OVER_VOLTAGE_ERROR          (0x02)
#define     MTR_OVER_SPEED_ERROR            (0x03)
#define     MTR_TIMEOUT_ERROR               (0x04)
#define     MTR_UNDER_VOLTAGE_ERROR         (0x07)
#define     MTR_OVER_IPMTEMPERATURE_ERROR   (0x08)
#define     MTR_UNKNOWN_ERROR               (0xff)

/* system mode */
#define     MTR_MODE_STOP                   (0x00)
#define     MTR_MODE_RUN                    (0x01)
#define     MTR_MODE_ERROR                  (0x02)
#define     MTR_SIZE_STATE                  (3)                          /* number of state */

/* event */
#define     MTR_EVENT_STOP                  (0x00)
#define     MTR_EVENT_RUN                   (0x01)
#define     MTR_EVENT_ERROR                 (0x02)
#define     MTR_EVENT_RESET                 (0x03)
#define     MTR_SIZE_EVENT                  (4)                          /* number of event */

typedef struct
{
    float32 f4_err;
    float32 f4_kp;
    float32 f4_ki;
    float32 f4_limit;
    float32 f4_refi;
    float32 f4_ilimit;
} MTR_PI_CTRL;

typedef struct
{
    float32 f4_mtr_rs;
    float32 f4_mtr_rr;
    float32 f4_mtr_m;
    float32 f4_mtr_ls;
    float32 f4_mtr_lr;
    float32 f4_mtr_m_lr;
    float32 f4_mtr_rr_lr;
    float32 f4_mtr_sigma;
    float32 f4_mtr_ls_sigma;
} MTR_PARAMETER;

typedef struct
{
    int16   s2_ref_speed;
    int16   s2_direction;
    float32 f4_kp_speed;
    float32 f4_ki_speed;
    float32 f4_kp_iq;
    float32 f4_ki_iq;
    float32 f4_speed_lpf_k;
    float32 f4_current_lpf_k;
    float32 f4_mtr_rs;
    float32 f4_mtr_rr;
    float32 f4_mtr_m;
    float32 f4_mtr_lls;
    float32 f4_mtr_llr;
    float32 f4_offset_lpf_k;
    int16   s2_max_speed;
    int16   s2_min_speed;
    float32 f4_ctrl_ref_id;
    float32 f4_boot_id_up_time;
    float32 f4_id_const_time;
    float32 f4_accel;
    float32 f4_fluctuation_limit;
    float32 f4_delay;
    float32 f4_offset_calc_time;
    float32 f4_voltage_drop;
    float32 f4_voltage_drop_k;
} MTR_ICS_INPUT;

/******************************************************************************
* Global function definitions
******************************************************************************/
void        R_MTR_IcsInput(MTR_ICS_INPUT *);            /* set value to buffer */
float32     R_MTR_GetSpeed(void);                       /* get speed value */
void        R_MTR_SetDir(uint8);                        /* set rotation direction */
uint8       R_MTR_GetDir(void);                         /* get rotation direction */
uint8       R_MTR_GetStatus(void);                      /* get status of motor control system */
void        mtr_error_check(void);                      /* check error status */
float32     mtr_pi_ctrl(volatile MTR_PI_CTRL *);        /* PI control */
void        mtr_set_variables(void);                    /* set variables */
void        mtr_set_id_ref(void);                       /* set id reference */
void        mtr_set_iq_ref(void);                       /* set iq reference */
void        mtr_set_speed_ref(void);                    /* set speed reference */
void        mtr_calc_mod(float32, float32, float32, float32);
                                                        /* calculate modulation */
float32     copysignf(float32, float32);

/* for sequence control */
void        R_MTR_InitSequence(void);                   /* initialize variable for sequence control */
void        R_MTR_ExecEvent(uint8);                     /* change motor state and select action */

#endif /* MTR_3IM_LESS_FOC_H */
