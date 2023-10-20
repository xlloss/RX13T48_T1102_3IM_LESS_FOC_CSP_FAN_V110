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
* File Name   : control_parameter.h
* Description : Definition of control parameters
******************************************************************************/
/******************************************************************************
* History : 28.05.2020  Ver.1.00
*         : 21.04.2021  Ver.1.10
******************************************************************************/

/* guard against multiple inclusion */
#ifndef CONTROL_PARAMETER_H
#define CONTROL_PARAMETER_H

/******************************************************************************
* Control Parameter Define
******************************************************************************/
#define     CP_ID_PI_KP                (0.05)                  /* proportional gain for gamma-axis current PI control */
#define     CP_ID_PI_KI                (0.05)                  /* integral gain for gamma-axis current PI control */
#define     CP_IQ_PI_KP                (0.5)                   /* proportional gain for delta-axis current PI control */
#define     CP_IQ_PI_KI                (0.5)                   /* integral gain for delta-axis current PI control */
#define     CP_SPEED_PI_KP             (0.0025)                /* proportional gain for speed PI control */
#define     CP_SPEED_PI_KI             (0.00005)               /* integral gain for speed PI control */
#define     CP_SPEED_LPF_K             (0.3)                   /* speed LPF gain */
#define     CP_CURRENT_LPF_K           (1.0)                   /* current LPF gain */
#define     CP_OFFSET_LPF_K            (0.1)                   /* current offset LPF gain */
#define     CP_MAX_SPEED_RPM           (1100)                  /* max speed [rpm] (mechanical angle) */
#define     CP_MIN_SPEED_RPM           (100)                   /* min speed [rpm] (mechanical angle) */
#define     CP_CTRL_REF_ID             (0.4f)                  /* gamma-axis current reference [A] */
#define     CP_BOOT_ID_UP_TIME         (100.0f)                /* rise time at gamma-axis current startup [ms] */
#define     CP_ID_CONST_TIME           (500.0f)                /* gamma-axis current/flux stabilization wait time [ms] */
#define     CP_ACCEL_MODE0             (0.1)                   /* speed command acceleration step [rad/s] */
#define     CP_FLUCTUATION_LIMIT       (200.0f)                /* speed fluctuation limit */
#define     CP_DELAY                   (2.0f)                  /* g_f4_angle_compensation = CP_DELAY * MTR_CTRL_PERIOD */
#define     CP_OFFSET_CALC_TIME        (256)                   /* current offset calculation time [ms] */
#define     CP_VOLTAGE_DROP            (8.0f)                  /* voltage drop caused by dead time and Vce [V] */
#define     CP_VOLTAGE_DROP_K          (0.0f)                  /* voltage drop corrective gain [V/A] */

#endif /* CONTROL_PARAMETER_H */

