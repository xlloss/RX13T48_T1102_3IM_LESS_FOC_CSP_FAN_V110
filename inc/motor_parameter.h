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
* File Name   : motor_parameter.h
* Description : Definition of the target motor parameters
******************************************************************************/
/******************************************************************************
* History : 28.05.2020  Ver.1.00
*         : 21.04.2021  Ver.1.10
******************************************************************************/

/* guard against multiple inclusion */
#ifndef MOTOR_PARAMETER_H
#define MOTOR_PARAMETER_H

/*******************************************************************************
 Oriental motor: MRS-25T(200V/50Hz/0.4A/70W)
********************************************************************************/
#define     MP_POLE_PAIRS                   (2)                      /* number of pole pairs */
#define     MP_STATOR_RESISTANCE            (10.0f)                  /* stator resistance[ohm] */
#define     MP_ROTOR_RESISTANCE             (5.5f)                  /* rotor resistance [ohm] */
#define     MP_MUTUAL_INDUCTANCE            (2.00f)                  /* mutual inductance [H] */
#define     MP_STATOR_LEAKAGE_INDUCTANCE    (0.080f)                 /* stator leakage inductance [H] */
#define     MP_ROTOR_LEAKAGE_INDUCTANCE     (0.080f)                 /* rotor leakage inductance [H] */

#endif /* MOTOR_PARAMETER_H */

