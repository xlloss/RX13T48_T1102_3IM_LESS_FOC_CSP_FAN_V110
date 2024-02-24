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
* File Name   : mtr_3im_less_foc.c
* Description : The processing of a motor control layer
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : 28.05.2020  Ver.1.00
*         : 21.04.2021  Ver.1.10
***********************************************************************************************************************/

/***********************************************************************************************************************
* Includes <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include <machine.h>
#include <math.h>
#include "mtr_common.h"
#include "main.h"
#include "mtr_3im_less_foc.h"
#include "mtr_ctrl_rx13t48_t1102.h"

/******************************************************************************
* Exported global variables (to be accessed by other files)
******************************************************************************/
/* for sequence control */
volatile uint8    g_u1_mode_system;                 /* system mode */
volatile uint16   g_u2_run_mode;                    /* run mode */
volatile uint16   g_u2_ctrl_mode;                   /* control mode */
volatile uint8    g_u1_error_status;                /* error status */

/****** for ICS ******/
volatile uint8    g_u1_cnt_ics;                     /* counter for period of calling "ics_watchpoint" */
/*********************/

volatile float32  g_f4_vdc_ad;                      /* inverter bus voltage */
volatile float32  g_f4_vd_ref;                      /* gamma-axis output voltage command */
volatile float32  g_f4_vq_ref;                      /* delta-axis output voltage command */
volatile float32  g_f4_iu_ad;                       /* U phase current */
volatile float32  g_f4_pre_iu_ad;                   /* Previous U phase current */
volatile float32  g_f4_iv_ad;                       /* V phase current */
volatile float32  g_f4_pre_iv_ad;                   /* Previous V phase current */
volatile float32  g_f4_iw_ad;                       /* W phase current */
volatile float32  g_f4_pre_iw_ad;                   /* Previous W phase current */
volatile float32  g_f4_offset_iu;                   /* U phase current offset */
volatile float32  g_f4_offset_iv;                   /* V phase current offset */
volatile float32  g_f4_offset_iw;                   /* W phase current offset */
volatile float32  g_f4_id_lpf;                      /* gamma-axis current */
volatile float32  g_f4_iq_lpf;                      /* delta-axis current */
volatile float32  g_f4_pre_id_lpf;                  /* Previous gamma-axis current */
volatile float32  g_f4_pre_iq_lpf;                  /* Previous delta-axis current */
volatile float32  g_f4_kp_id;                       /* proportional gain for gamma-axis current PI control */
volatile float32  g_f4_ki_id;                       /* integral gain for gamma-axis current PI control */
volatile float32  g_f4_lim_id;                      /* gamma-axis current PI control limit value */
volatile float32  g_f4_ilim_id;                     /* gamma-axis current PI control integral limit value */
volatile float32  g_f4_kp_iq;                       /* proportional gain for delta-axis current PI control */
volatile float32  g_f4_ki_iq;                       /* integral gain for delta-axis current PI control */
volatile float32  g_f4_lim_rotor_speed_rad;         /* rotor speed PI control limit value */
volatile float32  g_f4_ilim_rotor_speed_rad;        /* rotor speed PI control integral limit value */
volatile float32  g_f4_id_ref;                      /* gamma-axis current command */
volatile float32  g_f4_id_ref2;                     /* gamma-axis current command */
volatile float32  g_f4_iq_ref;                      /* delta-axis current command */
volatile float32  g_f4_ref_stator_speed_rad;        /* stator speed command */
volatile float32  g_f4_slip_speed_rad;              /* slip speed */
volatile float32  g_f4_slip_k;                      /* slip speed gain */
volatile float32  g_f4_speed_rad;                   /* calculated speed value */
volatile float32  g_f4_ref_speed_rad_pi;            /* speed command for PI control */
volatile float32  g_f4_ref_speed_rad;               /* speed command */
volatile float32  g_f4_angle_rad;                   /* stator interlinkage flux angle */
volatile float32  g_f4_refu;                        /* U phase voltage command */
volatile float32  g_f4_refv;                        /* V phase voltage command */
volatile float32  g_f4_refw;                        /* W phase voltage command */
volatile float32  g_f4_inv_limit;                   /* voltage limit */
volatile float32  g_f4_speed_lpf_k;                 /* speed LPF gain */
volatile float32  g_f4_current_lpf_k;               /* current LPF gain */
volatile float32  g_f4_offset_lpf_k;                /* current offset LPF gain */
volatile uint8    g_u1_direction;                   /* rotational direction (0:CW ,1:CCW) */
volatile uint8    g_u1_enable_write;                /* ICS write enable flag */
volatile uint16   g_u2_cnt_adjust;                  /* counter for current offset calculation */
volatile uint8    g_u1_flag_id_ref;                 /* gamma-axis current command management flag */
volatile float32  g_f4_temp_speed_rad;              /* for holding speed value */
volatile float32  g_f4_temp_ref_speed_rad;          /* for holding speed command value */
volatile float32  g_f4_angle_compensation;          /* angle compensation gain */
volatile float32  g_f4_offset_calc_time;            /* current offset calculation time */
volatile float32  g_f4_voltage_drop;                /* voltage drop */
volatile float32  g_f4_voltage_drop_k;              /* voltage drop correction gain */
volatile float32  g_f4_modu;                        /* U phase modulation factor */
volatile float32  g_f4_modv;                        /* V phase modulation factor */
volatile float32  g_f4_modw;                        /* W phase modulation factor */
volatile MTR_PI_CTRL rotor_speed;                   /* PI control structure */
volatile MTR_PI_CTRL id_ACR;                        /* PI control structure */
volatile MTR_PARAMETER mtr_p;                       /* motor parameters and control parameters structure */

/******************************************************************************
* Private global variables and functions
******************************************************************************/
volatile float32  g_f4_kp_speed;                    /* proportional gain for speed PI control */
volatile float32  g_f4_ki_speed;                    /* integral gain for speed PI control */
volatile float32  g_f4_lim_iq;                      /* delta-axis current PI control limit value */
volatile float32  g_f4_ilim_iq;                     /* delta-axis current PI control integral limit value */
volatile float32  g_f4_max_speed_rad;               /* maximum speed */
volatile float32  g_f4_min_speed_rad;               /* minimum speed */
volatile uint8    g_u1_dir_buff;                    /* rotational direction command */
volatile float32  g_f4_id_ref_buff;                 /* gamma-axis current command buffer */
volatile float32  g_f4_iq_ref_buff;                 /* delta-axis current command buffer */
volatile uint8    g_u1_flag_iq_ref;                 /* delta-axis current command management flag */
volatile uint8    g_u1_flag_offset_calc;            /* current offset calculation flag */
volatile uint8    g_u1_flag_speed_ref;              /* speed command management flag */
volatile float32  g_f4_boot_id_up_step;             /* gamma-axis current step at startup */
volatile float32  g_f4_fluctuation_limit;           /* speed fluctuation limit */
volatile float32  g_f4_ctrl_ref_id;                 /* gamma-axis current command */
volatile uint16   g_u2_cnt_id_const;                /* gamma-axis current and flux stabilization wait time counter */
volatile float32  g_f4_id_const_time;               /* gamma-axis current and flux stabilization wait time */
volatile float32  g_f4_accel;                       /* speed command acceleration step */
volatile float32  g_f4_ipm_temperature_ad;          /* IPM temperature voltage conversion value */
volatile MTR_PI_CTRL speed;                         /* PI control structure */
volatile MTR_ICS_INPUT ics_input_buff;              /* struct for ICS input */

static void     mtr_start_init(void);
static void     mtr_stop_init(void);

/* for sequence control */
static uint8      mtr_act_run(uint8 u1_state);
static uint8      mtr_act_stop(uint8 u1_state);
static uint8      mtr_act_reset(uint8 u1_state);
static uint8      mtr_act_error(uint8 u1_state);
static uint8      mtr_act_none(uint8 u1_state);

extern uint16     boot_slow_start;
extern float32    vdc_ad_k;
extern float32    vr1_to_rpm_old;
extern uint8      motor_dir_old;
extern uint8      com_u1_ics_control_enable;
static const volatile uint8 g_u1_def_state[MTR_SIZE_EVENT][MTR_SIZE_STATE] = {
/*    state              0:MTR_MODE_STOP, 1:MTR_MODE_RUN,  2:MTR_MODE_ERROR */
/* event   */
/* 0:MTR_EVENT_STOP  */{ MTR_MODE_STOP,   MTR_MODE_STOP,   MTR_MODE_ERROR },
/* 1:MTR_EVENT_RUN   */{ MTR_MODE_RUN,    MTR_MODE_RUN,    MTR_MODE_ERROR },
/* 2:MTR_EVENT_ERROR */{ MTR_MODE_ERROR,  MTR_MODE_ERROR,  MTR_MODE_ERROR },
/* 3:MTR_EVENT_RESET */{ MTR_MODE_STOP,   MTR_MODE_ERROR,  MTR_MODE_STOP  },};

static uint8 (* gp_u1_def_action[MTR_SIZE_EVENT][MTR_SIZE_STATE])(uint8 u1_state) = {
/*    state              0:MTR_MODE_STOP, 1:MTR_MODE_RUN,  2:MTR_MODE_ERROR */
/* event */
/* 0:MTR_EVENT_STOP  */{ mtr_act_none,    mtr_act_stop,    mtr_act_none  },
/* 1:MTR_EVENT_RUN   */{ mtr_act_run,     mtr_act_none,    mtr_act_none  },
/* 2:MTR_EVENT_ERROR */{ mtr_act_error,   mtr_act_error,   mtr_act_none  },
/* 3:MTR_EVENT_RESET */{ mtr_act_reset,   mtr_act_error,   mtr_act_reset },};

/******************************************************************************
* Function Name : R_MTR_InitSequence
* Description   : Initialize variable for sequence control
* Arguments     : none
* Return Value  : none
******************************************************************************/
void R_MTR_InitSequence(void)
{
    g_u1_mode_system = MTR_MODE_STOP;               /* system mode initialize : stop mode */
}

/******************************************************************************
* Function Name : R_MTR_ExecEvent
* Description   : Change motor state and select action
* Arguments     : u1_event - event
* Return Value  : none
******************************************************************************/
void R_MTR_ExecEvent(uint8 u1_event)
{
    uint8 u1_mode_system;

    u1_mode_system = g_u1_mode_system;

    if (u1_event >= MTR_SIZE_EVENT)                 /* check unknown event */
    {
        g_u1_mode_system = MTR_MODE_ERROR;          /* system mode update : error mode */
        g_u1_error_status = MTR_UNKNOWN_ERROR;      /* error status update : unknown error */
    }
    else if (u1_mode_system >= MTR_SIZE_STATE)      /* check unknown state */
    {
        g_u1_mode_system = MTR_MODE_ERROR;          /* system mode update : error mode */
        g_u1_error_status = MTR_UNKNOWN_ERROR;      /* error status update : unknown error */
    }
    else
    {
        /* change motor state and select action */
        g_u1_mode_system = (gp_u1_def_action[u1_event][u1_mode_system])(g_u1_def_state[u1_event][u1_mode_system]);
    }
}

/******************************************************************************
* Function Name : mtr_act_run
* Description   : Processing for motor control start
* Arguments     : u1_state - state
* Return Value  : u1_state - state
******************************************************************************/
static uint8 mtr_act_run(uint8 u1_state)
{
    mtr_start_init();                               /* initialize variables when motor control start */
    mtr_ctrl_start();                               /* PWM output enable */

    if (g_u1_mode_system == MTR_MODE_ERROR)         /* check system mode */
    {
        u1_state = MTR_MODE_ERROR;                  /* system mode update : error mode */
    }

    return (u1_state);
}

/******************************************************************************
* Function Name : mtr_act_stop
* Description   : Processing for motor control stop
* Arguments     : u1_state - state
* Return Value  : u1_state - state
******************************************************************************/
static uint8 mtr_act_stop(uint8 u1_state)
{
    mtr_ctrl_stop();                                /* PWM output disable */
    mtr_init_register();                            /* remove unintentional first pulse */
    mtr_stop_init();                                /* initialize variables when motor control stop */

    return (u1_state);
}

/******************************************************************************
* Function Name : mtr_act_none
* Description   : Do nothing
* Arguments     : u1_state - state
* Return Value  : u1_state - state
******************************************************************************/
static uint8 mtr_act_none(uint8 u1_state)
{
    return (u1_state);
}

/******************************************************************************
* Function Name : mtr_act_reset
* Description   : Processing for motor control reset
* Arguments     : u1_state - state
* Return Value  : u1_state - state
******************************************************************************/
static uint8 mtr_act_reset(uint8 u1_state)
{
    g_u1_mode_system = MTR_MODE_STOP;
    g_u2_run_mode = MTR_BOOT_MODE;
    g_u2_ctrl_mode = MTR_OPENLOOP_MODE;
    g_u1_error_status = 0;
    boot_slow_start = SLOW_START_TIME;
    vdc_ad_k        = VDC_AD_K_2;
    vr1_to_rpm_old = 0;
    motor_dir_old = 0;
    com_u1_ics_control_enable = 0;

/****** for ICS ******/
    g_u1_cnt_ics = 0;
/*********************/

    g_f4_vdc_ad = 0;
    g_f4_vd_ref = 0;
    g_f4_vq_ref = 0;
    g_f4_iu_ad = 0;
    g_f4_pre_iu_ad = 0;
    g_f4_iv_ad = 0;
    g_f4_pre_iv_ad = 0;
    g_f4_iw_ad = 0;
    g_f4_pre_iw_ad = 0;
    g_f4_offset_iu = 0;
    g_f4_offset_iv = 0;
    g_f4_offset_iw = 0;
    g_f4_id_lpf = 0;
    g_f4_iq_lpf = 0;
    g_f4_pre_id_lpf = 0;
    g_f4_pre_iq_lpf = 0;
    g_f4_kp_id = MTR_ID_PI_KP;
    g_f4_ki_id = MTR_ID_PI_KI;
    g_f4_kp_iq = MTR_IQ_PI_KP;
    g_f4_ki_iq = MTR_IQ_PI_KI;
    g_f4_kp_speed = MTR_SPEED_PI_KP;
    g_f4_ki_speed = MTR_SPEED_PI_KI;
    g_f4_lim_id = MTR_LIMIT_ID;
    g_f4_lim_rotor_speed_rad = MTR_LIMIT_ROTOR_SPEED_RAD;
    g_f4_lim_iq = MTR_LIMIT_IQ;
    g_f4_ilim_id = MTR_I_LIMIT_ID;
    g_f4_ilim_rotor_speed_rad = MTR_I_LIMIT_ROTOR_SPEED_RAD;
    g_f4_ilim_iq = MTR_I_LIMIT_IQ;
    g_f4_id_ref = 0;
    g_f4_id_ref2 = 0;
    g_f4_iq_ref = 0;
    g_f4_ref_stator_speed_rad = 0;
    g_f4_slip_speed_rad = 0;
    g_f4_slip_k = 0;
    g_f4_speed_rad = 0;
    g_f4_ref_speed_rad_pi = 0;
    g_f4_ref_speed_rad = 0;
    g_f4_angle_rad = 0;
    g_f4_max_speed_rad = MTR_MAX_SPEED_RAD;
    g_f4_min_speed_rad = MTR_MIN_SPEED_RAD;
    g_f4_refu = 0;
    g_f4_refv = 0;
    g_f4_refw = 0;
    g_f4_inv_limit = MTR_HALF_VDC;
    g_f4_speed_lpf_k = MTR_SPEED_LPF_K;
    g_f4_current_lpf_k = MTR_CURRENT_LPF_K;
    g_f4_offset_lpf_k = MTR_OFFSET_LPF_K;
    g_u1_direction = 0;
    g_u1_dir_buff = 0;
    g_u1_enable_write = 0;
    g_u2_cnt_adjust = 0;
    g_f4_id_ref_buff = 0;
    g_f4_iq_ref_buff = 0;
    g_u1_flag_id_ref = MTR_ID_UP;
    g_u1_flag_iq_ref = MTR_IQ_ZERO;
    g_u1_flag_speed_ref = MTR_SPEED_ZERO;
    g_f4_temp_speed_rad = 0;
    g_f4_temp_ref_speed_rad = 0;
    g_f4_angle_compensation = MTR_ANGLE_COMPENSATION;
    g_f4_offset_calc_time = MTR_OFFSET_CALC_TIME;
    g_f4_accel = MTR_ACCEL_MODE0;
    g_f4_voltage_drop = MTR_VOLTAGE_DROP;
    g_f4_voltage_drop_k = MTR_VOLTAGE_DROP_K;
    g_f4_modu = 0;
    g_f4_modv = 0;
    g_f4_modw = 0;
    id_ACR.f4_err = 0;
    id_ACR.f4_kp = g_f4_kp_id;
    id_ACR.f4_ki = g_f4_ki_id;
    id_ACR.f4_limit = g_f4_lim_id;
    id_ACR.f4_refi = 0;
    id_ACR.f4_ilimit = g_f4_ilim_id;
    rotor_speed.f4_err = 0;
    rotor_speed.f4_kp = g_f4_kp_iq;
    rotor_speed.f4_ki = g_f4_ki_iq;
    rotor_speed.f4_limit = g_f4_lim_rotor_speed_rad;
    rotor_speed.f4_refi = 0;
    rotor_speed.f4_ilimit = g_f4_ilim_rotor_speed_rad;
    speed.f4_err = 0;
    speed.f4_kp = g_f4_kp_speed;
    speed.f4_ki = g_f4_ki_speed;
    speed.f4_limit = g_f4_lim_iq;
    speed.f4_refi = 0;
    speed.f4_ilimit = g_f4_ilim_iq;
    mtr_p.f4_mtr_rs = MTR_RS;
    mtr_p.f4_mtr_rr = MTR_RR;
    mtr_p.f4_mtr_m = MTR_M;
    mtr_p.f4_mtr_ls = MTR_LS;
    mtr_p.f4_mtr_lr = MTR_LR;
    mtr_p.f4_mtr_m_lr = MTR_M_LR;
    mtr_p.f4_mtr_rr_lr = MTR_RR_LR;
    mtr_p.f4_mtr_sigma = MTR_SIGMA;
    mtr_p.f4_mtr_ls_sigma = MTR_LS_SIGMA;

    g_u1_flag_offset_calc = MTR_EVERY_TIME;
    g_f4_boot_id_up_step = MTR_BOOT_ID_UP_STEP;
    g_f4_fluctuation_limit = MTR_FLUCTUATION_LIMIT;
    g_f4_ctrl_ref_id = MTR_CTRL_REF_ID;
    g_u2_cnt_id_const = 0;
    g_f4_id_const_time = MTR_ID_CONST_TIME;
    g_f4_ipm_temperature_ad = 0;
    ics_input_buff.s2_ref_speed = 0;
    ics_input_buff.s2_direction = 0;
    ics_input_buff.f4_kp_speed = 0;
    ics_input_buff.f4_ki_speed = 0;
    ics_input_buff.f4_kp_iq = 0;
    ics_input_buff.f4_ki_iq = 0;
    ics_input_buff.f4_speed_lpf_k = 0;
    ics_input_buff.f4_current_lpf_k = 0;
    ics_input_buff.f4_mtr_rs = 0;
    ics_input_buff.f4_mtr_rr = 0;
    ics_input_buff.f4_mtr_m = 0;
    ics_input_buff.f4_mtr_lls = 0;
    ics_input_buff.f4_mtr_llr = 0;
    ics_input_buff.f4_offset_lpf_k = 0;
    ics_input_buff.s2_max_speed = 0;
    ics_input_buff.s2_min_speed = 0;
    ics_input_buff.f4_ctrl_ref_id = 0;
    ics_input_buff.f4_boot_id_up_time = 0;
    ics_input_buff.f4_id_const_time = 0;
    ics_input_buff.f4_accel = 0;
    ics_input_buff.f4_fluctuation_limit = 0;
    ics_input_buff.f4_delay = 0;
    ics_input_buff.f4_offset_calc_time = 0;
    ics_input_buff.f4_voltage_drop = 0;
    ics_input_buff.f4_voltage_drop_k = 0;

    return (u1_state);
}

/******************************************************************************
* Function Name : mtr_act_error
* Description   : Processing when detecting an error
* Arguments     : u1_state - state
* Return Value  : u1_state - state
******************************************************************************/
static uint8 mtr_act_error(uint8 u1_state)
{
    mtr_ctrl_stop();                                /* PWM output disable */
    mtr_init_register();                            /* remove unintentional first pulse */

    return (u1_state);
}

/******************************************************************************
* Function Name : mtr_start_init
* Description   : Initialize variables when motor control start
* Arguments     : none
* Return Value  : none
******************************************************************************/
static void mtr_start_init(void)
{
    g_u2_cnt_adjust = 0;
    if (MTR_ONE_TIME == g_u1_flag_offset_calc)
    {
        if (MTR_BOOT_MODE != g_u2_run_mode)
        {
            g_u2_cnt_adjust = g_f4_offset_calc_time;
        }
    }

    g_u2_run_mode = MTR_BOOT_MODE;
    g_u2_ctrl_mode = MTR_OPENLOOP_MODE;
    g_f4_id_ref_buff = 0;
    g_f4_iq_ref_buff = 0;
    g_f4_vd_ref = 0;
    g_f4_vq_ref = 0;
    g_f4_id_ref = 0;
    g_f4_id_ref2 = 0;
    g_f4_iq_ref = 0;
    g_f4_ref_stator_speed_rad = 0;
    g_f4_slip_speed_rad = 0;
    g_f4_slip_k = 0;
    id_ACR.f4_refi = 0;
    rotor_speed.f4_refi = 0;
    speed.f4_refi = 0;
    g_f4_refu = 0;
    g_f4_refv = 0;
    g_f4_refw = 0;
    g_f4_speed_rad = 0;
    g_f4_ref_speed_rad_pi = 0;
    g_f4_angle_rad = 0;
    g_u1_flag_id_ref = MTR_ID_UP;
    g_u1_flag_iq_ref = MTR_IQ_ZERO;
    g_u1_flag_speed_ref = MTR_SPEED_ZERO;
    g_f4_modu = 0;
    g_f4_modv = 0;
    g_f4_modw = 0;
    g_u2_cnt_id_const = 0;
}

/******************************************************************************
* Function Name : mtr_stop_init
* Description   : Initialize variables when motor control stop
* Arguments     : none
* Return Value  : none
******************************************************************************/
static void mtr_stop_init(void)
{
    g_u2_run_mode = MTR_BOOT_MODE;
    g_u2_ctrl_mode = MTR_OPENLOOP_MODE;
    g_f4_id_ref_buff = 0;
    g_f4_iq_ref_buff = 0;
    g_f4_vd_ref = 0;
    g_f4_vq_ref = 0;
    g_f4_id_ref = 0;
    g_f4_id_ref2 = 0;
    g_f4_iq_ref = 0;
    g_f4_ref_stator_speed_rad = 0;
    g_f4_slip_speed_rad = 0;
    g_f4_slip_k = 0;
    id_ACR.f4_refi = 0;
    rotor_speed.f4_refi = 0;
    speed.f4_refi = 0;
    g_f4_refu = 0;
    g_f4_refv = 0;
    g_f4_refw = 0;
    g_f4_speed_rad = 0;
    g_f4_ref_speed_rad_pi = 0;
    g_f4_angle_rad = 0;
    g_u1_flag_id_ref = MTR_ID_UP;
    g_u1_flag_iq_ref = MTR_IQ_ZERO;
    g_u1_flag_speed_ref = MTR_SPEED_ZERO;
    g_f4_modu = 0;
    g_f4_modv = 0;
    g_f4_modw = 0;
    g_u2_cnt_id_const = 0;
    boot_slow_start = SLOW_START_TIME;
    vdc_ad_k = VDC_AD_K_2;
    vr1_to_rpm_old = 0;
    motor_dir_old = 0;
    com_u1_ics_control_enable = 0;
}

/******************************************************************************
* Function Name : mtr_pi_ctrl
* Description   : PI control
* Arguments     : PI control input structure
* Return Value  : PI control output value
******************************************************************************/
float32 mtr_pi_ctrl(volatile MTR_PI_CTRL *pi_ctrl)
{
    float32 f4_err, f4_kp, f4_ki, f4_refp, f4_refi, f4_limit, f4_ref, f4_ilimit;
    float32 f4_temp0;

    f4_err   = pi_ctrl->f4_err;
    f4_kp    = pi_ctrl->f4_kp;
    f4_ki    = pi_ctrl->f4_ki;
    f4_limit = pi_ctrl->f4_limit;
    f4_refi  = pi_ctrl->f4_refi;
    f4_ilimit = pi_ctrl->f4_ilimit;

    f4_refp = f4_err * f4_kp;                       /* proportional part */

    f4_refi += f4_err * f4_ki;                      /* integral part */

    /*** integral part limit ***/
    f4_temp0 = fabsf(f4_refi);
    if ( f4_temp0 > f4_ilimit )
    {
        f4_refi = copysignf(f4_limit, f4_refi);
    }
    pi_ctrl->f4_refi = f4_refi;

    f4_ref = f4_refp + f4_refi;                     /* PI output */

    /*** output value limit ***/
    f4_temp0 = fabsf(f4_ref);
    if (f4_temp0 > f4_limit)
    {
        f4_ref = copysignf(f4_limit, f4_ref);
    }

    return (f4_ref);
}

/******************************************************************************
* Function Name : mtr_set_variables
* Description   : Set variables
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_set_variables(void)
{
    /* rotation direction */
    g_u1_dir_buff = (uint8)ics_input_buff.s2_direction;

    /* reference speed */
    if (MTR_CW == g_u1_dir_buff)
    {
        g_f4_ref_speed_rad = (float32)ics_input_buff.s2_ref_speed * MTR_POLE_PAIRS * MTR_RPM_RAD;
                                                        /* [rpm]->[rad/s] */
    }
    else if (MTR_CCW == g_u1_dir_buff)
    {
        g_f4_ref_speed_rad = - (float32)ics_input_buff.s2_ref_speed * MTR_POLE_PAIRS * MTR_RPM_RAD;
                                                        /* [rpm]->[rad/s] */
    }

    /* speed PI gain */
    g_f4_kp_speed = ics_input_buff.f4_kp_speed;
    g_f4_ki_speed = ics_input_buff.f4_ki_speed;

    /* q-axis current PI gain */
    g_f4_kp_iq = ics_input_buff.f4_kp_iq;
    g_f4_ki_iq = ics_input_buff.f4_ki_iq;

    /* speed LPF gain */
    g_f4_speed_lpf_k = ics_input_buff.f4_speed_lpf_k;

    /* current LPF gain */
    g_f4_current_lpf_k = ics_input_buff.f4_current_lpf_k;

    /* motor parameter */
    mtr_p.f4_mtr_rs = ics_input_buff.f4_mtr_rs;
    mtr_p.f4_mtr_rr = ics_input_buff.f4_mtr_rr;
    mtr_p.f4_mtr_m = ics_input_buff.f4_mtr_m;
    mtr_p.f4_mtr_ls = mtr_p.f4_mtr_m + ics_input_buff.f4_mtr_lls;
    mtr_p.f4_mtr_lr = mtr_p.f4_mtr_m + ics_input_buff.f4_mtr_llr;
    mtr_p.f4_mtr_m_lr = mtr_p.f4_mtr_m / mtr_p.f4_mtr_lr;
    mtr_p.f4_mtr_rr_lr = mtr_p.f4_mtr_rr / mtr_p.f4_mtr_lr;
    mtr_p.f4_mtr_sigma = 1.0f - (mtr_p.f4_mtr_m / mtr_p.f4_mtr_ls) * mtr_p.f4_mtr_m_lr;
    mtr_p.f4_mtr_ls_sigma = mtr_p.f4_mtr_ls * mtr_p.f4_mtr_sigma;

    /* current offset LPF gain */
    g_f4_offset_lpf_k = ics_input_buff.f4_offset_lpf_k;

    /* max speed */
    g_f4_max_speed_rad = (float32)ics_input_buff.s2_max_speed * MTR_POLE_PAIRS * MTR_RPM_RAD;
                                                        /* [rpm]->[rad/s] */

    g_f4_min_speed_rad = (float32)ics_input_buff.s2_min_speed * MTR_POLE_PAIRS * MTR_RPM_RAD;
                                                        /* [rpm]->[rad/s] */

    g_f4_lim_rotor_speed_rad = g_f4_max_speed_rad;
    g_f4_ilim_rotor_speed_rad = g_f4_max_speed_rad;

    /* Id reference */
    g_f4_ctrl_ref_id = ics_input_buff.f4_ctrl_ref_id;

    /* addition value of id reference when boot */
    g_f4_boot_id_up_step = ics_input_buff.f4_ctrl_ref_id / ics_input_buff.f4_boot_id_up_time;

    /* id const time when boot */
    g_f4_id_const_time = ics_input_buff.f4_id_const_time;

    /* accel value [rad/s^2] */
    g_f4_accel = ics_input_buff.f4_accel;

    /* speed fluctuation limit */
    g_f4_fluctuation_limit = ics_input_buff.f4_fluctuation_limit;

    /* angle compensation */
    g_f4_angle_compensation = ics_input_buff.f4_delay * MTR_CTRL_PERIOD;

    /* current offset calculation time */
    g_f4_offset_calc_time = ics_input_buff.f4_offset_calc_time;

    /* voltage drop caused by dead time and Vce */
     g_f4_voltage_drop = ics_input_buff.f4_voltage_drop;

    /* voltage drop corrective gain */
     g_f4_voltage_drop_k = ics_input_buff.f4_voltage_drop_k;

}

/******************************************************************************
* Function Name : R_MTR_IcsInput
* Description   : Set value to buffer
* Arguments     : ICS input structure
* Return Value  : none
******************************************************************************/
void R_MTR_IcsInput(MTR_ICS_INPUT *ics_input)
{
    /* rotation direction */
    ics_input_buff.s2_direction = ics_input->s2_direction;

    /* reference speed */
    ics_input_buff.s2_ref_speed = ics_input->s2_ref_speed;

    /* speed PI gain */
    ics_input_buff.f4_kp_speed = ics_input->f4_kp_speed;
    ics_input_buff.f4_ki_speed = ics_input->f4_ki_speed;

    /* q-axis current PI gain */
    ics_input_buff.f4_kp_iq = ics_input->f4_kp_iq;
    ics_input_buff.f4_ki_iq = ics_input->f4_ki_iq;

    /* speed LPF gain */
    ics_input_buff.f4_speed_lpf_k = ics_input->f4_speed_lpf_k;

    /* current LPF gain */
    ics_input_buff.f4_current_lpf_k = ics_input->f4_current_lpf_k;

    /* motor parameter */
    ics_input_buff.f4_mtr_rs = ics_input->f4_mtr_rs;
    ics_input_buff.f4_mtr_rr = ics_input->f4_mtr_rr;
    ics_input_buff.f4_mtr_m = ics_input->f4_mtr_m;
    ics_input_buff.f4_mtr_lls = ics_input->f4_mtr_lls;
    ics_input_buff.f4_mtr_llr = ics_input->f4_mtr_llr;

    /* current offset LPF gain */
    ics_input_buff.f4_offset_lpf_k = ics_input->f4_offset_lpf_k;

    /* max speed */
    ics_input_buff.s2_max_speed = ics_input->s2_max_speed;

    /* max speed */
    ics_input_buff.s2_min_speed = ics_input->s2_min_speed;

    /* Id reference */
    ics_input_buff.f4_ctrl_ref_id = ics_input->f4_ctrl_ref_id;

    /* time to set id reference when boot */
    ics_input_buff.f4_boot_id_up_time = ics_input->f4_boot_id_up_time;

    /* id const time when boot  */
    ics_input_buff.f4_id_const_time = ics_input->f4_id_const_time;

    /* accel value [rad/s^2] */
    ics_input_buff.f4_accel = ics_input->f4_accel;

    /* speed fluctuation limit */
    ics_input_buff.f4_fluctuation_limit = ics_input->f4_fluctuation_limit;

    /* g_f4_angle_compensation = ics_input_buff.f4_delay * MTR_CTRL_PERIOD */
    ics_input_buff.f4_delay = ics_input->f4_delay;

    /* current offset calculation time */
    ics_input_buff.f4_offset_calc_time = ics_input->f4_offset_calc_time;

    /* voltage drop caused by dead time and Vce */
    ics_input_buff.f4_voltage_drop = ics_input->f4_voltage_drop;

    /* voltage drop corrective gain */
    ics_input_buff.f4_voltage_drop_k = ics_input->f4_voltage_drop_k;

    g_u1_enable_write = MTR_FLG_SET;
}

/******************************************************************************
* Function Name : R_MTR_SetDir
* Description   : Set rotation direction
* Arguments     : dir - Rotational direction
* Return Value  : none
******************************************************************************/
void R_MTR_SetDir(uint8 dir)
{
    g_u1_direction = dir;
}

/******************************************************************************
* Function Name : R_MTR_GetSpeed
* Description   : Get speed value
* Arguments     : none
* Return Value  : g_s2_rpm - Speed
******************************************************************************/
float32 R_MTR_GetSpeed(void)
{
    float32 f4_speed_rpm;

    f4_speed_rpm = (g_f4_speed_rad / (MTR_RPM_RAD * MTR_POLE_PAIRS));

    return (f4_speed_rpm);
}

/******************************************************************************
* Function Name : R_MTR_GetDir
* Description   : Get rotation direction
* Arguments     : none
* Return Value  : g_u1_direction - Rotational direction
******************************************************************************/
uint8 R_MTR_GetDir(void)
{
    return (g_u1_direction);
}

/******************************************************************************
* Function Name : R_MTR_GetStatus
* Description   : Get status of motor control system
* Arguments     : none
* Return Value  : g_u1_mode_system - Motor status
******************************************************************************/
uint8 R_MTR_GetStatus(void)
{
    return (g_u1_mode_system);
}

/******************************************************************************
* Function Name : mtr_error_check
* Description   : Check error status
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_error_check(void)
{
    float32 f4_temp0;

    /*==================================*/
    /*     over current error check     */
    /*==================================*/
    f4_temp0 = fabsf(g_f4_iu_ad);
    if (f4_temp0 > MTR_OVERCURRENT_LIMIT)
    {
        R_MTR_ExecEvent(MTR_EVENT_ERROR);               /* execute error event */
        g_u1_error_status = MTR_OVER_CURRENT_ERROR;     /* over current error */
    }
    f4_temp0 = fabsf(g_f4_iv_ad);
    if (f4_temp0 > MTR_OVERCURRENT_LIMIT)
    {
        R_MTR_ExecEvent(MTR_EVENT_ERROR);               /* execute error event */
        g_u1_error_status = MTR_OVER_CURRENT_ERROR;     /* over current error */
    }
    f4_temp0 = fabsf(g_f4_iw_ad);
    if (f4_temp0 > MTR_OVERCURRENT_LIMIT)
    {
        R_MTR_ExecEvent(MTR_EVENT_ERROR);               /* execute error event */
        g_u1_error_status = MTR_OVER_CURRENT_ERROR;     /* over current error */
    }

    /*==================================*/
    /*     over voltage error check     */
    /*==================================*/
    if (g_f4_vdc_ad > MTR_OVERVOLTAGE_LIMIT)
    {
        R_MTR_ExecEvent(MTR_EVENT_ERROR);               /* execute error event */
        g_u1_error_status = MTR_OVER_VOLTAGE_ERROR;     /* over voltage error */
    }

    /*===================================*/
    /*     under voltage error check     */
    /*===================================*/
    if (g_f4_vdc_ad < MTR_UNDERVOLTAGE_LIMIT)
    {
        R_MTR_ExecEvent(MTR_EVENT_ERROR);               /* execute error event */
        g_u1_error_status = MTR_UNDER_VOLTAGE_ERROR;    /* under voltage error */
    }

    /*================================*/
    /*     over speed error check     */
    /*================================*/
    f4_temp0 = fabsf(g_f4_speed_rad);
    if (f4_temp0 > MTR_SPEED_LIMIT)
    {
        R_MTR_ExecEvent(MTR_EVENT_ERROR);               /* execute error event */
        g_u1_error_status = MTR_OVER_SPEED_ERROR;       /* over speed error */
    }
}

/******************************************************************************
* Function Name : mtr_set_speed_ref
* Description   : Set speed reference
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_set_speed_ref(void)
{
    float32 f4_temp0;
    float32 f4_ref_speed_max;
    float32 f4_ref_speed_min;

    switch (g_u1_flag_speed_ref)
    {
        case MTR_SPEED_ZERO:
            g_f4_temp_ref_speed_rad = 0;
            if (g_u2_run_mode == MTR_CTRL_MODE)
            {
                g_u1_flag_speed_ref = MTR_SPEED_CHANGE;
            }
        break;

        case MTR_SPEED_CHANGE:
            f4_temp0 = g_f4_ref_speed_rad - g_f4_temp_ref_speed_rad;
            f4_temp0 = copysignf(g_f4_accel, f4_temp0);
            g_f4_temp_ref_speed_rad = g_f4_temp_ref_speed_rad + f4_temp0;

            if (f4_temp0 >= 0)
            {
                if (g_f4_temp_ref_speed_rad > g_f4_ref_speed_rad)
                {
                    g_f4_temp_ref_speed_rad = g_f4_ref_speed_rad;
                }
            }
            else if (f4_temp0 < 0)
            {
                if (g_f4_temp_ref_speed_rad < g_f4_ref_speed_rad)
                {
                    g_f4_temp_ref_speed_rad = g_f4_ref_speed_rad;
                }
            }
        break;

        default:
        break;
    }

    if (MTR_SPEED_ZERO != g_u1_flag_speed_ref)
    {
        /* speed reference accel limit */
        f4_ref_speed_max = g_f4_temp_speed_rad + g_f4_fluctuation_limit;
        f4_ref_speed_min = g_f4_temp_speed_rad - g_f4_fluctuation_limit;

        if (g_f4_temp_ref_speed_rad > f4_ref_speed_max)
        {
            g_f4_temp_ref_speed_rad = f4_ref_speed_max;
        }
        else if(g_f4_temp_ref_speed_rad < f4_ref_speed_min)
        {
            g_f4_temp_ref_speed_rad = f4_ref_speed_min;
        }

        /* speed reference limit */
        f4_temp0 = fabsf(g_f4_temp_ref_speed_rad);
        if (f4_temp0 > g_f4_max_speed_rad)
        {
            g_f4_temp_ref_speed_rad = copysignf(g_f4_max_speed_rad, g_f4_temp_ref_speed_rad);
        }
    }

    /* set speed reference */
    g_f4_ref_speed_rad_pi = g_f4_temp_ref_speed_rad;
}

/******************************************************************************
* Function Name : mtr_set_iq_ref
* Description   : Set delta-axis current reference
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_set_iq_ref(void)
{
    switch (g_u1_flag_iq_ref)
    {
        case MTR_IQ_ZERO:
            g_f4_iq_ref_buff = 0;
            if (g_u2_run_mode == MTR_CTRL_MODE)
            {
                g_u1_flag_iq_ref = MTR_IQ_SPEED_PI_OUTPUT;
            }
        break;

        case MTR_IQ_SPEED_PI_OUTPUT:
            /*** speed PI control ***/
            if (g_u1_mode_system == MTR_MODE_RUN)
            {
                speed.f4_err = g_f4_temp_ref_speed_rad - g_f4_temp_speed_rad;
                speed.f4_kp = g_f4_kp_speed;
                speed.f4_ki = g_f4_ki_speed;
                speed.f4_limit = g_f4_lim_iq;
                speed.f4_ilimit = g_f4_ilim_iq;
                g_f4_iq_ref_buff = mtr_pi_ctrl(&speed);
            }
            else
            {
                g_f4_iq_ref_buff = 0;
            }
        break;

        default:
        break;
    }

    /* set iq reference */
    g_f4_iq_ref = g_f4_iq_ref_buff;
}

/******************************************************************************
* Function Name : mtr_set_id_ref
* Description   : Set gamma-axis current reference
* Arguments     : none
* Return Value  : none
******************************************************************************/
void mtr_set_id_ref(void)
{
    switch (g_u1_flag_id_ref)
    {
        case MTR_ID_UP:
            g_f4_id_ref_buff = g_f4_id_ref_buff + g_f4_boot_id_up_step;
            if (g_f4_id_ref_buff > g_f4_ctrl_ref_id)
            {
                g_f4_id_ref_buff = g_f4_ctrl_ref_id;

                g_u1_flag_id_ref = MTR_ID_CONST;
            }
        break;

        case MTR_ID_CONST:
            g_u2_cnt_id_const++;
            if (g_u2_cnt_id_const >= g_f4_id_const_time)
            {
                g_u1_flag_id_ref = MTR_ID_CONST_CTRL;
            }
        break;

        case MTR_ID_CONST_CTRL:
            g_f4_id_ref_buff = g_f4_ctrl_ref_id;
        break;

        default:
        break;
    }

    /* set id reference */
    g_f4_id_ref = g_f4_id_ref_buff;
}

/******************************************************************************
* Function Name : mtr_calc_mod
* Description   : calculate modulation
* Arguments     : float32 f4_vu, float32 f4_vv, float32 f4_vw, float32 f4_vdc
* Return Value  : none
******************************************************************************/
void mtr_calc_mod(float32 f4_vu, float32 f4_vv, float32 f4_vw, float32 f4_vdc)
{
    float32 f4_temp0;

    f4_temp0 = f4_vdc / 2;

    g_f4_modu = f4_vu / f4_temp0;
    g_f4_modv = f4_vv / f4_temp0;
    g_f4_modw = f4_vw / f4_temp0;
}
