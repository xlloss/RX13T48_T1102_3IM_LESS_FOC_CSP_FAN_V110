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
* File Name   : mtr_interrupt.c
* Description : The interrupt handler
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
#include "mtr_ctrl_rx13t48.h"
#include "mtr_ctrl_t1102.h"
#include "iodefine.h"
#include "ICS2_RX13T.h"
#include "mtr_ctrl_rx13t48_t1102.h"

/******************************************************************************
* Interrupt vector define
******************************************************************************/
#pragma interrupt (mtr_over_current_interrupt(vect=VECT_POE_OEI4))
static void mtr_over_current_interrupt(void);

#pragma interrupt (mtr_mtu4_interrupt(vect=VECT_MTU4_TCIV4))
static void mtr_mtu4_interrupt(void);

#pragma interrupt (mtr_cmt0_interrupt(vect=VECT_CMT0_CMI0))
static void mtr_cmt0_interrupt(void);

/******************************************************************************
* Private functions
******************************************************************************/
static void mtr_over_current_interrupt(void);

/******************************************************************************
* Exported global variables
******************************************************************************/
extern volatile float32 g_f4_iu_ad_err_det;
extern volatile float32 g_f4_iv_ad_err_det;
extern volatile float32 g_f4_iw_ad_err_det;
/* for sequence control */
extern volatile uint8    g_u1_mode_system;          /* system mode */
extern volatile uint16   g_u2_run_mode;             /* run mode */
extern volatile uint16   g_u2_ctrl_mode;            /* control mode */
extern volatile uint8    g_u1_error_status;         /* error status */

/****** for ICS ******/
extern volatile uint8    g_u1_cnt_ics;              /* counter for period of calling "ics_watchpoint" */
/*********************/

extern volatile int16    g_boot_delay;
extern volatile int16    g_boot_slow_start;
extern volatile float32  g_f4_vdc_ad;               /* inverter bus voltage */
extern volatile float32  g_f4_vd_ref;               /* gamma-axis output voltage command */
extern volatile float32  g_f4_vq_ref;               /* delta-axis output voltage command */
extern volatile float32  g_f4_iu_ad;                /* U phase current */
extern volatile float32  g_f4_pre_iu_ad;            /* Previous U phase current */
extern volatile float32  g_f4_iv_ad;                /* V phase current */
extern volatile float32  g_f4_pre_iv_ad;            /* Previous V phase current */
extern volatile float32  g_f4_iw_ad;                /* W phase current */
extern volatile float32  g_f4_pre_iw_ad;            /* Previous W phase current */
extern volatile float32  g_f4_offset_iu;            /* U phase current offset */
extern volatile float32  g_f4_offset_iv;            /* V phase current offset */
extern volatile float32  g_f4_offset_iw;            /* W phase current offset */
extern volatile float32  g_f4_id_lpf;               /* gamma-axis current */
extern volatile float32  g_f4_iq_lpf;               /* delta-axis current */
extern volatile float32  g_f4_pre_id_lpf;           /* Previous gamma-axis current */
extern volatile float32  g_f4_pre_iq_lpf;           /* Previous delta-axis current */
extern volatile float32  g_f4_kp_id;                /* proportional gain for gamma-axis current PI control */
extern volatile float32  g_f4_ki_id;                /* integral gain for gamma-axis current PI control */
extern volatile float32  g_f4_lim_id;               /* gamma-axis current PI control limit value */
extern volatile float32  g_f4_ilim_id;              /* gamma-axis current PI control integral limit value */
extern volatile float32  g_f4_kp_iq;                /* proportional gain for delta-axis current PI control */
extern volatile float32  g_f4_ki_iq;                /* integral gain for delta-axis current PI control */
extern volatile float32  g_f4_lim_rotor_speed_rad;  /* rotor speed PI control limit value */
extern volatile float32  g_f4_ilim_rotor_speed_rad; /* rotor speed PI control integral limit value */
extern volatile float32  g_f4_id_ref;               /* gamma-axis current command */
extern volatile float32  g_f4_id_ref2;              /* gamma-axis current command */
extern volatile float32  g_f4_iq_ref;               /* delta-axis current command */
extern volatile float32  g_f4_ref_stator_speed_rad; /* stator speed command */
extern volatile float32  g_f4_slip_speed_rad;       /* slip speed */
extern volatile float32  g_f4_slip_k;               /* slip speed gain */
extern volatile float32  g_f4_speed_rad;            /* calculated speed value */
extern volatile float32  g_f4_ref_speed_rad_pi;     /* speed command for PI control */
extern volatile float32  g_f4_ref_speed_rad;        /* speed command */
extern volatile float32  g_f4_angle_rad;            /* stator interlinkage flux angle */
extern volatile float32  g_f4_refu;                 /* U phase voltage command */
extern volatile float32  g_f4_refv;                 /* V phase voltage command */
extern volatile float32  g_f4_refw;                 /* W phase voltage command */
extern volatile float32  g_f4_inv_limit;            /* voltage limit */
extern volatile float32  g_f4_speed_lpf_k;          /* speed LPF gain */
extern volatile float32  g_f4_current_lpf_k;        /* current LPF gain */
extern volatile float32  g_f4_offset_lpf_k;         /* current offset LPF gain */
extern volatile uint8    g_u1_direction;            /* rotational direction (0:CW ,1:CCW) */
extern volatile uint8    g_u1_enable_write;         /* ICS write enable flag */
extern volatile uint16   g_u2_cnt_adjust;           /* counter for current offset calculation */
extern volatile uint8    g_u1_flag_id_ref;          /* gamma-axis current command management flag */
extern volatile float32  g_f4_temp_speed_rad;       /* for holding speed value */
extern volatile float32  g_f4_temp_ref_speed_rad;   /* for holding speed command value */
extern volatile float32  g_f4_angle_compensation;   /* angle compensation gain */
extern volatile float32  g_f4_offset_calc_time;     /* current offset calculation time */
extern volatile float32  g_f4_voltage_drop;         /* voltage drop */
extern volatile float32  g_f4_voltage_drop_k;       /* voltage drop correction gain */
extern volatile float32  g_f4_modu;                 /* U phase modulation factor */
extern volatile float32  g_f4_modv;                 /* V phase modulation factor */
extern volatile float32  g_f4_modw;                 /* W phase modulation factor */
extern volatile MTR_PI_CTRL rotor_speed;            /* PI control structure */
extern volatile MTR_PI_CTRL id_ACR;                 /* PI control structure */
extern volatile MTR_PARAMETER mtr_p;                /* motor parameters and control parameters structure */
extern volatile float32 vdc_ad_k;
/******************************************************************************
* Function Name : mtr_over_current_interrupt
* Description   : POE0# interrupt
* Arguments     : none
* Return Value  : none
******************************************************************************/
static void mtr_over_current_interrupt(void)
{
    R_MTR_ExecEvent(MTR_EVENT_ERROR);                       /* execute error event */
    g_u1_error_status = MTR_OVER_CURRENT_ERROR;             /* error status update : over current error */

    mtr_clear_oc_flag();                                    /* clear forced cutoff flag (detect over current) */
}

/******************************************************************************
* Function Name : mtr_mtu4_interrupt
* Description   : MTU3_4 interrupt (Period: 125 [us])
* Arguments     : none
* Return Value  : none
******************************************************************************/
static void mtr_mtu4_interrupt(void)
{
    float32 f4_temp0;
    float32 f4_temp1;
    float32 f4_temp2;
    float32 f4_temp3;
    float32 f4_temp4;
    float32 f4_temp5;

    /*=============================================*/
    /*     A/D : current & voltage detection       */
    /*=============================================*/
clrpsw_i();
    mtr_get_iuiviwvdc(&g_f4_iu_ad, &g_f4_iv_ad, &g_f4_iw_ad, &g_f4_vdc_ad);

    /*
     * IA = (ADC VAL - no load ADC val) / 819 * 33.333A
     * 4096  -> 5v
     * 2016  -> 2.4v
     * 33.33 -> opa current opamp multiplication
     */
    g_f4_iu_ad_err_det = (fabsf(g_f4_iu_ad - 2016) / 819) * 33.33;
    g_f4_iv_ad_err_det = (fabsf(g_f4_iv_ad - 2016) / 819) * 33.33;
    g_f4_iw_ad_err_det = (fabsf(g_f4_iw_ad - 2016) / 819) * 33.33;

setpsw_i();

#if ADC_WORKAROUND
    /* 1980 is 2.4v on static volt */
    g_f4_iu_ad = g_f4_iu_ad - 2016;
    g_f4_iv_ad = g_f4_iv_ad - 2016;
    g_f4_iw_ad = g_f4_iw_ad - 2016;

    g_f4_iu_ad = (g_f4_iu_ad * (4.00f / 4095.0f));
    g_f4_iv_ad = (g_f4_iv_ad * (8.00f / 4095.0f));
    g_f4_iw_ad = (g_f4_iw_ad * (10.0f / 4095.0f));

#else
    g_f4_iu_ad = g_f4_iu_ad - MTR_ADC_SCALING;
    g_f4_iv_ad = g_f4_iv_ad - MTR_ADC_SCALING;
    g_f4_iw_ad = g_f4_iw_ad - MTR_ADC_SCALING;

    g_f4_iu_ad = g_f4_iu_ad * MTR_CURRENT_SCALING;          /* U-phase current */
    g_f4_iv_ad = g_f4_iv_ad * MTR_CURRENT_SCALING;          /* V-phase current */
    g_f4_iw_ad = g_f4_iw_ad * MTR_CURRENT_SCALING;          /* W-phase current */
#endif

    /*** current offset adjustment ***/
    if (g_f4_offset_calc_time == g_u2_cnt_adjust)
    {
        g_f4_iu_ad = -g_f4_iu_ad + g_f4_offset_iu;
        g_f4_iv_ad = -g_f4_iv_ad + g_f4_offset_iv;
        g_f4_iw_ad = -g_f4_iw_ad + g_f4_offset_iw;

        g_f4_iu_ad = g_f4_pre_iu_ad + g_f4_current_lpf_k * (g_f4_iu_ad - g_f4_pre_iu_ad);
        g_f4_pre_iu_ad = g_f4_iu_ad;
        g_f4_iv_ad = g_f4_pre_iv_ad + g_f4_current_lpf_k * (g_f4_iv_ad - g_f4_pre_iv_ad);
        g_f4_pre_iv_ad = g_f4_iv_ad;
        g_f4_iw_ad = g_f4_pre_iw_ad + g_f4_current_lpf_k * (g_f4_iw_ad - g_f4_pre_iw_ad);
        g_f4_pre_iw_ad = g_f4_iw_ad;
    }
    else
    {
        /*** adjust offset ***/
        g_f4_offset_iu += g_f4_offset_lpf_k * (g_f4_iu_ad - g_f4_offset_iu);
        g_f4_iu_ad = -g_f4_iu_ad + g_f4_offset_iu;
        g_f4_offset_iv += g_f4_offset_lpf_k * (g_f4_iv_ad - g_f4_offset_iv);
        g_f4_iv_ad = -g_f4_iv_ad + g_f4_offset_iv;
        g_f4_offset_iw += g_f4_offset_lpf_k * (g_f4_iw_ad - g_f4_offset_iw);
        g_f4_iw_ad = -g_f4_iw_ad + g_f4_offset_iw;

        g_u2_cnt_adjust++;
        if (g_u2_cnt_adjust >= g_f4_offset_calc_time)
        {
            g_u2_cnt_adjust = g_f4_offset_calc_time;
        }
    }

    g_f4_vdc_ad = g_f4_vdc_ad * MTR_VDC_SCALING;            /* Vdc */
    /*============================*/
    /*        check error         */
    /*============================*/
    mtr_error_check();

    /*==============================================================*/
    /*  coordinate transformation(UVW -> alpha&beta ->gamma-delta)  */
    /*==============================================================*/
    f4_temp0 = cosf(g_f4_angle_rad) / MTR_SQRT_2;
    f4_temp1 = sinf(g_f4_angle_rad) / MTR_SQRT_2;
    f4_temp2 = MTR_SQRT_3 * f4_temp0;
    f4_temp3 = MTR_SQRT_3 * f4_temp1;
    g_f4_id_lpf = f4_temp2 * g_f4_iu_ad + f4_temp1 * (g_f4_iv_ad - g_f4_iw_ad);
    g_f4_iq_lpf = - f4_temp3 * g_f4_iu_ad + f4_temp0 * (g_f4_iv_ad - g_f4_iw_ad);

    /*=============================*/
    /*     current PI control      */
    /*=============================*/
    if (g_u1_mode_system == MTR_MODE_RUN)
    {
        /*** PI control(Id) ***/
        id_ACR.f4_err = g_f4_id_ref - g_f4_id_lpf;
        id_ACR.f4_kp = g_f4_kp_id;
        id_ACR.f4_ki = g_f4_ki_id;
        id_ACR.f4_limit = g_f4_lim_id;
        id_ACR.f4_ilimit = g_f4_ilim_id;
        g_f4_id_ref2 = mtr_pi_ctrl( &id_ACR );

        /*** PI control(Iq) ***/
        rotor_speed.f4_err = g_f4_iq_ref - g_f4_iq_lpf;
        rotor_speed.f4_kp = g_f4_kp_iq;
        rotor_speed.f4_ki = g_f4_ki_iq;
        rotor_speed.f4_limit = g_f4_lim_rotor_speed_rad;
        rotor_speed.f4_ilimit = g_f4_ilim_rotor_speed_rad;
        g_f4_speed_rad = mtr_pi_ctrl( &rotor_speed );
    }
    else
    {
        g_f4_id_ref2 = 0.0f;
        g_f4_speed_rad = 0.0f;
    }

    /*==============================*/
    /*   stator speed estimation    */
    /*==============================*/
    g_f4_slip_speed_rad = g_f4_slip_k * g_f4_iq_ref;
    g_f4_ref_stator_speed_rad = g_f4_speed_rad + g_f4_slip_speed_rad;

    /*===========================*/
    /*     calculate voltage     */
    /*===========================*/
    if (g_u1_mode_system == MTR_MODE_RUN)
    {
        g_f4_vd_ref = mtr_p.f4_mtr_rs * g_f4_id_ref2 - g_f4_ref_stator_speed_rad * mtr_p.f4_mtr_ls_sigma * g_f4_iq_ref;
        g_f4_vq_ref = mtr_p.f4_mtr_rs * g_f4_iq_ref + g_f4_ref_stator_speed_rad * mtr_p.f4_mtr_ls * g_f4_id_ref2;
    }
    else
    {
        g_f4_vd_ref = 0.0f;
        g_f4_vq_ref = 0.0f;
    }

    /*================================================*/
    /*      coordinate transformation(dq -> UVW)      */
    /*================================================*/
    /* angle compensation */
    g_f4_angle_rad = g_f4_angle_rad + g_f4_ref_stator_speed_rad * g_f4_angle_compensation;
    while (g_f4_angle_rad > MTR_TWOPI)
    {
        g_f4_angle_rad = g_f4_angle_rad - MTR_TWOPI;
    }
    while (g_f4_angle_rad < 0.0f)
    {
        g_f4_angle_rad = g_f4_angle_rad + MTR_TWOPI;
    }

    f4_temp0 = cosf(g_f4_angle_rad);
    f4_temp1 = sinf(g_f4_angle_rad);

    f4_temp2 = f4_temp0 * f4_temp0 * f4_temp0;
    f4_temp3 = f4_temp1 * f4_temp1 * f4_temp1;

    f4_temp4 = ((f4_temp0 / 2) + (2 * f4_temp2) / 3) * g_f4_vd_ref;
    f4_temp5 = ((3 * f4_temp1) / 2 - (2 * f4_temp3) / 3) * g_f4_vq_ref;
    g_f4_refu = MTR_SQRT_2_3 * (f4_temp4 - f4_temp5);

    f4_temp4 = (- f4_temp0 + (2 * f4_temp2) / 3 + (MTR_SQRT_3 * f4_temp1) / 2) * g_f4_vd_ref;
    f4_temp5 = ((- MTR_SQRT_3 * f4_temp0) / 2 - (2 * f4_temp3) / 3) * g_f4_vq_ref;
    g_f4_refv = MTR_SQRT_2_3 * (f4_temp4 - f4_temp5);

    f4_temp4 = (- f4_temp0 + (2 * f4_temp2) / 3 - (MTR_SQRT_3 * f4_temp1) / 2) * g_f4_vd_ref;
    f4_temp5 = ((MTR_SQRT_3 * f4_temp0) / 2 - (2 * f4_temp3) / 3) * g_f4_vq_ref;
    g_f4_refw = MTR_SQRT_2_3 * (f4_temp4 - f4_temp5);

    /*===================================*/
    /*     voltage drop compensation     */
    /*===================================*/
    f4_temp0 = g_f4_voltage_drop_k * g_f4_iu_ad;
    f4_temp1 = fabsf(f4_temp0);
    if (f4_temp1 > g_f4_voltage_drop)
    {
        f4_temp0 = copysignf(g_f4_voltage_drop, f4_temp0);
    }
    g_f4_refu = g_f4_refu + f4_temp0;

    f4_temp0 = g_f4_voltage_drop_k * g_f4_iv_ad;
    f4_temp1 = fabsf(f4_temp0);
    if (f4_temp1 > g_f4_voltage_drop)
    {
        f4_temp0 = copysignf(g_f4_voltage_drop, f4_temp0);
    }
    g_f4_refv = g_f4_refv + f4_temp0;

    f4_temp0 = g_f4_voltage_drop_k * g_f4_iw_ad;
    f4_temp1 = fabsf(f4_temp0);
    if (f4_temp1 > g_f4_voltage_drop)
    {
        f4_temp0 = copysignf(g_f4_voltage_drop, f4_temp0);
    }
    g_f4_refw = g_f4_refw + f4_temp0;

    /*=======================*/
    /*     limit vlotage     */
    /*=======================*/
    g_f4_inv_limit = vdc_ad_k * g_f4_vdc_ad;
    f4_temp0 = fabsf(g_f4_refu);
    if (f4_temp0 > g_f4_inv_limit)
    {
        g_f4_refu = copysignf(g_f4_inv_limit, g_f4_refu);
    }

    f4_temp0 = fabsf(g_f4_refv);
    if (f4_temp0 > g_f4_inv_limit)
    {
        g_f4_refv = copysignf(g_f4_inv_limit, g_f4_refv);
    }

    f4_temp0 = fabsf(g_f4_refw);
    if (f4_temp0 > g_f4_inv_limit)
    {
        g_f4_refw = copysignf(g_f4_inv_limit, g_f4_refw);
    }

    if (g_u1_mode_system != MTR_MODE_RUN)
    {
        g_f4_refu = 0.0f;
        g_f4_refv = 0.0f;
        g_f4_refw = 0.0f;
    }

    /*==============================*/
    /*     calculate modulation     */
    /*==============================*/
    mtr_calc_mod(g_f4_refu, g_f4_refv, g_f4_refw, g_f4_vdc_ad);

    /*==============================*/
    /*     PWM reference setting    */
    /*==============================*/
    mtr_inv_set_uvw(g_f4_modu, g_f4_modv, g_f4_modw);

    /****** for ICS ******/
    g_u1_cnt_ics++;
    if (g_u1_cnt_ics >= 3)
    {
        g_u1_cnt_ics = 0;
        ics2_watchpoint();                              /* call ICS */
    }
    /*********************/

    /*** set variables ***/
    if (MTR_FLG_SET == g_u1_enable_write)
    {
        mtr_set_variables();
        g_u1_enable_write = MTR_FLG_CLR;
    }

    mtr_clear_mtu4_flag();
}

/******************************************************************************
* Function Name : mtr_cmt0_interrupt
* Description   : CMT0 interrupt (Period: 1ms)
* Arguments     : none
* Return Value  : none
******************************************************************************/
static void mtr_cmt0_interrupt(void)
{
setpsw_i();

    /* temporary variables */
    g_f4_temp_speed_rad = g_f4_speed_rad;                   /* speed */
    g_f4_temp_ref_speed_rad = g_f4_ref_speed_rad_pi;        /* reference speed */

    /*====================*/
    /*   set reference    */
    /*====================*/
    if (MTR_MODE_RUN == g_u1_mode_system)
    {
        /***** decide direction *****/
        if (g_f4_temp_speed_rad >= 0)
        {
            g_u1_direction = MTR_CW;
        }
        else if (g_f4_temp_speed_rad < 0)
        {
            g_u1_direction = MTR_CCW;
        }

        /***** Id, Iq, speed reference setting *****/
        switch (g_u2_run_mode)
        {
            case MTR_BOOT_MODE:
                if (g_f4_offset_calc_time == g_u2_cnt_adjust)
                {
                    g_u2_run_mode = MTR_START_MODE;
                }
            break;

            case MTR_START_MODE:
                mtr_set_speed_ref();
                mtr_set_iq_ref();
                mtr_set_id_ref();
                if (MTR_ID_CONST_CTRL == g_u1_flag_id_ref)
                {
                    g_u2_run_mode = MTR_CTRL_MODE;
                }
            break;

            case MTR_CTRL_MODE:
                mtr_set_speed_ref();
                mtr_set_iq_ref();
                mtr_set_id_ref();
            break;

            default:
            break;
        }
    }

    /*===========================*/
    /*   calcurate slip factor   */
    /*===========================*/
    if(g_f4_id_ref < 0.1)
    {
        g_f4_slip_k = 0;
    }
    else
    {
        g_f4_slip_k = mtr_p.f4_mtr_rr_lr / g_f4_id_ref;
    }

    mtr_clear_cmt0_flag();

    if (g_boot_delay > 0)
        g_boot_delay--;

    if (g_boot_slow_start > 0 && g_u1_mode_system == MTR_MODE_RUN)
        g_boot_slow_start--;

}

