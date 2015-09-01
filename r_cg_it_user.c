/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only 
* intended for use with Renesas products. No other uses are authorized. This 
* software is owned by Renesas Electronics Corporation and is protected under 
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING 
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT 
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE 
* AND NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS 
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE 
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR 
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE 
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software 
* and to discontinue the availability of this software.  By using this software, 
* you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2011, 2013 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_it_user.c
* Version      : CodeGenerator for RL78/G13 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : This file implements device driver for IT module.
* Creation Date: 2015/8/14
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt INTIT r_it_interrupt
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_it.h"
/* Start user code for include. Do not edit comment generated here */
#include "ahrs.h"
#include "r_cg_port.h"
#include "r_cg_serial.h"
#include "math.h"
#include "r_cg_timer.h"
#include "r_cg_intc.h"
#include "ArduCopter.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
extern _Q_ANGLE   Q_ANGLE;
unsigned long systemtime=0;
uint8_t Attitude_500Hz = 0;
uint8_t Debug_5Hz = 0;
uint8_t Control_100Hz =0;
uint8_t ADC_1Hz = 0; 
uint8_t camera_flag=0;
extern uint8_t SystemInitOK;
extern float current_hight;
extern uint16_t PWM_InCh3 ;
extern uint16_t hight_target;
extern int16_t  target_x_cm,target_y_cm;
extern uint16_t basis_throttle;
extern uint8_t surface_quality;
extern uint8_t TASK;
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: r_it_interrupt
* Description  : This function is INTIT interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_it_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    static uint16_t ms2=0,ms10=0,ms200=0,ms300=0,ms500=0,ms1000=0;
    systemtime ++;
    if(SystemInitOK){
    if ( ++ms10 == 10) { 
           ms10 = 0;
	   Control_100Hz = 1;
    }
    if ( ++ms200 == 200){ 
           Debug_5Hz = 1;
	   ms200 = 0;
	   if(TASK != 0)return_blackline();
	   else
	   return_angle();
    }
    if ( ++ms500 == 500){
           ms500 = 0;
	   if(TASK != 0)return_hight((int)current_hight);
           else                              
	   return_hight(PWM_InCh3);
    }
    if ( ++ms1000 == 1000){
           ms1000=0;
	   ADC_1Hz = 1;
//	   return_basis_throttle();
//	   return_target_angle();
    }
    }

    if(systemtime >= 4000000000)
    systemtime=0;
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
unsigned long millisecond(void)
{
	return systemtime;
}

/* End user code. Do not edit comment generated here */
