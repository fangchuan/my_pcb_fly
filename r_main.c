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
* File Name    : r_main.c
* Version      : CodeGenerator for RL78/G13 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : This file implements main function.
* Creation Date: 2015/8/14
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */

/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_port.h"
#include "r_cg_intc.h"
#include "r_cg_serial.h"
#include "r_cg_adc.h"
#include "r_cg_timer.h"
#include "r_cg_it.h"
/* Start user code for include. Do not edit comment generated here */
#include "IIC.h"
#include "ahrs.h"
#include "delay.h"
#include "ArduCopter.h" 
#include "cmd.h"
#include "ccd.h"
#include "task.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
extern uint8_t Attitude_500Hz;
extern uint8_t Debug_5Hz;
extern uint8_t Control_100Hz;
extern uint8_t ADC_1Hz;
extern uint8_t rx_buffer;
extern uint8_t Rx_Buffer[20];

extern uint8_t camera_flag;
uint16_t PWM_InCh1,PWM_InCh2,PWM_InCh3,PWM_InCh4;
uint8_t SystemInitOK=0;
uint32_t current_time;

extern void debug_print(void);

//unit ms
#define TASK_PERIOD 10
#define TASK_TIP_MAX (TASK_PERIOD /10)
int task_tip = 0;
int task_valid = 0;
task_t current_task;

void BSP_Start(void)
{
    R_UART0_Start();

    R_KEY_Start();
    R_TAU0_Channel0_Start();
    R_TAU0_Channel5_Start();
    R_TAU0_Channel6_Start();
    R_IT_Start();
//    R_ADC_Set_OperationOn();
//    R_ADC_Start();

    I2C_Init();
    CCD_init();
}

void route_receive_start(void)
{
    R_UART0_Start();
    R_UART0_Receive(&rx_buffer,1);
}

void task_loop(void)
{
	//control period
	task_tip++;
	if(task_tip < TASK_TIP_MAX)
	{
		return;
	}
	task_tip = 0;
	
	//get task
	if(!task_valid)
	{
		int ret = task_array_get(&current_task);
		if(!ret)
		{
			task_valid = 1;
			//set initialize target
			arducopter_set_target(&current_task.target_angle, current_task.target_height,
                                               current_task.relay_io,current_task.camera_io);
		}
	}
	//check again
	if(!task_valid)
	{
		return;
	}
	//do task
	// ....
	
	current_task.duration -= TASK_PERIOD;
	//check stop condition
	if(current_task.stop_type == TASK_TYPE_STOP_BY_TIME)
	{
		if(current_task.duration < 0)
		{
			task_valid = 0;
//			debug_prints("task finish\r\n");
		}
	}
	else if(current_task.stop_type == TASK_TYPE_STOP_BY_LINE_WIDTH)
	{
		if(current_task.wait_greater)
		{
			if(!in_lost && blackline_width > current_task.line_width)
			{
				task_valid = 0;
				debug_prints("find cross line\r\n");
			}
		}
		else
		{
			if(in_lost || blackline_width < current_task.line_width)
			{
				task_valid = 0;
				debug_prints("leave cross line\r\n");
			}
		}
	}
	else if(current_task.stop_type == TASK_TYPE_MOTOR_SWITCH)
	{
		motor_enable = current_task.motor_enable;
		task_valid = 0;
	}
}

/* End user code. Do not edit comment generated here */
void R_MAIN_UserInit(void);

/***********************************************************************************************************************
* Function Name: main
* Description  : This function implements main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void main(void)
{
    R_MAIN_UserInit();
    /* Start user code. Do not edit comment generated here */
    task_array_init();
    current_time = millisecond();  
    while((millisecond()-current_time)<20000)mpu_tool();
    
    ArduCopter_Init();
    
    LED2_ON;
    SystemInitOK = 1;
    
    route_receive_start(); 
    
//    camera_task();
//    relay_task();    

    while (1U)
    {
         mpu_tool();
	 if(Control_100Hz)
	 {
	    Control_100Hz = 0;
	    fast_loop();
	    task_loop();
	 }
//       if(ADC_1Hz)
//       {
//          ADC_1Hz = 0;
//          R_ADC_Start();
//       }

	 Commad_Analyze();
#ifdef DEBUG_PRINT
         if(Debug_5Hz)
	 {
	    Debug_5Hz = 0;
//	    send_euler();
//          send_throttle();
//	    send_position();

	 }
	 
#endif
    }
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: R_MAIN_UserInit
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MAIN_UserInit(void)
{
    /* Start user code. Do not edit comment generated here */
    EI();
    BSP_Start();
    delay_nms(200);
    if(dmp_init())
       while(1){
                BEEP_ON; 
               }
//    delay_nms(8000);
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
