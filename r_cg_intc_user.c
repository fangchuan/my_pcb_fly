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
* File Name    : r_cg_intc_user.c
* Version      : CodeGenerator for RL78/G13 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : This file implements device driver for INTC module.
* Creation Date: 2015/8/14
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt INTP0 r_intc0_interrupt
#pragma interrupt INTKR r_key_interrupt
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_intc.h"
/* Start user code for include. Do not edit comment generated here */
#include "ahrs.h"
#include "delay.h"
#include "r_cg_port.h"
#include "task.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
uint8_t TASK = 0;
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: r_intc0_interrupt
* Description  : This function is INTP0 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_intc0_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    gyro_data_ready_cb();
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_key_interrupt
* Description  : This function is INTKR interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_key_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    delay_nms(10);
	//A to B
    if(P7.2 == 0)
    {	
		int i;
		int take_off_angle = 400;
		int move_forward_angle = 350;
		int fly_height = 50;
		
		TASK = TASK_1;
		BEEP_ON;
		
		task_array_init();
		//turn on camera 
		task_array_add_time_task(0, 0, 0, 0, 1000, 0, CAMERA_EN_ON);
		task_array_add_time_task(0, 0, 0, 0, 0, 0, CAMERA_EN_OFF);
		//wait to run away
		task_array_add_time_task(0, 0, 0, 0, 5000, 0, CAMERA_EN_OFF);
		//enable motor
		task_array_add_motor_switch_task(1);
		//take off
		task_array_add_time_task(take_off_angle, 0, 0, fly_height, 500, 0, CAMERA_EN_OFF);
		for(i = 0; i < 2; i++)
		{
			//find cross line whose width greater than 60
			task_array_add_line_width_task(move_forward_angle, 0, 0, fly_height, WAIT_GREATER, 60,0,CAMERA_EN_OFF);
			//wait for some time
			task_array_add_time_task(-200 * i, 0, 0, fly_height, 400, 0, CAMERA_EN_OFF);
			//find line whose width less than 60 
			task_array_add_line_width_task(move_forward_angle, 0, 0, fly_height, WAIT_LESS, 60,0,CAMERA_EN_OFF);
		}
//        //find cross line whose width greater than 60
//		task_array_add_line_width_task(move_forward_angle, 0, 0, fly_height, WAIT_GREATER, 60);
//		//pause
//		task_array_add_time_task(-500, 0, 0, fly_height, 500,0,0);
		//land
		task_array_add_time_task(-200, 0, 0, 0, 700,0, CAMERA_EN_OFF);
		//disable motor
		task_array_add_motor_switch_task(0);
		//turn off camera
		task_array_add_time_task(0, 0, 0, 0, 700, 0, CAMERA_EN_ON);
		task_array_add_time_task(0, 0, 0, 0, 0, 0, CAMERA_EN_OFF);
    }
    if(P7.3 == 0){
		//A to B-land-to A
	    int i;
		int take_off_angle = 400;
		int move_forward_angle = 350;
		int move_backward_angle = -400;
		int fly_height = 80;
        TASK = TASK_2;
		LED2_OFF;

		task_array_init();
		//wait to run away
		task_array_add_time_task(0, 0, 0, 0, 5000, RELAY_OFF, 1);
		//enable motor
		task_array_add_motor_switch_task(1);
		//take off
		task_array_add_time_task(take_off_angle, 0, 0, fly_height, 500, RELAY_OFF, 1);
		//move forward
		for(i = 0; i < 2; i++)
		{
			//find cross line whose width greater than 60
			task_array_add_line_width_task(move_forward_angle, 0, 0, fly_height, WAIT_GREATER, 60,RELAY_OFF,1);
			//wait for some time
			task_array_add_time_task(-200 * i, 0, 0, fly_height, 400, RELAY_OFF, 1);
			//find line whose width less than 60 
			task_array_add_line_width_task(move_forward_angle, 0, 0, fly_height, WAIT_LESS, 60,RELAY_OFF,1);
		}
		
		//land
		task_array_add_time_task(-800, 0, 0, 0, 3000, RELAY_ON, 1);
//		//move backward
//        task_array_add_time_task(move_backward_angle, 0, 0, fly_height, 4500, RELAY_OFF, 1);
		for(i = 0; i < 3; i++)
		{
			//find cross line whose width greater than 60
			task_array_add_line_width_task(move_backward_angle, 0, 0, fly_height, WAIT_GREATER, 60,RELAY_ON, 1);
			//wait for some time
			task_array_add_time_task(50 * i, 0, 0, fly_height, 400, RELAY_ON, 1);
			//find line whose width less than 60 
			task_array_add_line_width_task(move_backward_angle, 0, 0, fly_height, WAIT_LESS, 60,RELAY_ON,1);
		}

		//land
		task_array_add_time_task(0, 0, 0, 0, 1000, RELAY_ON, 0);
		//disable motor
		task_array_add_motor_switch_task(0);
    }
    if(P7.4 == 0){
		//A to B to A
	    int i;
		int take_off_angle = 400;
		int move_forward_angle = 350;
		int move_backward_angle = -400;
		int fly_height = 80;
        TASK = TASK_3;
		BEEP_ON;

		task_array_init();
		//wait to run away
		task_array_add_time_task(0, 0, 0, 0, 5000, RELAY_ON, 1);
		//enable motor
		task_array_add_motor_switch_task(1);
		//take off
		task_array_add_time_task(take_off_angle, 0, 0, fly_height, 500, RELAY_ON, 1);
		//move forward
		for(i = 0; i < 2; i++)
		{
			//find cross line whose width greater than 60
			task_array_add_line_width_task(move_forward_angle, 0, 0, fly_height, WAIT_GREATER, 60,RELAY_ON,1);
			//wait for some time
			task_array_add_time_task(-200 * i, 0, 0, fly_height, 400, RELAY_ON, 1);
			//find line whose width less than 60 
			task_array_add_line_width_task(move_forward_angle, 0, 0, fly_height, WAIT_LESS, 60,RELAY_ON,1);
		}
		
		//wait a few seconds
		task_array_add_time_task(-800, 0, 0, fly_height, 1000, RELAY_ON, 1);
		//drop the zhongwu
		task_array_add_time_task(-800, 0, 0, fly_height, 0, RELAY_OFF, 1);
//		//move backward
//        task_array_add_time_task(move_backward_angle, 0, 0, fly_height, 4500, RELAY_OFF, 1);
		for(i = 0; i < 3; i++)
		{
			//find cross line whose width greater than 60
			task_array_add_line_width_task(move_backward_angle, 0, 0, fly_height, WAIT_GREATER, 60,RELAY_OFF, 1);
			//wait for some time
			task_array_add_time_task(50 * i, 0, 0, fly_height, 400, RELAY_OFF, 1);
			//find line whose width less than 60 
			task_array_add_line_width_task(move_backward_angle, 0, 0, fly_height, WAIT_LESS, 60,RELAY_OFF,1);
		}

		//land
		task_array_add_time_task(0, 0, 0, 0, 1000, RELAY_OFF, 0);
		//disable motor
		task_array_add_motor_switch_task(0);
    }
    if(P7.5 == 0){
		int i;
		int take_off_angle = 400;
		int move_forward_angle = 350;
		int turn_left_angle = 9000;
		int fly_height = 45;
		TASK = TASK_4;
		LED2_OFF;
		
		task_array_init();
		//wait to run away
		task_array_add_time_task(0, 0, 0, 0, 5000, RELAY_OFF, 1);
		//enable motor
		task_array_add_motor_switch_task(1);
		//take off
		task_array_add_time_task(take_off_angle, 0, 0, fly_height, 1500, RELAY_OFF, 1);
		//turn head 90
		task_array_add_time_task(0, 0,turn_left_angle , fly_height, 0, RELAY_OFF, 1);
		//MOVE FORWARD
		task_array_add_time_task(move_forward_angle, 0, 0, fly_height, 4000, RELAY_OFF, 1);
		//turn head 90
		task_array_add_time_task(0, 0,turn_left_angle , fly_height, 0, RELAY_OFF, 1);
		//move forward
		task_array_add_time_task(move_forward_angle, 0, 0, fly_height, 2000, RELAY_OFF, 1);
		//turn head 90
		task_array_add_time_task(0, 0,turn_left_angle , fly_height, 0, RELAY_OFF, 1);
		//move forward
		task_array_add_time_task(move_forward_angle, 0, 0, fly_height, 4000, RELAY_OFF, 1);
		//turn head 90
		task_array_add_time_task(0, 0,turn_left_angle , fly_height, 0, RELAY_OFF, 1);
		//move forward
		task_array_add_time_task(move_forward_angle, 0,0 ,fly_height, 1000, RELAY_OFF, 1);
		//land
		task_array_add_time_task(-200, 0,0 ,0, 500, RELAY_OFF, 1);
		//enable motor
		task_array_add_motor_switch_task(0);
    }
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
