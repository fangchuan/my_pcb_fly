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
* File Name    : r_cg_serial.c
* Version      : CodeGenerator for RL78/G13 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : This file implements device driver for Serial module.
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
#include "r_cg_serial.h"
/* Start user code for include. Do not edit comment generated here */
#include "AP_Math.h"
#include "ahrs.h"
#include "r_cg_it.h"
#include "AP_MotorsQuard.h"
#include "stdlib.h"
#include "AP_OpticalFlow.h"
#include <stdio.h>
#include <stdarg.h>
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
volatile uint8_t * gp_uart0_tx_address;        /* uart0 transmit buffer address */
volatile uint16_t  g_uart0_tx_count;           /* uart0 transmit data number */
volatile uint8_t * gp_uart0_rx_address;        /* uart0 receive buffer address */
volatile uint16_t  g_uart0_rx_count;           /* uart0 receive data number */
volatile uint16_t  g_uart0_rx_length;          /* uart0 receive data length */
/* Start user code for global. Do not edit comment generated here */
char buffer[SHOW_BUFFER_SIZE];
extern _Q_ANGLE Q_ANGLE;
extern uint16_t PWM_InCh3;
extern uint16_t PWM_InCh1,PWM_InCh2,PWM_InCh3,PWM_InCh4;
extern int16_t  PWM_THROTTLE,PWM_HIGHT;
extern Vector3f _angle_ef_target;
extern int16_t  PWM_P,PWM_D;
extern uint16_t basis_throttle;

extern float vlon,vlat;
extern uint8_t surface_quality;
extern int16_t target_x_cm,taregt_y_cm;
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_SAU0_Create
* Description  : This function initializes the SAU0 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_SAU0_Create(void)
{
    SAU0EN = 1U;    /* supply SAU0 clock */
    NOP();
    NOP();
    NOP();
    NOP();
    SPS0 = _0001_SAU_CK00_FCLK_1 | _0010_SAU_CK01_FCLK_1;
    R_UART0_Create();
}

/***********************************************************************************************************************
* Function Name: R_UART0_Create
* Description  : This function initializes the UART0 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART0_Create(void)
{
    ST0 |= _0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON;    /* disable UART0 receive and transmit */
    STMK0 = 1U;    /* disable INTST0 interrupt */
    STIF0 = 0U;    /* clear INTST0 interrupt flag */
    SRMK0 = 1U;    /* disable INTSR0 interrupt */
    SRIF0 = 0U;    /* clear INTSR0 interrupt flag */
    SREMK0 = 1U;   /* disable INTSRE0 interrupt */
    SREIF0 = 0U;   /* clear INTSRE0 interrupt flag */
    /* Set INTST0 low priority */
    STPR10 = 1U;
    STPR00 = 1U;
    /* Set INTSR0 low priority */
    SRPR10 = 1U;
    SRPR00 = 1U;
    SMR00 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_TRIGGER_SOFTWARE |
            _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
    SCR00 = _8000_SAU_TRANSMISSION | _0000_SAU_INTSRE_MASK | _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 |
            _0007_SAU_LENGTH_8;
    SDR00 = _8800_UART0_TRANSMIT_DIVISOR;
    NFEN0 |= _01_SAU_RXD0_FILTER_ON;
    SIR01 = _0004_SAU_SIRMN_FECTMN | _0002_SAU_SIRMN_PECTMN | _0001_SAU_SIRMN_OVCTMN;    /* clear error flag */
    SMR01 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0100_SAU_TRIGGER_RXD | _0000_SAU_EDGE_FALL |
            _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
    SCR01 = _4000_SAU_RECEPTION | _0000_SAU_INTSRE_MASK | _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 |
            _0007_SAU_LENGTH_8;
    SDR01 = _8800_UART0_RECEIVE_DIVISOR;
    SO0 |= _0001_SAU_CH0_DATA_OUTPUT_1;
    SOL0 |= _0000_SAU_CHANNEL0_NORMAL;    /* output level normal */
    SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;    /* enable UART0 output */
    /* Set RxD0 pin */
    PM1 |= 0x02U;
    /* Set TxD0 pin */
    P1 |= 0x04U;
    PM1 &= 0xFBU;
}

/***********************************************************************************************************************
* Function Name: R_UART0_Start
* Description  : This function starts the UART0 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART0_Start(void)
{
    STIF0 = 0U;    /* clear INTST0 interrupt flag */
    STMK0 = 0U;    /* enable INTST0 interrupt */
    SRIF0 = 0U;    /* clear INTSR0 interrupt flag */
    SRMK0 = 0U;    /* enable INTSR0 interrupt */
    SREIF0 = 0U;   /* clear INTSRE0 interrupt flag */
    SREMK0 = 0U;   /* enable INTSRE0 interrupt */
    SO0 |= _0001_SAU_CH0_DATA_OUTPUT_1;    /* output level normal */
    SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;    /* enable UART0 output */
    SS0 |= _0002_SAU_CH1_START_TRG_ON | _0001_SAU_CH0_START_TRG_ON;    /* enable UART0 receive and transmit */
}

/***********************************************************************************************************************
* Function Name: R_UART0_Stop
* Description  : This function stops the UART0 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART0_Stop(void)
{
    ST0 |= _0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON;    /* disable UART0 receive and transmit */
    SOE0 &= ~_0001_SAU_CH0_OUTPUT_ENABLE;    /* disable UART0 output */
    STMK0 = 1U;    /* disable INTST0 interrupt */
    STIF0 = 0U;    /* clear INTST0 interrupt flag */
    SRMK0 = 1U;    /* disable INTSR0 interrupt */
    SRIF0 = 0U;    /* clear INTSR0 interrupt flag */
    SREMK0 = 1U;   /* disable INTSRE0 interrupt */
    SREIF0 = 0U;   /* clear INTSRE0 interrupt flag */
}

/***********************************************************************************************************************
* Function Name: R_UART0_Receive
* Description  : This function receives UART0 data.
* Arguments    : rx_buf -
*                    receive buffer pointer
*                rx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_UART0_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
    MD_STATUS status = MD_OK;

    if (rx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        g_uart0_rx_count = 0U;
        g_uart0_rx_length = rx_num;
        gp_uart0_rx_address = rx_buf;
    }

    return (status);
}

/***********************************************************************************************************************
* Function Name: R_UART0_Send
* Description  : This function sends UART0 data.
* Arguments    : tx_buf -
*                    transfer buffer pointer
*                tx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_UART0_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
    MD_STATUS status = MD_OK;

    if (tx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        gp_uart0_tx_address = tx_buf;
        g_uart0_tx_count = tx_num;
        STMK0 = 1U;    /* disable INTST0 interrupt */
        TXD0 = *gp_uart0_tx_address;
        gp_uart0_tx_address++;
        g_uart0_tx_count--;
        STMK0 = 0U;    /* enable INTST0 interrupt */
    }

    return (status);
}

/* Start user code for adding. Do not edit comment generated here */
MD_STATUS R_UART0_Send_Block(uint8_t * const tx_buf, uint16_t tx_num)
{
	int i, j;
	STMK0 = 1U;    /* disable INTST0 interrupt */
	for(i = 0; i < tx_num; i++)
	{
		TXD0 = *(tx_buf + i);
		//for(j = 5000; j > 0; j--);
		while((SSR00 & 0x40));
	}
	return MD_OK;
}

//MD_STATUS R_UART2_Send_Block(uint8_t * const tx_buf, uint16_t tx_num)
//{
//	int i, j;
//	STMK2 = 1U;    /* disable INTST0 interrupt */
//	for(i = 0; i < tx_num; i++)
//	{
//		TXD2 = *(tx_buf + i);
//		//for(j = 5000; j > 0; j--);
//		while((SSR00 & 0x40));
//	}
//	return MD_OK;
//}
//
//MD_STATUS R_UART2_Receive_Block(uint8_t * const rx_buf, uint16_t rx_num)
//{
//    MD_STATUS status = MD_OK;
//
//    if (rx_num < 1U)
//    {
//        status = MD_ARGERROR;
//    }
//    else
//    {
//        g_uart2_rx_count = 0U;
//        g_uart2_rx_length = rx_num;
//        gp_uart2_rx_address = rx_buf;
//    }
//    return (status);
//}

void send_euler(void)
{
     float_to_str(Q_ANGLE.X,5,buffer,20);
     R_UART0_Send_Block("Roll:",strlen("Roll:"));R_UART0_Send_Block(buffer,strlen(buffer));R_UART0_Send_Block("\r\n", strlen("\r\n"));
     float_to_str(Q_ANGLE.Y,5,buffer,20);
     R_UART0_Send_Block("Pitch:",strlen("Pitch:"));R_UART0_Send_Block(buffer,strlen(buffer));R_UART0_Send_Block("\r\n", strlen("\r\n"));
     float_to_str(Q_ANGLE.Z,5,buffer,20);
     R_UART0_Send_Block("Yaw:",strlen("Yaw:"));R_UART0_Send_Block(buffer,strlen(buffer));R_UART0_Send_Block("\r\n", strlen("\r\n"));
#ifdef print_gyro     
     float_to_str(Q_ANGLE.gyro[0],5,buffer,20);
     R_UART0_Send_Block("Roll_gyro:",strlen("Roll_gyro:"));R_UART0_Send_Block(buffer,strlen(buffer));R_UART0_Send_Block("\r\n", strlen("\r\n"));
     float_to_str(Q_ANGLE.gyro[1],5,buffer,20);
     R_UART0_Send_Block("Pitch_gyro:",strlen("Pitch_gyro:"));R_UART0_Send_Block(buffer,strlen(buffer));R_UART0_Send_Block("\r\n", strlen("\r\n"));
     float_to_str(Q_ANGLE.gyro[2],5,buffer,20);
     R_UART0_Send_Block("Yaw_gyro:",strlen("Yaw_gyro:"));R_UART0_Send_Block(buffer,strlen(buffer));R_UART0_Send_Block("\r\n", strlen("\r\n"));
#endif
}
void send_throttle (void)
{
     float_to_str((float)PWM_InCh3,2,buffer,20);
     R_UART0_Send_Block("PWM_InCh3:",strlen("PWM_InCh3:"));R_UART0_Send_Block(buffer,strlen(buffer));R_UART0_Send_Block("\r\n", strlen("\r\n"));
}
void send_nowtime(void)
{
     float_to_str((float)millisecond(),1,buffer,20);
     R_UART0_Send_Block("systemtime:",strlen("systemtime:"));R_UART0_Send_Block(buffer,strlen(buffer));R_UART0_Send_Block("\r\n", strlen("\r\n"));
}

void return_angle(void)
{
     sprintf(buffer,"BE:%d$%d$%d$\r\n",(int)Q_ANGLE.X ,(int)Q_ANGLE.Y  ,(int)Q_ANGLE.Z  );
     R_UART0_Send_Block(buffer,strlen(buffer));
}
void return_hight(int h)
{
     sprintf(buffer,"BH:%d$\r\n",h);
     R_UART0_Send_Block(buffer,strlen(buffer));
}
extern  int16_t Motor_1,Motor_2,Motor_3,Motor_4;
void return_motor(void)
{
     sprintf(buffer,"BM:%d$%d$%d$%d\r\n",Motor_1, Motor_2,Motor_3,Motor_4); 
     R_UART0_Send_Block(buffer,strlen(buffer));
}
void return_input(void)
{
     sprintf(buffer,"BI:%d$%d$%d$%d\r\n",PWM_InCh1,PWM_InCh2,PWM_InCh3,PWM_InCh4);
     R_UART0_Send_Block(buffer,strlen(buffer));
}
void return_target_angle(void)
{
     sprintf(buffer,"BT:%d$%d$%d\r\n",(int)_angle_ef_target.x ,(int)_angle_ef_target.y ,(int)_angle_ef_target.z);
     R_UART0_Send_Block(buffer,strlen(buffer));
}
void return_gyro(void)
{
     sprintf(buffer,"BG:%d$%d$%d$\r\n",(int)(Q_ANGLE.gyro[0]*100),(int)(Q_ANGLE.gyro[1]*100),(int)(Q_ANGLE.gyro[2]*100));
     R_UART0_Send_Block(buffer,strlen(buffer));
}
void return_accel(void)
{
     sprintf(buffer,"BA:%d$%d$%d$\r\n",(int)(Q_ANGLE.accel[0]*100),(int)(Q_ANGLE.accel[1]*100),(int)(Q_ANGLE.accel[2]*100));
     R_UART0_Send_Block(buffer,strlen(buffer));
}
void return_hight_pwm(void)
{
     sprintf(buffer,"BP:%d$%d\r\n",PWM_P,PWM_D);
     R_UART0_Send_Block(buffer,strlen(buffer));
}
void return_basis_throttle(void)
{
     sprintf(buffer,"BP:%d$%d\r\n",basis_throttle,PWM_P);
     R_UART0_Send_Block(buffer,strlen(buffer));
}

extern int16_t POS_X_P,POS_Y_P;
void return_pos_pid(void)
{
     sprintf(buffer,"BE:%d$%d$%d$\r\n",POS_X_P ,POS_Y_P ,0 );
     R_UART0_Send_Block(buffer,strlen(buffer));
}
void return_target_pos(void)
{
     sprintf(buffer,"BT:%d$%d\r\n",target_x_cm ,target_y_cm);
     R_UART0_Send_Block(buffer,strlen(buffer));
}
extern int8_t blackline_width;
extern uint16_t blackline_center;
extern uint8_t leftblack,rightblack;
extern uint16_t threshold;
void return_blackline(void)
{
     sprintf(buffer,"BP:%d$%d\r\n",blackline_width,blackline_center);
//     sprintf(buffer,"BP:%d$%d\r\n",leftblack,rightblack);
     R_UART0_Send_Block(buffer,strlen(buffer));	 
}
void return_threshold(void)
{
     sprintf(buffer,"BH:%d$\r\n",threshold);
     R_UART0_Send_Block(buffer,strlen(buffer));
}

void debug_print(void) 
{ 
	R_UART0_Send_Block(buffer, strlen(buffer));
}

void debug_prints(const char *buf)
{
	R_UART0_Send_Block(buf, strlen(buf));
}

/* End user code. Do not edit comment generated here */
