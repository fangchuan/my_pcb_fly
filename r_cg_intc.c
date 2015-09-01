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
* File Name    : r_cg_intc.c
* Version      : CodeGenerator for RL78/G13 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : This file implements device driver for INTC module.
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
#include "r_cg_intc.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_INTC_Create
* Description  : This function initializes INTP module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_INTC_Create(void)
{
    PMK0 = 1U;    /* disable INTP0 operation */
    PIF0 = 0U;    /* clear INTP0 interrupt flag */
    PMK1 = 1U;    /* disable INTP1 operation */
    PIF1 = 0U;    /* clear INTP1 interrupt flag */
    PMK2 = 1U;    /* disable INTP2 operation */
    PIF2 = 0U;    /* clear INTP2 interrupt flag */
    PMK3 = 1U;    /* disable INTP3 operation */
    PIF3 = 0U;    /* clear INTP3 interrupt flag */
    PMK4 = 1U;    /* disable INTP4 operation */
    PIF4 = 0U;    /* clear INTP4 interrupt flag */
    PMK5 = 1U;    /* disable INTP5 operation */
    PIF5 = 0U;    /* clear INTP5 interrupt flag */
    PMK6 = 1U;    /* disable INTP6 operation */
    PIF6 = 0U;    /* clear INTP6 interrupt flag */
    PMK7 = 1U;    /* disable INTP7 operation */
    PIF7 = 0U;    /* clear INTP7 interrupt flag */
    PMK8 = 1U;    /* disable INTP8 operation */
    PIF8 = 0U;    /* clear INTP8 interrupt flag */
    PMK9 = 1U;    /* disable INTP9 operation */
    PIF9 = 0U;    /* clear INTP9 interrupt flag */
    PMK10 = 1U;    /* disable INTP10 operation */
    PIF10 = 0U;    /* clear INTP10 interrupt flag */
    PMK11 = 1U;    /* disable INTP11 operation */
    PIF11 = 0U;    /* clear INTP11 interrupt flag */
    /* Set INTP0 high priority */
    PPR10 = 0U;
    PPR00 = 0U;
    EGN0 = _01_INTP0_EDGE_FALLING_SEL;
}

/***********************************************************************************************************************
* Function Name: R_INTC0_Start
* Description  : This function clears INTP0 interrupt flag and enables interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_INTC0_Start(void)
{
    PIF0 = 0U;    /* clear INTP0 interrupt flag */
    PMK0 = 0U;    /* enable INTP0 interrupt */
}

/***********************************************************************************************************************
* Function Name: R_INTC0_Stop
* Description  : This function disables INTP0 interrupt and clears interrupt flag.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_INTC0_Stop(void)
{
    PMK0 = 1U;    /* disable INTP0 interrupt */
    PIF0 = 0U;    /* clear INTP0 interrupt flag */
}

/***********************************************************************************************************************
* Function Name: R_KEY_Create
* Description  : This function initializes the key return module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_KEY_Create(void)
{
    volatile uint8_t  w_count;

    KRMK = 1U;  /* disable INTKR operation */
    KRIF = 0U;  /* clear INTKR interrupt flag */
    /* Set INTKR low priority */
    KRPR1 = 1U;
    KRPR0 = 1U;
    KRM = _00_KR0_SIGNAL_DETECT_OFF | _00_KR1_SIGNAL_DETECT_OFF | _04_KR2_SIGNAL_DETECT_ON | _08_KR3_SIGNAL_DETECT_ON |
          _10_KR4_SIGNAL_DETECT_ON | _20_KR5_SIGNAL_DETECT_ON | _00_KR6_SIGNAL_DETECT_OFF | _00_KR7_SIGNAL_DETECT_OFF;
    /* Set KR2 pin */
    PM7 |= 0x04U;
    /* Set KR3 pin */
    PM7 |= 0x08U;
    /* Set KR4 pin */
    PM7 |= 0x10U;
    /* Set KR5 pin */
    PM7 |= 0x20U;

    /* Change the waiting time according to the system */
    for (w_count = 0U; w_count < KEY_WAITTIME; w_count++)
    {   
        NOP();
    }
}

/***********************************************************************************************************************
* Function Name: R_KEY_Start
* Description  : This function clears INTKR interrupt flag and enables interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_KEY_Start(void)
{
    KRIF = 0U;  /* clear INTKR interrupt flag */
    KRMK = 0U;  /* enable INTKR operation */
}

/***********************************************************************************************************************
* Function Name: R_KEY_Stop
* Description  : This function disables INTKR interrupt and clears interrupt flag.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_KEY_Stop(void)
{
    KRMK = 1U;  /* disable INTKR operation */
    KRIF = 0U;  /* clear INTKR interrupt flag */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
