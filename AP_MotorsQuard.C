#include "AP_MotorsQuard.h"
#include "r_cg_adc.h"
#include "r_cg_intc.h"

extern float current_voltage;
extern uint8_t TASK;
int16_t PWM_PITCH,PWM_ROLL,PWM_YAW,PWM_THROTTLE;
int16_t PWM_HIGHT;
uint8_t motor_enable = 0;

    // set_roll, set_pitch, set_yaw, set_throttle
void   _motors_set_roll(float roll_in)                       // range -4500 ~ 4500
{                    
       roll_in *= 0.1;
       PWM_ROLL = constrain_float(roll_in,-500.0f,500.0f);

}     

void   _motors_set_pitch(float pitch_in)                     // range -4500 ~ 4500
{                  
       pitch_in *= 0.1;
       PWM_PITCH = constrain_float(pitch_in,-500.0f,500.0f);  

}    

void   _motors_set_yaw(float yaw_in)                         // range -4500 ~ 4500
{                     
      yaw_in *= 0.1;
      PWM_YAW = constrain_float(yaw_in , -100.0f,100.0f);

}


void  _motors_set_high(float high_in)
{
	PWM_HIGHT =(int)high_in;	
	PWM_HIGHT = constrain(PWM_HIGHT,-100,100);
}

void   _motors_set_throttle(int16_t throttle_in)           
{          

       PWM_THROTTLE = constrain(throttle_in ,MIN_THROTTLE,MAX_THROTTLE);	

}    

int16_t Motor_1,Motor_2,Motor_3,Motor_4;
uint16_t basis_throttle=450;
extern _Q_ANGLE Q_ANGLE;
void PWM_Reload(void)
{
	if(fabs(Q_ANGLE.X) >= 45 || fabs(Q_ANGLE.Y)>=45 )
	{
			motor_enable = 0;
	}
	if(TASK != 0)
	{
        if(PWM_THROTTLE >= 1050)
		{
           PWM_THROTTLE = 1050;
		}
		   //motor_output rang from 1000~2000	
		Motor_1 = PWM_THROTTLE + PWM_ROLL - PWM_PITCH + PWM_YAW + PWM_HIGHT + basis_throttle;//CW //basis throttle to take-off
		Motor_2 = PWM_THROTTLE - PWM_ROLL - PWM_PITCH - PWM_YAW + PWM_HIGHT + basis_throttle;//CWW
		Motor_3 = PWM_THROTTLE - PWM_ROLL + PWM_PITCH + PWM_YAW + PWM_HIGHT + basis_throttle;//CW
		Motor_4 = PWM_THROTTLE + PWM_ROLL + PWM_PITCH - PWM_YAW + PWM_HIGHT + basis_throttle;//CWW
	}
	else
	{
		Motor_1 = PWM_THROTTLE + PWM_ROLL - PWM_PITCH + PWM_YAW;
		Motor_2 = PWM_THROTTLE - PWM_ROLL - PWM_PITCH - PWM_YAW;
		Motor_3 = PWM_THROTTLE - PWM_ROLL + PWM_PITCH + PWM_YAW;
		Motor_4 = PWM_THROTTLE + PWM_ROLL + PWM_PITCH - PWM_YAW;
	}

	Motor_1= constrain(Motor_1,MINPWM,MAXPWM);
	Motor_2= constrain(Motor_2,MINPWM,MAXPWM);
	Motor_3= constrain(Motor_3,MINPWM,MAXPWM);
	Motor_4= constrain(Motor_4,MINPWM,MAXPWM);

	if(/*!motor_enable ||*/ PWM_THROTTLE < 1050 )
	{
		Motor_1 =1000;
		Motor_2 =1000;
		Motor_3 =1000;
		Motor_4 =1000;
	}

	//TDRx/40000=duty
	TDR01=Motor_1 << 4;
	TDR02=Motor_2 << 4;
	TDR03=Motor_3 << 4;
	TDR04=Motor_4 << 4;
	
}