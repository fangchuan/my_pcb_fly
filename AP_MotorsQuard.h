#ifndef __AP_MOTORS_QUARD__
#define __AP_MOTORS_QUARD__
#include "r_cg_macrodriver.h"
#include "AP_Math.h"
#include "ArduCopter.h"
#include "Config.h"
#include "r_cg_serial.h"
#include "RC_Channel.h"
#include "r_cg_port.h"

extern 	int16_t Motor_1,Motor_2,Motor_3,Motor_4;
extern  uint16_t basis_throttle;
extern  uint8_t motor_enable;

#define    MIN_THROTTLE  1000//
#define    MAX_THROTTLE  1950
#define    MINPWM        1000
#define    MAXPWM        1950

#define    AP_MOTORS_MATRIX_MOTOR_CW   -1  
#define    AP_MOTORS_MATRIX_MOTOR_CCW  1  

#define    AP_MOTORS_MOT_1             0
#define    AP_MOTORS_MOT_2             1
#define    AP_MOTORS_MOT_3             2
#define    AP_MOTORS_MOT_4             3

extern int16_t PWM_PITCH,PWM_ROLL,PWM_YAW,PWM_THROTTLE,PWM_HIGHT;
void   _motors_set_roll(float roll_in);
void   _motors_set_pitch(float pitch_in) ;
void   _motors_set_yaw(float yaw_in);
void   _motors_set_high(float high_in);
void   _motors_set_throttle(int16_t throttle_in)  ;
void   PWM_Reload(void);


#endif

