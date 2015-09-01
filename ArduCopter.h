#ifndef _ARDUCOPTER_H
#define _ARDUCOPTER_H

#include <stdio.h>
#include <stdarg.h>
#include "AP_Math.h"    	// ArduPilot Mega Vector/Matrix math Library
#include "AC_PID.h"            // PID library
#include "AP_MotorsQuard.h"         // AP Motors library
#include "ahrs.h"
#include "RC_Channel.h"
#include "Config.h"
#include "r_cg_serial.h"

#define HEIGHT_UPDATE_PERIOD 2
#define OPTICAL_UPDATE_PERIOD 5
#define CCD_UPDATE_PERIOD     3

extern AC_PID   STABILIZE_ROLL_PID;
extern AC_PID   STABILIZE_PITCH_PID;
extern AC_PID   STABILIZE_YAW_PID;
extern AC_PID   STABILIZE_HIGHT_PID;
extern AC_PID   RATE_ROLL_PID;
extern AC_PID   RATE_PITCH_PID;
extern AC_PID   RATE_YAW_PID;
extern AC_PID   STABILIZE_POINT_X_PID;
extern AC_PID   STABILIZE_POINT_Y_PID;
extern Vector3f    _angle_ef_target;
extern Vector3f    _rate_ef_target;
extern Vector3f    _rate_bf_target;
extern Vector3f    accel;
extern uint16_t    hight_target;
extern int16_t    target_x_cm ,target_y_cm ;
void fast_loop(void);
float rate_bf_to_motor_roll(float rate_target_cds);
float rate_bf_to_motor_pitch(float rate_target_cds);
float rate_bf_to_motor_yaw(float rate_target_cds);
float rate_to_motor_high(float tar_high_rate);
void  set_servos_4(void);
void  get_stabilize_roll(long target_angle);
void  get_stabilize_pitch(long target_angle);
void  get_stabilize_yaw(long target_angle);
void get_stabilize_hight(long target_hight);
void  ArduCopter_Init(void);
void do_task_first(void);
void do_task_second(void);
void camera_task(void);
void relay_task(void);
void system_start(void);
void system_halt(void);

extern void arducopter_set_target(Vector3f *angle, uint16_t height,uint8_t relay,uint8_t camera);

#endif