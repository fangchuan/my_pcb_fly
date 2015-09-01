#include "r_cg_macrodriver.h"
#include "ArduCopter.h"
#include "r_cg_port.h"
#include "r_cg_timer.h"
#include "r_cg_intc.h"
#include "ahrs.h"
#include "ccd.h"
#include "AP_OpticalFlow.h"
#include "AP_OpticalFlow_ADNS3080.h"

Vector3f current_omega = {0,0,0};
//Vector3f current_accel = {0,0,0};

float cos_pitch_x=1,cos_roll_x=1,sin_yaw_y,cos_yaw_x=1,sin_pitch,sin_roll;
static float G_Dt = 0.01;

AC_PID   STABILIZE_ROLL_PID;
AC_PID   STABILIZE_PITCH_PID;
AC_PID   STABILIZE_YAW_PID;
AC_PID   RATE_ROLL_PID;
AC_PID   RATE_PITCH_PID;
AC_PID   RATE_YAW_PID;
AC_PID   STABILIZE_HIGHT_PID;
AC_PID   STABILIZE_POINT_X_PID;
AC_PID   STABILIZE_POINT_Y_PID;

Vector3f    _angle_ef_target;
Vector3f    _rate_ef_target;
Vector3f    _rate_bf_target;

uint16_t    hight_target;
int16_t    target_x_cm ,target_y_cm ;
float rate_bf_target_hight=0;

RC_Channel    rc;

extern _Q_ANGLE Q_ANGLE;
extern Vector3f omega;
extern Vector3f accel;
extern float current_hight;
extern uint8_t TASK;

// init_targets - resets target angles to current angles
void init_targets()
{
    // set earth frame angle targets to current lean angles
    _angle_ef_target.x = 0;
    _angle_ef_target.y = 0;
    _angle_ef_target.z = Q_ANGLE.Z *100;
    
    hight_target=0;
    target_x_cm = 0;
    target_y_cm = 0;
}

void arducopter_set_target(Vector3f *angle, uint16_t height,uint8_t relay,uint8_t camera)
{
	_angle_ef_target.x = (*angle).x;
	_angle_ef_target.y = (*angle).y;
	_angle_ef_target.z += (*angle).z;
	hight_target = height;
	RELAY_CONTROL = relay;
	CAMERA_CONTROL = camera;

}


void ArduCopter_Init(void)
{
	init_targets();
	
	vector3f_init(&_rate_ef_target,0,0,0);
	vector3f_init(&_rate_bf_target,0,0,0);
	
	rc_channel_init(&rc);
	
	PI_init(&STABILIZE_ROLL_PID, STABILIZE_ROLL_P, STABILIZE_ROLL_I, STABILIZE_ROLL_IMAX);
	PI_init(&STABILIZE_PITCH_PID, STABILIZE_PITCH_P, STABILIZE_PITCH_I, STABILIZE_PITCH_IMAX);
	PI_init(&STABILIZE_YAW_PID, STABILIZE_YAW_P, STABILIZE_YAW_I, STABILIZE_YAW_IMAX);
	
	PID_Filter_init(&RATE_ROLL_PID, RATE_ROLL_P, RATE_ROLL_I, RATE_ROLL_D, RATE_ROLL_IMAX,AC_PID_D_TERM_FILTER);
	PID_Filter_init(&RATE_PITCH_PID, RATE_PITCH_P, RATE_PITCH_I, RATE_PITCH_D, RATE_PITCH_IMAX,AC_PID_D_TERM_FILTER);
	PID_Filter_init(&RATE_YAW_PID, RATE_YAW_P, RATE_YAW_I, RATE_YAW_D, RATE_YAW_IMAX,AC_PID_D_TERM_FILTER);

	PID_Filter_init(&STABILIZE_HIGHT_PID,STABILIZE_HIGH_P,STABILIZE_HIGH_I,STABILIZE_HIGH_D,STABILIZE_HIGH_IMAX,AC_PID_D_TERM_FILTER);

	PID_Filter_init(&STABILIZE_POINT_X_PID,STABILIZE_POS_X_P,STABILIZE_POS_X_I,STABILIZE_POS_X_D,STABILIZE_POS_X_IMAX,AC_PID_D_TERM_FILTER);
    PID_Filter_init(&STABILIZE_POINT_Y_PID,STABILIZE_POS_Y_P,STABILIZE_POS_Y_I,STABILIZE_POS_Y_D,STABILIZE_POS_Y_IMAX,AC_PID_D_TERM_FILTER);

}

void read_AHRS(void)
{
	Matrix3f *temp_matrix = &Q_ANGLE.dcm_matrixs;
	float temp;
   
	current_omega.x = omega.x;
	current_omega.y = omega.y;
	current_omega.z = omega.z;
	
//	current_accel.x = accel.x;
//	current_accel.y = accel.y;
//	current_accel.z = accel.z;  //m/s^2
	
	//rotate
    matrix3f_from_euler(temp_matrix,current_omega.x ,current_omega.y,current_omega.z);//update current rate

	//update triangle value in order to calculate current goal jiaosudu 
	cos_pitch_x = safe_sqrt(1 - (temp_matrix->c.x * temp_matrix->c.x));     // level = 1
	cos_roll_x = temp_matrix->c.z / cos_pitch_x;                       // level = 1
	cos_pitch_x = (float)constrain(cos_pitch_x, 0, 1.0);
	cos_roll_x  = (float)constrain(cos_roll_x, -1.0, 1.0);

	sin_yaw_y = temp_matrix->b.x; // 1y = north
	cos_yaw_x = temp_matrix->a.x; // 0x = north
	temp = (float)sqrt(sin_yaw_y * sin_yaw_y + cos_yaw_x * cos_yaw_x);
	sin_yaw_y /= temp;
	cos_yaw_x /= temp;
	sin_yaw_y = (float)constrain(sin_yaw_y, -1.0, 1.0);
	cos_yaw_x  = (float)constrain(cos_yaw_x, -1.0, 1.0);

	sin_pitch = -temp_matrix->c.x;
	sin_roll = temp_matrix->c.y / cos_pitch_x;
  
}
//
// earth-frame <-> body-frame conversion functions
//
// frame_conversion_ef_to_bf - converts earth frame vector to body frame vector
void frame_conversion_ef_to_bf( Vector3f *ef_vector, Vector3f* bf_vector)
{
    // convert earth frame rates to body frame rates
    bf_vector->x = ef_vector->x - sin_pitch * ef_vector->z;
    bf_vector->y = cos_roll_x  * ef_vector->y + sin_roll * cos_pitch_x * ef_vector->z;
    bf_vector->z = -sin_roll * ef_vector->y + cos_pitch_x * cos_roll_x * ef_vector->z;
}

//
void update_altitude()
{
     Ultrasnio_Trigger = 1;
     delay_nus(6);
     Ultrasnio_Trigger = 0;
}
// throttle_loop - should be run at 50 hz
// ---------------------------
void throttle_loop()
{
	static uint8_t i=0;

	if(++i == HEIGHT_UPDATE_PERIOD){
	     i = 0;
	     update_altitude();
         if(TASK != 0)
		 {
            get_stabilize_hight(hight_target);
		 }
	}
}
int16_t POS_X_P,POS_Y_P;
extern uint16_t blackline_center;
void  update_line_control(void)
{
     float  p,i,d,pid;
     float  error_x = (float)blackline_center - 64;
     error_x = constrain_float(error_x,-64.0,64.0);

     p = get_p(error_x ,&STABILIZE_POINT_X_PID);
     POS_X_P = p;
     i = get_i(error_x, CCD_UPDATE_PERIOD*G_Dt ,&STABILIZE_POINT_X_PID);
     
     d = get_d(error_x, CCD_UPDATE_PERIOD*G_Dt ,&STABILIZE_POINT_X_PID);
     POS_Y_P = d;
     
     pid = constrain_float(p+i+d,-6.0f,6.0f);

     _angle_ef_target.x = pid *100;

}

//read ccd sensor for Identify the runway
void read_ccd (void)
{
     static uint8_t i =0;
     if(++i == CCD_UPDATE_PERIOD){
            i = 0;
            update_ccd();
			if(TASK != 0)
			{
	           update_line_control();
			}
     }
}
// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void stabilize_run()
{
	
	get_stabilize_yaw(_angle_ef_target.z);

	get_stabilize_roll(_angle_ef_target.x);
	
	get_stabilize_pitch(_angle_ef_target.y);

}
//
// rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
//      should be called at 100hz or more
//
void rate_controller_run(void)
{
    // call rate controllers and send output to motors object
    // To-Do: should the outputs from get_rate_roll, pitch, yaw be int16_t which is the input to the motors library?
    // To-Do: skip this step if the throttle out is zero?
    frame_conversion_ef_to_bf(&_rate_ef_target, &_rate_bf_target);
	
    _motors_set_roll(rate_bf_to_motor_roll(_rate_bf_target.x));
    _motors_set_pitch(rate_bf_to_motor_pitch(_rate_bf_target.y));
    _motors_set_yaw(rate_bf_to_motor_yaw(_rate_bf_target.z));
    _motors_set_high(rate_bf_target_hight);
} 
// Main loop - 100hz
void fast_loop()
{

    // IMU DCM Algorithm
    // --------------------
    read_AHRS();

   // run the attitude controllers
    stabilize_run();
    // run low level rate controllers that only require IMU data
    rate_controller_run();

    // write out the servo PWM values
    set_servos_4();

    // Inertial Nav
    read_ccd();

    throttle_loop();

    // rc_loops - reads user input from transmitter/receiver
    rc_loop();

}

//
// body-frame angle controller
//
void get_stabilize_roll(long target_angle)
{
	long target_rate;
	long i_stab;

	// angle error
	target_angle = wrap_180(target_angle - Q_ANGLE.X*100);

	// limit the error we're feeding to the PID
	target_angle = constrain(target_angle, -4500, 4500);
	// convert to desired Rate:
	target_rate = get_p(target_angle ,&STABILIZE_ROLL_PID);

//	i_stab;
	if(fabs(Q_ANGLE.X) < 5) {
		target_angle = constrain(target_angle, -500, 500);
		i_stab = get_i(target_angle, G_Dt ,&STABILIZE_ROLL_PID);
	}else{
		i_stab = (long)STABILIZE_ROLL_PID._integrator;
	}
	// set targets for rate controller
	_rate_ef_target.x  = target_rate + i_stab;
}

void get_stabilize_pitch(long target_angle)
{
	long target_rate;
	long i_stab;

	// angle error
	target_angle = wrap_180(target_angle - Q_ANGLE.Y *100);

	// limit the error we're feeding to the PID
	target_angle = constrain(target_angle, -4500, 4500);

	// convert to desired Rate:
	target_rate = get_p(target_angle ,&STABILIZE_PITCH_PID);

//	i_stab;
	if(fabs(Q_ANGLE.Y ) < 5) {
		target_angle = constrain(target_angle, -500, 500);
		i_stab = get_i( target_angle, G_Dt,&STABILIZE_PITCH_PID);
	}else{
		i_stab = (long)STABILIZE_PITCH_PID._integrator;
	}

	// set targets for rate controller
	_rate_ef_target.y = target_rate + i_stab;

}

void get_stabilize_yaw(long target_angle)
{
	long target_rate,i_term;
	long angle_error;

	// angle error
	angle_error = wrap_180(target_angle - Q_ANGLE.Z *100);

	// limit the error we're feeding to the PID
	angle_error = constrain(angle_error, -4500, 4500);
	
	// convert angle error to desired Rate:
	target_rate = get_p( angle_error,&STABILIZE_YAW_PID);
	i_term = get_i(angle_error, G_Dt ,&STABILIZE_YAW_PID);

	// set targets for rate controller
	_rate_ef_target.z  = target_rate + i_term;

}
int16_t PWM_P,PWM_D;
//body frame hight controller
void get_stabilize_hight(long target_hight)
{
	float p, i,d;
        float error;
	// hight error
	error = target_hight - (long)current_hight;
//	return_hight((int)current_hight);
	// limit the error we're feeding to the PID
	error = constrain(error, -40, 40);
	// convert to desired Rate:
	p = get_p(error ,&STABILIZE_HIGHT_PID);
    PWM_P = p;
//        i = get_integrator(&STABILIZE_HIGHT_PID);
    if(fabs(error) < 40){
	   i = get_i(error, HEIGHT_UPDATE_PERIOD*G_Dt ,&STABILIZE_HIGHT_PID);
	}
//	else
//	   i = STABILIZE_HIGHT_PID._integrator;
        d = get_d_filter(error, HEIGHT_UPDATE_PERIOD*G_Dt ,&STABILIZE_HIGHT_PID);
	PWM_D = d;
	// set targets for rate controller
        rate_bf_target_hight = constrain_float ((p+i+d),-200.0f,200.0f);
}

//
// body-frame rate controller
//
// rate_bf_to_motor_roll - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float rate_bf_to_motor_roll(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (current_omega.x * RAD_TO_DEG_X100);

    // calculate error and call pid controller
    rate_error = rate_target_cds - current_rate;
    p = get_p(rate_error , &RATE_ROLL_PID);

    // get i term
    i = get_integrator(&RATE_ROLL_PID);

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
//    if (((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = get_i(rate_error, G_Dt ,&RATE_ROLL_PID);
//    }

    // get d term
    d = get_d(rate_error, G_Dt ,&RATE_ROLL_PID);

    // constrain output and return
    return constrain_float((p+i+d), -5000.0f, 5000.0f);

    // To-Do: allow logging of PIDs?
}

// rate_bf_to_motor_pitch - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float rate_bf_to_motor_pitch(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (current_omega.y * RAD_TO_DEG_X100);

    // calculate error and call pid controller
    rate_error = rate_target_cds - current_rate;
    p = get_p(rate_error ,&RATE_PITCH_PID);

    // get i term
    i = get_integrator(&RATE_PITCH_PID);

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
//    if ( ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = get_i(rate_error, G_Dt,&RATE_PITCH_PID);
//    }

    // get d term
    d = get_d(rate_error, G_Dt,&RATE_PITCH_PID);

    // constrain output and return
    return constrain_float((p+i+d), -5000.0f, 5000.0f);

    // To-Do: allow logging of PIDs?
}

// rate_bf_to_motor_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float rate_bf_to_motor_yaw(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (current_omega.z * RAD_TO_DEG_X100);

    // calculate error and call pid controller
    rate_error  = rate_target_cds - current_rate;

    // separately calculate p, i, d values for logging
    p = get_p(rate_error ,&RATE_YAW_PID);

    // get i term
    i = get_integrator(&RATE_YAW_PID);

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if ( ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = get_i(rate_error, G_Dt ,&RATE_YAW_PID);
    }

    // get d value
    d = get_d(rate_error, G_Dt ,&RATE_YAW_PID);

    // constrain output and return
    return constrain_float((p+i+d), -5000.0f, 5000.0f);

    // To-Do: allow logging of PIDs?
}

void set_servos_4(void)
{
     _motors_set_throttle(rc.throttle);
     PWM_Reload();
}

//camera_task
void camera_task(void)
{
     CAMERA_CONTROL =0;
     delay_nms(1000);
     CAMERA_CONTROL = 1;
}
//relay task
void relay_task(void)
{
     RELAY_CONTROL = 1;
     delay_nms(2000);
     RELAY_CONTROL = 0;
}
extern uint8_t SystemInitOK;
extern int16_t PWM_THROTTLE;
void system_start(void)
{
     SystemInitOK = 1;
     PWM_THROTTLE = 1000;
}

void system_halt(void)
{
     SystemInitOK = 0;
     PWM_THROTTLE = 1000;
}