/// @file	AC_PID.cpp
/// @brief	Generic PID algorithm
#include "r_cg_macrodriver.h"
#include "AP_Math.h"
#include "AC_PID.h"
#include "Config.h"

void PI_init(AC_PID *apm_pi, float kp, float ki, float imax)
{
     apm_pi->_kp = kp;
     apm_pi->_ki = ki;
     apm_pi->_imax = imax;
     apm_pi->_integrator = 0;
}

//PID param initial
void PID_init(AC_PID * ap_pid, float initial_p, float initial_i, float initial_d, float initial_imax)
{
	ap_pid->_kp = initial_p;
	ap_pid->_ki = initial_i;
	ap_pid->_kd = initial_d;
	ap_pid->_imax = fabsf(initial_imax);

	ap_pid->_integrator = 0;
	ap_pid->_last_derivative = 0;
}

void PID_Filter_init(AC_PID * ap_pid, float initial_p, float initial_i, float initial_d, float initial_imax,float lpf)
{
	ap_pid->_kp = initial_p;
	ap_pid->_ki = initial_i;
	ap_pid->_kd = initial_d;
	ap_pid->_imax = fabsf(initial_imax);

	ap_pid->_integrator = 0;
	ap_pid->_last_derivative = 0;
	ap_pid->_d_lpf_alpha = lpf;
}
float get_p(float error, AC_PID*pid)
{
    return (float)error * pid-> _kp;
}

float get_i(float error, float dt ,AC_PID* pid)
{
    if((pid->_ki != 0) && (dt != 0)) {
        pid->_integrator += ((float)error * pid->_ki) * dt;
        if (pid->_integrator < -pid->_imax) {
            pid->_integrator = -pid->_imax;
        } else if (pid->_integrator > pid->_imax) {
            pid->_integrator = pid->_imax;
        }
        return pid->_integrator;
    }
    return 0;
}

float       get_integrator(AC_PID*pid)
{
      return pid->_integrator;
}

void        set_integrator(float i,AC_PID*pid) 
{
      pid->_integrator = i;
}

float get_d(float input, float dt ,AC_PID* pid)
{
    if ((pid->_kd != 0) && (dt != 0)) {
        float derivative;
		if (isnan(pid->_last_derivative)) {
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change			
			derivative = 0;
			pid->_last_derivative = 0;
		} else {
			// calculate instantaneous derivative
			derivative = (input - pid->_last_input) / dt;
		}

        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy
        derivative = pid->_last_derivative + pid->_d_lpf_alpha * (derivative - pid->_last_derivative);

        // update state
        pid->_last_input             = input;
        pid->_last_derivative    = derivative;

        // add in derivative component
        return pid->_kd * derivative;
    }
    return 0;
}
float get_d_filter(float input, float dt ,AC_PID* pid)
{
      float derivative;
      derivative = (input - pid->_last_input) / dt;
      
      derivative = pid->_last_derivative + (dt/(AC_PID_FILTER + dt)) * (derivative - pid->_last_derivative);

        // update state
      pid->_last_input             = input;
      pid->_last_derivative    = derivative;

        // add in derivative component
      return pid->_kd * derivative;
      
}
float get_pi(float error, float dt ,AC_PID* pid)
{
    return get_p(error , pid) + get_i(error, dt, pid);
}


float get_pid(float error, float dt, AC_PID* pid)
{
    return get_p(error, pid) + get_i(error, dt, pid) + get_d(error, dt, pid);
}

void reset_I(AC_PID* pid)
{
    pid->_integrator = 0;
	// mark derivative as invalid
    pid->_last_derivative = -1;
}



void set_d_lpf_alpha(short cutoff_frequency, float time_step,AC_PID*pid)
{    
    // calculate alpha
    float rc = 1/(2*PI*cutoff_frequency);
    pid-> _d_lpf_alpha = time_step / (time_step + rc);
}


