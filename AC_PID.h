// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#ifndef __AC_PID_H__
#define __AC_PID_H__


#include <stdlib.h>
#include <math.h>               // for fabs()

// Examples for _filter:
// f_cut = 10 Hz -> _alpha = 0.385869
// f_cut = 15 Hz -> _alpha = 0.485194
// f_cut = 20 Hz -> _alpha = 0.556864
// f_cut = 25 Hz -> _alpha = 0.611015
// f_cut = 30 Hz -> _alpha = 0.653373
#define AC_PID_D_TERM_FILTER 0.556864f    // Default 100Hz Filter Rate with 20Hz Cutoff Frequency
#define AC_PID_FILTER        7.9577e-3f // Set to  "1 / ( 2 * PI * f_cut )"
/// @class	AC_PID
/// @brief	Object managing one PID control
typedef struct _ac_pid{
    float        _kp;
    float        _ki;
    float        _kd;
    short        _imax;

    float           _integrator;                                ///< integrator value
    float           _last_input;                                ///< last input for derivative
    float           _last_derivative;                           ///< last derivative for low-pass filter
    float           _d_lpf_alpha;                               ///< alpha used in D-term LPF
}AC_PID;

    /// Iterate the PID, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error	The measured error value
    /// @param dt		The time delta in milliseconds (note
    ///					that update interval cannot be more
    ///					than 65.535 seconds due to limited range
    ///					of the data type).
    /// @param scaler	An arbitrary scale factor
    ///
    /// @returns		The updated control output.
    ///
float get_pid(float error, float dt, AC_PID* pid);
float get_pi(float error, float dt ,AC_PID* pid);
float get_p(float error, AC_PID*pid);
float get_i(float error, float dt ,AC_PID* pid);
float get_d(float input, float dt ,AC_PID* pid);
float get_d_filter(float input, float dt ,AC_PID* pid);
float       get_integrator(AC_PID*pid);
void        set_integrator(float i,AC_PID*pid) ;
    /// Reset the PID integrator
void        reset_I(AC_PID* pid);
    
    /// Sets filter Alpha for D-term LPF
void set_d_lpf_alpha(short cutoff_frequency, float time_step,AC_PID*pid);

void PI_init(AC_PID *apm_pi, float kp, float ki, float imax);
void PID_init(AC_PID * ap_pid, float initial_p, float initial_i, float initial_d, float initial_imax);
void PID_Filter_init(AC_PID * ap_pid, float initial_p, float initial_i, float initial_d, float initial_imax,float lpf);

#endif // __AC_PID_H__
