#include "RC_Channel.h"

extern RC_Channel rc;
extern Vector3f    _angle_ef_target;
extern uint16_t PWM_InCh1,PWM_InCh2,PWM_InCh3,PWM_InCh4;
// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void rc_loop()
{
    // Read radio and 3-position switch on radio
    read_radio(&rc);
	  
//	  _angle_ef_target.x = rc.roll;
//	  _angle_ef_target.y = rc.pitch ;
//	  _angle_ef_target.z = rc.yaw;
}
void rc_channel_init(RC_Channel *rc_channel)
{
	rc_channel->throttle = 0;
	rc_channel->roll = 0;
	rc_channel->pitch = 0;
        rc_channel->yaw = 0;
	
}

void read_radio(RC_Channel *rc_channel)
{
//	rc_channel->roll = PWM_InCh1 -AIL_MIDDLE;
//	rc_channel->pitch = PWM_InCh2 -ELF_MIDDLE;
	rc_channel->throttle = PWM_InCh3;
//	rc_channel->yaw = PWM_InCh4 -RUD_MIDDLE;
}