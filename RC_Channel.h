#ifndef __RC_CHANNEL__
#define __RC_CHANNEL__

#include "r_cg_macrodriver.h"
#include "AP_Math.h"
#include "ArduCopter.h"

typedef struct _RC_Channel
{
	uint16_t throttle;
        uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
  
} RC_Channel;


#define AIL_MIDDLE     1520
#define ELF_MIDDLE     1520
#define RUD_MIDDLE     1520

void rc_channel_init(RC_Channel *rc_channel);
void rc_loop(void);
void read_radio(RC_Channel *rc_channel);

#endif
