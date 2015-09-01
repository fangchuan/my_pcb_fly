#ifndef __PROTOCOL_H_
#define __PROTOCOL_H_

#define PROTOCOL_VERSION 2

#define PROTOCOL_START_0                 0x46
#define PROTOCOL_START_1                 0x41
#define PROTOCOL_END                     0x48


#define PROTOCOL_OPTICAL_RESET			0x30
#define PROTOCOL_SERVO_UP			0x30
#define PROTOCOL_SERVO_DOWN			0x31
#define PROTOCOL_SERVO_LEFT			0x32
#define PROTOCOL_SERVO_RIGHT			0x33


#define PROTOCOL_THROTTLE_SET			 0x31
#define PROTOCOL_THROTTLE_UP			 0x30
#define PROTOCOL_THROTTLE_DOWN		         0x31
#define PROTOCOL_THROTTLE_LITTLE_UP		 0x32
#define PROTOCOL_THROTTLE_LITTLE_DOWN	         0x33

#define PROTOCOL_ATTITUDE_SET			 0x32
#define PROTOCOL_ATTITUDE_FRONT		         0x30
#define PROTOCOL_ATTITUDE_BACK			 0x31
#define PROTOCOL_ATTITUDE_LEFT			 0x32
#define PROTOCOL_ATTITUDE_RIGHT			 0x33
#define PROTOCOL_HOVER_HOLD			 0x34
#define PROTOCOL_HEIGHT_HOLD                     0x35
#define PROTOCOL_YAW_LEFT                        0x36
#define PROTOCOL_YAW_RIGHT                       0x37

#define PROTOCOL_PID_CHANGE                      0x33
#define	PROTOCOL_PID_X_CHANGE                    0x30
        #define PROTOCOL_PID_X_P_CHANGE              0X31
        #define PROTOCOL_PID_X_I_CHANGE              0X32
        #define PROTOCOL_PID_X_D_CHANGE              0X33
#define	PROTOCOL_PID_Y_CHANGE 			 0x31
        #define PROTOCOL_PID_Y_P_CHANGE              0X31
        #define PROTOCOL_PID_Y_I_CHANGE              0X32
        #define PROTOCOL_PID_Y_D_CHANGE              0X33
#define	PROTOCOL_PID_Z_CHANGE			 0x32
        #define PROTOCOL_PID_Z_P_CHANGE              0X31
        #define PROTOCOL_PID_Z_I_CHANGE              0X32
	#define PROTOCOL_PID_Z_D_CHANGE              0X33
#define PROTOCOL_PID_H_CHANGE			 0x33
        #define PROTOCOL_PID_H_P_CHANGE              0X31
        #define PROTOCOL_PID_H_I_CHANGE              0X32
	#define PROTOCOL_PID_H_D_CHANGE              0X33
#define PROTOCOL_PID_P_X_CHANGE			 0x34
        #define PROTOCOL_PID_PX_P_CHANGE              0X31
        #define PROTOCOL_PID_PX_I_CHANGE              0X32
	#define PROTOCOL_PID_PX_D_CHANGE              0X33
#define PROTOCOL_PID_P_Y_CHANGE			 0x35
        #define PROTOCOL_PID_PY_P_CHANGE              0X31
        #define PROTOCOL_PID_PY_I_CHANGE              0X32
	#define PROTOCOL_PID_PY_D_CHANGE              0X33

#define PROTOCOL_MODE_SET		         0x34
#define PROTOCOL_MODE_HALT			 0x30
#define PROTOCOL_MODE_START	                 0x31
#define PROTOCOL_MODE_H_HOLD_FRONT		 0x32
#define PROTOCOL_MODE_H_HOLD_HOVER		 0x33

#define PROTOCOL_TARGET_ATTITUDE_LOCK            0X35

#define PROTOCOL_LINK_OK                         0x36

#define PROTOCOL_SET_H                           0X37

#define PROTOCOL_SET_X_DISTANCE                  0X38
        
#define PROTOCOL_SET_Y_DISTANCE                  0X39

#endif
