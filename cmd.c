#include "cmd.h"
#include "AP_OpticalFlow_ADNS3080.h"

extern uint8_t Commad_Array[20];

extern AC_PID RATE_ROLL_PID;
extern AC_PID RATE_PITCH_PID;
extern AC_PID RATE_YAW_PID;
extern AC_PID   STABILIZE_HIGHT_PID;
extern AC_PID   STABILIZE_POINT_X_PID;
extern AC_PID   STABILIZE_POINT_Y_PID;
extern uint8_t SystemInitOK;
extern int16_t PWM_THROTTLE;
extern uint16_t hight_target;
extern int16_t target_x_cm,target_y_cm;
extern uint16_t basis_throttle;
void control_attitude_set(uint8_t buffer)
{
        
     switch (buffer)
     {
     case PROTOCOL_ATTITUDE_FRONT:_angle_ef_target.x -= 500;
          break;
     case PROTOCOL_ATTITUDE_BACK:_angle_ef_target.x += 500;
	  break;
     case PROTOCOL_ATTITUDE_LEFT:_angle_ef_target.y += 500;
	  break;
     case PROTOCOL_ATTITUDE_RIGHT:_angle_ef_target.y -= 500;
	  break;
     case PROTOCOL_HOVER_HOLD: break;
     case PROTOCOL_HEIGHT_HOLD:break;
     case PROTOCOL_YAW_LEFT:      _angle_ef_target.z -=1000;
          break;
     case PROTOCOL_YAW_RIGHT:     _angle_ef_target.z +=1000;
          break;
      }  
}
void control_on_or_off(uint8_t buffer)
{
      switch(buffer)
      {
      case PROTOCOL_MODE_HALT:system_halt();break;
      case PROTOCOL_MODE_START:system_start();break;
      }
}
void control_up_or_down(uint8_t buffer)
{
     uint8_t length = Commad_Array[2] -0x30;
     uint16_t sum = 0;
     uint8_t i = 1;
     for (;i < length;i++)
          sum += (Commad_Array[i+4] - 0x30)*pow(10,(length-i));
     switch(buffer)
     {
     case PROTOCOL_THROTTLE_UP:  basis_throttle = sum;break;
     case PROTOCOL_THROTTLE_DOWN:PWM_THROTTLE -= 20;break;
     }
}
void control_pid_change(uint8_t buffer)
{
     uint8_t length = Commad_Array[2]-0x30 -3;
     uint8_t i=0;
     float sum=0;
     switch(buffer)
     {
     case PROTOCOL_PID_X_CHANGE:if(Commad_Array[5] == PROTOCOL_PID_X_P_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));RATE_ROLL_PID._kp = sum *0.001;}//
                                if(Commad_Array[5] == PROTOCOL_PID_X_I_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));RATE_ROLL_PID._ki = sum *0.001;}//   
                                if(Commad_Array[5] == PROTOCOL_PID_X_D_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));RATE_ROLL_PID._kd = sum *0.001;}//
     break;
     case PROTOCOL_PID_Y_CHANGE:if(Commad_Array[5] == PROTOCOL_PID_Y_P_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));RATE_PITCH_PID._kp = sum *0.001;}//
                                if(Commad_Array[5] == PROTOCOL_PID_Y_I_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));RATE_PITCH_PID._ki = sum *0.001;}//  
                                if(Commad_Array[5] == PROTOCOL_PID_Y_D_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));RATE_PITCH_PID._kd = sum *0.001;}//
     break;
     case PROTOCOL_PID_Z_CHANGE:if(Commad_Array[5] == PROTOCOL_PID_Z_P_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));RATE_YAW_PID._kp = sum *0.001;}//
                                if(Commad_Array[5] == PROTOCOL_PID_Z_I_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));RATE_YAW_PID._ki = sum *0.001;}//    
                                if(Commad_Array[5] == PROTOCOL_PID_Z_D_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));RATE_YAW_PID._kd = sum *0.001;}//
     break;
     case PROTOCOL_PID_H_CHANGE:if(Commad_Array[5] == PROTOCOL_PID_H_P_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));STABILIZE_HIGHT_PID._kp = sum *0.1;}
                                if(Commad_Array[5] == PROTOCOL_PID_H_I_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));STABILIZE_HIGHT_PID._ki = sum *0.01;}
                                if(Commad_Array[5] == PROTOCOL_PID_H_D_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));STABILIZE_HIGHT_PID._kd = sum *0.01;}
     break;
     case PROTOCOL_PID_P_X_CHANGE:if(Commad_Array[5] == PROTOCOL_PID_PX_P_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));STABILIZE_POINT_X_PID._kp = sum *0.01;}
                                  if(Commad_Array[5] == PROTOCOL_PID_PX_I_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));STABILIZE_POINT_X_PID._ki = sum *0.001;}
                                  if(Commad_Array[5] == PROTOCOL_PID_PX_D_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));STABILIZE_POINT_X_PID._kd = sum *0.01;}
     break;
     case PROTOCOL_PID_P_Y_CHANGE:if(Commad_Array[5] == PROTOCOL_PID_PY_P_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));STABILIZE_POINT_Y_PID._kp = sum *0.01;}
                                  if(Commad_Array[5] == PROTOCOL_PID_PY_I_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));STABILIZE_POINT_Y_PID._ki = sum *0.001;}
                                  if(Commad_Array[5] == PROTOCOL_PID_PY_D_CHANGE){for(i=1;i<=length;i++)sum +=(Commad_Array[i+5]-0x30)*pow(10,(length-i));STABILIZE_POINT_Y_PID._kd = sum *0.01;}
     break;
    }          
}
void control_h_set(void)
{
     uint8_t length = Commad_Array[2] -0x30;
     uint16_t sum = 0;
     uint8_t i = 0;
     for (;i < length;i++)
          sum += (Commad_Array[i+4] - 0x30)*pow(10,(length-i));
      
     hight_target = sum;
        
}
void control_x_distance_set(void)//cm
{
     uint8_t length = Commad_Array[2] -0x30;
     uint16_t sum = 0;
     uint8_t i = 0;
     for (;i < length;i++)
          sum += (Commad_Array[i+4] - 0x30)*pow(10,(length-i));

     target_x_cm = sum;
}

void control_y_distance_set(void)//cm
{
     uint8_t length = Commad_Array[2] -0x30;
     uint16_t sum = 0;
     uint8_t i = 0;
     for (;i < length;i++)
          sum += (Commad_Array[i+4] - 0x30)*pow(10,(length-i));

     target_y_cm = sum;
}

void  Commad_Analyze(void)
{    
	if(Commad_Array[1] == PROTOCOL_START_1)    
      {
	switch (Commad_Array[3])
        {
         case PROTOCOL_THROTTLE_SET:  control_up_or_down(Commad_Array[4]);   BEEP_TOGGLE;    break;
         case PROTOCOL_ATTITUDE_SET:  control_attitude_set(Commad_Array[4]); LED2_TOGGLE;    break;
         case PROTOCOL_PID_CHANGE:    control_pid_change(Commad_Array[4]);   LED2_TOGGLE;    break;
         case PROTOCOL_MODE_SET:      control_on_or_off(Commad_Array[4]);    BEEP_TOGGLE;    break;
         case PROTOCOL_TARGET_ATTITUDE_LOCK:                                   break;
         case PROTOCOL_OPTICAL_RESET:       /*clear_motion();*/                    break;
         case PROTOCOL_SET_H:               control_h_set();LED2_TOGGLE;       break;
	     case PROTOCOL_SET_X_DISTANCE:    control_x_distance_set();LED2_TOGGLE;break;
	     case PROTOCOL_SET_Y_DISTANCE:    control_y_distance_set();LED2_TOGGLE;break; 
        }
        Commad_Array[1] = 0x21;//
      }
         
}
