#ifndef _TASK_
#define _TASK_

#include "Vector3f.h"

typedef enum 
{
	TASK_TYPE_STOP_BY_TIME,
	TASK_TYPE_STOP_BY_LINE_WIDTH,
	TASK_TYPE_MOTOR_SWITCH,
}task_stop_type_e;

typedef struct 
{
	task_stop_type_e stop_type;
	uint16_t target_height;
	Vector3f target_angle;
	//relative to time
	//unit: ms
	int duration;
	uint8_t relay_io;
	uint8_t camera_io;
	//relative to line_width
	int wait_greater;
	int8_t  line_width;
	//relative to motor switch
	uint8_t motor_enable;
}task_t;

#define WAIT_GREATER 1
#define WAIT_LESS    0

#define RELAY_ON 1
#define RELAY_OFF 0
#define CAMERA_EN_ON 0
#define CAMERA_EN_OFF 1

extern void task_array_init(void);
extern int task_array_put(task_t *task);
extern int task_array_get(task_t *task);
extern int task_array_add_time_task(float pitch, float roll, float yaw, uint16_t height, int duration,uint8_t relay,uint8_t camera);
extern int task_array_add_line_width_task(float pitch, float roll, float yaw, uint16_t height, int wait_greater, int8_t line_width,uint8_t relay,uint8_t camera);
extern int task_array_add_motor_switch_task(uint8_t enable);

#endif