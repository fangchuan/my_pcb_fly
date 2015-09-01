#include "task.h"

#define TASK_MAX_NUM 30
task_t task_array[TASK_MAX_NUM];
int task_index;
int task_size;

void task_array_init(void)
{
	task_index = 0;
	task_size = 0;
}

int task_array_put(task_t *task)
{
	if(task_size >= TASK_MAX_NUM)
	{
		return -1;
	}
	task_array[task_size] = *task;
	task_size++;
	return 0;
}

int task_array_add_time_task(float pitch, float roll, float yaw, uint16_t height, int duration,uint8_t relay,uint8_t camera)
{
	task_t task;
	task.stop_type = TASK_TYPE_STOP_BY_TIME;
	task.target_angle.y = pitch;
	task.target_angle.x = roll;
	task.target_angle.z = yaw;
	task.target_height = height;
	task.duration = duration;
	task.relay_io = relay;
	task.camera_io = camera;
	return task_array_put(&task);
}

int task_array_add_line_width_task(float pitch, float roll, float yaw, uint16_t height, int wait_greater, int8_t line_width,uint8_t relay,uint8_t camera)
{
	task_t task;
	task.stop_type = TASK_TYPE_STOP_BY_LINE_WIDTH;
	task.target_angle.y = pitch;
	task.target_angle.x = roll;
	task.target_angle.z = yaw;
	task.target_height = height;
	task.wait_greater = wait_greater;
	task.line_width = line_width;
	task.relay_io = relay;
	task.camera_io = camera;
	return task_array_put(&task);
}

int task_array_add_motor_switch_task(uint8_t enable)
{
	task_t task;
	task.stop_type = TASK_TYPE_MOTOR_SWITCH;
	task.motor_enable = enable;
	return task_array_put(&task);
}

int task_array_get(task_t *task)
{
	if(task_index >= task_size)
	{
		return -1;
	}
	*task = task_array[task_index];
	task_index++;
	return 0;
}