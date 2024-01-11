#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H


typedef struct
{
	struct
	{
		float yaw_angle;
		float yaw_v;
		float pitch_angle;
		float pitch_v;
	}Target;
	
	
	int sendvalue[2];

}Gimbal_Info_t;

extern Gimbal_Info_t Gimbal_Info;

#endif
