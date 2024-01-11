#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

typedef struct
{
	struct
	{
		float frict1_v;

		float l_trigger_v;
	}Target;
	int sendvalue[3];
	
}Shoot_info_t;

extern Shoot_info_t Shoot_info;

#endif

