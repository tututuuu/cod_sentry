#ifndef SERIAL_TASK_H
#define SERIAL_TASK_H
#include "bsp_can.h"
#include "remote_control.h"

typedef struct
{
	CAN_TxFrameTypeDef *txMsg;
	Remote_Info_Typedef *remote;
	uint8_t direction;
	int16_t vis_vx,vis_vy,vis_vz;

}Chassis_t;

extern Chassis_t Chassis; 
#endif
