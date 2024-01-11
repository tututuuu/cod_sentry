#include "CANTx_Task.h"
#include "cmsis_os.h"
#include "Gimbal_Task.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "INS_Task.h"
#include "Shoot_Task.h"

Gimbal_Info_t Gimbal_Info;
Shoot_info_t Shoot_info;
void CANTx_Task(void const * argument)
{
   
  for(;;)
  {
		if(((remote_ctrl.rc.s[0]==2) && (remote_ctrl.rc.s[1]==2)) || (remote_ctrl.rc.s[0]==0) || (remote_ctrl.rc.s[1]==0))
		{
			Gimbal_Info.Target.yaw_angle = INS_Info.yaw_tolangle;
			Gimbal_Info.Target.pitch_angle = INS_Info.rol_angle;
			Shoot_info.Target.frict1_v = 0;
//			Shoot_info.sendvalue[0] = 0;
//			Shoot_info.sendvalue[1] = 0;
			Shoot_info.sendvalue[2] = 0;
			Gimbal_Info.sendvalue[0] = 0;
			Gimbal_Info.sendvalue[1] = 0;


		}
		gimbal_TxFrame.data[0] = (uint8_t)(Gimbal_Info.sendvalue[0] >> 8);
		gimbal_TxFrame.data[1] = (uint8_t)(Gimbal_Info.sendvalue[0]);
		gimbal_TxFrame.data[2] = (uint8_t)(Gimbal_Info.sendvalue[1] >> 8);
		gimbal_TxFrame.data[3] = (uint8_t)(Gimbal_Info.sendvalue[1]);
		USER_CAN_TxMessage(&gimbal_TxFrame);

		shoot_TxFrame1.data[0] = (uint8_t)(Shoot_info.sendvalue[0] >> 8);
		shoot_TxFrame1.data[1] = (uint8_t)(Shoot_info.sendvalue[0]);
		shoot_TxFrame1.data[2] = (uint8_t)(Shoot_info.sendvalue[1] >> 8);
		shoot_TxFrame1.data[3] = (uint8_t)(Shoot_info.sendvalue[1]);
		shoot_TxFrame1.data[4] = (uint8_t)(Shoot_info.sendvalue[2] >> 8);
		shoot_TxFrame1.data[5] = (uint8_t)(Shoot_info.sendvalue[2]);
		USER_CAN_TxMessage(&shoot_TxFrame1);
		
		osDelay(1);
	}

}
