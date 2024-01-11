#include "Serial_Task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "minipc.h"


Chassis_t Chassis;
int txbuf,tx_index;
static void Interact_remote(void); 
static void Interact_Visdata(void);

void Serial_Task(void const * argument)
{

  for(;;)
  {
		
		Interact_remote();
		Interact_Visdata();
		
    osDelay(3);
  }
}

//static void Interact_remote()
//{
//	  Chassis.direction = 1;
//	  Chassis.txMsg->data[0] = (uint8_t)(remote_ctrl.rc.s[0]) << 6 |  (uint8_t)(remote_ctrl.rc.s[1]) << 4|(uint8_t)Chassis.direction;	
//    Chassis.txMsg->data[2] = (uint8_t)(remote_ctrl.rc.ch[2]>>8);
//    Chassis.txMsg->data[3] = (uint8_t)(remote_ctrl.rc.ch[2]);	
//    Chassis.txMsg->data[4] = (uint8_t)(remote_ctrl.rc.ch[3]>>8);
//    Chassis.txMsg->data[5] = (uint8_t)(remote_ctrl.rc.ch[3]);

//	    USER_CAN_TxMessage(&INTER_ACT);
//}

static void Interact_remote()
{
	  Chassis.direction = 1;
	  INTER_ACT.data[0] = (uint8_t)(remote_ctrl.rc.s[0]) << 6 |  (uint8_t)(remote_ctrl.rc.s[1]) << 4|(uint8_t)Chassis.direction;	
    INTER_ACT.data[2] = (uint8_t)(remote_ctrl.rc.ch[2]>>8);
    INTER_ACT.data[3] = (uint8_t)(remote_ctrl.rc.ch[2]);	
    INTER_ACT.data[4] = (uint8_t)(remote_ctrl.rc.ch[3]>>8);
    INTER_ACT.data[5] = (uint8_t)(remote_ctrl.rc.ch[3]);

	    USER_CAN_TxMessage(&INTER_ACT);
}

static void Interact_Visdata()
{
//	VIS_ACT.data[0] = (int8_t)((MiniPC_ReceivePacket.vx)>>24);
//	VIS_ACT.data[1] = (uint8_t())

	Chassis.vis_vx = (int16_t)(MiniPC_ReceivePacket.linear_x*1000);
	Chassis.vis_vy = (int16_t)(MiniPC_ReceivePacket.linear_y*1000);
	Chassis.vis_vz = (int16_t)(MiniPC_ReceivePacket.angular_z*1000);
	VIS_ACT.data[0] = (uint8_t)(Chassis.vis_vx>>8);
	VIS_ACT.data[1] = (uint8_t)(Chassis.vis_vx);
	VIS_ACT.data[2] = (uint8_t)(Chassis.vis_vy>>8);
	VIS_ACT.data[3] = (uint8_t)(Chassis.vis_vy);
	VIS_ACT.data[4] = (uint8_t)(Chassis.vis_vz>>8);
	VIS_ACT.data[5] = (uint8_t)(Chassis.vis_vz);
	
	USER_CAN_TxMessage(&VIS_ACT);
	

}
	

