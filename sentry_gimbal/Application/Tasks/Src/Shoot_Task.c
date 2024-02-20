#include "Shoot_Task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "motor.h"
#include "pid.h"
#include "CANTx_Task.h"
#include "remote_control.h"
#include "Vision_Task.h"

PID_Info_TypeDef SHOOT_PID;

float SHOOT_pid[3][6] = 
{
	[0] = {10,0.001,0,0,5000,16300},
	[1] = {10,0.001,0,0,5000,16300},
	[2] = {10,0.001,0,0,5000,16300},
};

//float SHOOT_pid[5][6] = 
//{
//	[0] = {10,0.001,0,0,5000,16300},
//	[1] = {10,0.001,0,0,5000,16300},
//	[2] = {10,0.001,0,0,5000,16300},
//	[3] = {10,0.001,0,0,5000,16300},
//	[4] = {10,0.001,0,0,5000,16300},
//};
float k_trigger_v = 8.1;
int v_frict1 = 5700;
//int v_frict1 = 0;
int v_trigger = 5346;
int stuck_flag = 1;
uint16_t cnt = 0;
int fztime = 300; 
int flag = 0;

int f = 1;
int ch4_trigger_v = 0;

static void SHOOT_Init(void);
static void SHOOT_CAL(Shoot_info_t *Shoot_info);
bool JUDGE_IF_BLOCK(void);
static void Trigger_handle(void);
//static void JUDGE_IF_FIRE(void);



void Shoot_Task(void const * argument)
{
	SHOOT_Init();
	osDelay(10);
	
  for(;;)
  {
		Trigger_handle();
		SHOOT_CAL(&Shoot_info);

    osDelay(1);
  }
}

static void SHOOT_Init(void)
{
	PID_Init(&SHOOT_PID,PID_POSITION,SHOOT_pid[0]);
	PID_Init(&SHOOT_PID,PID_POSITION,SHOOT_pid[1]);
	PID_Init(&SHOOT_PID,PID_POSITION,SHOOT_pid[2]);
}

//static void SHOOT_Init(void)
//{
//	PID_Init(&SHOOT_PID,PID_POSITION,SHOOT_pid[0]);
//	PID_Init(&SHOOT_PID,PID_POSITION,SHOOT_pid[1]);
//	PID_Init(&SHOOT_PID,PID_POSITION,SHOOT_pid[2]);
//	PID_Init(&SHOOT_PID,PID_POSITION,SHOOT_pid[3]);
//	PID_Init(&SHOOT_PID,PID_POSITION,SHOOT_pid[4]);
//}


//static void SHOOT_CAL(Shoot_info_t *Shoot_info)
//{

//	if((remote_ctrl.rc.s[1] == 3) || (remote_ctrl.rc.s[1] == 1 && remote_ctrl.rc.s[0] == 2))
//	{
//		if(remote_ctrl.rc.ch[4] != 0)
//		{
//			ch4_trigger_v = -660;
//		}
//		if(remote_ctrl.rc.ch[4] == 0)
//		{
//		  ch4_trigger_v = 0;
//		}
//		Shoot_info->Target.frict1_v = v_frict1;
//		Shoot_info->Target.l_trigger_v = ch4_trigger_v * k_trigger_v * stuck_flag;
//	}
//	if(remote_ctrl.rc.s[1] == 1 && remote_ctrl.rc.s[0] == 2)
//  {
//		Shoot_info->Target.frict1_v = 0;
//		Shoot_info->Target.l_trigger_v = 0;

//	
//	}
//	if(remote_ctrl.rc.s[1] == 1 && remote_ctrl.rc.s[0] == 3)
//	{
//		Shoot_info->Target.frict1_v = v_frict1;
//		Shoot_info->Target.l_trigger_v = Vision_Info.IF_Fire_Accept * v_trigger *stuck_flag;

//	}

//	Shoot_info->sendvalue[0] = f_PID_Calculate(&SHOOT_PID,-Shoot_info->Target.frict1_v,Shoot[l_Frict1].Data.velocity);
//	Shoot_info->sendvalue[1] = f_PID_Calculate(&SHOOT_PID,Shoot_info->Target.frict1_v,Shoot[r_Frict1].Data.velocity);
//	
//	Shoot_info->sendvalue[2] = f_PID_Calculate(&SHOOT_PID,-Shoot_info->Target.frict1_v,Shoot[l_Frict2].Data.velocity);
//	Shoot_info->sendvalue[3] = f_PID_Calculate(&SHOOT_PID,Shoot_info->Target.frict1_v,Shoot[r_Frict2].Data.velocity);

//	Shoot_info->sendvalue[4] = f_PID_Calculate(&SHOOT_PID,Shoot_info->Target.l_trigger_v,Shoot[l_Trigger].Data.velocity);
//		

//}

static void SHOOT_CAL(Shoot_info_t *Shoot_info)
{
	
	if((remote_ctrl.rc.s[1] == 3) || (remote_ctrl.rc.s[1] == 1 && remote_ctrl.rc.s[0] == 2))
	{
		if(remote_ctrl.rc.ch[4] != 0)
		{
			ch4_trigger_v = -660;
		}
		if(remote_ctrl.rc.ch[4] == 0)
		{
		  ch4_trigger_v = 0;
		}
		Shoot_info->Target.frict1_v = v_frict1;
		Shoot_info->Target.l_trigger_v = ch4_trigger_v * k_trigger_v * stuck_flag;
	}
	
	if(remote_ctrl.rc.s[1] == 1 && remote_ctrl.rc.s[0] == 2)
  {
		Shoot_info->Target.frict1_v = 0;
		Shoot_info->Target.l_trigger_v = 0;

	}
	if(remote_ctrl.rc.s[1] == 1 && remote_ctrl.rc.s[0] == 3)
	{
		Shoot_info->Target.frict1_v = v_frict1;
		Shoot_info->Target.l_trigger_v = Vision_Info.IF_Fire_Accept * v_trigger *stuck_flag;

	}

	Shoot_info->sendvalue[0] = f_PID_Calculate(&SHOOT_PID,Shoot_info->Target.frict1_v,Shoot[l_Frict1].Data.velocity);
	Shoot_info->sendvalue[1] = f_PID_Calculate(&SHOOT_PID,-Shoot_info->Target.frict1_v,Shoot[r_Frict1].Data.velocity);

	Shoot_info->sendvalue[2] = f_PID_Calculate(&SHOOT_PID,Shoot_info->Target.l_trigger_v,Shoot[l_Trigger].Data.velocity);
		

}

bool Judge_IF_block(void)
{
	bool res = false;
	if((ABS(Shoot_info.Target.l_trigger_v - Shoot[l_Trigger].Data.velocity)>300) && (Shoot[l_Trigger].Data.velocity<300))
	{
		res = true;
		flag = 1;
	}
	else flag = 0;
	return res;
}

static void Trigger_handle(void)
{
	static uint16_t time;
	if(Judge_IF_block() == true)
	{
		cnt++;
		if(cnt>300)
		{
			stuck_flag = -1;
		  cnt = 0;
		}
	}
	else 
  {
		cnt = 0;
	}
	
	if(stuck_flag == -1)	time++;
	if(stuck_flag == -1 && time>fztime)
	{
		stuck_flag = 1;
		time = 0;		
	}			
}
