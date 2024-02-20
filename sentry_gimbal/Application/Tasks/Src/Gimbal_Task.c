#include "Gimbal_Task.h"
#include "motor.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "INS_Task.h"
#include "pid.h"
#include "Vision_Task.h"

PID_Info_TypeDef GIMBAL_PID[2][2];

float Gimbal_pid[2][2][6] = 
{
	[0] = 
	{
		[0] = {110,0,0,0,0,7000},       //yaw_angle  90    80     60
		[1] = {75,0.01,0,0,10000,30000},    //yaw_v  70   65    110
	},
	[1] = 
	{
		[0] = {100,0,0,0,0,5000},         //pitch_angle   100   130   100
		[1] = {80,0,0,0,12000,24000}      //pitch_v     -80   -100    80
	},
};
int vision_flag = 0;
float angle_err[2];

static void Gimbal_Init(void);
static void Gimbal_cal(void);
static void Gimbal_Mode(void);

void Gimbal_Task(void const * argument)
{
	Gimbal_Init();
  osDelay(2);
	
  for(;;)
  {
		Gimbal_Mode();
		Gimbal_cal();
    osDelay(1);
  }

}

static void Gimbal_Init(void)
{
	PID_Init(&GIMBAL_PID[0][0],PID_POSITION,Gimbal_pid[0][0]);
	PID_Init(&GIMBAL_PID[0][1],PID_POSITION,Gimbal_pid[0][1]);
	PID_Init(&GIMBAL_PID[1][0],PID_POSITION,Gimbal_pid[1][0]);
	PID_Init(&GIMBAL_PID[1][1],PID_POSITION,Gimbal_pid[1][1]);
}

float target_yaw_angle;
float target_pitch_angle;
static void Gimbal_Mode()
{
//  if(remote_ctrl.rc.s[1] == 3 || remote_ctrl.rc.s[1] == 1)
	if(remote_ctrl.rc.s[1] == 3 || ((remote_ctrl.rc.s[1]==2) && (remote_ctrl.rc.s[0]==3)))
	{

		Gimbal_Info.Target.yaw_angle -= 0.0002*remote_ctrl.rc.ch[0];
		Gimbal_Info.Target.pitch_angle -= 0.0002*remote_ctrl.rc.ch[1]; 
	 // VAL_LIMIT(Gimbal_Info.Target.pitch_angle,0,17);
		angle_err[0] = Gimbal_Info.Target.yaw_angle - INS_Info.yaw_tolangle;
		angle_err[1] = Gimbal_Info.Target.pitch_angle - INS_Info.rol_angle;
	}
	
	if((remote_ctrl.rc.s[1] == 1 && remote_ctrl.rc.s[0] == 3) || (remote_ctrl.rc.s[1] == 1 && remote_ctrl.rc.s[0] == 2))
	{
		vision_flag = 1;
	}
	else
	{
		vision_flag = 0;	
	}
	
		if((vision_flag == 1) && (Vision_Info.IF_Aiming_Enable))
		{

			VAL_LIMIT(Vision_Info.target_Pitch,-15,6);
      if(ABS(Vision_Info.target_Pitch)>40)
			{
				Vision_Info.target_Pitch = 0;
			}
			
			
//			target_yaw_angle += 0.0001 * Vision_Info.target_Yaw;
//			target_pitch_angle -= 0.0001 * Vision_Info.target_Pitch;
//           target_yaw_angle = Vision_Info.target_Yaw + 5;
			angle_err[0] = Vision_Info.target_Yaw - INS_Info.yaw_tolangle;
			angle_err[1] = -Vision_Info.target_Pitch - INS_Info.rol_angle;
			

//			if(Vision_Info.target_Yaw < 0.07)
//			{
//         angle_err[0] = 0;
//			}
//			if(Vision_Info.target_Pitch < 0.07)
//			{
//					angle_err[1] = 0;
//			}
			
//			angle_err[0] = target_yaw_angle - INS_Info.yaw_tolangle;
//			angle_err[1] = target_pitch_angle - INS_Info.rol_angle;
//		

				if(angle_err[0] >180) angle_err[0] =  angle_err[0]-360;
				else if(angle_err[0] < -180) angle_err[0] = 360 + angle_err[0];
		
				if(ABS(angle_err[0])>90.f)angle_err[0]=0;
//			if(ABS(angle_err[0])>180) angle_err[0] = angle_err[0]-angle_err[0]/ABS(angle_err[0])*360;
//		  if(ABS(angle_err[0])>90.f)angle_err[0]=0;
		
			Gimbal_Info.Target.yaw_angle = INS_Info.yaw_tolangle;
			Gimbal_Info.Target.pitch_angle = INS_Info.rol_angle;
		}
//		else
//		{
//			angle_err[0] = 0;
//			angle_err[1] = 0;
//		}
//		
	
	
	if(remote_ctrl.rc.s[1] == 1 && remote_ctrl.rc.s[0] == 1)
	{
	
	}
		
}

static void Gimbal_cal(void)
{
	Gimbal_Info.Target.yaw_v = f_PID_Calculate(&GIMBAL_PID[0][0],angle_err[0],0);
	Gimbal_Info.sendvalue[0] = f_PID_Calculate(&GIMBAL_PID[0][1],Gimbal_Info.Target.yaw_v-INS_Info.yaw_gyro,0);
	Gimbal_Info.Target.pitch_v = f_PID_Calculate(&GIMBAL_PID[1][0],angle_err[1],0);
	Gimbal_Info.sendvalue[1] = f_PID_Calculate(&GIMBAL_PID[1][1],Gimbal_Info.Target.pitch_v-INS_Info.rol_gyro,0);
	
}






