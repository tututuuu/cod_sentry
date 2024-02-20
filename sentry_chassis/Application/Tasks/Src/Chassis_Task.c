#include "Chassis_Task.h"
#include "motor.h"
#include "bsp_can.h"
#include "pid.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "remote_control.h"
#include "INS_Task.h"
#include "minipc.h"

//18:22
float res_angle,middle_angle=0,sinres,cosres;

PID_Info_TypeDef Wheel_PID[4],yaw_speed_PID,yaw_angle_PID,location_x_PID,location_y_PID;
Chassis_Info_t wheel;
ACTline_t Present_line,Goal_line;
int p = 18;
int pos_cnt = 0;

float m = 0.2;
float res_v =8; 

float K_T = 18.f/22.f;
float k_vis = 2.8;
//int k_vis = 2000;
float sin_angle;
float cos_angle;



float Chassis_PID[4][6]=
{
	[LF] = {18,0.001,0,0,3000,15000},
	[LR] = {18,0.001,0,0,3000,15000},
	[RR] = {18,0.001,0,0,3000,15000},
	[RF] = {18,0.001,0,0,3000,15000},
};

float location_X_parm[6]={3.6f ,0.001f ,0.f  ,0.f  ,20.f  ,1111};
float location_Y_parm[6]={3.6f ,0.001f ,0.f  ,0.f  ,20.f  ,1111};

float YAW_pid[2][6]=
{
	[0]={-5,0,0,0,3000,1000},
	[1]={-4,0,0,0,3000,700},
};
Chassis_Info_t Remote =
{
	.stdid = 0x300,
};
Chassis_Info_t visdata =
{
	.stdid = 0x400,
};

ACTline_t Line_t[pos_num] =
{
	[pos1] = 
	{
		.point.x = 0,
		.point.y = -700,
	},
	[pos2] = 
	{
		.point.x = 0,
		.point.y = 700,
	},

};

int h = 20;
int ins_v = 600;
float x_com,y_com,w_com;
float g_lf_com,g_lr_com,g_rf_com,g_rr_com;
float sin_pit_angle,sin_roll_angle;
float com_angle;
float gain_k1 = 0.1,gain_k2 = 1.5,gain_k;
int patrol = 0,patrol_v;
float location_x,location_y,location_w,last_x,last_y;
float updata_x,updata_y,up_x,up_y,YAW_angle;
float GOAL_X,GOAL_Y;
int z = 0;



static void Chassis_Init(void);
static void Get_location(void);
static void Chassis_Gimbal_MODE(void);
static void Chassis_cal(Chassis_Info_t *wheel);  
static void G_com(void);
static void Chassis_sent(Chassis_Info_t *wheel);
static void Position(void);



void Chassis_Task(void const * argument)
{
	Chassis_Init();
	osDelay(10);
	
	for(;;)
  {
		PID_Init(&location_x_PID,PID_POSITION,location_X_parm);
		PID_Init(&location_y_PID,PID_POSITION,location_Y_parm);
		Get_location();
		Position();
		Chassis_Gimbal_MODE();
		Chassis_cal(&wheel);
		G_com();
		Chassis_sent(&wheel);
		
		osDelay(2);
	}

}
static void Chassis_Init()
{
	PID_Init(&Wheel_PID[LF],PID_POSITION,Chassis_PID[0]);
	PID_Init(&Wheel_PID[LR],PID_POSITION,Chassis_PID[1]);
	PID_Init(&Wheel_PID[RF],PID_POSITION,Chassis_PID[2]);
	PID_Init(&Wheel_PID[RR],PID_POSITION,Chassis_PID[3]);
	PID_Init(&yaw_angle_PID,PID_POSITION,YAW_pid[0]);
	PID_Init(&yaw_speed_PID,PID_POSITION,YAW_pid[1]);
}


static void Get_location()
{

	location_x = (chassis[RF].Data.angle+chassis[LF].Data.angle-chassis[LR].Data.angle-chassis[RR].Data.angle)/360.f*3.141592*152/(1.414/2.f)/4.f;
	location_y = (chassis[LF].Data.angle+chassis[LR].Data.angle-chassis[RF].Data.angle-chassis[RR].Data.angle)/360.f*3.141592*152/(1.414/2.f)/4.f;
	location_w = (chassis[LF].Data.angle+chassis[LR].Data.angle+chassis[RF].Data.angle+chassis[RR].Data.angle)/4.f*(152.f/520.f)+0;
	

	if(ABS(location_x-last_x)> 0.000001f)
	{
		updata_x = location_x-last_x;
		last_x = location_x;
	}
	else updata_x = 0;
	if(ABS(location_y-last_y) > 0.000001f)
	{
		updata_y = location_y - last_y;
		last_y = location_y;
	}
	else updata_y = 0;
//	up_x = arm_cos_f32(location_w*CHANGE_TO_RADIAN)*updata_x + arm_sin_f32(location_w*CHANGE_TO_RADIAN)*updata_y;
//	up_y = arm_cos_f32(location_w*CHANGE_TO_RADIAN)*updata_y - arm_sin_f32(location_w*CHANGE_TO_RADIAN)*updata_x;
	up_x = arm_cos_f32(location_w*CHANGE_TO_RADIAN)*updata_x - arm_sin_f32(location_w*CHANGE_TO_RADIAN)*updata_y;
	up_y = -arm_cos_f32(location_w*CHANGE_TO_RADIAN)*updata_y - arm_sin_f32(location_w*CHANGE_TO_RADIAN)*updata_x;
	
	
	Present_line.point.x += up_x;
	Present_line.point.y += up_y;
	YAW_angle = location_w + z;
	Present_line.yaw_angle=(	ABS(location_w)>360? (location_w/ABS(location_w))*(ABS(location_w)-(int)(location_w/360)*360):location_w);
 // Present_line.yaw_angle = (ABS(YAW_angle>360) ? YAW_angle-ABS(YAW_angle)/YAW_angle*360 : YAW_angle);

}

static void Position()
{
	Goal_line.point.x = Line_t[pos_cnt].point.x;
  Goal_line.point.y = Line_t[pos_cnt].point.y;
	if(sqrtf(powf(Goal_line.point.x-Present_line.point.x,2.f)+powf(Goal_line.point.y-Present_line.point.y,2.f))<80.f)
	{
		pos_cnt++;
		if(pos_cnt>1)
		{
			pos_cnt = 0;
		}
	}
}




static void Chassis_Gimbal_MODE()
{
	Goal_line.yaw_angle=CcltLineAngle(&Present_line, &Goal_line);
	if(Remote.s1 == 3)
	{
		res_angle = -yaw.Data.angle*K_T - middle_angle; 
		res_angle = ABS(res_angle)>360 ? ((int16_t)(res_angle)%360) : res_angle;
	}
	else if((Remote.s1 == 2) && (Remote.s0 == 3))
	{
		res_angle = YAW_angle - Goal_line.yaw_angle;
	}
	else res_angle = 0;
	res_angle = ABS(res_angle) > 180 ? res_angle-res_angle/ABS(res_angle)*360 : res_angle;
	if(Remote.s1 == 3)
	{

//			wheel.vx = arm_cos_f32(res_angle*CHANGE_TO_RADIAN)*(Remote.ch2*h) + arm_sin_f32(res_angle*CHANGE_TO_RADIAN)*(-Remote.ch3*h);
//			wheel.vy = arm_sin_f32(res_angle*CHANGE_TO_RADIAN)*(Remote.ch2*h) + arm_cos_f32(res_angle*CHANGE_TO_RADIAN)*(Remote.ch3*h);
			wheel.vx = arm_cos_f32(res_angle*CHANGE_TO_RADIAN)*(Remote.ch3*h) + arm_sin_f32(res_angle*CHANGE_TO_RADIAN)*(-Remote.ch2*h);
			wheel.vy = arm_sin_f32(res_angle*CHANGE_TO_RADIAN)*(-Remote.ch3*h) + arm_cos_f32(res_angle*CHANGE_TO_RADIAN)*(-Remote.ch2*h);
		
		  wheel.vw = 0;
			if(Remote.s0 == 3)  //folo
			{
				wheel.vw = f_PID_Calculate(&yaw_speed_PID,f_PID_Calculate(&yaw_angle_PID,res_angle,0),INS_Info.yaw_gyro);
			}
			if(Remote.s0 == 1)
			{
				wheel.vw = f_PID_Calculate(&yaw_speed_PID,ins_v,INS_Info.yaw_gyro);
			}
	}
  if(Remote.s1 == 1 && Remote.s0 == 2)
	{
		wheel.vx = 0;
		wheel.vy = 0;
    wheel.vw = 0;
	
	}
	if(Remote.s1 == 1 && Remote.s0 == 1)
	{
		res_angle = 0;
		wheel.vw = 0;
//		wheel.vx = Remote.ch2 * h;
//		wheel.vy = Remote.ch3 * h;
		wheel.vx = MiniPC_ReceivePacket.vx*k_vis;
		wheel.vy = MiniPC_ReceivePacket.vy*k_vis;
		
//    wheel.vx = -visdata.v_y * k_vis;
//    wheel.vy = visdata.v_x * k_vis;		
	}
	
	if(Remote.s1 == 2 && Remote.s0 == 3)
	{
		sin_angle = arm_sin_f32(res_angle*CHANGE_TO_RADIAN);
		cos_angle = arm_cos_f32(res_angle*CHANGE_TO_RADIAN);
		GOAL_X = f_PID_Calculate(&location_x_PID,Goal_line.point.x-Present_line.point.x,0);
		GOAL_Y = f_PID_Calculate(&location_y_PID,Goal_line.point.y-Present_line.point.y,0);
		wheel.vx = res_v*cos_angle*(sqrt(powf(GOAL_X,2)+powf(GOAL_Y,2)));
		wheel.vy = res_v*sin_angle*(sqrt(powf(GOAL_X,2)+powf(GOAL_Y,2)));	

	}
	VAL_LIMIT(wheel.vx,-15000,15000);
	VAL_LIMIT(wheel.vy,-15000,15000);

}

static void Chassis_cal(Chassis_Info_t *wheel)
{
	
//		wheel->Target.LF_v = 1/M_R*(wheel->vx+wheel->vy+M_L*wheel->vw);
//		wheel->Target.LR_v = 1/M_R*((-wheel->vx)+wheel->vy+M_L*wheel->vw);
//		wheel->Target.RR_v = 1/M_R*((-wheel->vx)-wheel->vy+M_L*wheel->vw);
//		wheel->Target.RF_v = 1/M_R*(wheel->vx-wheel->vy+M_L*wheel->vw);
	
  x_com = 0.25 *(chassis[LF].Data.velocity + chassis[RF].Data.velocity - chassis[LR].Data.velocity - chassis[RR].Data.velocity)*m;
	y_com = 0.25 *(chassis[LF].Data.velocity + chassis[LR].Data.velocity - chassis[RF].Data.velocity - chassis[RR].Data.velocity)*m;
	w_com = 0.0625 *(chassis[LF].Data.velocity + chassis[LR].Data.velocity + chassis[RF].Data.velocity + chassis[RR].Data.velocity)*m;
	wheel->Target.LF_v = 1/M_R*((wheel->vx-x_com) + (wheel->vy-y_com) + (M_L*(wheel->vw-w_com)));
	wheel->Target.LR_v = 1/M_R*((-wheel->vx-x_com) + (wheel->vy-y_com) + (M_L*(wheel->vw-w_com)));
	wheel->Target.RR_v = 1/M_R*((-wheel->vx-x_com) - (wheel->vy-y_com) + (M_L*(wheel->vw-w_com)));
	wheel->Target.RF_v = 1/M_R*((wheel->vx-x_com) - (wheel->vy-y_com) + (M_L*(wheel->vw-w_com)));

}

static void G_com(void)
{
	com_angle=ABS(INS_Info.pit_angle)>ABS(INS_Info.rol_angle) ? INS_Info.pit_angle:INS_Info.rol_angle;
	if(ABS(com_angle)<9.f) gain_k = gain_k1;
	else gain_k = gain_k2;
//	sin_pit_angle = ABS(arm_sin_f32(INS_Info.pit_angle*CHANGE_TO_RADIAN));
//	sin_roll_angle = ABS(arm_sin_f32(INS_Info.rol_angle*CHANGE_TO_RADIAN));
//	g_lf_com = -gain_k*((wheel.vx-x_com) + (wheel.vy-y_com))*com_angle;
//	g_lr_com = -gain_k*((-wheel.vx-x_com) + (wheel.vy-y_com))*com_angle;
//	g_rr_com = gain_k*((-wheel.vx-x_com) - (wheel.vy-y_com))*com_angle;
//	g_rf_com = -gain_k*((wheel.vx-x_com) - (wheel.vy-y_com))*com_angle;
	g_lf_com = -gain_k*((-x_com) + (-y_com))*com_angle;
	g_lr_com = -gain_k*((-x_com) + (-y_com))*com_angle;
	g_rr_com = gain_k*((-x_com) - (-y_com))*com_angle;
	g_rf_com = -gain_k*((-x_com) - (-y_com))*com_angle;
	
	VAL_LIMIT(g_lf_com,-666,666);
	VAL_LIMIT(g_lr_com,-666,666);
	VAL_LIMIT(g_rf_com,-666,666);
	VAL_LIMIT(g_rr_com,-666,666);
	
}

static void Chassis_sent(Chassis_Info_t *wheel)
{
		wheel->sentvalue[0] = f_PID_Calculate(&Wheel_PID[LF],wheel->Target.LF_v,chassis[0].Data.velocity) + g_lf_com;
		wheel->sentvalue[1] = f_PID_Calculate(&Wheel_PID[LR],wheel->Target.LR_v,chassis[1].Data.velocity) + g_lr_com;
		wheel->sentvalue[2] = f_PID_Calculate(&Wheel_PID[RF],wheel->Target.RF_v,chassis[2].Data.velocity) + g_rf_com;
		wheel->sentvalue[3] = f_PID_Calculate(&Wheel_PID[RR],wheel->Target.RR_v,chassis[3].Data.velocity) + g_rr_com;

}

//float Calangle(float angle1,float angle2)
//{
//	float result = angle1 - angle2;
//	result = (ABS(result > 180) ? result-ABS(result)/result*360 : result);
//	return result;

//}

float CcltLineAngle(ACTline_t *pointStart, ACTline_t *pointEnd)
{
	int16_t a = 0;
	int16_t b = 0;
	
	a = pointEnd->point.y - pointStart->point.y;
	b = pointEnd->point.x - pointStart->point.x;
	//atan2f范围可以包含-180到180  
	return (atan2f(a, b) * CHANGE_TO_ANGLE); //CHANGE_TO_ANGLE:360/(2*pai)，弧度转角度
}
void get_Gimbal_DATE1(uint32_t *canId,uint8_t *rxBuf,Chassis_Info_t*remote)
{
    if(*canId != remote->stdid)  return;
				remote->s0 = (int16_t)rxBuf[0]>>6;
				remote->s1 = ((int16_t)rxBuf[0] & ((uint8_t)(3)<<4)) >> 4 ;
				remote->ch2 = (int16_t) rxBuf[2] << 8 | (int16_t) rxBuf[3];
		    remote->ch3 = (int16_t) rxBuf[4] << 8 | (int16_t) rxBuf[5];
		
}

void get_Gimbal_DATE2(uint32_t *canId,uint8_t *rxBuf,Chassis_Info_t*visdata)
{
	if(*canId != visdata->stdid) return;
	visdata->v_x = (int16_t)rxBuf[0] << 8 | (int16_t)rxBuf[1];
	visdata->v_y = (int16_t)rxBuf[2] << 8 | (int16_t)rxBuf[3];
	visdata->v_w = (int16_t)rxBuf[4] << 8 | (int16_t)rxBuf[5];
}


