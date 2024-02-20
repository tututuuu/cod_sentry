#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "main.h"
#include "remote_control.h"

//角度制转换为弧度制系数
#define CHANGE_TO_RADIAN    0.017453f  // 3.1415926/180
//弧度制转换为角度制系数
#define CHANGE_TO_ANGLE     57.2958f    //180/3.1415926

#define COSSIN45 0.70710678
//#define K_T 18.f/22.f                    //0.81818181818182                   //0.81818181                    //1.2222222                        //0.81818181          //18/22

#define M_LW 	(32.f)
#define M_L 	(32.f)
#define M_R 	(7.5f)


typedef struct
{
	float x;
	float y;
}POINT_T;

enum
{
	pos1,
	pos2,
	pos_num,
};

typedef struct
{
	POINT_T point;
	float yaw_angle;
	
}ACTline_t;

typedef struct
{
	struct
	{
		float LF_v;
		float LR_v;
		float RR_v;
		float RF_v;
	}Target;
	uint16_t stdid;
	int s0,s1;
	int16_t ch0,ch1,ch2,ch3;
	double vx,vy,vw;
	int16_t v_x,v_y,v_w;
	int sentvalue[4];
//	world_point_t world_point;

}Chassis_Info_t;

extern Chassis_Info_t point,wheel,Remote,visdata;
extern void get_Gimbal_DATE1(uint32_t *canId,uint8_t *rxBuf,Chassis_Info_t*remote);
extern void get_Gimbal_DATE2(uint32_t *canId,uint8_t *rxBuf,Chassis_Info_t*visdata);
extern float Calangle(float angle1,float angle2);
extern float CcltLineAngle(ACTline_t *pointStart, ACTline_t *pointEnd);

#endif
