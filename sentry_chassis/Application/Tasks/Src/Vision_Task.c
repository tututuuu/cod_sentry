/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Vision_Task.c
  * @brief          : Vision task
  * @author         : Yan Yuanbin
  * @date           : 2023/07/23
  * @version        : v2.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Vision_Task.h"
#include "INS_Task.h"
#include "api_trajectory.h"

/* Private variables -----------------------------------------------------------*/
/**
 * @brief structure that contains the information for the solved trajectory.
 */
SolveTrajectory_Typedef SolveTrajectory={
  .Camera_Yaw_Vertical = 0.119f,
  .Camera_Yaw_Horizontal = 0.02f,
  .Time_Offset = 0.15f,
	.Armor_Yaw_Limit = 0.005f,
	.Armor_Yaw_Limit_Offset = 0.001f,
};

/**
 * @brief structure that contains the information for the Vision.
 */
Vision_Info_Typedef Vision_Info;

/* USER CODE BEGIN Header_Vision_Task */
/**
* @brief Function implementing the StartVisionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Vision_Task */
void Vision_Task(void const * argument)
{
  /* USER CODE BEGIN Vision_Task */
//  TickType_t systick = 0;

  /* Infinite loop */
  for(;;)
  {
//    systick = osKernelSysTick();

    /* update the transmit euler angle in radians */
    MiniPC_SendPacket.gyro01 = INS_Info.gyro[0];
    MiniPC_SendPacket.gyro02   = INS_Info.gyro[1];
    MiniPC_SendPacket.gyro03  = INS_Info.gyro[2];
		MiniPC_SendPacket.accel01 = INS_Info.accel[0];
		MiniPC_SendPacket.accel02 = INS_Info.accel[1];
		MiniPC_SendPacket.accel03 = INS_Info.accel[2];
		MiniPC_SendPacket.quat01 = Quaternion_Info.quat[0];
		MiniPC_SendPacket.quat02 = Quaternion_Info.quat[1];
		MiniPC_SendPacket.quat03 = Quaternion_Info.quat[2];
		MiniPC_SendPacket.quat04 = Quaternion_Info.quat[3];

//		MiniPC_SendPacket.quat01 = 10;
//		MiniPC_SendPacket.quat02 = 18;
//    MiniPC_SendPacket.quat03 = 10;
//		MiniPC_SendPacket.quat04 = 10;


    /* transmit the minipc frame data */
    MiniPC_SendFrameInfo(&MiniPC_SendPacket);

    osDelay(2);
  }
  /* USER CODE END Vision_Task */
}
//------------------------------------------------------------------------------

