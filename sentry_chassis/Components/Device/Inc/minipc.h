/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : minipc.c
  * @brief          : minipc interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MINIPC_H
#define DEVICE_MINIPC_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/**
 * @brief macro definition Number of MiniPC data to be sent
 */
#define MINIPC_SENDLENGTH     28U

/**
 * @brief macro definition Number of MiniPC to be receive
 */
#define MINIPC_REVCLENGTH     48U


/* Exported types ------------------------------------------------------------*/

/* cancel byte alignment */
#pragma  pack(1)

/**
 * @brief typedef structure that contains the information  for the MiniPC Receive Data.
 */
typedef struct 
{
  uint8_t header;
	float quat01;//IMU
	float quat02;
	float quat03;
	float quat04;
	float gyro01;
	float gyro02;
	float gyro03;
	float accel01;
	float accel02;
	float accel03;
  uint16_t checksum;
}MiniPC_SendPacket_Typedef;

/**
 * @brief typedef structure that contains the information for the MiniPC Transmit Data.
 */
typedef struct 
{
  uint8_t header;

  float vx;
  float vy;
  float vz;

  uint16_t checksum;
}MiniPC_ReceivePacket_Typedef;

/* restore byte alignment */
#pragma  pack()


/* Exported variables --------------------------------------------------------*/
/**
 * @brief structure that contains the information for the MiniPC Transmit Data.
 */
extern MiniPC_SendPacket_Typedef MiniPC_SendPacket;
/**
 * @brief structure that contains the information for the MiniPC Receive Data.
 */
extern MiniPC_ReceivePacket_Typedef MiniPC_ReceivePacket;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Send the MiniPC frame Information according the USB CDC
  */
extern void MiniPC_SendFrameInfo(MiniPC_SendPacket_Typedef *SendPacket);
/**
  * @brief  Receive the MiniPC frame Information according the USB CDC
  */
extern void MiniPC_RecvFrameInfo(uint8_t* Buf, const uint32_t *Len);

#endif //DEVICE_MINIPC_H

