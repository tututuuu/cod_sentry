/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Vision_Task.h
  * @brief          : Vision task
  * @author         : Yan Yuanbin
  * @date           : 2023/05/21
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef VISION_TASK_H
#define VISION_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the Vision.
 */
typedef struct
{

  float yawerror;      /*!< target pitch angle in radians */
}Vision_Info_Typedef;

/* Exported variables --------------------------------------------------------*/
extern Vision_Info_Typedef Vision_Info;

/* Exported functions prototypes ---------------------------------------------*/

#endif
