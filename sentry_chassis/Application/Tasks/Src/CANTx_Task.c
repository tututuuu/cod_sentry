#include "CANTx_Task.h"
#include "cmsis_os.h"
#include "Chassis_Task.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "motor.h"

void CANTx_Task(void const * argument)
{
   
  for(;;)
  {
		if((Remote.s0 == 2 && Remote.s1==2) || Remote.s0==0 || Remote.s1==0)
		{
			wheel.sentvalue[0] = 0;
			wheel.sentvalue[1] = 0;
			wheel.sentvalue[2] = 0;
			wheel.sentvalue[3] = 0;	
		}
		 ChassisTxFrame.data[0] = (uint8_t)(wheel.sentvalue[0] >>8) ;
		 ChassisTxFrame.data[1] = (uint8_t)(wheel.sentvalue[0] );
		 ChassisTxFrame.data[2] = (uint8_t)(wheel.sentvalue[1] >>8) ;
		 ChassisTxFrame.data[3] = (uint8_t)(wheel.sentvalue[1] );
		 ChassisTxFrame.data[4] = (uint8_t)(wheel.sentvalue[2] >>8) ;
		 ChassisTxFrame.data[5] = (uint8_t)(wheel.sentvalue[2] );
		 ChassisTxFrame.data[6] = (uint8_t)(wheel.sentvalue[3] >>8) ;
		 ChassisTxFrame.data[7] = (uint8_t)(wheel.sentvalue[3] );
		 USER_CAN_TxMessage(&ChassisTxFrame);
		
		osDelay(1);
	}

}
