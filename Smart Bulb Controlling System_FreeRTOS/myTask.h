/**
  ******************************************************************************
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYTASK_H
#define __MYTASK_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
	
/* Exported types ------------------------------------------------------------*/
 typedef struct
 {
	 char c_row1[16];
	 char c_row2[16];
 } LCD_Data;

/* Exported constants --------------------------------------------------------*/
extern unsigned long ulIdleCycleCount;

/* Exported functions ----------------------------------------*/
void vTask_16x2LCD_Print( void *pvParameters );
void vTask_AnalogRead_LDR( void *pvParameters );
void vTask_AnalogRead_PIR( void *pvParameters );
void vTask_SmartBulb_Ctrl( void *pvParameters );
void vTask_Button_Handler( void *pvParameters );
void vTask_RunTimeStats( void *pvParameters );
void vSetupTimerForRunTimeStats( void );
void vTask_Timer_Test( void *pvParameters );
int Delay_ms(uint32_t nCount);

/*-----------------------------------------------------------*/

#ifdef __cplusplus
}

#endif
#endif /* __MAIN_H */

/*****************************END OF FILE****/




