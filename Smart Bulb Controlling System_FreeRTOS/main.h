/**
  ******************************************************************************
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

/* LPC Library includes. */
#include "lpc_types.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_rtc.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_pinsel.h"

/* Demo includes. */
#include "basic_io.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "gpio.h"
#include "timer.h"
#include "LCD.h"
#include "adc.h"
#include "extint.h"
#include "myTask.h"


/* Exported types ------------------------------------------------------------*/
 typedef enum
 {
	 utFalse = 0,
	 utTrue  = 1,
 }t_bool;

/* Used as a loop counter to create a very crude delay. */
#define 	mainDELAY_LOOP_COUNT		( 0xfffff )

#define 	NOP		__asm volatile( "NOP" );

/* Exported constants --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/



#ifdef __cplusplus
}

#endif
#endif /* __MAIN_H */

/*****************************END OF FILE****/





