/**
  * @}
  */
/************************************************************/
/************************ prologue **************************/
/*                                                          */
/* University of California Extension, Santa Cruz           */
/*                                                          */
/* Real-time Embedded Systems Programming                   */
/*                                                          */
/* Instructor: Anil Gathala                                 */
/*                                                          */
/* Author: Nagaraju Seelam                                  */
/*                                                          */
/* Assignment : SMART BULB CTRL PRIJECT                     */
/*                                                          */
/*                                                          */
/* Date: 08/31/2016                                         */
/*                                                          */
/* Objective:  Smart bulb controlling system based on LDR,  */
/*             PIR sensors and push button.                 */
/*															*/
/*                                                          */
/************************************************************/
/**
  * @}
  */
 /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "LCD.h"

/* Private define ------------------------------------------------------------*/
#define 	uxInitialCount 	 1
#define 	uxMaxCount 		 100


/* Private typedef -----------------------------------------------------------*/


/* Global variables ---------------------------------------------------------*/
/* Declare a variable of type xQueueHandle. */
xQueueHandle ADC_Queue;
xQueueHandle LCD_Queue;

/* Variables used by the trace hook macros. */
xTaskHandle xTaskThatWasRunning = NULL;

/* Declare a variable of type xSemaphoreHandle. */
xSemaphoreHandle xButtonSemaphore;
xSemaphoreHandle xADCSemaphoreMutex;
xSemaphoreHandle xLCDSemaphoreMutex;
xSemaphoreHandle xSmartBulb_CountingSemaphore =NULL;

/*  Task Handles. */
xTaskHandle AnalogReadLDR_TaskHandle = NULL, AnalogReadPIR_TaskHandle = NULL;
xTaskHandle SmartBulbCtrl_TaskHandle = NULL, Timer_TaskHandle=NULL, Button_TaskHandle = NULL;

/* Private variables ------------------------------------------------------*/

/* Private Functions ------------------------------------------------------*/

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
int main( void )
{
	/* Init the semi-hosting. */
	printf( "\n" );

	// led pins initialization
	GPIO_Init();
	// lcd module initialization
	LCD_init();
	// gpio interrupt initialization
	setup_gpio_interrupt();

	// display welcome message
	LCD_Print(0, " WELCOME TO UCSC ");
	LCD_Print(1, "SYS INITIALISING");

	/* The queue is created to hold a maximum of 5 structures of type xData. */
	LCD_Queue = xQueueCreate( 5, sizeof(LCD_Data) );

	xLCDSemaphoreMutex = xSemaphoreCreateMutex();
	xADCSemaphoreMutex =  xSemaphoreCreateMutex();
	vSemaphoreCreateBinary( xButtonSemaphore );
	xSmartBulb_CountingSemaphore = xSemaphoreCreateCounting (uxMaxCount, uxInitialCount);

	/* Check the semaphore was created successfully. */
	if (xButtonSemaphore == NULL && xLCDSemaphoreMutex == NULL && xADCSemaphoreMutex==NULL && xSmartBulb_CountingSemaphore==NULL)
	{
		printf( "system initialization error");
		return 1;
	}

	/* Creating six  tasks. */
	xTaskCreate(	vTask_AnalogRead_LDR,		/* Pointer to the function that implements the task. */
					"LDR SEN",	/* Text name for the task.  This is to facilitate debugging only. */
					240,		/* Stack depth in words. */
					NULL,		/* We are not using the task parameter. */
					2,			/* This task will run at priority 2. */
					NULL );		/* We are not using the task handle. */

	xTaskCreate(	vTask_AnalogRead_PIR,		/* Pointer to the function that implements the task. */
					"PIR SEN",	/* Text name for the task.  This is to facilitate debugging only. */
					240,		/* Stack depth in words. */
					NULL,		/* We are not using the task parameter. */
					2,			/* This task will run at priority 2. */
					NULL );		/* We are not using the task handle. */

	xTaskCreate(	vTask_SmartBulb_Ctrl,		/* Pointer to the function that implements the task. */
					"SMR BLB",	/* Text name for the task.  This is to facilitate debugging only. */
					240,		/* Stack depth in words. */
					NULL,		/* We are not using the task parameter. */
					3,			/* This task will run at priority 3. */
					NULL );		/* We are not using the task handle. */

	xTaskCreate(	vTask_Button_Handler,		/* Pointer to the function that implements the task. */
					"INT ISR",	/* Text name for the task.  This is to facilitate debugging only. */
					240,		/* Stack depth in words. */
					NULL,		/* We are not using the task parameter. */
					3,			/* This task will run at priority 3. */
					NULL );		/* We are not using the task handle. */

	xTaskCreate(	vTask_16x2LCD_Print,		/* Pointer to the function that implements the task. */
					"LCD DIP",	/* Text name for the task.  This is to facilitate debugging only. */
					240,		/* Stack depth in words. */
					NULL,		/* We are not using the task parameter. */
					0,			/* This task will run at priority 0. */
					NULL );		/* We are not using the task handle. */

	xTaskCreate(	vTask_RunTimeStats,		/* Pointer to the function that implements the task. */
					"RUNSTAT",	/* Text name for the task.  This is to facilitate debugging only. */
					240,		/* Stack depth in words. */
					NULL,		/* We are not using the task parameter. */
					1,			/* This task will run at priority 1. */
					&Timer_TaskHandle  );		/* We are using Timer_TaskHandle task handle. */

	/* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();

	/* If all is well we will never reach here as the scheduler will now be
	running.  If we do reach here then it is likely that there was insufficient
	heap available for the idle task to be created. */
	for( ;; );
	return 0;
}


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
/* Idle hook functions MUST be called vApplicationIdleHook(), take no parameters, and return void. */
void vApplicationIdleHook( void )
{
	/* This hook function does nothing but increment a counter. */
	ulIdleCycleCount++;
}


/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task,
	 queue or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}

/*-----------------------------------------------------------*/
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.
	 Note that stack overflow checking does slow down the context switch implementation. */
	for( ;; );
}

/*-----------------------------------------------------------*/
void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing. */

}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/






