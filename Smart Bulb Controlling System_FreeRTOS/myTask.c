/**
  * @}
  */
 /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <cr_section_macros.h>
#include <NXP/crp.h>

// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;
/**
  * @}
  */
/* Private define ------------------------------------------------------------*/
#define		BLINK_INTERVAL	 100
#define 	ADC_REF_V		 3.3


/* Private typedef -----------------------------------------------------------*/
typedef enum {
	LDR = 0,
	PIR = 1,
}t_sensor;


/* Global variables ---------------------------------------------------------*/
unsigned long ulIdleCycleCount = 0UL;
const portTickType xBlockPeriod_ms = ( 100 / portTICK_RATE_MS );


/* Private variables ------------------------------------------------------*/
bool b_Smart_Bulb     = utFalse;
bool b_Button_State   = utFalse;
bool b_LDR_State      = utFalse;
bool b_Room_Occupancy = utFalse;

static int LDRsensorCutOff=0;
static int PIRsensorCutOff=0;

/* External variables ------------------------------------------------------*/
extern uint32_t timer0_m0_counter, timer1_m0_counter;
extern uint32_t timer0_m1_counter, timer1_m1_counter;

extern xSemaphoreHandle xADCSemaphoreMutex;
extern xSemaphoreHandle xLCDSemaphoreMutex;
extern xSemaphoreHandle xButtonSemaphore;
extern xSemaphoreHandle xSmartBulb_CountingSemaphore;
extern xTaskHandle AnalogReadLDR_TaskHandle, AnalogReadPIR_TaskHandle;
extern xTaskHandle SmartBulbCtrl_TaskHandle, Button_TaskHandle;
extern xQueueHandle LCD_Queue;

/* Private Functions --------------------------------------------------------*/
static int ADC_Read(t_sensor i_Sensor);
static int ADC_Calibration(t_sensor i_Sensor, int * o_ADC_CutOff);

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void vTask_AnalogRead_LDR( void *pvParameters )
{
	LCD_Data pLCD_Data;
	static bool  LDRsensorSwitch=false;
	bool newState = false;
	static int status=0;
	int readADCval=0;

	portTickType  xLastWakeTime1 = xTaskGetTickCount();

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
			vTaskDelayUntil( &xLastWakeTime1,  xBlockPeriod_ms/4 );

		    /* Print out the name of this task. */
		    // vPrintString( pvParameters );

		    if(xSemaphoreTake( xADCSemaphoreMutex, 500/portTICK_RATE_MS ) == pdTRUE)
		    {
				if ( status == 0 )
				{
					LCD_Print(0, " ** LDR TASK ** ");
					LCD_Print(1, "ADC INITIALISING");

					ADC_initialisation();
					vPrintString("\n ADC Initialized \n ");

					//vPrintString("\n Please vary light condition to start light calibration \n");
					status = ADC_Calibration (LDR, &readADCval);

					if ( status )
					{
						LDRsensorCutOff = readADCval;
						vPrintString("\n light calibration Success! \n" );
					}
					else
					{
						vPrintString("\n light calibration Fail! Do again ... \n" );
					}
				}

				if ( status )
				{
					  readADCval = ADC_Read(LDR);  // LDR   PIR

				 	  if (readADCval >= 2800)  ///  LDRsensorCutOff
						  newState =  utTrue;
					  else
						  newState =  utFalse;

					  if (LDRsensorSwitch == false && newState == true)
					  {
						  b_LDR_State = utTrue;
						  vPrintString("\n light sensor turn on light \n");
					  }

					  if (LDRsensorSwitch == true && newState == false)
					  {
						  b_LDR_State = utFalse;
						  vPrintString("\n light sensor turn off light \n");
					  }
			  }

			  if ( LDRsensorSwitch != newState )
			  {
				  LDRsensorSwitch = newState;
				  xSemaphoreGive( xSmartBulb_CountingSemaphore );
			  }

			  //---------- sending message --------
			  sprintf(pLCD_Data.c_row1, "** LDR TASK **");
			  sprintf(pLCD_Data.c_row2, "LDR:%d ADC:%d", b_LDR_State, readADCval);
			  xQueueSendToBack( LCD_Queue, &pLCD_Data, 10 ); // portMAX_DELAY

	    }xSemaphoreGive( xADCSemaphoreMutex );

		//test comment it later
		printf(" LDRstate: %d  ADC: %d\n", b_LDR_State, readADCval);

	}
  return;
}


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void vTask_AnalogRead_PIR( void *pvParameters )
{
	LCD_Data pLCD_Data;
	long Task_Woken=0; // portBASE_TYPE=long // pdFALSE=0
	portTickType  xLastWakeTime2 =  0;
	static bool PIRsensorSwitch=false;
	static bool newState = false;
	static int  status=0;
	int readADCval=0;
	float  voltage;
	static volatile uint32_t OverRun_Counter=0;
	unsigned portBASE_TYPE uxPriority;

	uxPriority = uxTaskPriorityGet( NULL );
	xLastWakeTime2 =  xTaskGetTickCount();

	//xSemaphoreTake( xPIRSemaphoreMutex, 0	 );
	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		 vTaskDelayUntil( &xLastWakeTime2,  xBlockPeriod_ms/4 );

		/* Print out the name of this task. */
		// vPrintString( pvParameters );

		if(xSemaphoreTake( xADCSemaphoreMutex, 500/portTICK_RATE_MS ) == pdTRUE)
		{
			if ( status == 0 )
			{
				LCD_Print(0, " ** PIR TASK ** ");
				LCD_Print(1, "ADC INITIALISING");

				ADC_initialisation();  // remove
				vPrintString("\n ADC Initialized \n ");

				vPrintString("\n Please vary pir status to start pir calibration \n");
				status = ADC_Calibration (PIR, &readADCval);

				if ( status == 1 )
				{
					PIRsensorCutOff = readADCval;
					vPrintString("\n PIR calibration Success!\n" );
				}
				else
				{
					vPrintString("\n PIR calibration Fail! Do again ... \n" );
				}
			}

			if ( status )
			{
				  readADCval = ADC_Read(PIR);
				  voltage = ( readADCval ) * (3.0 / 4095.0);

			      if (readADCval >= 2000) // PIRsensorCutOff
			    	  newState = utTrue;
			      else
			    	  newState = utFalse;

			      if (PIRsensorSwitch == false && newState == true)
			      {
			    	  b_Room_Occupancy = utTrue;
			      	  vPrintString("\n PIR sensor turn on light \n");
			      }

			      if (PIRsensorSwitch == true && newState == false)
			      {
			    	  b_Room_Occupancy = utFalse;
			    	  vPrintString("\n PIR sensor turn off light \n");
			      }
			  }

			  if ( PIRsensorSwitch != newState )
			  {
				  PIRsensorSwitch = newState;
				  xSemaphoreGive( xSmartBulb_CountingSemaphore );
			  }

			  //---------- sending message --------
			  sprintf(pLCD_Data.c_row1, "** PIR TASK **");
			  sprintf(pLCD_Data.c_row2, "PIR:%d ADC:%d", b_Room_Occupancy, readADCval);
			  xQueueSendToBack( LCD_Queue, &pLCD_Data, 10 );  // portMAX_DELAY


		} xSemaphoreGive( xADCSemaphoreMutex );

		printf(" PIRstate:%2d  ADC:%4d \n", b_Room_Occupancy, readADCval );
	}
   return;
}


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void vTask_Button_Handler( void *pvParameters )
{
	bool newButtonState = false;
	static bool ButtonState = false;
	LCD_Data pLCD_Data;
	volatile unsigned long ul;
	portTickType xLastWakeTime3 = xTaskGetTickCount();

	/* Take the semaphore once to start with so the semaphore is empty before the infinite loop is entered.
	   The semaphore was created before the scheduler was started so before this task ran for the first time. */
	xSemaphoreTake( xButtonSemaphore, 0 );

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		if(xSemaphoreTake( xButtonSemaphore, portMAX_DELAY ) == pdTRUE)
		{
			  newButtonState = BSP_PB1_GetState();

			  if ( ButtonState != newButtonState )
			  {
				  ButtonState = newButtonState;
				  sprintf(pLCD_Data.c_row1, "BUTTON CHG STAT:%d", b_Button_State);
			  }
			  else
			  {
				  sprintf(pLCD_Data.c_row1, "NO CHG STAT:%d", b_Room_Occupancy);
			  }
			  sprintf(pLCD_Data.c_row2, "%s", (b_Smart_Bulb>0) ? "  BULB ON " : "  BULB OFF ");
			  xQueueSendToBack( LCD_Queue, &pLCD_Data, 100 ); // portMAX_DELAY

			// test comment it later
			  printf(" Button:%2d  LDR STAT:%2d  RM-Occupancy:%2d\n", b_Button_State, b_LDR_State, b_Room_Occupancy);

			  xSemaphoreGive( xSmartBulb_CountingSemaphore );
		}
	}
}


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void vTask_SmartBulb_Ctrl( void *pvParameters )
{
	bool newButtonState = false;
	static bool newPIRstate = false;
	static bool newLDRstate = false;
	static bool ButtonState = false;
	LCD_Data pLCD_Data;
	volatile unsigned long ul;
	portTickType xLastWakeTime3 = xTaskGetTickCount();

	/* Take the semaphore once to start with so the semaphore is empty before the infinite loop is entered.
	   The semaphore was created before the scheduler was started so before this task ran for the first time. */
	   xSemaphoreTake( xSmartBulb_CountingSemaphore, 0 );


	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		if(xSemaphoreTake( xSmartBulb_CountingSemaphore, portMAX_DELAY ) == pdTRUE)
		{
			  newButtonState = b_Button_State;

			  // reverse the direction of the fading at the ends of the fade:
			  if ((b_Button_State == utTrue) || (  b_LDR_State == utFalse &&  b_Room_Occupancy == utTrue))
			  {
					b_Smart_Bulb = utTrue ;
					Board_LED_Set(SMART_BULB, OFF);
					Board_LED_Set(LCD_BCk_LED, OFF);
					Board_LED_Set(BLUE_LED, ON);
			  }
			  else
			  {
					b_Smart_Bulb = utFalse ;
					Board_LED_Set(SMART_BULB, ON);
					Board_LED_Set(BLUE_LED, OFF);
					Board_LED_Set(LCD_BCk_LED, ON);
			  }

			  if ( ButtonState != newButtonState )
			  {
				  	ButtonState = newButtonState;
				    sprintf(pLCD_Data.c_row1, "BUTTON CHG STAT:%d", b_Button_State);
			  }
			  else
			  if ( newLDRstate != b_LDR_State)
			  {
				    newLDRstate = b_LDR_State;
				    sprintf(pLCD_Data.c_row1, "LDR CHG STAT:%d", b_LDR_State);
			  }
			  else
			  if ( newPIRstate != b_Room_Occupancy)
			  {
				    newPIRstate = b_Room_Occupancy;
				    sprintf(pLCD_Data.c_row1, "PIR CHG STAT:%d", b_Room_Occupancy);
			  }
			  else
			  {
				    sprintf(pLCD_Data.c_row1, "NO CHG STAT:%d", b_Room_Occupancy);
			  }
			  sprintf(pLCD_Data.c_row2, "%s", (b_Smart_Bulb>0) ? "  BULB ON " : "   BULB OFF ");
			  xQueueSendToBack( LCD_Queue, &pLCD_Data, 1000 ); // portMAX_DELAY


			// test comment it later
			 printf( " Button:%2d  LDR STAT:%2d  ROOM Occupancy:%2d\n", b_Button_State, b_LDR_State, b_Room_Occupancy);

		}
	}
}



/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void vTask_RunTimeStats( void *pvParameters )
{
	portTickType xLastExecutionTime;

	/* The buffer used to hold the run time stats text needs to be quite large.  It
	is therefore declared static to ensure it is not allocated on the task stack.
	This makes this function non re-entrant. */
	static signed char cStringBuffer[ 512 ];

	/* The task will run every 5 seconds. */
	const portTickType xBlockPeriod = ( 2000 / portTICK_RATE_MS );

	/* Initialise xLastExecutionTime to the current time.  This is the only
	time this variable needs to be written to explicitly.  Afterwards it is
	updated internally within the xTaskDelayUntil() API function. */
	xLastExecutionTime = xTaskGetTickCount();

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Wait until it is time to run this task again. */
		vTaskDelayUntil( &xLastExecutionTime, xBlockPeriod );

		/* Generate a text table from the run time stats.  This must fit into
		the cStringBuffer array. */
		vTaskGetRunTimeStats( cStringBuffer );

		/* Print out column headings for the run time stats table. */
		consoleprint( "\nTask\t\tAbs Time \t%\n" );
		consoleprint( "------------------------------------\n" );

		/* Print out the run time stats themselves. */
		consoleprint( cStringBuffer );
	}
}


/*-----------------------------------------------------------*/
/* This function configures a timer to be used as the run time statistics time
base.  The function implementation requires knowledge of the LPC17xx peripheral
registers, and uses macros that are available when the LPC17xx.h header file is
used.  In this example LPC17xx.h is included from FreeRTOSConfig.h. */
void vSetupTimerForRunTimeStats( void )
{
	const unsigned long TCR_COUNT_RESET = 2, CTCR_CTM_TIMER = 0x00, TCR_COUNT_ENABLE = 0x01;

	/* Power up and drive the timer 0. */
	LPC_SC->PCONP |= 0x02UL;
	LPC_SC->PCLKSEL0 = ( LPC_SC->PCLKSEL0 & (~(0x3<<2)) ) | ( 0x01 << 2 );

	/* Reset Timer 0 */
	LPC_TIM0->TCR = TCR_COUNT_RESET;

	/* The timer needs to just count up continuously. */
	LPC_TIM0->CTCR = CTCR_CTM_TIMER;

	/* The clock driving the timer is prescaled to a frequency that is good enough
	to get a decent resolution,	but not so fast that the counter value will
	overflow too quickly. */
	LPC_TIM0->PR =  ( configCPU_CLOCK_HZ / 10000UL ) - 1UL;

	/* Start the counter. */
	LPC_TIM0->TCR = TCR_COUNT_ENABLE;
}



/*************************************************************************/
/*************************************************************************/
void vTask_16x2LCD_Print( void *pvParameters )
{
	LCD_Data  pLCD_Data;
	portTickType  xLastWakeTime=0;
	const portTickType xBlockPeriod_ms = 600 / portTICK_RATE_MS;

//	LCD_init();
	xLastWakeTime =  xTaskGetTickCount();

	for( ;; )
	{
		 vTaskDelayUntil( &xLastWakeTime, xBlockPeriod_ms );

		 // Block on the queue to wait for data to arrive.
		 while( xQueueReceive( LCD_Queue, &pLCD_Data, portMAX_DELAY ) != errQUEUE_EMPTY )
		 {
			xSemaphoreTake( xLCDSemaphoreMutex, portMAX_DELAY );
			{
				LCD_Print(0, pLCD_Data.c_row1);
				LCD_Print(1, pLCD_Data.c_row2);
			}
			xSemaphoreGive( xLCDSemaphoreMutex );
			vTaskDelayUntil( &xLastWakeTime,  xBlockPeriod_ms/10 );
		 }
	}
}


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
static int ADC_Read(t_sensor i_Sensor)
{
  int ADC_Value = ADCReadASingleValue(i_Sensor);
  return ADC_Value;
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
static int ADC_Calibration(t_sensor i_Sensor, int * o_ADC_CutOff)
{
	volatile unsigned long ulLoop=10;

	int ADC_High = 0;
	int ADC_Low  = 10000;
	int ADC_Value=0, stat=0, i=0;

	/* suspending the scheduler as method of mutual exclusion. */
	vTaskSuspendAll();
	{
		Board_LED_Set(RED_LED, ON);

		// calibrate
		for(i=0; i<10000; i++ )
		{
			ulLoop = 100;
			ADC_Value = ADCReadASingleValue(i_Sensor);
			if (ADC_Value > ADC_High)
			{
				ADC_High = ADC_Value;
			}
			if (ADC_Value < ADC_Low)
			{
				ADC_Low = ADC_Value;
			}

			while(ulLoop--) NOP;
		 }

		Board_LED_Set(RED_LED, OFF);
		*o_ADC_CutOff = (ADC_High + ADC_Low) /2;
		stat = 1;
	}
	xTaskResumeAll();

  	return stat;
}


/***********************************************************************
 * @brief 		vTask_Timer_Test
 * @param[in]	*pvParameters
 * @param[out]  -
 * @return 		-
 **********************************************************************/
void vTask_Timer_Test( void *pvParameters )
{
   portTickType xLastWakeTime;
   xLastWakeTime = xTaskGetTickCount();

  init_timer(TIMER_0, TIME_INTERVAL);
  //init_timer(TIMER_1, TIME_INTERVAL);

  /* Loop forever */
  while (1) // for( ;; )  /* Loop forever */
  {
	if ( (timer0_m0_counter > 0) && (timer0_m0_counter <= BLINK_INTERVAL) )
	{
		GREEN_LED_ON
	}
	if ( (timer0_m0_counter > BLINK_INTERVAL) && (timer0_m0_counter <= (BLINK_INTERVAL * 4)) )
	{
		GREEN_LED_OFF
	}
	else if ( timer0_m0_counter > (BLINK_INTERVAL * 4) )
	{
	  timer0_m0_counter = 0;
	}
	/*
	// Timer 1 blinky LED 1
	if ( (timer1_m0_counter > 0) && (timer1_m0_counter <= BLINK_INTERVAL) )
	{
		YELLOW_LED_OFF
	}
	if ( (timer1_m0_counter > BLINK_INTERVAL) && (timer1_m0_counter <= (BLINK_INTERVAL * 2)) )
	{
		YELLOW_LED_ON
	}
	else if ( timer1_m0_counter > (BLINK_INTERVAL * 2) )
	{
	  timer1_m0_counter = 0;
	}
	*/
  }
}


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
int Delay_ms(uint32_t nCount)
{
	volatile unsigned long ul;

	/* Delay for a period. */
	for( ul = 0; ul < nCount; ul++ )
	{
		NOP
		NOP
		/* This loop is just a very crude delay implementation.  There is
		nothing to do in here.  Later exercises will replace this crude
		loop with a proper delay/sleep function. */
	}
	return 1;
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/





