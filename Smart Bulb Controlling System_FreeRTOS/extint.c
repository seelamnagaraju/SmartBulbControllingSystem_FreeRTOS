/****************************************************************************
 *   $Id:: extint.c 5670 2010-11-19 01:33:16Z usb00423                      $
 *   Project: NXP LPC17xx EINT example
 *
 *   Description:
 *     This file contains EINT code example which include EINT 
 *     initialization, EINT interrupt handler, and APIs for EINT.
 *
*****************************************************************************/

#include "FreeRTOSConfig.h"
#include "lpc17xx.h"
#include "projdefs.h"
#include "gpio.h"
#include "extint.h"
#include "semphr.h"

volatile uint32_t eint0_counter;
volatile uint32_t eint3_counter;

extern xSemaphoreHandle xSmartBulb_CountingSemaphore;
extern xSemaphoreHandle xButtonSemaphore;
extern bool b_Button_State;

/****************************************************************************/
/****************************************************************************/
uint32_t setup_gpio_interrupt(void)
{
	//configure pin for gpio interrupt
	BSP_PB_Init (BUTTON_PORT, BUTTON_PIN1, BUTTON_MODE_EXTI, Rising_edge);  //  Falling_edge
																			//  Rising_edge
	//BSP_PB_Init (BUTTON_PORT, BUTTON_PIN2, BUTTON_MODE_EXTI, Falling_edge);  //  Falling_edge

	return 0;
}


/***************************************************************************
** Function name:		EINT_Handler
****************************************************************************/
void EINT3_IRQHandler (void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	NVIC_DisableIRQ(EINT3_IRQn);

#if 0
	eint3_counter++;
	if ( eint3_counter & 0x01 )	/* alternate the LED display */
	{
	  RED_LED_ON	/* turn on  */
	  GREEN_LED_OFF	/* turn oFF  */
	}
	else
	{
	  RED_LED_OFF	/* turn off  */
	  GREEN_LED_ON	/* turn on */
	}
#endif

#if 1
	  if (GPIO_GetIntStatus(BUTTON_PORT, BUTTON_PIN1, Rising_edge))
	  {
		  // Raising edge interrupt on pin 0.9 was fired
		  GPIO_ClearInt(BUTTON_PORT, BUTTON_PIN1); 	/* clear interrupt status */
		  printf("\n Rising edge on BUTTON_PIN1 :  Button toggles light \n");
		  b_Button_State = !b_Button_State;
	  }
	  if (GPIO_GetIntStatus(BUTTON_PORT, BUTTON_PIN1, Falling_edge))
	  {
		  // Falling edge interrupt on pin 0.9 was fired
		  GPIO_ClearInt(BUTTON_PORT, BUTTON_PIN1); 	/* clear interrupt status */
		  printf("falling edge on BUTTON_PIN1 \n");
	  }
	  if (GPIO_GetIntStatus(BUTTON_PORT, BUTTON_PIN2, Rising_edge))
	  {
		  // Raising edge interrupt on pin 0.9 was fired
		  GPIO_ClearInt(BUTTON_PORT, BUTTON_PIN2); 	/* clear interrupt status */
		  printf("rising edge on BUTTON_PIN2 \n");
	  }
	  if (GPIO_GetIntStatus(BUTTON_PORT, BUTTON_PIN2, Falling_edge))
	  {
		  // Falling edge interrupt on pin 0.9 was fired
		  GPIO_ClearInt(BUTTON_PORT, BUTTON_PIN2); 	/* clear interrupt status */
		  printf("falling edge on BUTTON_PIN2 \n");
	  }

#endif
/**/

	  NVIC_EnableIRQ(EINT3_IRQn);


      /* 'Give' the semaphore to unblock the task. */
      xSemaphoreGiveFromISR( xButtonSemaphore, &xHigherPriorityTaskWoken );
     // xSemaphoreGiveFromISR( xSmartBulb_CountingSemaphore, &xHigherPriorityTaskWoken );

      portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
      return;
}


/*****************************************************************************
** Function name:		EINT0_Handler
**
** Descriptions:		external INT handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void EINT0_IRQHandler (void) 
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  LPC_SC->EXTINT = EINT0;		/* clear interrupt */
  NVIC_DisableIRQ(EINT0_IRQn);

  eint0_counter++;
  if ( eint0_counter & 0x01 )	/* alternate the LED display */
  {
	  RED_LED_ON	/* turn on  */
	  GREEN_LED_OFF	/* turn oFF  */
  }
  else
  {
	  RED_LED_OFF	/* turn off  */
	  GREEN_LED_ON	/* turn on */
  }

  NVIC_EnableIRQ(EINT0_IRQn);

  /* 'Give' the semaphore to unblock the task. */
   xSemaphoreGiveFromISR( xSmartBulb_CountingSemaphore, &xHigherPriorityTaskWoken );

   portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
   return;
}


/*****************************************************************************
** Function name:		EINTInit
**
** Descriptions:		Initialize external interrupt pin and
**						install interrupt handler
**
** parameters:			None
** Returned value:		true or false, return false if the interrupt
**						handler can't be installed to the VIC table.
** 
*****************************************************************************/
uint32_t EINTInit( void )
{
  LPC_PINCON->PINSEL4 = 0x00100000;	/* set P2.10 as EINT0 and P2.0~7 GPIO output */

  LPC_GPIOINT->IO2IntEnF = 0x200;	/* Port2.10 is falling edge. */
  LPC_SC->EXTMODE = EINT0_EDGE;		/* INT0 edge trigger */
  LPC_SC->EXTPOLAR = 0;				/* INT0 is falling edge by default */

  NVIC_EnableIRQ(EINT0_IRQn);
  return( TRUE );
}

/******************************************************************************
**                            End Of File
******************************************************************************/



