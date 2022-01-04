/**
  ******************************************************************************
  * File Name          : gpio.h
  * Date               : 15/12/2014 16:30:38
  * Description        : This file contains all the functions prototypes for gpio  
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

 typedef enum
 {
 	ON  = 0,
 	OFF = 1
 } State;


 typedef enum
 {
 	INPUT_MODE  = 0,
 	OUTPUT_MODE = 1
 } GPIO_Mode;


typedef enum 
{  
	RED_LED     = 1,
	GREEN_LED   = 2,
	BLUE_LED    = 3,
	SMART_BULB  = 4,
	LCD_BCk_LED = 5
} LED_TypeDef;

typedef enum
{
  BUTTON_KEY1 = 0,
  BUTTON_KEY2 = 1,
  BUTTON_KEY3 = 2

} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;    

typedef enum
{
	Rising_edge  = 0,
	Falling_edge = 1
} Interrupt_TypeDef;

// on board RGB led
#define RED_LED_PORT       	0
#define GREEN_LED_PORT      3
#define BLUE_LED_PORT       3
#define RED_LED_PIN         22
#define GREEN_LED_PIN       25
#define BLUE_LED_PIN        26

// smart bulb
#define SMART_BULB_PORT     2
#define SMART_BULB_PIN	    11

// LCD back light
#define LCD_BCK_LED_PORT    2
#define LCD_BCK_LED_PIN	    12

#define BUTTON_PORT        	0
#define BUTTON_PIN1         9
#define BUTTON_PIN2        	8
#define BUTTON_PIN3         7


/************************************************************/
/*
#define RED_LED_DIR	  	 	IO0DIR // functionality
#define RED_LED_SET	   		IO0SET
#define RED_LED_CLR	   		IO0CLR
#define RED_LED_READ   		IO1PIN  // read status
*/

#define RED_LED_ON     		GPIO_ClearValue(RED_LED_PORT, RED_LED_PIN);
#define RED_LED_OFF	      	GPIO_SetValue(RED_LED_PORT, RED_LED_PIN);

#define GREEN_LED_ON     	GPIO_ClearValue(GREEN_LED_PORT, GREEN_LED_PIN);
#define GREEN_LED_OFF	    GPIO_SetValue(GREEN_LED_PORT, GREEN_LED_PIN);

#define BLUE_LED_ON     	GPIO_ClearValue(BLUE_LED_PORT, BLUE_LED_PIN);
#define BLUE_LED_OFF	    GPIO_SetValue(BLUE_LED_PORT, BLUE_LED_PIN);

#define SMART_BULB_ON    	GPIO_ClearValue( SMART_BULB_PORT, SMART_BULB_PIN );
#define SMART_BULB_OFF	    GPIO_SetValue( SMART_BULB_PORT, SMART_BULB_PIN );

#define LCD_BCK_LIGHT_ON   	GPIO_ClearValue(LCD_BCK_LED_PORT, LCD_BCK_LED_PIN);
#define LCD_BCK_LIGHT_OFF   GPIO_SetValue(LCD_BCK_LED_PORT, LCD_BCK_LED_PIN);


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/

void GPIO_Init(void);
void Board_LED_Set(LED_TypeDef i_LED, State state);

void BSP_PB_Init(uint8_t PortNum, uint32_t Pin, ButtonMode_TypeDef Mode, Interrupt_TypeDef Edge);
void BSP_PB1_Init(void);
uint8_t BSP_PB1_GetState(void);

uint8_t BSP_PB_GetState(Button_TypeDef Button);
void BSP_PB_IRQ_Disable(Button_TypeDef Button);
void BSP_PB_IRQ_Enable(Button_TypeDef Button);
void BSP_PB_NVIC_ClearPendingIRQ(Button_TypeDef Button);

void BUZZER_ON(void);
void BUZZER_OFF(void);

#ifdef __cplusplus

}

#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/*****************************END OF FILE****/




