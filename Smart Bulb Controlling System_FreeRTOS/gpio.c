
/**
  ******************************************************************************
  * File Name          : gpio.c
  * Date               : 24/12/2014 09:27:34
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define LED_COUNT (3)

/* LED pins:
   - LED0: P1_11 = GPIO1[11]
   - LED1: P1_29 = GPIO1[29]
   - LED2: P1_31 = GPIO1[31]
 */
const PIN LED_PIN[] =
{
  {1, 11},
  {1, 29},
  {1, 31}
};


void Board_LEDs_Init(GPIO_Mode i_Mode);

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void GPIO_Init(void)
{
	Board_LEDs_Init(OUTPUT_MODE);
	BSP_PB1_Init();
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void Board_LEDs_Init(GPIO_Mode i_Mode)
{
	GPIO_SetDir(RED_LED_PORT, RED_LED_PIN, (char)i_Mode);
	GPIO_SetDir(GREEN_LED_PORT, GREEN_LED_PIN, (char)i_Mode);
	GPIO_SetDir(BLUE_LED_PORT, BLUE_LED_PIN, (char)i_Mode);
	GPIO_SetDir(SMART_BULB_PORT, SMART_BULB_PIN, (char)i_Mode);
	GPIO_SetDir(LCD_BCK_LED_PORT, LCD_BCK_LED_PIN, (char)i_Mode);

	Board_LED_Set(RED_LED, OFF);
	Board_LED_Set(GREEN_LED, OFF);
	Board_LED_Set(BLUE_LED, OFF);
	Board_LED_Set(SMART_BULB,OFF);
	Board_LED_Set(LCD_BCk_LED,OFF);
  return 0;
}


/*-----------------------------------------------------------*/
/* Sets the state of a board LEDs to on or off */
void Board_LED_Set(LED_TypeDef LED, State state)
{
	switch (LED)
	{
		case 1:
		{
			if(state) 	RED_LED_OFF
			else		RED_LED_ON
			break;
		}
		case 2:
		{
			if(state) 	GREEN_LED_OFF
			else		GREEN_LED_ON
			break;
		}
		case 3:
		{
			if(state) 	BLUE_LED_OFF
			else		BLUE_LED_ON
			break;
		}

		case 4:
		{
			if(state) 	SMART_BULB_OFF
			else		SMART_BULB_ON
			break;
		}
		case 5:
		{
			if(state) 	LCD_BCK_LIGHT_OFF
			else		LCD_BCK_LIGHT_ON
			break;
		}
		default :
			break;

	}
}


void BSP_PB_Init(uint8_t PortNum, uint32_t Pin, ButtonMode_TypeDef Mode, Interrupt_TypeDef Edge)
{
	if (Mode == BUTTON_MODE_GPIO)
	{
		// Configure Button pin as input
		GPIO_SetDir(PortNum, Pin, INPUT_MODE);
		GPIO_SetValue(PortNum, Pin);
	}

	if (Mode == BUTTON_MODE_EXTI)
	{
		// PIN_Configure (uint8_t port, uint8_t pin, uint8_t function, uint8_t mode, uint8_t open_drain);
		PIN_Configure(PortNum, Pin, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
		GPIO_SetDir(PortNum, Pin, INPUT_MODE);
		GPIO_IntCmd(PortNum, Pin, Edge);

		// set nvic priority
		NVIC_SetPriority( EINT3_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
		//enable the interrupts on gpio
		NVIC_EnableIRQ(EINT3_IRQn);
	}
}

void BSP_PB1_Init(void)
{
	BSP_PB_Init(BUTTON_PORT, BUTTON_PIN1, BUTTON_MODE_GPIO, 0);
	//BSP_PB_Init(BUTTON_PORT, BUTTON_PIN1, BUTTON_MODE_EXTI, Falling_edge); // Rising_edge
	return;
}


uint8_t BSP_PB1_GetState(void)
{
	return GPIO_GetPinState(BUTTON_PORT, BUTTON_PIN1);
}


void BSP_PB_IRQ_Disable(Button_TypeDef Button)
{

}
void BSP_PB_IRQ_Enable(Button_TypeDef Button)
{

}
void BSP_PB_NVIC_ClearPendingIRQ(Button_TypeDef Button)
{

}

void BUZZER_ON(void)
{

}

void BUZZER_OFF(void)
{
}



#if  0
#define I2CDEV LPC_I2C2

int I2CRead(uint8_t addr, uint8_t* buf, uint32_t len)
{
	I2C_M_SETUP_Type rxsetup;
	rxsetup.sl_addr7bit = addr;
	rxsetup.tx_data = NULL; // Get address to read at writing address
	rxsetup.tx_length = 0;
	rxsetup.rx_data = buf;
	rxsetup.rx_length = len;
	rxsetup.retransmissions_max = 3;
	if (I2C_MasterTransferData(I2CDEV, &rxsetup, I2C_TRANSFER_POLLING) == SUCCESS)
		return 0;
	else
		return -1;
}

int I2CWrite(uint8_t addr, uint8_t* buf, uint32_t len)
{
	I2C_M_SETUP_Type txsetup;
	txsetup.sl_addr7bit = addr;
	txsetup.tx_data = buf;
	txsetup.tx_length = len;
	txsetup.rx_data = NULL;
	txsetup.rx_length = 0;
	txsetup.retransmissions_max = 3;
	if (I2C_MasterTransferData(I2CDEV, &txsetup, I2C_TRANSFER_POLLING) == SUCCESS)
		return 0;
    else
    	return -1;
}

void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;
	/* Initialize I2C2 pin connect
	 * P0.10 - I2C_SDA(Serial Data)
	 * P0.11 - I2C_SCL(Serial Clock line)
	 * */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);
	// Initialize I2C peripheral, 400KHz
	I2C_Init(I2CDEV, 400000);
	/* Enable I2C2 operation */
	I2C_Cmd(I2CDEV, ENABLE);
}
#endif

/**
  * @}
  */

/*****************************END OF FILE****/









