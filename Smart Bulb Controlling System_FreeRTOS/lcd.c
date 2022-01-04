#include "main.h"
#include "lcd.h"

typedef enum
{
	CMD  = 0,
	DATA = 1
} LCD_TypeDef;



void Lcd_Gpio_Init(void);
void Lcd_Reset(void);
void send_Data(LCD_TypeDef i_info, char nibble);
void EN_Pulse(void);
void LcdCmdWrite(char cmd);
void LcdDataWrite(char data);
void Lcd_Shift_Right();
void Lcd_Shift_Left();
void lcd_clear( void);
int lcd_gotoxy( unsigned int x, unsigned int y);


/******************************************************************************************/
/******************************************************************************************/
void LCD_init( void )
{
	Lcd_Gpio_Init();        //LCD Initialization
	LcdCmdWrite(0x38);      //Display Off, Cursor Off, Blink Off
	DelayMS(10);
	LcdCmdWrite(0x0E);      // Clear Screen & Returns the Cursor Home
	DelayMS(10);
	LcdCmdWrite(0x01);
	DelayMS(10);
	LcdCmdWrite(0x06);      //Inc cursor to the right when writing and don’t shift screen
	DelayMS(10);
	lcd_clear() ;
	LcdCmdWrite(0x80);
    return;
}

/******************************************************************************************/
void LCD_Print(unsigned char line, char *string)
{
	unsigned char len = MAX_CHAR_IN_LINE;
	lcd_gotoxy( line, 0 );    // Move the cursor to line
	printf(string); // test
	printf('\n');

	while(*string != '\0' && len--)
	{
		LcdDataWrite( *string );
		string++;
	}
	return;
}


/******************************************************************************************/
void LcdCmdWrite(char cmd)
{
	GPIO_PinWrite (LCD_PORT, LCD_RS_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_EN_PIN, 0);
	send_Data(0, cmd);
	EN_Pulse();
	return;
}

/******************************************************************************************/
void LcdDataWrite(char data)
{
	send_Data(1, data);
	GPIO_PinWrite (LCD_PORT, LCD_EN_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_RS_PIN, 1);
	EN_Pulse();
	return;
}

/******************************************************************************************/
void Lcd_Shift_Right()
{
	LcdCmdWrite(0x1C);
}

/******************************************************************************************/
void Lcd_Shift_Left()
{
	LcdCmdWrite(0x18);
}

/*****************************************************************************************
Function Name :	lcd_clear()
*****************************************************************************************/
void lcd_clear( void)
{
	LcdCmdWrite(0x01);
	DelayMS(10);
	return;
}

/*******************************************************************************************
Function Name :	lcd_gotoxy()
*********************************************************************************************/
int lcd_gotoxy( unsigned int x, unsigned int y)
{
	int retval = 0;

	if( (x > 1) && (y > 15) )
	{
		retval = -1;
	}
	else
	{
		if( x == 0 )  {
			lcd_clear();
			LcdCmdWrite( 0x80 + y );	/* command - position cursor at 0x00 (0x80 + 0x00 ) */
		}
		else if( x==1 ){
			LcdCmdWrite( 0xC0 + y );	/* command - position cursor at 0x40 (0x80 + 0x00 ) */
		}
	}
	return retval;
}


/******************************************************************************************/
void EN_Pulse(void)
{
  GPIO_PinWrite (LCD_PORT, LCD_EN_PIN, 1); //set EN to high
  DelayMS(10);
  GPIO_PinWrite (LCD_PORT, LCD_EN_PIN, 0); //set EN to low
}

/******************************************************************************************/
void Lcd_Gpio_Init(void)
{
	PIN_Configure(LCD_PORT, LCD_RS_PIN, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
	PIN_Configure(LCD_PORT, LCD_RW_PIN, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
	PIN_Configure(LCD_PORT, LCD_EN_PIN, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
	PIN_Configure(LCD_PORT, LCD_D0_PIN, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
	PIN_Configure(LCD_PORT, LCD_D1_PIN, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
	PIN_Configure(LCD_PORT, LCD_D2_PIN, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
	PIN_Configure(LCD_PORT, LCD_D3_PIN, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
	PIN_Configure(LCD_PORT, LCD_D4_PIN, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
	PIN_Configure(LCD_PORT, LCD_D5_PIN, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
	PIN_Configure(LCD_PORT, LCD_D6_PIN, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
	PIN_Configure(LCD_PORT, LCD_D7_PIN, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
	DelayMS(100);
	GPIO_SetDir(LCD_PORT, LCD_RS_PIN, GPIO_DIR_OUTPUT);
	GPIO_SetDir(LCD_PORT, LCD_RW_PIN, GPIO_DIR_OUTPUT);
	GPIO_SetDir(LCD_PORT, LCD_EN_PIN, GPIO_DIR_OUTPUT);
	GPIO_SetDir(LCD_PORT, LCD_D0_PIN, GPIO_DIR_OUTPUT);
	GPIO_SetDir(LCD_PORT, LCD_D1_PIN, GPIO_DIR_OUTPUT);
	GPIO_SetDir(LCD_PORT, LCD_D2_PIN, GPIO_DIR_OUTPUT);
	GPIO_SetDir(LCD_PORT, LCD_D3_PIN, GPIO_DIR_OUTPUT);
	GPIO_SetDir(LCD_PORT, LCD_D4_PIN, GPIO_DIR_OUTPUT);
	GPIO_SetDir(LCD_PORT, LCD_D5_PIN, GPIO_DIR_OUTPUT);
	GPIO_SetDir(LCD_PORT, LCD_D6_PIN, GPIO_DIR_OUTPUT);
	GPIO_SetDir(LCD_PORT, LCD_D7_PIN, GPIO_DIR_OUTPUT);
	DelayMS(100);
	GPIO_PinWrite (LCD_PORT, LCD_RS_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_RW_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_EN_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D0_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D1_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D2_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D3_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D4_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D5_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D6_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D7_PIN, 0);
	Lcd_Reset();
	DelayMS(10);
	return;
}

/******************************************************************************************/
void Lcd_Reset(void)
{
	GPIO_PinWrite (LCD_PORT, LCD_RS_PIN, 1);
	GPIO_PinWrite (LCD_PORT, LCD_RW_PIN, 1);
	GPIO_PinWrite (LCD_PORT, LCD_EN_PIN, 1);

	GPIO_PinWrite (LCD_PORT, LCD_D0_PIN, 1);
	GPIO_PinWrite (LCD_PORT, LCD_D1_PIN, 1);
	GPIO_PinWrite (LCD_PORT, LCD_D2_PIN, 1);
	GPIO_PinWrite (LCD_PORT, LCD_D3_PIN, 1);
	GPIO_PinWrite (LCD_PORT, LCD_D4_PIN, 1);
	GPIO_PinWrite (LCD_PORT, LCD_D5_PIN, 1);
	GPIO_PinWrite (LCD_PORT, LCD_D6_PIN, 1);
	GPIO_PinWrite (LCD_PORT, LCD_D7_PIN, 1);

	GPIO_PinWrite (LCD_PORT, LCD_RS_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_RW_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_EN_PIN, 0);
	DelayMS(10000);

	GPIO_PinWrite (LCD_PORT, LCD_D4_PIN, 1);
	GPIO_PinWrite (LCD_PORT, LCD_D5_PIN, 1);
	EN_Pulse();
	DelayMS(10000);

	GPIO_PinWrite (LCD_PORT, LCD_D4_PIN, 1);
	GPIO_PinWrite (LCD_PORT, LCD_D5_PIN, 1);
	EN_Pulse();
	DelayMS(10000);

	GPIO_PinWrite (LCD_PORT, LCD_D4_PIN, 1);
	GPIO_PinWrite (LCD_PORT, LCD_D5_PIN, 1);
	EN_Pulse();
	DelayMS(10000);

	GPIO_PinWrite (LCD_PORT, LCD_D4_PIN, 1);
	GPIO_PinWrite (LCD_PORT, LCD_D5_PIN, 0);
	EN_Pulse();
	DelayMS(10000);
	return;
}

/******************************************************************************************/
void send_Data(LCD_TypeDef i_info, char nibble)
{
	 // Clear previous data
	GPIO_PinWrite (LCD_PORT, LCD_D0_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D1_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D2_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D3_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D4_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D5_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D6_PIN, 0);
	GPIO_PinWrite (LCD_PORT, LCD_D7_PIN, 0);

	GPIO_PinWrite (LCD_PORT, LCD_D0_PIN, ((nibble >>0x00) & 0x01));
	GPIO_PinWrite (LCD_PORT, LCD_D1_PIN, ((nibble >>0x01) & 0x01));
	GPIO_PinWrite (LCD_PORT, LCD_D2_PIN, ((nibble >>0x02) & 0x01));
	GPIO_PinWrite (LCD_PORT, LCD_D3_PIN, ((nibble >>0x03) & 0x01));
	GPIO_PinWrite (LCD_PORT, LCD_D4_PIN, ((nibble >>0x04) & 0x01));
	GPIO_PinWrite (LCD_PORT, LCD_D5_PIN, ((nibble >>0x05) & 0x01));
	GPIO_PinWrite (LCD_PORT, LCD_D6_PIN, ((nibble >>0x06) & 0x01));
	GPIO_PinWrite (LCD_PORT, LCD_D7_PIN, ((nibble >>0x07) & 0x01));
	return;
}


/*********************************************************************************************/
void DelayMS(int count)
{
	volatile unsigned int j=0,i=0;
	for(j=0;j<count;j++)
	{
		/* DelayMS of 10 us */
		for(i=0;i<10;i++);
	}
}

/*********************************************************************************************/

