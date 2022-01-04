/* This program demonstrates a use of (HD44780) 16x2 Character Display on SimpleCortex board.
* The LCD is used in 4 bit configuration and following are the hardware connections.
* ---LCD---	--SimpleCortex--
* (2)VCC	5V
* (1)GND	gnd
* (4)RS  	p1.18
* (5)RW	    p1.19
* (6)EN	    p1.20
* (11)DB4	p1.21
* (12)DB5	p1.22
* (13)DB6	p1.23
* (14)DB7	p1.24
*
*/
#ifndef _LCD_H
#define _LCD_H

#include "lpc17xx.h"
#include "lpc_types.h"

#define MAX_CHAR_IN_LINE 16

#define LCD_PORT	 	1
#define LCD_RW_PIN	    18
#define LCD_RS_PIN	    19
#define LCD_EN_PIN	    20
#define LCD_D0_PIN 		21
#define LCD_D1_PIN 		22
#define LCD_D2_PIN 		23
#define LCD_D3_PIN 		24
#define LCD_D4_PIN 		25
#define LCD_D5_PIN 		26
#define LCD_D6_PIN 		27
#define LCD_D7_PIN 		28


enum ROW_NUMBERS
{
LINE1,
LINE2
};


void LCD_init(void);
void LCD_Print(unsigned char line, char *string);
void DelayMS(int count);

#endif
