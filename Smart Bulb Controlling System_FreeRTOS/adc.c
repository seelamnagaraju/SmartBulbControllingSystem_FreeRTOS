/****************************************************************************
 *   $Id:: adc.c 6089 2011-01-06 04:38:09Z nxp12832                         $
 *   Project: NXP LPC17xx ADC example
 *
 *   Description:
 *     This file contains ADC code example which include ADC 
 *     initialization, ADC interrupt handler, and APIs for ADC
 *     reading.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/
#include "lpc17xx.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_adc.h"
#include "adc.h"

#ifndef _BV
#define _BV(_x_) (1UL << (_x_))
#endif

// #define ADC_DONE 	(1 << 31)
// #define ADC_OVERRUN 	(1 << 30)

extern uint32_t SystemCoreClock;

volatile uint32_t ADCValue[ADC_NUM];
volatile uint32_t ADCIntDone = 0;
volatile uint32_t BurstCounter = 0;
volatile uint32_t OverRunCounter = 0;

#define DEBUG  0

#if BURST_MODE
volatile uint32_t channel_flag = 0; 
#endif

#if ADC_INTERRUPT_FLAG
/******************************************************************************
** Function name:		ADC_IRQHandler
**
** Descriptions:		ADC interrupt handler
**
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
void ADC_IRQHandler (void) 
{
  uint32_t regVal;
  volatile uint32_t dummy;
  int i;
  
  regVal = LPC_ADC->ADSTAT;		/* Read ADC will clear the interrupt */
  if ( regVal & 0x0000FF00 )	/* check OVERRUN error first */
  {
	OverRunCounter++;
  	for ( i = 0; i < ADC_NUM; i++ )
  	{
  	  regVal = (regVal & 0x0000FF00) >> 0x08;
  	  /* if overrun, just read ADDR to clear */
  	  /* regVal variable has been reused. */
      if ( regVal & _BV(i) )
  	  {
        dummy = LPC_ADC->DR[i];
  	  }
  	}
	LPC_ADC->ADCR &= ~((0x7<<24)|(0x1<<16));	/* stop ADC now, turn off BURST bit. */
  	ADCIntDone = 1;
  	return;	
  }

  for ( i = 0; i < ADC_NUM; i++ )
  {
    if ( regVal & _BV(i) )
	{
	  ADCValue[i] = ( LPC_ADC->DR[i] >> 4 ) & 0xFFF;
	}
  }

#if BURST_MODE
  BurstCounter++;
  channel_flag |= (regVal & 0xFF);
  if ( (channel_flag & 0xFF) == 0xFF )
  {
	/* All the bits in have been set, it indicates all the ADC 
	channels have been converted. */
	LPC_ADC->ADCR &= ~(0x1<<16);	/* Clear BURST mode, stop ADC now */
  	channel_flag = 0; 
	ADCIntDone = 1;
  }
#else
  LPC_ADC->ADCR &= ~(0x7<<24);	/* stop ADC now */
  ADCIntDone = 1;
#endif
  return;
}
#endif

/*****************************************************************************
** Function name:		ADCInit
**
** Descriptions:		initialize ADC channel
**
** parameters:			ADC clock rate
** Returned value:		None
** 
*****************************************************************************/
void ADCInit( uint32_t ADC_Clk )
{
  for ( int i = 0; i < ADC_NUM; i++ ) {
	ADCValue[i] = 0x00;
  }

 // Step 1: enable clock to ADC
  LPC_SC->PCONP |= (1 << 12);

  //Step 2: configure ADC 0-7 pins for ADC functionality
  LPC_PINCON->PINSEL0 &= ~0x000000F0;    /* P0.2-3, A0.6-7,   pin function 10 */
  LPC_PINCON->PINSEL0 |= 0x000000A0;
  LPC_PINCON->PINSEL1 &= ~0x003FC000;    /* P0.23-26, A0.0-3, pin function 01 */
  LPC_PINCON->PINSEL1 |= 0x00154000;
  LPC_PINCON->PINSEL3 |= 0xF0000000;    /* P1.30-31, A0.4-5,  pin function 11 */

  // Step 3: select neither pull-up nor pull-down (function 10) on all ADC pins
  LPC_PINCON->PINMODE0 &= ~0x000000F0;
  LPC_PINCON->PINMODE0 |= 0x000000A0;
  LPC_PINCON->PINMODE1 &= ~0x003FC000;
  LPC_PINCON->PINMODE1 |= 0x002A8000;
  LPC_PINCON->PINMODE3 &= ~0xF0000000;
  LPC_PINCON->PINMODE3 |= 0xA0000000;

#if DEBUG
  //checkout the system clock
  printf("\n SystemCoreClock = %d\n", SystemCoreClock);
#endif

  //Step 4:configure A/D control register
  LPC_ADC->ADCR = ( 0x01 << 0 ) |  /* SEL=1,select channel 0~7 on ADC0 */
        ( ( SystemCoreClock / ADC_Clk - 1 ) << 8 ) |  /* CLKDIV = Fpclk / ADC_Clk - 1 */
        ( 0 << 16 ) |         /* BURST = 0, no BURST, software controlled */
        ( 1 << 21 ) |          /* PDN = 1, normal operation */
        ( 0 << 24 ) |          /* START = 0 A/D conversion stops */
        ( 0 << 27 );        /* EDGE = 0 (CAP/MAT singal falling,trigger A/D conversion) */


  /* If POLLING, no need to do the following */
  #if ADC_INTERRUPT_FLAG
    NVIC_EnableIRQ(ADC_IRQn);
  #if BURST_MODE
    LPC_ADC->ADINTEN = 0xFF;		/* Enable all interrupts */
  #else
    LPC_ADC->ADINTEN = 0x1FF;		/* Enable all interrupts */
  #endif
  #endif
  return;
}

uint32_t ADCReadASingleValue( uint8_t channelNum )
{
  uint32_t regVal, ADC_Data;

  //Step 1. disable all channels: 0-7
  LPC_ADC->ADCR &= 0xFFFFFF00;

  // Step 2: enable just the channel that we are interested in
  // Step 3: start A/D conversion
  LPC_ADC->ADCR |= (1 << 24) | (1 << channelNum);

  // Step 4: in a loop wait until the A/D conversion is done
  while ( 1 )
  {
    regVal = LPC_ADC->ADGDR;
    /* read result of A/D conversion */
    if ( regVal & ADC_DONE )
    {
      break;
    }
  }

  // Step 5: stop A/D conversion for now
  LPC_ADC->ADCR &= 0xF8FFFFFF;

  // Step 6: check if there was an overrun and if lost any data
  if ( regVal & ADC_OVERRUN )
  {
//if we lost data, return zero.
    return ( 0 );
  }

  // Step 7: extract actual data from the data register
  ADC_Data = ( regVal >> 4 ) & 0xFFF;

  // Step 8: return the ADC data
  return ( ADC_Data );
}


int ADC_initialisation(void)
{
   //Step 1: First, initialize the A/D convertor
   ADCInit(ADC_CLK); //1 MHZ clock rate

   //Step 2: in a loop read values from AD.0 and print them out
   for ( int i = 0; i < ADC_NUM; i++ )
   {
       ADCValue[i] = ADCReadASingleValue(i);
#if DEBUG
       printf("\n ADC CH:%d Value = %d \n", i+1, ADCValue[i]);
#endif
   }
   return 1;
}




/*********************************************************************************
**
*********************************************************************************/
#define ADC_RESOLUTION        12        /* Number of A/D converter bits       */

static volatile uint16_t AD_last;       /* Last converted value               */
static volatile uint8_t  AD_done;       /* AD conversion done flag            */


/* ADC pins:   AD0.2: P0_25    */
const PIN ADC_PIN[] = {  {0, 25}, {0, 26} };


/**-----------------------------------------------------------
  \fn          void ADC_IRQHandler (void)
  \brief       Analog-to-Digital Converter Interrupt Handler
--------------------------------------------------------------*/
void ADC_IRQHandler(void)
{
  volatile uint32_t adstat;

  adstat = LPC_ADC->ADSTAT;                        /* Read ADC clears interrupt     */

  AD_last = (LPC_ADC->ADGDR >> 4) & 0xFFF;         /* Store converted value   */

  AD_done = 1;
}


/**-----------------------------------------------------------
  \fn          int32_t ADC_Initialize (void)
  \brief       Initialize Analog-to-Digital Converter
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*-----------------------------------------------------------*/
int32_t ADC_Initialize (void)
{
  LPC_SC->PCONP |= ((1 << 12) | (1 << 15));  /* enable power to ADC & IOCON   */

  PIN_Configure (ADC_PIN[0].Portnum, ADC_PIN[0].Pinnum, PIN_FUNC_1, PIN_PINMODE_TRISTATE, PIN_PINMODE_NORMAL);



  LPC_ADC->ADCR    =  ( 1 <<  2) |           /* select AD0.2 pin              */
                      ( 4 <<  8) |           /* ADC clock is 25MHz/5          */
                      ( 1 << 21);            /* enable ADC                    */

  LPC_ADC->ADINTEN =  ( 1 <<  8);            /* global ADC enable interrupt   */

  NVIC_EnableIRQ(ADC_IRQn);                  /* enable ADC Interrupt          */

  return 0;
}

/**-----------------------------------------------------------
  \fn          int32_t ADC_Uninitialize (void)
  \brief       De-initialize Analog-to-Digital Converter
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*-----------------------------------------------------------*/
int32_t ADC_Uninitialize (void) {

  NVIC_DisableIRQ (ADC_IRQn);            /* Disable ADC Interrupt               */
  LPC_ADC->ADINTEN &= ~( 1 <<  8);       /* Disable global ADC enable interrupt */

  PIN_Configure (ADC_PIN[0].Portnum, ADC_PIN[0].Pinnum, 0, 0, 0);

  LPC_SC->PCONP &= ~(1 << 12);           /* Disable ADC clock */
  return 0;
}

/**-----------------------------------------------------------
  \fn          int32_t ADC_StartConversion (void)
  \brief       Start conversion
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*-----------------------------------------------------------*/
int32_t ADC_StartConversion (void)
{
  LPC_ADC->ADCR &= ~( 7 << 24);              /* stop conversion               */
  LPC_ADC->ADCR |=  ( 1 << 24);              /* start conversion              */

  return 0;
}

/**-----------------------------------------------------------
  \fn          int32_t ADC_ConversionDone (void)
  \brief       Check if conversion finished
  \returns
   - \b  0: conversion finished
   - \b -1: conversion in progress
*-----------------------------------------------------------*/
int32_t ADC_ConversionDone (void) {
  return (AD_done ? 0 : -1);
}

/**-----------------------------------------------------------
  \fn          int32_t ADC_GetValue (void)
  \brief       Get converted value
  \returns
   - <b> >=0</b>: converted value
   - \b -1: conversion in progress or failed
*-----------------------------------------------------------*/
int32_t ADC_GetValue (void) {

  if (AD_done) {
    AD_done = 0;
    return AD_last;
  }
  return -1;
}

/**-----------------------------------------------------------
  \fn          uint32_t ADC_GetResolution (void)
  \brief       Get resolution of Analog-to-Digital Converter
  \returns     Resolution (in bits)
*-----------------------------------------------------------*/
uint32_t ADC_GetResolution (void)
{
  return ADC_RESOLUTION;
}

/*-----------------------------------------------------------*/


