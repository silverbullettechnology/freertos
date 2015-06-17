/*
    FreeRTOS V8.2.1 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*-----------------------------------------------------------
 * Simple IO routines to control the LEDs.
 * This file is called ParTest.c for historic reasons.  Originally it stood for
 * PARallel port TEST.
 *-----------------------------------------------------------*/

#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo includes. */
#include "partest.h"

#include "s3ma-regs.h"

#define PINS_PER_BANK	8

enum gpio_direction {
	GPIO_DIRECTION_IN,
	GPIO_DIRECTION_OUT,
};

struct gpio_regs {
	uint8_t gpiodata[0x400];	/* 0x000 ... 0x3fc */
	uint8_t gpiodir;			/* 0x400 */
};

#define DATA_REG_ADDR(gpio)		(1 << (gpio + 2))

/*-----------------------------------------------------------*/
static struct gpio_regs* xGpioGetBase(unsigned gpio, unsigned int* bit)
{
	unsigned int bank;
	struct gpio_regs* regs = NULL;
	bank = gpio / PINS_PER_BANK;

	switch(bank){
	case 0:
		regs = (struct gpio_regs *)GPIO0_APB_ABSOLUTE_BASE;
		break;
	case 1:
		regs = (struct gpio_regs *)GPIO1_APB_ABSOLUTE_BASE;
		break;
	case 2:
		regs = (struct gpio_regs *)GPIO2_APB_ABSOLUTE_BASE;
		break;

	default:
		regs = NULL;
	}

	*bit = 0;

	if(regs){
		*bit = gpio % PINS_PER_BANK;
	}

	return regs;
}

int SetDirectionPin(unsigned pin, enum gpio_direction dir)
{
	uint32_t bit;
	struct gpio_regs *regs = xGpioGetBase(pin, &bit);
	uint32_t val, *addr;

	if(regs){
	   addr = (uint32_t*)&regs->gpiodir;
	   val = *addr;

	   if (dir == GPIO_DIRECTION_OUT){
	   	val |= 1 << bit;
	   }else{
	   	val &= ~(1 << bit);
	   }

	   *addr = val;

	   return 0;
	}
	else
	{
		return -1;
	}
}


int WritePin(unsigned pin, unsigned value)
{
	struct gpio_regs *regs;
	uint32_t bit;

	regs = xGpioGetBase(pin, &bit);

	if(regs){
		uint32_t *addr = (uint32_t*)&regs->gpiodata[DATA_REG_ADDR(bit)];
		*addr = (value << bit);
		return 0;
	}
	else{

		return -1;
	}
}

int ReadPin(unsigned pin)
{
	uint32_t bit;
	struct gpio_regs *regs = xGpioGetBase(pin, &bit);
	uint32_t val, *addr;

	if(regs){
		addr = (uint32_t*)&regs->gpiodata[DATA_REG_ADDR(bit)];
		val = *addr;;

		return !!val;
	}
	else{
		return 0;
	}
}

/*-----------------------------------------------------------*/

#define partstNUM_LEDS			( 1 )
#define partstDIRECTION_OUTPUT	( GPIO_DIRECTION_OUT )
#define partstOUTPUT_ENABLED	( 1 )
#define partstLED_OUTPUT		( 21 )


void vParTestInitialise( void )
{
	/* Enable outputs and set low. */
	SetDirectionPin( partstLED_OUTPUT, partstDIRECTION_OUTPUT );
	WritePin(partstLED_OUTPUT, 0);
}
/*-----------------------------------------------------------*/

void vParTestSetLED( UBaseType_t uxLED, BaseType_t xValue )
{
	( void ) uxLED;
	WritePin(partstLED_OUTPUT, 1);
}
/*-----------------------------------------------------------*/

void vParTestToggleLED( unsigned portBASE_TYPE uxLED )
{
BaseType_t xLEDState;

	( void ) uxLED;

	xLEDState = ReadPin( partstLED_OUTPUT );
	WritePin(partstLED_OUTPUT, !xLEDState);
}



