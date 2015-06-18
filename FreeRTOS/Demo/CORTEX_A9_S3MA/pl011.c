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

/* UART Driver for the PL011 Peripheral. */

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
/*----------------------------------------------------------------------------*/

#define UART_USE_INTERRUPT	1

#define UART0_BASE			( UART0_APB_ABSOLUTE_BASE )		/* S3MA UART0. */
#define UART1_BASE			( UART1_APB_ABSOLUTE_BASE )		/* S3MA UART1. */
#define UART_DEVICE_TYPE	0

#define UARTDR(x)			( (volatile unsigned char *)	( (x) + 0x0000UL ) )	/* Data Register */
#define UARTRSR_UARTECR(x)	( (volatile unsigned char *)	( (x) + 0x0004UL ) )	/* Receive Status Register / Error Clear Register */
#define UARTFR(x)			( (volatile unsigned short *)( (x) + 0x0018UL ) )	/* Flag Register */
#define UARTILPR(x)			( (volatile unsigned char *)	( (x) + 0x0020UL ) )	/* IrDA Low-Power Counter Register */
#define UARTIBRD(x)			( (volatile unsigned short *)( (x) + 0x0024UL ) )	/* Integer Baud Rate Register */
#define UARTFBRD(x)			( (volatile unsigned char *)	( (x) + 0x0028UL ) )	/* Fractional Baud Rate Register */
#define UARTLCR_H(x)		( (volatile unsigned char *)	( (x) + 0x002CUL ) )	/* Line Control Register */
#define UARTCR(x)			( (volatile unsigned short *)( (x) + 0x0030UL ) )	/* Control Register */
#define UARTIFLS(x)			( (volatile unsigned char *)	( (x) + 0x0034UL ) )	/* Interrupt FIFO Level Select Register */
#define UARTIMSC(x)			( (volatile unsigned short *)( (x) + 0x0038UL ) )	/* Interrupt Mask Set/Clear Register */
#define UARTRIS(x)			( (volatile unsigned short *)( (x) + 0x003CUL ) )	/* Raw Interrupt Status Register */
#define UARTMIS(x)			( (volatile unsigned short *)( (x) + 0x0040UL ) )	/* Masked Interrupt Status Register */
#define UARTICR(x)			( (volatile unsigned short *)( (x) + 0x0044UL ) )	/* Interrupt Clear Register */
#define UARTDMACR(x)		( (volatile unsigned char *)	( (x) + 0x0048UL ) )	/* DMA Control Register */
#define UARTPeriphID0(x)	( (volatile unsigned char *)	( (x) + 0x0FE0UL ) )	/* UARTPeriphID0 Register */
#define UARTPeriphID1(x)	( (volatile unsigned char *)	( (x) + 0x0FE4UL ) )	/* UARTPeriphID1 Register */
#define UARTPeriphID2(x)	( (volatile unsigned char *)	( (x) + 0x0FE8UL ) )	/* UARTPeriphID2 Register */
#define UARTPeriphID3(x)	( (volatile unsigned char *)	( (x) + 0x0FECUL ) )	/* UARTPeriphID3 Register */
#define UARTPCellID0(x)		( (volatile unsigned char *)	( (x) + 0x0FF0UL ) )	/* UARTPCellID0 Register */
#define UARTPCellID1(x)		( (volatile unsigned char *)	( (x) + 0x0FF4UL ) )	/* UARTPCellID1 Register */
#define UARTPCellID2(x)		( (volatile unsigned char *)	( (x) + 0x0FF8UL ) )	/* UARTPCellID2 Register */
#define UARTPCellID3(x)		( (volatile unsigned char *)	( (x) + 0x0FFCUL ) )	/* UARTPCellID3 Register */

#define UART_INT_STATUS_RT	( 1 << 6 )
#define UART_INT_STATUS_TX	( 1 << 5 )
#define UART_INT_STATUS_RX	( 1 << 4 )

#define UART_FLAG_TXFE		( 1 << 7 )
#define UART_FLAG_TXFF		( 1 << 5 )
#define UART_FLAG_RXFE		( 1 << 4 )

#define UART_CLK_HZ				( configCLOCK_HZ )

#define UART_FIFO_SIZE_BYTES	( 32UL )
#define UART0_VECTOR_ID			( 100 )
#define UART1_VECTOR_ID			( 101 )

#define TX_QUEUE				( 0 )
#define RX_QUEUE				( 1 )
/*----------------------------------------------------------------------------*/

#if UART_USE_INTERRUPT

static xQueueHandle xUartQueues[3][2] = { { NULL, NULL } };
/*----------------------------------------------------------------------------*/

void vUARTInterruptHandler( void *pvBaseAddress )
{
unsigned long ulBase = (unsigned long)pvBaseAddress;
unsigned short usStatus = 0;
signed char cTransmitChar = 0;
signed char cReceiveChar = 0;
portBASE_TYPE xTaskWoken = pdFALSE;
xQueueHandle xTxQueue = NULL;
xQueueHandle xRxQueue = NULL;
unsigned long ulUART = 0;

	/* Select the UART Queues. */
	switch ( ulBase )
	{
	case UART0_BASE:
		ulUART = 0;
		break;
	case UART1_BASE:
		ulUART = 1;
		break;
	}

	xTxQueue = xUartQueues[ulUART][TX_QUEUE];
	xRxQueue = xUartQueues[ulUART][RX_QUEUE];

	/* Figure out the reason for the interrupt. */
	usStatus = *UARTMIS(ulBase);

	if ( usStatus & UART_INT_STATUS_TX )
	{
		/* Buffer is almost empty, try to refill. */
		while ( *UARTFR(ulBase) & UART_FLAG_TXFF )
		{
			if ( pdTRUE == xQueueReceiveFromISR( xTxQueue, &cTransmitChar, &xTaskWoken ) )
			{
				*UARTDR(ulBase) = cTransmitChar;
			}
			else
			{
				/* Run out of characters to send. */
				break;
			}
		}
	}

	if ( usStatus & (UART_INT_STATUS_RX | UART_INT_STATUS_RT) )
	{
		/* Receive Buffer is almost full. */
		while ( !( *UARTFR(ulBase) & UART_FLAG_RXFE ) )
		{
			cReceiveChar = *UARTDR(ulBase);
			if ( pdTRUE != xQueueSendFromISR( xRxQueue, &cReceiveChar, &xTaskWoken ) )
			{
				/* Receive Queue is Full. */
				/* Not good as we are going to lose this character. */
				break;
			}
		}
	}

	/* Here we should deal with any errors. */

	/* Acknownledge the interrupt. */
	*UARTICR(ulBase) = usStatus;

	/* Finally, switch task if necessary. */
	portEND_SWITCHING_ISR(xTaskWoken);
}
/*----------------------------------------------------------------------------*/

#endif /* UART_USE_INTERRUPT */

void vUARTInitialise(unsigned long ulUARTPeripheral, unsigned long ulBaud, unsigned long ulQueueSize )
{
extern void vPortInstallInterruptHandler( void (*vHandler)(void *), void *pvParameter, unsigned long ulVector, unsigned char ucEdgeTriggered, unsigned char ucPriority, unsigned char ucProcessorTargets );
unsigned long ulBase = 0;
#if UART_USE_INTERRUPT
unsigned long ulVectorID = 0;
#endif
unsigned int temp;
unsigned int divider;
unsigned int remainder;
unsigned int fraction;

	switch ( ulUARTPeripheral )
	{
	case 0:
		ulBase = UART0_BASE;
		#if UART_USE_INTERRUPT
		ulVectorID = UART0_VECTOR_ID;
		/* Create the Queues to Handle the bytes. */
		xUartQueues[0][TX_QUEUE] = xQueueCreate( ulQueueSize, sizeof( char ) );
		xUartQueues[0][RX_QUEUE] = xQueueCreate( ulQueueSize, sizeof( char ) );
		#endif
		break;
	case 1:
		ulBase = UART1_BASE;
		#if UART_USE_INTERRUPT
		ulVectorID = UART1_VECTOR_ID;
		/* Create the Queues to Handle the bytes. */
		xUartQueues[1][TX_QUEUE] = xQueueCreate( ulQueueSize, sizeof( char ) );
		xUartQueues[1][RX_QUEUE] = xQueueCreate( ulQueueSize, sizeof( char ) );
		#endif
		break;
	}

	/*
	* Set baud rate
	*
	* IBRD = UART_CLK / (16 * BAUD_RATE)
	* FBRD = RND((64 * MOD(UART_CLK,(16 * BAUD_RATE)))
	*		/ (16 * BAUD_RATE))
	*/
	temp = 16 * ulBaud;
	divider = UART_CLK_HZ / temp;
	remainder = UART_CLK_HZ % temp;
	temp = (8 * remainder) / ulBaud;
	fraction = (temp >> 1) + (temp & 1);

	/* First Disable the Peripheral. */
	*UARTCR(ulBase) = 0UL;

	/* Configure the Peripheral. */
	*UARTIBRD(ulBase) = divider;
	*UARTFBRD(ulBase) = fraction;
	*UARTLCR_H(ulBase) = ( 3 << 5 ) | ( 1 << 4 );

	/* Configure the Interrupts. */
	*UARTIFLS(ulBase) = ( 0 << 3 ) | ( 0 << 0 );
	*UARTICR(ulBase) = 0xFFFF;	/* Clear all Iterrupts. */

#if UART_USE_INTERRUPT
	*UARTIMSC(ulBase) = UART_INT_STATUS_TX | UART_INT_STATUS_RX | UART_INT_STATUS_RT;
	vPortInstallInterruptHandler( vUARTInterruptHandler, (void *)ulBase, ulVectorID, pdFALSE, configMAX_SYSCALL_INTERRUPT_PRIORITY, 1 );
#endif /* UART_USE_INTERRUPT */

	/* Finally enable the peripheral. */
	*UARTCR(ulBase) = ( 1 << 9 ) | ( 1 << 8 ) | /* Loopback Enable ( 1 << 7 ) | */ ( 1 << 0 );
}
/*----------------------------------------------------------------------------*/

portBASE_TYPE xUARTSendCharacter( unsigned long ulUARTPeripheral, signed char cChar, portTickType xDelay )
{
unsigned long ulBase = 0;
portBASE_TYPE xReturn = pdFALSE;
#if UART_USE_INTERRUPT
xQueueHandle xTxQueue = NULL;
unsigned char ucStatus = 0;
#endif	/* UART_USE_INTERRUPT */

	switch ( ulUARTPeripheral )
	{
	case 0:
		ulBase = UART0_BASE;
		#if UART_USE_INTERRUPT
		xTxQueue = xUartQueues[0][TX_QUEUE];
		#endif
		break;
	case 1:
		ulBase = UART1_BASE;
		#if UART_USE_INTERRUPT
		xTxQueue = xUartQueues[1][TX_QUEUE];
		#endif
		break;
	}

	if ( 0 != ulBase )
	{
#if UART_USE_INTERRUPT
		xReturn = xQueueSend( xTxQueue, &cChar, xDelay );
		taskENTER_CRITICAL();
			ucStatus = *UARTFR(ulBase);
		taskEXIT_CRITICAL();

		/* Put a byte directly in the fifo if there is room */
		if ( (ucStatus & UART_FLAG_TXFF) == 0 )
		{
			/* Need to kick of the Tx. */
			(void)xQueueReceive( xTxQueue, &cChar, 0 );
			*UARTDR(ulBase) = cChar;
			xReturn = pdTRUE;
		}
#else
		while ( !(*UARTFR(ulBase) & UART_FLAG_TXFE) );
		*UARTDR(ulBase) = cChar;
		xReturn = pdTRUE;
#endif	/* UART_USE_INTERRUPT */
	}
	return xReturn;
}
/*----------------------------------------------------------------------------*/

portBASE_TYPE xUARTReceiveCharacter( unsigned long ulUARTPeripheral, signed char *pcChar, portTickType xDelay )
{
unsigned long ulBase = 0;
portBASE_TYPE xReturn = pdFALSE;
#if UART_USE_INTERRUPT
xQueueHandle xRxQueue = NULL;
#endif

	switch ( ulUARTPeripheral )
	{
	case 0:
		ulBase = UART0_BASE;
		#if UART_USE_INTERRUPT
		xRxQueue = xUartQueues[0][RX_QUEUE];
		#endif
		break;
	case 1:
		ulBase = UART1_BASE;
		#if UART_USE_INTERRUPT
		xRxQueue = xUartQueues[1][RX_QUEUE];
		#endif
		break;
	}

	if ( 0 != ulBase )
	{
#if UART_USE_INTERRUPT
		xReturn = xQueueReceive( xRxQueue, pcChar, xDelay );
#else
		if ( ( *UARTFR(ulBase) & UART_FLAG_RXFE ) == 0 )
		{
			*pcChar = *UARTDR(ulBase);
			xReturn = pdTRUE;
		}
#endif /* UART_USE_INTERRUPT */
	}

	return xReturn;
}
/*----------------------------------------------------------------------------*/


