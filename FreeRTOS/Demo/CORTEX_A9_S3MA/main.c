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

/* Standard includes. */
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "port.h"

/* Demo Includes. */
#include "integer.h"
#include "PollQ.h"
#include "semtest.h"
#include "BlockQ.h"
#include "countsem.h"
#include "dynamic.h"
#include "flop.h"
#include "GenQTest.h"
#include "QPeek.h"
#include "recmutex.h"
#include "serial.h"
#include "comtest2.h"
/*----------------------------------------------------------------------------*/

/* Delay between cycles of the 'check' task. */
#define mainCHECK_DELAY						( ( portTickType ) 5000 / portTICK_RATE_MS )

/* Demo task priorities. */
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainGEN_Q_PRIORITY			( tskIDLE_PRIORITY )

/* Misc. */
#define mainQUEUE_SIZE				( 3 )
#define mainNO_DELAY				( ( portTickType ) 0 )

#define mainPRINT_PORT				( configUART_PORT )
#ifdef configPALLADIUM
/* 907 KHz */
//# define mainPRINT_BAUDRATE			2034437
/* 1.5MHz */
//# define mainPRINT_BAUDRATE			1209448  /* 1524KHz current baudrate*/
/* 1.7MHz */
//# define mainPRINT_BAUDRATE			2123502  /* 1736KHz current baudrate*/
/* 1.9MHz */
# define mainPRINT_BAUDRATE			1887557 /* 1953KHz current baudrate */
#else
# define mainPRINT_BAUDRATE			115200
#endif



/*----------------------------------------------------------------------------*/

/* Periodically checks to see whether the demo tasks are still running. */
static void vCheckTask( void *pvParameters );
/*----------------------------------------------------------------------------*/

/*
 * Configure the clocks, GPIO and other peripherals as required by the demo.
 */
static void prvSetupHardware( void );
/*----------------------------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName );

int main( void )
{
	/* Initialise the Hardware. */
	prvSetupHardware();

	/* Start the standard demo tasks. */
	vStartIntegerMathTasks( tskIDLE_PRIORITY );
	vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
	vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
	vStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );
	vStartCountingSemaphoreTasks();
	vStartDynamicPriorityTasks();
	vStartMathTasks( tskIDLE_PRIORITY );
	vStartGenericQueueTasks( mainGEN_Q_PRIORITY );
	vStartQueuePeekTasks();
	vStartRecursiveMutexTasks();

	/* Start the tasks defined within the file. */
	xTaskCreate( vCheckTask, (const char *)"Check", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Should never reach here. */
	vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)"Should never reach here!\r\n", 26 );

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}
/*----------------------------------------------------------------------------*/

void vApplicationTickHook( void )
{

}
/*----------------------------------------------------------------------------*/

void vApplicationIdleHook( void )
{
signed char cChar;
	if ( pdTRUE == xSerialGetChar( (xComPortHandle)mainPRINT_PORT, &cChar, 0UL ) )
	{
		(void)xSerialPutChar( (xComPortHandle)mainPRINT_PORT, cChar, 0UL );
	}
}
/*----------------------------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	( void ) pxTask;
	( void ) pcTaskName;
	 printf("StackOverflowHook\n");
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	identify which task has overflowed its stack. */
	for( ;; );
}



static void vCheckTask( void *pvParameters )
{
portBASE_TYPE xErrorOccurred = pdFALSE;
portTickType xLastExecutionTime, xFailureTime = 0;
int i;
signed char cBuffer[64];

	/* First Task Started. */
	vSerialPutString((xComPortHandle)mainPRINT_PORT,(const signed char * const)"FTS\r\n", 5 );

	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
	works correctly. */
	xLastExecutionTime = xTaskGetTickCount();

	for( ;; )
	{
		/* Perform this check every mainCHECK_DELAY milliseconds. */
		vTaskDelayUntil( &xLastExecutionTime, mainCHECK_DELAY );

		/* Has an error been found in any task? */

		if( xAreIntegerMathsTaskStillRunning() != pdTRUE )
		{
			xErrorOccurred = pdTRUE;
			vSerialPutString( (xComPortHandle)mainPRINT_PORT, (const signed char * const)"Fail: Integer Maths\r\n", strlen( "Fail: Integer Maths\r\n" ) );
		}

		if( xArePollingQueuesStillRunning() != pdTRUE )
		{
			xErrorOccurred = pdTRUE;
			vSerialPutString( (xComPortHandle)mainPRINT_PORT, (const signed char * const)"Fail: Polling Queues\r\n", strlen( "Fail: Polling Queues\r\n" ) );
		}

		if( xAreSemaphoreTasksStillRunning() != pdTRUE )
		{
			xErrorOccurred = pdTRUE;
			vSerialPutString( (xComPortHandle)mainPRINT_PORT, (const signed char * const)"Fail: Sem Tasks\r\n", strlen( "Fail: Sem Tasks\r\n" ) );
		}

		if( xAreBlockingQueuesStillRunning() != pdTRUE )
		{
			xErrorOccurred = pdTRUE;
			vSerialPutString( (xComPortHandle)mainPRINT_PORT, (const signed char * const)"Fail: BlockQ\r\n", strlen( "Fail: BlockQ\r\n" ) );
		}

		if( xAreCountingSemaphoreTasksStillRunning() != pdTRUE )
		{
			xErrorOccurred = pdTRUE;
			vSerialPutString( (xComPortHandle)mainPRINT_PORT, (const signed char * const)"Fail: SemCount\r\n", strlen( "Fail: SemCount\r\n" ) );
		}

		if( xAreDynamicPriorityTasksStillRunning() != pdTRUE )
		{
			xErrorOccurred = pdTRUE;
			vSerialPutString( (xComPortHandle)mainPRINT_PORT, (const signed char * const)"Fail: DynamicPrio\r\n", strlen( "Fail: DynamicPrio\r\n" ) );
		}

		if( xAreMathsTaskStillRunning() != pdTRUE )
		{
			xErrorOccurred = pdTRUE;
			vSerialPutString( (xComPortHandle)mainPRINT_PORT, (const signed char * const)"Fail: FloatMaths\r\n", strlen( "Fail: FloatMaths\r\n" ) );
		}

		if( xAreGenericQueueTasksStillRunning() != pdTRUE )
		{
			xErrorOccurred = pdTRUE;
			vSerialPutString( (xComPortHandle)mainPRINT_PORT, (const signed char * const)"Fail: GenQ\r\n", strlen( "Fail: GenQ\r\n" ) );
		}

		if( xAreQueuePeekTasksStillRunning() != pdTRUE )
		{
			xErrorOccurred = pdTRUE;
			vSerialPutString( (xComPortHandle)mainPRINT_PORT, (const signed char * const)"Fail: PeekQ\r\n", strlen( "Fail: PeekQ\r\n" ) );
		}

		if( xAreRecursiveMutexTasksStillRunning() != pdTRUE )
		{
			xErrorOccurred = pdTRUE;
			vSerialPutString( (xComPortHandle)mainPRINT_PORT, (const signed char * const)"Fail: RecMutex\r\n", strlen( "Fail: RecMutex\r\n" ) );
		}


		/* Send either a pass or fail message.  If an error is found it is
		never cleared again.  We do not write directly to the LCD, but instead
		queue a message for display by the print task. */
		if( xErrorOccurred == pdTRUE )
		{
			if ( 0 == xFailureTime )
			{
				xFailureTime = xLastExecutionTime;
			}
			sprintf( (char *)cBuffer, "Fail: %lu\r\n", (unsigned long)(xFailureTime / 1000) );
		}
		else
		{
			i=sprintf( (char *)cBuffer, "Pass: %lu\r\n", (unsigned long)(xLastExecutionTime / 1000) );

			sprintf( (char *)&cBuffer[i], "From Core: %ld\r\n", (long)portCORE_ID());
		
		}
		vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)cBuffer, strlen((char *)cBuffer) );
	}
}
/*-----------------------------------------------------------*/

static unsigned long prvReadP15_C1_C0_0( void )
{
volatile unsigned long ulReturn = 0UL;
	/* Read Configuration Register C15, c0, 0. */
	__asm volatile(
			" mrc p15, 0, %[retval], c1, c0, 0 			\n"
			: [retval] "=r" (ulReturn) : :
			);
	return ulReturn;
}

/*----------------------------------------------------------------------------*/

static unsigned long prvReadP15_C1_C0_1( void )
{
volatile unsigned long ulReturn = 0UL;
	/* Read Configuration Register C15, c0, 0. */
	__asm volatile(
			" mrc p15, 0, %[retval], c1, c0, 1 			\n"
			: [retval] "=r" (ulReturn) : :
			);
	return ulReturn;
}
/*----------------------------------------------------------------------------*/

static unsigned long prvReadP15_C1_C0_2( void )
{
volatile unsigned long ulReturn = 0UL;
	/* Read Configuration Register C15, c0, 0. */
	__asm volatile(
			" mrc p15, 0, %[retval], c1, c0, 2 			\n"
			: [retval] "=r" (ulReturn) : :
			);
	return ulReturn;
}
/*----------------------------------------------------------------------------*/

static unsigned long prvReadP15_C1_C1_0( void )
{
volatile unsigned long ulReturn = 0UL;
	/* Read Configuration Register C15, c0, 0. */
	__asm volatile(
			" mrc p15, 0, %[retval], c1, c1, 0 			\n"
			: [retval] "=r" (ulReturn) : :
			);
	return ulReturn;
}

/*----------------------------------------------------------------------------*/

static unsigned long prvReadP15_C1_C1_1( void )
{
volatile unsigned long ulReturn = 0UL;
	/* Read Configuration Register C15, c0, 0. */
	__asm volatile(
			" mrc p15, 0, %[retval], c1, c1, 1 			\n"
			: [retval] "=r" (ulReturn) : :
			);
	return ulReturn;
}
/*----------------------------------------------------------------------------*/

static unsigned long prvReadP15_C1_C1_2( void )
{
volatile unsigned long ulReturn = 0UL;
	/* Read Configuration Register C15, c0, 0. */
	__asm volatile(
			" mrc p15, 0, %[retval], c1, c1, 2 			\n"
			: [retval] "=r" (ulReturn) : :
			);
	return ulReturn;
}

/*----------------------------------------------------------------------------*/

static unsigned long prvReadP15_C12_C0_1( void )
{
volatile unsigned long ulReturn = 0UL;
	/* Read Configuration Register C15, c0, 0. */
	__asm volatile(
			" mrc p15, 0, %[retval], c12, c0, 1 		\n"
			: [retval] "=r" (ulReturn) : :
			);
	return ulReturn;
}

/*----------------------------------------------------------------------------*/

static void prvSetupHardware( void )
{
unsigned long ulVector = 0UL;
unsigned long ulValue = 0UL;
char cAddress[32];

	portDISABLE_INTERRUPTS();

	/* Install the Spurious Interrupt Handler to help catch interrupts. */
extern void vPortUnknownInterruptHandler( void *pvParameter );
extern void vPortInstallInterruptHandler( void (*vHandler)(void *), void *pvParameter, unsigned long ulVector, unsigned char ucEdgeTriggered, unsigned char ucPriority, unsigned char ucProcessorTargets );
	for ( ulVector = 0; ulVector < portMAX_VECTORS; ulVector++ )
		vPortInstallInterruptHandler( vPortUnknownInterruptHandler, (void *)ulVector, ulVector, pdTRUE, configMAX_SYSCALL_INTERRUPT_PRIORITY, 1 );

extern void vUARTInitialise(unsigned long ulUARTPeripheral, unsigned long ulBaud, unsigned long ulQueueSize );
	vUARTInitialise( mainPRINT_PORT, mainPRINT_BAUDRATE, 64 );

	ulValue = portCORE_ID();
	sprintf( cAddress, "Core: %ld\r\n", ulValue );
	vSerialPutString((xComPortHandle)configUART_PORT,(const signed char * const)cAddress, strlen(cAddress) );

	ulValue = (unsigned long)main;
	sprintf( cAddress, "Main: 0x%08lX\r\n", ulValue );
	vSerialPutString((xComPortHandle)configUART_PORT,(const signed char * const)cAddress, strlen(cAddress) );

	/* Read the Configuration Registers to find the vital information. */
	ulValue = prvReadP15_C1_C0_0();
	sprintf( cAddress, "SCTLR: 0x%08lX\r\n", ulValue );
	vSerialPutString((xComPortHandle)configUART_PORT,(const signed char * const)cAddress, strlen(cAddress) );

	/* Read the Configuration Registers to find the vital information. */
	ulValue = prvReadP15_C1_C0_1();
	sprintf( cAddress, "ACTLR: 0x%08lX\r\n", ulValue );
	vSerialPutString((xComPortHandle)configUART_PORT,(const signed char * const)cAddress, strlen(cAddress) );

	/* Read the Configuration Registers to find the vital information. */
	ulValue = prvReadP15_C1_C0_2();
	sprintf( cAddress, "CPACR: 0x%08lX\r\n", ulValue );
	vSerialPutString((xComPortHandle)configUART_PORT,(const signed char * const)cAddress, strlen(cAddress) );

	/* Read the Configuration Registers to find the vital information. */
	ulValue = prvReadP15_C1_C1_0();
	sprintf( cAddress, "SCR: 0x%08lX\r\n", ulValue );
	vSerialPutString((xComPortHandle)configUART_PORT,(const signed char * const)cAddress, strlen(cAddress) );

	/* Read the Configuration Registers to find the vital information. */
	ulValue = prvReadP15_C1_C1_1();
	sprintf( cAddress, "SDER: 0x%08lX\r\n", ulValue );
	vSerialPutString((xComPortHandle)configUART_PORT,(const signed char * const)cAddress, strlen(cAddress) );

	/* Read the Configuration Registers to find the vital information. */
	ulValue = prvReadP15_C1_C1_2();
	sprintf( cAddress, "NSACR: 0x%08lX\r\n", ulValue );
	vSerialPutString((xComPortHandle)configUART_PORT,(const signed char * const)cAddress, strlen(cAddress) );

	/* Read the Configuration Registers to find the vital information. */
	ulValue = prvReadP15_C12_C0_1();
	sprintf( cAddress, "MVBAR: 0x%08lX\r\n", ulValue );
	vSerialPutString((xComPortHandle)configUART_PORT,(const signed char * const)cAddress, strlen(cAddress) );

	/* Perform any other peripheral configuration. */
}
/*----------------------------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	__asm volatile (" smc #0 ");
}
/*----------------------------------------------------------------------------*/

void vAssertCalled( const char *file, unsigned long line )
{
	printf("Assertion failed at %s, line %lu\n\r",file,line);
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

