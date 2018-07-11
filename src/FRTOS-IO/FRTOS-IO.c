/*
 * FRTOS-IO.c
 *
 *  Created on: 2/10/2015
 *      Author: pablo
 *
 */

#include <FRTOS-IO/FRTOS-IO.h>

#ifdef DEBUG_I2C
#include "../spx.h"
#endif

//------------------------------------------------------------------------------------
// FUNCIONES Privadas
//------------------------------------------------------------------------------------
static void pv_enqueue( UART_device_control_t *pUart, char *pC );
static bool pv_queueReachHighWaterMark( UART_device_control_t *pUart);
static bool pv_queueReachLowWaterMark( UART_device_control_t *pUart);

//------------------------------------------------------------------------------------
// FUNCIONES GENERALES FreeRTOS ( son las que usa la aplicacion )
//------------------------------------------------------------------------------------
// FUNCIONES DE UART PROVISTAS AL FREERTOS
//------------------------------------------------------------------------------------
size_t FreeRTOS_UART_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes )
{
	// Esta funcion debe poner los caracteres apuntados en pvBuffer en la cola de trasmision.
	// Actua como si fuese rprintfStr.
	// Debe tomar el semaforo antes de trasmitir. Los semaforos los manejamos en la capa FreeRTOS
	// y no en la de los drivers.

char cChar;
char *p;
size_t bytes2tx;
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
UART_device_control_t *pUart;
size_t wBytes = 0;

	pUart = pxPeripheralControl->phDevice;
	// Controlo no hacer overflow en la cola de trasmision
	bytes2tx = xBytes;

	// Espero el semaforo en forma persistente.
	while ( xSemaphoreTake(pxPeripheralControl->xBusSemaphore, ( TickType_t ) 1 ) != pdTRUE )
		//taskYIELD();
		vTaskDelay( ( TickType_t)( 1 ) );

	// Trasmito.
	// Espero que los buffers esten vacios. ( La uart se va limpiando al trasmitir )
	while  ( ringBufferGetCount( pUart->txStruct ) > 0 )
	//	taskYIELD();
		vTaskDelay( ( TickType_t)( 1 ) );

	// Cargo el buffer en la cola de trasmision.
	p = (char *)pvBuffer;
	while (*p && (bytes2tx-- > 0) ) {

		// Voy cargando la cola de a uno.
		cChar = *p;
		ringBufferPoke( pUart->txStruct, &cChar  );
		p++;
		wBytes++;	// Cuento los bytes que voy trasmitiendo

		// Si la cola esta llena, empiezo a trasmitir y espero que se vacie.
		if (  pv_queueReachHighWaterMark(pUart) ) {
			// Habilito a trasmitir para que se vacie
			vUartInterruptOn(pxPeripheralControl->portId);
			// Y espero que se haga mas lugar.
			while ( ! pv_queueReachLowWaterMark(pUart) )
				//taskYIELD();
				vTaskDelay( ( TickType_t)( 1 ) );
		}
	}

	// Luego inicio la trasmision invocando la interrupcion.
	vUartInterruptOn(pxPeripheralControl->portId);

	// Espero que trasmita todo
	while ( ! pv_queueReachLowWaterMark(pUart) )
		vTaskDelay( ( TickType_t)( 1 ) );

	xSemaphoreGive( pxPeripheralControl->xBusSemaphore );

	//return xBytes;	// Puse todos los caracteres en la cola.
	return (wBytes);

}
//------------------------------------------------------------------------------------
static bool pv_queueReachLowWaterMark( UART_device_control_t *pUart)
{
bool retS = false;

	if ( ringBufferGetCount( pUart->txStruct ) < (int)(0.2 * pUart->txBufferLength ))
		retS = true;
	return(retS);

}
//------------------------------------------------------------------------------------
static bool pv_queueReachHighWaterMark( UART_device_control_t *pUart)
{
bool retS = false;

	if ( ringBufferGetCount( pUart->txStruct ) > (int)(0.8 * pUart->txBufferLength ))
		retS = true;

	return(retS);

}
//------------------------------------------------------------------------------------
static void pv_enqueue( UART_device_control_t *pUart, char *pC )
{
	ringBufferPoke( pUart->txStruct, pC  );

}
//------------------------------------------------------------------------------------
size_t FreeRTOS_UART_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes )
{

	// Lee caracteres de la cola de recepcion y los deja en el buffer.
	// El timeout lo fijo con ioctl.

Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
size_t xBytesReceived = 0U;
portTickType xTicksToWait;
xTimeOutType xTimeOut;
UART_device_control_t *pUart;

	pUart = pxPeripheralControl->phDevice;

	xTicksToWait = pxPeripheralControl->xBlockTime;
	xTicksToWait = 10;
	vTaskSetTimeOutState( &xTimeOut );

	/* Are there any more bytes to be received? */
	while( xBytesReceived < xBytes )
	{

		if ( pUart->rxBufferType == FIFO ) {
			// Los fifo no tienen timeout, retornan enseguida
			if( ringBufferPop( pUart->rxStruct, &((char *)pvBuffer)[ xBytesReceived ] ) == pdPASS ) {
				xBytesReceived++;
			}
	//		else {
	//			uxSwitchBuffers(pUart->rxStruct);
	//		}
		}

		// Espero xTicksToWait antes de volver a chequear
		vTaskDelay( ( TickType_t)( xTicksToWait ) );

		/* Time out has expired ? */
		if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
		{
			break;
		}
	}

	return xBytesReceived;

}
//------------------------------------------------------------------------------------
char *FreeRTOS_UART_getFifoPtr(Peripheral_Control_t *UART)
{
	// Retorna un puntero al comienzo de buffer de la fifo de una UART.
	// Se usa para imprimir dichos buffers
	// Funcion PELIGROSA !!!

UART_device_control_t *uartDevice;
ringBuffer_s *uartFifo;
char *p;

	uartDevice = (UART_device_control_t *) UART->phDevice;
	if ( uartDevice->rxBufferType != FIFO )
		return(NULL);

	uartFifo = (ringBuffer_s *) uartDevice->rxStruct;
	p = (char *)uartFifo->buff;
	return(p);
}
//------------------------------------------------------------------------------------


