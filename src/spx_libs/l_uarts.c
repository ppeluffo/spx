/*
 * uarts_sp5K.c
 *
 *  Created on: 15 de nov. de 2016
 *      Author: pablo
 */


#include <l_uarts.h>

#include "frtos-io.h"
//------------------------------------------------------------------------------------
void CMD_write( const void *pvBuffer, const size_t xBytes )
{
	// En el SP6K el USB y BT operan juntos como I/O de la tarea de comando
	// Para simplificar la escritura usamos esta funcion de modo que en el programa
	// no tenemos que escribir en ambos handles.

//uint16_t ticks_to_flush_queue;

	// SI la terminal esta desconectada salgo.
	if ( IO_read_TERMCTL_PIN() == 1 )
		return;

//	USB_write( pvBuffer, xBytes );
	frtos_ioctl (fdUSB,ioctlOBTAIN_BUS_SEMPH, NULL );
	frtos_write( fdUSB, pvBuffer, xBytes );
	frtos_ioctl (fdUSB,ioctlRELEASE_BUS_SEMPH, NULL);

	//	ticks_to_flush_queue  = ( UART_BT_TXBUFFER_LEN * 10 * configTICK_RATE_HZ / 115200 )  + 1;
	//vTaskDelay( ( TickType_t)( ticks_to_flush_queue ) );
//	vTaskDelay( ( TickType_t)( 20 / portTICK_RATE_MS ) );


}
//------------------------------------------------------------------------------------
size_t CMD_read(  void *pvBuffer, const size_t xBytes )
{

	// Como el USB y BT operan en paralelo para el modo comando, los caracteres pueden entrar
	// por cualquiera de los handles.
	// Lee caracteres de ambas FIFO de recepcion de la USB y BT
	// No considera el caso que los handles sean QUEUES !!!!

size_t xBytesReceived = 0U;
portTickType xTicksToWait;
xTimeOutType xTimeOut;
UART_device_control_t *pUartUSB;

	pUartUSB = pdUART_USB.phDevice;

	xTicksToWait = pdUART_USB.xBlockTime;
	xTicksToWait = 5;
	vTaskSetTimeOutState( &xTimeOut );

	/* Are there any more bytes to be received? */
	while( xBytesReceived < xBytes )
	{
		/* Receive the next character. */
		// Los FIFO no tienen timeout, retornan enseguida
		if(  ringBufferPop( pUartUSB->rxStruct, &((char *)pvBuffer)[ xBytesReceived ] ) == pdPASS )
		{
			xBytesReceived++;
		} else {
			// Espero xTicksToWait antes de volver a chequear
			vTaskDelay( ( TickType_t)( xTicksToWait ) );
//			uxSwitchBuffers(pUartUSB->rxStruct);
//			uxSwitchBuffers(pUartBT->rxStruct);
		}

		/* Time out has expired ? */
		if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
		{
			break;
		}
	}

	return xBytesReceived;

}
//------------------------------------------------------------------------------------
void CMD_writeChar (unsigned char c)
{
	// Funcion intermedia necesaria por cmdline para escribir de a 1 caracter en consola
	// El tema es que el prototipo de funcion que requiere cmdlineSetOutputFunc no se ajusta
	// al de FreeRTOS_UART_write, por lo que defino esta funcion intermedia.

char cChar;

	cChar = c;
	CMD_write( &cChar, sizeof(char));
}
//------------------------------------------------------------------------------------
