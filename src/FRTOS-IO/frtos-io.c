/*
 * frtos-io.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 */

#include "frtos-io.h"

//------------------------------------------------------------------------------------
int frtos_open( file_descriptor_t fd, uint32_t flags)
{
	// Funcion general para abrir el puerto que invoca a una mas
	// especializada para c/periferico.
	// Es la que invoca la aplicacion.
	// Retorna -1 en error o un nro positivo ( fd )

int8_t retS = -1;

	switch(fd) {
	case fdUSB:
		retS = frtos_uart_open( &xComUSB, flags );
		break;
	default:
		break;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
int frtos_ioctl( file_descriptor_t fd, uint32_t ulRequest, void *pvValue )
{
int8_t retS = -1;

	switch(fd) {
	case fdUSB:
		retS = frtos_uart_ioctl( &xComUSB, ulRequest, pvValue );
		break;
	default:
		break;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
int frtos_write( file_descriptor_t fd ,const char *pvBuffer, const size_t xBytes )
{
int8_t retS = -1;

	switch(fd) {
	case fdUSB:
		retS = frtos_uart_write( &xComUSB, pvBuffer, xBytes );
		break;
	default:
		break;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
int frtos_uart_open( periferico_serial_port_t *xCom, uint32_t flags)
{

	xCom->fd = fdUSB;
	xCom->xBusSemaphore = xSemaphoreCreateMutex();
	xCom->xBlockTime = (10 / portTICK_RATE_MS );
	// Inicializo la uart del usb (iUART_USB) y la asocio al periferico
	xCom->uart = drv_uart_init( iUART_USB, flags );

	return(xCom->fd);

}
//------------------------------------------------------------------------------------
int frtos_uart_write( periferico_serial_port_t *xCom, const char *pvBuffer, const size_t xBytes )
{
	// Esta funcion debe poner los caracteres apuntados en pvBuffer en la cola de trasmision del
	// puerto serial apuntado por xCom
	// Actua como si fuese rprintfStr.
	// Debe tomar el semaforo antes de trasmitir. Los semaforos los manejamos en la capa FreeRTOS
	// y no en la de los drivers.

char cChar;
char *p;
size_t bytes2tx;
size_t wBytes = 0;

	// Controlo no hacer overflow en la cola de trasmision
	bytes2tx = xBytes;

	// Espero el semaforo en forma persistente.
	// Lo elimino y debo pedirlo con frtos_ioctl !!!
//	while ( xSemaphoreTake(xCom->xBusSemaphore, ( TickType_t ) 1 ) != pdTRUE )
//		vTaskDelay( ( TickType_t)( 1 ) );

	// Trasmito.
	// Espero que los buffers esten vacios. ( La uart se va limpiando al trasmitir )
	while  ( rBufferGetCount( &xCom->uart->TXringBuffer ) > 0 )
		vTaskDelay( ( TickType_t)( 1 ) );

	// Cargo el buffer en la cola de trasmision.
	p = (char *)pvBuffer;
	while (*p && (bytes2tx-- > 0) ) {

		// Voy cargando la cola de a uno.
		cChar = *p;
		rBufferPoke( &xCom->uart->TXringBuffer, &cChar  );
		//rBufferPoke( &uart_usb.TXringBuffer, &cChar  );
		p++;
		wBytes++;	// Cuento los bytes que voy trasmitiendo

		// Si la cola esta llena, empiezo a trasmitir y espero que se vacie.
		if (  rBufferReachHighWaterMark( &xCom->uart->TXringBuffer ) ) {
			// Habilito a trasmitir para que se vacie
			drv_uart_interruptOn( xCom->uart->uart_id );
			// Y espero que se haga mas lugar.
			while ( ! rBufferReachLowWaterMark( &xCom->uart->TXringBuffer ) )
				vTaskDelay( ( TickType_t)( 1 ) );
		}
	}

	// Luego inicio la trasmision invocando la interrupcion.
	drv_uart_interruptOn( xCom->uart->uart_id );

	// Espero que trasmita todo
	while  ( rBufferGetCount( &xCom->uart->TXringBuffer ) > 0 )
		vTaskDelay( ( TickType_t)( 1 ) );

//	xSemaphoreGive( xCom->xBusSemaphore );

	return (wBytes);
}
//------------------------------------------------------------------------------------
int frtos_uart_ioctl( periferico_serial_port_t *xCom, uint32_t ulRequest, void *pvValue )
{

int xReturn = 0;

	switch( ulRequest )
	{

		case ioctlOBTAIN_BUS_SEMPH:
			// Espero el semaforo en forma persistente.
			while ( xSemaphoreTake(xCom->xBusSemaphore, ( TickType_t ) 1 ) != pdTRUE )
				taskYIELD();
			break;
		case ioctlRELEASE_BUS_SEMPH:
			xSemaphoreGive( xCom->xBusSemaphore );
			break;
		case ioctlSET_TIMEOUT:
			xCom->xBlockTime = *((uint8_t *)pvValue);
			break;
		case ioctl_UART_CLEAR_RX_BUFFER:
			rBufferFlush(&xCom->uart->RXringBuffer);
			break;
		case ioctl_UART_CLEAR_TX_BUFFER:
			rBufferFlush(&xCom->uart->RXringBuffer);
			break;
		default :
			xReturn = -1;
			break;
	}

	return xReturn;

}
//------------------------------------------------------------------------------------
