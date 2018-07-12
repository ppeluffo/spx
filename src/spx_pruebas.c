/*
 * spx_pruebas.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 */


#include "spx.h"
#include "l_uarts.h"
#include "../drivers/drv_uart_spx.h"

// Creamos las estructuras y procedimientos para manejar ringbuffers.
// Lo problamos cargando un ringbuffer desde un string y luego leyendolo e imprimiendolo

// Area de almacenamiento de buffers raw
#define STORAGE_SIZE	32
uint8_t in_storage[STORAGE_SIZE];
uint8_t out_storage[STORAGE_SIZE];

uart_control_t uart;

// Estructuctura de un periferico
typedef struct {
	int	pId;
	SemaphoreHandle_t xBusSemaphore;		//
	uint8_t xBlockTime;						// ticks to block in read operations. Set by ioctl
	uart_control_t uart;					// puntero a una estructura con los miembros adecuados al periferico
} periferico_s;

// Dispositivos a los que voy a poder acceder.
periferico_s pUsb;

char test_buffer[STORAGE_SIZE];

uint16_t pos;

void UartInterruptOn(void);
void UartInterruptOff(void);

void uartInit( uart_control_t *xUart, uint8_t *in_buffer, uint8_t *out_buffer, uint16_t in_size, uint16_t out_size );
void perifericoUartInit( periferico_s *pDev,uint8_t *in_buffer, uint8_t *out_buffer, uint16_t in_size, uint16_t out_size );
void cargarRingBuffer( ringBuffer_s *rB, uint8_t *buffer, uint16_t xBytes );
void volcarRingBuffer( ringBuffer_s *rB, uint8_t *buffer, uint16_t xBytes );
int UART_write( void *pvBuffer, const size_t xBytes );

void uartInit( uart_control_t *xUart, uint8_t *in_buffer, uint8_t *out_buffer, uint16_t in_size, uint16_t out_size )
{
	rBufferCreateStatic( &xUart->RXringBuffer, in_buffer, in_size );
	rBufferCreateStatic( &xUart->TXringBuffer, out_buffer, out_size );
}
//------------------------------------------------------------------------------------
void perifericoUartInit( periferico_s *pDev,uint8_t *in_buffer, uint8_t *out_buffer, uint16_t in_size, uint16_t out_size )
{
	pDev->pId = 1;
	pDev->xBlockTime = 10;
	pDev->xBusSemaphore = xSemaphoreCreateMutex();
	uartInit(&pDev->uart, in_storage, out_storage, STORAGE_SIZE, STORAGE_SIZE );
}
//------------------------------------------------------------------------------------
void cargarRingBuffer( ringBuffer_s *rB, uint8_t *buffer, uint16_t xBytes )
{

uint8_t cChar;
uint16_t bytesToPoke = xBytes;

	while (*buffer && (bytesToPoke-- > 0) ) {
		cChar = *buffer++;
		//ringBufferPoke( rB, (char *)&cChar );
		rBufferPoke( rB, (char *)&cChar );
		pos++;
	}

}
//------------------------------------------------------------------------------------
void volcarRingBuffer( ringBuffer_s *rB, uint8_t *buffer, uint16_t xBytes )
{

uint8_t cChar;
uint16_t bytesToPop = 0;

	while ( bytesToPop < xBytes) {
		//if ( ringBufferPop( rB, (char *)&cChar ) ) {
		if ( rBufferPop( rB, (char *)&cChar ) ) {
			buffer[bytesToPop++] = toupper(cChar);
			pos++;
			if ( cChar == '\0')
				return;
		} else {
			break;
		}
	}

}
//------------------------------------------------------------------------------------
void puebas_ring_buffer(void)
{

	uartInit(&uart,&in_storage[0], &out_storage[0], STORAGE_SIZE, STORAGE_SIZE);

	while(1) {
	strcpy(test_buffer,"PabloTomasPeluffoFleitas\r\n\0");
	pos = 0;

	//ringBufferFlush(&uart.inBuffer);
	rBufferFlush(&uart.TXringBuffer);
	cargarRingBuffer(&uart.TXringBuffer, (uint8_t *)&test_buffer, STORAGE_SIZE);

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Cargar %d\r\n\0"),pos);
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );

	pos = 0;
	memset(test_buffer,'\0',STORAGE_SIZE);
	volcarRingBuffer( &uart.TXringBuffer, (uint8_t *)&test_buffer, STORAGE_SIZE);

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Volcar %d\r\n\0"),pos);
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );

//	CMD_write( &test_buffer, sizeof(test_buffer) );

	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
	}
}
//------------------------------------------------------------------------------------
void puebas2_ring_buffer(void)
{

	perifericoUartInit(&pUsb ,&in_storage[0], &out_storage[0], STORAGE_SIZE, STORAGE_SIZE);
	strcpy(test_buffer,"SpymovilKeynetAvanza\r\n\0");
	pos = 0;

	//ringBufferFlush(&pUsb.uart.inBuffer);
	rBufferFlush(&uart.RXringBuffer);
	cargarRingBuffer(&pUsb.uart.TXringBuffer, (uint8_t *)&test_buffer, STORAGE_SIZE);
	UartInterruptOn();
	return;

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Cargar %d\r\n\0"),pos);
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );

	pos = 0;
	memset(test_buffer,'\0',STORAGE_SIZE);
	volcarRingBuffer( &pUsb.uart.TXringBuffer, (uint8_t *)&test_buffer, STORAGE_SIZE);

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Volcar %d\r\n\0"),pos);
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );

//	CMD_write( &test_buffer, sizeof(test_buffer) );
}
//------------------------------------------------------------------------------------
void puebas3_ring_buffer(void)
{
	// Cargo un ring buffer y lo envio por el serial

//	uartInit(&uart,&in_storage[0], &out_storage[0], STORAGE_SIZE, STORAGE_SIZE);

	while (1) {
		strcpy(test_buffer,"PabloTomasPeluffoFleitas456\r\n\0");
		//rBufferFlush(&uart.TXringBuffer);
		//cargarRingBuffer(&uart.TXringBuffer, (uint8_t *)&test_buffer, STORAGE_SIZE);
		rBufferFlush(&uart_usb.TXringBuffer);
		cargarRingBuffer(&uart_usb.TXringBuffer, (uint8_t *)&test_buffer, STORAGE_SIZE);

		UartInterruptOn();

		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
	}
}
//------------------------------------------------------------------------------------
void UARTInit( void )
{

	portENTER_CRITICAL();
		// Corresponde a PORTD

		PORTD.DIRSET   = PIN3_bm;	/* PD3 (TXD0) as output. */
		PORTD.DIRCLR   = PIN2_bm;	/* PD2 (RXD0) as input. */

		/* USARTD0, 8 Data bits, No Parity, 1 Stop bit. */
		USARTD0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

		USARTD0.BAUDCTRLA = (uint8_t) 2094;
		USARTD0.BAUDCTRLB = ( -7 << USART_BSCALE0_bp)|(2094 >> 8);

		// Habilito la TX y RX
		USARTD0.CTRLB |= USART_RXEN_bm;
		USARTD0.CTRLB |= USART_TXEN_bm;

		// Habilito la interrupcion de Recepcion ( low level )
		// low level, RXint enabled
		USARTD0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
		USARTD0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
		//USARTD0.CTRLA = ( USARTD0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;

	portEXIT_CRITICAL();
}
//------------------------------------------------------------------------------------
void UartInterruptOn(void)
{

uint8_t tempCTRLA;

		tempCTRLA = USARTD0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTD0.CTRLA = tempCTRLA;
}
//------------------------------------------------------------------------------------
void UartInterruptOff(void)
{

uint8_t tempCTRLA;

		tempCTRLA = USARTD0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTD0.CTRLA = tempCTRLA;
}
//------------------------------------------------------------------------------------
int UART_write( void *pvBuffer, const size_t xBytes )
{
	// Esta funcion debe poner los caracteres apuntados en pvBuffer en la cola de trasmision.
	// Actua como si fuese rprintfStr.
	// Debe tomar el semaforo antes de trasmitir. Los semaforos los manejamos en la capa FreeRTOS
	// y no en la de los drivers.

char cChar;
char *p;
size_t bytes2tx;
size_t wBytes = 0;

	// Controlo no hacer overflow en la cola de trasmision
	bytes2tx = xBytes;
	// Trasmito.
	// Cargo el buffer en la cola de trasmision.
	p = (char *)pvBuffer;
	while (*p && (bytes2tx-- > 0) ) {

		// Voy cargando la cola de a uno.
		cChar = *p;
		ringBufferPoke( &uart.TXringBuffer, &cChar  );
		p++;
		wBytes++;	// Cuento los bytes que voy trasmitiendo
	}

	// Luego inicio la trasmision invocando la interrupcion.
	UartInterruptOn();

	//return xBytes;	// Puse todos los caracteres en la cola.
	return (wBytes);

}
//------------------------------------------------------------------------------------
// UART ISR:
//------------------------------------------------------------------------------------
/*
ISR(USARTD0_DRE_vect)
{

char cChar;
int8_t res = pdFALSE;

	res = rBufferPop( &uart.TXringBuffer, (char *)&cChar );

	if( res == pdTRUE ) {

		USARTD0.DATA = cChar;
	} else {

		UartInterruptOff();
	}
}
*/
/*
//------------------------------------------------------------------------------------
ISR(USARTD0_RXC_vect)
{

char cChar;

	cChar = USARTD0.DATA;

	if( rBufferPokeFromISR( &pUsb.uart.RXringBuffer, &cChar ) ) {
		taskYIELD();
	}
}
//------------------------------------------------------------------------------------
*/
