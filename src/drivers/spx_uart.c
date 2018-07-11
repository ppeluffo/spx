/*
 * sp5K_uart.c
 *
 *  Created on: 4/10/2015
 *      Author: pablo
 *
 *  TRASMISION:
 *  El mecanismo elegido por eficiencia es tener una queue en la cual se escriben los
 *  caracteres a trasmitir y se habilita la interrupcion.
 *  Esta, saca los caracteres y los va enviando a la UART.
 *  Esto hace, que la rutina de interrupcion deba saber a priori cual es la cola de trasmision.
 *  Como es una ISR, no se le puede pasar como parametro.
 *  Por otro lado, no  importaria mucho tenerlo definido de antemano ya que se trata de un
 *  sistema embebido.
 */


#include <drivers/spx_uart.h>
#include "FRTOS-IO.h"
#include "l_ringBuffer.h"
#include "spx.h"

/*------------------------------------------------------------------------------------*/
void pvUARTInit( const int UARTx )
{

	// La inicializacion de las UART se hace como:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX

	portENTER_CRITICAL();

	//
	switch ( UARTx ) {
	case pUART_USB:

		// Corresponde a PORTD

		PORTD.DIRSET   = PIN3_bm;	/* PD3 (TXD0) as output. */
		PORTD.DIRCLR   = PIN2_bm;	/* PD2 (RXD0) as input. */

		/* USARTD0, 8 Data bits, No Parity, 1 Stop bit. */
		USARTD0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

#if SYSMAINCLK == 32
		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 32 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 * 32Mhz
		 * BSEL = 2094
		 * BSCALE = -7
		 * CLK2X = 0
		 * %error = 0,01%
		 */
		USARTD0.BAUDCTRLA = (uint8_t) 2094;
		USARTD0.BAUDCTRLB = ( -7 << USART_BSCALE0_bp)|(2094 >> 8);
#endif

#if SYSMAINCLK == 8
		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 32 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 * 8Mhz
		 * BSEL = 983
		 * BSCALE = -7
		 * CLK2X = 1
		 * %error = 0,01%
		 */
		USARTD0.BAUDCTRLA = (uint8_t) 983;
		USARTD0.BAUDCTRLB = ( -7 << USART_BSCALE0_bp)|(983 >> 8);
		// Habilito CLK2X
		USARTD0.CTRLB |= USART_CLK2X_bm;
#endif

#if SYSMAINCLK == 2
		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 2 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 * 2Mhz
		 * BSEL = 11
		 * BSCALE = -7
		 * CLK2X = 0
		 * %error = 0,08%
		 */
		USARTD0.BAUDCTRLA = (uint8_t) 11;
		USARTD0.BAUDCTRLB = ( -7 << USART_BSCALE0_bp)|(11 >> 8);
#endif
	//	USARTD0.BAUDCTRLA = (uint8_t) 1659;
	//	USARTD0.BAUDCTRLB = ( -3 << USART_BSCALE0_bp)|(1659 >> 8);

		// Habilito la TX y RX
		USARTD0.CTRLB |= USART_RXEN_bm;
		USARTD0.CTRLB |= USART_TXEN_bm;

		// Habilito la interrupcion de Recepcion ( low level )
		// low level, RXint enabled
		USARTD0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
		USARTD0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
		//USARTD0.CTRLA = ( USARTD0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;

		break;
	}

	portEXIT_CRITICAL();

	return;
}
/*------------------------------------------------------------------------------------*/
void vUartInterruptOn(int UARTx)
{

	// Habilito la interrupcion TX del UART lo que hace que se ejecute la ISR_TX y
	// esta vaya a la TXqueue y si hay datos los comienze a trasmitir.

uint8_t tempCTRLA;

	switch(UARTx) {
	case pUART_USB:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTD0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTD0.CTRLA = tempCTRLA;
		break;
	}


}
/*------------------------------------------------------------------------------------*/
void vUartInterruptOff(int UARTx)
{

uint8_t tempCTRLA;

	switch(UARTx) {
	case pUART_USB:
		// TXint disabled
		// Espero que no halla nada en el DREG
		tempCTRLA = USARTD0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTD0.CTRLA = tempCTRLA;
		break;
	}

}
/*------------------------------------------------------------------------------------*/
// UART ISR:
/*------------------------------------------------------------------------------------*/
/*
ISR(USARTD0_DRE_vect)
{

char cChar;
int8_t res = pdFALSE;
UART_device_control_t *pUart;

	pUart = pdUART_USB.phDevice;

	res = ringBufferPopFromISR( pUart->txStruct, &cChar );

	if( res == pdTRUE ) {

		USARTD0.DATA = cChar;
	} else {

		vUartInterruptOff(pUART_USB);
	}
}
*/
/*
ISR(USARTD0_RXC_vect)
{

char cChar;
UART_device_control_t *pUart;

	pUart = pdUART_USB.phDevice;

	cChar = USARTD0.DATA;

	if( ringBufferPokeFromISR( pUart->rxStruct, &cChar ) ) {
		taskYIELD();
	}
}
*/
