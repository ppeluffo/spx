/*
 * drv_uart_spx.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 */

#include "drv_uart_spx.h"

//----------------------------------------------------------------------------------------
uart_control_t *drv_uart_init( uart_id_t iUART, uint32_t baudrate )
{
	// El puerto del USB es PORTD:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX

uart_control_t *pUart;

	switch(iUART) {
	case iUART_USB:
		// Abro el puerto serial y fijo su velocidad
		drv_uart_usb_open(baudrate);
		// Inicializo los ringBuffers que manejan el puerto. Son locales al driver.
		rBufferCreateStatic( &uart_usb.RXringBuffer, &usb_rxStorage[0], USB_RXSTORAGE_SIZE );
		rBufferCreateStatic( &uart_usb.TXringBuffer, &usb_txStorage[0], USB_RXSTORAGE_SIZE );
		// Asigno el identificador
		uart_usb.uart_id = iUART_USB;
		// Devuelvo la direccion de uart_usb para que la asocie al dispositvo USB el frtos.
		pUart = (uart_control_t *)&uart_usb;
		break;
	}

	return(pUart);
}
//----------------------------------------------------------------------------------------
void drv_uart_interruptOn(uart_id_t iUART)
{
	// Habilito la interrupcion TX del UART lo que hace que se ejecute la ISR_TX y
	// esta vaya a la TXqueue y si hay datos los comienze a trasmitir.

uint8_t tempCTRLA;

	switch(iUART) {
	case iUART_USB:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTD0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTD0.CTRLA = tempCTRLA;
		break;
	}

}
//----------------------------------------------------------------------------------------
void drv_uart_interruptOff(uart_id_t iUART)
{

uint8_t tempCTRLA;

	switch(iUART) {
	case iUART_USB:
		// TXint disabled
		// Espero que no halla nada en el DREG
		tempCTRLA = USARTD0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTD0.CTRLA = tempCTRLA;
		break;
	}

}
//----------------------------------------------------------------------------------------
void drv_set_baudrate(uint32_t baudRate, uint8_t *baudA, uint8_t *baudB, uint8_t *ctl )
{
#if F_CPU == 32000000
	/* Set Baudrate to 115200 bps:
	 * Use the default I/O clock frequency that is 32 MHz.
	 * Los valores los extraigo de la planilla provista por Atmel
	 * 32Mhz
	 * BSEL = 2094
	 * BSCALE = -7
	 * CLK2X = 0
	 * %error = 0,01%
	 */
	*baudA = (uint8_t) 2094;
	*baudB = ( -7 << USART_BSCALE0_bp)|(2094 >> 8);
#endif

#if F_CPU == 8000000
		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 32 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 * 8Mhz
		 * BSEL = 983
		 * BSCALE = -7
		 * CLK2X = 1
		 * %error = 0,01%
		 */
	*baudA = (uint8_t) 983;
	*baudB = ( -7 << USART_BSCALE0_bp)|(983 >> 8);
		// Habilito CLK2X
	*ctl |= USART_CLK2X_bm;
#endif

#if F_CPU == 2000000
		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 2 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 * 2Mhz
		 * BSEL = 11
		 * BSCALE = -7
		 * CLK2X = 0
		 * %error = 0,08%
		 */
		*baudA = (uint8_t) 11;
		*baudB = ( -7 << USART_BSCALE0_bp)|(11 >> 8);
#endif
}
//----------------------------------------------------------------------------------------
void drv_uart_usb_open( uint32_t baudrate )
{
	// El puerto del USB es PORTD:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX

uint8_t baudA, baudB, ctl;

	PORTD.DIRSET   = PIN3_bm;	// PD3 (TXD0) as output.
	PORTD.DIRCLR   = PIN2_bm;	// PD2 (RXD0) as input.
	// USARTD0, 8 Data bits, No Parity, 1 Stop bit.
	USARTD0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

	ctl = USARTD0.CTRLB;
	drv_set_baudrate( baudrate, &baudA, &baudB, &ctl);
	USARTD0.BAUDCTRLA = baudA;
	USARTD0.BAUDCTRLB = baudB;
	USARTD0.CTRLB = ctl;

	// Habilito la TX y RX
	USARTD0.CTRLB |= USART_RXEN_bm;
	USARTD0.CTRLB |= USART_TXEN_bm;

	// Habilito la interrupcion de Recepcion ( low level )
	// low level, RXint enabled
	USARTD0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
	USARTD0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
	//USARTD0.CTRLA = ( USARTD0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;

	return;
}
//----------------------------------------------------------------------------------------
// UART USB ISR:
//----------------------------------------------------------------------------------------
ISR(USARTD0_DRE_vect)
{

char cChar;
int8_t res = false;

	res = rBufferPop( &uart_usb.TXringBuffer, (char *)&cChar );

	if( res == true ) {
		// Send the next character queued for Tx
		USARTD0.DATA = cChar;
	} else {
		// Queue empty, nothing to send.
		drv_uart_interruptOff(uart_usb.uart_id);
	}
}
//----------------------------------------------------------------------------------------
ISR(USARTD0_RXC_vect)
{

char cChar;

	cChar = USARTD0.DATA;

	if( rBufferPokeFromISR( &uart_usb.RXringBuffer, &cChar ) ) {
		taskYIELD();
	}
}
//----------------------------------------------------------------------------------------


