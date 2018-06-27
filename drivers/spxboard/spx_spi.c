/*
 * spx_spi.c
 *
 *  Created on: 2 de oct. de 2017
 *      Author: pablo
 */

#include "spx_spi.h"
#include "../../spx.h"

static inline uint8_t pv_SPI_transceive_byte(uint8_t data);
//------------------------------------------------------------------------------------
void SPI_init(void)
{
	// En nuestra arquitectura vamos a usar la SPIC

 	/* MOSI, SS and SCK as output. */
//	PORTC.DIRSET  = SPI_MOSI_bm | SPI_SCK_bm | SPI_SS_bm;


	// El pin CS es output e inicialimente esta HIGH
	// Se inicializa en initMCU().
//	MEMSPI_CS_PORT.DIR |= _BV(MEMSPI_CS_PIN);			// MEMSPI_CS output
//	MEMSPI_CS_PORT.OUT |= _BV(MEMSPI_CS_PIN);			// MEMSPI_CS High

	SPIC.CTRL   = SPI_PRESCALER_DIV64_gc |      /* SPI prescaler. */
							0 |     			/* SPI Clock double. */
							SPI_ENABLE_bm |     /* Enable SPI module. */
							0 |  				/* Data order. */
							SPI_MASTER_bm |     /* SPI master. */
							SPI_MODE_0_gc;      /* SPI mode. */

	/* Interrupt level. */
	SPIC.INTCTRL = SPI_INTLVL_OFF_gc;	// Disabled


}
//------------------------------------------------------------------------------------
bool SPI_TransceivePacket(char *TXdataPacket, size_t TXbytes, char *RXdataPacket, size_t RXbytes)
{
	// Debo enviar los bytes de TXdataPacket ( TXbytes ) y luego recibir RXbytes que
	// los voy a almacenar en RXdataPacket.
	// Para esto debo clockear dummy bytes RXbytes y leer las respuestas

	// Envio el comando/direccion/etc.
#ifdef DEBUG_SPI
		FRTOS_snprintf_P( stdout_buff,CHAR128,PSTR("SPI_TP_TX: %d\r\n\0"),TXbytes);
		CMD_write( stdout_buff, sizeof(stdout_buff) );
#endif

 	while ( TXbytes-- > 0 ) {
 		pv_SPI_transceive_byte(*TXdataPacket++);
	}

	// Recibo datos: para esto debo mandar el clock con dummy bytes (0xFF).
 	// En un ciclo de escritura no recibo nada.
#ifdef DEBUG_SPI
		FRTOS_snprintf_P( stdout_buff,CHAR128,PSTR("SPI_TP_RX: %d\r\n\0"),RXbytes);
		CMD_write( stdout_buff, sizeof(stdout_buff) );
#endif

	while ( RXbytes-- > 0 ) {
		*RXdataPacket++ = pv_SPI_transceive_byte(0xFF);
	}

	return true;
}
//------------------------------------------------------------------------------------
static inline uint8_t pv_SPI_transceive_byte(uint8_t data)
{
	// Trasmito escribiendo en el registro DATA.
	// Asumo que esta libre.
	// Se utiliza en casos que no necesito la respuesta como ser WREN, WDIS,
	// CHIP ERASE, DEEP_PWR_MODE.

uint8_t rsp;

	SPIC.DATA = data;
	// Espero que la trasmision se complete
	while(!(SPIC.STATUS & SPI_IF_bm)) {
		taskYIELD();
	}

	rsp = SPIC.DATA;
#ifdef DEBUG_SPI
		FRTOS_snprintf_P( stdout_buff,CHAR128,PSTR("SPI_TB: tx:0x%02x, rx:0x%02x\r\n\0"),data, rsp );
		CMD_write( stdout_buff, sizeof(stdout_buff) );
#endif

	return( rsp );

}
//------------------------------------------------------------------------------------
