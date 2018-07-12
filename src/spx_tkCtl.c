/*
 * spx_tkCtl.c
 *
 *  Created on: 4 de oct. de 2017
 *      Author: pablo
 */

#include <spx.h>

//------------------------------------------------------------------------------------
static void pv_tkCtl_wink_led(void);

static char ctl_printfBuff[CHAR128];

//------------------------------------------------------------------------------------
void tkCtl(void * pvParameters)
{

( void ) pvParameters;

	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
uint8_t i = 0;

	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("\r\nstarting tkControl..\r\n\0"));
	CMD_write(ctl_printfBuff, sizeof(ctl_printfBuff) );


	for( ;; )
	{

		WDT_Reset();

		// Para entrar en tickless.
		// Cada 5s hago un chequeo de todo. En particular esto determina el tiempo
		// entre que activo el switch de la terminal y que esta efectivamente responde.
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );

		pv_tkCtl_wink_led();

//		i++;
//		FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Contando++ %i\r\n\0"),i);
//		CMD_write(ctl_printfBuff, sizeof(ctl_printfBuff) );

//		puebas_ring_buffer();
//		puebas2_ring_buffer();
//		puebas3_ring_buffer();

	}
}
//------------------------------------------------------------------------------------
static void pv_tkCtl_wink_led(void)
{


	// Prendo los leds
	IO_set_LED_KA();
	IO_set_LED_COMMS();

	vTaskDelay( ( TickType_t)( 5 / portTICK_RATE_MS ) );

	// Apago
	IO_clr_LED_KA();
	IO_clr_LED_COMMS();

}
//------------------------------------------------------------------------------------
