/*
 * FRTOS-IO.h
 *
 *  Created on: 2/10/2015
 *      Author: pablo
 */

#ifndef SRC_FRTOS_IO_FRTOS_IO_H_
#define SRC_FRTOS_IO_FRTOS_IO_H_

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <drivers/spx_uart.h>
#include <FreeRTOS.h>
#include <l_iopines.h>
#include <semphr.h>
#include <task.h>

#include "l_ringBuffer.h"

/* FreeRTOS includes. */

// Describo los diferentes perifericos a los  que puede acceder la aplicacion.
// Para c/u voy a implementar funciones de init,write,read.
typedef enum {
	pUART_USB = 0,
} t_perifericos;


/* Peripheral handles are void * for data hiding purposes. */
typedef const void * Peripheral_Descriptor_t;

/* Types that define valid read(), write() and ioctl() functions. */
typedef size_t ( *Peripheral_write_Function_t )( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes );
typedef size_t ( *Peripheral_read_Function_t )( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes );
typedef portBASE_TYPE ( *Peripheral_ioctl_Function_t )( Peripheral_Descriptor_t const pxPeripheral, uint32_t ulRequest, void *pvValue );

//------------------------------------------------------------------------------------
// Estructura generica de todos los perifericos.
// Los elementos particulares quedan en una estructura que se accede por el punter
// phDevice

typedef struct {

	t_perifericos	portId;
	SemaphoreHandle_t xBusSemaphore;		//
	uint8_t xBlockTime;						// ticks to block in read operations. Set by ioctl

	Peripheral_write_Function_t write;		// The function used to write to the peripheral
	Peripheral_read_Function_t read;		// The function used to read from the peripheral
	Peripheral_ioctl_Function_t ioctl;		// The function used for ioctl access to the peripheral

	void *phDevice;			// puntero a una estructura con los miembros adecuados al periferico

} Peripheral_Control_t;

// Dispositivos a los que voy a poder acceder.
Peripheral_Control_t pdUART_USB;

#define FreeRTOS_write( xPeripheral, pvBuffer, xBytes ) ( ( Peripheral_Control_t * ) xPeripheral )->write( ( ( Peripheral_Control_t * ) xPeripheral ), ( pvBuffer ), ( xBytes ) )
#define FreeRTOS_read( xPeripheral, pvBuffer, xBytes ) ( ( Peripheral_Control_t * ) xPeripheral )->read( ( ( Peripheral_Control_t * ) xPeripheral ), ( pvBuffer ), ( xBytes ) )

//------------------------------------------------------------------------------------
// Estructura particular de UARTs
//------------------------------------------------------------------------------------
#define FIFO	0
//-----------------------------------------------------------------------
// Defino el tipo de almacenamiento que requiere la aplicacion
// 0: queue, 1: fifo
#define UART_RXFIFO		1
#define UART_TXFIFO		2
#define UART_RXQUEUE	4
#define UART_TXQUEUE	8
//-----------------------------------------------------------------------
// Defino el tamanio de los buffers
#define  UART_USB_RXBUFFER_LEN ( ( uint8_t ) ( 128 ))
#define  UART_USB_TXBUFFER_LEN ( ( uint8_t ) ( 128 ))

typedef struct {

	// En las UART solo requiero un estructura de almacenamiento de datos.
	uint8_t rxBufferType;
	void *rxStruct;			// uso puntero void para poder asignarle cualquier estructura.
	uint16_t rxBufferLength;

	uint8_t txBufferType;
	void *txStruct;
	uint16_t txBufferLength;

} UART_device_control_t;

//------------------------------------------------------------------------------------

size_t FreeRTOS_UART_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes );
size_t FreeRTOS_UART_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes );

char *FreeRTOS_UART_getFifoPtr(Peripheral_Control_t *UART);


#endif /* SRC_FRTOS_IO_FRTOS_IO_H_ */
