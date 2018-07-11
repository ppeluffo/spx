/*
 * frtos-io.h
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 *
 * Funcionamiento:
 * Usamos los servicios que nos brindan los drivers.
 * Definimos para cada periferico una estructura de control que depende del periferico.
 * En el caso del los puertos seriales es periferico_serial_port_t.
 * Este tiene un elemento que es un puntero a una uart definida en el driver.
 *
 * Cada periferico se asocia a un file descriptor de modo que las funciones genericas
 * frtos_open/ioctl/read/write por medio de un switch redirigen a funciones mas especializadas
 * en cada tipo de periferico.
 *
 */

//#ifndef SRC_FRTOS_IO_FRTOS_IO_H_
//#define SRC_FRTOS_IO_FRTOS_IO_H_

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "FreeRTOS.h"
#include "task.h"
#include <semphr.h>

#include "../drivers/drv_uart_spx.h"

// Identificador de los file descriptor.
typedef enum {
	fdUSB = 1,

} file_descriptor_t;

// Estructuctura generica de un periferico tipo serial port.
typedef struct {
	file_descriptor_t fd;
	SemaphoreHandle_t xBusSemaphore;		//
	uint8_t xBlockTime;						// ticks to block in read operations. Set by ioctl
	uart_control_t *uart;					// puntero a una estructura con los miembros adecuados al periferico
	                                        // La uart pertenece al driver. El periferico la apunta.
} periferico_serial_port_t;

// Periferico real.
periferico_serial_port_t xComUSB;


#define ioctlOBTAIN_BUS_SEMPH		1
#define ioctlRELEASE_BUS_SEMPH		2
#define ioctlSET_TIMEOUT			3
#define ioctl_UART_CLEAR_RX_BUFFER	4
#define ioctl_UART_CLEAR_TX_BUFFER	5

//-----------------------------------------------------------------------
int frtos_open( file_descriptor_t fd, uint32_t flags);
int frtos_uart_open( periferico_serial_port_t *xCom, uint32_t flags);

int frtos_write( file_descriptor_t fd ,const char *pvBuffer, const size_t xBytes );
int frtos_uart_write( periferico_serial_port_t *xCom, const char *pvBuffer, const size_t xBytes );

int frtos_ioctl( file_descriptor_t fd, uint32_t ulRequest, void *pvValue );
int frtos_uart_ioctl( periferico_serial_port_t *xCom, uint32_t ulRequest, void *pvValue );

//#endif /* SRC_FRTOS_IO_FRTOS_IO_H_ */
