/*
 * sp5K_uart.h
 *
 *  Created on: 4/10/2015
 *      Author: pablo
 *
 *  Un tema importante es la estructura donde almacenamos los buffers.
 *  Si usamos queues, debemos tener en cuenta que el largo maximo viene dado por
 *  una variable tipo unsigned char que entonces solo almacena 255 valores.
 */

#ifndef SRC_DRIVERS_SP5K_UART_H_
#define SRC_DRIVERS_SP5K_UART_H_

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "FreeRTOS.h"

#include "FRTOS-IO.h"

//-----------------------------------------------------------------------
void pvUARTInit( const int UARTx );
void vUartInterruptOn(int UARTx);
void vUartInterruptOff(int UARTx);

#endif /* SRC_DRIVERS_SP5K_UART_H_ */
