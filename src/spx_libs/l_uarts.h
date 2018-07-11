/*
 * uarts_sp5K.h
 *
 *  Created on: 15 de nov. de 2016
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_UARTS_H_
#define SRC_SPX_LIBS_L_UARTS_H_

#include <FRTOS-IO/FRTOS-IO.h>

#include "../drivers/spx_uart.h"

#define USB_write( dataBuff, xsize ) ( FreeRTOS_write( &pdUART_USB, dataBuff, xsize ) )
#define USB_read( dataBuff, xsize ) ( FreeRTOS_read( &pdUART_USB, dataBuff, xsize ) )
#define USB_RXflush() ( FreeRTOS_ioctl( &pdUART_USB,ioctl_UART_CLEAR_RX_BUFFER, NULL) )
#define USB_TXflush() ( FreeRTOS_ioctl( &pdUART_USB,ioctl_UART_CLEAR_TX_BUFFER, NULL) )

#define BT_write( dataBuff, xsize ) ( FreeRTOS_write( &pdUART_BT, dataBuff, xsize ) )
#define BT_read( dataBuff, xsize ) ( FreeRTOS_read( &pdUART_BT, dataBuff, xsize ) )
#define BT_RXflush() ( FreeRTOS_ioctl( &pdUART_BT,ioctl_UART_CLEAR_RX_BUFFER, NULL) )
#define BT_TXflush() ( FreeRTOS_ioctl( &pdUART_BT,ioctl_UART_CLEAR_TX_BUFFER, NULL) )

#define GPRS_write( dataBuff, xsize ) ( FreeRTOS_write( &pdUART_GPRS, dataBuff, xsize ) )
#define GPRS_read( dataBuff, xsize ) ( FreeRTOS_read( &pdUART_GPRS, dataBuff, xsize ) )
#define GPRS_RXflush() ( FreeRTOS_ioctl( &pdUART_GPRS,ioctl_UART_CLEAR_RX_BUFFER, NULL) )
#define GPRS_TXflush() ( FreeRTOS_ioctl( &pdUART_GPRS,ioctl_UART_CLEAR_TX_BUFFER, NULL) )

#define XBEE_write( dataBuff, xsize ) ( FreeRTOS_write( &pdUART_XBEE, dataBuff, xsize ) )
#define XBEE_read( dataBuff, xsize ) ( FreeRTOS_read( &pdUART_XBEE, dataBuff, xsize ) )
#define XBEE_RXflush() ( FreeRTOS_ioctl( &pdUART_XBEE,ioctl_UART_CLEAR_RX_BUFFER, NULL) )
#define XBEE_TXflush() ( FreeRTOS_ioctl( &pdUART_XBEE,ioctl_UART_CLEAR_TX_BUFFER, NULL) )

void CMD_write( const void *pvBuffer, const size_t xBytes );
size_t CMD_read( void *pvBuffer, const size_t xBytes );
void CMD_writeChar (unsigned char c);
size_t uXBEE_read(  void *pvBuffer, const size_t xBytes );
#endif /* SRC_SPX_LIBS_L_UARTS_H_ */
