/*
 * spx_spi.h
 *
 *  Created on: 2 de oct. de 2017
 *      Author: pablo
 */

#ifndef SRC_DRIVERS_SPXBOARD_SPX_SPI_H_
#define SRC_DRIVERS_SPXBOARD_SPX_SPI_H_

#include <avr/io.h>
#include <stdbool.h>
#include "FRTOS-IO.h"

void SPI_init(void);
bool SPI_TransceivePacket(char *TXdataPacket, size_t TXbytes, char *RXdataPacket, size_t RXbytes);


#endif /* SRC_DRIVERS_SPXBOARD_SPX_SPI_H_ */
