/*
 * l_spiflash.h
 *
 *  Created on: 3 de oct. de 2017
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_SPIFLASH_H_
#define SRC_SPX_LIBS_L_SPIFLASH_H_

#include "FRTOS-IO.h"
#include "stdio.h"

#include "../spx.h"

// La memoria SPI usada es una 25AA1024 de 512 paginas de 256 bytes c/u
#define SPIMEM_25AA1024_PAGES	512

#define SPICMD_PAGE_ERASE		0x42
#define SPICMD_READ_STATUS		0x05
#define SPICMD_CHIP_ERASE		0xC7
#define SPICMD_WRITE_ENABLE		0x06
#define SPICMD_WRITE_DISABLE	0x04
#define SPICMD_WRITE			0x02
#define SPICMD_READ				0x03

//------------------------------------------------------------------------------------

#define SPIFLASH_GET_SEMAPHORE() FreeRTOS_ioctl(&pdSPI,ioctlOBTAIN_BUS_SEMPH, NULL)
#define SPIFLAHS_RELEASE_SEMAPHORE() FreeRTOS_ioctl(&pdSPI,ioctlRELEASE_BUS_SEMPH, NULL)

bool SPIFLASH_page_erase(uint16_t page);
bool SPIFLASH_read( uint16_t pageAddress, char *data, uint8_t length );
bool SPIFLASH_write( uint16_t pageAddress, char *data, uint8_t length );
bool SPIFLASH_write_enable(void);
bool SPIFLASH_write_disable(void);
uint8_t SPIFLASH_read_status(void);
bool SPIFLASH_chip_erase(void);

//bool SPIFLASH_read( uint16_t eeAddress, char *data, uint8_t length );
//bool SPIFLASH_write( uint16_t pageAddress, char *data, uint8_t length );

bool SPIFLASH_test_write(char *s0, char *s1);
bool SPIFLASH_test_read(char *s0, char *s1, char *s2);
void SPIFLASH_test_assertCS(void);
void SPIFLASH_test_releaseCS(void);
void SPIFLASH_test_CS(void);


#endif /* SRC_SPX_LIBS_L_SPIFLASH_H_ */
