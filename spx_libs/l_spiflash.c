/*
 * l_spiflash.c
 *
 *  Created on: 3 de oct. de 2017
 *      Author: pablo
 */

#include "../spx_libs/l_spiflash.h"

static bool pv_spiflash_write_enable(void);
static bool pv_spiflash_write_disable(void);
static uint8_t pv_spiflash_read_status(void);
static bool pv_spiflash_chip_erase(void);
static bool pv_spiflash_page_erase(uint16_t pageAddress);
static bool pv_spiflash_write( uint16_t pageAddress, char *data, uint8_t length );
static bool pv_spiflash_read( uint16_t pageAddress, char *data, uint8_t length );

// Las operaciones diretas de SPI no manejan el semaforo.
// Lo hacen las operaciones de TEST y las de FILE.

//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS QUE SI USAN EL SEMAFORO
//------------------------------------------------------------------------------------
bool SPIFLASH_read( uint16_t pageAddress, char *data, uint8_t length )
{
	// Escribo un buffer de datos en la memoria. Se escribe siempre
	// a partir de una frontera de pagina.

	// Lo primero es obtener el semaforo
	SPIFLASH_GET_SEMAPHORE();

	// Leo
	pv_spiflash_read( pageAddress, data, length );

	// Libero el semaforo.
	SPIFLAHS_RELEASE_SEMAPHORE();

	return(true);
}
//------------------------------------------------------------------------------------
bool SPIFLASH_write( uint16_t pageAddress, char *data, uint8_t length )
{
	// Escribo un buffer de datos en la memoria. Se escribe siempre
	// a partir de una frontera de pagina.
	// En realidad la eeAddress es la page address que debe ser < 512 para la 25AA1024.

	// Lo primero es obtener el semaforo
	SPIFLASH_GET_SEMAPHORE();
	// Habilito a escribir
	pv_spiflash_write_enable();
	vTaskDelay( ( TickType_t)( 1 ));

	// Escribo
	pv_spiflash_write( pageAddress, data, length );
	vTaskDelay( ( TickType_t)( 1 ));

	// Deshabilito la escritura
	pv_spiflash_write_disable();
	vTaskDelay( ( TickType_t)( 1 ));

	// Libero el semaforo.
	SPIFLAHS_RELEASE_SEMAPHORE();

	return(true);

}
//------------------------------------------------------------------------------------
bool SPIFLASH_write_enable(void)
{
bool retS = false;

	SPIFLASH_GET_SEMAPHORE();
	retS = pv_spiflash_write_enable();
	SPIFLAHS_RELEASE_SEMAPHORE();
	return(retS);
}
//------------------------------------------------------------------------------------
bool SPIFLASH_write_disable(void)
{
bool retS = false;

	SPIFLASH_GET_SEMAPHORE();
	retS = pv_spiflash_write_disable();
	SPIFLAHS_RELEASE_SEMAPHORE();
	return(retS);
}
//------------------------------------------------------------------------------------
uint8_t SPIFLASH_read_status(void)
{
uint8_t retData = 0x00;

		SPIFLASH_GET_SEMAPHORE();
		retData = pv_spiflash_read_status();
		SPIFLAHS_RELEASE_SEMAPHORE();
		return(retData);
}
//------------------------------------------------------------------------------------
bool SPIFLASH_page_erase(uint16_t page)
{
bool retS = false;

	SPIFLASH_GET_SEMAPHORE();

	// Habilito a escribir
	pv_spiflash_write_enable();
	vTaskDelay( ( TickType_t)( 1 ));

	retS = pv_spiflash_page_erase(page);
	vTaskDelay( ( TickType_t)( 1 ));

	// Deshabilito la escritura
	//pv_spiflash_write_disable();

	SPIFLAHS_RELEASE_SEMAPHORE();
	return(retS);
}
//------------------------------------------------------------------------------------
bool SPIFLASH_chip_erase(void)
{
bool retS = false;

	SPIFLASH_GET_SEMAPHORE();

	// Habilito a escribir
	pv_spiflash_write_enable();

	retS = pv_spiflash_chip_erase();

	// Deshabilito la escritura
	//pv_spiflash_write_disable();

	SPIFLAHS_RELEASE_SEMAPHORE();
	return(retS);
}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS QUE NO USAN EL SEMAFORO
//------------------------------------------------------------------------------------
static bool pv_spiflash_read( uint16_t pageAddress, char *data, uint8_t length )
{
	// Escribo un buffer de datos en la memoria. Se escribe siempre
	// a partir de una frontera de pagina.

uint8_t spi_command_buffer[4];

	memset(spi_command_buffer, '\0', sizeof(spi_command_buffer));

	// ASSERT CS
	FreeRTOS_ioctl(&pdSPI, ioctl_SPI_FLASH_ASSERT_CS, NULL);
	// Indicamos el comando SPI a ejecutar
	spi_command_buffer[0] = SPICMD_READ;
	// Indicamos la pagina
	spi_command_buffer[1] = pageAddress >> 8;		// Addr 23..16
	spi_command_buffer[2] = pageAddress & 0xFF;		// Addr 8..15
	spi_command_buffer[3] = 0x00;					// Addr 0..7

	// Ejecuto
	// Envio el comando. page solo requiere 4 bytes con el comando RS=0x02
	FreeRTOS_write(&pdSPI, spi_command_buffer, 4);
	// Ahora leo el buffer.
	FreeRTOS_read(&pdSPI, data, length );

	// RELEASE CS
	FreeRTOS_ioctl(&pdSPI,ioctl_SPI_FLASH_RELEASE_CS, NULL);

	return(true);
}
//------------------------------------------------------------------------------------
static bool pv_spiflash_write( uint16_t pageAddress, char *data, uint8_t length )
{
	// Escribo un buffer de datos en la memoria. Se escribe siempre
	// a partir de una frontera de pagina.
	// En realidad la eeAddress es la page address que debe ser < 512 para la 25AA1024.

uint8_t spi_command_buffer[4];

	memset(spi_command_buffer, '\0', sizeof(spi_command_buffer));

	// ASSERT CS
	FreeRTOS_ioctl(&pdSPI, ioctl_SPI_FLASH_ASSERT_CS, NULL);
	// Indicamos el comando SPI a ejecutar
	spi_command_buffer[0] = SPICMD_WRITE;
	// Indicamos la pagina
	spi_command_buffer[1] = pageAddress >> 8;		// Addr 23..16
	spi_command_buffer[2] = pageAddress & 0xFF;		// Addr 8..15
	spi_command_buffer[3] = 0x00;					// Addr 0..7

	// Ejecuto
	// Envio el comando. page solo requiere 4 bytes con el comando RS=0x02
	FreeRTOS_write(&pdSPI, spi_command_buffer, 4);

	// Ahora mando el buffer.
	FreeRTOS_write(&pdSPI, data, length );

	// RELEASE CS
	FreeRTOS_ioctl(&pdSPI,ioctl_SPI_FLASH_RELEASE_CS, NULL);

	return(true);
}
//------------------------------------------------------------------------------------
static bool pv_spiflash_write_enable(void)
{
	// Antes de escribir debe destrabarse la memoria con un write enable

uint8_t spi_command_buffer[3];

	memset(spi_command_buffer, '\0', sizeof(spi_command_buffer));

	// ASSERT CS
	FreeRTOS_ioctl(&pdSPI, ioctl_SPI_FLASH_ASSERT_CS, NULL);
	// Indicamos el comando SPI a ejecutar
	spi_command_buffer[0] = SPICMD_WRITE_ENABLE;

	// Ejecuto
	// Envio el comando. write enable solo requiere 1 byte con el comando RS=0x06
	FreeRTOS_write(&pdSPI, spi_command_buffer, 1);
	// No hay respuesta

	// RELEASE CS
	FreeRTOS_ioctl(&pdSPI,ioctl_SPI_FLASH_RELEASE_CS, NULL);

	return(true);
}
//------------------------------------------------------------------------------------
static bool pv_spiflash_write_disable(void)
{
	// Bloquea la memoria de intentos de borrado

uint8_t spi_command_buffer[3];

	// ASSERT CS
	FreeRTOS_ioctl(&pdSPI, ioctl_SPI_FLASH_ASSERT_CS, NULL);
	// Indicamos el comando SPI a ejecutar
	spi_command_buffer[0] = SPICMD_WRITE_DISABLE;

	// Ejecuto
	// Envio el comando. write disable solo requiere 1 byte con el comando RS=0x04
	FreeRTOS_write(&pdSPI, spi_command_buffer, 1);
	// No hay respuesta

	// RELEASE CS
	FreeRTOS_ioctl(&pdSPI,ioctl_SPI_FLASH_RELEASE_CS, NULL);

	return(true);
}
//------------------------------------------------------------------------------------
static uint8_t pv_spiflash_read_status(void)
{
uint8_t spi_command_buffer[3];
uint8_t status;

	memset(spi_command_buffer, '\0', sizeof(spi_command_buffer));

	// ASSERT CS
	FreeRTOS_ioctl(&pdSPI, ioctl_SPI_FLASH_ASSERT_CS, NULL);
	// Indicamos el comando SPI a ejecutar
	spi_command_buffer[0] = SPICMD_READ_STATUS;

	// Ejecuto
	// Envio el comando. Read status solo requiere 1 byte con el comando RS=0x05
	FreeRTOS_write(&pdSPI, spi_command_buffer, 1);
	// Leo la respuesta
	FreeRTOS_read(&pdSPI, &status, 1);

	// RELEASE CS
	FreeRTOS_ioctl(&pdSPI,ioctl_SPI_FLASH_RELEASE_CS, NULL);

	return(status);
}
//----------------------------------------------------------------------------------
static bool pv_spiflash_page_erase(uint16_t pageAddress)
{
	// Borra una pagina.

uint8_t spi_command_buffer[4];

	memset(spi_command_buffer, '\0', sizeof(spi_command_buffer));

	// ASSERT CS
	FreeRTOS_ioctl(&pdSPI, ioctl_SPI_FLASH_ASSERT_CS, NULL);
	// Indicamos el comando SPI a ejecutar
	spi_command_buffer[0] = SPICMD_PAGE_ERASE;
	// Indicamos la pagina
	spi_command_buffer[1] = pageAddress >> 8;		// Addr 23..16
	spi_command_buffer[2] = pageAddress & 0xFF;		// Addr 8..15
	spi_command_buffer[3] = 0x00;

	// Ejecuto
	// Envio el comando. page solo requiere 4 bytes con el comando RS=0x42
	FreeRTOS_write(&pdSPI, spi_command_buffer, 4);
	// No hay respuesta

	// RELEASE CS
	FreeRTOS_ioctl(&pdSPI,ioctl_SPI_FLASH_RELEASE_CS, NULL);

	return(true);

}
//----------------------------------------------------------------------------------
static bool pv_spiflash_chip_erase(void)
{
	// Borra toda la memoria.

uint8_t spi_command_buffer[3];

	memset(spi_command_buffer, '\0', sizeof(spi_command_buffer));

	// ASSERT CS
	FreeRTOS_ioctl(&pdSPI, ioctl_SPI_FLASH_ASSERT_CS, NULL);
	// Indicamos el comando SPI a ejecutar
	spi_command_buffer[0] = SPICMD_CHIP_ERASE;
	// Ejecuto
	// Envio el comando. write enable solo requiere 1 byte con el comando RS=0x06
	FreeRTOS_write(&pdSPI, spi_command_buffer, 1);
	// No hay respuesta

	// Luego del comando debo liberar el CS para que comienze el proceso de borrado.
	// RELEASE CS
	FreeRTOS_ioctl(&pdSPI,ioctl_SPI_FLASH_RELEASE_CS, NULL);

	return(true);
}
//----------------------------------------------------------------------------------
// FUNCIONES AUXILIARES
//----------------------------------------------------------------------------------
bool SPIFLASH_test_write(char *s0, char *s1)
{
	/* Se usa para testear desde el modo comando las funciones de escribir la flash.
	 * Desde el modo comando ingresamos 2 parametros que son 2 strings: la direccion y el texto
	 * Para usar SPI_write debemos calcular el largo del texto antes de invocarla
	 */

uint8_t length = 0;
char *p;
bool retS = false;

	// Calculamos el largo del texto a escribir en la eeprom.
	p = s1;
	while (*p != 0) {
		p++;
		length++;
	}
//	FRTOS_snprintf_P( stdout_buff,sizeof(stdout_buff),PSTR("S=[%s](%d)\r\n\0"),s1, length);
//	CMD_write( stdout_buff, sizeof(stdout_buff) );

	retS = SPIFLASH_write( (uint16_t)(atoi(s0)), s1, length );

	return(retS);
}
//------------------------------------------------------------------------------------
bool SPIFLASH_test_read(char *s0, char *s1, char *s2)
{
	/* Se usa para testear desde el modo comando las funciones de leer la flash.
	 * Desde el modo comando ingresamos 2 parametros que son 2 strings: la direccion y el largo
	 */

bool retS = false;

	retS = SPIFLASH_read( (uint16_t)(atoi(s0)), s1, (uint8_t)(atoi(s2)) );
	return(retS);
}
//------------------------------------------------------------------------------------
void SPIFLASH_test_assertCS(void)
{
	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdSPI,ioctlOBTAIN_BUS_SEMPH, NULL);
	// ASSERT CS
	FreeRTOS_ioctl(&pdSPI, ioctl_SPI_FLASH_ASSERT_CS, NULL);
	// Libero el semaforo.
	FreeRTOS_ioctl(&pdSPI,ioctlRELEASE_BUS_SEMPH, NULL);

}
//------------------------------------------------------------------------------------
void SPIFLASH_test_releaseCS(void)
{
	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdSPI,ioctlOBTAIN_BUS_SEMPH, NULL);
	// ASSERT CS
	FreeRTOS_ioctl(&pdSPI, ioctl_SPI_FLASH_RELEASE_CS, NULL);
	// Libero el semaforo.
	FreeRTOS_ioctl(&pdSPI,ioctlRELEASE_BUS_SEMPH, NULL);

}
//------------------------------------------------------------------------------------
