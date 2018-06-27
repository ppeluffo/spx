/*
 * l_ina3221.c
 *
 *  Created on: 13 de oct. de 2017
 *      Author: pablo
 */

#include "../spx_libs/l_ina3221.h"
#include "spx.h"

#define INA3221_VCC_SETTLE_TIME	500

//------------------------------------------------------------------------------------
bool INA3221_read( uint8_t devAddress, uint8_t regAddress, char *data, uint8_t length )
{
	// C/registro es de 2 bytes de informacion.

size_t xReturn = 0U;
uint8_t val = 0;
uint8_t xBytes = 0;

//		FRTOS_snprintf_P( stdout_buff, sizeof(stdout_buff),PSTR( "INArd: devAddr=0x%02x,regAddr=0x%02x\r\n\0"), devAddress, regAddress );
//		CMD_write( stdout_buff, sizeof(stdout_buff) );

		// Lo primero es obtener el semaforo
		FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);

		// Luego indicamos el periferico i2c en el cual queremos leer
		val = devAddress;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
//		FRTOS_snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: devAddr=0x%02x\r\n\0"), val );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

		// Luego indicamos el registro desde donde leer: largo ( 1 bytes )
		val = 1;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
//		FRTOS_snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: length=0x%02x\r\n\0"), val );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

		// y direccion
		val = regAddress;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
//		FRTOS_snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: regAddr=0x%02x\r\n\0"), val );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

		// Por ultimo leemos.( 2 bytes )
		xBytes = 2;
		xReturn = FreeRTOS_read(&pdI2C, data, xBytes);
//		FRTOS_snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: size=0x%02x\r\n\0"), xBytes );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

		// Y libero el semaforo.
		FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

		if (xReturn != xBytes ) {
			return ( false );
		}

		return(true);

}
//------------------------------------------------------------------------------------
bool INA3221_write( uint8_t devAddress, uint8_t regAddress, char *data, uint8_t length )
{
	// Escribe en el INA3221 en la posicion regAddress, la cantidad
	// 'length' de bytes apuntados por 'data'
	// En los INA3221 siempre se escriben solo 2 bytes de datos !!!
	//
size_t xReturn = 0U;
uint8_t val = 0;
uint8_t xBytes = 0;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);

	// Luego indicamos el periferico i2c ( INA3221 ) en el cual queremos leer
	val = devAddress;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);

	// Luego indicamos la direccion ( el registro ) a partir de donde escribir: largo ( 1 bytes )
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
	// y direccion ( registro )
	val = regAddress;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);

	// Por ultimo escribimos xBytes.
	//xBytes = length;
	xBytes = 2;	// En los INA siempre son 2 bytes
	xReturn = FreeRTOS_write(&pdI2C, data, xBytes);

	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	if (xReturn != xBytes ) {
		return ( false );
	}

	return(true);

}
//------------------------------------------------------------------------------------
uint8_t INA3221_id2busaddr( uint8_t id )
{
	switch(id) {
	case 0:
		return(INA3221_ADDR_0);	// Canales 0,1,2
		break;
	case 1:
		return(INA3221_ADDR_1); // Canales 3,4,5
		break;
	default:
		return(99);
		break;

	}

	return(99);

}
//------------------------------------------------------------------------------------
