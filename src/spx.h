/*
 * spx.h
 *
 *  Created on: 20 de oct. de 2016
 *      Author: pablo
 */

#ifndef SRC_SPXR1_H_
#define SRC_SPXR1_H_

//------------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <avr/sleep.h>
#include <avr_compiler.h>
#include "limits.h"
#include <ctype.h>
#include <inttypes.h>

#include <FreeRTOS.h>
#include <list.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#include <FRTOS-IO/FRTOS_stdio.h>

#include <pmic_driver.h>
#include <TC_driver.h>
#include <wdt_driver.h>
#include <clksys_driver.h>


#include <l_iopines.h>
#include "frtos-io.h"

#include "l_eeprom.h"
//#include "l_uarts.h"

//------------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------------
#define SPX_FW_REV "0.0.20"
#define SPX_FW_DATE "@ 20180712"

#define SPX_HW_MODELO "TEST HW:xmega256A3B R1.0"
#define SPX_FTROS_VERSION "FW:FRTOS10 TICKLESS"

// El datalogger tiene 6 canales fisicos pero 5 disponibles
// ya que uno esta para monitorear la bateria.
#define NRO_ANALOG_CHANNELS		5
#define NRO_DIGITAL_CHANNELS	4

#define F_CPU (32000000UL)

//#define SYSMAINCLK 2
//#define SYSMAINCLK 8
#define SYSMAINCLK 32

#define CHAR32	32
#define CHAR64	64
#define CHAR128	128
#define CHAR256	256

#define tkCtl_STACK_SIZE		384
#define tkCmd_STACK_SIZE		384

#define tkCtl_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )

#define DLGID_LENGTH		7
#define PARAMNAME_LENGTH	5
#define IP_LENGTH			24
#define APN_LENGTH			32
#define PORT_LENGTH			7
#define SCRIPT_LENGTH		64
#define PASSWD_LENGTH		15
#define PARAMNAME_LENGTH	5

TaskHandle_t xHandle_idle, xHandle_tkCtl,xHandle_tkCmd;

char stdout_buff[CHAR64];

//------------------------------------------------------------------------------------
typedef enum { DEBUG_NONE = 0, DEBUG_GPRS, DEBUG_RANGEMETER, DEBUG_DIGITAL } t_debug;
typedef enum { OUT_OFF = 0, OUT_CONSIGNA, OUT_NORMAL } t_outputs;
typedef enum { CONSIGNA_DIURNA = 0, CONSIGNA_NOCTURNA } t_consigna_aplicada;
typedef enum { modoPWRSAVE_OFF = 0, modoPWRSAVE_ON } t_pwrSave;
typedef enum { XBEE_OFF = 0, XBEE_MASTER, XBEE_SLAVE } t_modoXbee;
//------------------------------------------------------------------------------------

// Estructura para manejar la hora de aplicar las consignas
typedef struct {
	uint8_t hour;
	uint8_t min;
} time_t;

typedef struct {
	uint8_t modo;
	time_t hora_start;
	time_t hora_fin;
} pwrsave_t;

// Estructura para manejar las OUTPUTS
typedef struct {
	uint8_t modo;
	uint8_t out_A;
	uint8_t out_B;
	time_t consigna_diurna;
	time_t consigna_nocturna;
	uint8_t consigna_aplicada;
} outputs_t;

// Estructura para manejar los canales ANALOGICOS
typedef struct {
	float mag_val[NRO_ANALOG_CHANNELS];
} st_analog_frame;

// Estructura para manejar los canales DIGITALES.
typedef struct {
	uint8_t level[NRO_DIGITAL_CHANNELS];
	uint16_t counter[NRO_DIGITAL_CHANNELS];
	float magnitud[NRO_DIGITAL_CHANNELS];
} st_digital_frame;

typedef struct {
	float analog_val[NRO_ANALOG_CHANNELS];
	float digital_val[NRO_DIGITAL_CHANNELS];
} st_remote_values;


typedef struct {
	// Variables de trabajo.

	char dlgId[DLGID_LENGTH];
	char apn[APN_LENGTH];
	char server_tcp_port[PORT_LENGTH];
	char server_ip_address[IP_LENGTH];
	char dlg_ip_address[IP_LENGTH];
	char serverScript[SCRIPT_LENGTH];
	char passwd[PASSWD_LENGTH];

	// Configuracion de Canales analogicos
	uint16_t coef_calibracion[NRO_ANALOG_CHANNELS];
	uint8_t imin[NRO_ANALOG_CHANNELS];	// Coeficientes de conversion de I->magnitud (presion)
	uint8_t imax[NRO_ANALOG_CHANNELS];
	float mmin[NRO_ANALOG_CHANNELS];
	float mmax[NRO_ANALOG_CHANNELS];
	char an_ch_name[NRO_ANALOG_CHANNELS][PARAMNAME_LENGTH];
	char a_ch_modo[NRO_ANALOG_CHANNELS];

	// Configuracion de canales digitales
	// Niveles logicos
	char d_ch_name[NRO_DIGITAL_CHANNELS][PARAMNAME_LENGTH];
	float d_ch_magpp[NRO_DIGITAL_CHANNELS];
	char d_ch_type[NRO_DIGITAL_CHANNELS];
	char d_ch_modo[NRO_DIGITAL_CHANNELS];

	uint16_t timerPoll;
	uint32_t timerDial;

	uint8_t csq;
	uint8_t dbm;
	t_debug debug;

	outputs_t outputs;

	pwrsave_t pwrSave;

	bool rangeMeter_enabled;

	t_modoXbee xbee;

	// El checksum DEBE ser el ultimo byte del systemVars !!!!
	uint8_t checksum;

} systemVarsType;

systemVarsType systemVars;

bool startTask;

char cmd_printfBuff[256];
//------------------------------------------------------------------------------------
// PROTOTIPOS
//------------------------------------------------------------------------------------
void tkCtl(void * pvParameters);
void tkCmd(void * pvParameters);

xSemaphoreHandle sem_SYSVars;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

// Utils
void u_configure_systemMainClock(void);
void u_configure_RTC32(void);
void initMCU(void);

void puebas_ring_buffer(void);
void puebas2_ring_buffer(void);
void puebas3_ring_buffer(void);
void UARTInit( void );

#endif /* SRC_SPXR1_H_ */
