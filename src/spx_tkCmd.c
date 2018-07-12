/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */
#include <FRTOS-IO/FRTOS-CMD.h>
#include <spx.h>


//static char test_buffer[FF_RECD_SIZE];
//static char cmp_buffer[FF_RECD_SIZE];

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);

static void pv_cmd_rwEE(uint8_t cmd_mode );

#define WR_CMD 0
#define RD_CMD 1
//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdHelpFunction(void);
static void cmdClearScreen(void);
static void cmdResetFunction(void);
static void cmdWriteFunction(void);
static void cmdReadFunction(void);
static void cmdStatusFunction(void);
static void cmdConfigFunction(void);
static void cmdKillFunction(void);

#define WR_CMD 0
#define RD_CMD 1

#define WDG_CMD_TIMEOUT	60

//------------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

uint8_t c;

( void ) pvParameters;

	FRTOS_CMD_init();

	// Registro los comandos y los mapeo al cmd string.
	FRTOS_CMD_register( "cls\0", cmdClearScreen );
	FRTOS_CMD_register( "reset\0", cmdResetFunction);
	FRTOS_CMD_register( "write\0", cmdWriteFunction);
	FRTOS_CMD_register( "read\0", cmdReadFunction);
	FRTOS_CMD_register( "help\0", cmdHelpFunction );
	FRTOS_CMD_register( "status\0", cmdStatusFunction );
	FRTOS_CMD_register( "config\0", cmdConfigFunction );
	FRTOS_CMD_register( "kill\0", cmdKillFunction );
//	FRTOS_CMD_register( "test\0", cmdTestEEpromFunction );

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("starting tkCmd..\r\n\0"));
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );

	//FRTOS_CMD_regtest();
	// loop
	for( ;; )
	{

			c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
			// el read se bloquea 50ms. lo que genera la espera.
			while ( CMD_read( (char *)&c, 1 ) == 1 ) {
			//while ( frtos_read(fdUSB, &c, 1 ) == 1 ) {
				FRTOS_CMD_process(c);
			}
	}
}
//------------------------------------------------------------------------------------
static void cmdStatusFunction(void)
{

char aux_str[32];
uint8_t channel;
uint8_t pos;

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );

	// DlgId
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("dlgid: %s\r\n\0"), systemVars.dlgId );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SERVER
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Server:\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// APN
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  apn: %s\r\n\0"), systemVars.apn );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SERVER IP:SERVER PORT
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  server ip:port: %s:%s\r\n\0"), systemVars.server_ip_address,systemVars.server_tcp_port );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SERVER SCRIPT
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  server script: %s\r\n\0"), systemVars.serverScript );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SERVER PASSWD
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  passwd: %s\r\n\0"), systemVars.passwd );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// MODEM
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Modem:\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// DLG IP ADDRESS
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ip address: %s\r\n\0"), systemVars.dlg_ip_address);
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CSQ */
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  signalQ: csq=%d, dBm=%d\r\n\0"), systemVars.csq, systemVars.dbm );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );


	// CONFIG
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Config:\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	switch(systemVars.xbee) {
	case XBEE_OFF:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  xbee: off\r\n\0") );
		break;
	case XBEE_MASTER:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  xbee: master\r\n\0") );
		break;
	case XBEE_SLAVE:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  xbee: slave\r\n\0") );
		break;
	}
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	switch(systemVars.debug) {
	case DEBUG_NONE:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debug: none\r\n\0") );
		break;
	case DEBUG_GPRS:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debug: gprs\r\n\0") );
		break;
	case DEBUG_RANGEMETER:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debug: range\r\n\0") );
		break;
	case DEBUG_DIGITAL:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debug: digital\r\n\0") );
		break;
	}
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerPoll: [%d s]\r\n\0"),systemVars.timerPoll );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// PULSE WIDTH
	if ( systemVars.rangeMeter_enabled ) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  RangeMeter: ON\r\n"));
	} else {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  RangeMeter: OFF\r\n"));
	}
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// PWR SAVE:
	if ( systemVars.pwrSave.modo ==  modoPWRSAVE_OFF ) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  pwrsave=off\r\n\0"));
	} else {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  pwrsave=on start[%02d:%02d], end[%02d:%02d]\r\n\0"), systemVars.pwrSave.hora_start.hour, systemVars.pwrSave.hora_start.min, systemVars.pwrSave.hora_fin.hour, systemVars.pwrSave.hora_fin.min);
	}
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// OUTPUTS:
	switch( systemVars.outputs.modo ) {
	case OUT_OFF:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: OFF\r\n"));
		break;
	case OUT_CONSIGNA:
		switch( systemVars.outputs.consigna_aplicada ) {
		case CONSIGNA_DIURNA:
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: CONSIGNA [diurna] (c_dia=%02d:%02d, c_noche=%02d:%02d)\r\n"), systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
			break;
		case CONSIGNA_NOCTURNA:
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: CONSIGNA [nocturna] (c_dia=%02d:%02d, c_noche=%02d:%02d)\r\n"), systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
			break;
		default:
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: CONSIGNA [error] (c_dia=%02d:%02d, c_noche=%02d:%02d)\r\n"), systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
			break;
		}
		break;
	case OUT_NORMAL:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: NORMAL (out_A=%d, out_B=%d)\r\n"), systemVars.outputs.out_A, systemVars.outputs.out_B );
		break;
	default:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: ERROR(%d) (out_A=%d, out_B=%d)\r\n"), systemVars.outputs.modo, systemVars.outputs.out_A, systemVars.outputs.out_B );
		break;
	}
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Configuracion de canales analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		if ( systemVars.a_ch_modo[channel] == 'R') {
			pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  a%d(*)\0"),channel );
		} else {
			pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  a%d( )\0"),channel );
		}
		FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" [%d-%d mA/ %.02f,%.02f | %04d | %s]\r\n\0"), systemVars.imin[channel],systemVars.imax[channel],systemVars.mmin[channel],systemVars.mmax[channel], systemVars.coef_calibracion[channel], systemVars.an_ch_name[channel] );
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	// Configuracion de canales digitales
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		if ( systemVars.d_ch_modo[channel] == 'R') {
			pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  d%d(*)\0"),channel );
		} else {
			pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  d%d( )\0"),channel );
		}

		if ( systemVars.d_ch_type[channel] == 'C') {
			// Los canales de contadores de pulsos 'C' muestran el factor de conversion
			FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" [ C | %s | %.02f ]\r\n\0"), systemVars.d_ch_name[channel],systemVars.d_ch_magpp[channel] );
		} else {
			// Los canales de nivel solo muestran el nombre.
			FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" [ L | %s ]\r\n\0"), systemVars.d_ch_name[channel] );
		}
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	}


}
//-----------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
	// Resetea al micro prendiendo el watchdog

	FRTOS_CMD_makeArgv();

	cmdClearScreen();

	CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */


}
//------------------------------------------------------------------------------------
static void cmdWriteFunction(void)
{

	FRTOS_CMD_makeArgv();

	// EE
	// write ee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		pv_cmd_rwEE(WR_CMD);
		return;
	}

	// CMD NOT FOUND
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
//------------------------------------------------------------------------------------
static void cmdReadFunction(void)
{

	FRTOS_CMD_makeArgv();

	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		pv_cmd_rwEE(RD_CMD);
		return;
	}

	// CMD NOT FOUND
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;

}
//------------------------------------------------------------------------------------
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\x1B[2J\0"));
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );
}
//------------------------------------------------------------------------------------
static void cmdConfigFunction(void)
{

bool retS = false;


	pv_snprintfP_ERR();
	return;

}
//------------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

	FRTOS_CMD_makeArgv();

	// HELP WRITE
	if (!strcmp_P( strupr(argv[1]), PSTR("WRITE\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-write\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc YYMMDDhhmm\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  consigna {diurna|nocturna}, outputs {x,x}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee,nvmee,rtcram {pos} {string}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ina {id} conf {value}, sens12V {on|off}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  clrd {0|1}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  out sleep|reset|phase(A/B)|enable(A/B) {0|1}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  out pulse (A/B) (+/-) (ms)\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  out power {on|off}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  bt {on|off}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  gprs (pwr|sw|cts|dtr) {on|off}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  gprs cmd {atcmd}, redial\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  xbee (pwr|sleep|reset) {on|off}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  xbee msg\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pulse {on|off}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  alarm (secs)\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;

	}

	// HELP READ
	else if (!strcmp_P( strupr(argv[1]), PSTR("READ\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-read\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc, frame\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee,nvmww,rtcram {pos} {lenght}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ina {id} {conf|chXshv|chXbusv|mfid|dieid}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  memory\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  an {0..4}, battery\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  din\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  gprs (rsp,rts,dcd,ri)\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pulse\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;

	}

	// HELP RESET
	else if (!strcmp_P( strupr(argv[1]), PSTR("RESET\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-reset\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  memory {soft|hard}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  alarm\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;

	}

	// HELP CONFIG
	else if (!strcmp_P( strupr(argv[1]), PSTR("CONFIG\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-config\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  analog {0..4} aname imin imax mmin mmax\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  cfactor {ch} {coef}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  digital {0..3} type(L,C) dname magPP\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rangemeter {on|off}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  modo {analog|digital} {0..n} {local|remoto}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  xbee {off|master|slave}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  outputs {off}|{normal o0 o1}|{consigna hhmm_dia hhmm_noche}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerpoll, timerdial, dlgid {name}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pwrsave modo [{on|off}] [{hhmm1}, {hhmm2}]\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  apn, port, ip, script, passwd\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debug {none,gprs,digital,range}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  default\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  save\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;

	}

	// HELP KILL
	else if (!strcmp_P( strupr(argv[1]), PSTR("KILL\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-kill {data,digi,gprstx,gprsrx,outputs}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;

	} else {

		// HELP GENERAL
		memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
		CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );
		CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Available commands are:\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-cls\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-help\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-status\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-reset...\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-kill...\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-write...\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-read...\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-config...\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	}

//	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-test eeprom {pages}\r\n\0"));
//	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

}
//------------------------------------------------------------------------------------
static void cmdKillFunction(void)
{

	FRTOS_CMD_makeArgv();


	pv_snprintfP_OK();
	return;
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void )
{
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ok\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_ERR(void)
{
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("error\r\n\0"));
	CMD_write(  cmd_printfBuff, sizeof(cmd_printfBuff) );
}
//------------------------------------------------------------------------------------
static void pv_cmd_rwEE(uint8_t cmd_mode )
{

bool retS = false;
uint8_t length = 0;
char *p;

	// read ee {pos} {lenght}
	if ( cmd_mode == RD_CMD ) {
		memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
//		FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
		retS = EE_read( (uint32_t)(atol(argv[2])), cmd_printfBuff, (uint8_t)(atoi(argv[3]) ) );
//		res = EE_read( (uint32_t)(atol(s0)), s1, (uint8_t)(atoi(s2)) );
//		FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);
		if ( retS ) {
			// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
			FRTOS_snprintf_P( &cmd_printfBuff[atoi(argv[3])], sizeof(cmd_printfBuff),PSTR( "\r\n\0"));
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// write ee pos string
	if ( cmd_mode == WR_CMD ) {

		// Calculamos el largo del texto a escribir en la eeprom.
		p = argv[3];
		while (*p != 0) {
			p++;
			length++;
		}

//		frtos_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
		retS = EE_write( (uint32_t)(atol(argv[2])), argv[3], length );
//		retS = EE_write( (uint32_t)(atol(argv[2])), cmd_printfBuff, 32 );
//		FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

}
//------------------------------------------------------------------------------------
