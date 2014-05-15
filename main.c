/*

	Test code for using ufo daughter board as slave. Master is initialized but unused.
	Collect sensor data and sid's are mockups.

*/
#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "twi_slave_driver.h"
#include <string.h>
#include <util/delay.h>	

/*! Defining an example slave address. */
// this is the slave address of the device itself, not a slave it talks to
#define SLAVE_ADDRESS    0x55

// #define ARDUSLAVE        0x4F

/*! CPU speed 2MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED   2000000
#define BAUDRATE	 100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

/*! Defining number of bytes in local arrays. */
#define SENSOR_NVAL     2
#define DEBUG_NVAL	    4  // debug >= data
#define DATA_BYTES      4  // 4 = float, 2 = int16, 1 = byte

#define FIRMWARE_VERSION 0
#define HARDWARE_VERSION 0

/* Global variables */
TWI_Master_t twicMaster;    /*!< TWI master module. */
TWI_Slave_t twicSlave;      /*!< TWI slave module. */

float data_values[DEBUG_NVAL] = {-99.0, -99.0, -99.0, -99.0};
float null_value = -99.0;
uint8_t sensor_sid[SENSOR_NVAL] = {0x01, 0x02};
uint8_t debug_sid[DEBUG_NVAL] = {0x01, 0x02, 0x03, 0x04};

float * return_datum;

uint8_t data_ready = 0;

uint8_t command = 0;
uint8_t command_argument = 0;
uint8_t iSID = 0;

uint8_t matchSID(uint8_t SID){
	for (uint8_t iSID = 0; iSID<DEBUG_NVAL; iSID++){
		if(debug_sid[iSID] == SID) return iSID;
	}
	return -1;
}

void collectSensorData(void){
	data_ready = 0; 

	data_values[0] = null_value; 
	data_values[1] = null_value;
	data_values[2] = null_value;
	data_values[3] = null_value;

	_delay_ms(500);

	data_values[0] = 100.0; 
	data_values[1] = 200.0;
	data_values[2] = 300.0;
	data_values[3] = 400.0;

	data_ready = 1; 
}

// Our slave is always on port C
void TWIC_SlaveProcessData(void)
{

	uint8_t bufIndex = twicSlave.bytesReceived;
	command = twicSlave.receivedData[0];
	if (bufIndex>0) command_argument = twicSlave.receivedData[1];

	switch (command){
		case 0x10:
			// data ready query
			twicSlave.sendData[0] = data_ready;
			break;
		case 0x20:
			// send sensor value, regular or debug
			iSID = matchSID(command_argument);
			if (iSID>=0){
				return_datum = &data_values[iSID];			
			} else {
				return_datum = &null_value;
			}
			memcpy((void *)twicSlave.sendData, return_datum, sizeof(float));			
			break;
		case 0x30:
			// send sensor nval
			twicSlave.sendData[0] = SENSOR_NVAL;
			break;
		case 0x40:
			// send debug nval
			twicSlave.sendData[0] = DEBUG_NVAL;
			break;	
		case 0x50:
			// send sensor sid array
			memcpy((void *)twicSlave.sendData, sensor_sid, SENSOR_NVAL);
			break;	
		case 0x60:
			// send debug sid array
			memcpy((void *)twicSlave.sendData, debug_sid, DEBUG_NVAL);
			break;	
		case 0x70:
			// send data type (== number of bytes)
			twicSlave.sendData[0] = DATA_BYTES;
			break;
		case 0x80:
			// trigger new set of measurements
			data_ready = 0;
			collectSensorData();
			break;

	}
}


int main(void)
{
	// the setup
	data_ready = 0;
	for (uint8_t i=0; i<DEBUG_NVAL; i++){
		data_values[i] = null_value;
	}
	
	// When daughter talks to chips on board, it is port E
	// When daughter talks to arduino as slave, it is port C
	/* Initialize TWI master. */
	TWI_MasterInit(&twicMaster,
	              &TWIC,
	              TWI_MASTER_INTLVL_LO_gc,
	              TWI_BAUDSETTING);

	/* Initialize TWI slave. */
	TWI_SlaveInitializeDriver(&twicSlave, &TWIC, TWIC_SlaveProcessData);
	TWI_SlaveInitializeModule(&twicSlave,
	                          SLAVE_ADDRESS,
	                          TWI_SLAVE_INTLVL_LO_gc);

	/* Enable LO interrupt level. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();

	// collect measurements as master
	collectSensorData();

	// the loop
	while (1) {

	}
}

/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect){
	TWI_MasterInterruptHandler(&twicMaster);
}

/*! TWIC Slave Interrupt vector. */
ISR(TWIC_TWIS_vect){
	TWI_SlaveInterruptHandler(&twicSlave);
}
