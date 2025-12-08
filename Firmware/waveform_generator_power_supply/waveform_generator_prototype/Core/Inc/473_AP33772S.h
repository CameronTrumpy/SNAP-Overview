/*
 * 473_AP33772S.h
 *
 * Hardware Interface to the AP33772S PD Chip
 * Modified under GNU GPLv3.
 *
 *  Created on: Oct 7, 2025
 *      Author: cameron
 */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* This code utilizes reference material from https://github.com/CentyLab/AP33772S-CentyLab

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef INC_473_AP33772S_H_
#define INC_473_AP33772S_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define MAX_PDO_ENTRIES 13  // USB-C PD is limited to 6 EPR and 7 SPR Power Data Object (PDO) Entries

#define I2C_TIMEOUT 1000

#define AP33772S_ADDRESS (0x52 << 1)
#define READ_BUFF_LENGTH 128
#define WRITE_BUFF_LENGTH 6
#define SRCPDO_LENGTH 28

#define CMD_STATUS    0x01 //Reset to 0 after very Read
#define CMD_MASK      0x02
#define CMD_OPMODE    0x03
#define CMD_CONFIG    0x04
#define CMD_PDCONFIG  0x05
#define CMD_SYSTEM    0x06
// Temperature setting register
#define CMD_TR25     0x0C
#define CMD_TR50     0x0D
#define CMD_TR75     0x0E
#define CMD_TR100    0x0F

//Power reading related
#define CMD_VOLTAGE   0x11
#define CMD_CURRENT   0x12
#define CMD_TEMP      0x13
#define CMD_VREQ      0x14
#define CMD_IREQ      0x15

#define CMD_VSELMIN   0x16 //Minimum Selection Voltage
#define CMD_UVPTHR    0x17
#define CMD_OVPTHR    0x18
#define CMD_OCPTHR    0x19
#define CMD_OTPTHR    0x1A
#define CMD_DRTHR     0x1B

#define CMD_SRCPDO    0x20

#define CMD_PD_REQMSG 0x31
#define CMD_PD_CMDMSG 0x32
#define CMD_PD_MSGRLT 0x33

// PDO Type
#define PDO_FIXED 0

#define AP_DEAD 0
#define AP_ALIVE 1

typedef struct {
	union {
		struct {
			unsigned int voltage_max :8;   // Bits 7:0, VOLTAGE_MAX field
			unsigned int peak_current :2;  // Bits 9:8, PEAK_CURRENT field
			unsigned int current_max :4;   // Bits 13:10, CURRENT_MAX field
			unsigned int type :1;          // Bit 14, TYPE field
			unsigned int detect :1;        // Bit 15, DETECT field
		} fixed;
		struct {
			unsigned int voltage_max :8;   // Bits 7:0, VOLTAGE_MAX field
			unsigned int voltage_min :2;   // Bits 9:8, VOLTAGE_MIN field
			unsigned int current_max :4;   // Bits 13:10, CURRENT_MAX field
			unsigned int type :1;          // Bit 14, TYPE field
			unsigned int detect :1;        // Bit 15, DETECT field
		} pps;
		struct {
			unsigned int voltage_max :8;   // Bits 7:0, VOLTAGE_MAX field
			unsigned int voltage_min :2;   // Bits 9:8, VOLTAGE_MIN field
			unsigned int current_max :4;   // Bits 13:10, CURRENT_MAX field
			unsigned int type :1;          // Bit 14, TYPE field
			unsigned int detect :1;        // Bit 15, DETECT field
		} avs;
		struct {
			uint8_t byte0;
			uint8_t byte1;
		};
	};
	unsigned long data;
} SRC_SPRandEPR_PDO_Fields;


typedef struct {
	union {
		struct {
			unsigned int VOLTAGE_SEL :8;  // Bits 7:0, Output Voltage Select
			unsigned int CURRENT_SEL :4;  // Bits 11:8, Operating Current Select
			unsigned int PDO_INDEX :4;  // Bits 15:12, Source PDO index select

		} REQMSG_Fields;
		struct {
			uint8_t byte0;
			uint8_t byte1;
		};
		unsigned long data;
	};
} RDO_DATA_T;


typedef struct{
	I2C_HandleTypeDef *hi2c;
	uint8_t tgtAddr;						/* Next I2C register to access */
    uint8_t readBuf[READ_BUFF_LENGTH];		/* I2C read buffer */
    uint8_t writeBuf[WRITE_BUFF_LENGTH];	/* I2C write buffer */
    int indexPPSUser; // for getPPSIndex();
    int indexAVSUser; // for getAVSIndex();
    RDO_DATA_T rdoData;
    SRC_SPRandEPR_PDO_Fields SRC_SPRandEPRpdoArray[MAX_PDO_ENTRIES];
    uint8_t validPDOs;
} AP33772S_t;


/* -------------------------------------------------------------------------- */
/*                          Device Initialization							  */
/* -------------------------------------------------------------------------- */

/* *
 * @brief Clears memory and assigns an i2c handle for the device to use
 */
void AP33772S_Init(AP33772S_t *dev, I2C_HandleTypeDef *hi2c);
/* *
 * @brief Loads the initial handshake process with the charger
 * @return true on success, false on failure of any of the I2C transactions
*/
bool AP33772S_Begin(AP33772S_t *handle);
/* *
 * @brief Map retrieved information on supported PPS / AVS profiles to a fixed index
*/
void AP33772S_MapPPSAVSInfo(AP33772S_t *dev);

/* *
 * @brief Check if the device is alive via a read to the SYSTEM register
 * @return AP_ALIVE if I2C read succeeds, AP_DEAD if i2c read fails
*/
uint8_t AP33772S_CheckAlive(AP33772S_t *handle);


/**
 * @brief Request an updated set of PDOs from the device
*/
void AP33772S_ReadPDOs(AP33772S_t *dev);

/* -------------------------------------------------------------------------- */
/*                          Device Configuration							  */
/* -------------------------------------------------------------------------- */

/**
 * @brief configure the chip's internal TR curve to match the selected NTC thermistor
 */
void AP33772S_SetTRCurve();

/**
 * @brief enable/disable the output NMOS switch (0=off, 1=on), through the SYSTEM register
 * @return true if the operation succeeded
 * @return false if there is an error
 */
bool AP33772S_SetSYSTEM(AP33772S_t *dev, int VOUTCTL);


/* Adjustment functions */
bool AP33772S_SetFixedPDO(AP33772S_t *dev, int pdoIndex);
bool AP33772S_SetPPSPDO(AP33772S_t *dev, int pdoIndex, int target_mV, int max_mA);

/* -------------------------------------------------------------------------- */
/*                          Performance Parameter Monitoring				  */
/* -------------------------------------------------------------------------- */

/**
 * @brief read the last requested voltage
 * @return latest requested voltage in V, -1 if read fails
 */
float AP33772S_readREQVoltage(AP33772S_t *dev);

/**
 * @brief read the actual output voltage measured by the chip
 * @return latest voltage in V, -1 if read fails
 */
float AP33772S_readVoltage(AP33772S_t *handle);
/**
 * @brief read the last requested current Limit
 * @return latest requested current limit in A, -1 if read fails
 */
float AP33772S_readREQCurrent(AP33772S_t *dev);
/**
 * @brief read the actual output current measured by the chip
 * @return latest current in V, -1 if read fails
 */
float AP33772S_readCurrent(AP33772S_t *handle);

/**
 * @brief read the approximate board temp
 * @return temperature in deg. C, 255 if read fails
 */
uint8_t AP33772S_readTemp(AP33772S_t *handle);

/* -------------------------------------------------------------------------- */
/*                           Helpers										  */
/* -------------------------------------------------------------------------- */

/**
 * @brief take in current in mA unit
 * @return value from 0 to 15
 * @return -1 if there is an error
 */
int AP33772S_currentMap(int current);

/**
 * @brief take in current bin value
 * @return value in A
 * @return -1 if there is an error
 */
float AP33772S_currentMapRev(int currentIdx);

/**
 * @brief Internal helper to read a register from the AP33772S, over I2C
 *
 *  This requires that dev->tgtAddr has been populated with the desired read address
 *
 * @param AP33772S_t *dev: a pointer to the AP33772S device struct
 * @param uint16_t rdLen: the number of bytes to be read
 * @param uint32_t timeout: the number of HAL ticks to wait before timing out
 * @return HAL_StatusTypeDef indicating success or failure of the I2C transaction
 */
HAL_StatusTypeDef AP33772S_i2c_read(AP33772S_t *dev, uint16_t rdLen, uint32_t timeout);
/**
 * @brief Internal helper to write a register from the AP33772S, over I2C
 *
 * This requires that dev->tgtAddr and dev->writeBuf have
 * already been populated to with the desired values.
 *
 * @param AP33772S_t *dev: a pointer to the AP33772S device struct
 * @param uint16_t wrLen: the number of bytes to be written
 * @param uint32_t timeout: the number of HAL ticks to wait before timing out
 * @return HAL_StatusTypeDef indicating success or failure of the I2C transaction
 */
HAL_StatusTypeDef AP33772S_i2c_write(AP33772S_t *dev, uint16_t wrLen, uint32_t timeout);


#endif /* INC_473_AP33772S_H_ */
