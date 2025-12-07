/*
 * power_supply_ll.h
 *
 *  Created on: Oct 22, 2025
 *      Author: ctrumpy
 */

#ifndef INC_POWER_SUPPLY_LL_H_
#define INC_POWER_SUPPLY_LL_H_

#include "473_AP33772S.h"
#include "snap_module.h"
#include <stdbool.h>
#include <string.h>

// Internal datastructure for PSU status tracking
typedef struct {
	// latest requested outputs
	float reqCurrentLim;
	float reqVoltage;
	// latest measured actual outputs
	float currentActual;
	float voltageActual;
	// board temperature
	float temp;
    union {
        struct {
            uint8_t selectedProfile : 4;	// selected PDO profile (0-13)
            uint8_t F5V_en         : 1;		// Fixed 5V Output Enabled
            uint8_t F3V_en         : 1;		// Fixed 3V Output Enabled
            uint8_t PD_en          : 1;		// Adjustable Voltage Enabled
            uint8_t reserved       : 1;  	// padding for future features
        };
        uint8_t flags;
    };
}PSUStatus;

typedef struct {
	float reqCurrentLim;
	float reqVoltage;
    union {
        struct {
            uint8_t selectedProfile : 4; 	// selected PDO profile (0-13)
            uint8_t F5V_en         : 1;		// Fixed 5V Output Enabled
            uint8_t F3V_en         : 1; 	// Fixed 3V Output Enabled
            uint8_t PD_en		   : 1;		// Adjustable Voltage Enabled
            uint8_t reserved       : 1;  	// padding for future features
        };
        uint8_t flags;
    };
}OutputCommand;


typedef struct {
	PSUStatus status;
	AP33772S_t* ap;		//Tracking active AP33772S_t struct
	bool usbConnected; //is the USB-C power cable plugged in?
} PSUObject;


void initPSUObject(PSUObject* psu, AP33772S_t* ap, I2C_HandleTypeDef *hi2c);

//////////////////////////////////////////////////////////////////////////////
//							PSU Health Check Task							//
//////////////////////////////////////////////////////////////////////////////
void PDHealthTask(void *argument);

//////////////////////////////////////////////////////////////////////////////
//								Command Interfacing 						//
//////////////////////////////////////////////////////////////////////////////
/** @brief Process command received from the mainboard */
void parsePDCommand(PSUObject* psu, CommandMessage* msg, CommandResponse* resp);


/* *
 * @brief Process command requesting info on basic proof of life
 *If not alive, resp contains a RESPONSE_FAIL value
*/
void processAliveCommand(PSUObject* psu, CommandMessage* msg, CommandResponse* resp);

/* *
 * @brief Process command requesting info on PSU capabilities
 * Response contains an array of power modes available for the user to select
*/
void processCapabsCommand(PSUObject* psu, CommandMessage* msg, CommandResponse* resp);
/** @brief generate data required for response to CapabsCommand */
void reportCapabs(PSUObject* psu, CommandResponse* resp);

/* *
 * @brief Process command requesting info on PSU status
 * Response contains the contents of PSUObject* psu's PSUStatus struct.
*/
void processStatusCommand(PSUObject* psu, CommandMessage* msg, CommandResponse* resp);
/** @brief generate data required for response to CapabsCommand */
void reportStatus(PSUObject* psu, CommandResponse* resp);

/** @brief generate data required for response to CapabsCommand */
void processPDOutputCommand(PSUObject* psu, CommandMessage* msg, CommandResponse* resp);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//											Low-Level Command Issuing												//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief enable/disable fixed 5V output*/
void setF5V(PSUObject* psu, uint8_t enabled);
/** @brief enable/disable the fixed 3V output*/
void setF3V(PSUObject* psu, uint8_t enabled);
/* *
 * @brief Set a new output configuration
 * If the selected PDO is a PPS/AVS profile, reqVoltage will be applied
 * If the selected PDO is a fixed profile, only reqCurrent will be applied,
 * reqVoltage will have no impact as the voltage is specified by the profile itself.
 *
 * @param AP33772S_t* ap : AP33772S to execute this command on
 * @param uint8_t pdo : desired PDO index to be used
 * @param float reqVoltage : The requested voltage to be output
 * @param float reqCurrent : The requested current limit to be applied
*/
bool commandSetPDO(AP33772S_t* ap, uint8_t pdo, float reqVoltage, float reqCurrent);
#endif /* INC_POWER_SUPPLY_LL_H_ */
