/*
 * power_supply_ll.c
 *
 *  Created on: Oct 22, 2025
 *      Author: camer
 */

#include "snap_module.h"
#include "473_AP33772S.h"
#include "power_supply_ll.h"
#include "cmsis_os.h"
#include "main.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//										PSU Object Initialization													//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initPSUObject(PSUObject* psu, AP33772S_t* ap, I2C_HandleTypeDef *hi2c){
	psu->ap = ap;

	// Initialize device
	AP33772S_Init(psu->ap, hi2c);

	//Connect to device and map PDO profile info
	AP33772S_Begin(psu->ap);

	//populate initial state of 3V/5V enable status.
	psu->status.F3V_en = HAL_GPIO_ReadPin(EN_3VO_GPIO_Port, EN_3VO_Pin);
	psu->status.F5V_en = HAL_GPIO_ReadPin(EN_3VO_GPIO_Port, EN_3VO_Pin);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//										Health Check Task															//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PDHealthTask(void *argument)
{
	TaskHandle_t commandTaskHandle = xTaskGetCurrentTaskHandle();
    PSUObject* psu = (PSUObject*) argument;

    for(;;)
    {
    	vTaskDelay(500); // Only check periodically

    	if(!psu->usbConnected){
			//retry initialization
    		AP33772S_Begin(psu->ap);
    	}

    	psu->usbConnected = AP33772S_CheckAlive(psu->ap);
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//										Command interfacing between Mainboard and Module							//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Parse command coming from the mainboard to the PD module
void parsePDCommand(PSUObject* psu, CommandMessage* msg, CommandResponse* resp){
	switch(msg->code){
	case COMMAND_PD_ALIVE:
		processAliveCommand(psu, msg, resp);
		break;
	case COMMAND_PD_QUERY_CAPABS:
		processCapabsCommand(psu, msg, resp);
		break;
	case COMMAND_PD_QUERY_STATUS:
		processStatusCommand(psu, msg, resp);
		break;
	case COMMAND_PD_REQUEST_OUTPUT:
		processPDOutputCommand(psu, msg, resp);
		break;
	default:
		break;
	};
}


// Process command requesting info on if a USB-C connector is plugged in.
void processAliveCommand(PSUObject* psu, CommandMessage* msg, CommandResponse* resp){
	resp->code = (psu->usbConnected == true) ? RESPONSE_SUCCESS : RESPONSE_FAIL;
	resp->data_len = 0;
}

// Process command requesting info on capabilities
void processCapabsCommand(PSUObject* psu, CommandMessage* msg, CommandResponse* resp){
	// check that the USB-C is actually available
	if(!psu->usbConnected){
		processAliveCommand(psu, msg, resp);
 		return;
	}
	AP33772S_Begin(psu->ap);

	reportCapabs(psu, resp);
}



void reportCapabs(PSUObject* psu, CommandResponse* resp){
	//Fill response buffer with the array of power modes available
	memcpy(resp->data, &psu->ap->SRC_SPRandEPRpdoArray, MAX_PDO_ENTRIES * sizeof(SRC_SPRandEPR_PDO_Fields));
	resp->data_len = MAX_PDO_ENTRIES * sizeof(SRC_SPRandEPR_PDO_Fields);
	resp->code = RESPONSE_SUCCESS;
}


// Process command requesting info on PSU status
void processStatusCommand(PSUObject* psu, CommandMessage* msg, CommandResponse* resp){
	if (!psu->usbConnected) {
		resp->code = RESPONSE_FAIL;
		resp->data_len = 0;
		return;
	}

	psu->status.reqCurrentLim = AP33772S_readREQCurrent(psu->ap);
	psu->status.reqVoltage = AP33772S_readREQVoltage(psu->ap);

	psu->status.currentActual = AP33772S_readCurrent(psu->ap);
	psu->status.voltageActual = AP33772S_readVoltage(psu->ap);
	psu->status.temp = (float)AP33772S_readTemp(psu->ap);

	reportStatus(psu, resp);
}

void reportStatus(PSUObject* psu, CommandResponse* resp){
	memcpy(resp->data, &psu->status, sizeof(PSUStatus));
	resp->data_len = sizeof(PSUStatus);
	resp->code = RESPONSE_SUCCESS;
}


// Process command requesting a new output configuration
void processPDOutputCommand(PSUObject* psu, CommandMessage* msg, CommandResponse* resp){

	OutputCommand cmd;
	memcpy(&cmd, msg->data, sizeof(OutputCommand));

	setF5V(psu, cmd.F5V_en);
	setF3V(psu, cmd.F3V_en);
	if(AP33772S_SetSYSTEM(psu->ap, cmd.PD_en) == true){
		psu->status.PD_en = cmd.PD_en;
	}

	if(commandSetPDO(psu->ap, cmd.selectedProfile, cmd.reqVoltage, cmd.reqCurrentLim) != true){
		resp->data_len = 0;
		resp->code = RESPONSE_INVALID;
		return;
	}

	resp->data_len = 0;
	resp->code = RESPONSE_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//											Low-Level Command Issuing												//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//enable/disable fixed 5V output
void setF5V(PSUObject* psu, uint8_t enabled){
	HAL_GPIO_WritePin(EN_5VO_GPIO_Port, EN_5VO_Pin, enabled);
	psu->status.F5V_en = enabled;
}
//enable/disable fixed 3V output
void setF3V(PSUObject* psu, uint8_t enabled){
	HAL_GPIO_WritePin(EN_3VO_GPIO_Port, EN_3VO_Pin, enabled);
	psu->status.F3V_en = enabled;
}



bool commandSetPDO(AP33772S_t* ap, uint8_t pdo, float reqVoltage, float reqCurrent){
	int rV = (int)(reqVoltage * 1000.0f);
	int rI = (int)(reqCurrent * 1000.0f);

	//check if we are attempting to use PPS, or a fixed profile
	if(pdo < 8 && ap->SRC_SPRandEPRpdoArray[pdo-1].pps.type == 1){
		//arbitrate which function to call based on selected PDO
		return AP33772S_SetPPSPDO(ap, pdo, rV, rI);
	} else if(pdo < 8 && ap->SRC_SPRandEPRpdoArray[pdo-1].pps.type == 0) {
		return AP33772S_SetFixedPDO(ap,pdo);
	}
	return false;
}
