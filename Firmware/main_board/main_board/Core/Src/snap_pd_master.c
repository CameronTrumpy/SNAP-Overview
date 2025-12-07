/*
 * snap_pd_master.c
 *
 *  Created on: Nov 6, 2025
 *      Author: Andre
 */

#include "snap_pd_master.h"
#include "snap_master.h"
#include <string.h>


int pd_alive(bool* usb_connected){
	// Acquire I2C mutex before accessing I2C peripheral
	if (osMutexAcquire(i2cMutexHandle, osWaitForever) != osOK) {
		return -1;  // Mutex acquisition failed
	}

	uint8_t cmd_buffer[sizeof(struct CommandMessage) + COMMAND_BODY_SIZE];
	struct CommandMessage* cmd = (struct CommandMessage*)cmd_buffer;

	cmd->code = COMMAND_PD_ALIVE;
	cmd->data_len = 0;

	uint8_t resp_buffer[sizeof(struct CommandResponse) + COMMAND_BODY_SIZE] = {0};
	struct CommandResponse* resp = (struct CommandResponse*)resp_buffer;

	int result = sendCommand(&hi2c1, SLAVE_ADDR_WAVPD, cmd, resp, TEST_TIMEOUT);

	if(result == SNAP_OK){
		*usb_connected = (resp->code == RESPONSE_SUCCESS);
	}

	// Release I2C mutex
	osMutexRelease(i2cMutexHandle);

	return result;
}


int pd_query_capabs(PSUCapabilities* capabs_out){
	if (capabs_out == NULL) {
		return SNAP_ERR_NULL_PARAM;
	}

	// Acquire I2C mutex before accessing I2C peripheral
	if (osMutexAcquire(i2cMutexHandle, osWaitForever) != osOK) {
		return SNAP_ERR_BUSY;
	}

	uint8_t cmd_buffer[sizeof(struct CommandMessage) + COMMAND_BODY_SIZE];
	struct CommandMessage* cmd = (struct CommandMessage*)cmd_buffer;

	cmd->code = COMMAND_PD_QUERY_CAPABS;
	cmd->data_len = 0;

	uint8_t resp_buffer[sizeof(struct CommandResponse) + COMMAND_BODY_SIZE] = {0};
	struct CommandResponse* resp = (struct CommandResponse*)resp_buffer;

	int result = sendCommand(&hi2c1, SLAVE_ADDR_WAVPD, cmd, resp, TEST_TIMEOUT);

	// Copy PDO array from response
	if (result == SNAP_OK && resp->data_len > 0) {
		size_t copy_size = (resp->data_len < sizeof(capabs_out->pdo_entries))
						  ? resp->data_len
						  : sizeof(capabs_out->pdo_entries);
		memcpy(capabs_out->pdo_entries, resp->data, copy_size);
		capabs_out->num_entries = resp->data_len / sizeof(SRC_SPRandEPR_PDO_Fields);
	} else {
		capabs_out->num_entries = 0;
	}

	// Release I2C mutex
	osMutexRelease(i2cMutexHandle);

	return result;
}


int pd_query_status(PSUStatus* status_out){
	// Acquire I2C mutex before accessing I2C peripheral
	if (osMutexAcquire(i2cMutexHandle, osWaitForever) != osOK) {
		return -1;  // Mutex acquisition failed
	}

	uint8_t cmd_buffer[sizeof(struct CommandMessage) + COMMAND_BODY_SIZE];
	struct CommandMessage* cmd = (struct CommandMessage*)cmd_buffer;

	cmd->code = COMMAND_PD_QUERY_STATUS;
	cmd->data_len = 0;

	uint8_t resp_buffer[sizeof(struct CommandResponse) + COMMAND_BODY_SIZE] = {0};
	struct CommandResponse* resp = (struct CommandResponse*)resp_buffer;

	int result = sendCommand(&hi2c1, SLAVE_ADDR_WAVPD, cmd, resp, TEST_TIMEOUT);

	// Release I2C mutex
	osMutexRelease(i2cMutexHandle);

	memcpy(status_out, resp->data, sizeof(PSUStatus));

	return result;
}


int pd_request_output(PSUOutputCommand* output){
	// Acquire I2C mutex before accessing I2C peripheral
	if (osMutexAcquire(i2cMutexHandle, osWaitForever) != osOK) {
		return -1;  // Mutex acquisition failed
	}

	uint8_t cmd_buffer[sizeof(struct CommandMessage) + COMMAND_BODY_SIZE];
	struct CommandMessage* cmd = (struct CommandMessage*)cmd_buffer;

	cmd->code = COMMAND_PD_REQUEST_OUTPUT;
	cmd->data_len = sizeof(PSUOutputCommand);
	memcpy(cmd->data, output, sizeof(PSUOutputCommand));

	uint8_t resp_buffer[sizeof(struct CommandResponse) + COMMAND_BODY_SIZE] = {0};
	struct CommandResponse* resp = (struct CommandResponse*)resp_buffer;

	int result = sendCommand(&hi2c1, SLAVE_ADDR_WAVPD, cmd, resp, TEST_TIMEOUT);

	// Release I2C mutex
	osMutexRelease(i2cMutexHandle);

	return result;
}
