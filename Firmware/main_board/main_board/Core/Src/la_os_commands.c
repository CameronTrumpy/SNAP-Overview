/*
 * la_os_commands.c
 *
 *  Created on: Nov 15, 2025
 *      Author: Andre
 */

#include "la_os_commands.h"
#include "snap_config.h"

// HELPER
int no_body_command(CommandCode code) {
	// select proper slave addr
	uint8_t SLAVE_ADDR;
	if(code == COMMAND_SET_WAVEFORM || (COMMAND_PD_ALIVE <= code && code <= COMMAND_PD_REQUEST_OUTPUT)){
		SLAVE_ADDR = SLAVE_ADDR_WAVPD;
	}
	else{
		SLAVE_ADDR = SLAVE_ADDR_LAOS;
	}

    // Acquire I2C mutex
    if (osMutexAcquire(i2cMutexHandle, osWaitForever) != osOK) {
        return -1;
    }

    uint8_t cmd_buffer[sizeof(struct CommandMessage) + COMMAND_BODY_SIZE];
    struct CommandMessage* cmd = (struct CommandMessage*)cmd_buffer;

    cmd->code = code;
    cmd->data_len = 0;

    uint8_t resp_buffer[sizeof(struct CommandResponse) + COMMAND_BODY_SIZE] = {0};
    struct CommandResponse* resp = (struct CommandResponse*)resp_buffer;

    int result = sendCommand(&hi2c1, SLAVE_ADDR, cmd, resp, TEST_TIMEOUT);

    osMutexRelease(i2cMutexHandle);

    return result;
}

// HELPER
int command_config(uint32_t freq, CommandCode command) {
	// config only happens for LA/OS
	const uint8_t SLAVE_ADDR = SLAVE_ADDR_LAOS;

    // Acquire I2C mutex before accessing I2C peripheral
    if (osMutexAcquire(i2cMutexHandle, osWaitForever) != osOK) {
        return -1;  // Mutex acquisition failed
    }

    uint8_t cmd_buffer[sizeof(struct CommandMessage) + COMMAND_BODY_SIZE];
    struct CommandMessage* cmd = (struct CommandMessage*)cmd_buffer;

    cmd->code = command;
    cmd->data_len = sizeof(uint32_t);
    memcpy(cmd->data, &freq, sizeof(freq));

    uint8_t resp_buffer[sizeof(struct CommandResponse) + COMMAND_BODY_SIZE] = {0};
    struct CommandResponse* resp = (struct CommandResponse*)resp_buffer;

    int result = sendCommand(&hi2c1, SLAVE_ADDR, cmd, resp, CONFIG_TIMEOUT);

    // Release I2C mutex
    osMutexRelease(i2cMutexHandle);

    return result;
}

// LA COMMANDS
int command_la_start_continuous(void) {
    return no_body_command(COMMAND_LA_START_CONTINUOUS);
}

int command_la_stop_continuous(void) {
    return no_body_command(COMMAND_LA_STOP_CONTINUOUS);
}

int command_la_config(uint32_t freq){
	return command_config(freq, COMMAND_LA_CONFIG);
}

int command_la_get_chunk(uint32_t num_chunks, uint8_t cdc_port){
	return command_get_chunk(num_chunks, cdc_port, COMMAND_LA_GET_CHUNK, NULL);
}
// END LA

// OS COMMANDS
int command_os_start_continuous(void){
	return no_body_command(COMMAND_OS_START_CONTINUOUS);
}

int command_os_stop_continuous(void){
	return no_body_command(COMMAND_OS_STOP_CONTINUOUS);
}

int command_os_config(uint32_t freq){
	return command_config(freq, COMMAND_OS_CONFIG);
}

int command_os_get_chunk(uint32_t num_chunks, uint8_t cdc_port){
	return command_get_chunk(num_chunks, cdc_port, COMMAND_OS_GET_CHUNK, NULL);
}
// END OS

