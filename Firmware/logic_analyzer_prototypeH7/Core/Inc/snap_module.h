/*
 * snap_module.h
 *
 *  Created on: Oct 10, 2025
 *      Author:
 */

#ifndef INC_SNAP_MODULE_H_
#define INC_SNAP_MODULE_H_

#include "stdint.h"
#include "stdbool.h"

//Interface common to all modules interfacing with the main board
typedef enum {
    DUMMY_COMMAND = 0,
    COMMAND_DATA_TRANSFER,
    COMMAND_SET_WAVEFORM,
    COMMAND_LA_START_CONTINUOUS,
    COMMAND_LA_STOP_CONTINUOUS,
    COMMAND_LA_GET_CHUNK,
    COMMAND_LA_CONFIG,
	COMMAND_OS_START_CONTINUOUS,
	COMMAND_OS_STOP_CONTINUOUS,
	COMMAND_OS_GET_CHUNK,
	COMMAND_OS_CONFIG,
	COMMAND_PD_ALIVE,			
	COMMAND_PD_QUERY_CAPABS,
	COMMAND_PD_QUERY_STATUS,
	COMMAND_PD_REQUEST_OUTPUT,
} CommandCode;

typedef enum {
    DUMMY_RESPONSECODE,
	RESPONSE_BUSY,
	RESPONSE_SUCCESS,
	RESPONSE_FAIL,
	RESPONSE_INVALID
} ResponseCode;

typedef enum {
    SPI_STATE_IDLE,
    SPI_STATE_DATA_READY,
    SPI_STATE_TRANSMITTING,
    SPI_STATE_COMPLETE
} SPI_TransferState_t;

extern volatile SPI_TransferState_t spi_transfer_state;
extern uint64_t spi_data_to_send;

/*
 Assuming I2C data is written into rxbuf databuffer

 CommandResponse* response = (CommandResponse*)rxbuf
 uint8_t *buff = response->data //data buffer
*/
struct CommandMessage{
    CommandCode code;
    uint16_t data_len;
    uint8_t  data[]; //flexible array that has no size
}__attribute__ ((packed));

struct CommandResponse{
    ResponseCode code;
    uint16_t data_len;
    uint8_t  data[]; //flexible array that has no size, feature of C99. This is a pointer to the first byte AFTER the CommandResponse buffer
}__attribute__ ((packed));


// Receive a command from the main board,
// parse/process the command
// construct a command response
// @param cmd: command message received.
// @return: the constructed response to be issued back to the main board.
struct CommandResponse* receiveCommand(struct CommandMessage *cmd);

// Issue a command response to the main board
// @param cmd: command response to the previously recieved command.
// @param buf: data buffer to send back to main board.
// @return: int indicating success or failure to send response
int issueCommandResponse(struct CommandResponse *cmd, uint8_t *buf);

// This should be specific to each module, can be implemented in main
void processCommand(struct CommandMessage *cmd, struct CommandResponse *resp);

// SPI and I2C watchdog
#define STOP_WATCHDOG_SPI true
#define STOP_WATCHDOG_I2C false
void stop_fsm_watchdog(bool spi);

#endif /* INC_SNAP_MODULE_H_ */
