/*
 * snap_master.h
 *
 *  Created on: Oct 10, 2025
 *      Author:
 */

#ifndef INC_SNAP_MASTER_H_
#define INC_SNAP_MASTER_H_

#include "cmsis_os.h"
#include "main.h"
#include "stdint.h"

// Simple error codes - negative values indicate the error type
#define SNAP_OK 0
#define SNAP_ERR_NULL_PARAM -1    // NULL parameter (hi2c/cmd/resp/buffer)
#define SNAP_ERR_I2C_TX -2        // I2C transmit failed
#define SNAP_ERR_I2C_RX -3        // I2C receive failed
#define SNAP_ERR_SPI_RX -4        // SPI receive failed
#define SNAP_ERR_BUSY -5          // Slave device busy (can retry)
#define SNAP_ERR_INVALID_SIZE -6  // Invalid data size
#define SNAP_ERR_CANCELLED -7     // Canceled operation
#define SNAP_ERR_TIMEOUT -9       // Operation timeout

#define SAMPLE_BUF_SIZE UINT16_MAX * 2

// I2C addresses
#define SLAVE_ADDR_WAVPD 0x2B
#define SLAVE_ADDR_LAOS 0x54

extern uint8_t sample_buffer[SAMPLE_BUF_SIZE];
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern osMutexId_t i2cMutexHandle;
extern osSemaphoreId_t spiSemaphoreHandle;

// Command and response definitions
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
    COMMAND_PD_ALIVE,  // Check if PD Module is connected & active
    COMMAND_PD_QUERY_CAPABS,
    COMMAND_PD_QUERY_STATUS,
    COMMAND_PD_REQUEST_OUTPUT,
    COMMAND_PING,
    COMMAND_USB_SPEED_TEST,  // Test USB throughput
} CommandCode;

typedef enum {
    DUMMY_RESPONSECODE,
    RESPONSE_BUSY,
    RESPONSE_SUCCESS,
    RESPONSE_FAIL,
    RESPONSE_INVALID
} ResponseCode;

struct CommandMessage {
    CommandCode code;
    uint16_t data_len;
    uint8_t data[];  // flexible array that has no size
} __attribute__((packed));

struct CommandResponse {
    ResponseCode code;
    uint16_t data_len;
    uint8_t data[];  // flexible array that has no size
} __attribute__((packed));

// Note: Mutex and hardware handles are declared above (lines 34-35)

// Send a command to a slave module and receive the response
// @param hi2c: I2C handle to use for communication
// @param slave_addr: I2C address of the slave device
// @param cmd: command message to send
// @param resp: buffer to store the response (must be large enough)
// @param timeout: timeout in milliseconds
// @return: 0 on success, negative on error
int sendCommand(void* hi2c, uint16_t slave_addr, struct CommandMessage* cmd,
                struct CommandResponse* resp, uint32_t timeout);

// Receive a response from a slave module (blocking)
// @param hi2c: I2C handle to use for communication
// @param slave_addr: I2C address of the slave device
// @param resp: buffer to store the response
// @param max_len: maximum size of response buffer
// @param timeout: timeout in milliseconds
// @return: 0 on success, negative on error
int receiveResponse(void* hi2c, uint16_t slave_addr, struct CommandResponse* resp,
                    uint16_t max_len, uint32_t timeout);

// Receive data via SPI by running clock for specified cycles
// @param hspi: SPI handle to use for communication
// @param data_len: number of bytes to receive
// @param buffer: buffer to store received data (must be at least data_len bytes)
// @return: 0 on success, negative on error
int receiveSPIdata(void* hspi, uint16_t data_len, uint8_t* buffer);

#endif /* INC_SNAP_MASTER_H_ */
