/*
 * snap_master.c
 *
 * Created on: Oct 10, 2025
 * Author:
 */

#include "snap_master.h"

#include <string.h>

#include "cmsis_os.h"
#include "main.h"
#include "snap_config.h"

/**
 * @brief Reset I2C peripheral to recover from error state
 * @param hi2c: Pointer to I2C handle
 */
static void I2C_Reset(I2C_HandleTypeDef* hi2c) {
    // Disable the I2C peripheral
    __HAL_I2C_DISABLE(hi2c);

    // Small delay to ensure peripheral is fully stopped
    HAL_Delay(1);

    // Re-enable the I2C peripheral
    __HAL_I2C_ENABLE(hi2c);

    // Reset the HAL state
    hi2c->State = HAL_I2C_STATE_READY;
}

int sendCommand(void* hi2c, uint16_t slave_addr, struct CommandMessage* cmd,
                struct CommandResponse* resp, uint32_t timeout) {
    if (hi2c == NULL || cmd == NULL || resp == NULL) {
        return SNAP_ERR_NULL_PARAM;
    }

    I2C_HandleTypeDef* i2c = (I2C_HandleTypeDef*)hi2c;
    HAL_StatusTypeDef status;

    uint8_t command[sizeof(CommandCode) + sizeof(uint16_t) + COMMAND_BODY_SIZE];
    memcpy(command, &cmd->code, sizeof(CommandCode));
    memcpy(&command[sizeof(CommandCode)], &cmd->data_len, sizeof(uint16_t));
    memcpy(&command[sizeof(CommandCode) + sizeof(uint16_t)], &cmd->data,
           COMMAND_BODY_SIZE);

    status =
        HAL_I2C_Master_Transmit(i2c, slave_addr << 1, command, sizeof(command), timeout);
    if (status != HAL_OK) {
        // Reset I2C peripheral to recover from error state
        I2C_Reset(i2c);
        return SNAP_ERR_I2C_TX;
    }

    // Receive response from slave
    int stat = receiveResponse(hi2c, slave_addr, resp, MAX_RESPONSE_SIZE, timeout);
    while (stat == SNAP_ERR_BUSY) {
        HAL_Delay(10);
        stat = receiveResponse(hi2c, slave_addr, resp, MAX_RESPONSE_SIZE, timeout);
    }
    return stat;
}

int receiveResponse(void* hi2c, uint16_t slave_addr, struct CommandResponse* resp,
                    uint16_t max_len, uint32_t timeout) {
    if (hi2c == NULL || resp == NULL) {
        return SNAP_ERR_NULL_PARAM;
    }

    I2C_HandleTypeDef* i2c = (I2C_HandleTypeDef*)hi2c;
    HAL_StatusTypeDef status;

    // First, read the response header (code + data_len)
    uint8_t header_buf[sizeof(ResponseCode) + sizeof(uint16_t) + MAX_RESPONSE_SIZE];
    status = HAL_I2C_Master_Receive(i2c, slave_addr << 1, header_buf, sizeof(header_buf),
                                    timeout);
    if (status != HAL_OK) {
        // Reset I2C peripheral to recover from error state
        I2C_Reset(i2c);
        return SNAP_ERR_I2C_RX;
    }

    // Parse header
    memcpy(&resp->code, header_buf, sizeof(ResponseCode));
    memcpy(&resp->data_len, &header_buf[sizeof(ResponseCode)], sizeof(uint16_t));
    memcpy(&resp->data, &header_buf[sizeof(ResponseCode) + sizeof(uint16_t)],
           MAX_RESPONSE_SIZE);

    // Check if response is BUSY
    if (resp->code == RESPONSE_BUSY) {
        return SNAP_ERR_BUSY;  // Caller should retry
    }

    return SNAP_OK;
}

int receiveSPIdata(void* hspi, uint16_t data_len, uint8_t* buffer) {
    if (hspi == NULL || buffer == NULL || data_len == 0) {
        return SNAP_ERR_NULL_PARAM;
    }

    SPI_HandleTypeDef* spi = (SPI_HandleTypeDef*)hspi;
    HAL_StatusTypeDef status;

    // Receive data_len bytes via SPI (runs clock for data_len cycles)
    status = HAL_SPI_Receive_DMA(spi, buffer, data_len);
    if (status != HAL_OK) {
        return SNAP_ERR_SPI_RX;
    }

    return SNAP_OK;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi->Instance == SPI1) {
        HAL_SPI_RxCpltCallback_Streaming(hspi);
    }
}
