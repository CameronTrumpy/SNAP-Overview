/*
* snap_module.c
*
* Created on: Oct 10, 2025
* Author: Ben Wolin, Ethan, Andre
*/

#include "snap_module.h"
#include "main.h"
#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

#define HEADER_SIZE sizeof(struct CommandMessage)
#define RESPONSE_HEADER_SIZE (sizeof(ResponseCode) + sizeof(uint16_t))
#define BODY_SIZE 512

// Buffers - fixed size with 512 byte body
uint8_t commandBuffer[HEADER_SIZE + BODY_SIZE];
uint8_t processingBuffer[HEADER_SIZE + BODY_SIZE]; // Buffer for command processing
uint8_t responseBuffer[RESPONSE_HEADER_SIZE + BODY_SIZE];

// Flags
volatile bool responseReady = false;
volatile bool spiTransferActive = false;
volatile bool i2cTransferActive = false;

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3; // state machine watchdog
extern SPI_HandleTypeDef hspi1;

// Task handle for the command processing task
TaskHandle_t commandTaskHandle = NULL;

#ifndef DISABLE_WATCHDOG
// Set watchdog timeout in milliseconds
// APB clock is 270MHz, timer prescaler and period will be calculated
void set_fsm_watchdog_timeout(uint32_t timeout_ms) {
    HAL_TIM_Base_Stop_IT(&htim3);
    NVIC_EnableIRQ(TIM3_IRQn);

    uint64_t ticks = (uint64_t)timeout_ms * 270000000ULL / 1000ULL;

    uint32_t prescaler = 26999;
    uint32_t period = (ticks / (prescaler + 1)) - 1;

    if (period > 0xFFFF) {
        period = 0xFFFF;
    }

    htim3.Instance->PSC = prescaler;
    htim3.Instance->ARR = period;
    htim3.Instance->CNT = 0;
    htim3.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
}

static inline void start_fsm_watchdog(void) {
    HAL_TIM_Base_Stop_IT(&htim3); // Stop first
    __HAL_TIM_SET_COUNTER(&htim3, 0); // Reset counter
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE); // Clear flag
    HAL_NVIC_ClearPendingIRQ(TIM3_IRQn); // Clear any pending interrupts
    HAL_TIM_Base_Start_IT(&htim3); // Start timer with interrupt
}

// stop state machine watchdog
void stop_fsm_watchdog(bool spi) {
// this happens from isr
    if (spi && i2cTransferActive) {
        return;
    }

    if (!spi && spiTransferActive) {
        return;
    }

    HAL_TIM_Base_Stop_IT(&htim3); // Stop timer
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim3, 0); // Reset counter
}

// NOTE: needs to be called by void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) if htim->Instance == TIM3
void i2c_fsm_watchdog_callback(){
    // stop the watchdog
    i2cTransferActive = false;
	stop_fsm_watchdog(STOP_WATCHDOG_SPI);

    // reset state machine
    responseReady = false;

    // Check if SPI transfer was active and reset if needed
    if (spiTransferActive) {
        HAL_SPI_Abort(&hspi1);
        HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
        spi_transfer_state = SPI_STATE_IDLE;
        spi_data_to_send = 0;
        spiTransferActive = false;
    }

    // reset SPI peripheral
	HAL_SPI_DeInit(&hspi1);
	HAL_SPI_Init(&hspi1);

    // reset I2C peripheral
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_Init(&hi2c1);
    HAL_I2C_EnableListen_IT(&hi2c1);
}
#else
// Empty stub functions when watchdog is disabled
static inline void start_fsm_watchdog(void) {}
static inline void stop_fsm_watchdog(bool spi) {}
void i2c_fsm_watchdog_callback() {}
void set_fsm_watchdog_timeout(uint32_t timeout_ms) {}
#endif // DISABLE_WATCHDOG

// I2C Callbacks
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t dir, uint16_t addr)
{
    if (hi2c->Instance == I2C1)
    {
        start_fsm_watchdog(); // transaction initiated, start watchdog
        i2cTransferActive = true;

        if (dir == I2C_DIRECTION_TRANSMIT)
        {
            // Master is sending data - receive entire packet (header + 512 byte body)
            HAL_I2C_Slave_Sequential_Receive_IT(hi2c, commandBuffer,
                                                HEADER_SIZE + BODY_SIZE,
                                                I2C_FIRST_AND_LAST_FRAME);
        }
        else
        {
            // Master is reading response
            if (responseReady) {
                // Send entire response (header + 512 byte body)
                HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, responseBuffer,
                                                     RESPONSE_HEADER_SIZE + BODY_SIZE,
                                                     I2C_FIRST_AND_LAST_FRAME);
            } else {
                // Response not ready, send busy code with 512 bytes
                uint8_t busyResp[RESPONSE_HEADER_SIZE + BODY_SIZE];
                struct CommandResponse *busy = (struct CommandResponse*)busyResp;
                busy->code = RESPONSE_BUSY;
                busy->data_len = 0;
                memset(&busyResp[RESPONSE_HEADER_SIZE], 0, BODY_SIZE); // Clear body
                HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, busyResp,
                                                     RESPONSE_HEADER_SIZE + BODY_SIZE,
                                                     I2C_FIRST_AND_LAST_FRAME);
            }
        }
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        // Entire command received (header + body)
        stop_fsm_watchdog(STOP_WATCHDOG_I2C); // stop watchdog, transaction complete

        i2cTransferActive = false;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(commandTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        // Entire response sent (header + body)
        stop_fsm_watchdog(STOP_WATCHDOG_I2C); // stop watchdog, transaction complete
        i2cTransferActive = false;
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

// API Implementation
struct CommandResponse* receiveCommand(struct CommandMessage *cmd) {
    if (cmd == NULL) return NULL;

    struct CommandResponse *response = (struct CommandResponse*)responseBuffer;
    processCommand(cmd, response);
    return response;
}

int issueCommandResponse(struct CommandResponse *resp, uint8_t *buf) {
    if (resp == NULL || buf == NULL) return -1;

    uint16_t total_size = RESPONSE_HEADER_SIZE + BODY_SIZE; // Always send full 512 bytes
    if (buf != (uint8_t*)resp) {
        memcpy(buf, resp, total_size);
    }
    return 0;
}

// Command processing task - create this in main
void CommandProcessingTask(void *argument)
{
    commandTaskHandle = xTaskGetCurrentTaskHandle();
    HAL_I2C_EnableListen_IT(&hi2c1);

    // Set timeout to 5 second
    // TODO: make this lower when not testing
    set_fsm_watchdog_timeout(200);

    for(;;)
    {
        // Wait for notification from ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        responseReady = false; // mark response not ready

        // Copy command to processing buffer first to avoid race condition
        // This allows I2C to receive the next command while we process this one
        memcpy(processingBuffer, commandBuffer, HEADER_SIZE + BODY_SIZE);

        // Process the command from the processing buffer (not the receive buffer)
        struct CommandMessage *cmd = (struct CommandMessage*)processingBuffer;
        struct CommandResponse *resp = receiveCommand(cmd);
        issueCommandResponse(resp, responseBuffer);

        // Signal response is ready
        responseReady = true;
    }
}

// Reset state machine on error
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        i2c_fsm_watchdog_callback();
    }
}

//SPI stuff
volatile SPI_TransferState_t spi_transfer_state = SPI_STATE_IDLE;
uint64_t spi_data_to_send = 0;

// generic spi error callback
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        i2c_fsm_watchdog_callback();
    }
}

