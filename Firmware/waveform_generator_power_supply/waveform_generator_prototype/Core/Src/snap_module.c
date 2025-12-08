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

#define HEADER_SIZE sizeof(CommandMessage)
#define RESPONSE_HEADER_SIZE (sizeof(ResponseCode) + sizeof(uint16_t))
#define BODY_SIZE 512

// Buffers - fixed size with 512 byte body
uint8_t commandBuffer[HEADER_SIZE + BODY_SIZE];
uint8_t processingBuffer[HEADER_SIZE + BODY_SIZE]; // Buffer for command processing
uint8_t responseBuffer[RESPONSE_HEADER_SIZE + BODY_SIZE];

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3; // state machine watchdog

// Task handle for the command processing task
TaskHandle_t commandTaskHandle = NULL;

// Flags
volatile bool responseReady = false;

// start state machine watchdog
static inline void start_fsm_watchdog(void) {
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim3, 0); // Reset counter
    HAL_TIM_Base_Start_IT(&htim3); // Start timer with interrupt
}

// stop state machine watchdog
static inline void stop_fsm_watchdog(void) {
    HAL_TIM_Base_Stop_IT(&htim3); // Stop timer
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim3, 0); // Reset counter
}

// NOTE: needs to be called by void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) if htim->Instance == TIM3
void i2c_fsm_watchdog_callback(){
    // reset state machine
    responseReady = false;

    // reset I2C peripheral
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_Init(&hi2c1);
    HAL_I2C_EnableListen_IT(&hi2c1);

    // stop the watchdog
    stop_fsm_watchdog();
}

// I2C Callbacks
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t dir, uint16_t addr)
{
    if (hi2c->Instance == I2C1)
    {
        start_fsm_watchdog(); // transaction initiated, start watchdog

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
                CommandResponse *busy = (CommandResponse*)busyResp;
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
        responseReady = false; // reset state machine
        // Entire command received (header + body)
        stop_fsm_watchdog(); // stop watchdog, transaction complete

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
        stop_fsm_watchdog(); // stop watchdog, transaction complete
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

// API Implementation
CommandResponse* receiveCommand(CommandMessage *cmd) {
    if (cmd == NULL) return NULL;

    CommandResponse *response = (CommandResponse*)responseBuffer;
    processCommand(cmd, response);
    return response;
}

int issueCommandResponse(CommandResponse *resp, uint8_t *buf) {
    if (resp == NULL || buf == NULL) return -1;

    uint16_t total_size = RESPONSE_HEADER_SIZE + BODY_SIZE; // Always send full 512 bytes
    if (buf != (uint8_t*)resp) { //?
        memcpy(buf, resp, total_size);
    }
    return 0;
}

// Command processing task - create this in main
void CommandProcessingTask(void *argument)
{
    commandTaskHandle = xTaskGetCurrentTaskHandle();

    for(;;)
    {
    	// Wait for notification from ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        responseReady = false; // mark response not ready


        // Copy command to processing buffer first to avoid race condition
        // This allows I2C to receive the next command while we process this one
        memcpy(processingBuffer, commandBuffer, HEADER_SIZE + BODY_SIZE);

        // Process the command from the processing buffer (not the receive buffer)
        CommandMessage *cmd = (CommandMessage*)processingBuffer;
        CommandResponse *resp = receiveCommand(cmd);
        issueCommandResponse(resp, responseBuffer);

        // Signal response is ready
        responseReady = true;
    }
}

// Reset state machine on error
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {

        stop_fsm_watchdog(); // stop watchdog on error

        // Re-enable listening
        HAL_I2C_EnableListen_IT(hi2c);
    }
}
