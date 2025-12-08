/*
 * la_os_shared.c
 *	Shared functionality between Oscilloscope and Logic Analyzer
 *
 *  Created on: Nov 12, 2025
 *      Author: Andre
 */

#include "la_os_shared.h"

volatile bool buffer_half_full = false;  // Indicates first half of buffer is ready
volatile bool buffer_full = false;       // Indicates second half of buffer is ready

volatile uint32_t chunk_counter = 0;  // Total chunks produced
volatile int8_t current_chunk = -1;   // Current ready chunk
volatile uint32_t missed_chunks = 0;

volatile bool la_continuous_active = false;
volatile bool os_continuous_active = false;

/**
 * @brief DMA full transfer complete callback
 * @param hdma: DMA handle
 */
void HAL_DMA_ConvCpltCallback(DMA_HandleTypeDef* hdma) {
    if (hdma->Instance == p_hdma_logic->Instance) {
        if (!la_continuous_mode) {
            LogicAnalyzer_Stop();
        }
        buffer_full = true;

        if (p_la_config && p_la_config->task_to_notify) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(p_la_config->task_to_notify,
                                   &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    } else if (hdma->Instance == p_hdma_os->Instance) {
        // one of logic analyzer/oscilloscope can work at a time
        buffer_full = true;

        if (p_os_config && p_os_config->task_to_notify) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(p_os_config->task_to_notify,
                                   &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * @brief DMA half transfer complete callback
 * @param hdma: DMA handle
 */
void HAL_DMA_ConvHalfCpltCallback(DMA_HandleTypeDef* hdma) {
    if (hdma->Instance == p_hdma_logic->Instance) {
        buffer_half_full = true;

        if (p_la_config && p_la_config->task_to_notify) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;

            vTaskNotifyGiveFromISR(p_la_config->task_to_notify,
                                   &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    } else if (hdma->Instance == p_hdma_os->Instance) {
        buffer_half_full = true;

        if (p_os_config && p_os_config->task_to_notify) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;

            vTaskNotifyGiveFromISR(p_os_config->task_to_notify,
                                   &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * @brief DMA error callback
 * @param hdma: DMA handle
 */
void HAL_DMA_ErrorCallback(DMA_HandleTypeDef* hdma) {
    if (hdma->Instance == p_hdma_logic->Instance) {
        LogicAnalyzer_Stop();
    } else if (hdma->Instance == p_hdma_os->Instance) {
        Oscilloscope_Stop();
    }
}

/**
 * @brief Checks if the first half of the buffer is ready for reading.
 * @note Reading the status automatically clears it.
 * @return true if first half of buffer contains new data, false otherwise.
 */
int IsHalfBufferReady(void) {
    bool status = buffer_half_full;
    buffer_half_full = false;
    return status;
}

/**
 * @brief Checks if the second half of the buffer is ready for reading.
 * @note Reading the status automatically clears it.
 * @return true if second half of buffer contains new data, false otherwise.
 */
int IsFullBufferReady(void) {
    bool status = buffer_full;
    buffer_full = false;
    return status;
}

int32_t GetReadyChunk(void) {
    if (current_chunk == -1) {
        return -1;
    }
    return (int32_t)(chunk_counter - 1);  // return the last produced chunk
}

void ConsumeChunk(void) { current_chunk = -1; }

uint32_t GetMissedChunks(void) { return missed_chunks; }

void ResetMissedChunks(void) { missed_chunks = 0; }

void HandleNewChunk(uint8_t half) {
    // check if previous chunk has been consumed
    if (current_chunk != -1) {
        missed_chunks++;
    }

    // mark new chunk as ready
    current_chunk = half;
    chunk_counter++;
}

// SPI transmission complete callback
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi->Instance == SPI1) {
        stop_fsm_watchdog(STOP_WATCHDOG_SPI);
        spi_transfer_state = SPI_STATE_COMPLETE;
        spi_data_to_send = 0;
        spiTransferActive = false;
        // Chip select high
        HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
        // Data successfully transmitted
        if (la_continuous_active || os_continuous_active) {
            ConsumeChunk();
        }
    }
}

// global sampling vars
Sampler_Config la_config;
Sampler_Config os_config;
extern uint8_t sample_buffer[SAMPLE_BUFFER_SIZE];

extern osThreadId_t continuousTaskHandle;

// Continuous Sampling Task
void ContinuousSamplingTask(void* argument) {
    // --- Initialize the Logic Analyzer ---
    la_config.htim = &htim2;
    la_config.hdma = &hdma_tim2_up;
    la_config.gpio_port = GPIOG;
    la_config.dma_buffer = sample_buffer;
    la_config.dma_buffer_size = SAMPLE_BUFFER_SIZE;
    la_config.task_to_notify = continuousTaskHandle;
    la_config.apb_clock_hz =
        275000000;  // H7 APB1 Timer clock (check CubeMX actual value)

    if (LogicAnalyzer_Init(&la_config) != HAL_OK) {
        // Logic Analyzer Init Failed!
        osThreadTerminate(NULL);
    }

    if (LogicAnalyzer_SetSamplingRate(1000000) != HAL_OK) {
        // Logic Analyzer 1MHz sample set Failed!
        osThreadTerminate(NULL);
    }

    if (LogicAnalyzer_SetSampleCount(SAMPLE_BUFFER_SIZE) != HAL_OK) {
        // Set Sample Count Failed!
        osThreadTerminate(NULL);
    }
    // END INIT

    // --- Initialize the Oscilloscope ---
    os_config.apb_clock_hz =
        275000000;  // H7 APB2 Timer clock (check CubeMX actual value)
    os_config.dma_buffer = sample_buffer;
    os_config.dma_buffer_size = SAMPLE_BUFFER_SIZE;
    os_config.gpio_port = GPIOF;
    os_config.hdma = &hdma_tim15_up;
    os_config.htim = &htim15;
    os_config.task_to_notify = continuousTaskHandle;

    if (Oscilloscope_Init(&os_config) != HAL_OK) {
        // Oscilloscope Init Failed!
        osThreadTerminate(NULL);
    }
    // END INIT

    for (;;) {
        // Wait for DMA callback notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Check which half is ready
        bool half_ready = IsHalfBufferReady();
        bool full_ready = IsFullBufferReady();

        if (!half_ready && !full_ready) {
            continue;  // Spurious wakeup
        }

        // Determine which half just completed
        uint8_t ready_half = full_ready ? 1 : 0;

        // Notify logic analyzer module about new chunk
        HandleNewChunk(ready_half);
    }
}
