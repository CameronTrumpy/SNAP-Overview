/*
 * la_os_shared.h
 *
 *  Created on: Nov 12, 2025
 *      Author: Andre
 */

#ifndef INC_LA_OS_SHARED_H_
#define INC_LA_OS_SHARED_H_

#include "main.h"
#include "FreeRTOS.h"
#include "snap_module.h"
#include "cmsis_os.h"
#include "stdint.h"
#include "stdbool.h"

#define SAMPLE_BUFFER_SIZE UINT16_MAX // 65kb

/**
 * @brief Configuration structure.
 * All HAL handles are expected to be initialized by CubeMX before calling LogicAnalyzer_Init.
 */
typedef struct {
    // --- HAL Handles (Initialized by CubeMX) ---
    TIM_HandleTypeDef* htim;           // Pointer to the timer handle (e.g., &htim2)
    DMA_HandleTypeDef* hdma;           // Pointer to the DMA handle (e.g., &hdma_tim2_up)
    GPIO_TypeDef* gpio_port;      // GPIO port to sample (e.g., GPIOA)

    // --- Data Buffer Configuration ---
    uint8_t* dma_buffer;     // Pointer to the destination buffer for 8-bit samples.
    uint32_t dma_buffer_size;// The total size of the buffer (number of samples).

    // --- Clock Configuration ---
    uint32_t            apb_clock_hz;  // APB bus clock frequency in Hz.

    // --- RTOS Integration ---
    osThreadId_t        task_to_notify; // Handle of the task to notify when data is ready.

} Sampler_Config;


//Shared vars
extern volatile bool buffer_half_full;
extern volatile bool buffer_full;
extern volatile uint32_t chunk_counter;      // Total chunks produced
extern volatile int8_t current_chunk;       // Current ready chunk
extern volatile uint32_t missed_chunks;

extern bool spiTransferActive;
extern SPI_HandleTypeDef hspi1;
extern uint64_t spi_data_to_send;
extern SPI_HandleTypeDef hspi1;

extern volatile bool la_continuous_active;
extern volatile bool os_continuous_active;

// Request/Response structs for getChunk command
struct GetChunkResponse {
    uint32_t chunk_number;     // Sequential chunk ID
    uint32_t samples_available;
    uint32_t missed_chunks;
};


// LA vars
extern bool la_continuous_mode;
extern TIM_HandleTypeDef* p_htim_logic;
extern DMA_HandleTypeDef* p_hdma_logic;
extern const Sampler_Config* p_la_config;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_up;



// Oscilloscope vars
extern const Sampler_Config* p_os_config;
extern TIM_HandleTypeDef* p_htim_os;
extern DMA_HandleTypeDef* p_hdma_os;
extern TIM_HandleTypeDef htim15;
extern DMA_HandleTypeDef hdma_tim15_up;

/**
 * @brief DMA full transfer complete callback
 * @param hdma: DMA handle
 */
void HAL_DMA_ConvCpltCallback(DMA_HandleTypeDef *hdma);

/**
 * @brief DMA half transfer complete callback
 * @param hdma: DMA handle
 */
void HAL_DMA_ConvHalfCpltCallback(DMA_HandleTypeDef *hdma);

/**
 * @brief DMA error callback
 * @param hdma: DMA handle
 */
void HAL_DMA_ErrorCallback(DMA_HandleTypeDef *hdma);

/**
 * @brief Checks if the first half of the buffer is ready for reading.
 * @note Reading the status automatically clears it.
 * @return true if first half of buffer contains new data, false otherwise.
 */
int IsHalfBufferReady(void);

/**
 * @brief Checks if the second half of the buffer is ready for reading.
 * @note Reading the status automatically clears it.
 * @return true if second half of buffer contains new data, false otherwise.
 */
int IsFullBufferReady(void);

/**
 * @brief get the next ready chunk number
 * @return returns the chunk numbre of the currently ready chunk. -1 on error
 */
int32_t GetReadyChunk(void);

/**
 * @brief mark the current chunk as consumed
 */
void ConsumeChunk(void);

/**
 * @brief get the number of missed chunks because of overflow
 */
uint32_t GetMissedChunks(void);

/**
 * @brief set the number of missed chunks to 0
 */
void ResetMissedChunks(void);

/**
 * @brief Internal function to handle new chunk availability
 * called by continuous sampling task when DMA callback happens
 */
void HandleNewChunk(uint8_t half);

// Task to handle 
void ContinuousSamplingTask(void *argument);

#endif /* INC_LA_OS_SHARED_H_ */
