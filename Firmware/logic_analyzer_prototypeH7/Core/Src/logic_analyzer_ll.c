/*
 * logic_analyzer_ll.c
 *
 *  Created on: Sep 29, 2025
 *      Author: Andre
 */

#include "logic_analyzer_ll.h"

// pointers to the user's handles and config, provided by Init.
TIM_HandleTypeDef* p_htim_logic = NULL;
DMA_HandleTypeDef* p_hdma_logic = NULL;
const Sampler_Config* p_la_config = NULL;

static uint32_t apb1_clock_hz = 0;  // Store APB1 clock frequency
static uint32_t sample_count = 0;    // Number of samples to capture

// continuous sampling vars
bool la_continuous_mode = false;

/**
 * @brief Sets the sampling rate for the logic analyzer.
 * @param sampling_rate_hz: Desired sampling frequency in Hz.
 * @retval HAL status.
 */
HAL_StatusTypeDef LogicAnalyzer_SetSamplingRate(uint32_t sampling_rate_hz) {
  if (p_htim_logic == NULL || sampling_rate_hz == 0 || sampling_rate_hz > apb1_clock_hz) {
      return HAL_ERROR;
  }

  // Calculate total division needed: APB1_CLK / ((PSC + 1) * (ARR + 1))
  uint32_t total_div = apb1_clock_hz / sampling_rate_hz;

  uint32_t psc_value;
  uint32_t arr_value;

  if (total_div <= 0x10000) {
      // Fits in ARR
      psc_value = 0;
      arr_value = total_div - 1;
  } else {
      // Use prescaler
      psc_value = (total_div / 0x10000);
      arr_value = (total_div / (psc_value + 1)) - 1;

      // Check if values exceed 16-bit range
      if (psc_value > 0xFFFF || arr_value > 0xFFFF) {
          return HAL_ERROR;
      }
  }

  // Stop timer if running
  if (p_htim_logic->State == HAL_TIM_STATE_BUSY) {
      HAL_TIM_Base_Stop(p_htim_logic);
  }

  // Update registers
  __HAL_TIM_SET_PRESCALER(p_htim_logic, psc_value);
  __HAL_TIM_SET_AUTORELOAD(p_htim_logic, arr_value);
  p_htim_logic->Instance->EGR = TIM_EGR_UG;  // Load prescaler immediately

  return HAL_OK;
}

/**
 * @brief This function simply stores the pointers to the handles that were
 * fully initialized by CubeMX. It performs no hardware configuration.
 */
HAL_StatusTypeDef LogicAnalyzer_Init(const Sampler_Config* config) {
    if (config == NULL || config->htim == NULL || config->hdma == NULL || 
        config->dma_buffer == NULL || config->apb_clock_hz == 0) {
        return HAL_ERROR;
    }

    // Store pointers to the already-initialized handles.
    p_la_config = config;
    p_htim_logic = config->htim;
    p_hdma_logic = config->hdma;
    apb1_clock_hz = config->apb_clock_hz;
    sample_count = config->dma_buffer_size; // Default to full buffer

    //register callbacks
    HAL_DMA_RegisterCallback(p_hdma_logic, HAL_DMA_XFER_CPLT_CB_ID, HAL_DMA_ConvCpltCallback);
	HAL_DMA_RegisterCallback(p_hdma_logic, HAL_DMA_XFER_HALFCPLT_CB_ID, HAL_DMA_ConvHalfCpltCallback);
	HAL_DMA_RegisterCallback(p_hdma_logic, HAL_DMA_XFER_ERROR_CB_ID, HAL_DMA_ErrorCallback);

    return HAL_OK;
}

/**
 * @brief Starts the sampling process using the pre-configured handles.
 */
HAL_StatusTypeDef LogicAnalyzer_Start(void) {
    if (p_la_config == NULL || p_htim_logic == NULL || p_hdma_logic == NULL) {
        return HAL_ERROR;
    }

    uint32_t src_address = (uint32_t)&p_la_config->gpio_port->IDR;
    uint32_t dest_address = (uint32_t)p_la_config->dma_buffer;

    HAL_StatusTypeDef dma_status = HAL_DMA_Start_IT(p_hdma_logic, src_address, dest_address, sample_count);
    if (dma_status != HAL_OK) {
        return HAL_ERROR;
    }

    // Enable the DMA request source on the timer.
    __HAL_TIM_ENABLE_DMA(p_htim_logic, TIM_DMA_UPDATE);

    // Start the timer to begin generating triggers.
    HAL_StatusTypeDef tim_status = HAL_TIM_Base_Start(p_htim_logic);
    return tim_status;
}

/**
 * @brief Stops the sampling process.
 */
HAL_StatusTypeDef LogicAnalyzer_Stop(void) {
    if (p_htim_logic == NULL || p_hdma_logic == NULL) {
        return HAL_ERROR;
    }

    // Stop the timer first to halt triggers.
    if (HAL_TIM_Base_Stop(p_htim_logic) != HAL_OK) {
        return HAL_ERROR;
    }

    // Disable the DMA request source.
    __HAL_TIM_DISABLE_DMA(p_htim_logic, TIM_DMA_UPDATE);

    // reset to normal mode
    if(la_continuous_mode){
    	la_continuous_mode = false;
    	p_hdma_logic->Init.Mode = DMA_NORMAL;
    	HAL_DMA_Init(p_hdma_logic);
    }

    // Abort the DMA transfer.
    return HAL_DMA_Abort_IT(p_hdma_logic);
}

/**
 * @brief Sets the number of samples to capture.
 * @param sample_count: Number of samples to capture. Must be <= buffer size.
 * @retval HAL status.
 */
HAL_StatusTypeDef LogicAnalyzer_SetSampleCount(uint32_t new_sample_count) {
    if (p_la_config == NULL || new_sample_count == 0 || new_sample_count > p_la_config->dma_buffer_size) {
        return HAL_ERROR;
    }

    // Can't change sample count while running
    if (p_htim_logic->State == HAL_TIM_STATE_BUSY) {
        return HAL_ERROR;
    }

    sample_count = new_sample_count;
    return HAL_OK;
}

/**
 * @brief Gets the current configured sample count.
 * @return The number of samples configured to be captured.
 */
uint32_t LogicAnalyzer_GetSampleCount(void) {
    return sample_count;
}

HAL_StatusTypeDef LogicAnalyzer_StartContinuous(void){
	if(p_la_config == NULL || p_htim_logic == NULL || p_hdma_logic == NULL){
		return HAL_ERROR;
	}

	uint32_t src_address = (uint32_t)&p_la_config->gpio_port->IDR;
	uint32_t dest_address = (uint32_t)p_la_config->dma_buffer;

	// enable circular mode for dma
	p_hdma_logic->Init.Mode = DMA_CIRCULAR;
	if(HAL_DMA_Init(p_hdma_logic) != HAL_OK){
		return HAL_ERROR;
	}

	la_continuous_mode = true;
	buffer_half_full = false;
	buffer_full = false;
	chunk_counter = 0;
	current_chunk = -1;
	missed_chunks = 0;

	// start dma
	if(HAL_DMA_Start_IT(p_hdma_logic, src_address, dest_address, p_la_config->dma_buffer_size) != HAL_OK){
		return HAL_ERROR;
	}

	// start timer
	__HAL_TIM_ENABLE_DMA(p_htim_logic, TIM_DMA_UPDATE);
	return HAL_TIM_Base_Start(p_htim_logic);
}

bool LogicAnalyzer_IsContinuousMode(void){
	return la_continuous_mode;
}

uint8_t* LogicAnalyzer_GetHalfBufferPointer(uint8_t half){
	if(p_la_config == NULL || half > 1) {
		return NULL;
	}

	uint32_t half_size = p_la_config->dma_buffer_size / 2;
	uint32_t offset = (half == 1) ? half_size : 0;

	return &p_la_config->dma_buffer[offset];
}

uint32_t LogicAnalyzer_GetHalfBufferSize(void){
	if(p_la_config == NULL){
		return 0;
	}
	return p_la_config->dma_buffer_size / 2;
}
