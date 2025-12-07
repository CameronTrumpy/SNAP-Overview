/*
 * oscilloscope_ll.h
 *
 *  Created on: Nov 12, 2025
 *      Author: Andre
 */

#ifndef INC_OSCILLOSCOPE_LL_H_
#define INC_OSCILLOSCOPE_LL_H_

#include "la_os_shared.h"
/**
 * @brief Initializes the oscilloscope driver with pre-configured handles.
 * @param config: Pointer to the configuration structure.
 * @retval HAL status.
 */
HAL_StatusTypeDef Oscilloscope_Init(const Sampler_Config* config);

/**
 * @brief Sets the sampling rate for the oscilloscope.
 * @param sampling_rate_hz: Desired sampling frequency in Hz.
 * @retval HAL status.
 */
HAL_StatusTypeDef Oscilloscope_SetSamplingRate(uint32_t sampling_rate_hz);

/**
 * @brief Starts the sampling process using the pre-configured handles.
 */
HAL_StatusTypeDef Oscilloscope_Start(void);

/**
 * @brief Stops the GPIO sampling process.
 * @retval HAL status.
 */
HAL_StatusTypeDef Oscilloscope_Stop(void);

/**
 * @brief Sets the number of samples to capture.
 * @param sample_count: Number of samples to capture. Must be <= buffer size / sizeof(uint16_t).
 * @retval HAL status.
 */
HAL_StatusTypeDef Oscilloscope_SetSampleCount(uint32_t new_sample_count);

/**
 * @brief Gets the current configured sample count
 * @return the number of samples to be captured
 */
uint32_t Oscilloscope_GetSampleCount(void);

HAL_StatusTypeDef Oscilloscope_StartContinuous(void);

bool Oscilloscope_IsContinuousMode(void);

uint8_t* Oscilloscope_GetHalfBufferPointer(uint8_t half);

uint32_t Oscilloscope_GetHalfBufferSize(void);


#endif /* INC_OSCILLOSCOPE_LL_H_ */
