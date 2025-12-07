/*
 * logic_analyzer_ll.h
 *
 *  Created on: Sep 29, 2025
 *      Author: Andre
 */

#ifndef INC_LOGIC_ANALYZER_LL_H_
#define INC_LOGIC_ANALYZER_LL_H_

#include "la_os_shared.h"

/**
 * @brief Initializes the logic analyzer driver with pre-configured handles.
 * @param config: Pointer to the configuration structure.
 * @retval HAL status.
 */
HAL_StatusTypeDef LogicAnalyzer_Init(const Sampler_Config* config);

/**
 * @brief Starts the GPIO sampling process.
 * @retval HAL status.
 */
HAL_StatusTypeDef LogicAnalyzer_Start(void);

/**
 * @brief Stops the GPIO sampling process.
 * @retval HAL status.
 */
HAL_StatusTypeDef LogicAnalyzer_Stop(void);

/**
 * @brief Sets the sampling rate for the logic analyzer.
 * @param sampling_rate_hz: Desired sampling frequency in Hz.
 * @retval HAL status.
 */
HAL_StatusTypeDef LogicAnalyzer_SetSamplingRate(uint32_t sampling_rate_hz);

/**
 * @brief Sets the number of samples to capture.
 * @param sample_count: Number of samples to capture. Must be <= buffer size.
 * @retval HAL status.
 */
HAL_StatusTypeDef LogicAnalyzer_SetSampleCount(uint32_t sample_count);

/**
 * @brief Gets the current configured sample count.
 * @return The number of samples configured to be captured.
 */
uint32_t LogicAnalyzer_GetSampleCount(void);

/**
 * @brief Starts continuous sampling on the logic analyzer
 * @return HAL_OK on success
 */
HAL_StatusTypeDef LogicAnalyzer_StartContinuous(void);

/**
 * @brief Check if we are sampling in Continuous Mode
 * @return true if continuous mode
 */
bool LogicAnalyzer_IsContinuousMode(void);

/**
 * @brief return a pointer to the proper half of the sample buffer
 * @param half 0 for first half 1 for second half
 * @return pointer
 */
uint8_t* LogicAnalyzer_GetHalfBufferPointer(uint8_t half);

/**
 * @brief get the size of half the buffer
 * @return the size of half the sample buffer
 */
uint32_t LogicAnalyzer_GetHalfBufferSize(void);


#endif /* INC_LOGIC_ANALYZER_LL_H_ */



