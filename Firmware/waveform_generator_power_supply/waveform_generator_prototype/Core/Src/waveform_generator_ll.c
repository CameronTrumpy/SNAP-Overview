/*
 * waveform_generator_ll.c
 *
 *  Created on: Oct 5, 2025
 *      Author: Andre
 */

#include "waveform_generator_ll.h"
#include <string.h>
#include <assert.h>

#define ERROR 1
#define SUCCESS 0

uint16_t waveform_buffer[MAX_SAMPLES];

// External handles (declared in main.c)
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim2;

// Convert voltage to DAC value (accounts for external gain)
uint16_t voltage_to_dac(float voltage) {
    // Divide by gain to get the voltage needed at DAC output
    float dac_voltage = OUTPUT_OFFSET + (voltage / GAIN);

    if (dac_voltage < 0.0f) dac_voltage = 0.0f;
    if (dac_voltage > DAC_MAX_VOLTAGE) dac_voltage = DAC_MAX_VOLTAGE;
    return (uint16_t)(dac_voltage / DAC_MAX_VOLTAGE * (DAC_RESOLUTION - 1));
}

// Convert DAC value to voltage (accounts for external gain)
float dac_to_voltage(uint16_t dac_value) {
    float dac_voltage = (float)dac_value / (DAC_RESOLUTION - 1) * DAC_MAX_VOLTAGE;
    // Multiply by gain to get actual output voltage
    return (dac_voltage - OUTPUT_OFFSET) * GAIN;
}

void setBufSine(struct WaveForm *wave_in, uint32_t num_samples) {
	float phase = wave_in->sine.Phase;
	for (uint32_t i = 0; i < num_samples; i++) {
		float t = (float)i / num_samples;
		float voltage = wave_in->sine.Offset +
			(wave_in->sine.Amplitude) * sinf(2.0f * M_PI * t + phase);
		waveform_buffer[i] = voltage_to_dac(voltage);
	}
}

void setBufSquare(struct WaveForm *wave_in, uint32_t num_samples) {
	float phase = wave_in->square.Phase;
	for (uint32_t i = 0; i < num_samples; i++) {
		float t = (float)i / num_samples;
		float value = sinf(2.0f * M_PI * t + phase) >= 0 ? 1.0f : -1.0f;
		float voltage = wave_in->square.Offset +
			(wave_in->square.Amplitude) * value;
		waveform_buffer[i] = voltage_to_dac(voltage);
	}
}

void setBufTriangle(struct WaveForm *wave_in, uint32_t num_samples) {
	float phase = wave_in->triangle.Phase;
	for (uint32_t i = 0; i < num_samples; i++) {
		float t = (float)i / num_samples;
		float value = 2.0f * fabsf(2.0f * (t + phase/(2.0f*M_PI) - floorf(t + phase/(2.0f*M_PI) + 0.5f))) - 1.0f;
		float voltage = wave_in->triangle.Offset +
			(wave_in->triangle.Amplitude) * value;
		waveform_buffer[i] = voltage_to_dac(voltage);
	}
}

void setBufRamp(struct WaveForm *wave_in, uint32_t num_samples) {
	float phase = wave_in->ramp.Phase;
	bool upwards = wave_in->ramp.Upwards;
	for (uint32_t i = 0; i < num_samples; i++) {
		float t = (float)i / num_samples;
		float value = upwards ? t : 1.0f - t;
		float voltage = wave_in->ramp.Offset +
			(wave_in->ramp.Amplitude) * value;
		waveform_buffer[i] = voltage_to_dac(voltage);
	}
}

void setBufPulse(struct WaveForm *wave_in, uint32_t num_samples) {
	float phase = wave_in->pulse.Phase;
	float duty = wave_in->pulse.DutyCycle / 100.0f;
	for (uint32_t i = 0; i < num_samples; i++) {
		float t = (float)i / num_samples;
		float value = (fmodf(t + phase/(2.0f*M_PI), 1.0f) < duty) ? 1.0f : 0.0f;
		float voltage = wave_in->pulse.Offset +
			(wave_in->pulse.Amplitude) * value;
		waveform_buffer[i] = voltage_to_dac(voltage);
	}
}

int setDAC(struct WaveForm *wave_in) {
	// Determine number of samples and DAC rate
	float freq = 0.0f;
	switch (wave_in->type) {
		case WAVEFORM_SINE:
			freq = wave_in->sine.Frequency;
			break;
		case WAVEFORM_SQUARE:
			freq = wave_in->square.Frequency;
			break;
		case WAVEFORM_TRIANGLE:
			freq = wave_in->triangle.Frequency;
			break;
		case WAVEFORM_RAMP:
			freq = wave_in->ramp.Frequency;
			break;
		case WAVEFORM_PULSE:
			freq = wave_in->pulse.Frequency;
			break;
		default:
			return ERROR;
	}

	// we are bounded by the max output rate
	uint32_t num_samples = (uint32_t)(MAX_OUTPUT_RATE / freq);
	uint32_t dac_rate = MAX_OUTPUT_RATE;
	// we are also bounded by the amount of samples we can hold in memory
	if (num_samples > MAX_SAMPLES) {
		num_samples = MAX_SAMPLES;
		// adjust output frequency as needed
		dac_rate = num_samples * freq;
	}

	// Populate waveform_buffer
	switch (wave_in->type) {
		case WAVEFORM_SINE:
			setBufSine(wave_in, num_samples);
			break;
		case WAVEFORM_SQUARE:
			setBufSquare(wave_in, num_samples);
			break;
		case WAVEFORM_TRIANGLE:
			setBufTriangle(wave_in, num_samples);
			break;
		case WAVEFORM_RAMP:
			setBufRamp(wave_in, num_samples);
			break;
		case WAVEFORM_PULSE:
			setBufPulse(wave_in, num_samples);
			break;
		default:
			return ERROR;
	}

	// Configure timer
	// Calculate total division needed: APB1_CLK / ((PSC + 1) * (ARR + 1))
	uint32_t total_div = APB1_CLOCK / dac_rate;

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

	__HAL_TIM_SET_AUTORELOAD(&htim2, arr_value);
	__HAL_TIM_SET_PRESCALER(&htim2, psc_value);
	  htim2.Instance->EGR = TIM_EGR_UG;  // Load prescaler immediately

	// Stop any existing DAC DMA operation
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	HAL_TIM_Base_Stop(&htim2);

	// Start DMA and DAC
	HAL_StatusTypeDef status;
	status = HAL_DAC_Start_DMA(&hdac1,
							   DAC_CHANNEL_1,
							   (uint32_t*)waveform_buffer,
							   num_samples,
							   DAC_ALIGN_12B_R);
	if (status == HAL_ERROR) {
		Error_Handler();
	}

	else if (status == HAL_BUSY){
		return HAL_BUSY;
	}

	// Start timer
	status = HAL_TIM_Base_Start(&htim2);
	if (status != HAL_OK) {
		Error_Handler();
	}
	return SUCCESS;
}

void commandSetWaveform(CommandMessage *cmd, CommandResponse *resp){
	//assume the data in the cmd message is the actual waveform struct that we have to set
	struct WaveForm waveFormToSet;
	assert(cmd->data_len <= sizeof(waveFormToSet));
	memcpy(&waveFormToSet, cmd->data, cmd->data_len);

	//set the appropriate waveform
	int ret = setDAC(&waveFormToSet);

	//set response
	if(ret == HAL_BUSY){
		resp->code = RESPONSE_BUSY;
	}
	else{
		resp->code = ret ? RESPONSE_FAIL : RESPONSE_SUCCESS;
	}
	resp->data_len = 0;
}
