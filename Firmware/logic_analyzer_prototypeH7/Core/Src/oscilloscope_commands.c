/*
 * oscilloscope_commands.c
 *
 *  Created on: Nov 14, 2025
 *      Author: Andre
 */

#include "snap_module.h"
#include "oscilloscope_commands.h"
#include "main.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "oscilloscope_ll.h"
#include "stdbool.h"
#include "la_os_shared.h"
#include "logic_analyzer_ll.h"


void commandOSStartContinuous(struct CommandMessage *cmd, struct CommandResponse *resp){
	LogicAnalyzer_Stop();
	la_continuous_active = false;


	// reset state
	ResetMissedChunks();
	os_continuous_active = true;

	HAL_StatusTypeDef status = Oscilloscope_StartContinuous();

	if(status == HAL_OK){
		resp->code = RESPONSE_SUCCESS;
	}
	else{
		resp->code = RESPONSE_FAIL;
		os_continuous_active = false;
	}
	resp->data_len = 0;
}

void commandOSStopContinuous(struct CommandMessage *cmd, struct CommandResponse* resp){
	os_continuous_active = false;
	Oscilloscope_Stop();

	// return total missed chunks
	uint32_t final_missed = GetMissedChunks();
	resp->code = RESPONSE_SUCCESS;
	resp->data_len = sizeof(uint32_t);
	memcpy(resp->data, &final_missed, sizeof(uint32_t));
}

void commandOSConfig(struct CommandMessage *cmd, struct CommandResponse *resp){
	uint32_t sampling_rate_hz;
	if(cmd->data_len != sizeof(uint32_t)){
		resp->code = RESPONSE_FAIL;
		resp->data_len = 0;
		return;
	}

	memcpy(&sampling_rate_hz, cmd->data, sizeof(uint32_t));
	HAL_StatusTypeDef status = Oscilloscope_SetSamplingRate(sampling_rate_hz);
	if(status != HAL_OK){
		resp->code = RESPONSE_FAIL;
		resp->data_len = 0;
		return;
	}
	resp->code = RESPONSE_SUCCESS;
	resp->data_len = 0;
	return;
}

void commandOSGetChunk(struct CommandMessage *cmd, struct CommandResponse *resp){
	struct GetChunkResponse *chunk_resp = (struct GetChunkResponse*)resp->data;

	// check if data is available
	int32_t ready_chunk = GetReadyChunk();
	if(ready_chunk < 0){
		resp->code = RESPONSE_SUCCESS;
		resp->data_len = sizeof(struct GetChunkResponse);
		// no samples available
		chunk_resp->chunk_number = 0;
		chunk_resp->samples_available = 0;
		return;
	}

	uint8_t buffer_half = ready_chunk & 1;
	uint8_t *buffer_ptr = Oscilloscope_GetHalfBufferPointer(buffer_half);
	uint32_t half_size = Oscilloscope_GetHalfBufferSize();

	if(buffer_ptr == NULL){
		resp->code = RESPONSE_FAIL;
		resp->data_len = 0;
		chunk_resp->chunk_number = 0;
		chunk_resp->samples_available = 0;
		return;
	}

	// send a full chunk worth of samples
	chunk_resp->chunk_number = ready_chunk;
	chunk_resp->samples_available = half_size;
	chunk_resp->missed_chunks = GetMissedChunks();

	resp->code = RESPONSE_SUCCESS;
	resp->data_len = sizeof(struct GetChunkResponse);

	// ready spi
	spi_data_to_send = half_size;
	spi_transfer_state = SPI_STATE_DATA_READY;

	// flag for watchdog
	spiTransferActive = true;

	HAL_SPI_Transmit_DMA(&hspi1, buffer_ptr, half_size);
	// self chip select
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
}


