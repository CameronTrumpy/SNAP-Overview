/*
 * logic_analyzer_commands.c
 *
 *  Created on: Oct 20, 2025
 *      Author: Andre
 */

#include "snap_module.h"
#include "logic_analyzer_commands.h"
#include "main.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "logic_analyzer_ll.h"
#include "stdbool.h"
#include "la_os_shared.h"
#include "oscilloscope_ll.h"


void commandLAStartContinuous(struct CommandMessage *cmd, struct CommandResponse *resp){
	Oscilloscope_Stop();
	os_continuous_active = false;

	// reset state
	ResetMissedChunks();
	la_continuous_active = true;

	HAL_StatusTypeDef status = LogicAnalyzer_StartContinuous();

	if(status == HAL_OK){
		resp->code = RESPONSE_SUCCESS;
		resp->data_len = 0;
	}
	else{
		resp->code = RESPONSE_FAIL;
		resp->data_len = 0;
		la_continuous_active = false;
	}
}

void commandLAStopContinuous(struct CommandMessage *cmd, struct CommandResponse *resp){
	la_continuous_active = false;
	LogicAnalyzer_Stop();

	uint32_t final_missed = GetMissedChunks();
	resp->code = RESPONSE_SUCCESS;
	resp->data_len = sizeof(uint32_t);
	memcpy(resp->data, &final_missed, sizeof(uint32_t));
}

void commandLAConfig(struct CommandMessage *cmd, struct CommandResponse *resp){
	uint32_t sampling_rate_hz;

	if(cmd->data_len != sizeof(uint32_t)){
		resp->code = RESPONSE_FAIL;
		resp->data_len = 0;
		return;
	}

	memcpy(&sampling_rate_hz, cmd->data, sizeof(uint32_t));
	HAL_StatusTypeDef status = LogicAnalyzer_SetSamplingRate(sampling_rate_hz);
	if(status != HAL_OK){
		resp->code = RESPONSE_FAIL;
		resp->data_len = 0;
		return;
	}
	resp->code = RESPONSE_SUCCESS;
	resp->data_len = 0;
	return;
}


void commandLAGetChunk(struct CommandMessage *cmd, struct CommandResponse *resp){
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
	uint8_t *buffer_ptr = LogicAnalyzer_GetHalfBufferPointer(buffer_half);
	uint32_t half_size = LogicAnalyzer_GetHalfBufferSize();

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
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); // self chip select
}
