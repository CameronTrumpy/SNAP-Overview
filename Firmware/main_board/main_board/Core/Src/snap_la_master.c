/*
 * snap_la_master.c
 *
 *  Created on: Nov 10, 2025
 *      Author: Andre
 */

#include "cmsis_os.h"
#include "main.h"
#include "packet.h"
#include "snap_master.h"
#include "snap_config.h"
#include "stdbool.h"
#include "tusb.h"

// streaming state variables
static volatile bool streaming_active = false;
static volatile uint32_t streaming_chunks_remaining = 0;

// half buffer semaphores for ping-pong
static osSemaphoreId_t bufferHalf0SemHandle;
static osSemaphoreId_t bufferHalf1SemHandle;

// USB send task handle and semaphore
static osThreadId_t usbSendTaskHandle;
static osSemaphoreId_t usbSendSemaphoreHandle;
static osMutexId_t usb_send_mutex;
extern osMutexId_t usb_flush_mutex;

// Global variable to track which buffer semaphore to release in SPI callback
static osSemaphoreId_t spi_buffer_sem_to_release = NULL;

// USB send task data
static struct {
    uint8_t* buffer;
    uint32_t size;
    uint8_t cdc_port;
    osSemaphoreId_t semaphore;  // semaphore to release when done
} usb_send_data;

// Split the existing sample buffer into two halves
#define HALF_BUFFER_SIZE (SAMPLE_BUF_SIZE / 2)  // 32KB each half
#define BUFFER_HALF_0 (&sample_buffer[0])
#define BUFFER_HALF_1 (&sample_buffer[HALF_BUFFER_SIZE])

/**
 * @brief USB Send Task - sends buffer data over USB
 */
void usb_send_task(void* param) {
    (void)param;

    uint8_t* local_buffer;
    uint32_t local_size;
    uint8_t local_cdc_port;
    osSemaphoreId_t local_semaphore;

    while (1) {
        // wait for signal to send data
        osSemaphoreAcquire(usbSendSemaphoreHandle, osWaitForever);

        // copy the job data locally, protected by the mutex
        osMutexAcquire(usb_send_mutex, osWaitForever);
        local_buffer = usb_send_data.buffer;
        local_size = usb_send_data.size;
        local_cdc_port = usb_send_data.cdc_port;
        local_semaphore = usb_send_data.semaphore;
        osMutexRelease(usb_send_mutex);

        uint32_t bytes_sent = 0;
        const uint32_t CHUNK_SIZE = 64;

        osMutexAcquire(usb_flush_mutex, osWaitForever);
        // Send until we have covered the whole local_size
        while (bytes_sent < local_size) {
            uint32_t written = tud_cdc_n_write(local_cdc_port, &local_buffer[bytes_sent],
                                               local_size - bytes_sent);

            bytes_sent += written;

            tud_task();
        }

        tud_cdc_n_write_flush(local_cdc_port);
        osMutexRelease(usb_flush_mutex);

        // Wait for transmission complete
        while (tud_cdc_n_write_available(local_cdc_port) < CHUNK_SIZE) {
            tud_task();
        }

        // release the correct semaphore for this job
        if (local_semaphore != NULL) {
            osSemaphoreRelease(local_semaphore);
        }
    }
}

/**
 * @brief Trigger USB send task
 */
static void trigger_usb_send(uint8_t* buffer, uint32_t size, uint8_t cdc_port,
                             osSemaphoreId_t semaphore) {
    // Acquire mutex to safely modify the global struct
    osMutexAcquire(usb_send_mutex, osWaitForever);

    usb_send_data.buffer = buffer;
    usb_send_data.size = size;
    usb_send_data.cdc_port = cdc_port;
    usb_send_data.semaphore = semaphore;

    // Release mutex
    osMutexRelease(usb_send_mutex);

    osSemaphoreRelease(usbSendSemaphoreHandle);  // Signal the USB task to start
}

/**
 * @brief Get one chunk into specific buffer half
 *
 * @param buffer_half Pointer to buffer half (BUFFER_HALF_0 or BUFFER_HALF_1)
 * @param bytes_received Output: number of bytes received
 * @param buffer_sem Semaphore protecting this buffer (released in SPI callback)
 * @return Status code
 */
static int get_one_chunk_to_buffer(uint8_t* buffer_half, uint32_t* bytes_received,
                                   osSemaphoreId_t buffer_sem, CommandCode command) {
    // Acquire I2C mutex for command/response
    if (osMutexAcquire(i2cMutexHandle, osWaitForever) != osOK) {
        return SNAP_ERR_NULL_PARAM;
    }

    uint8_t cmd_buffer[sizeof(struct CommandMessage) + COMMAND_BODY_SIZE];
    struct CommandMessage* cmd = (struct CommandMessage*)cmd_buffer;
    cmd->code = command;
    cmd->data_len = 0;

    uint8_t resp_buffer[sizeof(struct CommandResponse) + COMMAND_BODY_SIZE] = {0};
    struct CommandResponse* resp = (struct CommandResponse*)resp_buffer;

    int status = sendCommand(&hi2c1, SLAVE_ADDR_LAOS, cmd, resp, I2C_TIMEOUT);
    if (status != 0) {
        osMutexRelease(i2cMutexHandle);
        return status;
    }

    struct GetChunkResponse {
        uint32_t chunk_number;
        uint32_t samples_available;
        uint32_t missed_chunks;
    };
    struct GetChunkResponse* chunkresp = (struct GetChunkResponse*)&resp->data;

    // Wait for samples to be available
    while (chunkresp->samples_available == 0) {
        status = sendCommand(&hi2c1, SLAVE_ADDR_LAOS, cmd, resp, I2C_TIMEOUT);
        if (status != 0) {
            osMutexRelease(i2cMutexHandle);
            return status;
        }
        chunkresp = (struct GetChunkResponse*)&resp->data;
    }

    *bytes_received = chunkresp->samples_available;

    // I2C work done
    osMutexRelease(i2cMutexHandle);

    // Acquire SPI semaphore for DMA transfer
    if (osSemaphoreAcquire(spiSemaphoreHandle, osWaitForever) != osOK) {
        return SNAP_ERR_NULL_PARAM;
    }

    // Store buffer semaphore for callback to release
    spi_buffer_sem_to_release = buffer_sem;

    // Start async SPI DMA receive
    status = receiveSPIdata(&hspi1, *bytes_received, buffer_half);

    if (status != 0) {
        // On error, release semaphores
        osSemaphoreRelease(spiSemaphoreHandle);
        if (buffer_sem != NULL) {
            osSemaphoreRelease(buffer_sem);
        }
        spi_buffer_sem_to_release = NULL;
        return status;
    }

    // SPI and buffer semaphores will be released in callback
    return SNAP_OK;
}

/**
 * @brief SPI DMA completion callback. releases SPI and buffer semaphores
 * must be called from HAL_SPI_RxCpltCallback
 */
void HAL_SPI_RxCpltCallback_Streaming(SPI_HandleTypeDef* hspi) {
    if (hspi->Instance == SPI1) {
        // Release SPI semaphore (ISR-safe)
        if (spiSemaphoreHandle != NULL) {
            xSemaphoreGiveFromISR(spiSemaphoreHandle, pdFALSE);
        }

        // Release buffer semaphore - signals data is ready (ISR-safe)
        if (spi_buffer_sem_to_release != NULL) {
            xSemaphoreGiveFromISR(spi_buffer_sem_to_release, pdFALSE);
            spi_buffer_sem_to_release = NULL;
        }
    }
}

/**
 * @brief Get N chunks from la or os
 *
 * @param num_chunks Number of chunks to retrieve (1 for single, >1 for streaming)
 * @param cdc_port CDC port to send data over
 * @return Status code
 */
int command_get_chunk(uint32_t num_chunks, uint8_t cdc_port, CommandCode command,
                      volatile uint8_t* stop_flag) {
    if (num_chunks == 0) {
        return SNAP_ERR_INVALID_SIZE;
    }

    const uint32_t requested_chunks = num_chunks;
    int curr_half = 0;
    uint32_t bytes_received = 0;
    osSemaphoreId_t half_semaphores[2] = {bufferHalf0SemHandle, bufferHalf1SemHandle};
    uint8_t* half_buffers[2] = {BUFFER_HALF_0, BUFFER_HALF_1};
    uint32_t bytes_in_half[2];

    // Early stop
    if (stop_flag && *stop_flag) {
        return SNAP_ERR_CANCELLED;
    }

    // Receive first chunk into buffer half 0
    if (osSemaphoreAcquire(half_semaphores[0], osWaitForever) != osOK) {
        return SNAP_ERR_NULL_PARAM;
    }

    int result = get_one_chunk_to_buffer(half_buffers[0], &bytes_received,
                                         half_semaphores[0], command);
    if (result != 0) {
        osSemaphoreRelease(half_semaphores[0]);
        return result;
    }

    // Wait for receive to complete
    if (osSemaphoreAcquire(half_semaphores[0], osWaitForever) != osOK) {
        return SNAP_ERR_NULL_PARAM;
    }

    bytes_in_half[0] = bytes_received;
    uint32_t chunks_left = (requested_chunks > 0) ? (requested_chunks - 1) : 0;

    // Send header
    if (stop_flag && *stop_flag) {
        return SNAP_ERR_CANCELLED;
    }

    // Send packet response header with total byte count as payload
    uint32_t total_bytes = bytes_in_half[0] * requested_chunks;
    PacketResponseHeader header = {.start_marker = PACKET_START_MARKER_RESPONSE,
                                   .status = PACKET_STATUS_OK,
                                   .payload_length = sizeof(total_bytes)};

    tud_cdc_n_write(cdc_port, (uint8_t*)&header, PACKET_HEADER_SIZE);
    tud_cdc_n_write(cdc_port, (uint8_t*)&total_bytes, sizeof(total_bytes));
    tud_cdc_n_write_flush(cdc_port);

    // Ping-pong loop
    while (chunks_left > 0) {
        int next_half = 1 - curr_half;

        // Acquire semaphore for next half
        if (osSemaphoreAcquire(half_semaphores[next_half], osWaitForever) != osOK) {
            osSemaphoreRelease(
                half_semaphores[curr_half]);  // Release current one if we fail
            return SNAP_ERR_NULL_PARAM;
        }

        // Trigger async USB send of current half
        trigger_usb_send(half_buffers[curr_half], bytes_in_half[curr_half], cdc_port,
                         half_semaphores[curr_half]);

        if (stop_flag && *stop_flag) {
            osSemaphoreRelease(half_semaphores[next_half]);
            return SNAP_ERR_CANCELLED;
        }
        // Start receiving next chunk (half_semaphores[next_half] released in callback)
        result = get_one_chunk_to_buffer(half_buffers[next_half], &bytes_received,
                                         half_semaphores[next_half], command);
        if (result != 0) {
            osSemaphoreRelease(half_semaphores[next_half]);
            return result;
        }

        // Wait for receive to complete
        if (osSemaphoreAcquire(half_semaphores[next_half], osWaitForever) != osOK) {
            return SNAP_ERR_NULL_PARAM;
        }

        bytes_in_half[next_half] = bytes_received;
        chunks_left--;
        curr_half = next_half;
    }

    // Send final chunk
    trigger_usb_send(half_buffers[curr_half], bytes_in_half[curr_half], cdc_port,
                     half_semaphores[curr_half]);

    // Wait for final send to complete
    if (osSemaphoreAcquire(half_semaphores[curr_half], osWaitForever) == osOK) {
        osSemaphoreRelease(half_semaphores[curr_half]);
    }

    return SNAP_OK;
}

int la_stream_init(void) {
    // Create half-buffer binary semaphores
    bufferHalf0SemHandle = osSemaphoreNew(1, 1, NULL);
    bufferHalf1SemHandle = osSemaphoreNew(1, 1, NULL);

    // Create USB send signaling semaphore
    usbSendSemaphoreHandle = osSemaphoreNew(1, 0, NULL);

    // usb data mutex
    usb_send_mutex = osMutexNew(NULL);

    if (bufferHalf0SemHandle == NULL || bufferHalf1SemHandle == NULL ||
        usbSendSemaphoreHandle == NULL) {
        return -1;
    }

    // Create USB send task
    const osThreadAttr_t usb_send_task_attributes = {
        .name = "usb_send",
        .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };

    usbSendTaskHandle = osThreadNew(usb_send_task, NULL, &usb_send_task_attributes);
    if (usbSendTaskHandle == NULL) {
        return -1;
    }

    return 0;
}
