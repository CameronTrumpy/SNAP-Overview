/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "la_os_commands.h"
#include "packet.h"
#include "snap_master.h"
#include "snap_pd_master.h"
#include "snap_wf_master.h"
#include "tusb.h"
#include "wavetypes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CDC_MAIN 0
#define CDC_SIGROK 1

// Task stack sizes (in words, 4 bytes each)
#define USBD_STACK_SIZE (2048)
#define SUMP_STACK_SIZE (1024)
#define COMMAND_STACK_SIZE (1024)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef bsp_com_init;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_HS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* USER CODE BEGIN PV */

osThreadId_t chunkTaskHandle;
osSemaphoreId_t chunkStartSemaphore;
osMutexId_t chunkRequestMutex;
osMutexId_t usb_flush_mutex;
osSemaphoreId_t processCommandSemaphore;

static volatile uint32_t num_chunks_requested = 0;
static volatile uint8_t chunk_task_running = 0;
static volatile uint8_t chunk_cdc_port = CDC_MAIN;
static volatile uint8_t chunk_cancel_requested = 0;
static volatile CommandCode chunk_command =
    COMMAND_OS_GET_CHUNK;  // track if scope or LA data requested

// FreeRTOS task handles
osThreadId_t usbTaskHandle;
osThreadId_t sumpTaskHandle;
osThreadId_t commandTaskHandle;

// Task attributes
const osThreadAttr_t usb_task_attributes = {
    .name = "usb_device",
    .stack_size = USBD_STACK_SIZE * 4,
    .priority = (osPriority_t)osPriorityRealtime7,
};

const osThreadAttr_t command_task_attributes = {
    .name = "command_handler",
    .stack_size = COMMAND_STACK_SIZE * 4,
    .priority = (osPriority_t)osPriorityHigh,
};

const osThreadAttr_t chunk_task_attr = {
    .name = "chunk_streamer",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

// Mutexes for resource protection
osMutexId_t i2cMutexHandle;
osSemaphoreId_t spiSemaphoreHandle;

const osMutexAttr_t mutex_attributes = {
    .name = NULL,
    .attr_bits = osMutexRecursive | osMutexPrioInherit,
    .cb_mem = NULL,
    .cb_size = 0U};

#include "bsp/board_api.h"

size_t board_get_unique_id(uint8_t id[], size_t max_len) {
    (void)max_len;
    volatile uint32_t* stm32_uuid = (volatile uint32_t*)UID_BASE;
    uint32_t* id32 = (uint32_t*)(uintptr_t)id;
    uint8_t const len = 12;

    id32[0] = stm32_uuid[0];
    id32[1] = stm32_uuid[1];
    id32[2] = stm32_uuid[2];

    return len;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_OTG_HS_PCD_Init(void);
void StartDefaultTask(void* argument);

/* USER CODE BEGIN PFP */
// FreeRTOS task functions
void usb_device_task(void* param);
void command_handler_task(void* param);

int send_example_waveform(enum WaveformType type);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Invoked when device is mounted
void tud_mount_cb(void) {
    // Device mounted
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
    // Device unmounted
}

// Invoked when usb bus is suspended
void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    // Device suspended
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
    // Device resumed
}

struct GetChunkResponse {
    uint32_t chunk_number;  // Sequential chunk ID
    uint32_t samples_available;
    uint32_t missed_chunks;
};

uint8_t sample_buffer[SAMPLE_BUF_SIZE] __attribute__((section(".sram_d1"))) = {0};

/**
 * @brief Send OK response with optional payload data
 * @param cdc_interface: CDC interface number (CDC_MAIN or CDC_LOGIC)
 * @param data: Optional payload data (can be NULL if len is 0)
 * @param len: Length of payload data (0-255)
 */
static void send_ok_response(uint8_t cdc_interface, uint8_t* data, uint8_t len) {
    PacketResponseHeader header;
    header.start_marker = PACKET_START_MARKER_RESPONSE;
    header.status = PACKET_STATUS_OK;
    header.payload_length = len;

    osMutexAcquire(usb_flush_mutex, osWaitForever);
    // Send header
    tud_cdc_n_write(cdc_interface, (uint8_t*)&header, PACKET_HEADER_SIZE);

    // Send payload if present
    if (len > 0 && data != NULL) {
        tud_cdc_n_write(cdc_interface, data, len);
    }

    tud_cdc_n_write_flush(cdc_interface);
    osMutexRelease(usb_flush_mutex);
}

/**
 * @brief Send error response with error code
 * @param cdc_interface: CDC interface number (CDC_MAIN or CDC_LOGIC)
 * @param snap_error: SNAP error code (negative value)
 */
static void send_error_response(uint8_t cdc_interface, int8_t snap_error) {
    PacketResponseHeader header;
    header.start_marker = PACKET_START_MARKER_RESPONSE;
    header.status = snap_error_to_packet_status(snap_error);
    header.payload_length = 0;  // No payload for errors

    osMutexAcquire(usb_flush_mutex, osWaitForever);
    tud_cdc_n_write(cdc_interface, (uint8_t*)&header, PACKET_HEADER_SIZE);
    tud_cdc_n_write_flush(cdc_interface);
    osMutexRelease(usb_flush_mutex);
}

/**
 * @brief Initialize USB OTG HS peripheral hardware (clocks, GPIO, PHY)
 * Called before tusb_init() since TinyUSB DCD doesn't configure hardware
 */
static void usb_hardware_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    // Configure USB peripheral clock to use HSI48
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }

    // Enable USB voltage detector
    HAL_PWREx_EnableUSBVoltageDetector();

    // Enable GPIO clocks for USB ULPI pins
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure USB_OTG_HS ULPI pins
    // PC0 -> USB_OTG_HS_ULPI_STP
    // PC2_C -> USB_OTG_HS_ULPI_DIR
    // PC3_C -> USB_OTG_HS_ULPI_NXT
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // PA3 -> USB_OTG_HS_ULPI_D0
    // PA5 -> USB_OTG_HS_ULPI_CK
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PB0, PB1, PB5, PB10-13 -> USB_OTG_HS_ULPI_D1-D7
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11 |
                          GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Enable USB OTG HS peripheral and ULPI clocks
    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();
    __HAL_RCC_USB_OTG_HS_ULPI_CLK_ENABLE();

    // Enable USB OTG HS interrupt with priority 5 (FreeRTOS compatible)
    HAL_NVIC_SetPriority(OTG_HS_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
}

/**
 * @brief USB Device Task - Highest Priority
 * Runs tud_task() which processes all USB events
 * CRITICAL: Must be initialized AFTER FreeRTOS starts, not before
 */
void usb_device_task(void* param) {
    (void)param;

    // Initialize USB hardware (clocks, GPIO, PHY) before TinyUSB init
    // TinyUSB DCD driver only configures USB peripheral registers, not clocks/GPIO
    usb_hardware_init();

    tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_HIGH};
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    // Wait a bit for USB to stabilize
    osDelay(100);

    // RTOS forever loop
    while (1) {
        tud_task();

        // flush CDC writes
        for (uint8_t itf = 0; itf < CFG_TUD_CDC; itf++) {
            tud_cdc_n_write_flush(itf);
        }

        osDelay(1);
    }
}

/**
 * @brief USB speed test - streams test data to measure throughput
 * @param test_size_kb: Size of test data in KB
 * @param cdc_port: CDC interface to use (CDC_MAIN or CDC_LOGIC)
 * @return SNAP_OK on success, negative on error
 */
static int usb_speed_test(uint32_t test_size_kb, uint8_t cdc_port) {
    uint32_t total_bytes = test_size_kb * 1024;

    // Send response header with metadata (total bytes to be sent)
    PacketResponseHeader header = {.start_marker = PACKET_START_MARKER_RESPONSE,
                                   .status = PACKET_STATUS_OK,
                                   .payload_length = sizeof(total_bytes)};

    tud_cdc_n_write(cdc_port, (uint8_t*)&header, PACKET_HEADER_SIZE);
    tud_cdc_n_write(cdc_port, (uint8_t*)&total_bytes, sizeof(total_bytes));
    tud_cdc_n_write_flush(cdc_port);

    // Pre-fill sample_buffer with test pattern (incrementing 0x00-0xFF)
    // This avoids recreating the pattern on every iteration
    const uint32_t PATTERN_SIZE = 4096;  // Use 4KB pattern (reuse existing buffer)
    for (uint32_t i = 0; i < PATTERN_SIZE && i < SAMPLE_BUF_SIZE; i++) {
        sample_buffer[i] = (uint8_t)(i & 0xFF);
    }

    // Stream test data - tight loop with NO osDelay()
    uint32_t bytes_sent = 0;
    while (bytes_sent < total_bytes) {
        uint32_t bytes_remaining = total_bytes - bytes_sent;

        // Try to write as much as TinyUSB will accept
        // Use pattern buffer, wrapping if needed
        uint32_t bytes_to_send =
            (bytes_remaining < PATTERN_SIZE) ? bytes_remaining : PATTERN_SIZE;

        uint32_t written = tud_cdc_n_write(cdc_port, sample_buffer, bytes_to_send);
        bytes_sent += written;

        // Process USB events - NO delay!
        tud_task();
    }

    tud_cdc_n_write_flush(cdc_port);

    // Wait for transmission complete - tight loop, NO osDelay()
    const uint32_t MIN_AVAILABLE = 64;
    while (tud_cdc_n_write_available(cdc_port) < MIN_AVAILABLE) {
        tud_task();
    }

    return SNAP_OK;
}

void chunk_stream_task(void* param) {
    (void)param;

    while (1) {
        // Sleep until work arrives
        osSemaphoreAcquire(chunkStartSemaphore, osWaitForever);

        chunk_cancel_requested = 0;
        osMutexAcquire(chunkRequestMutex, osWaitForever);
        chunk_task_running = 1;
        osMutexRelease(chunkRequestMutex);

        while (1) {
            if (chunk_cancel_requested) {
                // Drain request queue
                osMutexAcquire(chunkRequestMutex, osWaitForever);
                num_chunks_requested = 0;
                osMutexRelease(chunkRequestMutex);

                chunk_cancel_requested = 0;
                break;
            }

            uint32_t chunks_to_send = 0;

            // Safely pull counter under mutex
            osMutexAcquire(chunkRequestMutex, osWaitForever);
            chunks_to_send = num_chunks_requested;
            num_chunks_requested = 0;
            osMutexRelease(chunkRequestMutex);

            if (chunks_to_send == 0) {
                break;
            }

            // Blocking stream execution
            command_get_chunk(chunks_to_send, chunk_cdc_port, chunk_command,
                              &chunk_cancel_requested);
        }

        osMutexAcquire(chunkRequestMutex, osWaitForever);
        chunk_task_running = 0;
        osMutexRelease(chunkRequestMutex);
    }
}

void tud_cdc_rx_cb(uint8_t itf) { osSemaphoreRelease(processCommandSemaphore); }

/**
 * @brief Command Handler Task
 * Handles waveform generation and other module commands
 * Uses mutexes to protect I2C/SPI resources
 */
void command_handler_task(void* param) {
    (void)param;

    // Wait for USB and system to be ready
    osDelay(300);

    uint8_t cdc_to_poll[2] = {CDC_MAIN, CDC_SIGROK};

    // RTOS forever loop
    while (1) {
        osSemaphoreAcquire(processCommandSemaphore, osWaitForever);
        for (size_t i = 0; i < sizeof(cdc_to_poll); i++) {
            // Check CDC_MAIN for commands
            if (tud_cdc_n_available(cdc_to_poll[i])) {
                // Ensure we have a full header before attempting to parse
                if (tud_cdc_n_available(cdc_to_poll[i]) < PACKET_HEADER_SIZE) {
                    osSemaphoreRelease(processCommandSemaphore);
                    osDelay(1);
                    continue;
                }

                uint8_t header_buf[PACKET_HEADER_SIZE] = {0};
                uint32_t header_count =
                    tud_cdc_n_read(cdc_to_poll[i], header_buf, PACKET_HEADER_SIZE);

                // Validate header
                if (header_count < PACKET_HEADER_SIZE ||
                    header_buf[0] != PACKET_START_MARKER_REQUEST) {
                    tud_cdc_n_read_flush(cdc_to_poll[i]);
                    continue;
                }

                PacketRequestHeader header;
                header.start_marker = header_buf[0];
                header.command = header_buf[1];
                header.payload_length = header_buf[2];

                if (header.payload_length > PACKET_MAX_PAYLOAD_SIZE) {
                    send_error_response(cdc_to_poll[i], SNAP_ERR_INVALID_SIZE);
                    tud_cdc_n_read_flush(cdc_to_poll[i]);
                    continue;
                }

                // Read payload data if any
                uint8_t payload_buf[PACKET_MAX_PAYLOAD_SIZE] = {0};
                if (header.payload_length > 0) {
                    // Wait for payload to be available
                    uint32_t wait_count = 0;
                    while (tud_cdc_n_available(cdc_to_poll[i]) < header.payload_length &&
                           wait_count < 100) {
                        osDelay(1);
                        wait_count++;
                    }

                    if (tud_cdc_n_available(cdc_to_poll[i]) >= header.payload_length) {
                        uint32_t payload_count = tud_cdc_n_read(
                            cdc_to_poll[i], payload_buf, header.payload_length);
                        if (payload_count < header.payload_length) {
                            send_error_response(cdc_to_poll[i], SNAP_ERR_TIMEOUT);
                            tud_cdc_n_read_flush(cdc_to_poll[i]);
                            continue;
                        }
                    } else {
                        // Timeout waiting for payload
                        send_error_response(cdc_to_poll[i], SNAP_ERR_TIMEOUT);
                        tud_cdc_n_read_flush(cdc_to_poll[i]);
                        continue;
                    }
                }

                // Parse command from header
                CommandCode cmd = (CommandCode)header.command;
                int result = 0;

                switch (cmd) {
                    case DUMMY_COMMAND: {
                        continue;  // no op
                    }
                    case COMMAND_PING: {
                        uint8_t pong[] = {'0' + cdc_to_poll[i], 'p', 'o', 'n', 'g'};
                        send_ok_response(cdc_to_poll[i], (uint8_t*)pong, sizeof(pong));
                        continue;
                    }

                    case COMMAND_SET_WAVEFORM:
                        // Parse WaveForm struct from payload
                        if (header.payload_length >= sizeof(struct WaveForm)) {
                            struct WaveForm wave;
                            memcpy(&wave, payload_buf, sizeof(struct WaveForm));
                            result = send_waveform(&wave);
                        } else {
                            result = SNAP_ERR_INVALID_SIZE;
                        }
                        break;

                    case COMMAND_LA_CONFIG:
                        // Parse frequency from payload (4 bytes)
                        if (chunk_task_running) {
                            result = SNAP_ERR_BUSY;
                            break;
                        }
                        if (header.payload_length >= 4) {
                            uint32_t freq;
                            memcpy(&freq, payload_buf, sizeof(uint32_t));
                            result = command_la_config(freq);
                        } else {
                            result = SNAP_ERR_INVALID_SIZE;
                        }
                        break;

                    case COMMAND_LA_START_CONTINUOUS:
                        if (chunk_task_running) {
                            result = SNAP_ERR_BUSY;
                            break;
                        }
                        result = command_la_start_continuous();
                        break;

                    case COMMAND_LA_STOP_CONTINUOUS:
                        chunk_cancel_requested = 1;
                        result = command_la_stop_continuous();
                        break;

                    case COMMAND_LA_GET_CHUNK:
                    case COMMAND_OS_GET_CHUNK: {
                        if (chunk_task_running) {
                            result = SNAP_ERR_BUSY;
                            break;
                        }
                        uint32_t num_chunks = 1;

                        if (header.payload_length >= 4) {
                            memcpy(&num_chunks, payload_buf, sizeof(uint32_t));
                        }

                        // Select mode
                        chunk_cdc_port = cdc_to_poll[i];
                        chunk_command = (CommandCode)header.command;

                        // Safely accumulate requests
                        osMutexAcquire(chunkRequestMutex, osWaitForever);
                        num_chunks_requested += num_chunks;
                        osMutexRelease(chunkRequestMutex);

                        osSemaphoreRelease(chunkStartSemaphore);

                        continue;  // Response already sent by command_la_get_chunk in the
                                   // chunk task
                    }
                        // OS COMMANDS
                    case COMMAND_OS_CONFIG:
                        if (chunk_task_running) {
                            result = SNAP_ERR_BUSY;
                            break;
                        }
                        // Parse frequency from payload (4 bytes)
                        if (header.payload_length >= 4) {
                            uint32_t freq;
                            memcpy(&freq, payload_buf, sizeof(uint32_t));
                            result = command_os_config(freq);
                        } else {
                            result = SNAP_ERR_INVALID_SIZE;
                        }
                        break;

                    case COMMAND_OS_START_CONTINUOUS:

                        if (chunk_task_running) {
                            result = SNAP_ERR_BUSY;
                            break;
                        }
                        result = command_os_start_continuous();
                        break;

                    case COMMAND_OS_STOP_CONTINUOUS:
                        chunk_cancel_requested = 1;
                        result = command_os_stop_continuous();
                        break;

                        // PD COMMANDS
                    case COMMAND_PD_ALIVE: {
                        bool usb_conn = false;
                        result = pd_alive(&usb_conn);
                        HAL_GPIO_WritePin(PD_CONN_LED_GPIO_Port, PD_CONN_LED_Pin,
                                          result == SNAP_OK);

                        if (result == SNAP_OK) {
                            // Send OK response with usb_conn as payload (1 byte)
                            send_ok_response(cdc_to_poll[i], &usb_conn, 1);
                            continue;
                        }
                        break;
                    }

                    case COMMAND_PD_QUERY_CAPABS: {
                        PSUCapabilities capabs;
                        result = pd_query_capabs(&capabs);

                        if (result == SNAP_OK) {
                            // Send OK response with PDO data as payload
                            size_t data_size =
                                capabs.num_entries * sizeof(SRC_SPRandEPR_PDO_Fields);
                            send_ok_response(cdc_to_poll[i], (uint8_t*)capabs.pdo_entries,
                                             (uint8_t)data_size);
                            continue;
                        }
                        break;
                    }

                    case COMMAND_PD_QUERY_STATUS: {
                        PSUStatus status;
                        result = pd_query_status(&status);

                        if (result == SNAP_OK) {
                            // Send OK response with PSUStatus struct as payload
                            send_ok_response(cdc_to_poll[i], (uint8_t*)&status,
                                             sizeof(PSUStatus));
                            continue;
                        }
                        break;
                    }

                    case COMMAND_PD_REQUEST_OUTPUT: {
                        // Parse PSUOutputCommand from payload
                        if (header.payload_length >= sizeof(PSUOutputCommand)) {
                            PSUOutputCommand psu_cmd;
                            memcpy(&psu_cmd, payload_buf, sizeof(PSUOutputCommand));
                            result = pd_request_output(&psu_cmd);
                        } else {
                            result = SNAP_ERR_INVALID_SIZE;
                        }
                        break;
                    }

                    case COMMAND_USB_SPEED_TEST: {
                        if (chunk_task_running) {
                            result = SNAP_ERR_BUSY;
                            break;
                        }
                        // Parse test size from payload (4 bytes)
                        if (header.payload_length >= 4) {
                            uint32_t test_size_kb;
                            memcpy(&test_size_kb, payload_buf, sizeof(uint32_t));

                            result = usb_speed_test(test_size_kb, cdc_to_poll[i]);
                            if (result == SNAP_OK) {
                                continue;  // Response already sent by usb_speed_test
                            }
                        } else {
                            result = SNAP_ERR_INVALID_SIZE;
                        }
                        break;
                    }

                    default:
                        // Unknown command
                        result = SNAP_ERR_INVALID_SIZE;
                        break;
                }

                // Send response with new packet format
                if (result == SNAP_OK) {
                    send_ok_response(cdc_to_poll[i], NULL, 0);
                } else {
                    send_error_response(cdc_to_poll[i], result);
                }
            }
        }
        osDelay(1);
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USB_OTG_HS_PCD_Init();
    /* USER CODE BEGIN 2 */
    // CRITICAL: Do NOT call tud_init() here!
    // USB initialization MUST happen inside usb_device_task AFTER FreeRTOS starts
    // Calling it here will cause race conditions with USB interrupts and FreeRTOS
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

    /* USER CODE BEGIN RTOS_MUTEX */
    // Create mutexes for resource protection
    usb_flush_mutex = osMutexNew(NULL);
    i2cMutexHandle = osMutexNew(&mutex_attributes);
    spiSemaphoreHandle = osSemaphoreNew(1, 1, NULL);

    if (i2cMutexHandle == NULL || spiSemaphoreHandle == NULL) {
        Error_Handler();  // Mutex creation failed
    }
    processCommandSemaphore = osSemaphoreNew(1, 0, NULL);

    chunkStartSemaphore = osSemaphoreNew(1, 0, NULL);
    chunkRequestMutex = osMutexNew(NULL);
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    // Create FreeRTOS tasks in priority order
    // CRITICAL: USB task MUST be created so USB init happens after scheduler starts
    usbTaskHandle = osThreadNew(usb_device_task, NULL, &usb_task_attributes);
    if (usbTaskHandle == NULL) {
        Error_Handler();  // USB task creation failed
    }

    commandTaskHandle = osThreadNew(command_handler_task, NULL, &command_task_attributes);
    if (commandTaskHandle == NULL) {
        Error_Handler();  // Command task creation failed
    }

    chunkTaskHandle = osThreadNew(chunk_stream_task, NULL, &chunk_task_attr);

    if (chunkStartSemaphore == NULL || chunkRequestMutex == NULL ||
        chunkTaskHandle == NULL) {
        Error_Handler();  // Command task creation failed
    }

    // Initialize Logic Analyzer Streaming
    la_stream_init();
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

    /* Initialize leds */
    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_YELLOW);
    BSP_LED_Init(LED_RED);

    /* Initialize USER push-button, will be used to trigger an interrupt each time it's
     * pressed.*/
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
    bsp_com_init.BaudRate = 115200;
    bsp_com_init.WordLength = COM_WORDLENGTH_8B;
    bsp_com_init.StopBits = COM_STOPBITS_1;
    bsp_com_init.Parity = COM_PARITY_NONE;
    bsp_com_init.HwFlowCtl = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &bsp_com_init) != BSP_ERROR_NONE) {
        Error_Handler();
    }

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    // Code never reaches here after osKernelStart()
    // All functionality is now in FreeRTOS tasks
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = 64;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 34;
    RCC_OscInitStruct.PLL.PLLP = 1;
    RCC_OscInitStruct.PLL.PLLQ = 9;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 3072;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                  RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {
    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00601A5C;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        Error_Handler();
    }

    /** I2C Enable Fast Mode Plus
     */
    HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {
    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
    hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 0x0;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {
    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USB_OTG_HS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_HS_PCD_Init(void) {
    /* USER CODE BEGIN USB_OTG_HS_Init 0 */

    /* USER CODE END USB_OTG_HS_Init 0 */

    /* USER CODE BEGIN USB_OTG_HS_Init 1 */

    /* USER CODE END USB_OTG_HS_Init 1 */
    hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
    hpcd_USB_OTG_HS.Init.dev_endpoints = 9;
    hpcd_USB_OTG_HS.Init.speed = PCD_SPEED_HIGH;
    hpcd_USB_OTG_HS.Init.dma_enable = DISABLE;
    hpcd_USB_OTG_HS.Init.phy_itface = USB_OTG_ULPI_PHY;
    hpcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
    hpcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
    hpcd_USB_OTG_HS.Init.lpm_enable = DISABLE;
    hpcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
    hpcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
    hpcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_OTG_HS) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USB_OTG_HS_Init 2 */

    /* USER CODE END USB_OTG_HS_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOF, PD_CONN_LED_Pin | OS_CONN_LED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : PD_CONN_LED_Pin OS_CONN_LED_Pin */
    GPIO_InitStruct.Pin = PD_CONN_LED_Pin | OS_CONN_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /*Configure GPIO pin : SPI_CS_Pin */
    GPIO_InitStruct.Pin = SPI_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void* argument) {
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    osDelay(500);
    for (;;) {
        osDelay(1000);
    }
    /* USER CODE END 5 */
}

/* MPU Configuration */

void MPU_Config(void) {
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
