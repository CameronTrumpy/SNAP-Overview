/*
 * packet.h
 *
 * Lightweight USB packet structure definitions for SNAP EE Workstation
 * Provides uniform packet format with minimal overhead for embedded constraints
 */

#ifndef INC_PACKET_H_
#define INC_PACKET_H_

#include <stdint.h>

// Packet header markers
#define PACKET_START_MARKER_REQUEST  0xAA
#define PACKET_START_MARKER_RESPONSE 0x55

// Maximum payload size (0-255 bytes)
#define PACKET_MAX_PAYLOAD_SIZE 255

// Header size (constant for all packets)
#define PACKET_HEADER_SIZE 3

// Status codes for responses
#define PACKET_STATUS_OK              0x00
#define PACKET_STATUS_ERR_NULL_PARAM  0x01  // NULL parameter
#define PACKET_STATUS_ERR_I2C_TX      0x02  // I2C transmit failed
#define PACKET_STATUS_ERR_I2C_RX      0x03  // I2C receive failed
#define PACKET_STATUS_ERR_SPI_RX      0x04  // SPI receive failed
#define PACKET_STATUS_ERR_BUSY        0x05  // Slave device busy
#define PACKET_STATUS_ERR_INVALID_SIZE 0x06 // Invalid data size
#define PACKET_STATUS_ERR_INVALID_HEADER 0x07 // Invalid packet header
#define PACKET_STATUS_ERR_INVALID_LENGTH 0x08 // Invalid payload length
#define PACKET_STATUS_ERR_TIMEOUT     0x09  // Operation timeout
#define PACKET_STATUS_ERR_INVALID_CMD 0x0A  // Invalid command code
#define PACKET_STATUS_ERR_UNKNOWN     0xFF  // Unknown error

// Request packet header structure
typedef struct {
    uint8_t start_marker;   // Always PACKET_START_MARKER_REQUEST (0xAA)
    uint8_t command;        // Command code
    uint8_t payload_length; // Length of payload data (0-255)
} __attribute__((packed)) PacketRequestHeader;

// Response packet header structure
typedef struct {
    uint8_t start_marker;   // Always PACKET_START_MARKER_RESPONSE (0x55)
    uint8_t status;         // Status code (0=OK, 1-255=error codes)
    uint8_t payload_length; // Length of payload data (0-255)
} __attribute__((packed)) PacketResponseHeader;

// Helper function to convert SNAP error codes to packet status codes
static inline uint8_t snap_error_to_packet_status(int8_t snap_error) {
    if (snap_error == 0) {
        return PACKET_STATUS_OK;
    }

    // Convert negative SNAP error codes to positive packet status codes
    uint8_t error_magnitude = (uint8_t)(-snap_error);

    // Map SNAP errors to packet status codes
    switch (error_magnitude) {
        case 1: return PACKET_STATUS_ERR_NULL_PARAM;
        case 2: return PACKET_STATUS_ERR_I2C_TX;
        case 3: return PACKET_STATUS_ERR_I2C_RX;
        case 4: return PACKET_STATUS_ERR_SPI_RX;
        case 5: return PACKET_STATUS_ERR_BUSY;
        case 6: return PACKET_STATUS_ERR_INVALID_SIZE;
        default: return PACKET_STATUS_ERR_UNKNOWN;
    }
}

#endif /* INC_PACKET_H_ */
