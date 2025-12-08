/**
 * @file snap_config.h
 * @brief Central configuration constants for SNAP EE Workstation
 *
 * This header contains common constants used across multiple modules
 * to avoid redefinition and maintain consistency.
 */

#ifndef INC_SNAP_CONFIG_H_
#define INC_SNAP_CONFIG_H_

// Common buffer sizes
#define COMMAND_BODY_SIZE 512    // Size of command message data buffer
#define MAX_RESPONSE_SIZE 512    // Maximum response size

// Common timeout values (in milliseconds)
#define I2C_TIMEOUT       10000    // I2C transaction timeout
#define CONFIG_TIMEOUT    1000     // Configuration command timeout
#define TEST_TIMEOUT      1000     // Short test timeout
#define WAVEFORM_TIMEOUT  1000000  // Waveform generation timeout (1 second)

#endif /* INC_SNAP_CONFIG_H_ */
