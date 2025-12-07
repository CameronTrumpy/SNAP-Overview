/*
 * snap_wf_master.c
 *
 * Waveform generator module control implementation
 * Handles communication with the external waveform generator via I2C
 */

#include "snap_wf_master.h"
#include "main.h"
#include "cmsis_os.h"
#include "snap_config.h"
#include <string.h>

// External variables from main.c
extern I2C_HandleTypeDef hi2c1;
extern osMutexId_t i2cMutexHandle;

/**
 * Send a waveform configuration to the waveform generator module
 *
 * @param wave Pointer to WaveForm struct containing waveform parameters
 * @return SNAP_OK on success, negative error code on failure
 */
int send_waveform(const struct WaveForm* wave) {
    if (wave == NULL) {
        return SNAP_ERR_NULL_PARAM;
    }

    // Acquire I2C mutex
    if (osMutexAcquire(i2cMutexHandle, osWaitForever) != osOK) {
        return SNAP_ERR_BUSY;
    }

    uint8_t cmd_buffer[sizeof(struct CommandMessage) + COMMAND_BODY_SIZE];
    struct CommandMessage* cmd = (struct CommandMessage*)cmd_buffer;

    cmd->code = COMMAND_SET_WAVEFORM;
    cmd->data_len = sizeof(struct WaveForm);

    // Copy the provided waveform struct into command data
    memcpy(cmd->data, wave, sizeof(struct WaveForm));

    uint8_t resp_buffer[sizeof(struct CommandResponse) + 64] = {0};
    struct CommandResponse* resp = (struct CommandResponse*)resp_buffer;

    int result = sendCommand(&hi2c1, SLAVE_ADDR_WAVPD, cmd, resp, WAVEFORM_TIMEOUT);

    osMutexRelease(i2cMutexHandle);

    return result;
}

/**
 * Send an example waveform of the specified type with default parameters
 * (1.0V amplitude, 10kHz frequency, 1.0V offset, 0 phase)
 *
 * @param type Type of waveform to send
 * @return SNAP_OK on success, negative error code on failure
 */
int send_example_waveform(enum WaveformType type) {
    // Acquire I2C mutex
    if (osMutexAcquire(i2cMutexHandle, osWaitForever) != osOK) {
        return SNAP_ERR_BUSY;
    }

    uint8_t cmd_buffer[sizeof(struct CommandMessage) + COMMAND_BODY_SIZE];
    struct CommandMessage* cmd = (struct CommandMessage*)cmd_buffer;

    cmd->code = COMMAND_SET_WAVEFORM;
    cmd->data_len = sizeof(struct WaveForm);

    struct WaveForm wave;
    wave.type = type;

    // Configure waveform parameters based on type
    switch (type) {
        case WAVEFORM_SINE:
            wave.sine.Amplitude = 1.0f;
            wave.sine.Frequency = 10000.0f;
            wave.sine.Offset = 1.0f;
            wave.sine.Phase = 0.0f;
            break;

        case WAVEFORM_SQUARE:
            wave.square.Amplitude = 1.0f;
            wave.square.Frequency = 10000.0f;
            wave.square.Offset = 1.0f;
            wave.square.Phase = 0.0f;
            break;

        case WAVEFORM_TRIANGLE:
            wave.triangle.Amplitude = 1.0f;
            wave.triangle.Frequency = 10000.0f;
            wave.triangle.Offset = 1.0f;
            wave.triangle.Phase = 0.0f;
            break;

        case WAVEFORM_RAMP:
            wave.ramp.Amplitude = 1.0f;
            wave.ramp.Frequency = 10000.0f;
            wave.ramp.Offset = 1.0f;
            wave.ramp.Phase = 0.0f;
            wave.ramp.Upwards = true;
            break;

        case WAVEFORM_PULSE:
            wave.pulse.Amplitude = 1.0f;
            wave.pulse.Frequency = 10000.0f;
            wave.pulse.Offset = 1.0f;
            wave.pulse.DutyCycle = 10.0f;
            wave.pulse.Phase = 0.0f;
            break;

        default:
            osMutexRelease(i2cMutexHandle);
            return SNAP_ERR_INVALID_SIZE;  // Invalid waveform type
    }

    memcpy(cmd->data, &wave, sizeof(struct WaveForm));

    uint8_t resp_buffer[sizeof(struct CommandResponse) + 64] = {0};
    struct CommandResponse* resp = (struct CommandResponse*)resp_buffer;

    int result = sendCommand(&hi2c1, SLAVE_ADDR_WAVPD, cmd, resp, WAVEFORM_TIMEOUT);

    osMutexRelease(i2cMutexHandle);

    return result;
}
