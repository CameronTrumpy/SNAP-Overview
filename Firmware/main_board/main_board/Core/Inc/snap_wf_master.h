/*
 * snap_wf_master.h
 *
 * Waveform generator module control interface
 * Handles communication with the external waveform generator via I2C
 */

#ifndef SNAP_WF_MASTER_H
#define SNAP_WF_MASTER_H

#include "wavetypes.h"
#include "snap_master.h"

/**
 * Send a waveform configuration to the waveform generator module
 *
 * @param wave Pointer to WaveForm struct containing waveform parameters
 * @return SNAP_OK on success, negative error code on failure
 */
int send_waveform(const struct WaveForm* wave);

/**
 * Send an example waveform of the specified type with default parameters
 * (1.0V amplitude, 10kHz frequency, 1.0V offset, 0 phase)
 *
 * @param type Type of waveform to send
 * @return SNAP_OK on success, negative error code on failure
 */
int send_example_waveform(enum WaveformType type);

#endif // SNAP_WF_MASTER_H
