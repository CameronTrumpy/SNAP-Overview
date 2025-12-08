/*
 * waveform_generator_ll.h
 *
 *  Created on: Oct 5, 2025
 *      Author: Andre
 */

#ifndef INC_WAVEFORM_GENERATOR_LL_H_
#define INC_WAVEFORM_GENERATOR_LL_H_

#include "stdbool.h"
#include "stdint.h"
#include "main.h"
#include <math.h>
#include "snap_module.h"

#define SAMPLE_BUFFER_B 48000 // 48kB buffer
#define MAX_SAMPLES SAMPLE_BUFFER_B/2 // 2B samples
#define MAX_OUTPUT_RATE 10000000.0f // 10MS/s
#define DAC_MAX_VOLTAGE 3.3f  // Volts (VREF)
#define DAC_MIN_VOLTAGE 0.0f  // Volts
#define DAC_RESOLUTION 4096   // 12-bit DAC (2^12)
#define GAIN 3.0303f  // (5V - (-5V)) / 3.3V = 10V / 3.3V
#define OUTPUT_OFFSET 1.65f  // DAC voltage for 0V output (midpoint of 0-3.3V)
#define APB1_CLOCK 170000000.0f // 170MHz

extern uint16_t waveform_buffer[MAX_SAMPLES];


struct Sine {
    float Amplitude; // in Volts
    float Frequency; // Frequency in Hz
    float Offset;    // Offset from 0V
    float Phase;     // radians
};

struct Square {
    float Amplitude;  	// in Volts
    float Frequency;  	// Frequency in Hz
    float Offset;  		// Offset from 0V
    float Phase;  		// radians
};

struct Triangle {
    float Amplitude;  	// in Volts
    float Frequency;  	// Frequency in Hz
    float Offset;  		// Offset from 0V
    float Phase;  		// radians
};

struct Ramp {
    float Amplitude;	// in Volts
    float Frequency;	// Frequency in Hz
    float Offset;  		// Offset from 0V
    float Phase;  		// radians
    bool Upwards;
};

struct Pulse {
    float Amplitude;  	// in Volts
    float Frequency;  	// Frequency in Hz
    float Offset; 		// Offset from 0V
    float DutyCycle; 	// percentage (0-100)
    float Phase;  		// radians
};

enum WaveformType {
    WAVEFORM_SINE,
    WAVEFORM_SQUARE,
    WAVEFORM_TRIANGLE,
    WAVEFORM_RAMP,
    WAVEFORM_PULSE
};

struct WaveForm {
    enum WaveformType type;
    union {
        struct Sine sine;
        struct Square square;
        struct Triangle triangle;
        struct Ramp ramp;
        struct Pulse pulse;
    };
};



/* @brief Fill buffer with setBuf{waveform type} data*/
void setBufSine(struct WaveForm *wave_in, uint32_t num_samples);
void setBufSquare(struct WaveForm *wave_in, uint32_t num_samples);
void setBufTriangle(struct WaveForm *wave_in, uint32_t num_samples);
void setBufRamp(struct WaveForm *wave_in, uint32_t num_samples);
void setBufPulse(struct WaveForm *wave_in, uint32_t num_samples);

// Set DAC sets the onboard DAC the parameters defined in wave
// @param wave_in: waveform struct that defines the waveform to produce
// @return: 0 on success, nonzero otherwise
int setDAC(struct WaveForm *wave_in);

/*
 * @brief turn off the output
 * @return: 0 on success, nonzero otherwise
 */
int turnDACOff();

// command handler for setting the waveform
// parameters taken from I2C request
void commandSetWaveform(CommandMessage *cmd, CommandResponse *resp);

#endif /* INC_WAVEFORM_GENERATOR_LL_H_ */
