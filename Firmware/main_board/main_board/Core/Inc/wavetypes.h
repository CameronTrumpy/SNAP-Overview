#ifndef INC_WAVETYPES_H_
#define INC_WAVETYPES_H_

#include <stdbool.h>

// works up to 300kHz
struct Sine {
    float Amplitude;  // in Volts
    float Frequency;  // Frequency in Hz
    float Offset;     // Offset from 0V
    float Phase;      // radians
};

// works up to 200kHz
struct Square {
    float Amplitude;  // in Volts
    float Frequency;  // Frequency in Hz
    float Offset;     // Offset from 0V
    float Phase;      // radians
};

// works up to 200kHz
struct Triangle {
    float Amplitude;  // in Volts
    float Frequency;  // Frequency in Hz
    float Offset;     // Offset from 0V
    float Phase;      // radians
};

struct Ramp {
    float Amplitude;  // in Volts
    float Frequency;  // Frequency in Hz
    float Offset;     // Offset from 0V
    float Phase;      // radians
    // Ramp can be up or down, add a direction flag if needed
    bool Upwards;
};

struct Pulse {
    float Amplitude;  // in Volts
    float Frequency;  // Frequency in Hz
    float Offset;     // Offset from 0V
    float DutyCycle;  // percentage (0-100)
    float Phase;      // radians
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

#endif  // INC_WAVETYPES_H_