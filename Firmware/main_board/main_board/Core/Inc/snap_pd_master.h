/*
 * snap_pd_master.h
 *
 *  Created on: Nov 6, 2025
 *      Author: Andre
 */

#ifndef INC_SNAP_PD_MASTER_H_
#define INC_SNAP_PD_MASTER_H_

#include "snap_master.h"
#include "snap_config.h"
#include "stdbool.h"
#include "stdint.h"

#define SLAVE_ADDR 0x2B
#define MAX_PDO_ENTRIES 13

// Power Delivery Object (PDO) structure
typedef struct {
    union {
        struct {
            unsigned int voltage_max : 8;   // Bits 7:0, VOLTAGE_MAX field
            unsigned int peak_current : 2;  // Bits 9:8, PEAK_CURRENT field
            unsigned int current_max : 4;   // Bits 13:10, CURRENT_MAX field
            unsigned int type : 1;          // Bit 14, TYPE field
            unsigned int detect : 1;        // Bit 15, DETECT field
        } fixed;
        struct {
            unsigned int voltage_max : 8;  // Bits 7:0, VOLTAGE_MAX field
            unsigned int voltage_min : 2;  // Bits 9:8, VOLTAGE_MIN field
            unsigned int current_max : 4;  // Bits 13:10, CURRENT_MAX field
            unsigned int type : 1;         // Bit 14, TYPE field
            unsigned int detect : 1;       // Bit 15, DETECT field
        } pps;
        struct {
            unsigned int voltage_max : 8;  // Bits 7:0, VOLTAGE_MAX field
            unsigned int voltage_min : 2;  // Bits 9:8, VOLTAGE_MIN field
            unsigned int current_max : 4;  // Bits 13:10, CURRENT_MAX field
            unsigned int type : 1;         // Bit 14, TYPE field
            unsigned int detect : 1;       // Bit 15, DETECT field
        } avs;
        struct {
            uint8_t byte0;
            uint8_t byte1;
        };
    };
    unsigned long data;
} SRC_SPRandEPR_PDO_Fields;

// PSU Capabilities structure containing all available PDO profiles
typedef struct {
    SRC_SPRandEPR_PDO_Fields pdo_entries[MAX_PDO_ENTRIES];
    uint8_t num_entries;
} PSUCapabilities;

// Internal datastructures for tracking
typedef struct {
    // latest requested outputs
    float req_current_lim;
    float req_voltage;
    // latest measured actual outputs
    float current_actual;
    float voltage_actual;
    // board temperature
    float temp;
    union {
        struct {
            uint8_t selected_profile : 4;
            uint8_t f5v_en : 1;
            uint8_t f3v_en : 1;
            uint8_t reserved : 2;  // padding for future features
        };
        uint8_t flags;
    };
} PSUStatus;

// Internal datastructures for tracking
typedef struct {
    float req_current_lim;
    float req_voltage;
    union {
        struct {
            uint8_t selected_profile : 4;
            uint8_t f5v_en : 1;
            uint8_t f3v_en : 1;
            uint8_t reserved : 2;  // padding for future features
        };
        uint8_t flags;
    };
} PSUOutputCommand;

/**
 * @brief returns true if pd_alive (RESPONSE_SUCCESS)
 *  on RESPONSE_FAILED returns false
 *  usb_connected contains a bool, t/f, that indicates if usb-c is connected or not
 */
int pd_alive(bool* usb_connected);

/**
 * @brief queries the capabilities from the pd sink
 * @param capabs_out PSUCapabilities struct that will be populated by the command
 * @return SNAP_OK on success, error code on failure
 */
int pd_query_capabs(PSUCapabilities* capabs_out);

/**
 * @brief queries the pd module for status
 * @param status_out PSUStatus struct that will be populated by the command
 */
int pd_query_status(PSUStatus* status_out);

/**
 * @brief sends an output command to the pd sink
 * @param output the struct that will be sent over I2C containing output config
 */
int pd_request_output(PSUOutputCommand* output);

#endif /* INC_SNAP_PD_MASTER_H_ */
