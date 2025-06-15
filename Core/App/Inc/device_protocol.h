/*
 * device_protocol.h
 *
 *  Created on: Jun 14, 2025
 *      Author: Marko Srpak
 */

#ifndef APP_INC_DEVICE_PROTOCOL_H_
#define APP_INC_DEVICE_PROTOCOL_H_

/*--------------------------- INCLUDES ---------------------------------------*/
#include <stdio.h>
#include <stdbool.h>

#include "main.h"
#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
/*--------------------------- MACROS AND DEFINES -----------------------------*/
/*--------------------------- TYPEDEFS AND STRUCTS ---------------------------*/

// Enum to represent result/status codes for UWB operations
typedef enum {
    UWB_OK = 0,              // Operation completed successfully
    UWB_ERROR,               // General error
    UWB_TIMEOUT,             // Operation timed out
    UWB_INVALID_PARAM,       // One or more parameters were invalid
    UWB_DEVICE_NOT_FOUND,    // Specified UWB device was not found
    UWB_NOT_INITIALIZED,     // Device or system not initialized
	UWB_NOT_CONFIGURED,      // Device or system not configured
    UWB_BUSY,                // Device or resource is busy
    UWB_COMM_ERROR,          // Communication failure
    UWB_MEMORY_ERROR         // Memory allocation or access error
} uwb_result_e;

// Enum to represent the type of UWB device
typedef enum {
    ANCHOR,  // Device is an anchor
    TAG      // Device is a tag
} device_type_e;

// Struct to represent a 3D coordinate
typedef struct {
    double x;  // X-coordinate
    double y;  // Y-coordinate
    double z;  // Z-coordinate
} coord_t;

// Struct to represent a UWB (Ultra-Wideband) device
typedef struct {
    uint32_t partID;                // Unique 32-bit Part ID from the UWB IC (chip-specific hardware ID)
    uint64_t lotID;                 // Unique 64-bit Lot ID from the UWB IC (identifies manufacturing lot/batch)
    uint32_t deviceHash;            // Deterministic hash generated from partID and lotID to identify the device in code

    uint16_t panID;                 // Personal Area Network ID used for logical grouping of UWB devices (optional if no filtering)
    uint16_t address16;             // 16-bit short address used in the UWB protocol stack (optional if no filtering)

    uint16_t device_id;             // Logical ID
    device_type_e device_type;      // Device role

    bool is_serial;                 // True if device communicates over a serial interface (e.g., UART/USB), false for pure RF nodes
    bool is_initialized;            // Tracks whether the device has completed all initialization/configuration steps

    dwt_config_t config;            // DW3xxx chip configuration (channel, PRF, data rate, preamble length, etc.)

    uint16_t tx_ant_dly;            // Calibrated transmit antenna delay, used for accurate timestamp calculations
    uint16_t rx_ant_dly;            // Calibrated receive antenna delay, used for accurate timestamp calculations

    coord_t coord;                  // 3D spatial coordinates (x, y, z) of the device, typically used for anchors
} uwb_device_t;


/*--------------------------- EXTERN -----------------------------------------*/
/*--------------------------- GLOBAL FUNCTION PROTOTYPES ---------------------*/
uwb_result_e uwb_device_init(uwb_device_t *uwb_device);
uwb_result_e uwb_send_payload(uwb_device_t *uwb_device, uint16_t target_device_address, uint8_t* data, uint32_t data_size);

#endif /* APP_INC_DEVICE_PROTOCOL_H_ */
