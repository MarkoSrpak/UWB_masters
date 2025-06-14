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
    uint8_t device_id;          // Unique device identifier
    device_type_e device_type; 	// Type of the device: ANCHOR or TAG
    bool is_serial;             // Indicates if the device communicates over serial
    bool is_initialized;        // Indicates if the device has been initialized
    dwt_config_t config;		// DW1000/3000 configuration structure (e.g., channel, data rate, PRF, etc.)
    uint16_t tx_ant_dly;		// Transmit antenna delay (used for precise timestamp corrections)
    uint16_t rx_ant_dly;		// Receive antenna delay (used for precise timestamp corrections)
    coord_t coord;              // 3D coordinates of the device
} uwb_device_t;
/*--------------------------- EXTERN -----------------------------------------*/
/*--------------------------- GLOBAL FUNCTION PROTOTYPES ---------------------*/

#endif /* APP_INC_DEVICE_PROTOCOL_H_ */
