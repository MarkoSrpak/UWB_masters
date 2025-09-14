/*
 * position_protocol.h
 *
 *  Created on: Jun 21, 2025
 *      Author: Marko Srpak
 */

#ifndef APP_INC_POSITION_PROTOCOL_H_
#define APP_INC_POSITION_PROTOCOL_H_

/*--------------------------- INCLUDES ---------------------------------------*/
#include <stdio.h>
#include <stdbool.h>

#include "main.h"
#include "device_protocol.h"
#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
/*--------------------------- MACROS AND DEFINES -----------------------------*/
/*--------------------------- TYPEDEFS AND STRUCTS ---------------------------*/
/*--------------------------- EXTERN -----------------------------------------*/
extern const coord_t anchor1;
extern const coord_t anchor2;
extern const coord_t anchor3;
extern const coord_t anchor4;
/*--------------------------- GLOBAL FUNCTION PROTOTYPES ---------------------*/
void range_with(uwb_device_t *uwb_device, uint16_t target_address, double *distance, coord_t *coord);
void self_position_device_2(uwb_device_t *uwb_device);
void self_position_device_3(uwb_device_t *uwb_device);
void self_position_device_4(uwb_device_t *uwb_device);
void self_position_device_5(uwb_device_t *uwb_device);

#endif /* APP_INC_POSITION_PROTOCOL_H_ */
