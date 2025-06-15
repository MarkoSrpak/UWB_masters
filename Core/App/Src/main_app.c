/*
 * main_app.c
 *
 *  Created on: Jun 13, 2025
 *      Author: Marko Srpak
 */

/*--------------------------- INCLUDES ---------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#include "main_app.h"
#include "device_protocol.h"
/*--------------------------- MACROS AND DEFINES -----------------------------*/
#define ASSERT_OK(expr)                                      \
    do {                                                     \
        uwb_result_e res = (expr);                           \
        if (res != UWB_OK) {                                 \
            printf("ASSERT FAILED: %s returned %d\n", #expr, res); \
        }                                                    \
    } while (0)
/*--------------------------- TYPEDEFS AND STRUCTS ---------------------------*/
/*--------------------------- STATIC FUNCTION PROTOTYPES ---------------------*/
static uwb_device_t uwb_device = {0};
/*--------------------------- VARIABLES --------------------------------------*/
/*--------------------------- STATIC FUNCTIONS -------------------------------*/
/*--------------------------- GLOBAL FUNCTIONS -------------------------------*/
void main_app_task(void *parameters)
{
	ASSERT_OK(uwb_device_init(&uwb_device));

	uint8_t demo_data[] = "Ovo je testna poruka";
	while(1)
	{
		ASSERT_OK(uwb_send_payload(&uwb_device, 0x5678, demo_data, sizeof(demo_data)));
		vTaskDelay(1000);
	}
}
