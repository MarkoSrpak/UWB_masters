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

    if (uwb_device.device_id == 1)
    {
        // Transmitter mode
        const uint8_t demo_data[] = "Ovo je testna poruka";

        while (1)
        {
            uwb_result_e result = uwb_send_payload(&uwb_device, 0x0002, demo_data, sizeof(demo_data));
            if (result != UWB_OK)
            {
                printf("TX error: %d\n\r", result);
            }
            else
            {
                printf("TX sent: %s\n\r", demo_data);
            }

            vTaskDelay(pdMS_TO_TICKS(1000));

            result = uwb_send_payload(&uwb_device, 0x0005, demo_data, sizeof(demo_data));
			if (result != UWB_OK)
			{
				printf("TX error: %d\n\r", result);
			}
			else
			{
				printf("TX sent: %s\n\r", demo_data);
			}

			vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    else
    {
        // Receiver mode
        uint8_t rx_data[128];
        uint32_t received_size = 0;
        uint16_t sender_address = 0;

        while (1)
        {
            uwb_result_e result = uwb_receive_poll(&uwb_device, &sender_address, rx_data, sizeof(rx_data), &received_size);

            if (result == UWB_OK)
            {
                printf("RX from 0x%04X: ", sender_address);
                for (uint32_t i = 0; i < received_size; i++)
                {
                    putchar(rx_data[i]);
                }
                putchar('\n');
                putchar('\r');
            }
            else if (result != UWB_TIMEOUT && result != UWB_WRONG_ADDRESS)
            {
                printf("RX error: %d\n\r", result);
            }

            // Loop again to receive the next packet
        }
    }
}
