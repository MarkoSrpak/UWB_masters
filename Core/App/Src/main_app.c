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
#include "position_protocol.h"
#include "device_protocol.h"
/*--------------------------- MACROS AND DEFINES -----------------------------*/
/*--------------------------- TYPEDEFS AND STRUCTS ---------------------------*/
/*--------------------------- STATIC FUNCTION PROTOTYPES ---------------------*/
static void start_receive_loop();
static void start_calibration();
/*--------------------------- VARIABLES --------------------------------------*/
static uwb_device_t uwb_device = {0};
static uint8_t rx_data[128];
static uint32_t received_size = 0;
static uint16_t sender_address = 0;
static uwb_msg_t rx_msg;
static uwb_msg_t tx_msg;
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
/*--------------------------- STATIC FUNCTIONS -------------------------------*/
static void start_receive_loop()
{
	while(1){
		uwb_result_e result = uwb_receive_poll(&uwb_device, &sender_address, rx_data, sizeof(rx_data), &received_size);
		if(result == UWB_OK && received_size == sizeof(uwb_msg_t)){
			rx_msg = *(uwb_msg_t *)rx_data;
// COMMAND_RANGING_REQUEST --------------------------------------------- RESPOND WITH RANGE AND COORDS
			if(rx_msg.command_type == COMMAND_RANGING_REQUEST){
				uint32_t resp_tx_time;

				// Retrieve poll reception timestamp.
				poll_rx_ts = get_rx_timestamp_u64();

				// Compute response message transmission time.
				resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

				// Response TX timestamp is the transmission time we programmed plus the antenna delay.
				resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + uwb_device.tx_ant_dly;

				// Write all timestamps in the final message.
				tx_msg.rx_ts = poll_rx_ts;
				tx_msg.tx_ts = resp_tx_ts;

				// Write coords in final message
				tx_msg.coord = uwb_device.coord;

				tx_msg.command_type = COMMAND_RANGING_RESPONSE;

				tx_msg.result = UWB_OK;

				ASSERT_OK(uwb_send_msg(&uwb_device, sender_address, &tx_msg, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED));
			}
// COMMAND_RANGING_RESPONSE-------------------------------------------- SHOULD NOT HAPPEN HERE
			else if(rx_msg.command_type == COMMAND_RANGING_RESPONSE){
				printf("ERROR: got ranging response without asking for it from %d\r\n", sender_address);
			}
// COMMAND_POSITION_YOURSELF------------------------------------------- Every device should position itself by predefined way
			else if(rx_msg.command_type == COMMAND_POSITION_YOURSELF){
				if(uwb_device.device_id == 2){
					self_position_device_2(&uwb_device);
				}
				else if(uwb_device.device_id == 3){
					self_position_device_3(&uwb_device);
				}
				else if(uwb_device.device_id == 4){
					self_position_device_4(&uwb_device);
				}
				else if(uwb_device.device_id == 5){
					self_position_device_5(&uwb_device);
				}
			}
			else if(rx_msg.command_type == COMMAND_POSITION_ANNOUNCEMENT){
				if(uwb_device.is_serial){
					printf("%d: (%lf, %lf, %lf)\r\n", sender_address, rx_msg.coord.x, rx_msg.coord.y, rx_msg.coord.z);
					return;
				}
			}
		}
	}
}

static void start_calibration()
{
	uwb_msg_t tx_msg_position_yourself;
	tx_msg_position_yourself.rx_ts = 0;
	tx_msg_position_yourself.tx_ts = 0;
	// Write coords in final message
	tx_msg_position_yourself.coord = uwb_device.coord;
	tx_msg_position_yourself.command_type = COMMAND_POSITION_YOURSELF;
	tx_msg_position_yourself.result = UWB_OK;
	while(1){
		ASSERT_OK(uwb_send_msg(&uwb_device, 0x0002, &tx_msg_position_yourself, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED));
		start_receive_loop();
		ASSERT_OK(uwb_send_msg(&uwb_device, 0x0003, &tx_msg_position_yourself, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED));
		start_receive_loop();
		vTaskDelay(100);
	}
}
/*--------------------------- GLOBAL FUNCTIONS -------------------------------*/
void main_app_task(void *parameters)
{
    ASSERT_OK(uwb_device_init(&uwb_device));

    if(uwb_device.device_type == ANCHOR && uwb_device.is_serial){
    	start_calibration();
    }
    else{
    	start_receive_loop();
    }


    /*
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
    }*/
}
