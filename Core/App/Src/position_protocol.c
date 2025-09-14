/*
 * position_protocol.c
 *
 *  Created on: Jun 21, 2025
 *      Author: Marko Srpak
 */

/*--------------------------- INCLUDES ---------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "position_protocol.h"
#include "device_protocol.h"
#include "main_app.h"
#include "math.h"
#include "multilateration.h"
/*--------------------------- MACROS AND DEFINES -----------------------------*/
/*--------------------------- TYPEDEFS AND STRUCTS ---------------------------*/
/*--------------------------- STATIC FUNCTION PROTOTYPES ---------------------*/
static void anounce_coords(uwb_device_t *uwb_device, uwb_command_e command_type, uint8_t mode);
/*--------------------------- VARIABLES --------------------------------------*/
const coord_t anchor1 = {0, 0, 0};
const coord_t anchor2 = {4, 0, 0};
const coord_t anchor3 = {2, 4, 0};
const coord_t anchor4 = {2, 2, 2};
const coord_t anchors_predef[4] = {anchor1, anchor2, anchor3, anchor4};
static double tof;
static uwb_msg_t rx_msg;
static uwb_msg_t tx_msg;
static uint16_t sender_address = 0;
static uint8_t rx_data[128];
static uint32_t received_size = 0;
/*--------------------------- STATIC FUNCTIONS -------------------------------*/
/*--------------------------- GLOBAL FUNCTIONS -------------------------------*/
void range_with(uwb_device_t *uwb_device, uint16_t target_address, double *distance, coord_t *coord){
	tx_msg.rx_ts = 0;
	tx_msg.tx_ts = 0;
	tx_msg.coord = uwb_device->coord;
	tx_msg.command_type = COMMAND_RANGING_REQUEST;
	tx_msg.result = UWB_OK;
	ASSERT_OK(uwb_send_msg(uwb_device, target_address, &tx_msg, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED));
	uwb_result_e result = uwb_receive_poll(uwb_device, &sender_address, rx_data, sizeof(rx_data), &received_size);
	if(result == UWB_OK && received_size == sizeof(uwb_msg_t)){
		rx_msg = *(uwb_msg_t *)rx_data;
		// We expect ranging response after ranging request. Anything else is not good
		if(rx_msg.command_type == COMMAND_RANGING_RESPONSE)
		{
			uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
			int32_t rtd_init, rtd_resp;
			float clockOffsetRatio;

			// Retrieve poll transmission and response reception timestamps.
			poll_tx_ts = dwt_readtxtimestamplo32();
			resp_rx_ts = dwt_readrxtimestamplo32(0);

			// Read carrier integrator value and calculate clock offset ratio.
			clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

			// Get timestamps embedded in response message.
			poll_rx_ts = rx_msg.rx_ts;
			resp_tx_ts = rx_msg.tx_ts;

			// Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates
			rtd_init = resp_rx_ts - poll_tx_ts;
			rtd_resp = resp_tx_ts - poll_rx_ts;

			tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
			*distance = tof * SPEED_OF_LIGHT;
			*coord = rx_msg.coord;
			//printf("Distance to addr: %d = %lf\r\n", sender_address, *distance);
		}
		else
		{
			printf("ERROR: Didnt get ranging response\r\n");
		}
	}
}

static void anounce_coords(uwb_device_t *uwb_device, uwb_command_e command_type, uint8_t mode)
{
	tx_msg.rx_ts = 0;
	tx_msg.tx_ts = 0;
	tx_msg.coord = uwb_device->coord;
	tx_msg.command_type = command_type;

/*	if(command_type == COMMAND_POSITION_ANNOUNCEMENT){
		printf("Linear method autocalibrated anchors:\r\n");
	}
	else if(command_type == COMMAND_POSITION_ANNOUNCEMENT_GN)
	{
		printf("GN method autocalibrated anchors:\r\n");
	}
	else if(command_type == COMMAND_POSITION_ANNOUNCEMENT_PREDEF)
	{
		printf("Linear method predefined anchors:\r\n");
	}
	else if(command_type == COMMAND_POSITION_ANNOUNCEMENT_PREDEF_GN)
	{
		printf("GN method predefined anchors:\r\n");
	}

	printf("Announced coords (%lf, %lf, %lf) \r\n", uwb_device->coord.x, uwb_device->coord.y, uwb_device->coord.z);
*/
	ASSERT_OK(uwb_send_msg(uwb_device, 0x0001, &tx_msg, mode));
	return;
}

void self_position_device_2(uwb_device_t *uwb_device)
{
	double distance = 0.f;
	coord_t rx_coord = {0};
	double distance_uk = 0.f;
	int times = 1000;
	for(int i = 0; i < times; i++){
		range_with(uwb_device, 0x0001, &distance, &rx_coord);
		distance_uk += distance;
	}

	uwb_device->coord.x = rx_coord.x + distance_uk/times;
	uwb_device->coord.y = rx_coord.y;
	uwb_device->coord.z = rx_coord.z;
	anounce_coords(uwb_device, COMMAND_POSITION_ANNOUNCEMENT, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
	return;
}

void self_position_device_3(uwb_device_t *uwb_device)
{
	double distance1 = 0.f;
	coord_t rx_coord1 = {0};
	double distance2 = 0.f;
	coord_t rx_coord2 = {0};
	double distance_uk1 = 0.f;
	double distance_uk2 = 0.f;
	int times = 500;
	for(int i = 0; i < times; i++){
		range_with(uwb_device, 0x0001, &distance1, &rx_coord1);
		range_with(uwb_device, 0x0002, &distance2, &rx_coord2);
		distance_uk1 += distance1;
		distance_uk2 += distance2;
	}
	distance1 = distance_uk1 / times;
	distance2 = distance_uk2 / times;
	uwb_device->coord.x = (distance1*distance1 - distance2*distance2 + rx_coord2.x*rx_coord2.x) / (2*rx_coord2.x);
	double temp = distance1*distance1 - uwb_device->coord.x*uwb_device->coord.x;
	if(temp <= 0)
	{
		uwb_device->coord.y = 0;
	}
	else
	{
		uwb_device->coord.y = sqrt(temp);
	}
	uwb_device->coord.z = rx_coord1.z;
	anounce_coords(uwb_device, COMMAND_POSITION_ANNOUNCEMENT, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
	return;
}

void self_position_device_4(uwb_device_t *uwb_device)
{
	double distance1 = 0.f;
	coord_t rx_coord1 = {0};
	double distance2 = 0.f;
	coord_t rx_coord2 = {0};
	double distance3 = 0.f;
	coord_t rx_coord3 = {0};
	double distance_uk1 = 0.f;
	double distance_uk2 = 0.f;
	double distance_uk3 = 0.f;
	int times = 100;
	for(int i = 0; i < times; i++){
		range_with(uwb_device, 0x0001, &distance1, &rx_coord1);
		range_with(uwb_device, 0x0002, &distance2, &rx_coord2);
		range_with(uwb_device, 0x0003, &distance3, &rx_coord3);
		distance_uk1 += distance1;
		distance_uk2 += distance2;
		distance_uk3 += distance3;
	}
	distance1 = distance_uk1 / times;
	distance2 = distance_uk2 / times;
	distance3 = distance_uk3 / times;
	uwb_device->coord.x = (distance1*distance1 - distance2*distance2 + rx_coord2.x*rx_coord2.x) / (2*rx_coord2.x);
	uwb_device->coord.y = (distance2*distance2 - distance3*distance3 - rx_coord2.x*rx_coord2.x + rx_coord3.x*rx_coord3.x + rx_coord3.y*rx_coord3.y + 2*uwb_device->coord.x*(rx_coord2.x - rx_coord3.x))
			/ (2 * rx_coord3.y);
	double temp = distance1*distance1 - uwb_device->coord.x*uwb_device->coord.x - uwb_device->coord.y*uwb_device->coord.y;
	if(temp <= 0)
	{
		uwb_device->coord.z = 0;
	}
	else
	{
		uwb_device->coord.z = sqrt(temp);
	}

	anounce_coords(uwb_device, COMMAND_POSITION_ANNOUNCEMENT, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
	return;
}

void self_position_device_5(uwb_device_t *uwb_device)
{
	double distance[4] = {0.f, 0.f, 0.f, 0.f};
	coord_t rx_coord[4] = {{0}, {0}, {0}, {0}};
	double distance_uk[4] = {0.f, 0.f, 0.f, 0.f};
	int times = 1;
	for(int i = 0; i < times; i++){
		range_with(uwb_device, 0x0001, &distance[0], &rx_coord[0]);
		range_with(uwb_device, 0x0002, &distance[1], &rx_coord[1]);
		range_with(uwb_device, 0x0003, &distance[2], &rx_coord[2]);
		range_with(uwb_device, 0x0004, &distance[3], &rx_coord[3]);
		distance_uk[0] += distance[0];
		distance_uk[1] += distance[1];
		distance_uk[2] += distance[2];
		distance_uk[3] += distance[3];
	}
	distance[0] = distance_uk[0] / times;
	distance[1] = distance_uk[1] / times;
	distance[2] = distance_uk[2] / times;
	distance[3] = distance_uk[3] / times;

/*	printf("Distances: (%lf, %lf, %lf, %lf)\r\n", distance[0], distance[1], distance[2], distance[3]);
	// Printanje koordinata svih anchor-a
	for(int i = 0; i < 4; i++){
	    printf("Anchor %d coord: x=%lf, y=%lf, z=%lf\r\n", i+1, rx_coord[i].x, rx_coord[i].y, rx_coord[i].z);
	}
*/
	multilat_aprox_matrix(rx_coord, distance, &uwb_device->coord);
	anounce_coords(uwb_device, COMMAND_POSITION_ANNOUNCEMENT, DWT_START_TX_IMMEDIATE);
	vTaskDelay(1000);
	multilat_gauss_iter_matrix(rx_coord, distance, &uwb_device->coord);
	anounce_coords(uwb_device, COMMAND_POSITION_ANNOUNCEMENT_GN, DWT_START_TX_IMMEDIATE);
	vTaskDelay(1000);
	multilat_aprox_matrix(anchors_predef, distance, &uwb_device->coord);
	anounce_coords(uwb_device, COMMAND_POSITION_ANNOUNCEMENT_PREDEF, DWT_START_TX_IMMEDIATE);
	vTaskDelay(1000);
	multilat_gauss_iter_matrix(anchors_predef, distance, &uwb_device->coord);
	anounce_coords(uwb_device, COMMAND_POSITION_ANNOUNCEMENT_PREDEF_GN, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);


	//multilat_aprox_matrix(rx_coord, distance, &uwb_device->coord);
	//multilat_gauss_iter_matrix(rx_coord, distance, &uwb_device->coord);
	//anounce_coords(uwb_device, COMMAND_POSITION_ANNOUNCEMENT, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
	return;
}
