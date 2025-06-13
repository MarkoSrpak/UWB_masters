/*
 * example_app.c
 *
 *  Created on: Jun 13, 2025
 *      Author: Marko Srpak
 */

/*--------------------------- INCLUDES ---------------------------------------*/
#include <stdio.h>
#include <ctype.h>
#include <string.h>

#include "spi.h"
#include "gpio.h"
#include "port.h"
#include "examples_defines.h"
#include "example_app.h"
/*--------------------------- MACROS AND DEFINES -----------------------------*/
/*--------------------------- TYPEDEFS AND STRUCTS ---------------------------*/
/*--------------------------- STATIC FUNCTION PROTOTYPES ---------------------*/
/*--------------------------- VARIABLES --------------------------------------*/
// Ove treba imati
SPI_HandleTypeDef   *hcurrent_active_spi = &hspi1;/*clocked from 72MHz*/ //SPI Handle
uint16_t            pin_io_active_spi = DW_NSS_Pin; // DW_NSS_Pin
GPIO_PinState       SPI_CS_state = GPIO_PIN_RESET; //Determine the CS for the IO
host_using_spi_e    host_spi = SPI_1;
SPI_HandleTypeDef   hspi4;
/*--------------------------- STATIC FUNCTIONS -------------------------------*/
/*--------------------------- GLOBAL FUNCTIONS -------------------------------*/
void test_run_info(unsigned char *data)
{
    for (size_t i = 0; i < strlen((const char *)data); i++) {
        if (isalnum(data[i]) || isgraph(data[i])) {
            printf("%c", data[i]);
        } else {
            printf(".");
        }
    }
    printf("\n\r");
    for (size_t i = 0; i < strlen((const char *)data); i++) {
		printf("0x%02X ", data[i]);
	}
    printf("\n\r");
}

void example_app_task(void *parameters)
{
	build_examples();
	example_pointer();
}


