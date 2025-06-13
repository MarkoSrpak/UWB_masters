/*
 * example_app.h
 *
 *  Created on: Jun 13, 2025
 *      Author: Marko Srpak
 */

#ifndef INC_EXAMPLE_APP_H_
#define INC_EXAMPLE_APP_H_

/*--------------------------- INCLUDES ---------------------------------------*/
#include "examples_defines.h"
#include "port.h"
/*--------------------------- MACROS AND DEFINES -----------------------------*/
/*--------------------------- TYPEDEFS AND STRUCTS ---------------------------*/
/*--------------------------- EXTERN -----------------------------------------*/
extern example_ptr example_pointer;
/*--------------------------- GLOBAL FUNCTION PROTOTYPES ---------------------*/

void test_run_info(unsigned char *data);
void example_app_task(void *parameters);


#endif /* INC_EXAMPLE_APP_H_ */
