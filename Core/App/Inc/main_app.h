/*
 * main_app.h
 *
 *  Created on: Jun 13, 2025
 *      Author: Marko Srpak
 */

#ifndef INC_MAIN_APP_H_
#define INC_MAIN_APP_H_

/*--------------------------- INCLUDES ---------------------------------------*/
/*--------------------------- MACROS AND DEFINES -----------------------------*/
#define ASSERT_OK(expr)                                      \
    do {                                                     \
        uwb_result_e res = (expr);                           \
        if (res != UWB_OK) {                                 \
            printf("ASSERT FAILED: %s returned %d\n", #expr, res); \
        }                                                    \
    } while (0)

#define POLL_RX_TO_RESP_TX_DLY_UUS 650
/*--------------------------- TYPEDEFS AND STRUCTS ---------------------------*/
/*--------------------------- EXTERN -----------------------------------------*/
/*--------------------------- GLOBAL FUNCTION PROTOTYPES ---------------------*/
void main_app_task(void *parameters);


#endif /* INC_EXAMPLE_APP_H_ */
