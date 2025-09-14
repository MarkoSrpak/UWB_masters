/*
 * multilateration.h
 *
 *  Created on: Sep 2, 2025
 *      Author: Marko Srpak
 */

#ifndef APP_INC_MULTILATERATION_H_
#define APP_INC_MULTILATERATION_H_

/*--------------------------- INCLUDES ---------------------------------------*/
#include <stdio.h>
#include <stdbool.h>
#include "device_protocol.h"
/*--------------------------- MACROS AND DEFINES -----------------------------*/
/*--------------------------- TYPEDEFS AND STRUCTS ---------------------------*/
/*--------------------------- EXTERN -----------------------------------------*/
/*--------------------------- GLOBAL FUNCTION PROTOTYPES ---------------------*/

void multilat_aprox_matrix(
    const coord_t anchors[4], // 4 anchora sa x,y,z
    const double distances[4],  // 4 udaljenosti s1,s2,s3,s4
    coord_t *est		  // izlaz: x,y,z estimacije
);

void multilat_gauss_iter_matrix(
    const coord_t anchors[4], // 4 anchora sa x,y,z
    const double distances[4],  // 4 udaljenosti s1,s2,s3,s4
    coord_t *est		  // izlaz: x,y,z estimacije
);

#endif /* APP_INC_MULTILATERATION_H_ */
