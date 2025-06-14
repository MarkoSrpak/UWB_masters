/*
 * retarget.c
 *
 *  Created on: Apr 21, 2025
 *      Author: Marko Srpak
 */
#include "usart.h"
#include <stdio.h>
/*
 // ONE OPTION IMPLEMENT fputc
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}*/
/*
 // ONE OPTION OVERRIDE _write
int _write( int file, char *ptr, int len ){
    int i;
    for(i = 0 ; i < len ; i++){
        HAL_UART_Transmit(&huart3,(uint8_t*)&ptr[i],1,10);
    }
    char ch='\r';
    HAL_UART_Transmit(&huart3,(uint8_t*)&ch,1,10);
    return len;
}*/

#ifdef __GNUC__
	/* With GCC, small printf (option LD Linker->Libraries->Small printf set to
	 * 'Yes') calls __io_putchar() */
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}
