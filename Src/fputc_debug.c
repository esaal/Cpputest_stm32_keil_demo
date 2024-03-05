#include <stdio.h>
#include <stm32h7xx.h>

int fputc(int c, FILE *stream)
{
	return(ITM_SendChar(c));
}
