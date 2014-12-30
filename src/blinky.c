/*
 * blinky.c
 *
 *  Created on: 24 Dec 2014
 *      Author: FMA
 */

#include "stdio.h"


int main()
{
	prvSetupHardware();

	while(1)
	{
		printf("Hallo");
	}

	return 0;

}


void _sbrk() {}

void _exit() {}
