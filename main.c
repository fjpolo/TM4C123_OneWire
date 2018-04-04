// standard C
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
// inc
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
// driverlib
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/fpu.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
#include <driverlib/gpio.h>

// OneWire
#include "oneWire.h"
//
//
//
static double Ts1,Ts2,Ts3,Ts4,Ts5,Ts6;

int main(void)
{
	int i;
	int v = 0;
	int f = 0;

	//
	//
	//
	Initialize();
	// Configure OneWire
	InitOneWire(0);
	// Get ROM addresses
	OneWiregetROMs();
	/*
	// Validate ROMs
	for(i = 0; i < allRoms.cantRoms; i++)
	{
		if(oneWireValidatePkg(allRoms.roms[i].rom, 8))
			v++;
		else
			f++;
	}
	 */
	//
	//
	//
	while(1)
	{
		//
		OneWireStartConvertion();
		delayMS(2000);
		//
		// Leo Ts1		LEE SIEMPRE
		//3
		OneWireGetTemp(&Ts1, 3);
		//timerDelayMs(200);
		// Leo Ts2
		//1
		OneWireGetTemp(&Ts2, 1);
		//timerDelayMs(200);
		// Ts3 			DESOLDADO
		//
		// Leo Ts4
		//2
		OneWireGetTemp(&Ts4, 2);
		//timerDelayMs(200);
		// Leo Ts5
		//4
		OneWireGetTemp(&Ts5, 4);
		//timerDelayMs(200);
		// Leo Ts6		NO LEE NADA
		//0
		OneWireGetTemp(&Ts6, 0);
		//timerDelayMs(200);
		delayMS(1);
	}
}

