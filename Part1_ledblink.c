/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    MKL25Z128xxx4_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "MKL25Z4.h"
#include "pin_mux.h"
//#include "fsl_debug_console.h"

#define RED_LED_SHIFT   (18)	// on port B
#define GREEN_LED_SHIFT (19)	// on port B
#define BLUE_LED_SHIFT  (1)		// on port D
#define MASK(x) (1UL << (x))

/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
/*void Delay(volatile unsigned int time_del) {
  while (time_del--) {
	;
  }
}*/

static void delay(volatile uint32_t number){
	while (number!=0){
		asm volatile("NOP");
		number--;
	}
}



void KL25Z_RGB_Flasher(void) {
	//int num;
	// Enable clock to Port B and Port D
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;

	// Make 3 pins GPIO
	//PORTB->PCR[RED_LED_SHIFT] &= ~PORT_PCR_MUX_MASK;
	//PORTB->PCR[RED_LED_SHIFT] |= PORT_PCR_MUX(1);
	//PORTB->PCR[GREEN_LED_SHIFT] &= ~PORT_PCR_MUX_MASK;
	//PORTB->PCR[GREEN_LED_SHIFT] |= PORT_PCR_MUX(1);
	PORTD->PCR[BLUE_LED_SHIFT] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED_SHIFT] |= PORT_PCR_MUX(1);

	// Set ports to outputs
	//PTB->PDDR |= MASK(RED_LED_SHIFT) | MASK(GREEN_LED_SHIFT);
	PTD->PDDR |= MASK(BLUE_LED_SHIFT);

	//PTB->PCOR |= MASK(RED_LED_SHIFT) | MASK(GREEN_LED_SHIFT);
	PTD->PCOR |= MASK(BLUE_LED_SHIFT);

	//while (1) {


		/*Starting blink cycle*/
		  PTD->PSOR = MASK(BLUE_LED_SHIFT);
		  delay(500000);
		  PTD->PCOR = MASK(BLUE_LED_SHIFT);
		 delay(500000);

		 PTD->PSOR = MASK(BLUE_LED_SHIFT);
		 delay(1000000);
		 PTD->PCOR = MASK(BLUE_LED_SHIFT);
		 delay(500000);

		PTD->PSOR = MASK(BLUE_LED_SHIFT);
		delay(2000000);
		PTD->PCOR = MASK(BLUE_LED_SHIFT);
		delay(500000);

		PTD->PSOR = MASK(BLUE_LED_SHIFT);
		delay(3000000);
		PTD->PCOR = MASK(BLUE_LED_SHIFT);
		delay(500000);

	 // }
	}



/*
 * @brief   Application entry point.
 */
int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    printf("Hello World\n");

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(i<10) {
        i++ ;
        printf("\n%d",i);
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
         KL25Z_RGB_Flasher();
    }
    return 0 ;
}
