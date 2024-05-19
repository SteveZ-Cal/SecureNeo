#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>

#ifndef NEORV32_PACKAGE
#define NEORV32_PACKAGE
#include "../neorv32_library/neorv32_package.h"
#endif 

#ifndef NEORV32_GPIO_RAW
#define NEORV32_GPIO_RAW
#include "../neorv32_library/neorv32_gpio_raw.h"
#endif

// uint32_t DATA_ADDRESS = 0x80000000; // data address of the neorv32

uint32_t DATA_ADDRESS = 0x80002200; // data address of the neorv32

// void task4(void)
// {
	
// 	while(true){
		
// 		printf("Task 4 ID: %u\n", k_current_get());

// 		printf("InLoop S Task4\n");

// 		uint32_t* c = DATA_ADDRESS;

// 		printf("DATA_ADDRESS 0x%x: [%d]\n", DATA_ADDRESS, *c);

// 		DATA_ADDRESS += 4;

// 		printf("InLoop E Task4\n");

// 		k_yield();

// 	}
// }

void task4(void){
	int ch;

   for( ch = 75 ; ch <= 100; ch++ ) {
      printf("ASCII value = %d, Character = %c\n", ch , ch );
   }

   return(0);
}