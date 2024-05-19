#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <string.h>

#ifndef NEORV32_PACKAGE
#define NEORV32_PACKAGE
#include "../neorv32_library/neorv32_package.h"
#endif 

#ifndef NEORV32_GPIO_RAW
#define NEORV32_GPIO_RAW
#include "../neorv32_library/neorv32_gpio_raw.h"
#endif

// #define PRINT_GETC(a) neorv32_uart0_getc()
// #define PRINT_PUTC(a) neorv32_uart0_putc(a)

void task0(void){

    /*************Start Dynamic Task Status*****************/

    printf("Task 0 ID: %u\n", k_current_get());

	printf("Task0 BEGIN!!\n");

	printf("Start Dynamic Task Status\n");

	printf("Assign test secure cfs register[10]\n");
	NEORV32_CFS->REG[10] = 10;
	// uint32_t counter = 0;

	uint32_t num = 0;

	while(true){

        k_tid_t current_thread = k_current_get();
		NEORV32_CFS->REG[0] = current_thread; // set task3 thread ID to its correponding cfs_reg

		/*Exist If No Interrupt Occur*/
		if(NEORV32_CFS->REG[NEORV32_INT_REG] != 1){
			printf("TASK0 yield: [%d]\n", NEORV32_CFS->REG[NEORV32_INT_REG]);
			k_yield();
			
		}

		printf("InLoop S Task0\n");
		
		/**Test for secure register and buffer**/
		
		printf("Register At [10] is %d\n", NEORV32_CFS->REG[10]);

		// start();

		/*FOR INPUT UPDATE STATUS TABLE*/

		/** ADD PC FETCH **/
		uint32_t PC_fetch = NEORV32_CFS->REG[NEORV32_PC_REG];
		printf("PC Fetch at Task0: [%x]\n", PC_fetch);
        
        printf("Task0 STATUS TABLE INPUT:> ");
        char c = neorv32_uart_getc(NEORV32_UART0);
        neorv32_uart_putc(NEORV32_UART0, c); // echo
        printf("\n");
		num = c - 48;
        task_status_table(num%3);

		/*END OF USER INPUT*/
		
		//task_status_table(counter%3);

		printf("InLoop E Task0\n");
		
		k_yield();
		
		//counter++;
	}

}