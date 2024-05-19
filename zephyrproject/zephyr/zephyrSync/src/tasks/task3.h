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

#define TASK3_NUM_ITERATION 5

// static const struct device *gpio_ct_dev_task3 = DEVICE_DT_GET(DT_NODELABEL(gpio_lo));

// uint32_t task3_to_wait = 1000000; 

void neorv32_gpio_raw3(struct device *gpio_ct_dev_task, uint32_t gpio_pin, uint32_t task_to_wait, uint32_t num_iteration){

    int ret;

	// /** ADD PC FETCH **/
	// uint32_t PC_fetch = NEORV32_CFS->REG[NEORV32_PC_REG];
	// printf("PC Fetch at NEORV32_GPIO_RAW3: [%x]\n", PC_fetch);
    
    for (uint32_t i = 0; i < num_iteration; i++){

        ret = gpio_pin_set_raw(gpio_ct_dev_task, gpio_pin, 1);
		if (ret!=0){
			return;
		}

		// /** ADD PC FETCH **/
		// uint32_t PC_fetch1 = NEORV32_CFS->REG[NEORV32_PC_REG];
		// printf("PC Fetch at gpio_pin_set_raw3: [%x]\n", PC_fetch1);

		k_busy_wait(task_to_wait);

		// /** ADD PC FETCH **/
		// uint32_t PC_fetch2 = NEORV32_CFS->REG[NEORV32_PC_REG];
		// printf("PC Fetch at k_busy_wait3: [%x]\n", PC_fetch2);

		ret = gpio_pin_set_raw(gpio_ct_dev_task, gpio_pin, 0);
		if (ret!=0){
			return;
		}

		k_busy_wait(task_to_wait);

    }

}

void neorv32_gpio_raw3_v2(struct device *gpio_ct_dev_task, gpio_port_pins_t mask, gpio_port_value_t value, uint32_t task_to_wait, uint32_t num_iteration){

	int ret;

	for (uint32_t i = 0; i < num_iteration; i++){

		ret = gpio_port_set_masked(gpio_ct_dev_task, mask, value);
		if (ret!=0){
			return;
		}

		k_busy_wait(task_to_wait);

		ret = gpio_port_set_masked(gpio_ct_dev_task, mask, 0);
		if (ret!=0){
			return;
		}

		k_busy_wait(task_to_wait);

	}

}


void task3(void)
{

	static const struct device *gpio_ct_dev_task3 = DEVICE_DT_GET(DT_NODELABEL(gpio_lo));

	k_sched_time_slice_set(1000, 10);

	gpio_port_pins_t myPortPins = 1023;

	//gpio_port_value_t myPortValue = 341;

	gpio_port_value_t myPortValue = 1023;

	uint32_t task3_to_wait = 1000000; 

	printf("Task 3 ID: %u\n", k_current_get());

	printf("Task3 BEGIN!!\n");

	if (!device_is_ready(gpio_ct_dev_task3)){
		return;
	}

	printf("Task3 Device Ready!!\n");
	
	int ret;
	ret = gpio_pin_configure(gpio_ct_dev_task3, 3, GPIO_OUTPUT_ACTIVE); // set gpio_lo pin 3 as output

	printf("Task3 Configure Ready!!\n");

	if (ret!=0){
		return;
	}

	while(true){

		/****************************************************CHECK FOR CONTEXT SWITCH LATENCY (END)*****/

		// if(latencyEnable){
		// 	latencyEnd = NEORV32_CFS->REG[NEORV32_TIMER_REG];
		// 	//latencyEnd = neorv32_mtime_get_time();
		// 	//latencyEnd = sys_clock_cycle_get_64(); 
		// }

		// if(latencyVerbose){
		// 	printf("latencyStart: [%u]\n", latencyStart);
		// 	printf("latencyEnd: [%u]\n", latencyEnd);
		// }
    
		// if(latencyVerbose)
    	// 	printf("@@@@@@@@@@@@@@@@@ Context Switch Latency @@@@@@@@@@@@@@@@@: [%u]\n", cpu_get_mycycle(latencyStart, latencyEnd));

		/****************************************************CHECK FOR CONTEXT SWITCH LATENCY (END)*****/

		// k_tid_t current_thread = k_current_get();
		// NEORV32_CFS->REG[0] = current_thread; // set task3 thread ID to its correponding cfs_reg

		printf("\TASK3 LOOP:\n");
		uint32_t before;
		__asm__ __volatile__ (
			"mv %0, x26"
			: "=r" (before)
		);
		printf("	xreg_before[%x]\n", before);
		
		uint32_t after;
		__asm__ __volatile__ (
			"li x26, 3"
		);

		__asm__ __volatile__ (
			"mv %0, x26"
			: "=r" (after)
		);
		printf("	xreg_after[%x]\n", after);

		// /** ADD PC FETCH **/
		// printf("\nPC Fetch at Task3:\n");
		// uint32_t PC_fetch = NEORV32_CFS->REG[NEORV32_PC_REG];
		// printf("	pc_fetch[%x]\n", PC_fetch);

		unsigned int pc;
		__asm__ __volatile__ (
			"auipc %0, 0\n"    // Load the upper 20 bits of PC into %0
			//"jalr %0, %0, 4"   // Add the lower 12 bits of PC to %0
			: "=r" (pc)
		);
		printf("	pc_asm[%x]\n", pc);

		printf("InLoop <<S>> Task3\n");

		// printf("	cfs_reg[0]: [%u]\n", NEORV32_CFS->REG[0]);
		// printf("	cfs_reg[1]: [%u]\n", NEORV32_CFS->REG[1]);
		// printf("	cfs_reg[2]: [%x]\n", NEORV32_CFS->REG[CFS_CTRL_REG]);
		// printf("	cfs_reg[62]: [%x]\n", NEORV32_CFS->REG[62]);
		// printf("	cfs_reg[63]: [%x]\n", NEORV32_CFS->REG[63]);

		// printf("Register At [10] is %d\n", NEORV32_CFS->REG[10]);

		// ret = gpio_pin_set_raw(gpio_ct_dev_task3, 3, 1);
		// if (ret!=0){
		// 	return;
		// }

		// k_busy_wait(task3_to_wait);

		// ret = gpio_pin_set_raw(gpio_ct_dev_task3, 3, 0);
		// if (ret!=0){
		// 	return;
		// }

		// k_busy_wait(task3_to_wait);


		//neorv32_gpio_raw3(gpio_ct_dev_task3, 3, task3_to_wait, 2);

		neorv32_gpio_raw1_v2(gpio_ct_dev_task3, myPortPins, myPortValue, task3_to_wait, TASK3_NUM_ITERATION);

		printf("InLoop >>E<< Task3\n");		

		//NEORV32_CFS->REG[0] = 3;
		
		//__asm__ volatile ("jalr ra, %0" : : "r" (0x00020000)); 

		//__asm__ volatile ("li t0, %[input_i]; jr t0" :  : [input_i] "i" (0xFFF)); // jump to beginning of 0x20000


		/****************************************************CHECK FOR CONTEXT SWITCH LATENCY (START)*****/

		// if(latencyEnable){
		// 	latencyStart = NEORV32_CFS->REG[NEORV32_TIMER_REG];
		// 	//latencyStart = neorv32_mtime_get_time();
		// 	//latencyStart = sys_clock_cycle_get_64(); // I/O access off
		// }

		/***************************************************END OF CHECK FOR CONTEXT SWITCH LATENCY (START)*****/

		k_yield();

	}
}
