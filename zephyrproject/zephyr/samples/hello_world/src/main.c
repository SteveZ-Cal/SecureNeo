/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h> 
#include <zephyr/drivers/gpio.h>
#include <stdio.h>

#ifndef NEORV32_PACKAGE
#define NEORV32_PACKAGE
#include "neorv32_package.h"
#endif 

int main(void)
{
	uint32_t task_to_wait = 1000000;
	while(1){
		printf("\nEntering MAIN ...\n");
    	printf("  	NEORV32_CFS->REG[0]: [%x]\n", NEORV32_CFS->REG[TASK0_STATE_REG]);
    	printf("  	NEORV32_CFS->REG[1]: [%x]\n", NEORV32_CFS->REG[TASK_NUM_REG]);
    	printf("  	NEORV32_CFS->REG[2]: [%x]\n", NEORV32_CFS->REG[CFS_CTRL_REG]);
    	printf("  	NEORV32_CFS->REG[3]: [%x]\n", NEORV32_CFS->REG[CFS_UPDATE_REG]);
		printf("	MTVEC: [%x]\n", neorv32_cpu_csr_read(CSR_MTVEC));
    	printf("	MEPC:  [%x]\n", neorv32_cpu_csr_read(CSR_MEPC));
		printf("Hello World! %s\n", CONFIG_BOARD);
		k_busy_wait(task_to_wait);
	}
	
	return 0;
}