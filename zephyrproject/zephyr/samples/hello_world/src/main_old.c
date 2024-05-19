
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

void task1(){
    printf("TASK1 START cfs_reg[63]: [%x]\n", NEORV32_CFS->REG[63]);
    printf("CSR_MEPC [%x]\n", neorv32_cpu_csr_read(CSR_MEPC));
    for (int i = 65; i < 91; ++i) {
        printf("*********** [%c] \n", i);
    }
    printf("TASK1 END cfs_reg[63]: [%x]\n", NEORV32_CFS->REG[63]);
    printf("CSR_MEPC [%x]\n", neorv32_cpu_csr_read(CSR_MEPC));
    return;
}

//void task1() __attribute__((section(".task1")));

int main(void)
{

    printf("cfs_reg[0]: [%x]\n", NEORV32_CFS->REG[0]);
    printf("cfs_reg[1]: [%x]\n", NEORV32_CFS->REG[1]);

    printf("START cfs_reg[63]: [%x]\n", NEORV32_CFS->REG[63]);
	printf("CSR_MEPC [%x]\n", neorv32_cpu_csr_read(CSR_MEPC));
    printf("Hello World! %s\n", CONFIG_BOARD);

    //task1();

    printf("END cfs_reg[63]: [%x]\n", NEORV32_CFS->REG[63]);
    printf("CSR_MEPC [%x]\n", neorv32_cpu_csr_read(CSR_MEPC));
	return 0;
}
