/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

// (stevez)
#include <stdio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led1)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	int ret;

	// (stevez) for debugging purpose
	printf("blinky! %s\n", CONFIG_BOARD);

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}
	
	// (stevez) for debugging purpose
	printf("READY! %s\n", CONFIG_BOARD);

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	
	// (stevez) for debugging purpose
	printf("CONFIGURE! %s\n", CONFIG_BOARD);

	// (stevez) for testing purpose
	int counter = 0;
	while (counter < 50){
		printf("Line S %s\n", CONFIG_BOARD);
		ret = gpio_pin_toggle_dt(&led);
		// printf("Line 2 %s\n", CONFIG_BOARD);
		if(ret!=0){
			printf("Return %s\n", CONFIG_BOARD);
			return 0;
		}
		// printf("Line 3 %s\n", CONFIG_BOARD);
		// k_msleep(SLEEP_TIME_MS);
		counter+=1;
		printf("Line E %s\n", CONFIG_BOARD);
	}
	printf("DONE! %s\n", CONFIG_BOARD);

	while (1) {
		printf("Line 1 %s\n", CONFIG_BOARD);
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			printf("Return %s\n", CONFIG_BOARD);
			return 0;
		}
		// k_msleep(SLEEP_TIME_MS);
		printf("Line 2 %s\n", CONFIG_BOARD);

	}
	return 0;
}
