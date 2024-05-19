#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>

void neorv32_gpio_raw(struct device *gpio_ct_dev_task, uint32_t gpio_pin, uint32_t task_to_wait, uint32_t num_iteration){

    int ret;

	// /** ADD PC FETCH **/
	// uint32_t PC_fetch = NEORV32_CFS->REG[NEORV32_PC_REG];
	// printf("PC Fetch at NEORV32_GPIO_RAW: [%x]\n", PC_fetch);
    
    for (uint32_t i = 0; i < num_iteration; i++){

        ret = gpio_pin_set_raw(gpio_ct_dev_task, gpio_pin, 1);
		if (ret!=0){
			return;
		}

		// /** ADD PC FETCH **/
		// uint32_t PC_fetch1 = NEORV32_CFS->REG[NEORV32_PC_REG];
		// printf("PC Fetch at gpio_pin_set_raw: [%x]\n", PC_fetch1);

		k_busy_wait(task_to_wait);

		// /** ADD PC FETCH **/
		// uint32_t PC_fetch2 = NEORV32_CFS->REG[NEORV32_PC_REG];
		// printf("PC Fetch at k_busy_wait: [%x]\n", PC_fetch2);

		ret = gpio_pin_set_raw(gpio_ct_dev_task, gpio_pin, 0);
		if (ret!=0){
			return;
		}

		k_busy_wait(task_to_wait);

    }

}

