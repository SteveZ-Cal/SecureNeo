#include <zephyr/kernel.h>
#include <zephyr/device.h> 
#include <zephyr/drivers/gpio.h>
#include <stdio.h>

#include <zephyr/irq.h> // zephyr interrupt 

#include "tasks/task1.h"
#include "tasks/task2.h"
#include "tasks/task3.h"

// /* additional tasks up to 10tasks */
// #include "tasks/task4.h"
// #include "tasks/task5.h"
// #include "tasks/task6.h"
// #include "tasks/task7.h"
// #include "tasks/task8.h"
// #include "tasks/task9.h"
// #include "tasks/task10.h"

#ifndef NEORV32_PACKAGE
#define NEORV32_PACKAGE
#include "neorv32_library/neorv32_package.h"
#endif 


/**
 * @file
 *
 * two-task on DE0-CV NEORV32
 *	CFS_REG[] use --> REG[0] (current thread ID input from OS), REG[1] (whether the task should be block from NEORV32), REG[2] (status of thread1), REG[3] (status of thread2)
 */


/** For Testing Purpose REG[5] (will store the PC for NEORV32) **/

// static const struct device *mtimer = DEVICE_DT_GET(DT_NODELABEL(mtimer));

// evluation for timing
// timing_t start_time, end_time;
// uint64_t total_cycles;
//uint64_t total_ns;

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* dummy variable for set highest priority so all thread get timeslice schdule*/
#define HIGHEST_PRIORITY 10

/*time_slice for preemptive scheduling*/
#define TIME_SLICE 10000 // in (ms) --> roughly around 10 seconds

/* Try to Manually Memory Allocate the User Tasks to the Corresponding Address in NEORV32 */
// #define task1_data ((task1_data_struct*) (TASK1_BASE))

K_THREAD_STACK_DEFINE(task1_stack_area, STACKSIZE);
static struct k_thread task1_data;

K_THREAD_STACK_DEFINE(task2_stack_area, STACKSIZE);
static struct k_thread task2_data;

/***** (stevez) 9/28/2023 add another task mainly for the test_status_table purpose *****/
K_THREAD_STACK_DEFINE(task3_stack_area, STACKSIZE);
static struct k_thread task3_data;

// K_THREAD_STACK_DEFINE(task4_stack_area, STACKSIZE);
// static struct k_thread task4_data;

// /***** (stevez) 9/29/2023 add task0 for the test_status_table purpose *****/
// // K_THREAD_STACK_DEFINE(task0_stack_area, STACKSIZE);
// // static struct k_thread task0_data;

// K_THREAD_STACK_DEFINE(task5_stack_area, STACKSIZE);
// static struct k_thread task5_data;

// K_THREAD_STACK_DEFINE(task6_stack_area, STACKSIZE);
// static struct k_thread task6_data;

// K_THREAD_STACK_DEFINE(task7_stack_area, STACKSIZE);
// static struct k_thread task7_data;

// K_THREAD_STACK_DEFINE(task8_stack_area, STACKSIZE);
// static struct k_thread task8_data;

// K_THREAD_STACK_DEFINE(task9_stack_area, STACKSIZE);
// static struct k_thread task9_data;

// K_THREAD_STACK_DEFINE(task10_stack_area, STACKSIZE);
// static struct k_thread task10_data;

/***********END OF IRQ REFERENCE******/


/********* Define IRQ ***********/
#define MY_DEV_IRQ 17
#define MY_DEV_IRQ_PRIO 1
#define MY_ISR_ARG 0
#define MY_IRQ_FLAGS 0

/*******Define how ISR handle Interrupt ********/

//volatile uint32_t *mip_register = (volatile uint32_t *)MIP_BASE; // Machine Interrupt Register

volatile uint32_t *PC = (volatile uint32_t *)NEORV32_DATA_BASE; 
volatile uint32_t *STATE = (volatile uint32_t *)(NEORV32_DATA_BASE+4); 

void my_isr(void *arg){

  // verbose = 1; --> for debugging

  // if(latencyEnable)
  //   timing_start();

  // if(latencyEnable)
  //   start_time = timing_counter_get(); // start of evluating latency for handling interrupt from zephyr
  
  printf("\n<<<<<ENTER MY_ISR>>>>>.........................................\n");

  if(irq_timing){
    // set up the irq initialization timer
    neorv32_gptmr_enable(); // enable timer
    neorv32_gptmr_restart(); // restart timer

    neorv32_gptmr_setup(irq_clock_prescaler, 0, irq_clock_prescaler_threshold); // main processor clock (clock_prescaler) division by 4096, timer operates in single shot mode, threshold = 1000

    irq_start = neorv32_gptmr_curtime();

    printf("IRQ Start Time: [%d]\n", irq_start);
  }
  
  // if(latencyEnable){
  //   latencyStart = NEORV32_CFS->REG[NEORV32_TIMER_REG];
  //   //latencyStart = neorv32_mtime_get_time();
  //   //latencyStart = sys_clock_cycle_get_64(); //  start of evluating latency for handling interrupt from zephyr
  // }
  
  unsigned int sp;

  __asm__ __volatile__ (
      "auipc %0, 0\n"    
      //"jalr %0, %0, 4"   
      : "=r" (NEORV32_CFS->REG[INT_PC_BACKWARD_REG]) // store the interrupt jump PC from zephyr to neorv32 task0
  );

  // if(verbose){
  //   printf("my_isr:\n");
  //   printf("  NEORV32_CFS->REG[5]: [%x]\n", NEORV32_CFS->REG[INT_PC_BACKWARD_REG]);
  //   printf("  NEORV32_CFS->REG[6]: [%x]\n", NEORV32_CFS->REG[INT_FLAG_REG]);
  // }
  
  if(NEORV32_CFS->REG[INT_FLAG_REG]!=INT_FLAG_REG_UNSET){
    
    neorv32_cpu_csr_clr(CSR_MIP, 1 << CFS_FIRQ_PENDING);
    
    void (*task_pnt)(void);
    task_pnt = (void*)NEORV32_CFS->REG[INT_PC_FORWARD_REG]; 

    // save context, do not backup x0 and sp
    __asm__ volatile (
    #ifndef __riscv_32e
      "addi sp, sp, -30*4 \n"
    #else
      "addi sp, sp, -14*4 \n"
    #endif
      "sw x1,   0*4(sp) \n"
      "sw x3,   1*4(sp) \n"
      "sw x4,   2*4(sp) \n"
      "sw x5,   3*4(sp) \n"
      "sw x6,   4*4(sp) \n"
      "sw x7,   5*4(sp) \n"
      "sw x8,   6*4(sp) \n"
      "sw x9,   7*4(sp) \n"
      "sw x10,  8*4(sp) \n"
      "sw x11,  9*4(sp) \n"
      "sw x12, 10*4(sp) \n"
      "sw x13, 11*4(sp) \n"
      "sw x14, 12*4(sp) \n"
      "sw x15, 13*4(sp) \n"
    #ifndef __riscv_32e
      "sw x16, 14*4(sp) \n"
      "sw x17, 15*4(sp) \n"
      "sw x18, 16*4(sp) \n"
      "sw x19, 17*4(sp) \n"
      "sw x20, 18*4(sp) \n"
      "sw x21, 19*4(sp) \n"
      "sw x22, 20*4(sp) \n"
      "sw x23, 21*4(sp) \n"
      "sw x24, 22*4(sp) \n"
      "sw x25, 23*4(sp) \n"
      "sw x26, 24*4(sp) \n"
      "sw x27, 25*4(sp) \n"
      "sw x28, 26*4(sp) \n"
      "sw x29, 27*4(sp) \n"
      "sw x30, 28*4(sp) \n"
      "sw x31, 29*4(sp) \n"
    #endif
    );
    __asm__ volatile("mv %0, sp" : "=r" (NEORV32_CFS->REG[7]));
    //return;

  // if(latencyEnable)
  //   end_time = timing_counter_get(); 
    
  // if(latencyEnable) 
  //   total_cycles = timing_cycles_get(&start_time, &end_time);
  
  // if(latencyEnable){
  //   latencyEnd = NEORV32_CFS->REG[NEORV32_TIMER_REG];
  //   //latencyEnd = neorv32_mtime_get_time();
  //   //latencyEnd = sys_clock_cycle_get_64();
  // }
    
  // //total_ns = timing_cycles_to_ns(total_cycles);

  // // if(latencyVerbose)
  // //   printf("@@@@@@@@@@@@@@@@@ Latency of Receiving Interrupt @@@@@@@@@@@@@@@@@: [%lu]\n", total_cycles);

  // if(latencyVerbose)
  //   printf("@@@@@@@@@@@@@@@@@ Latency of Receiving Interrupt @@@@@@@@@@@@@@@@@: [%u]\n", cpu_get_mycycle(latencyStart, latencyEnd));

  // // if(latencyEnable)
	// // 	neorv32_mtime_set_time(0);

  // if(latencyVerbose) 
  //   printf("Zephyr (interrupt): Latency Start [%u]\n", latencyStart);
  // if(latencyVerbose)
  //   printf("Zephyr (interrupt): Latency End [%u]\n", latencyEnd);
  // if(latencyEnable)
  //   timing_stop();

  if(irq_timing){
    irq_end = neorv32_gptmr_curtime();
    printf("IRQ End Time: [%d]\n", irq_end);
    irq_cpu_time_used = irq_end - irq_start; 
    printf("IRQ Time Used: [%d]\n", irq_cpu_time_used);
    neorv32_gptmr_disable(); // disable timer
  }

    (*task_pnt)();
  }

  NEORV32_CFS->REG[INT_FLAG_REG] = INT_FLAG_REG_SET;

  // restore context, do not restore x0 and sp
  __asm__ volatile("mv sp, %0" : : "r" (NEORV32_CFS->REG[7]));
  __asm__ volatile (
    "lw x1,   0*4(sp) \n"
    "lw x3,   1*4(sp) \n"
    "lw x4,   2*4(sp) \n"
    "lw x5,   3*4(sp) \n"
    "lw x6,   4*4(sp) \n"
    "lw x7,   5*4(sp) \n"
    "lw x8,   6*4(sp) \n"
    "lw x9,   7*4(sp) \n"
    "lw x10,  8*4(sp) \n"
    "lw x11,  9*4(sp) \n"
    "lw x12, 10*4(sp) \n"
    "lw x13, 11*4(sp) \n"
    "lw x14, 12*4(sp) \n"
    "lw x15, 13*4(sp) \n"
    #ifndef __riscv_32e
      "lw x16, 14*4(sp) \n"
      "lw x17, 15*4(sp) \n"
      "lw x18, 16*4(sp) \n"
      "lw x19, 17*4(sp) \n"
      "lw x20, 18*4(sp) \n"
      "lw x21, 19*4(sp) \n"
      "lw x22, 20*4(sp) \n"
      "lw x23, 21*4(sp) \n"
      "lw x24, 22*4(sp) \n"
      "lw x25, 23*4(sp) \n"
      "lw x26, 24*4(sp) \n"
      "lw x27, 25*4(sp) \n"
      "lw x28, 26*4(sp) \n"
      "lw x29, 27*4(sp) \n"
      "lw x30, 28*4(sp) \n"
      "lw x31, 29*4(sp) \n"
    #endif
    #ifndef __riscv_32e
      "addi sp, sp, +30*4 \n"
    #else
      "addi sp, sp, +14*4 \n"
    #endif
   // "ret              \n"
  );

  printf("\n>>>>>EXIT MY_ISR<<<<<<*****************************************\n");
  printf("Machine exception program counter[%x]\n", neorv32_cpu_csr_read(CSR_MEPC));

}

void my_isr_installer(void){
	IRQ_CONNECT(MY_DEV_IRQ, MY_DEV_PRIO, my_isr, MY_ISR_ARG, MY_IRQ_FLAGS);
  irq_enable(MY_DEV_IRQ);
}

void timeslice_callback(struct k_thread *thread, void *data){
  printf("Time Slice ISR\n");
}

/*** Assign my_isr (confired section name in linker.ld include/zephyr/arch/riscv/common/linker.ld) ***/
void my_isr(void *arg) __attribute__ ((section(".my_isr")));

/*** Assign the Desired Tasks (confired section name in linker.ld include/zephyr/arch/riscv/common/linker.ld) ***/

void task1() __attribute__((section(".task1")));
void neorv32_gpio_raw1_v2() __attribute__((section(".task1")));

void task2() __attribute__((section(".task2")));
void neorv32_gpio_raw2_v2() __attribute__((section(".task2")));

void task3() __attribute__((section(".task3")));
void neorv32_gpio_raw3_v2() __attribute__((section(".task3")));

// // (stevez) 11/27/2023 I-O-Guard (at most 10 tasks)

// void task4() __attribute__((section(".task4")));
// void neorv32_gpio_raw4_v2() __attribute__((section(".task4")));

// void task5() __attribute__((section(".task5")));
// void neorv32_gpio_raw5_v2() __attribute__((section(".task5")));

// void task6() __attribute__((section(".task6")));
// void neorv32_gpio_raw6_v2() __attribute__((section(".task6")));

// void task7() __attribute__((section(".task7")));
// void neorv32_gpio_raw7_v2() __attribute__((section(".task7")));

// void task8() __attribute__((section(".task8")));
// void neorv32_gpio_raw8_v2() __attribute__((section(".task8")));

// void task9() __attribute__((section(".task9")));
// void neorv32_gpio_raw9_v2() __attribute__((section(".task9")));

// void task10() __attribute__((section(".task10")));
// void neorv32_gpio_raw10_v2() __attribute__((section(".task10")));


/*** End of Assign the Desired Tasks ***/



void main(void){
  
  // timing_init();
  
  // if(latencyEnable)
  //   timing_start();
  printf("START IN MAIN\n");

  if(zephyr_init_timing){
    // set up the zephyr initialization timer
    neorv32_gptmr_enable(); // enable timer
    neorv32_gptmr_restart(); // restart timer

    neorv32_gptmr_setup(zephyr_init_clock_prescaler, 0, zephyr_init_clock_prescaler_threshold); // main processor clock (clock_prescaler) division by 4096, timer operates in single shot mode, threshold = 1000

    zephyr_init_start = neorv32_gptmr_curtime();

    printf("Zephyr Initialization Start Time: [%d]\n", zephyr_init_start);
  }
  
  // if(latencyEnable)
  //   start_time = timing_counter_get(); // start of zephyr os booting

  // if(latencyEnable){
  //   latencyStart = NEORV32_CFS->REG[NEORV32_TIMER_REG];
  //   //latencyStart = neorv32_mtime_get_time();
  //   //latencyStart = sys_clock_cycle_get_64(); // start of zephyr os booting
  // }
	
  // if(verbose)
	//   printf("MAIN: Before cfs_reg[0]: [%u]\n", NEORV32_CFS->REG[0]);
	// //NEORV32_CFS->REG[0] = 2;
  // if(verbose)
	//   printf("MAIN: After cfs_reg[0]: [%u]\n", NEORV32_CFS->REG[0]);
  // if(verbose)
	//   printf("MAIN: Before cfs_reg[1]: [%u]\n", NEORV32_CFS->REG[1]);
	// //NEORV32_CFS->REG[1] = 3;
  // if(verbose)
	//   printf("MAIN: After cfs_reg[1]: [%u]\n", NEORV32_CFS->REG[1]);

  // if(verbose)
  //   printf("MAIN: cfs_reg[0]: [%x]\n", NEORV32_CFS->REG[0]);
  // if(verbose)
  //   printf("MAIN: cfs_reg[1]: [%x]\n", NEORV32_CFS->REG[1]);

	my_isr_installer();
  // printf("NO IRQ Installed in ZEPHYR OS\n");

	/* please enable the CFS first lol => IO_CFS_EN = true in FPGA TOP */    

	/****************Initilizing Threads**************************/

	k_tid_t task1_id = k_thread_create(&task1_data, task1_stack_area, K_THREAD_STACK_SIZEOF(task1_stack_area), 
					task1, NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);
	k_thread_name_set(task1_id, "task1");

	k_tid_t task2_id = k_thread_create(&task2_data, task2_stack_area, K_THREAD_STACK_SIZEOF(task2_stack_area), 
					task2, NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);
	k_thread_name_set(task2_id, "task2");

	/***** (stevez) 9/28/2023 add another task mainly for the test_status_table purpose *****/
	k_tid_t task3_id = k_thread_create(&task3_data, task3_stack_area, K_THREAD_STACK_SIZEOF(task3_stack_area), 
					task3, NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);
	k_thread_name_set(task3_id, "task3");

  // k_tid_t task4_id = k_thread_create(&task4_data, task4_stack_area, K_THREAD_STACK_SIZEOF(task4_stack_area), 
	// 				task4, NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);
	// k_thread_name_set(task4_id, "task4");

  // k_tid_t task5_id = k_thread_create(&task5_data, task5_stack_area, K_THREAD_STACK_SIZEOF(task5_stack_area), 
	// 				task5, NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);
	// k_thread_name_set(task5_id, "task5");

  // k_tid_t task6_id = k_thread_create(&task6_data, task6_stack_area, K_THREAD_STACK_SIZEOF(task6_stack_area), 
	// 				task6, NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);
	// k_thread_name_set(task6_id, "task6");

  // k_tid_t task7_id = k_thread_create(&task7_data, task7_stack_area, K_THREAD_STACK_SIZEOF(task7_stack_area), 
	// 				task7, NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);
	// k_thread_name_set(task7_id, "task7");

  // k_tid_t task8_id = k_thread_create(&task8_data, task8_stack_area, K_THREAD_STACK_SIZEOF(task8_stack_area), 
	// 				task8, NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);
	// k_thread_name_set(task8_id, "task8");

  // k_tid_t task9_id = k_thread_create(&task9_data, task9_stack_area, K_THREAD_STACK_SIZEOF(task9_stack_area), 
	// 				task9, NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);
	// k_thread_name_set(task9_id, "task9");

  // k_tid_t task10_id = k_thread_create(&task10_data, task10_stack_area, K_THREAD_STACK_SIZEOF(task10_stack_area), 
	// 				task10, NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);
	// k_thread_name_set(task10_id, "task10");

	/**************** Set TimeSlice for Each Task **************************/

  // k_thread_deadline_set(task1_id, 1000);
  // k_thread_deadline_set(task2_id, 1000);
  // k_thread_deadline_set(task3_id, 1000);

  // k_sched_time_slice_set(TIME_SLICE, HIGHEST_PRIORITY);

  // k_thread_time_slice_set(&task1_data, k_ms_to_ticks_ceil32(TIME_SLICE), timeslice_callback, NULL);
  // k_thread_time_slice_set(&task2_data, k_ms_to_ticks_ceil32(TIME_SLICE), timeslice_callback, NULL);
  // k_thread_time_slice_set(&task3_data, k_ms_to_ticks_ceil32(TIME_SLICE), timeslice_callback, NULL);

	/***** (stevez) 9/29/2023 add another task mainly for the test_status_table purpose *****/
	// k_tid_t task0_id = k_thread_create(&task0_data, task0_stack_area, K_THREAD_STACK_SIZEOF(task0_stack_area), 
	// 				task0, NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);
	// k_thread_name_set(task0_id, "task0");

	/**************** Start of Main Thread **************************/

	//printf("Assign Table ...\n");
	// NEORV32_CFS->REG[1] = 0; // store as malicious for thread 1 in the CFS REG[1]
	// NEORV32_CFS->REG[2] = 0; // store as normal for thread 2 in the CFS REG[2]
	// NEORV32_CFS->REG[3] = 0; // store as normal for thread 3 in the CFS REG[3]

	/**************END of Main Thread***********************/

	/*************Start Task1 to Task3***********************/
	

  // if(latencyEnable)
  //   end_time = timing_counter_get(); 

  // if(latencyEnable) 
  //   total_cycles = timing_cycles_get(&start_time, &end_time);

  // if(latencyEnable){
  //   latencyEnd = NEORV32_CFS->REG[NEORV32_TIMER_REG];
  //   //latencyEnable = neorv32_mtime_get_time();
  //   //latencyEnd = sys_clock_cycle_get_64(); 
  // }
    
  // //total_ns = timing_cycles_to_ns(total_cycles);
  
  // // if(latencyVerbose)
  // //   printf("@@@@@@@@@@@@@@@@@ Latency of Zephyr OS Booting @@@@@@@@@@@@@@@@@: [%lu]\n", total_cycles);
  // if(latencyVerbose)
  //   printf("@@@@@@@@@@@@@@@@@ Latency of Zephyr OS Booting @@@@@@@@@@@@@@@@@: [%u]\n", cpu_get_mycycle(latencyStart, latencyEnd));

  // // if(latencyEnable)
	// // 	neorv32_mtime_set_time(0);

  // if(latencyVerbose) 
  //   printf("Zephyr (booting): Latency Start [%u]\n", latencyStart);
  // if(latencyVerbose)
  //   printf("Zephyr (booting): Latency End [%u]\n", latencyEnd);

  if(zephyr_init_timing){
    zephyr_init_end = neorv32_gptmr_curtime();
    printf("Zephyr Initialization End Time: [%d]\n", zephyr_init_end);
    zephyr_init_cpu_time_used = zephyr_init_end - zephyr_init_start; 
    printf("Zephyr Initialization Time Used: [%d]\n", zephyr_init_cpu_time_used);
    neorv32_gptmr_disable(); // disable timer
  }

  // if(latencyEnable)
  //   timing_stop();
  printf("Start ALL KERNEL TASK\n");
	// k_thread_start(task0_id);
	k_thread_start(task1_id);
	k_thread_start(task2_id);
	k_thread_start(task3_id);
  // k_thread_start(task4_id);
	// k_thread_start(task5_id);
	// k_thread_start(task6_id);
  // k_thread_start(task7_id);
	// k_thread_start(task8_id);
	// k_thread_start(task9_id);
  // k_thread_start(task10_id);
	
}
