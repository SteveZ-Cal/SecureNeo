#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <neorv32.h>

#ifndef BOOTLOADER_PACKAGE
#define BOOTLOADER_PACKAGE
#include "bootloader_package.h"
#endif 


/****************SPI Interrupt Handler*************/

void cfs_irq_handler(void);

/*****************I-O-Guard_task0****************/

void Boot_OS(void);

void Get_PT(void); // Get the binary file through UART (from server)

// cfs_reg => 32bits => 4B
// each task requires => 5 cfs_reg

void Parsing_Bin(); // name 2B, activity 2B, access 8B, time 8B => for each Task, HMAC 20B => one at the end

void Check_HMAC(); // Recalcualte the HMAC

void CFS_Write(uint32_t num_tasks); // If HMAC is correct, write the policy table on the CFS regs

void Send_Ack(); // Write on the terminal through UART (!)

/***********/

void cfs_irq_handler(void){

  neorv32_uart0_printf("Entered IRQ Handler ++++++++++++++++++++++++ \n");

  neorv32_cpu_csr_clr(CSR_MIP, 1 << CFS_FIRQ_PENDING);

  neorv32_uart0_printf("Exit    IRQ Handler ++++++++++++++++++++++++ \n");
}

void cfs_irq_handler_old(void){

//void __attribute__((naked)) cfs_irq_handler(void){

  unsigned int ra_value;
  __asm__ volatile ("mv %0, ra" : "=r" (ra_value));
  neorv32_uart0_printf("cfs_irq_handler [ra]: [%x]\n", ra_value);
  NEORV32_CFS->REG[4] = ra_value;

  //NEORV32_CFS->REG[0] == 1;

  // neorv32_uart0_printf("Inside NEORV32 CFS_IRQ_HANDLER \n");
  
  // call task program
	// uint32_t task_program_base_addr = TASK0_BASE_ADDR;
	// void (*task_pnt)(void);
	// task_pnt= (void*)task_program_base_addr;
	// (*task_pnt)();

  // print project logo via UART (testing purpose)

  neorv32_uart0_printf("-----Enter NEORV32 CFS_IRQ_HANDLER-----\n");

  //if(verbose){
    //  neorv32_rte_print_logo();
    // neorv32_uart0_printf("Machine trap-handler base address[%x]\n", neorv32_cpu_csr_read(CSR_MTVEC));
    // neorv32_uart0_printf("Machine exception program counter[%x]\n", neorv32_cpu_csr_read(CSR_MEPC));
  //}

  if(verbose)
    neorv32_uart0_printf("Before Jump --> [CSR_MEPC]: [%x]\n", neorv32_cpu_csr_read(CSR_MEPC));
  
  uint32_t task_program_base_addr = NEORV32_IMEM_BASE;

  //uint32_t task_program_base_addr = 0x00002000U;
  void (*task_pnt)(void);
  task_pnt = (void*)task_program_base_addr;

  // uint32_t pc;

  // __asm__ __volatile__ (
  //     "auipc %0, 0\n"    // Load the upper 20 bits of PC into %0
  //     "jalr %0, %0, 4"   // Add the lower 12 bits of PC to %0
  //     : "=r" (pc)
  // );
  
  if(verbose)
    neorv32_uart0_printf("[SHT0]********************\n");

  unsigned int sp;

  unsigned int pc;

  __asm__ __volatile__ (
      "auipc %0, 0\n"    // Load the upper 20 bits of PC into %0
      //"jalr %0, %0, 4"   // Add the lower 12 bits of PC to %0
      : "=r" (pc)
  );

  

  if(NEORV32_CFS->REG[3]!=1){

    NEORV32_CFS->REG[5] = pc;
    
    if(verbose)
      neorv32_uart0_printf("Before Jump --> PC [%x]\n", pc);

    if(verbose)
      neorv32_uart0_printf("[FUCK0]###################\n");
    
    neorv32_cpu_csr_clr(CSR_MIP, 1 << CFS_FIRQ_PENDING);

    if(verbose)
      neorv32_uart0_printf("Tempatory Exit\n");

    //return;

    __asm__ volatile ("mv %0, sp" : "=r" (sp));
    neorv32_uart0_printf("Start of cfs_irq_handler [sp]: [%x]\n", sp);

    // save context, do not backup x0 and sp
    asm volatile (
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
    asm volatile("mv %0, sp" : "=r" (NEORV32_CFS->REG[6]));

    //return;
    (*task_pnt)();

  }

  // neorv32_uart0_printf("Restore Context, Don't Restore x0 and sp\n");

  // unsigned int ra_val;
  // __asm__ volatile ("mv %0, ra" : "=r" (ra_val));
  // neorv32_uart0_printf("cfs_irq_handler (ready to jump back) [ra]: [%x]\n", ra_val);
  // neorv32_uart0_printf("cfs_irq_handler (original) [ra]: [%x]\n", NEORV32_CFS->REG[4]);

  __asm__ volatile ("mv %0, sp" : "=r" (sp));
  neorv32_uart0_printf("Mid of cfs_irq_handler [sp]: [%x]\n", sp);

  // restore context, do not restore x0 and sp
  asm volatile("mv sp, %0" : : "r" (NEORV32_CFS->REG[6]));
  asm volatile (
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

  __asm__ volatile ("mv %0, sp" : "=r" (sp));
  neorv32_uart0_printf("END of cfs_irq_handler [sp]: [%x]\n", sp);

  unsigned int ra;
  __asm__ volatile ("mv %0, ra" : "=r" (ra));
  neorv32_uart0_printf("END of cfs_irq_handler [ra]: [%x]\n", ra);

//   __asm__ __volatile__ (
//     "jalr %0, %1, 0"
//     : "=r" (pc + 4)
//     : "r" (task_pnt)
// );

  NEORV32_CFS->REG[3] = 0;

  //asm volatile ("li t0, %[input_i]; jr t0" :  : [input_i] "i" (0x0000f000U));
  //asm volatile ("jalr ra, %0" : : "r" (0x0000f000U)); 

  neorv32_uart0_printf(">>>>>Exit NEORV32 CFS_IRQ_HANDLER<<<<<\n");

  // unsigned int ra;
  // __asm__ volatile ("mv %0, ra" : "=r" (ra));
  // neorv32_uart0_printf("END of cfs_irq_handler [ra]: [%x]\n", ra);

  //NEORV32_CFS->REG[0] == -1;
  // asm volatile (
  //       "mv ra, %0"   // Move the value into ra (x1)
  //       :
  //       : "r" (neorv32_cpu_csr_read(CSR_MEPC))
  // );
  neorv32_uart0_printf("END of cfs_irq_handler [CSR_MEPC]: [%x]\n", neorv32_cpu_csr_read(CSR_MEPC));

  // uint32_t csr_mepc_base_addr = neorv32_cpu_csr_read(CSR_MEPC);
  // void (*task_pnt_rtn)(void);
  // task_pnt_rtn = (void*)csr_mepc_base_addr;
  // (*task_pnt_rtn)();

  return;

}


/*********BOOTLOADER PROTOTYPE (referenced based on Stephan Nolting)**********/ 

void Boot_OS(void){ // run one-time at the start of the booting, Upload main.c on 0x0000


  exe_available = 0; // global variable for executable size; 0 means there is no exe available

  while (1) {
    
    // neorv32_rte_setup();
    // neorv32_rte_handler_install(CFS_RTE_ID, cfs_irq_handler); // CFS to RTE
    // neorv32_cpu_csr_set(CSR_MIE, 1 << CFS_FIRQ_ENABLE); // enable CFS FIRQ1
    // neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts;

    neorv32_uart0_printf("\nCMD (TASK0):> ");
    char c = neorv32_uart_getc(NEORV32_UART0);
    neorv32_uart_putc(NEORV32_UART0, c); // echo
    neorv32_uart0_printf("\n");

    // unsigned int sp;
    // __asm__ volatile ("mv %0, sp" : "=r" (sp));
    // if(verbose)
    //   neorv32_uart0_printf("Boot_OS [sp]: [%x]\n", sp);
    //neorv32_uart0_printf("SHT*******************SHT");

    if (c == 'r') { // restart bootloader
      NEORV32_CFS->REG[TASK0_STATE_REG] = 0; // reset the task0_state
      __asm__ volatile ("li t0, %[input_i]; jr t0" :  : [input_i] "i" (BOOTLOADER_BASE_ADDRESS)); // jump to beginning of boot ROM (update task0)
    }
    else if (c == 't') { // restart bootloader of task0 
      NEORV32_CFS->REG[TASK0_STATE_REG] = 0; // reset the task0_state
      __asm__ volatile ("li t0, %[input_i]; jr t0" :  : [input_i] "i" (TASK0_BASE_ADDR)); // jump to beginning of boot ROM (TASK0)
    }
    else if (c == 'h') { // help menu
      print_help();
    }
    else if (c == 'u') { // get executable via UART
      NEORV32_CFS->REG[TASK0_STATE_REG] += 1; // go to next state
      if(verbose){
        neorv32_uart0_printf("NEORV32_CFS->REG[0]: [%x]\n", NEORV32_CFS->REG[TASK0_STATE_REG]);
        neorv32_uart0_printf("NEORV32_CFS->REG[1]: [%x]\n", NEORV32_CFS->REG[TASK_NUM_REG]);
        neorv32_uart0_printf("NEORV32_CFS->REG[2]: [%x]\n", NEORV32_CFS->REG[CFS_CTRL_REG]);
        neorv32_uart0_printf("NEORV32_CFS->REG[3]: [%x]\n", NEORV32_CFS->REG[CFS_UPDATE_REG]);
        neorv32_uart0_printf("MTVEC: [%x]\n", neorv32_cpu_csr_read(CSR_MTVEC));
        neorv32_uart0_printf("MEPC:  [%x]\n", neorv32_cpu_csr_read(CSR_MEPC));
      }  
      get_exe(EXE_STREAM_UART);
    }
    else if (c == 'e') { // start application program from IMEM
      if (exe_available == 0) { // executable available?
        neorv32_uart0_printf("No executable.");
      }
      else {
        NEORV32_CFS->REG[CFS_CTRL_REG] = CFS_CTRL_REG_SET; // set, so Zephyr won't set all cfs_reg to 0
        NEORV32_CFS->REG[CFS_UPDATE_REG] = CFS_UPDATE_REG_UNSET; // unset, so the timer for policy table will activate
        start_app(); // run app from IMEM
      }
    }
    else if (c == '?') {
      neorv32_uart0_printf("(c) Inspired by Stephan Nolting\ngithub.com/stnolting/neorv32");
    }
    else { // unknown command
      neorv32_uart0_printf("Invalid CMD");
    }

  } // while(1)

}

/**********************************************************************//**
 * Upload Task_Status_Table binary file (.bin) to TASK_STATUS_TABLE_BASE (0x2F000).
 *
 **************************************************************************/
void Get_PT(void){ // update task_status_table at (0x0002F000U)

  neorv32_uart0_printf("\n***************************************************\n");
  neorv32_uart0_printf("Update Policy Table\n");
  neorv32_uart0_printf("***************************************************\n");
  
  NEORV32_CFS->REG[CFS_CTRL_REG] = CFS_CTRL_REG_UNSET; // unset, allow write 0 to cfs_reg since not by zephyr but by user/task0
  NEORV32_CFS->REG[CFS_UPDATE_REG] = CFS_UPDATE_REG_SET; // set, so the timer for policy table will deactivate 

  while (1) {
    
    // neorv32_rte_setup();
    // neorv32_rte_handler_install(CFS_RTE_ID, cfs_irq_handler); // CFS to RTE
    // neorv32_cpu_csr_set(CSR_MIE, 1 << CFS_FIRQ_ENABLE); // enable CFS FIRQ1
    // neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts;

    neorv32_uart0_printf("\nCMD (Task0 **Get_PT**):> ");
    char c = neorv32_uart_getc(NEORV32_UART0);
    neorv32_uart_putc(NEORV32_UART0, c); // echo
    neorv32_uart0_printf("\n");

    if (c == 'h') { // help menu
      print_help_PT();
    }
    else if (c == 'u') { // get executable via UART
      if(verbose){
        neorv32_uart0_printf("  NEORV32_CFS->REG[0]: [%x]\n", NEORV32_CFS->REG[TASK0_STATE_REG]);
        neorv32_uart0_printf("  NEORV32_CFS->REG[1]: [%x]\n", NEORV32_CFS->REG[TASK_NUM_REG]);
        neorv32_uart0_printf("  NEORV32_CFS->REG[2]: [%x]\n", NEORV32_CFS->REG[CFS_CTRL_REG]);
        neorv32_uart0_printf("  NEORV32_CFS->REG[3]: [%x]\n", NEORV32_CFS->REG[CFS_UPDATE_REG]);
      }
      get_exe_PT(EXE_STREAM_UART);
    }
    else if(c == 'q'){
      break;
    }
    else { // unknown command
      neorv32_uart0_printf("Invalid CMD");
    }

  }

}


/**********************************************************************//**
 * Write task_status table to CFS_REG.
 *
 * @param num_tasks total number of tasks on run or in task_status table.
 **************************************************************************/
void CFS_Write(uint32_t num_tasks){

    if(num_tasks > MAX_NUM_TASKS){
        neorv32_uart0_printf("******Exceed Maximum Number of Tasks!!******\n");
        while(1);
        return;
    }

    uint32_t TASKN_ENDING_REG = TASK1_STARTING_REG + 5*num_tasks; 

    uint32_t *pnt = (uint32_t*)TASK_STATUS_TABLE_BASE;

    //uint32_t task_status_table_index = 0;

    for(int i=TASK1_STARTING_REG; i <TASKN_ENDING_REG; i++){

      NEORV32_CFS->REG[i] = pnt[i-TASK1_STARTING_REG];
      
      if(verbose)
        neorv32_uart0_printf("REG [%u]: [%x]\n", i, NEORV32_CFS->REG[i]); // for debugging purpose check binary content of policy table

    }
    
    NEORV32_CFS->REG[CFS_CTRL_REG] = CFS_CTRL_REG_SET; // set, so Zephyr won't set all cfs_reg to 0
    NEORV32_CFS->REG[CFS_UPDATE_REG] = CFS_UPDATE_REG_UNSET; // unset, so the timer for policy table will activate 

}