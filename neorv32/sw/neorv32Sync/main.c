#ifndef UTILS
#define UTILS
#include "util.h"
#endif 

#include <neorv32.h>
//#include <string.h>

/**********TASK0 **************/

void task0(void){

  /*******Determine the Current State of the task0********/

  neorv32_uart0_printf("\n**************** Entering TASK0 **************\n");
  neorv32_uart0_printf("TASK0_STATE:    [%x]\n", NEORV32_CFS->REG[TASK0_STATE_REG]);


  // STATE = 0 ==> Initialization and Boot the Zephyr OS
  if (NEORV32_CFS->REG[TASK0_STATE_REG] == TASK0_STATE_REG_BOOTING){
    
    if(task0_init_timing){
      // set up the task0 initialization timer
      neorv32_gptmr_enable(); // enable timer
      neorv32_gptmr_restart(); // restart timer

      neorv32_gptmr_setup(task0_init_clock_prescaler, 0, task0_init_clock_prescaler_threshold); // main processor clock (clock_prescaler) division by 4096, timer operates in single shot mode, threshold = 1000

      task0_init_start = neorv32_gptmr_curtime();
    }

    /*******Initial Enviornmental Setup from Stnolting*****/

    // capture all exceptions and give debug infro via UART
    // this is not required, but keeps us safe
    if(enable)
      neorv32_rte_setup();

    // setup UART at default baud rate, no interrupts
    if(enable)
      neorv32_uart0_setup(BAUD_RATE, 0);

    // check if UART0 uint is implemented at all
    if(neorv32_uart0_available() == 0){
      return 1;
    }

    /*******End of Initial Enviornmental Setup*******/

    // intro
    if (verbose) {
      neorv32_uart0_printf("Task0 BEGIN!!\n");
      neorv32_uart0_printf("Start Dynamic Task Status\n");
    }
      
    // check if CFS unit is implemented at all
    if (neorv32_cfs_available() == 0){
      neorv32_uart0_printf("ERROR! No CFS unit implemented.");
      return 1;
    }

    // first install IRQ system for RTE_TRAP_FIRQ_1 or cfs_irq in the NEORV32 core
    if(enable){
      neorv32_rte_handler_install(CFS_RTE_ID, cfs_irq_handler); // CFS to RTE
      neorv32_uart0_printf("Installed CFS_IRQ_HANDLER in NEORV32\n");
    }
    
    // enable IRQ system (later to be installed in Zephyr)
    if(enable && verbose)
      neorv32_uart0_printf("Finish installing CFS_IRQ_HANDLER\n");
    if(enable)
      neorv32_cpu_csr_set(CSR_MIE, 1 << CFS_FIRQ_ENABLE); // enable CFS FIRQ1
    if(enable && verbose)
      neorv32_uart0_printf("Enabled CFS FIRQ1\n");
    if(enable)
      neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts;
    if(enable && verbose)
      neorv32_uart0_printf("Enabled machine-mode interrupts\n");
    
    /********First Time Initiate its Internal Bootloader*******/
    
    NEORV32_CFS->REG[TASK_NUM_REG] = 3; // number of tasks on run (Ex: up from 1 to 9)
        
    if(verbose)
      neorv32_uart0_printf("TASK0: Number of Tasks [%u]\n", NEORV32_CFS->REG[TASK_NUM_REG]);

    if(verbose)
      neorv32_uart0_printf("TASK0: Current State [%u]\n", NEORV32_CFS->REG[TASK0_STATE_REG]);

    if(task0_init_timing){
      task0_init_end = neorv32_gptmr_curtime();
      task0_init_cpu_time_used = task0_init_end - task0_init_start; 
      neorv32_uart0_printf("Task0 Initialization Time Used: [%d]\n", task0_init_cpu_time_used);
    }

    Boot_OS(); // NEORV32 RISC-V Processor will upload and execute I-O-Guard_main.c (Zephyr) at this point on

    /********END OF Initiate its Internal Bootloader*****/

  }
  else {
    
    // verbose = 1; --> for debugging
    
    // STATE = 1 ==> Interrupt from Zephyr to Perform Hybrid Attestation
    if (NEORV32_CFS->REG[TASK0_STATE_REG] == TASK0_STATE_REG_INTERRUPT){

        neorv32_uart0_printf("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");

        neorv32_uart0_printf("Ready to Hybrid Attestation\n");

        neorv32_uart0_printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n\n");

        // Get_PT(); // Get Policy Table

        // uint32_t num_tasks = NEORV32_CFS->REG[TASK_NUM_REG];

        // CFS_Write(num_tasks); // Upload Policy Table on Rom to cfs_reg


        /**********************************************************************//**
        * NEORV32 Hybrid Attestation Testing --------------------------------------------------- [2]
        **************************************************************************/

        uint32_t num_trials = 1; // 5 random IV tested
        uint32_t max_num_blocks_cal = 30000; // up from 1 to 10 blocks
        // see Table 35. GPTMR prescaler configuration for more information 
        uint32_t clock_prescaler = 7; // 7 represent 4096 --> subjected to change to fit for your data plot
        uint32_t clock_prescaler_num = 64; // (above)
        uint32_t clock_prescaler_threshold = 100000;  // (recommended) set it to as big as possible
        uint32_t hash_length_uint32 = 8;

        uint32_t mem_address = 0x80000000u; // (starting address of the memory to be tested)

        if(!hybrid_att_testing)
          mem_address = 0x00000000u; // (start from ROM)

        uint32_t IV_uint32_list[5][8] = {
              {0x3a5e2a96, 0xe1a4a803, 0x49d8c21b, 0xc8f41b70, 0x71eb9128, 0xf333762a, 0xe1659a54, 0xf946d5f5},
              {0x82db8f14, 0xef6f605e, 0xeedd2f5d, 0xb5976c1d, 0x0b76c2bf, 0x40cf60df, 0xc72bb5a4, 0xd032ea5c},
              {0x6b6be13d, 0xfbbd2c52, 0x5877f40e, 0x0d3d11c3, 0xe8cbcd4c, 0x7be04512, 0xb29c292a, 0x35c4b06e},
              {0x2e09484e, 0xfce0d97a, 0x811097a3, 0xc24e93fe, 0x4c6ebc8e, 0xe5f41a76, 0x8b1b0d4f, 0x3c96e830},
              {0x91802d09, 0x592ce705, 0xdab0be1f, 0x36db8e1e, 0x849a792e, 0xcce78417, 0xa9e072f2, 0xe4cb32d2}
        };        

        neorv32_uart0_printf("-------------Start of Hybrid Attestation-------------\n\n");

        neorv32_uart0_printf("RESULTING CLOCK PRE-SCALAR: [%d]\n\n", clock_prescaler_num);

        for (uint32_t trial_num = 0; trial_num < num_trials; trial_num++){

          neorv32_uart0_printf("[[[[[[TRIAL NUM %d]]]]]]\n\n", trial_num);

          neorv32_IV_uint32_print(IV_uint32_list[trial_num]);

          for (uint32_t this_num_blocks_cal = max_num_blocks_cal; this_num_blocks_cal <= max_num_blocks_cal; this_num_blocks_cal++){

            neorv32_uart0_printf("{Block Num %d}\n", this_num_blocks_cal);

            Hybrid_Attestation(this_num_blocks_cal, IV_uint32_list[trial_num], clock_prescaler, clock_prescaler_threshold, mem_address);

          }

          neorv32_uart0_printf("\n");

        }

        neorv32_uart0_printf("\n-------------End of Hybrid Attestation-------------\n\n");

        /*************** END of Hybrid Attestation **********/



        neorv32_uart0_printf("\n<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
        
        neorv32_uart0_printf("Ready to Jump Back to Zephyr\n");

        neorv32_uart0_printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");

        NEORV32_CFS->REG[INT_FLAG_REG] = INT_FLAG_REG_UNSET;

        void (*task_pnt)(void);
        task_pnt = (void*)NEORV32_CFS->REG[INT_PC_BACKWARD_REG];
        if(verbose){
          neorv32_uart0_printf("task0:\n");
          neorv32_uart0_printf("  NEORV32_CFS->REG[5]: [%x]\n", NEORV32_CFS->REG[INT_PC_BACKWARD_REG]);
          neorv32_uart0_printf("  NEORV32_CFS->REG[6]: [%x]\n", NEORV32_CFS->REG[INT_FLAG_REG]);
        }
        (*task_pnt)(); 

        //neorv32_uart0_printf("Build Unreachable\n");

        __builtin_unreachable();
        while (1); // should never be reached

        //__asm__ volatile ("li t0, %[input_i]; jr t0" :  : [input_i] "i" (ZEPHYR_MY_ISR_BASE)); // jump to beginning of boot ROM (update task0)

        //__builtin_unreachable();

        //asm volatile ("jalr ra, %0" : : "r" (firq_PC)); // jump to previous PC before interrupt

        /********TO DO: context switching security measure*******/

    }
    else if (NEORV32_CFS->REG[TASK0_STATE_REG] == TASK0_STATE_REG_CONTEXT_SWITCH){

        /********TO DO: Re upload New Task_Status_Table*******/

        /*********1. Receive Binary File from UART0 and Upload to TASK_STATUS_TABLE_BASE  (0x0002F000U) *****/

        //Get_PT(EXE_STREAM_UART);

        /********2. Extract the Variables from the Address and Put in an array? ******/

        //Parsing_Bin();

        /********3. Check the HMAC of the Task_Status_Table (for validation purpose) ******/

        //Check_HMAC();

        /********4. Write Task_Status_Table content to CFS_REG ******/

        //CFS_Write(const short task_status_table[], size_t length, int num_tasks);

    }
    else{
        
        /*********TO DO: Others ********/

        neorv32_uart0_printf("ERROR STATUS ABORT ALL TASKS: REBOOT!!\n");

        __asm__ volatile ("li t0, %[input_i]; jr t0" :  : [input_i] "i" (BOOTLOADER_BASE_ADDRESS)); // jump to beginning of boot ROM (update task0)

        __builtin_unreachable();

    }
    
  }

}

/****EXECUTE TASK0****/
/****At Address [0x10000] *****/

//void task0() __attribute__((section(".task0")));

void main(void){
  
  neorv32_uart0_printf("*********************START OF I-O-Guard**********************\n");

  __asm__ __volatile__ (
      "auipc %0, 0\n"    // Load the upper 20 bits of PC into %0
      //"jalr %0, %0, 4"   // Add the lower 12 bits of PC to %0
      : "=r" (NEORV32_CFS->REG[INT_PC_FORWARD_REG]) // store the interrupt jump PC from zephyr to neorv32 task0
  );

	task0();

}