// #################################################################################################
// # << NEORV32 - "Hello World" Demo Program >>                                                    #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file hello_world/main.c
 * @author Stephan Nolting
 * @brief Classic 'hello world' demo program.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/

uint32_t verbose = 1;

void task1(){   

    // neorv32_uart0_printf("NEORV32_CFS->REG[63] (PC): [%x]\n", NEORV32_CFS->REG[63]);
    // print project logo via UART
    unsigned int pc;

    __asm__ __volatile__ (
        "auipc %0, 0\n"    // Load the upper 20 bits of PC into %0
        //"jalr %0, %0, 4"   // Add the lower 12 bits of PC to %0
        : "=r" (pc)
    );

    neorv32_uart0_printf("Enter Hello world (task1) INT:) ***** pc at [%x]\n", pc);

    neorv32_rte_print_logo();

    __asm__ __volatile__ (
        "auipc %0, 0\n"    // Load the upper 20 bits of PC into %0
        //"jalr %0, %0, 4"   // Add the lower 12 bits of PC to %0
        : "=r" (pc)
    );

    neorv32_uart0_printf("Leave Hello world (task1) INT:) ----- pc at [%x]\n", pc);

    NEORV32_CFS->REG[3] = 1;

    // // say hello
    //neorv32_uart0_puts("Hello world! :)\n");

    unsigned int ra_value = NEORV32_CFS->REG[5];
    void (*task_pnt)(void);
    task_pnt= (void*)ra_value;
    (*task_pnt)();

    //return;
}   

//void task1() __attribute__((section(".task1")));

/**********************************************************************//**
 * Main function; prints some fancy stuff via UART.
 *
 * @note This program requires the UART interface to be synthesized.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

    neorv32_uart0_printf("NEORV32_CFS->REG[0]: [%x]\n", NEORV32_CFS->REG[0]);
    neorv32_uart0_printf("NEORV32_CFS->REG[1]: [%x]\n", NEORV32_CFS->REG[1]);

    unsigned int sp;
    __asm__ volatile ("mv %0, sp" : "=r" (sp));
    neorv32_uart0_printf("Start of hello_world [sp]: [%x]\n", sp);
    unsigned int pc;

    // __asm__ __volatile__ (
    //     "auipc %0, 0\n"    // Load the upper 20 bits of PC into %0
    //     "jalr %0, %0, 4"   // Add the lower 12 bits of PC to %0
    //     : "=r" (pc)
    // );

    // if(verbose)
    //     neorv32_uart0_printf("Before Jump --> PC [%x]\n", pc);

    unsigned int ra_val;
    __asm__ volatile ("mv %0, ra" : "=r" (ra_val));
    neorv32_uart0_printf("hello_world (ready to jump back) [ra]: [%x]\n", ra_val);

    unsigned int ra_value = NEORV32_CFS->REG[5];
    // __asm__ volatile ("mv %0, ra" : "=r" (ra_value));

    neorv32_uart0_printf("[TASK1] Ready to Start ............\n");

    neorv32_uart0_printf("hello_world [ra]: [%x]\n", ra_value);
    
    if(verbose)
        neorv32_uart0_printf("CSR_MEPC [%x]\n", neorv32_cpu_csr_read(CSR_MEPC));

    if(verbose)
        neorv32_uart0_printf("NEORV32_CFS->REG[0]: [%x]\n", NEORV32_CFS->REG[0]);

    if(verbose)
        neorv32_uart0_printf("NEORV32_CFS->REG[63] (PC): [%x]\n", NEORV32_CFS->REG[63]);

    task1();

    neorv32_uart0_printf("[TASK1] FINSIH ....................\n");

    void (*task_pnt)(void);
    task_pnt= (void*)ra_value;

    //neorv32_uart0_printf("[SHT1]********************\n");

    NEORV32_CFS->REG[3] = 1;

    __asm__ volatile ("mv %0, sp" : "=r" (sp));
    neorv32_uart0_printf("End of hello_world [sp]: [%x]\n", sp);

    __asm__ __volatile__ (
        "auipc %0, 0\n"    // Load the upper 20 bits of PC into %0
        //"jalr %0, %0, 4"   // Add the lower 12 bits of PC to %0
        : "=r" (pc)
    );

    neorv32_uart0_printf("Before task1() pc at [%x]\n", pc);

    //(*task_pnt)();

    //neorv32_uart0_printf("[FUCK1]###################\n");

}