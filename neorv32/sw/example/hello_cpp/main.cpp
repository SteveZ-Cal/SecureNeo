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
 * @file hello_cpp/main.cpp
 * @author Gideon Zweijtzer
 * @brief Simple 'hello world' type of demo with static C++ constructors
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/

/**********************************************************************//**
 * DemoClass: Just a simple C++ class that holds one constant and can
 *            be asked to print it.
 *
 * @note This class will only successfully reveal its ID if the
 *       constructors are called prior to the main function.
 **************************************************************************/
class DemoClass
{
	const int identity;
public:
	DemoClass(int id) : identity(id) { }

	void print_id(void)
	{
		// In order to demonstrate just how constructors are called pre-main,
		// it is not necessary to use the C++ type streams to print something.
		neorv32_uart0_printf("I am DemoClass with instance ID: %d\n", identity);
	}
};

static DemoClass demo1(1);
static DemoClass demo2(2);

// void nop() {
//     asm("nop");
// }

void jump_back(void)  __attribute__ ((naked));

 void jump_back(void){

  uint32_t firq_PC = NEORV32_CFS->REG[61];
  uint32_t PC = NEORV32_CFS->REG[63];
  firq_PC = firq_PC%4 + firq_PC;
  neorv32_uart0_printf("Current PC: [%x]\n", PC);
  neorv32_uart0_printf("jump back to FIRQ -> MAIN [%x]\n", firq_PC);
  asm volatile ("jalr ra, %0" : : "r" (firq_PC));
  //asm volatile ("li t0, %[input_i]; jr t0" :  : [input_i] "i" (firq_PC)); 

}

/**********************************************************************//**
 * Main function; prints some fancy stuff via UART.
 *
 * @note This program requires the UART interface to be synthesized.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

      // Declare a pointer to an integer
    uint32_t *ptr;

    // Assign the address 0x80002000 to the pointer
    ptr = (uint32_t *)0x80002000;

    *ptr = 1111;
  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // for(uint32_t i = 0; i < 4; i++){
  //   nop();
  // }
  neorv32_gpio_port_set(2);
  neorv32_cpu_delay_ms(1000); // wait 1000ms using busy wait

  //neorv32_uart0_printf("[[$$$$$]]\n");
  //neorv32_uart0_printf("[[START OF HELLO_CPP!!]]\n");
  // print project logo via UART
  // neorv32_rte_print_logo();

  // say hello
  neorv32_uart0_puts("HELLO_CPP: $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ :)\n");
  neorv32_uart0_printf("Value from 0x20000 at 0x80002000 : [%d]\n", *ptr);

  // print the IDs of the two statically declared instances of DemoClass
  //demo1.print_id();
  //demo2.print_id();
  neorv32_gpio_port_set(0);
  neorv32_cpu_delay_ms(1000); // wait 1000ms using busy wait

  jump_back();

  neorv32_uart0_printf("<<Before Exit hello_cpp>>\n");
  //asm volatile ("li t0, %[input_i]; jr t0" :  : [input_i] "i" (0x0)); // jump to beginning of IMEM
  //asm volatile ("jalr ra, %0" : : "r" (0x0)); 

  /***JUMP Back to the Main Program***/
  // uint32_t firq_PC = NEORV32_CFS->REG[61];
  // uint32_t PC = NEORV32_CFS->REG[63];
  // firq_PC = firq_PC%4 + firq_PC;
  // neorv32_uart0_printf("Current PC: [%x]\n", PC);
  // neorv32_uart0_printf("jump back to FIRQ -> MAIN [%x]\n", firq_PC);
  // asm volatile ("jalr ra, %0" : : "r" (firq_PC));

    // asm volatile (
    //     "jalr x0, ra, 0"
    // );

  neorv32_uart0_printf("{{Exit hello_cpp}}\n");

  return 0;
}
