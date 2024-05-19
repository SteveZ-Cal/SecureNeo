/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/drivers/uart.h> // (stevez) added for uart0
#include <zephyr/kernel.h> // (stevez) added for kernelx

// uint32_t get_exe_word(int src, uint32_t addr) {

//   union {
//     uint32_t uint32;
//     uint8_t  uint8[sizeof(uint32_t)];
//   } data;

//   uint32_t i;
//   for (i=0; i<4; i++) {
//     if (src == EXE_STREAM_UART) {
//       data.uint8[i] = (uint8_t)PRINT_GETC();
//     }
//     else {
//       data.uint8[i] = spi_flash_read_byte(addr + i); // little-endian byte order
//     }
//   }

//   return data.uint32;
// }

// void get_exe(int src) {

//   getting_exe = 1; // to inform trap handler we were trying to get an executable

//   // flash image base address
//   uint32_t addr = (uint32_t)SPI_BOOT_BASE_ADDR;

//   // get image from UART?
//   if (src == EXE_STREAM_UART) {
//     printf("Awaiting task0_status_exe.bin... ");
//   }
//   // check if valid image
//   uint32_t signature = get_exe_word(src, addr + EXE_OFFSET_SIGNATURE);
//   if (signature != EXE_SIGNATURE) { // signature
//     system_error(ERROR_SIGNATURE);
//   }

//   // image size and checksum
//   uint32_t size  = get_exe_word(src, addr + EXE_OFFSET_SIZE); // size in bytes
//   uint32_t check = get_exe_word(src, addr + EXE_OFFSET_CHECKSUM); // complement sum checksum

//   // transfer program data
//   uint32_t *pnt = (uint32_t*)NEORV32_SYSINFO->ISPACE_BASE;
//   uint32_t checksum = 0;
//   uint32_t d = 0, i = 0;
//   addr = addr + EXE_OFFSET_DATA;
//   while (i < (size/4)) { // in words
//     d = get_exe_word(src, addr);
//     checksum += d;
//     pnt[i++] = d;
//     addr += 4;
//   }

//   // error during transfer?
//   if ((checksum + check) != 0) {
//     system_error(ERROR_CHECKSUM);
//   }
//   else {
//     PRINT_TEXT("OK");
//     exe_available = size; // store exe size
//   }

//   getting_exe = 0; // to inform trap handler we are done getting an executable
// }

static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

int main(void)
{
	unsigned char recv_char; 

	printf("Hello World!! %s\n", CONFIG_BOARD);
	//printf("Wait for input ...");
	//scanf("%d", num);
	//printf("Input is %d", num);
	// neorv32_uart0_printf("Hello World!");

	if (!device_is_ready(uart_dev)){
		printf("UART0 NOT READY!!\n");
		return 0;
	}

	printf("UART0 READY!!\n");

	while(1){
		printf("\nCMD:> ");
    	uart_poll_in(uart_dev, &recv_char);
    	printf("%c", recv_char);
    	printf("\n");
	}

	printf("SHOULD NEVER REACH!!\n");
	
	// start();

	return 0;
}
