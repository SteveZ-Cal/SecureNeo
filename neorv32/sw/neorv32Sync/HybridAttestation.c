/**********************************************************************//**
 * @file HybridAttestation.c
 * @author Steve Zang
 * @brief Hybrid Attestation on NEORV32
 **************************************************************************/

#include <neorv32.h>

#include <stdio.h>

#include <tinycrypt/sha256.h> // include the sha256 HASH function

#include <stdbool.h>

#include <errno.h>

#include "HybridAttestation.h"

/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/

void aux_print_hex_byte(uint8_t byte);
/**********************************************************************//**
 * Print HEX byte.
 *
 * @param[in] byte Byte to be printed as 2-cahr hex value.
 **************************************************************************/
void aux_print_hex_byte(uint8_t byte) {

  static const char symbols[] = "0123456789abcdef";

  neorv32_uart0_putc(symbols[(byte >> 4) & 0x0f]);
  neorv32_uart0_putc(symbols[(byte >> 0) & 0x0f]);
}


/**********************************************************************//**
 * PRINT CONTENT OF THE GIVEN ADDRESS SPACE
 **************************************************************************/

void neorv32_address_print(uint32_t lines, uint32_t mem_address){
  
  uint32_t mem_data_w1 = 0;
  neorv32_uart0_puts("NEORV32_ADDRESS_PRINT START!! \n\n");

  for(uint32_t i = 0; i<lines; i++){
    neorv32_uart0_puts("MEM_ADDRESS: ");
    //char str[] = mem_address; // Adjust the array size based on the expected length of the number
    // Use sprintf to convert the uint32_t to a string
    //sprintf(str, "%u", mem_address);
    //neorv32_uart0_printf(%u, );
    
    //neorv32_uart0_puts(" --> ");
    neorv32_uart0_printf("\n[0x%x] => ", mem_address);
    
    mem_data_w1 = (uint32_t)neorv32_cpu_load_unsigned_word(mem_address);
    neorv32_uart0_puts("val: ");
    neorv32_uart0_printf("[0x%u] => ", mem_data_w1);
    neorv32_uart0_puts(" Hex: ");
    aux_print_hex_byte((uint8_t)(mem_data_w1 >> 24));
    aux_print_hex_byte((uint8_t)(mem_data_w1 >> 16));
    aux_print_hex_byte((uint8_t)(mem_data_w1 >>  8));
    aux_print_hex_byte((uint8_t)(mem_data_w1 >>  0));
    neorv32_uart0_puts("\n");

      for(int i=0; i<4; i++){
        neorv32_uart0_printf("\n[0x%x] => ", mem_address);
        mem_data_w1 = (uint32_t)neorv32_cpu_load_unsigned_byte(mem_address); 
        neorv32_uart0_puts("val: ");
        neorv32_uart0_printf("[0x%u] => ", mem_data_w1);
        neorv32_uart0_puts(" Hex: ");
        mem_address += 1;
        aux_print_hex_byte(mem_data_w1);
        neorv32_uart0_puts("\n\n");
      }
    
    
   }

   neorv32_uart0_puts("NEORV32_ADDRESS_PRINT END :-) \n\n");

}


/****************************************************************************************************************************//**
BEGIN of SOFTWARE ATTESTATION HASH (software) HELPER FUNCTIONS
********************************************************************************************************************************/


/**********************************************************************//**
 * HASH INPUT TRANSFORMATION FOR MEMORY FROM 32unit to 8unint[]
 **************************************************************************/

void neorv32_memory_32s_to_8s(uint8_t *data_uint8, uint32_t lines, uint32_t mem_address, uint32_t print){

  size_t length = sizeof(data_uint8)*lines;

  //uint8_t data_uint8[length];

  uint32_t current_index = 0; // index for data_uint8[]

  uint32_t mem_data = 0; // initialize for get the value of mem_data

  for (uint32_t current_line = 0; current_line < lines; current_line++){

    mem_data = (uint32_t)neorv32_cpu_load_unsigned_word(mem_address);
    
    // byte loop is for each line of 32 bit line, so 4 iterations
    for (uint32_t byte = 0; byte < 4; byte++){

      data_uint8[current_index + byte] = (uint8_t)(mem_data >> (8*byte));

    }
  
    current_index += 4;

  }

  // this is for readiblity purpose print out values of data_uint8
  if(print == 1){

    neorv32_uart0_printf("\nNEORV32_MEMORY_32s_to_8s START!! \n");
    
    for (uint32_t i = 0; i<length; i++){

      neorv32_uart0_printf("\nUINT8_T[0x%u] => %x", i, data_uint8[i]);

    }

    neorv32_uart0_printf("\n\nNEORV32_MEMORY_32s_to_8s END :-) \n");

  }

  //return data_uint8;

}



/**********************************************************************//**
 * NEORV32 HASH FUNCTION SPECIFICALLY
 **************************************************************************/

void neorv32_hash256 (uint8_t* hash, uint32_t length, uint8_t *data){

  struct tc_sha256_state_struct sha256_state; 

  // neorv32_uart0_puts("\nNEORV32_HASH256 START!!\n\n"); // (NEEDED)

  uint32_t TC_CRYPTO_SUCCESS = 1;

  //TCSha256State_t sha256_state = (TCSha256State_t)malloc(sizeof(struct tc_sha256_state_struct));

  //TCSha256State_t sha256_state;

  uint32_t init_result = tc_sha256_init(&sha256_state);

  if (init_result != TC_CRYPTO_SUCCESS){
    // neorv32_uart0_printf("SHA256 Init Unsuccessful"); // (NEEDED)
    exit(errno);
  }
  else{
    // neorv32_uart0_printf("SHA256 Init Successful"); // (NEEDED)
  }

  //uint8_t data[] = "THIS IS SHA-256!";

  //uint32_t length = lines*4; // length of uint8_t
  size_t data_len = sizeof(*data)*length;

  //neorv32_uart0_printf("\n\nhash_data_input length: [%u]", data_len); for bebugging data_length only

  //eorv32_uart0_printf("\nSHA256 DATA INPUT : [%x]", data);

  uint32_t update_result = tc_sha256_update(&sha256_state, data, data_len);

  if (update_result != TC_CRYPTO_SUCCESS){
    // neorv32_uart0_printf("\n\nSHA256 Update Unsuccessful\n"); // (NEEDED)
    exit(errno);
  }
  else{
    // neorv32_uart0_printf("\n\nSHA256 Update Successful\n"); // (NEEDED)
  }

  //uint32_t hash_length = 32;

  //uint8_t hash[hash_length];

  int final_result = tc_sha256_final(hash, &sha256_state);
  if (final_result != TC_CRYPTO_SUCCESS){
    // neorv32_uart0_printf("\nSHA256 Final Unsuccessful\n"); // (NEEDED)
    exit(errno);
  }
  else{
    // neorv32_uart0_printf("\nSHA256 Final Successful\n"); // (NEEDED)
    }

      //********** HASH DEBUGGING ******//

      //neorv32_uart0_printf("[SHA256 Final Result: %u] ==> %x\n", i, hash[i]);
      //neorv32_uart0_printf("%02x",hash[i]);
      // hash_value[i] = hash[i];

      //********** HASH DEBUGGING ******//

    //neorv32_uart0_printf("[SIZE: ] ==> %u", sizeof(*hash));
    //neorv32_uart0_printf(final_result);

  // neorv32_uart0_puts("\nNEORV32_HASH256 END!!\n"); // (NEEDED)

  //return hash;

}


/**********************************************************************//**
 * NEORV32 HASH PRINT 32 BIT PER LINE
 **************************************************************************/

void neorv32_hash256_print(uint8_t *hash){

  uint32_t final_hashs[8];
  uint32_t final_hash = 0;
  uint32_t hash_length = 32;
  uint32_t index = 0;

  neorv32_uart0_puts("\n\nNEORV32_HASH256_PRINT START!!\n");
  
  for(uint32_t i = 0; i < hash_length; i++){

    final_hash = (uint32_t)(final_hash << 8) + (uint32_t)hash[i];
    //neorv32_uart0_printf("\n%x",final_hash);
    
    if((i+1)%4 == 0){
      //neorv32_uart0_printf("TRUE");
      final_hashs[index] = final_hash;
      neorv32_uart0_printf("\nHash Line %u: [%x]", index, final_hashs[index]); // print out the hash line memory calcualtion 
      final_hash = 0;
      index+=1;
    }

    //********** HASH DEBUGGING ******//

    //neorv32_uart0_printf("[SHA256 Final Result: %u] ==> %x\n", i, hash[i]);
    //neorv32_uart0_printf("%02x",hash[i]);
    // hash_value[i] = hash[i];

    //********** HASH DEBUGGING ******//

  }

  neorv32_uart0_puts("\n\nNEORV32_HASH256 END :-)\n");

  //return final_hashs;

}




/**********************************************************************//**
 * NEORV32 PREVIOUS_HASH_OUTPUT + CURRENT_MEMORY_CONTENT --> (CONCATENATION) AS ONE FINAL_HASH_INPUT For SHA256 (software) uint8_t type
 **************************************************************************/



void neorv32_hashinput_concatenation(uint8_t* final_hash_input, uint8_t* previous_hash_output, uint32_t previous_hash_output_length, uint8_t* hash_data_input, uint32_t hash_data_input_length){

  // final_hash_input will be the hash_input for the next hash_function
  // previous_hash_output will be the preivous output from that hash_function
  // hash_data_input will be the current memory_content

  uint32_t final_hash_input_length = previous_hash_output_length + hash_data_input_length;

  for (uint32_t i = 0; i < final_hash_input_length; i++){
    if(i < previous_hash_output_length){
      final_hash_input[i] = previous_hash_output[i];
    }
    else{
      final_hash_input[i] = hash_data_input[i-previous_hash_output_length];
    }
  }

}


/****************************************************************************************************************************//**
BEGIN of SOFTWARE ATTESTATION HASH (hardware) HELPER FUNCTIONS
********************************************************************************************************************************/


/**********************************************************************//**
 * NEORV32 Find Padding Zero for SHA256 (hardware) Message Preprocessing
 **************************************************************************/

uint64_t find_padding_zeros(size_t original_message_length) {
    size_t k = 0;
    while ((original_message_length + 1 + k) % 512 != 448) {
        k++;
    }
    return k;
}

/**********************************************************************//**
 * NEORV32 CFS Registers Reset
 **************************************************************************/

void neorv32_cfs_register_rst(){

  uint32_t cfs_reg_num = 64;

  for(uint32_t i = 0; i < cfs_reg_num; i++){
    NEORV32_CFS -> REG[i] = 0;
  }

}


/**********************************************************************//**
 * NEORV32 CFS Registers Assign
 **************************************************************************/

void neorv32_cfs_register_assign(uint32_t *padded_message, uint32_t num_register_required){

  uint32_t start_register_index = 16; 
  uint32_t index;

  for(uint32_t i = 0; i < num_register_required; i++){
    index = start_register_index + i;
    NEORV32_CFS -> REG[index] = padded_message[i];
    //neorv32_uart0_printf("CFS_REG[%d]: %x\n", index, NEORV32_CFS -> REG[index]);
  }

}

/**********************************************************************//**
 * NEORV32 PREVIOUS_HASH_OUTPUT + CURRENT_MEMORY_CONTENT --> (CONCATENATION) AS ONE FINAL_HASH_INPUT FOR SHA256 (hardware) unit32_t type
 **************************************************************************/

void neorv32_hashinput_concatenation_uint32(uint32_t* final_hash_input, uint32_t* previous_hash_output, uint32_t previous_hash_output_length, uint32_t* hash_data_input, uint32_t hash_data_input_length){
    
  // final_hash_input will be the hash_input for the next hash_function
  // previous_hash_output will be the preivous output from that hash_function
  // hash_data_input will be the current memory_content

  uint32_t final_hash_input_length = previous_hash_output_length + hash_data_input_length;

  for (uint32_t i = 0; i < final_hash_input_length; i++){
    if(i < previous_hash_output_length){
      final_hash_input[i] = previous_hash_output[i];
    }
    else{
      final_hash_input[i] = hash_data_input[i-previous_hash_output_length];
    }
  }

}

/**********************************************************************//**
 * NEORV32 GET MEMORY CONTENT PER MESSAGE
 **************************************************************************/


void neorv32_get_mem_uint32(uint32_t *get_mem_uint32, uint32_t mem_iter_lines, uint32_t mem_address){

  uint32_t mem_data = 0; // initialize for get the value of mem_data

  for (uint32_t current_line = 0; current_line < mem_iter_lines; current_line++){

    mem_data = (uint32_t)neorv32_cpu_load_unsigned_word(mem_address);
    
    get_mem_uint32[current_line] = mem_data;

    mem_address += 4;

  }

}


/**********************************************************************//**
 * Convert An Integer to Its binary representation using two uint32_t
 **************************************************************************/

void intToBinary(uint64_t num, uint32_t *binaryHigh, uint32_t *binaryLow) {
    *binaryLow = (uint32_t)(num & 0xFFFFFFFF);
    *binaryHigh = (uint32_t)((num >> 32) & 0xFFFFFFFF);
}


/**********************************************************************//**
 * NEORV32 Message Padding
 **************************************************************************/

// l --> length of the original message/hash_content_uint32 (not the padded one) in BITS, k --> Smallest number of zeroes appended to a message during the padding step in BITS.
void neorv32_message_padding(uint32_t *final_padded_message, uint32_t *hash_content_uint32, uint32_t final_padded_message_length, uint32_t l){

    uint32_t binaryHigh, binaryLow;
    uint64_t l_uint64 = (uint64_t)l;
    // neorv32_uart0_printf("[Message Padding Function --> ]\n"); // (NEEDED)
    intToBinary(l_uint64, &binaryHigh, &binaryLow);
    //neorv32_uart0_printf("[Padding 64-bit...]\n");
    // fill the 64-bit block padding [1]
    final_padded_message[final_padded_message_length-2] = binaryHigh;
    final_padded_message[final_padded_message_length-1] = binaryLow;

    uint32_t message_length = l/32;

    // neorv32_uart0_printf("[Done Padding 64-bit]\n"); // (NEEDED)
    // fill the original message [2]
    for (uint32_t i = 0; i < message_length; i++){
      //neorv32_uart0_printf("Iter[%d]\n", i);
      final_padded_message[i] = hash_content_uint32[i];
    } 
    // neorv32_uart0_printf("[Done Orignal Message]\n"); // (NEEDED)
    // fill that one bit along with other zero bits 
    final_padded_message[message_length] = 1 << 31;
    //neorv32_uart0_printf("[Padding zero bits...]\n");
    // fill the zero bits [3]
    for (uint32_t i = message_length+1; i < final_padded_message_length-2; i++){
      final_padded_message[i] = 0;
    }
    // neorv32_uart0_printf("[Done Padding zero bits]\n"); // (NEEDED)
}

/**********************************************************************//**
 * NEORV32 Print Content from CFS Registers
 **************************************************************************/
void neorv32_cfs_register_print(uint32_t start_index, uint32_t end_index){

    neorv32_uart0_printf("[CFS_REG]\n");

    for(uint32_t i = start_index; i <= end_index; i++){
      uint32_t val = NEORV32_CFS -> REG[i];
      neorv32_uart0_printf("REG[%d]: %x\n", i, val);
    }

}


/**********************************************************************//**
 * NEORV32 SHA256 DATA_READY
 **************************************************************************/
void data_ready(void){

  NEORV32_CFS -> REG[DATA_READY_REG] = 1;

}

/**********************************************************************//**
 * NEORV32 Get Hash Output From CFS REG
 **************************************************************************/
void neorv32_cfs_register_hash(uint32_t* hash_data_output_uint32, uint32_t data_out_reg_start, uint32_t data_out_reg_end){

  // neorv32_uart0_printf("[CFS_REGISTER_HASH]\n"); // (NEEDED)

  for (uint32_t i = data_out_reg_start; i <= data_out_reg_end; i++){

    hash_data_output_uint32[i-data_out_reg_start] = NEORV32_CFS -> REG[i];
    // neorv32_uart0_printf("HASH_DATA_OUTPUT_UINT32[%d]: %x\n", i-data_out_reg_start, hash_data_output_uint32[i-data_out_reg_start]); // (NEEDED)

  }

  // neorv32_uart0_printf("[END: CFS_REGISTER_HASH]\n"); // (NEEDED)
  
}

/**********************************************************************//**
 * NEORV32 Print IV (Initialization Vector)
 **************************************************************************/
void neorv32_IV_uint32_print(uint32_t *IV_uint32){
 
  neorv32_uart0_printf("[[IV]]\n");
 
  for(uint32_t i=0; i<8; i++){
    neorv32_uart0_printf("[%x]\n", IV_uint32[i]);
  }

  neorv32_uart0_printf("\n");

}


/**********************************************************************//**
 * NEORV32 Hybrid Attestation --------------------------------------------------- For Testing Purpose
 **************************************************************************/

void Hybrid_Attestation(uint32_t this_num_blocks_cal, uint32_t* IV_uint32, uint32_t clock_prescaler, uint32_t clock_prescaler_threshold, uint32_t mem_address){

  // uint32_t curmem_address = 0x80000000u; // current memory address (subjected to be modified by the user)
  // uint32_t mem_address = 0x80000000u; // initial memory address (subjected to be modified by the user)
  uint32_t curmem_address = mem_address; // current memory address (subjected to be modified by the user)
  uint32_t seed_val  = 4294967295; // subject to change based upon seed value
  uint32_t size = 4;

  /**********************************************************************//**
  BEGIN of ATTESTACTION PROCESS (HASH MEMORY CONTENT) ---> Hardware Hash Module
  **************************************************************************/

  // neorv32_uart0_printf("\nNeorv32 Hybrid Attestation: [sha256 hardware] \n\n"); // (NEEDED)
  
  // Calculation for n_blocks, msg_block_in

  uint32_t line_size = 32; // 32 bits or 4 byte per line
  uint32_t hash_length_uint32 = 8; // hash length in 32-bit

  uint32_t num_blocks_cal = this_num_blocks_cal; // number of blocks memory chunk (THIS IS THE INPUT FOR LATER HYBRID_ATTESTATION_TESTING BRANCH)

  uint32_t block_size = 512; // 512 bits per block in sha256 hardware module
  uint32_t mem_iter_lines = 2; // number of memory lines per each hash iterartion calculation
  uint32_t mem_total_lines = mem_iter_lines * num_blocks_cal; // total number of memory lines needs
  uint32_t mem_loop = mem_total_lines / mem_iter_lines; // based on above 2 variables calcualte the # of time to call sha256
  
  uint32_t mem_total_bit = mem_total_lines * line_size; 

  // l+1+k = 448 (mod) 512
  uint32_t l = mem_iter_lines * line_size + hash_length_uint32 * line_size; // length of Message, M, in bits (for one iteration only) view that as one block.
  uint32_t k = find_padding_zeros(l); // Number of zeroes appended to a message during the padding step. 
  uint32_t preprocessed_block_size = l + 1 + k + 64;
  
  // neorv32_uart0_printf("Neorv32 Varaible [l: %d] [k: %d] [preprocessed_block_size: %d]\n\n", l, k, preprocessed_block_size); // (NEEDED)

  uint32_t n_block = preprocessed_block_size / block_size; // n_block for that message M, normally 1

  uint32_t data_out_reg_start = 48; // start of hash output register
  uint32_t data_out_reg_end = 55; // end of hash output register

  // for attestation timing
  uint32_t start, end;
  uint32_t cpu_time_used;

  /*****INITIALIZE NEORV32 MEMORY SPACE (if needed)***/

  // neorv32_uart0_printf("Initialize Neorv32 Memory Space\n"); // (NEEDED)

  // only initialize the memory space if the memory address is 0x80000000u (for testing purpose), else just skip this step

  if (mem_address == 0x80000000u){
  
    for (uint32_t i = 0; i<mem_total_lines; i++){

        neorv32_cpu_store_unsigned_word(curmem_address, seed_val); // filling in the addressed space for calculating the hash
        // neorv32_uart0_printf("DONE:[%x]\n", neorv32_cpu_load_unsigned_word(curmem_address)); // (MAYBE NEEDED)
        curmem_address += size;
        
    }

  }

    /**END OF INITIALIZATION*/

  neorv32_uart0_printf("\nStart Hybrid Attestation!!\n"); // (NEEDED)

  // set up the attestation timer
  neorv32_gptmr_enable(); // enable timer
  neorv32_gptmr_restart(); // restart timer

  neorv32_gptmr_setup(clock_prescaler, 0, clock_prescaler_threshold); // main processor clock (clock_prescaler) division by 4096, timer operates in single shot mode, threshold = 1000

  start = neorv32_gptmr_curtime();
  neorv32_uart0_printf("\n----- Start Timing ----- [%d]\n", start); //(NEEDED)

  uint32_t hash_data_output_uint32[hash_length_uint32]; // initialize hash_data_output

  for (uint32_t loop = 0; loop < mem_loop; loop++){

    neorv32_cfs_register_rst(); // reset all cfs registers

    // neorv32_uart0_printf("\nMESSAGE[%d]\n", loop); //(NEEDED)

    uint32_t data_length_uint32 = mem_iter_lines; // number of memory lines per message block

    uint32_t hash_mem_uint32[data_length_uint32]; //raw memory content of this message only

    // neorv32_uart0_printf("Getting Memory Content...\n"); //(NEEDED)

    neorv32_get_mem_uint32(hash_mem_uint32, mem_iter_lines, mem_address); // get the (mem_iter_lines) of memory 

    uint32_t hash_content_uint32_length = hash_length_uint32 + data_length_uint32;

    uint32_t hash_content_uint32[hash_content_uint32_length]; //raw memory content + preivous hash ouput of this message

    // neorv32_uart0_printf("Goncatenate Memory Content & Previous Hash...\n"); // (NEEDED)

    // concatenate the hash_input
    if(loop == 0)
      neorv32_hashinput_concatenation_uint32(hash_content_uint32, IV_uint32, hash_length_uint32, hash_mem_uint32, data_length_uint32);
    else
      neorv32_hashinput_concatenation_uint32(hash_content_uint32, hash_data_output_uint32, hash_length_uint32, hash_mem_uint32, data_length_uint32);
    
    // neorv32_uart0_printf("Finalize Padded Message...\n"); // (NEEDED)

    uint32_t final_padded_message_length = block_size/32;
    uint32_t final_padded_message[final_padded_message_length];

    neorv32_message_padding(final_padded_message, hash_content_uint32, final_padded_message_length, l);

    // neorv32_uart0_printf("Assign to CFS Registers...\n"); // (NEEDED)
    neorv32_cfs_register_assign(final_padded_message, final_padded_message_length);

    // uint32_t middle = neorv32_gptmr_curtime();
    // neorv32_uart0_printf("\n----- Middle Timing ----- [%d]\n", middle); // (NEEDED)

    //for debugging purpose
   // neorv32_cfs_register_print(16, 31);

    // import n_block to indicate finish
    NEORV32_CFS -> REG[NUM_BLOCKS_REG] = n_block;

    data_ready();
    
    NEORV32_CFS -> REG[DATA_READY_REG] = 0;

    // neorv32_uart0_printf("Data Ready...\n"); // (NEEDED)

    // wait for the sha256 hardware finished signal
    while(true){

      //neorv32_uart0_printf("Checking Finished Signal...\n");

      if((uint32_t)NEORV32_CFS -> REG[FINISH_FLAG_REG] == 1){
        // neorv32_uart0_printf("[Finished Hash Calcualtion]\n"); // (NEEDED)
        break;
      }

      // neorv32_uart0_printf("Loop End...\n");
    
    }

    NEORV32_CFS -> REG[FINISH_FLAG_REG] == 0; // reset fininshed signal
    // neorv32_uart0_printf("Assign Hash Output...\n"); // (NEEDED)

    // assign hash_ouput
    neorv32_cfs_register_hash(hash_data_output_uint32, data_out_reg_start, data_out_reg_end);

    // for(int i = 0; i < hash_length_uint32; i++){
    //   neorv32_uart0_printf("HASH %d: [%x]\n", i, hash_data_output_uint32[i]);
    // }

    // neorv32_uart0_printf("\nEND OF Message[%d]\n", loop); // (NEEDED)

    mem_address += size * mem_iter_lines; // update initial mem_address for each iteration

  } // end of calculating first block_memory

  end = neorv32_gptmr_curtime();
  neorv32_uart0_printf("\n----- End Timing ----- [%d]\n", end); // (NEEDED)
  cpu_time_used = end - start; 
  neorv32_uart0_printf("Hybrid Attestation Time Used: [%d]\n", cpu_time_used);

  neorv32_uart0_printf("End of Hybrid Attestation"); // (NEEDED)

}



/**********************************************************************//**
 * NEORV32 Software Attestation --------------------------------------------------- For Testing Purpose
 **************************************************************************/

void Software_Attestation(uint32_t this_num_blocks_cal, uint8_t* IV_uint8, uint32_t clock_prescaler, uint32_t clock_prescaler_threshold){

  uint32_t curmem_address = 0x80000000u; // current memory address
  uint32_t mem_address = 0x80000000u; // initial memory address
  uint32_t seed_val  = 4294967295; // subject to change based upon seed value
  uint32_t size = 4;

  /**********************************************************************//**
  BEGIN of ATTESTACTION PROCESS (HASH MEMORY CONTENT) ---> Software Hash Module
  **************************************************************************/

  // neorv32_uart0_printf("\n\nNeorv32 Software Attestation: [sha256 software] \n\n"); // (NEEDED)

  uint32_t hash_length_uint8 = 32; // hash length in 32-bit = 32; // length of hash output 32 bytes  

  uint32_t num_blocks_cal = this_num_blocks_cal;
  uint32_t mem_iter_lines = 2; // number of memory lines per each hash iterartion calculation
  uint32_t mem_total_lines = mem_iter_lines * num_blocks_cal; // total number of memory lines needs
  uint32_t mem_loop = mem_total_lines / mem_iter_lines; // based on above 2 variables calcualte the # of time to call sha256

  /*****INITIALIZE NEORV32 MEMORY SPACE***/
  
  for (uint32_t i = 0; i<mem_total_lines; i++){
    
      neorv32_cpu_store_unsigned_word(curmem_address, seed_val); // filling in the addressed space for calculating the hash
      curmem_address += size;
      
    }

    /**END OF INITIALIZATION*/

  // for attestation timing
  uint32_t start, end;
  uint32_t cpu_time_used;

  // neorv32_uart0_printf("\nStart Software Attestation!!\n"); // (NEEDED)

  // set up the attestation timer
  neorv32_gptmr_enable(); // enable timer
  neorv32_gptmr_restart(); // restart timer

  neorv32_gptmr_setup(clock_prescaler, 0, clock_prescaler_threshold); // main processor clock (clock_prescaler) division by 4096, timer operates in single shot mode, threshold = 1000

  start = neorv32_gptmr_curtime();

  uint8_t hash_data_output[hash_length_uint8]; // initialize hash_data_output

  for (uint32_t loop = 0; loop < mem_loop; loop++){

    uint32_t data_length = mem_iter_lines*size;

    uint8_t hash_data_input[data_length]; // initialize hash_data_input

    neorv32_memory_32s_to_8s(hash_data_input, mem_iter_lines, mem_address, 0); // input of hash uint8_t

    //neorv32_uart0_printf("\nhash_data_input address: [%x]\n", hash_data_input); // for debugging purpose only

    //neorv32_uart0_printf("\nhash_data_[11]: [%x]\n", hash_data_input[11]); // for debugging purpose only

    uint32_t final_hash_input_length = hash_length_uint8 + data_length;

    uint8_t final_hash_input[final_hash_input_length];

    // concatenate the hash_input
    if(loop == 0)
      neorv32_hashinput_concatenation(final_hash_input, IV_uint8, hash_length_uint8, hash_data_input, data_length);
    else
      neorv32_hashinput_concatenation(final_hash_input, hash_data_output, hash_length_uint8, hash_data_input, data_length);
    // print the final_hash_input
    // neorv32_uart0_printf("\n\nFINAL_HASH_INPUT: ");
    // for(uint32_t i = 0; i < (final_hash_input_length); i++){
    //   neorv32_uart0_printf("\nByte %u: [%x]", i, final_hash_input[i]);
    // }

    neorv32_hash256(hash_data_output, final_hash_input_length, final_hash_input); // output of hash uint8_t[32]

    // for (int i = 0; i < 32; i++){
    //   neorv32_uart0_printf("\nByte %u: [%x]", i, hash_data_output[i]); // checked correct hash memory output (but not line, it's byte) debugging purpose
    // }

    //neorv32_hash256_print(hash_data_output); // output of hash uint32_t[8] same as above but different format

    mem_address += size * mem_iter_lines; // update initial mem_address for each iteration

  } // end of calculating first block_memory

  end = neorv32_gptmr_curtime();
  cpu_time_used = end - start; 
  neorv32_uart0_printf("Software Attestation Time Used: [%d]\n", cpu_time_used);

}




// /**********************************************************************//**
//  * Main function; prints some fancy stuff via UART.
//  *
//  * @note This program requires the UART interface to be synthesized.
//  *
//  * @return 0 if execution was successful
//  **************************************************************************/
// int main() {

//   // capture all exceptions and give debug info via UART
//   // this is not required, but keeps us safe
//   neorv32_rte_setup();

//   // setup UART at default baud rate, no interrupts
//   neorv32_uart0_setup(BAUD_RATE, 0);

//   // check available hardware extensions and compare with compiler flags
//   neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch

//   // print project logo via UART
//   //neorv32_rte_print_logo();

//   // say hello
//   //neorv32_uart0_puts("\nHello world! :)\n");

//   neorv32_uart0_puts("\nNEORV32 START !! :)\n\n");

//   // while(1){
//   //   neorv32_uart0_puts("loop\n");
//   // }

//   // load and store predefine

//   /********* Hybrid Attestation Testing**********/
  
//   uint32_t num_trials = 5; // 5 random IV tested
//   uint32_t max_num_blocks_cal = 10; // up from 1 to 10 blocks
//   // see Table 35. GPTMR prescaler configuration for more information 
//   uint32_t clock_prescaler = 3; // 7 represent 1024 --> subjected to change to fit for your data plot
//   uint32_t clock_prescaler_num = 64; // (above)
//   uint32_t clock_prescaler_threshold = 100000;  // (recommended) set it to as big as possible
//   uint32_t hash_length_uint32 = 8;
//   //uint32_t IV_uint32_list[num_trials][hash_length_uint32]; 



//   /**********************************************************************//**
//   * NEORV32 Software Attestation Testing --------------------------------------------------- [1]
//   **************************************************************************/

// //   uint8_t IV_uint8_list[5][32] = {
// //     {0x3a, 0x5e, 0x2a, 0x96, 0xe1, 0xa4, 0xa8, 0x03, 0x49, 0xd8, 0xc2, 0x1b, 0xc8, 0xf4, 0x1b, 0x70,
// //      0x71, 0xeb, 0x91, 0x28, 0xf3, 0x33, 0x76, 0x2a, 0xe1, 0x65, 0x9a, 0x54, 0xf9, 0x46, 0xd5, 0xf5},
// //     {0x82, 0xdb, 0x8f, 0x14, 0xef, 0x6f, 0x60, 0x5e, 0xee, 0xdd, 0x2f, 0x5d, 0xb5, 0x97, 0x6c, 0x1d,
// //      0x0b, 0x76, 0xc2, 0xbf, 0x40, 0xcf, 0x60, 0xdf, 0xc7, 0x2b, 0xb5, 0xa4, 0xd0, 0x32, 0xea, 0x5c},
// //     {0x6b, 0x6b, 0xe1, 0x3d, 0xfb, 0xbd, 0x2c, 0x52, 0x58, 0x77, 0xf4, 0x0e, 0x0d, 0x3d, 0x11, 0xc3,
// //      0xe8, 0xcb, 0xcd, 0x4c, 0x7b, 0xe0, 0x45, 0x12, 0xb2, 0x9c, 0x29, 0x2a, 0x35, 0xc4, 0xb0, 0x6e},
// //     {0x2e, 0x09, 0x48, 0x4e, 0xfc, 0xe0, 0xd9, 0x7a, 0x81, 0x10, 0x97, 0xa3, 0xc2, 0x4e, 0x93, 0xfe,
// //      0x4c, 0x6e, 0xbc, 0x8e, 0xe5, 0xf4, 0x1a, 0x76, 0x8b, 0x1b, 0x0d, 0x4f, 0x3c, 0x96, 0xe8, 0x30},
// //     {0x91, 0x80, 0x2d, 0x09, 0x59, 0x2c, 0xe7, 0x05, 0xda, 0xb0, 0xbe, 0x1f, 0x36, 0xdb, 0x8e, 0x1e,
// //      0x84, 0x9a, 0x79, 0x2e, 0xcc, 0xe7, 0x84, 0x17, 0xa9, 0xe0, 0x72, 0xf2, 0xe4, 0xcb, 0x32, 0xd2}
// // };

// //   neorv32_uart0_printf("-------------Start of Software Attestation-------------\n\n");

// //   neorv32_uart0_printf("RESULTING CLOCK PRE-SCALAR: [%d]\n\n", clock_prescaler_num);

// //   for (uint32_t trial_num = 0; trial_num < num_trials; trial_num++){

// //     neorv32_uart0_printf("[[[[[[TRIAL NUM %d]]]]]]\n\n", trial_num);

// //     neorv32_IV_uint32_print(IV_uint8_list[trial_num]);

// //     for (uint32_t this_num_blocks_cal = 1; this_num_blocks_cal <= max_num_blocks_cal; this_num_blocks_cal++){

// //       neorv32_uart0_printf("{Block Num %d}\n", this_num_blocks_cal);

// //       Software_Attestation(this_num_blocks_cal, IV_uint8_list[trial_num], clock_prescaler, clock_prescaler_threshold);

// //     }

// //     neorv32_uart0_printf("\n");

// //   }

// //   neorv32_uart0_printf("\n-------------End of Software Attestation-------------\n\n");




//   /**********************************************************************//**
//   * NEORV32 Hybrid Attestation Testing --------------------------------------------------- [2]
//   **************************************************************************/

//   uint32_t IV_uint32_list[5][8] = {
//         {0x3a5e2a96, 0xe1a4a803, 0x49d8c21b, 0xc8f41b70, 0x71eb9128, 0xf333762a, 0xe1659a54, 0xf946d5f5},
//         {0x82db8f14, 0xef6f605e, 0xeedd2f5d, 0xb5976c1d, 0x0b76c2bf, 0x40cf60df, 0xc72bb5a4, 0xd032ea5c},
//         {0x6b6be13d, 0xfbbd2c52, 0x5877f40e, 0x0d3d11c3, 0xe8cbcd4c, 0x7be04512, 0xb29c292a, 0x35c4b06e},
//         {0x2e09484e, 0xfce0d97a, 0x811097a3, 0xc24e93fe, 0x4c6ebc8e, 0xe5f41a76, 0x8b1b0d4f, 0x3c96e830},
//         {0x91802d09, 0x592ce705, 0xdab0be1f, 0x36db8e1e, 0x849a792e, 0xcce78417, 0xa9e072f2, 0xe4cb32d2}
//   };


//   neorv32_uart0_printf("-------------Start of Hybrid Attestation-------------\n\n");

//   neorv32_uart0_printf("RESULTING CLOCK PRE-SCALAR: [%d]\n\n", clock_prescaler_num);

//   for (uint32_t trial_num = 0; trial_num < num_trials; trial_num++){

//     neorv32_uart0_printf("[[[[[[TRIAL NUM %d]]]]]]\n\n", trial_num);

//     neorv32_IV_uint32_print(IV_uint32_list[trial_num]);

//     for (uint32_t this_num_blocks_cal = 1; this_num_blocks_cal <= max_num_blocks_cal; this_num_blocks_cal++){

//       neorv32_uart0_printf("{Block Num %d}\n", this_num_blocks_cal);

//       Hybrid_Attestation(this_num_blocks_cal, IV_uint32_list[trial_num], clock_prescaler, clock_prescaler_threshold);

//     }

//     neorv32_uart0_printf("\n");

//   }

//   neorv32_uart0_printf("\n-------------End of Hybrid Attestation-------------\n\n");


//   for(;;){

//   }

//   /********* Software Attestation Testing**********/






//   /********** CFS & CRC TESTING  *********/


//   /*--- CRC Testing ----*/

//   // //uint32_t ad = 0xFFFFC000;
//   // //uint32_t ad = 0x00000152;
//   // uint32_t CRC_data;
//   // uint32_t CRC_data_addr = 0xffffee08; // data address of CRC
//   // uint8_t da = 2;
//   // neorv32_cpu_store_unsigned_word(CRC_data_addr, da);
//   // neorv32_uart0_printf("\nCRC_DATA_Stored");
//   // //neorv32_uart0_printf("\nL1"); // used to debug bound check failure
//   // //neorv32_uart0_printf("\nL2\n"); // used to debug bound check failure
//   // CRC_data = (uint32_t)neorv32_cpu_load_unsigned_word(CRC_data_addr);
//   // neorv32_uart0_printf("\nCRC_DATA: [%d]", CRC_data);

//   /*--- CFC Testing ----*/

//   // uint32_t CFS_data;
//   // uint32_t CFS_data_addr = 0xffffeb00; // data address of REG[0]
//   // uint32_t da = 12345678;
//   // uint32_t reg_num_wr = 4;
//   // uint32_t reg_num_rd = 4;
//   // neorv32_uart0_printf("\nProcessing CFS Data: [%d]", da);
//   // // neorv32_uart0_printf("\nCFS_DATA before [%d]: [%d]", reg_num_rd, CFS_data);
//   // NEORV32_CFS -> REG[reg_num_wr] = da;
//   // CFS_data = NEORV32_CFS->REG[reg_num_rd];
//   // neorv32_uart0_printf("\nCFS_DATA[%d]: [%d]", reg_num_rd, CFS_data);
//   // neorv32_uart0_printf("\nCFS_DATA_Address[%d]: [%x]", reg_num_rd, &NEORV32_CFS->REG[reg_num_rd]);

//   // uint32_t CFS_data_2;
//   // uint32_t da2 = 87654321;
//   // uint32_t reg_num_wr2 = 5;
//   // uint32_t reg_num_rd2 = 5;
//   // neorv32_uart0_printf("\nProcessing CFS Data2: [%d]", da2);
//   // NEORV32_CFS -> REG[reg_num_wr2] = da2;
//   // CFS_data_2 = NEORV32_CFS->REG[reg_num_rd2];
//   // neorv32_uart0_printf("\nCFS_DATA2[%d]: [%d]", reg_num_rd2, CFS_data_2);
//   // neorv32_uart0_printf("\nCFS_DATA_Address2[%d]: [%x]", reg_num_rd2, &NEORV32_CFS->REG[reg_num_rd2]);

//   // uint32_t CFS_XOR;
//   // CFS_XOR = NEORV32_CFS->REG[6];
//   // neorv32_uart0_printf("\nCFS_DATA_XOR[6]: [%x]", CFS_XOR);


//   /********** TRNG OR PRNG TESTING **********/

  

//   // uint32_t ad = 0x00000000;

//   //   for(;;){
//   //     uint8_t eq = neorv32_cpu_load_unsigned_word(ad);
//   //     neorv32_uart0_printf("LOAD at %x: [%x]\n", ad, eq);
//   //     ad = ad+4;
//   //   }


//   // uint32_t TRNG_SYNTH = neorv32_trng_available();
//   // neorv32_uart0_printf("\nTRNG STATUS: [%u]\n", TRNG_SYNTH); // check for current status of TRNG,  1 for true, 0 for false

//   // uint32_t TRNG_SIM = neorv32_trng_check_sim_mode();
//   // neorv32_uart0_printf("\nTRNG SIM MODE: [%d]\n", TRNG_SIM); // check whether TRNG is in SIM MODE, -1 for PRNG, 0 for TRNG

//   // uint8_t *data1;
//   // neorv32_uart0_printf("\nTRNG DATA0: [%d]\n", *data1); // check whether TRNG is in SIM MODE, -1 for PRNG, 0 for TRNG
//   // uint32_t TRNG_DATA1 = neorv32_trng_get(data1);
//   // neorv32_uart0_printf("\nTRNG DATA1: [%d]\n", *data1); // check whether TRNG is in SIM MODE, -1 for PRNG, 0 for TRNG

//   // uint8_t *data2;
//   // uint32_t TRNG_DATA2 = neorv32_trng_get(data2);
//   // neorv32_uart0_printf("\nTRNG DATA2: [%d]\n", *data2); // check whether TRNG is in SIM MODE, -1 for PRNG, 0 for TRNG


//   /********************************************************************************************************************************************//**
//   BEGIN of ATTESTACTION PROCESS (HASH MEMORY CONTENT)
//   ************************************************************************************************************************************************/
  
//   //universal variables for attestation process 

//   uint32_t curmem_address = 0x80000000u; // current memory address
//   uint32_t mem_address = 0x80000000u; // initial memory address
//   uint32_t seed_val  = 4294967295; // subject to change based upon seed value
//   uint32_t size = 4;

//   /**********************************************************************//**
//   BEGIN of ATTESTACTION PROCESS (HASH MEMORY CONTENT) ---> Hardware Hash Module
//   **************************************************************************/

//   neorv32_uart0_printf("\nNeorv32 Software Attestation: [sha256 hardware] \n\n");
  
//   // Calculation for n_blocks, msg_block_in

//   uint32_t line_size = 32; // 32 bits or 4 byte per line
//   //uint32_t hash_length_uint32 = 8; // hash length in 32-bit

//   uint32_t num_blocks_cal = 2; // number of blocks memory chunk (THIS IS THE INPUT FOR LATER HYBRID_ATTESTATION_TESTING BRANCH)

//   uint32_t block_size = 512; // 512 bits per block in sha256 hardware module
//   uint32_t mem_iter_lines = 2; // number of memory lines per each hash iterartion calculation
//   uint32_t mem_total_lines = mem_iter_lines * num_blocks_cal; // total number of memory lines needs
//   uint32_t mem_loop = mem_total_lines / mem_iter_lines; // based on above 2 variables calcualte the # of time to call sha256
  
//   uint32_t mem_total_bit = mem_total_lines * line_size; 

//   // l+1+k = 448 (mod) 512
//   uint32_t l = mem_iter_lines * line_size + hash_length_uint32 * line_size; // length of Message, M, in bits (for one iteration only) view that as one block.
//   uint32_t k = find_padding_zeros(l); // Number of zeroes appended to a message during the padding step. 
//   uint32_t preprocessed_block_size = l + 1 + k + 64;
  
//   neorv32_uart0_printf("Neorv32 Varaible [l: %d] [k: %d] [preprocessed_block_size: %d]\n\n", l, k, preprocessed_block_size);

//   uint32_t n_block = preprocessed_block_size / block_size; // n_block for that message M, normally 1

//   uint32_t data_out_reg_start = 48; // start of hash output register
//   uint32_t data_out_reg_end = 55; // end of hash output register

//   // for attestation timing
//   uint32_t start, end;
//   uint32_t cpu_time_used;


//   // uint32_t IV_uint32[8] = {
//   //   0x01234567,
//   //   0x89ABCDEF,
//   //   0xDEADBEEF,
//   //   0x00112233,
//   //   0x44556677,
//   //   0x8899AABB,
//   //   0xCCDDEEFF,
//   //   0x10203040
//   // }; // initialization vector (IV) for sha256 (hardware)

//   uint32_t IV_uint32[8] = {
//     0x2948aed1,
//     0xf0dd56fb,
//     0xa12f20e3,
//     0xa4b0cc66,
//     0x931d6455,
//     0x1f0f32b3,
//     0x5ce4e596,
//     0x1e0e2837
//   }; // initialization vector (IV) for sha256 (hardware)

//   //neorv32_uart0_printf("Start: REG[%d]: %x\n", 2, NEORV32_CFS -> REG[2]);

//   /*****INITIALIZE NEORV32 MEMORY SPACE***/

//   neorv32_uart0_printf("Initialize Neorv32 Memory Space\n");
  
//   for (uint32_t i = 0; i<mem_total_lines; i++){

//       neorv32_cpu_store_unsigned_word(curmem_address, seed_val); // filling in the addressed space for calculating the hash
//       // neorv32_uart0_printf("DONE:[%x]\n", neorv32_cpu_load_unsigned_word(curmem_address));
//       curmem_address += size;
      
//   }

//     /**END OF INITIALIZATION*/

//   neorv32_uart0_printf("\nStart Software Attestation!!\n");


//   // set up the attestation timer
//   neorv32_gptmr_enable(); // enable timer
//   neorv32_gptmr_restart(); // restart timer

//   neorv32_gptmr_setup(7, 0, 1000); // main processor clock (clock_prescaler) division by 4096, timer operates in single shot mode, threshold = 1000

//   start = neorv32_gptmr_curtime();
//   neorv32_uart0_printf("\n----- Start Timing ----- [%d]\n", start);
//   // uint8_t hash_data_output[hash_length]; // initialize hash_data_output

//   uint32_t hash_data_output_uint32[hash_length_uint32]; // initialize hash_data_output

//   for (uint32_t loop = 0; loop < mem_loop; loop++){

//     neorv32_cfs_register_rst(); // reset all cfs registers

//     // print the above INITIALIZE NEORV32 MEMORY SPACE for calculation
//     // neorv32_address_print(block_lines, mem_address); 

//     neorv32_uart0_printf("\nMESSAGE[%d]\n", loop);

//     uint32_t data_length_uint32 = mem_iter_lines; // number of memory lines per message block

//     uint32_t hash_mem_uint32[data_length_uint32]; //raw memory content of this message only

//     neorv32_uart0_printf("Getting Memory Content...\n");

//     neorv32_get_mem_uint32(hash_mem_uint32, mem_iter_lines, mem_address); // get the (mem_iter_lines) of memory 
//     //neorv32_uart0_printf("\nhash_data_input address: [%x]\n", hash_data_input); // for debugging purpose only

//     //neorv32_uart0_printf("\nhash_data_[11]: [%x]\n", hash_data_input[11]); // for debugging purpose only

//     uint32_t hash_content_uint32_length = hash_length_uint32 + data_length_uint32;

//     uint32_t hash_content_uint32[hash_content_uint32_length]; //raw memory content + preivous hash ouput of this message

//     neorv32_uart0_printf("Goncatenate Memory Content & Previous Hash...\n");

//     // concatenate the hash_input
//    if(loop == 0)
//       neorv32_hashinput_concatenation_uint32(hash_content_uint32, IV_uint32, hash_length_uint32, hash_mem_uint32, data_length_uint32);
//     else
//       neorv32_hashinput_concatenation_uint32(hash_content_uint32, hash_data_output_uint32, hash_length_uint32, hash_mem_uint32, data_length_uint32);
//     // print the final_hash_input
//     // neorv32_uart0_printf("\n\nFINAL_HASH_INPUT: ");

//     // for debugging hash_content_uint32
//     // for(uint32_t i = 0; i < (hash_content_uint32_length); i++){
//     //   neorv32_uart0_printf("\nWord %u: [%x]", i, hash_content_uint32[i]);
//     // }
    
//     neorv32_uart0_printf("Finalize Padded Message...\n");

//     uint32_t final_padded_message_length = block_size/32;
//     uint32_t final_padded_message[final_padded_message_length];

//     neorv32_message_padding(final_padded_message, hash_content_uint32, final_padded_message_length, l);

//     // for debugging final_padded_message
//     // for(uint32_t i = 0; i < (final_padded_message_length); i++){
//     //   neorv32_uart0_printf("\nWord_P %u: [%x]", i, final_padded_message[i]);
//     // }
    
//     neorv32_uart0_printf("Assign to CFS Registers...\n");
//     neorv32_cfs_register_assign(final_padded_message, final_padded_message_length);

//     //for debugging purpose
//    neorv32_cfs_register_print(16, 31);

//     // import n_block to indicate finish
//     NEORV32_CFS -> REG[1] = n_block;

//     // neorv32_uart0_printf("Before Ready: REG[%d]: %x\n", 2, NEORV32_CFS -> REG[2]);

//     data_ready();
    
//     NEORV32_CFS -> REG[0] = 0;
//     // neorv32_uart0_printf("After Ready: REG[%d]: %x\n", 2, NEORV32_CFS -> REG[2]);

//     neorv32_uart0_printf("Data Ready...\n");

//     // hash output 
//     //neorv32_cfs_register_print(data_out_reg_start, data_out_reg_end);

//     // while(true){

//     //   neorv32_uart0_printf("Checking Finished Signal...\n");
//     //   // int temp = (int) NEORV32_CFS -> REG[63];
//     //   // neorv32_uart0_printf("REG[%d]: %x\n", 2, temp);
//     //   if((uint32_t)NEORV32_CFS -> REG[2] == 1){
//     //     neorv32_uart0_printf("[Finished Hash Calcualtion]\n");
//     //     break;
//     //   }

//     //   neorv32_uart0_printf("Loop End...\n");
    
//     // }


//     neorv32_uart0_printf("Assign Hash Output...\n");

//     // assign hash_ouput
//     neorv32_cfs_register_hash(hash_data_output_uint32, data_out_reg_start, data_out_reg_end);

//     // for(int i = 0; i < hash_length_uint32; i++){
//     //   neorv32_uart0_printf("HASH %d: [%x]\n", i, hash_data_output_uint32[i]);
//     // }

//     neorv32_uart0_printf("\nEND OF Message[%d]\n", loop);

//     // neorv32_hash256(hash_data_output, final_hash_input_length, final_hash_input); // output of hash uint8_t[32]

//     // for (int i = 0; i < 32; i++){
//     //   neorv32_uart0_printf("\nByte %u: [%x]", i, hash_data_output[i]); // checked correct hash memory output (but not line, it's byte) debugging purpose
//     // }

//     // neorv32_hash256_print(hash_data_output); // output of hash uint32_t[8] same as above but different format

//     // mem_address += size * mem_iter_lines; // update initial mem_address for each iteration

//   } // end of calculating first block_memory

//   end = neorv32_gptmr_curtime();
//   neorv32_uart0_printf("\n----- End Timing ----- [%d]\n", end);
//   cpu_time_used = end - start; 
//   neorv32_uart0_printf("Hybrid Attestation Time Used: [%d]\n", cpu_time_used);
    
//   for (;;){
    
//   }







//   /**********************************************************************//**
//   BEGIN of ATTESTACTION PROCESS (HASH MEMORY CONTENT) ---> Software Hash Function
//   **************************************************************************/

//   neorv32_uart0_printf("\n\nNeorv32 Software Attestation: [sha256 software] \n\n");

//   //uint32_t TRNG_Mem_Address = 0xfffffa00;
//   uint32_t block_lines = 4; // lines per each block
//   uint32_t lines = 2; // lines per each hash_data_input
//   uint32_t hash_length = 32; // length of hash output 32 bytes  


//   uint8_t IV[32] = {
//     0x01, 0x23, 0x45, 0x67,
//     0x89, 0xAB, 0xCD, 0xEF,
//     0xDE, 0xAD, 0xBE, 0xEF,
//     0x00, 0x11, 0x22, 0x33,
//     0x44, 0x55, 0x66, 0x77,
//     0x88, 0x99, 0xAA, 0xBB,
//     0xCC, 0xDD, 0xEE, 0xFF,
//     0x10, 0x20, 0x30, 0x40
//   }; // initialization vector (IV) for sha256 (software)

//   // uint8_t TRNG_Value = neorv32_cpu_load_unsigned_byte(TRNG_Mem_Address);
//   // neorv32_uart0_printf("TRNG: [%x]", TRNG_Value);

//   /*****INITIALIZE NEORV32 MEMORY SPACE***/
  
//   for (uint32_t i = 0; i<block_lines; i++){
    
//       neorv32_cpu_store_unsigned_word(curmem_address, seed_val); // filling in the addressed space for calculating the hash
//       curmem_address += size;
      
//     }

//     /**END OF INITIALIZATION*/

//   uint32_t total_loop_per_block = block_lines/lines; 

//   uint8_t hash_data_output[hash_length]; // initialize hash_data_output

//   for (uint32_t loop = 0; loop < total_loop_per_block; loop++){
//     // print the above INITIALIZE NEORV32 MEMORY SPACE for calculation
//     // neorv32_address_print(block_lines, mem_address); 

//     uint32_t data_length = lines*size;

//     uint8_t hash_data_input[data_length]; // initialize hash_data_input

//     neorv32_memory_32s_to_8s(hash_data_input, lines, mem_address, 0); // input of hash uint8_t

//     //neorv32_uart0_printf("\nhash_data_input address: [%x]\n", hash_data_input); // for debugging purpose only

//     //neorv32_uart0_printf("\nhash_data_[11]: [%x]\n", hash_data_input[11]); // for debugging purpose only

//     uint32_t final_hash_input_length = hash_length + data_length;

//     uint8_t final_hash_input[final_hash_input_length];

//     // concatenate the hash_input
//     if(loop == 0)
//       neorv32_hashinput_concatenation(final_hash_input, IV, hash_length, hash_data_input, data_length);
//     else
//       neorv32_hashinput_concatenation(final_hash_input, hash_data_output, hash_length, hash_data_input, data_length);
//     // print the final_hash_input
//     // neorv32_uart0_printf("\n\nFINAL_HASH_INPUT: ");
//     // for(uint32_t i = 0; i < (final_hash_input_length); i++){
//     //   neorv32_uart0_printf("\nByte %u: [%x]", i, final_hash_input[i]);
//     // }

//     neorv32_hash256(hash_data_output, final_hash_input_length, final_hash_input); // output of hash uint8_t[32]

//     // for (int i = 0; i < 32; i++){
//     //   neorv32_uart0_printf("\nByte %u: [%x]", i, hash_data_output[i]); // checked correct hash memory output (but not line, it's byte) debugging purpose
//     // }

//     neorv32_hash256_print(hash_data_output); // output of hash uint32_t[8] same as above but different format

//     mem_address += size*lines; // update initial mem_address for each iteration

//   } // end of calculating first block_memory

//   for (;;){
    
//   }



//   /**********************************************************************//**
//   END of ATTESTACTION PROCESS (HASH MEMORY CONTENT)
//   **************************************************************************/





//   //uint32_t mem_address = 0x80000000u;
//   char str[] = "0x8000000";
//   uint32_t mem_data_w1 = 0x41;
//   //char random[] = "32 bit random value";

//   //neorv32_cpu_store_unsigned_word(mem_address, random);
  
//   // for(;;){
//   //   neorv32_uart0_puts("MEM_ADDRESS: ");
//   //   //char str[] = mem_address; // Adjust the array size based on the expected length of the number
//   //   // Use sprintf to convert the uint32_t to a string
//   //   //sprintf(str, "%u", mem_address);
//   //   //neorv32_uart0_printf(%u, );
    
//   //   //neorv32_uart0_puts(" --> ");
//   //   for(int i=0; i<1; i++){
//   //     neorv32_uart0_printf("\n[0x%x] => ", mem_address);
//   //     mem_data_w1 = (uint32_t)neorv32_cpu_load_unsigned_word(mem_address); 
//   //     neorv32_uart0_puts("val: ");
//   //     neorv32_uart0_printf("[0x%x] => ", mem_data_w1);
//   //     neorv32_uart0_puts(" Hex: ");
//   //     mem_address += 4;
//   //     aux_print_hex_byte((uint8_t)(mem_data_w1 >> 24));
//   //     aux_print_hex_byte((uint8_t)(mem_data_w1 >> 16));
//   //     aux_print_hex_byte((uint8_t)(mem_data_w1 >>  8));
//   //     aux_print_hex_byte((uint8_t)(mem_data_w1 >>  0));
//   //     neorv32_uart0_puts("\n\n");
//   //   }
//   // }
  
  
//   /**********************************************************************//**
//   SHA256 Below ()
//   **************************************************************************/

//   struct tc_sha256_state_struct sha256_state; 

//   neorv32_uart0_puts("\nStart of SHA256 :)\n");

//   uint32_t TC_CRYPTO_SUCCESS = 1;

//   //TCSha256State_t sha256_state = (TCSha256State_t)malloc(sizeof(struct tc_sha256_state_struct));

//   //TCSha256State_t sha256_state;

//   uint32_t init_result = tc_sha256_init(&sha256_state);

//   if (init_result != TC_CRYPTO_SUCCESS){
//     neorv32_uart0_printf("SHA256 Init Unsuccessful");
//   }
//   else{
//     neorv32_uart0_printf("SHA256 Init Successful");
//   }

//   //uint8_t data[] = "THIS IS SHA-256!";
//   uint8_t data[] = "SHA-256";
//   size_t data_len = sizeof(data)-1;
  
//   neorv32_uart0_printf("\nSHA256 DATA INPUT : [%s]", data);

//   int update_result = tc_sha256_update(&sha256_state, data, data_len);

//   if (update_result != TC_CRYPTO_SUCCESS){
//     neorv32_uart0_printf("\n\nSHA256 Update Unsuccessful\n");
//   }
//   else{
//     neorv32_uart0_printf("\n\nSHA256 Update Successful\n");
//     neorv32_uart0_printf("[SHA256 Update Result 0/1: ] ==> %u", update_result);
//     //neorv32_uart0_printf(update_result);
//   }

//   //uint32_t hash_length = 32;

//   uint8_t hash[hash_length];

//   //char hash_value[hash_length];

//   uint32_t final_hash = 0;

//   uint32_t final_hashs[8];

//   uint32_t long_hashs[1500]; // for memory access debugging purpose (try to figure out the maximum it allowed to allocate on data address space)

//   //neorv32_uart0_printf("[String Length: ] ==> %u", strlen(hash_value));

//   //uint8_t hash2[32]="HI";

//   /*****BELOW IS JUST FOR THE SAKE OF UNDERSTANDING HOW THEY STORE VALUES ON DATA ADDRESS SPACE ******/

//   neorv32_uart0_printf("\n\n[SHA256 init HASH_LENGTH Address: ] ==> %x", &hash_length);
//   neorv32_uart0_printf("\n\n[SHA256 init HASH Address: ] ==> %x", hash);
//   neorv32_uart0_printf("\n\n[SHA256 init FINAL_HASHs Address: ] ==> %x", final_hashs);
//   neorv32_uart0_printf("\n\n[SHA256 init LONG_HASHs Address: ] ==> %x", long_hashs);

//   /***************************************************************************/

//   //neorv32_uart0_printf("\n\n[SHA256 init HASH2: ] ==> %x", hash2);

//   int final_result = tc_sha256_final(hash, &sha256_state);
//   if (final_result != TC_CRYPTO_SUCCESS){
//     neorv32_uart0_printf("\n\nSHA256 Final Unsuccessful");
//   }
//   else{
//     neorv32_uart0_printf("\n\nSHA256 Final Successful\n");
//     //final_hash = hash[0];
//     uint32_t index = 0;
//     for(uint32_t i = 0; i < hash_length; i++){

//       final_hash = (uint32_t)(final_hash << 8) + (uint32_t)hash[i];
      
//       if((i+1)%4 == 0){
//         //neorv32_uart0_printf("TRUE");
//         final_hashs[index] = final_hash;
//         //neorv32_uart0_printf("\n%x"final_hashs[i]);
//         final_hash = 0;
//         index+=1;
//       }

//       //********** HASH DEBUGGING ******//

//       //neorv32_uart0_printf("[SHA256 Final Result: %u] ==> %x\n", i, hash[i]);
//       //neorv32_uart0_printf("%02x",hash[i]);
//       // hash_value[i] = hash[i];

//       //********** HASH DEBUGGING ******//

//     }
//     //neorv32_uart0_printf("[SIZE: ] ==> %u", sizeof(*hash));
//     //neorv32_uart0_printf(final_result);
//   }



//   /**********************************************************************//**
//   PRINT SHA256 Below
//   **************************************************************************/
  
//   // for (int i = 0; i < 8; i++){
//   //   neorv32_uart0_printf("\n%x",final_hashs[i]);
//   // }

//   /**********************************************************************//**
//   PRINT SHA256 Above
//   **************************************************************************/

//   neorv32_uart0_printf("\n");
//   //neorv32_uart0_printf("\n\nFINAL HASH VALUE: %x", hash_value);

//   uint8_t correct_hash[] = {0xbb, 0xd0, 0x7c, 0x4f, 0xc0, 0x2c, 0x99, 0xb9, 0x71, 0x24, 0xfe, 0xbf, 0x42, 0xc7, 0xb6, 0x3b, 0x50, 0x11, 0xc0, 0xdf, 0x28, 0xd4, 0x09, 0xfb, 0xb4, 0x86, 0xb5, 0xa9, 0xd2, 0xe6, 0x15, 0xea};

//   uint32_t flag = 1;

//   for(uint32_t i = 0; i < hash_length; i++){
//     if (correct_hash[i]!=hash[i]){
//       flag = 0;
//       neorv32_uart0_printf("HASH INCORRECT COMPORMISED");
//       break;
//     }
//   }

//   if (flag)
//     neorv32_uart0_printf("HASH CORRECT NOT COMPORMISED");

//   /**********************************************************************//**
//   END of SHA256 Above
//   **************************************************************************/





  
//     //char random[] = "C char str[] = mem_address Adjust the array size based on the expected length of the numberC char str[] = mem_address Adjust the array size based on the expected length of the number";

//     uint32_t random = 4294967295;
//     neorv32_cpu_store_unsigned_word(mem_address, random);
//     uint32_t loader = (uint32_t)neorv32_cpu_load_unsigned_word(mem_address); 
//     neorv32_uart0_printf("\n\n[0x%u] => ", random);

//     neorv32_uart0_printf("\n[Results: %u] \n\n", loader);






//   /**********************************************************************//**
//   BEGIN of Finding the MAX/AVAILABLE address
//   **************************************************************************/

//     //mem_address = 0x7FFFFFFC; // FOR DEBUGING REGION BETWEEN INTERNAL IMEM TO INTERNAL DMEM
//     //mem_address = 0xF0000008; // FOR DEBUGING REGION BETWEEN MEMORY_MAPPED XIP FLASH TO INTERNAL BOOTLOADER ROM
//     //mem_address = 0x80000000; // FOR DEBUGING REGION BETWEEN INTERNAL DMEM TO MEMORY_MAPPED XIP FLASH
//     //mem_address = 0x00000000; // FOR DEBUGING INTERNAL IMEM
//     //mem_address = 0xFFFFFFFC; // END OF THE MEMORY ADDRESS SPACE

//     for(;;){
//       neorv32_cpu_store_unsigned_word(mem_address, random);
//       //neorv32_uart0_printf("\nL1"); // used to debug bound check failure
//       neorv32_uart0_printf("\n[0x%x]", mem_address);
//       //neorv32_uart0_printf("\nL2\n"); // used to debug bound check failure
//       mem_data_w1 = (uint32_t)neorv32_cpu_load_unsigned_word(mem_address);

//       // neorv32_uart0_puts("val: ");
//       // neorv32_uart0_printf("[0x%u] => ", mem_data_w1); 

//       //neorv32_uart0_printf("\nL3\n"); // used to debug bound check failure

//       // neorv32_uart0_puts(" Hex: ");
//       // aux_print_hex_byte((uint8_t)(mem_data_w1 >> 24));
//       // aux_print_hex_byte((uint8_t)(mem_data_w1 >> 16));
//       // aux_print_hex_byte((uint8_t)(mem_data_w1 >>  8));
//       // aux_print_hex_byte((uint8_t)(mem_data_w1 >>  0));
//       // neorv32_uart0_puts("\n");

//       mem_address+=4;
//     }

//   /**********************************************************************//**
//   END of Finding the MAX/AVALIABLE address
//   **************************************************************************/

//     for(;;){
//     neorv32_uart0_puts("MEM_ADDRESS: ");
//     //char str[] = mem_address; // Adjust the array size based on the expected length of the number
//     // Use sprintf to convert the uint32_t to a string
//     //sprintf(str, "%u", mem_address);
//     //neorv32_uart0_printf(%u, );
    
//     //neorv32_uart0_puts(" --> ");
//     neorv32_uart0_printf("\n[0x%x] => ", mem_address);
//     mem_data_w1 = (uint32_t)neorv32_cpu_load_unsigned_word(mem_address);
//     neorv32_uart0_puts("val: ");
//     neorv32_uart0_printf("[0x%u] => ", mem_data_w1);
//     neorv32_uart0_puts(" Hex: ");
//     aux_print_hex_byte((uint8_t)(mem_data_w1 >> 24));
//     aux_print_hex_byte((uint8_t)(mem_data_w1 >> 16));
//     aux_print_hex_byte((uint8_t)(mem_data_w1 >>  8));
//     aux_print_hex_byte((uint8_t)(mem_data_w1 >>  0));
//     neorv32_uart0_puts("\n");

//       for(int i=0; i<4; i++){
//         neorv32_uart0_printf("\n[0x%x] => ", mem_address);
//         mem_data_w1 = (uint32_t)neorv32_cpu_load_unsigned_byte(mem_address); 
//         neorv32_uart0_puts("val: ");
//         neorv32_uart0_printf("[0x%u] => ", mem_data_w1);
//         neorv32_uart0_puts(" Hex: ");
//         mem_address += 1;
//         aux_print_hex_byte(mem_data_w1);
//         neorv32_uart0_puts("\n\n");
//       }
    
    
//    }
  
//   // uint32_t mem_data_w1 = 0;
//   // mem_data_w1 = (uint32_t)neorv32_cpu_load_unsigned_word(mem_address); 
//   // neorv32_uart0_puts(mem_data_w1);
//   // neorv32_uart0_puts("\n\n");
//   // char mem_data_w2[] = "greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!greeting!!!!!!";
//   // neorv32_cpu_store_unsigned_word(mem_address, mem_data_w2);
//   // mem_data_w1 = (uint32_t)neorv32_cpu_load_unsigned_word(mem_address); 
//   // neorv32_uart0_puts(mem_data_w1);
//   // neorv32_uart0_puts("\n\n");

//   // mem_address += 4;
//   // mem_data_w1 = (uint32_t)neorv32_cpu_load_unsigned_word(mem_address); 
//   // neorv32_uart0_puts(mem_data_w1);
//   // neorv32_uart0_puts("\n\n");
//   // uint32_t mem_data_w = 'A';

  

//   // debugging for load and store

//   // int num = 0;

//   // for(;;){
//   //   num += 1;
//   //   neorv32_uart0_printf(num);
//   //   //neorv32_uart0_puts(num);
//   //   neorv32_uart0_puts(" : ");
//   //   neorv32_uart0_puts(mem_data_w);
//   //   neorv32_uart0_puts(" : ");

//   //   // printf(num);
//   //   // printf(" : ");
//   //   // printf(mem_data_w);
//   //   // printf(" : ");

//   //   neorv32_cpu_store_unsigned_word(mem_address, mem_data_w);

//   //   mem_data_w1 = (uint32_t)neorv32_cpu_load_unsigned_word(mem_address); 
//   //   neorv32_uart0_puts(mem_data_w1);
//   //   //printf(mem_data_w1);
//   //   mem_data_w += 1;
//   //   //mem_address += 32;
//   //   neorv32_uart0_puts("\n\n");
//   // }

//   neorv32_uart0_puts("DONE");

//   return 0;
// }


