#define NEORV32_IMEM_BASE       (0x00000000U) /**< Start of Instruction Address Space */
#define NEORV32_DATA_BASE       (0x80000000U) /*<Start of Data Address Space>*/
#define NEORV32_UART0_BASE      (0xFFFFFFA0U) /**< Primary Universal Asynchronous Receiver and Transmitter (UART0) */

#define TASK0_BASE_ADDR         (0x00020000U)
#define BOOTLOADER_BASE_ADDRESS (0xFFFF0000U)
#define TASK_STATUS_TABLE_BASE  (0x0002F000U)
#define ZEPHYR_MY_ISR_BASE      (0x00012000U)


/*************************************//**
* NEORV32 CFS_REG Configure Parameters
**************************************/
#define CFS_REG_NUM            (64) /**Total number of cfs_reg in NEORV32**/
#define MAX_NUM_TASKS          (10) /**Maximum number of tasks allowed to upload to CFS_REG*/
#define CFS_CTRL_REG_SET      (1) /**<set cfs_reg[2]>**/
#define CFS_CTRL_REG_UNSET    (2) /**<unset cfs_reg[2]>**/
#define CFS_UPDATE_REG_SET      (1) /**<set cfs_reg[3]>**/
#define CFS_UPDATE_REG_UNSET    (2) /**<unset cfs_reg[3]>**/
#define INT_FLAG_REG_SET      (1) /**<set cfs_reg[6]>**/
#define INT_FLAG_REG_UNSET    (2) /**<unset cfs_reg[6]>**/
#define TASK0_STATE_REG_BOOTING (0)
#define TASK0_STATE_REG_INTERRUPT (1)
#define TASK0_STATE_REG_CONTEXT_SWITCH (2) 

/*********************************//**
* NEORV32 CFS_REG Index Definition
**********************************/
#define TASK0_STATE_REG        (0)  /**< Track Task0 State, Status of Task0 (Ex: init stage, )>*/   
#define TASK_NUM_REG           (1)  /**< Keep track of Num of Tasks>*/       
#define CFS_CTRL_REG           (2)  /**< set before updating task_status_table >*/  
#define CFS_UPDATE_REG         (3)  /**< reset all parameter in vhdl upon upload/update new policy table>*/
#define INT_PC_FORWARD_REG     (4)  /**stores PC right before entering task0, later used by zephyr interrupt to jump to that PC*/
#define INT_PC_BACKWARD_REG    (5)  /**stores PC of interrupt handler of zephyr, for getting back to zephyr from task0*/
#define INT_FLAG_REG           (6)  /*check whether interrupt has gone to task0 already*/
#define TASK1_STARTING_REG     (10) /*Start of cfs_reg that could be used for the tasks*/
#define NEORV32_PC_REG         (63) /**< PC_Fetch REG# >**/
#define NEORV32_INT_REG        (62) /**< Interrupt REG# >**/


#define BAUD_RATE 19200

// global verbose to enable/disable debug printf
bool verbose = 0;
// global enable for rte
bool enable = 1;
// global enable for hybrid attestation
bool verbose_hybridAttestation = 0;

// hybrid attestation flag (1: testing --> mainly for visulization purpose, 0: actual attestation)
bool hybrid_att_testing = 1;

// task0 init timing flag
bool task0_init_timing = 0;
// for task0 initialization timing
uint32_t task0_init_start, task0_init_end;
uint32_t task0_init_cpu_time_used;
uint32_t task0_init_clock_prescaler = 7; // 7 represent f(clock frequency)/4096 --> subjected to change to fit for your data plot
uint32_t task0_init_clock_prescaler_threshold = 100000;  // (recommended) set it to as big as possible


/**********************************************************************//**
 * Print help menu.
 **************************************************************************/
void print_help(void) {

  // unsigned int sp;
  // __asm__ volatile ("mv %0, sp" : "=r" (sp));
  // if(verbose)
  //   neorv32_uart0_printf("print_help [sp]: [%x]\n", sp);

  // unsigned int ra_val;
  // __asm__ volatile ("mv %0, ra" : "=r" (ra_val));
  // if(verbose)
  //   neorv32_uart0_printf("print_help [ra]: [%x]\n", ra_val);

  neorv32_uart0_printf("Available CMDs:\n"
             " h: Help\n"
             " r: Restart\n"
             " t: Restart <Task0 Bootloader>\n"
             " u: Upload\n"
             " e: Execute\n");

  
}

/**********************************************************************//**
 * This global variable keeps the size of the available executable in bytes.
 * If =0 no executable is available (yet).
 **************************************************************************/
volatile uint32_t exe_available;


/**********************************************************************//**
 * Only set during executable fetch (required for capturing STORE BUS-TIMOUT exception).
 **************************************************************************/
volatile uint32_t getting_exe;


/**********************************************************************//**
  Executable stream source select (for copying into IMEM)
 **************************************************************************/
enum EXE_STREAM_SOURCE_enum {
  EXE_STREAM_UART  = 0, /**< Get executable via UART */
  EXE_STREAM_FLASH = 1  /**< Get executable via SPI flash */
};


/**********************************************************************//**
 * NEORV32 executable
 **************************************************************************/
enum NEORV32_EXECUTABLE_enum {
  EXE_OFFSET_SIGNATURE =  0, /**< Offset in bytes from start to signature (32-bit) */
  EXE_OFFSET_SIZE      =  4, /**< Offset in bytes from start to size (32-bit) */
  EXE_OFFSET_CHECKSUM  =  8, /**< Offset in bytes from start to checksum (32-bit) */
  EXE_OFFSET_DATA      = 12, /**< Offset in bytes from start to data (32-bit) */
};


/**********************************************************************//**
 * Valid executable identification signature
 **************************************************************************/
#define EXE_SIGNATURE 0x4788CAFE


/**********************************************************************//**
 * Get word from executable stream
 *
 * @param src Source of executable stream data. See #EXE_STREAM_SOURCE_enum.
 * @return 32-bit data word from stream.
 **************************************************************************/
uint32_t get_exe_word(int src) {

  union {
    uint32_t uint32;
    uint8_t  uint8[sizeof(uint32_t)];
  } data;

  uint32_t i;
  for (i=0; i<4; i++) {
    if (src == EXE_STREAM_UART) {
      data.uint8[i] = (uint8_t)neorv32_uart_getc(NEORV32_UART0);
    }
  }

  return data.uint32;
}


/**********************************************************************//**
 * Get executable stream.
 *
 * @param src Source of executable stream data. See #EXE_STREAM_SOURCE_enum.
 **************************************************************************/
void get_exe(int src) {

  getting_exe = 1; // to inform trap handler we were trying to get an executable

  // get image from UART?
  if (src == EXE_STREAM_UART) {
    neorv32_uart0_printf("Awaiting I-O-Guard_main_exe.bin... ");
  }

  // check if valid image
  uint32_t signature = get_exe_word(src);
  if (signature != EXE_SIGNATURE) { // signature
    neorv32_uart0_printf("ERROR EXE_SIGNATURE!!\n");
    while(1);
    return;
  }

  // image size and checksum
  uint32_t size  = get_exe_word(src); // size in bytes
  uint32_t check = get_exe_word(src); // complement sum checksum

  // transfer program data
  uint32_t *pnt = (uint32_t*)NEORV32_IMEM_BASE;
  uint32_t checksum = 0;
  uint32_t d = 0, i = 0;

  while (i < (size/4)) { // in words
    d = get_exe_word(src);
    checksum += d;
    pnt[i++] = d;
  }

  // error during transfer?
  if ((checksum + check) != 0) {
    neorv32_uart0_printf("ERROR CHECKSUM!!\n");
    while(1);
    return;
  }
  else {
    neorv32_uart0_printf("OK");
    exe_available = size; // store exe size
  }

  getting_exe = 0; // to inform trap handler we are done getting an executable
}


/**********************************************************************//**
 * Start application program.
 *
 **************************************************************************/
void start_app() {

  // deactivate global IRQs
  // neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

  register uint32_t app_base = NEORV32_IMEM_BASE; // default = start at beginning of IMEM

  neorv32_uart0_printf("Booting Zephyr OS from [%x] ....\n\n", app_base);

  // wait for UART0 to finish transmitting
  // while (neorv32_uart0_tx_busy());

  // // memory sync
  // if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZIFENCEI)) { // Zifenci ISA extension available?
  //   asm volatile ("fence.i");
  // }
  // asm volatile ("fence");

  // start application
  __asm__ volatile ("jalr ra, %0" : : "r" (app_base));

  __builtin_unreachable();
  while (1); // should never be reached
}


/**********************************************************************//**
 * Get executable stream.
 *
 * @param src Source of executable stream data. See #EXE_STREAM_SOURCE_enum.
 **************************************************************************/
void get_exe_PT(int src) {

  // get image from UART?
  if (src == EXE_STREAM_UART) {
    neorv32_uart0_printf("Awaiting I-O-Guard_task_status_table.bin... ");
  }

  // check if valid image
  uint32_t signature = get_exe_word(src);
  if (signature != EXE_SIGNATURE) { // signature
    neorv32_uart0_printf("ERROR EXE_SIGNATURE!!\n");
    while(1);
  }

  // task_status_table size
  uint32_t size  = get_exe_word(src); // size in bytes

  // transfer data
  uint32_t *pnt = (uint32_t*)TASK_STATUS_TABLE_BASE;
  uint32_t d = 0, i = 0;

  while (i < (size/4)) { // in words
    d = get_exe_word(src);
    //checksum += d;
    pnt[i++] = d;
  }

  neorv32_uart0_printf("OK");

}


/**********************************************************************//**
 * Print help menu. (Get_PT)
 **************************************************************************/
void print_help_PT(void) {

  neorv32_uart0_printf("Available CMDs:\n"
             " h: Help\n"
             " u: Upload\n"
             " q: Quit\n");

}