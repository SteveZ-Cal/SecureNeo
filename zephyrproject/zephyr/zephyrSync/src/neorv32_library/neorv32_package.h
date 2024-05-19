
#define NEORV32_CFS_BASE       (0xFFFFFE00U) /**< Custom Functions Subsystem (CFS) */
#define NEORV32_DATA_BASE      (0x800000000) /*<Start of Data Address Space>*/
#define NEORV32_UART0_BASE     (0xFFFFFFA0U) /**< Primary Universal Asynchronous Receiver and Transmitter (UART0) */
#define I_O_GUARD_TASK0_BASE   (0x00020000U) 
#define NEORV32_MTIME_BASE   (0xFFFFF400U) /**< Machine System Timer (MTIME) */

#define ZEPHYR_MY_ISR_BASE      (0x00012000U)
#define TASK1_REGION_BASE       (0x0000f000U)
#define TASK2_REGION_BASE       (0x0000f400U)
#define TASK3_REGION_BASE       (0x0000f800U)
#define TASK4_REGION_BASE       (0x0000fC00U)

#define TASK_STATUS_TABLE_BASE  (0x0002F000U)

#define MIP_BASE               (0x344)

/*************************************//**
* NEORV32 CFS_REG Configure Parameters
**************************************/
#define CFS_CTRL_REG_SET      (1) /**<set cfs_reg[2]>**/
#define CFS_CTRL_REG_UNSET    (2) /**<unset cfs_reg[2]>**/
#define CFS_UPDATE_REG_SET      (1) /**<set cfs_reg[3]>**/
#define CFS_UPDATE_REG_UNSET    (2) /**<unset cfs_reg[3]>**/
#define INT_FLAG_REG_SET      (1) /**<set cfs_reg[6]>**/
#define INT_FLAG_REG_UNSET    (2) /**<unset cfs_reg[6]>**/
#define MAX_NUM_TASKS          (10) /**Maximum number of tasks allowed to upload to CFS_REG*/

/*********************//**
* Define CFS_REG_NUM
************************/
#define TASK0_STATE_REG        (0)  /**< Track Task0 State>*/   
#define TASK_NUM_REG           (1)  /**< Keep track of Num of Tasks>*/       
#define CFS_CTRL_REG           (2)  /**< set before updating task_status_table >*/  
#define CFS_UPDATE_REG         (3)  /**< reset all parameter in vhdl upon upload/update new policy table>*/
#define INT_PC_FORWARD_REG     (4)  /**stores PC right before entering task0, later used by zephyr interrupt to jump to that PC*/
#define INT_PC_BACKWARD_REG    (5)  /**stores PC of interrupt handler of zephyr, for getting back to zephyr from task0*/
#define INT_FLAG_REG           (6)  /*check whether interrupt has gone to task0 already*/
#define TASK1_STARTING_REG     (10)  /*Start of cfs_reg that could be used for the tasks*/
// #define NEORV32_PC_REG         (63) /**< PC_Fetch REG# >**/
// #define NEORV32_INT_REG        (62) /**< Interrupt REG# >**/
// #define NEORV32_TIMER_REG      (60) /*<self-made timer REG#>*/

#define sizeof_uint32_t         (4)

// global verbose to enable/disable debug printf
bool verbose = 0;

// global enable for paper timing evaluation
bool latencyEnable = 1;
bool latencyVerbose = 1;
uint32_t latencyStart = 0;
uint32_t latencyEnd = 0;

// zephyr init timing flag
bool zephyr_init_timing = 0;
// for zephyr initialization timing
uint32_t zephyr_init_start, zephyr_init_end;
uint32_t zephyr_init_cpu_time_used;
uint32_t zephyr_init_clock_prescaler = 7; // 7 represent f(clock frequency)/4096 --> subjected to change to fit for your data plot
uint32_t zephyr_init_clock_prescaler_threshold = 100000;  // (recommended) set it to as big as possible

// irq timing flag
bool irq_timing = 1;
// for irq timing
uint32_t irq_start, irq_end;
uint32_t irq_cpu_time_used;
uint32_t irq_clock_prescaler = 7; // 7 represent f(clock frequency)/4096 --> subjected to change to fit for your data plot
uint32_t irq_clock_prescaler_threshold = 100000;  // (recommended) set it to as big as possible

/**********************************************************************//**
 * @name IO Device: Custom Functions Subsystem (CFS)
 **************************************************************************/
/**@{*/
/** CFS module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t REG[64]; /**< offset 4*0..4*63: CFS register 0..63, user-defined */
} neorv32_cfs_t;

/** CFS module hardware access (#neorv32_cfs_t) */
#define NEORV32_CFS ((neorv32_cfs_t*) (NEORV32_CFS_BASE))
/**@}*/


/**********************************************************************//**
 * @name IO Device: Primary/Secondary Universal Asynchronous Receiver and Transmitter (UART0 / UART1)
 **************************************************************************/
/**@{*/
/** UART module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;  /**< offset 0: control register (#NEORV32_UART_CTRL_enum) */
  uint32_t DATA;  /**< offset 4: data register  (#NEORV32_UART_DATA_enum) */
} neorv32_uart_t;

/** UART0 module hardware access (#neorv32_uart_t) */
#define NEORV32_UART0 ((neorv32_uart_t*) (NEORV32_UART0_BASE))

/** UART control register bits */
enum NEORV32_UART_CTRL_enum {
  UART_CTRL_EN            =  0, /**< UART control register(0)  (r/w): UART global enable */
  UART_CTRL_SIM_MODE      =  1, /**< UART control register(1)  (r/w): Simulation output override enable */
  UART_CTRL_HWFC_EN       =  2, /**< UART control register(2)  (r/w): Enable RTS/CTS hardware flow-control */
  UART_CTRL_PRSC0         =  3, /**< UART control register(3)  (r/w): clock prescaler select bit 0 */
  UART_CTRL_PRSC1         =  4, /**< UART control register(4)  (r/w): clock prescaler select bit 1 */
  UART_CTRL_PRSC2         =  5, /**< UART control register(5)  (r/w): clock prescaler select bit 2 */
  UART_CTRL_BAUD0         =  6, /**< UART control register(6)  (r/w): BAUD rate divisor, bit 0 */
  UART_CTRL_BAUD1         =  7, /**< UART control register(7)  (r/w): BAUD rate divisor, bit 1 */
  UART_CTRL_BAUD2         =  8, /**< UART control register(8)  (r/w): BAUD rate divisor, bit 2 */
  UART_CTRL_BAUD3         =  9, /**< UART control register(9)  (r/w): BAUD rate divisor, bit 3 */
  UART_CTRL_BAUD4         = 10, /**< UART control register(10) (r/w): BAUD rate divisor, bit 4 */
  UART_CTRL_BAUD5         = 11, /**< UART control register(11) (r/w): BAUD rate divisor, bit 5 */
  UART_CTRL_BAUD6         = 12, /**< UART control register(12) (r/w): BAUD rate divisor, bit 6 */
  UART_CTRL_BAUD7         = 13, /**< UART control register(13) (r/w): BAUD rate divisor, bit 7 */
  UART_CTRL_BAUD8         = 14, /**< UART control register(14) (r/w): BAUD rate divisor, bit 8 */
  UART_CTRL_BAUD9         = 15, /**< UART control register(15) (r/w): BAUD rate divisor, bit 9 */

  UART_CTRL_RX_NEMPTY     = 16, /**< UART control register(16) (r/-): RX FIFO not empty */
  UART_CTRL_RX_HALF       = 17, /**< UART control register(17) (r/-): RX FIFO at least half-full */
  UART_CTRL_RX_FULL       = 18, /**< UART control register(18) (r/-): RX FIFO full */
  UART_CTRL_TX_EMPTY      = 19, /**< UART control register(19) (r/-): TX FIFO empty */
  UART_CTRL_TX_NHALF      = 20, /**< UART control register(20) (r/-): TX FIFO not at least half-full */
  UART_CTRL_TX_FULL       = 21, /**< UART control register(21) (r/-): TX FIFO full */

  UART_CTRL_IRQ_RX_NEMPTY = 22, /**< UART control register(22) (r/w): Fire IRQ if RX FIFO not empty */
  UART_CTRL_IRQ_RX_HALF   = 23, /**< UART control register(23) (r/w): Fire IRQ if RX FIFO at least half-full */
  UART_CTRL_IRQ_RX_FULL   = 24, /**< UART control register(24) (r/w): Fire IRQ if RX FIFO full */
  UART_CTRL_IRQ_TX_EMPTY  = 25, /**< UART control register(25) (r/w): Fire IRQ if TX FIFO empty */
  UART_CTRL_IRQ_TX_NHALF  = 26, /**< UART control register(26) (r/w): Fire IRQ if TX FIFO not at least half-full */

  UART_CTRL_RX_OVER       = 30, /**< UART control register(30) (r/-): RX FIFO overflow */
  UART_CTRL_TX_BUSY       = 31  /**< UART control register(31) (r/-): Transmitter busy or TX FIFO not empty */
};

/** UART data register bits */
enum NEORV32_UART_DATA_enum {
  UART_DATA_RTX_LSB          =  0, /**< UART data register(0) (r/w): UART receive/transmit data, LSB */
  UART_DATA_RTX_MSB          =  7, /**< UART data register(7) (r/w): UART receive/transmit data, MSB */

  UART_DATA_RX_FIFO_SIZE_LSB =  8, /**< UART data register(8)  (r/-): log2(RX FIFO size), LSB */
  UART_DATA_RX_FIFO_SIZE_MSB = 11, /**< UART data register(11) (r/-): log2(RX FIFO size), MSB */

  UART_DATA_TX_FIFO_SIZE_LSB = 12, /**< UART data register(12) (r/-): log2(RX FIFO size), LSB */
  UART_DATA_TX_FIFO_SIZE_MSB = 15, /**< UART data register(15) (r/-): log2(RX FIFO size), MSB */
};
/**@}*/

/**********************************************************************//**
 * Send single char via UART.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 * @param[in] c Char to be send.
 **************************************************************************/
void neorv32_uart_putc(neorv32_uart_t *UARTx, char c) {

  // wait for previous transfer to finish
  while ((UARTx->CTRL & (1<<UART_CTRL_TX_FULL))); // wait for free space in TX FIFO
  UARTx->DATA = (uint32_t)c << UART_DATA_RTX_LSB;
}

/**********************************************************************//**
 * Get char from UART.
 *
 * @note This function is blocking.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 * @return Received char.
 **************************************************************************/
char neorv32_uart_getc(neorv32_uart_t *UARTx) {

  while (1) {
    if (UARTx->CTRL & (1<<UART_CTRL_RX_NEMPTY)) { // data available?
      return (char)(UARTx->DATA >> UART_DATA_RTX_LSB);
    }
  }
}


/**********************************************************************//**
 * Read data from CPU control and status register (CSR).
 *
 * @param[in] csr_id ID of CSR to read. See #NEORV32_CSR_enum.
 * @return Read data (uint32_t).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_csr_read(const int csr_id) {

  uint32_t csr_data;

  __asm__ volatile ("csrr %[result], %[input_i]" : [result] "=r" (csr_data) : [input_i] "i" (csr_id));

  return csr_data;
}

/**********************************************************************//**
 * Set bit(s) in CPU control and status register (CSR).
 *
 * @param[in] csr_id ID of CSR to write. See #NEORV32_CSR_enum.
 * @param[in] mask Bit mask (high-active) to set bits (uint32_t).
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_csr_set(const int csr_id, uint32_t mask) {

  uint32_t csr_data = mask;

  __asm__ volatile ("csrs %[input_i], %[input_j]" :  : [input_i] "i" (csr_id), [input_j] "r" (csr_data));
}

/**********************************************************************//**
 * Clear bit(s) in CPU control and status register (CSR).
 *
 * @param[in] csr_id ID of CSR to write. See #NEORV32_CSR_enum.
 * @param[in] mask Bit mask (high-active) to clear bits (uint32_t).
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_csr_clr(const int csr_id, uint32_t mask) {

  uint32_t csr_data = mask;

  __asm__ volatile ("csrc %[input_i], %[input_j]" :  : [input_i] "i" (csr_id), [input_j] "r" (csr_data));
}

/**********************************************************************//**
 * @name Fast Interrupt Requests (FIRQ) device aliases
**************************************************************************/
/** @name Custom Functions Subsystem (CFS) */
/**@{*/
#define CFS_FIRQ_ENABLE        CSR_MIE_FIRQ1E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define CFS_FIRQ_PENDING       CSR_MIP_FIRQ1P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define CFS_RTE_ID             RTE_TRAP_FIRQ_1   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define CFS_TRAP_CODE          TRAP_CODE_FIRQ_1  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */


/**********************************************************************//**
 * Available CPU Control and Status Registers (CSRs)
 **************************************************************************/
enum NEORV32_CSR_enum {
  /* hardware-only CSR, NEORV32-specific, not accessible by software */
//CSR_ZERO           = 0x000, /**< 0x000 - zero: Always zero */

  /* floating-point unit control and status */
  CSR_FFLAGS         = 0x001, /**< 0x001 - fflags: Floating-point accrued exception flags */
  CSR_FRM            = 0x002, /**< 0x002 - frm:    Floating-point dynamic rounding mode */
  CSR_FCSR           = 0x003, /**< 0x003 - fcsr:   Floating-point control/status register (frm + fflags) */

  /* machine control and status */
  CSR_MSTATUS        = 0x300, /**< 0x300 - mstatus:    Machine status register */
  CSR_MISA           = 0x301, /**< 0x301 - misa:       CPU ISA and extensions (read-only in NEORV32) */
  CSR_MIE            = 0x304, /**< 0x304 - mie:        Machine interrupt-enable register */
  CSR_MTVEC          = 0x305, /**< 0x305 - mtvec:      Machine trap-handler base address (for ALL traps) */
  CSR_MCOUNTEREN     = 0x306, /**< 0x305 - mcounteren: Machine counter enable register (controls access rights from U-mode) */

  CSR_MENVCFG        = 0x30a, /**< 0x30a - menvcfg: Machine environment configuration register */

  CSR_MSTATUSH       = 0x310, /**< 0x310 - mstatush: Machine status register - high word */

  CSR_MENVCFGH       = 0x31a, /**< 0x31a - menvcfgh: Machine environment configuration register - high word */

  CSR_MCOUNTINHIBIT  = 0x320, /**< 0x320 - mcountinhibit: Machine counter-inhibit register */

  /* hardware performance monitors - event configuration */
  CSR_MHPMEVENT3     = 0x323, /**< 0x323 - mhpmevent3:  Machine hardware performance monitor event selector 3  */
  CSR_MHPMEVENT4     = 0x324, /**< 0x324 - mhpmevent4:  Machine hardware performance monitor event selector 4  */
  CSR_MHPMEVENT5     = 0x325, /**< 0x325 - mhpmevent5:  Machine hardware performance monitor event selector 5  */
  CSR_MHPMEVENT6     = 0x326, /**< 0x326 - mhpmevent6:  Machine hardware performance monitor event selector 6  */
  CSR_MHPMEVENT7     = 0x327, /**< 0x327 - mhpmevent7:  Machine hardware performance monitor event selector 7  */
  CSR_MHPMEVENT8     = 0x328, /**< 0x328 - mhpmevent8:  Machine hardware performance monitor event selector 8  */
  CSR_MHPMEVENT9     = 0x329, /**< 0x329 - mhpmevent9:  Machine hardware performance monitor event selector 9  */
  CSR_MHPMEVENT10    = 0x32a, /**< 0x32a - mhpmevent10: Machine hardware performance monitor event selector 10 */
  CSR_MHPMEVENT11    = 0x32b, /**< 0x32b - mhpmevent11: Machine hardware performance monitor event selector 11 */
  CSR_MHPMEVENT12    = 0x32c, /**< 0x32c - mhpmevent12: Machine hardware performance monitor event selector 12 */
  CSR_MHPMEVENT13    = 0x32d, /**< 0x32d - mhpmevent13: Machine hardware performance monitor event selector 13 */
  CSR_MHPMEVENT14    = 0x32e, /**< 0x32e - mhpmevent14: Machine hardware performance monitor event selector 14 */
  CSR_MHPMEVENT15    = 0x32f, /**< 0x32f - mhpmevent15: Machine hardware performance monitor event selector 15 */
  CSR_MHPMEVENT16    = 0x330, /**< 0x330 - mhpmevent16: Machine hardware performance monitor event selector 16 */
  CSR_MHPMEVENT17    = 0x331, /**< 0x331 - mhpmevent17: Machine hardware performance monitor event selector 17 */
  CSR_MHPMEVENT18    = 0x332, /**< 0x332 - mhpmevent18: Machine hardware performance monitor event selector 18 */
  CSR_MHPMEVENT19    = 0x333, /**< 0x333 - mhpmevent19: Machine hardware performance monitor event selector 19 */
  CSR_MHPMEVENT20    = 0x334, /**< 0x334 - mhpmevent20: Machine hardware performance monitor event selector 20 */
  CSR_MHPMEVENT21    = 0x335, /**< 0x335 - mhpmevent21: Machine hardware performance monitor event selector 21 */
  CSR_MHPMEVENT22    = 0x336, /**< 0x336 - mhpmevent22: Machine hardware performance monitor event selector 22 */
  CSR_MHPMEVENT23    = 0x337, /**< 0x337 - mhpmevent23: Machine hardware performance monitor event selector 23 */
  CSR_MHPMEVENT24    = 0x338, /**< 0x338 - mhpmevent24: Machine hardware performance monitor event selector 24 */
  CSR_MHPMEVENT25    = 0x339, /**< 0x339 - mhpmevent25: Machine hardware performance monitor event selector 25 */
  CSR_MHPMEVENT26    = 0x33a, /**< 0x33a - mhpmevent26: Machine hardware performance monitor event selector 26 */
  CSR_MHPMEVENT27    = 0x33b, /**< 0x33b - mhpmevent27: Machine hardware performance monitor event selector 27 */
  CSR_MHPMEVENT28    = 0x33c, /**< 0x33c - mhpmevent28: Machine hardware performance monitor event selector 28 */
  CSR_MHPMEVENT29    = 0x33d, /**< 0x33d - mhpmevent29: Machine hardware performance monitor event selector 29 */
  CSR_MHPMEVENT30    = 0x33e, /**< 0x33e - mhpmevent30: Machine hardware performance monitor event selector 30 */
  CSR_MHPMEVENT31    = 0x33f, /**< 0x33f - mhpmevent31: Machine hardware performance monitor event selector 31 */

  /* machine trap control */
  CSR_MSCRATCH       = 0x340, /**< 0x340 - mscratch: Machine scratch register */
  CSR_MEPC           = 0x341, /**< 0x341 - mepc:     Machine exception program counter */
  CSR_MCAUSE         = 0x342, /**< 0x342 - mcause:   Machine trap cause */
  CSR_MTVAL          = 0x343, /**< 0x343 - mtval:    Machine trap value register */
  CSR_MIP            = 0x344, /**< 0x344 - mip:      Machine interrupt pending register */

  /* physical memory protection */
  CSR_PMPCFG0        = 0x3a0, /**< 0x3a0 - pmpcfg0: Physical memory protection configuration register 0 (entries 0..3) */
  CSR_PMPCFG1        = 0x3a1, /**< 0x3a1 - pmpcfg1: Physical memory protection configuration register 1 (entries 4..7) */
  CSR_PMPCFG2        = 0x3a2, /**< 0x3a2 - pmpcfg2: Physical memory protection configuration register 2 (entries 8..11) */
  CSR_PMPCFG3        = 0x3a3, /**< 0x3a3 - pmpcfg3: Physical memory protection configuration register 3 (entries 12..15) */

  CSR_PMPADDR0       = 0x3b0, /**< 0x3b0 - pmpaddr0: Physical memory protection address register 0 */
  CSR_PMPADDR1       = 0x3b1, /**< 0x3b1 - pmpaddr1: Physical memory protection address register 1 */
  CSR_PMPADDR2       = 0x3b2, /**< 0x3b2 - pmpaddr2: Physical memory protection address register 2 */
  CSR_PMPADDR3       = 0x3b3, /**< 0x3b3 - pmpaddr3: Physical memory protection address register 3 */
  CSR_PMPADDR4       = 0x3b4, /**< 0x3b4 - pmpaddr4: Physical memory protection address register 4 */
  CSR_PMPADDR5       = 0x3b5, /**< 0x3b5 - pmpaddr5: Physical memory protection address register 5 */
  CSR_PMPADDR6       = 0x3b6, /**< 0x3b6 - pmpaddr6: Physical memory protection address register 6 */
  CSR_PMPADDR7       = 0x3b7, /**< 0x3b7 - pmpaddr7: Physical memory protection address register 7 */
  CSR_PMPADDR8       = 0x3b8, /**< 0x3b8 - pmpaddr8: Physical memory protection address register 8 */
  CSR_PMPADDR9       = 0x3b9, /**< 0x3b9 - pmpaddr9: Physical memory protection address register 9 */
  CSR_PMPADDR10      = 0x3ba, /**< 0x3ba - pmpaddr10: Physical memory protection address register 10 */
  CSR_PMPADDR11      = 0x3bb, /**< 0x3bb - pmpaddr11: Physical memory protection address register 11 */
  CSR_PMPADDR12      = 0x3bc, /**< 0x3bc - pmpaddr12: Physical memory protection address register 12 */
  CSR_PMPADDR13      = 0x3bd, /**< 0x3bd - pmpaddr13: Physical memory protection address register 13 */
  CSR_PMPADDR14      = 0x3be, /**< 0x3be - pmpaddr14: Physical memory protection address register 14 */
  CSR_PMPADDR15      = 0x3bf, /**< 0x3bf - pmpaddr15: Physical memory protection address register 15 */

  /* on-chip debugger - hardware trigger module */
  CSR_TSELECT        = 0x7a0, /**< 0x7a0 - tselect:  Trigger select */
  CSR_TDATA1         = 0x7a1, /**< 0x7a1 - tdata1:   Trigger data register 0 */
  CSR_TDATA2         = 0x7a2, /**< 0x7a2 - tdata2:   Trigger data register 1 */
  CSR_TDATA3         = 0x7a3, /**< 0x7a3 - tdata3:   Trigger data register 2 */
  CSR_TINFO          = 0x7a4, /**< 0x7a4 - tinfo:    Trigger info */
  CSR_TCONTROL       = 0x7a5, /**< 0x7a5 - tcontrol: Trigger control */
  CSR_MCONTEXT       = 0x7a8, /**< 0x7a8 - mcontext: Machine context register */
  CSR_SCONTEXT       = 0x7aa, /**< 0x7aa - scontext: Supervisor context register */

  /* CPU debug mode CSRs - not accessible by software running outside of debug mode */
  CSR_DCSR           = 0x7b0, /**< 0x7b0 - dcsr:      Debug status and control register */
  CSR_DPC            = 0x7b1, /**< 0x7b1 - dpc:       Debug program counter */
  CSR_DSCRATCH0      = 0x7b2, /**< 0x7b2 - dscratch0: Debug scratch register */

  /* machine counters and timers */
  CSR_MCYCLE         = 0xb00, /**< 0xb00 - mcycle:        Machine cycle counter low word */
  //
  CSR_MINSTRET       = 0xb02, /**< 0xb02 - minstret:      Machine instructions-retired counter low word */
  CSR_MHPMCOUNTER3   = 0xb03, /**< 0xb03 - mhpmcounter3:  Machine hardware performance monitor 3  counter low word */
  CSR_MHPMCOUNTER4   = 0xb04, /**< 0xb04 - mhpmcounter4:  Machine hardware performance monitor 4  counter low word */
  CSR_MHPMCOUNTER5   = 0xb05, /**< 0xb05 - mhpmcounter5:  Machine hardware performance monitor 5  counter low word */
  CSR_MHPMCOUNTER6   = 0xb06, /**< 0xb06 - mhpmcounter6:  Machine hardware performance monitor 6  counter low word */
  CSR_MHPMCOUNTER7   = 0xb07, /**< 0xb07 - mhpmcounter7:  Machine hardware performance monitor 7  counter low word */
  CSR_MHPMCOUNTER8   = 0xb08, /**< 0xb08 - mhpmcounter8:  Machine hardware performance monitor 8  counter low word */
  CSR_MHPMCOUNTER9   = 0xb09, /**< 0xb09 - mhpmcounter9:  Machine hardware performance monitor 9  counter low word */
  CSR_MHPMCOUNTER10  = 0xb0a, /**< 0xb0a - mhpmcounter10: Machine hardware performance monitor 10 counter low word */
  CSR_MHPMCOUNTER11  = 0xb0b, /**< 0xb0b - mhpmcounter11: Machine hardware performance monitor 11 counter low word */
  CSR_MHPMCOUNTER12  = 0xb0c, /**< 0xb0c - mhpmcounter12: Machine hardware performance monitor 12 counter low word */
  CSR_MHPMCOUNTER13  = 0xb0d, /**< 0xb0d - mhpmcounter13: Machine hardware performance monitor 13 counter low word */
  CSR_MHPMCOUNTER14  = 0xb0e, /**< 0xb0e - mhpmcounter14: Machine hardware performance monitor 14 counter low word */
  CSR_MHPMCOUNTER15  = 0xb0f, /**< 0xb0f - mhpmcounter15: Machine hardware performance monitor 15 counter low word */
  CSR_MHPMCOUNTER16  = 0xb10, /**< 0xb10 - mhpmcounter16: Machine hardware performance monitor 16 counter low word */
  CSR_MHPMCOUNTER17  = 0xb11, /**< 0xb11 - mhpmcounter17: Machine hardware performance monitor 17 counter low word */
  CSR_MHPMCOUNTER18  = 0xb12, /**< 0xb12 - mhpmcounter18: Machine hardware performance monitor 18 counter low word */
  CSR_MHPMCOUNTER19  = 0xb13, /**< 0xb13 - mhpmcounter19: Machine hardware performance monitor 19 counter low word */
  CSR_MHPMCOUNTER20  = 0xb14, /**< 0xb14 - mhpmcounter20: Machine hardware performance monitor 20 counter low word */
  CSR_MHPMCOUNTER21  = 0xb15, /**< 0xb15 - mhpmcounter21: Machine hardware performance monitor 21 counter low word */
  CSR_MHPMCOUNTER22  = 0xb16, /**< 0xb16 - mhpmcounter22: Machine hardware performance monitor 22 counter low word */
  CSR_MHPMCOUNTER23  = 0xb17, /**< 0xb17 - mhpmcounter23: Machine hardware performance monitor 23 counter low word */
  CSR_MHPMCOUNTER24  = 0xb18, /**< 0xb18 - mhpmcounter24: Machine hardware performance monitor 24 counter low word */
  CSR_MHPMCOUNTER25  = 0xb19, /**< 0xb19 - mhpmcounter25: Machine hardware performance monitor 25 counter low word */
  CSR_MHPMCOUNTER26  = 0xb1a, /**< 0xb1a - mhpmcounter26: Machine hardware performance monitor 26 counter low word */
  CSR_MHPMCOUNTER27  = 0xb1b, /**< 0xb1b - mhpmcounter27: Machine hardware performance monitor 27 counter low word */
  CSR_MHPMCOUNTER28  = 0xb1c, /**< 0xb1c - mhpmcounter28: Machine hardware performance monitor 28 counter low word */
  CSR_MHPMCOUNTER29  = 0xb1d, /**< 0xb1d - mhpmcounter29: Machine hardware performance monitor 29 counter low word */
  CSR_MHPMCOUNTER30  = 0xb1e, /**< 0xb1e - mhpmcounter30: Machine hardware performance monitor 30 counter low word */
  CSR_MHPMCOUNTER31  = 0xb1f, /**< 0xb1f - mhpmcounter31: Machine hardware performance monitor 31 counter low word */

  CSR_MCYCLEH        = 0xb80, /**< 0xb80 - mcycleh:        Machine cycle counter high word */
  //
  CSR_MINSTRETH      = 0xb82, /**< 0xb82 - minstreth:      Machine instructions-retired counter high word */
  CSR_MHPMCOUNTER3H  = 0xb83, /**< 0xb83 - mhpmcounter3 :  Machine hardware performance monitor 3  counter high word */
  CSR_MHPMCOUNTER4H  = 0xb84, /**< 0xb84 - mhpmcounter4h:  Machine hardware performance monitor 4  counter high word */
  CSR_MHPMCOUNTER5H  = 0xb85, /**< 0xb85 - mhpmcounter5h:  Machine hardware performance monitor 5  counter high word */
  CSR_MHPMCOUNTER6H  = 0xb86, /**< 0xb86 - mhpmcounter6h:  Machine hardware performance monitor 6  counter high word */
  CSR_MHPMCOUNTER7H  = 0xb87, /**< 0xb87 - mhpmcounter7h:  Machine hardware performance monitor 7  counter high word */
  CSR_MHPMCOUNTER8H  = 0xb88, /**< 0xb88 - mhpmcounter8h:  Machine hardware performance monitor 8  counter high word */
  CSR_MHPMCOUNTER9H  = 0xb89, /**< 0xb89 - mhpmcounter9h:  Machine hardware performance monitor 9  counter high word */
  CSR_MHPMCOUNTER10H = 0xb8a, /**< 0xb8a - mhpmcounter10h: Machine hardware performance monitor 10 counter high word */
  CSR_MHPMCOUNTER11H = 0xb8b, /**< 0xb8b - mhpmcounter11h: Machine hardware performance monitor 11 counter high word */
  CSR_MHPMCOUNTER12H = 0xb8c, /**< 0xb8c - mhpmcounter12h: Machine hardware performance monitor 12 counter high word */
  CSR_MHPMCOUNTER13H = 0xb8d, /**< 0xb8d - mhpmcounter13h: Machine hardware performance monitor 13 counter high word */
  CSR_MHPMCOUNTER14H = 0xb8e, /**< 0xb8e - mhpmcounter14h: Machine hardware performance monitor 14 counter high word */
  CSR_MHPMCOUNTER15H = 0xb8f, /**< 0xb8f - mhpmcounter15h: Machine hardware performance monitor 15 counter high word */
  CSR_MHPMCOUNTER16H = 0xb90, /**< 0xb90 - mhpmcounter16h: Machine hardware performance monitor 16 counter high word */
  CSR_MHPMCOUNTER17H = 0xb91, /**< 0xb91 - mhpmcounter17h: Machine hardware performance monitor 17 counter high word */
  CSR_MHPMCOUNTER18H = 0xb92, /**< 0xb92 - mhpmcounter18h: Machine hardware performance monitor 18 counter high word */
  CSR_MHPMCOUNTER19H = 0xb93, /**< 0xb93 - mhpmcounter19h: Machine hardware performance monitor 19 counter high word */
  CSR_MHPMCOUNTER20H = 0xb94, /**< 0xb94 - mhpmcounter20h: Machine hardware performance monitor 20 counter high word */
  CSR_MHPMCOUNTER21H = 0xb95, /**< 0xb95 - mhpmcounter21h: Machine hardware performance monitor 21 counter high word */
  CSR_MHPMCOUNTER22H = 0xb96, /**< 0xb96 - mhpmcounter22h: Machine hardware performance monitor 22 counter high word */
  CSR_MHPMCOUNTER23H = 0xb97, /**< 0xb97 - mhpmcounter23h: Machine hardware performance monitor 23 counter high word */
  CSR_MHPMCOUNTER24H = 0xb98, /**< 0xb98 - mhpmcounter24h: Machine hardware performance monitor 24 counter high word */
  CSR_MHPMCOUNTER25H = 0xb99, /**< 0xb99 - mhpmcounter25h: Machine hardware performance monitor 25 counter high word */
  CSR_MHPMCOUNTER26H = 0xb9a, /**< 0xb9a - mhpmcounter26h: Machine hardware performance monitor 26 counter high word */
  CSR_MHPMCOUNTER27H = 0xb9b, /**< 0xb9b - mhpmcounter27h: Machine hardware performance monitor 27 counter high word */
  CSR_MHPMCOUNTER28H = 0xb9c, /**< 0xb9c - mhpmcounter28h: Machine hardware performance monitor 28 counter high word */
  CSR_MHPMCOUNTER29H = 0xb9d, /**< 0xb9d - mhpmcounter29h: Machine hardware performance monitor 29 counter high word */
  CSR_MHPMCOUNTER30H = 0xb9e, /**< 0xb9e - mhpmcounter30h: Machine hardware performance monitor 30 counter high word */
  CSR_MHPMCOUNTER31H = 0xb9f, /**< 0xb9f - mhpmcounter31h: Machine hardware performance monitor 31 counter high word */

  /* user counters and timers */
  CSR_CYCLE          = 0xc00, /**< 0xc00 - cycle:        Cycle counter low word (from MCYCLE) */
  //
  CSR_INSTRET        = 0xc02, /**< 0xc02 - instret:      Instructions-retired counter low word (from MINSTRET) */
  CSR_HPMCOUNTER3    = 0xc03, /**< 0xc03 - hpmcounter3:  User hardware performance monitor 3  counter low word */
  CSR_HPMCOUNTER4    = 0xc04, /**< 0xc04 - hpmcounter4:  User hardware performance monitor 4  counter low word */
  CSR_HPMCOUNTER5    = 0xc05, /**< 0xc05 - hpmcounter5:  User hardware performance monitor 5  counter low word */
  CSR_HPMCOUNTER6    = 0xc06, /**< 0xc06 - hpmcounter6:  User hardware performance monitor 6  counter low word */
  CSR_HPMCOUNTER7    = 0xc07, /**< 0xc07 - hpmcounter7:  User hardware performance monitor 7  counter low word */
  CSR_HPMCOUNTER8    = 0xc08, /**< 0xc08 - hpmcounter8:  User hardware performance monitor 8  counter low word */
  CSR_HPMCOUNTER9    = 0xc09, /**< 0xc09 - hpmcounter9:  User hardware performance monitor 9  counter low word */
  CSR_HPMCOUNTER10   = 0xc0a, /**< 0xc0a - hpmcounter10: User hardware performance monitor 10 counter low word */
  CSR_HPMCOUNTER11   = 0xc0b, /**< 0xc0b - hpmcounter11: User hardware performance monitor 11 counter low word */
  CSR_HPMCOUNTER12   = 0xc0c, /**< 0xc0c - hpmcounter12: User hardware performance monitor 12 counter low word */
  CSR_HPMCOUNTER13   = 0xc0d, /**< 0xc0d - hpmcounter13: User hardware performance monitor 13 counter low word */
  CSR_HPMCOUNTER14   = 0xc0e, /**< 0xc0e - hpmcounter14: User hardware performance monitor 14 counter low word */
  CSR_HPMCOUNTER15   = 0xc0f, /**< 0xc0f - hpmcounter15: User hardware performance monitor 15 counter low word */
  CSR_HPMCOUNTER16   = 0xc10, /**< 0xc10 - hpmcounter16: User hardware performance monitor 16 counter low word */
  CSR_HPMCOUNTER17   = 0xc11, /**< 0xc11 - hpmcounter17: User hardware performance monitor 17 counter low word */
  CSR_HPMCOUNTER18   = 0xc12, /**< 0xc12 - hpmcounter18: User hardware performance monitor 18 counter low word */
  CSR_HPMCOUNTER19   = 0xc13, /**< 0xc13 - hpmcounter19: User hardware performance monitor 19 counter low word */
  CSR_HPMCOUNTER20   = 0xc14, /**< 0xc14 - hpmcounter20: User hardware performance monitor 20 counter low word */
  CSR_HPMCOUNTER21   = 0xc15, /**< 0xc15 - hpmcounter21: User hardware performance monitor 21 counter low word */
  CSR_HPMCOUNTER22   = 0xc16, /**< 0xc16 - hpmcounter22: User hardware performance monitor 22 counter low word */
  CSR_HPMCOUNTER23   = 0xc17, /**< 0xc17 - hpmcounter23: User hardware performance monitor 23 counter low word */
  CSR_HPMCOUNTER24   = 0xc18, /**< 0xc18 - hpmcounter24: User hardware performance monitor 24 counter low word */
  CSR_HPMCOUNTER25   = 0xc19, /**< 0xc19 - hpmcounter25: User hardware performance monitor 25 counter low word */
  CSR_HPMCOUNTER26   = 0xc1a, /**< 0xc1a - hpmcounter26: User hardware performance monitor 26 counter low word */
  CSR_HPMCOUNTER27   = 0xc1b, /**< 0xc1b - hpmcounter27: User hardware performance monitor 27 counter low word */
  CSR_HPMCOUNTER28   = 0xc1c, /**< 0xc1c - hpmcounter28: User hardware performance monitor 28 counter low word */
  CSR_HPMCOUNTER29   = 0xc1d, /**< 0xc1d - hpmcounter29: User hardware performance monitor 29 counter low word */
  CSR_HPMCOUNTER30   = 0xc1e, /**< 0xc1e - hpmcounter30: User hardware performance monitor 30 counter low word */
  CSR_HPMCOUNTER31   = 0xc1f, /**< 0xc1f - hpmcounter31: User hardware performance monitor 31 counter low word */

  CSR_CYCLEH         = 0xc80, /**< 0xc80 - cycleh:        Cycle counter high word (from MCYCLEH) */
  //
  CSR_INSTRETH       = 0xc82, /**< 0xc82 - instreth:      Instructions-retired counter high word (from MINSTRETH) */
  CSR_HPMCOUNTER3H   = 0xc83, /**< 0xc83 - hpmcounter3h:  User hardware performance monitor 3  counter high word */
  CSR_HPMCOUNTER4H   = 0xc84, /**< 0xc84 - hpmcounter4h:  User hardware performance monitor 4  counter high word */
  CSR_HPMCOUNTER5H   = 0xc85, /**< 0xc85 - hpmcounter5h:  User hardware performance monitor 5  counter high word */
  CSR_HPMCOUNTER6H   = 0xc86, /**< 0xc86 - hpmcounter6h:  User hardware performance monitor 6  counter high word */
  CSR_HPMCOUNTER7H   = 0xc87, /**< 0xc87 - hpmcounter7h:  User hardware performance monitor 7  counter high word */
  CSR_HPMCOUNTER8H   = 0xc88, /**< 0xc88 - hpmcounter8h:  User hardware performance monitor 8  counter high word */
  CSR_HPMCOUNTER9H   = 0xc89, /**< 0xc89 - hpmcounter9h:  User hardware performance monitor 9  counter high word */
  CSR_HPMCOUNTER10H  = 0xc8a, /**< 0xc8a - hpmcounter10h: User hardware performance monitor 10 counter high word */
  CSR_HPMCOUNTER11H  = 0xc8b, /**< 0xc8b - hpmcounter11h: User hardware performance monitor 11 counter high word */
  CSR_HPMCOUNTER12H  = 0xc8c, /**< 0xc8c - hpmcounter12h: User hardware performance monitor 12 counter high word */
  CSR_HPMCOUNTER13H  = 0xc8d, /**< 0xc8d - hpmcounter13h: User hardware performance monitor 13 counter high word */
  CSR_HPMCOUNTER14H  = 0xc8e, /**< 0xc8e - hpmcounter14h: User hardware performance monitor 14 counter high word */
  CSR_HPMCOUNTER15H  = 0xc8f, /**< 0xc8f - hpmcounter15h: User hardware performance monitor 15 counter high word */
  CSR_HPMCOUNTER16H  = 0xc90, /**< 0xc90 - hpmcounter16h: User hardware performance monitor 16 counter high word */
  CSR_HPMCOUNTER17H  = 0xc91, /**< 0xc91 - hpmcounter17h: User hardware performance monitor 17 counter high word */
  CSR_HPMCOUNTER18H  = 0xc92, /**< 0xc92 - hpmcounter18h: User hardware performance monitor 18 counter high word */
  CSR_HPMCOUNTER19H  = 0xc93, /**< 0xc93 - hpmcounter19h: User hardware performance monitor 19 counter high word */
  CSR_HPMCOUNTER20H  = 0xc94, /**< 0xc94 - hpmcounter20h: User hardware performance monitor 20 counter high word */
  CSR_HPMCOUNTER21H  = 0xc95, /**< 0xc95 - hpmcounter21h: User hardware performance monitor 21 counter high word */
  CSR_HPMCOUNTER22H  = 0xc96, /**< 0xc96 - hpmcounter22h: User hardware performance monitor 22 counter high word */
  CSR_HPMCOUNTER23H  = 0xc97, /**< 0xc97 - hpmcounter23h: User hardware performance monitor 23 counter high word */
  CSR_HPMCOUNTER24H  = 0xc98, /**< 0xc98 - hpmcounter24h: User hardware performance monitor 24 counter high word */
  CSR_HPMCOUNTER25H  = 0xc99, /**< 0xc99 - hpmcounter25h: User hardware performance monitor 25 counter high word */
  CSR_HPMCOUNTER26H  = 0xc9a, /**< 0xc9a - hpmcounter26h: User hardware performance monitor 26 counter high word */
  CSR_HPMCOUNTER27H  = 0xc9b, /**< 0xc9b - hpmcounter27h: User hardware performance monitor 27 counter high word */
  CSR_HPMCOUNTER28H  = 0xc9c, /**< 0xc9c - hpmcounter28h: User hardware performance monitor 28 counter high word */
  CSR_HPMCOUNTER29H  = 0xc9d, /**< 0xc9d - hpmcounter29h: User hardware performance monitor 29 counter high word */
  CSR_HPMCOUNTER30H  = 0xc9e, /**< 0xc9e - hpmcounter30h: User hardware performance monitor 30 counter high word */
  CSR_HPMCOUNTER31H  = 0xc9f, /**< 0xc9f - hpmcounter31h: User hardware performance monitor 31 counter high word */

  /* machine information registers */
  CSR_MVENDORID      = 0xf11, /**< 0xf11 - mvendorid:  Vendor ID */
  CSR_MARCHID        = 0xf12, /**< 0xf12 - marchid:    Architecture ID */
  CSR_MIMPID         = 0xf13, /**< 0xf13 - mimpid:     Implementation ID/version */
  CSR_MHARTID        = 0xf14, /**< 0xf14 - mhartid:    Hardware thread ID (always 0) */
  CSR_MCONFIGPTR     = 0xf15, /**< 0xf15 - mconfigptr: Machine configuration pointer register */

  CSR_MXISA          = 0xfc0  /**< 0xfc0 - mxisa: NEORV32-specific machine "extended CPU ISA and extensions" */
};


/**********************************************************************//**
 * CPU <b>mip</b> CSR (r/c): Machine interrupt pending
 **************************************************************************/
enum NEORV32_CSR_MIP_enum {
  CSR_MIP_MSIP    =  3, /**< CPU mip CSR  (3): MSIP - Machine software interrupt pending (r/c) */
  CSR_MIP_MTIP    =  7, /**< CPU mip CSR  (7): MTIP - Machine timer interrupt pending (r/c) */
  CSR_MIP_MEIP    = 11, /**< CPU mip CSR (11): MEIP - Machine external interrupt pending (r/c) */

  /* NEORV32-specific extension: Fast Interrupt Requests (FIRQ) */
  CSR_MIP_FIRQ0P  = 16, /**< CPU mip CSR (16): FIRQ0P - Fast interrupt channel 0 pending (r/c) */
  CSR_MIP_FIRQ1P  = 17, /**< CPU mip CSR (17): FIRQ1P - Fast interrupt channel 1 pending (r/c) */
  CSR_MIP_FIRQ2P  = 18, /**< CPU mip CSR (18): FIRQ2P - Fast interrupt channel 2 pending (r/c) */
  CSR_MIP_FIRQ3P  = 19, /**< CPU mip CSR (19): FIRQ3P - Fast interrupt channel 3 pending (r/c) */
  CSR_MIP_FIRQ4P  = 20, /**< CPU mip CSR (20): FIRQ4P - Fast interrupt channel 4 pending (r/c) */
  CSR_MIP_FIRQ5P  = 21, /**< CPU mip CSR (21): FIRQ5P - Fast interrupt channel 5 pending (r/c) */
  CSR_MIP_FIRQ6P  = 22, /**< CPU mip CSR (22): FIRQ6P - Fast interrupt channel 6 pending (r/c) */
  CSR_MIP_FIRQ7P  = 23, /**< CPU mip CSR (23): FIRQ7P - Fast interrupt channel 7 pending (r/c) */
  CSR_MIP_FIRQ8P  = 24, /**< CPU mip CSR (24): FIRQ8P - Fast interrupt channel 8 pending (r/c) */
  CSR_MIP_FIRQ9P  = 25, /**< CPU mip CSR (25): FIRQ9P - Fast interrupt channel 9 pending (r/c) */
  CSR_MIP_FIRQ10P = 26, /**< CPU mip CSR (26): FIRQ10P - Fast interrupt channel 10 pending (r/c) */
  CSR_MIP_FIRQ11P = 27, /**< CPU mip CSR (27): FIRQ11P - Fast interrupt channel 11 pending (r/c) */
  CSR_MIP_FIRQ12P = 28, /**< CPU mip CSR (28): FIRQ12P - Fast interrupt channel 12 pending (r/c) */
  CSR_MIP_FIRQ13P = 29, /**< CPU mip CSR (29): FIRQ13P - Fast interrupt channel 13 pending (r/c) */
  CSR_MIP_FIRQ14P = 30, /**< CPU mip CSR (30): FIRQ14P - Fast interrupt channel 14 pending (r/c) */
  CSR_MIP_FIRQ15P = 31  /**< CPU mip CSR (31): FIRQ15P - Fast interrupt channel 15 pending (r/c) */
};



/**********************************************************************//**
 * Print help menu.
 **************************************************************************/
void print_help(void) {

  printf("Available CMDs:\n"
             " h: Help\n"
             " u: Upload\n"
             " q: Quit\n");

}


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

  // get image from UART?
  if (src == EXE_STREAM_UART) {
    printf("Awaiting I-O-Guard_task_status_table.bin... ");
  }

  // check if valid image
  uint32_t signature = get_exe_word(src);
  if (signature != EXE_SIGNATURE) { // signature
    printf("ERROR EXE_SIGNATURE!!\n");
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

  printf("OK");

}


/**********************************************************************//**
 * Get cycle counter from cycle[h].
 *
 * @return Current cycle counter (64 bit).
 **************************************************************************/
uint64_t neorv32_cpu_get_cycle(void) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } cycles;

  uint32_t tmp1, tmp2, tmp3;
  while(1) {
    tmp1 = neorv32_cpu_csr_read(CSR_CYCLEH);
    tmp2 = neorv32_cpu_csr_read(CSR_CYCLE);
    tmp3 = neorv32_cpu_csr_read(CSR_CYCLEH);
    if (tmp1 == tmp3) {
      break;
    }
  }

  cycles.uint32[0] = tmp2;
  cycles.uint32[1] = tmp3;

  return cycles.uint64;
}

/**********************************************************************//**
 * @name IO Device: Machine System Timer (MTIME)
 **************************************************************************/
/**@{*/
/** MTIME module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t TIME_LO;    /**< offset 0:  time register low word */
  uint32_t TIME_HI;    /**< offset 4:  time register high word */
  uint32_t TIMECMP_LO; /**< offset 8:  compare register low word */
  uint32_t TIMECMP_HI; /**< offset 12: compare register high word */
} neorv32_mtime_t;

/** MTIME module hardware access (#neorv32_mtime_t) */
#define NEORV32_MTIME ((neorv32_mtime_t*) (NEORV32_MTIME_BASE))
/**@}*/

/**********************************************************************//**
 * Get current system time.
 *
 * @note The MTIME timer increments with the primary processor clock.
 *
 * @return Current system time (uint64_t)
 **************************************************************************/
uint64_t neorv32_mtime_get_time(void) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } cycles;

  uint32_t tmp1, tmp2, tmp3;
  while(1) {
    tmp1 = NEORV32_MTIME->TIME_HI;
    tmp2 = NEORV32_MTIME->TIME_LO;
    tmp3 = NEORV32_MTIME->TIME_HI;
    if (tmp1 == tmp3) {
      break;
    }
  }

  cycles.uint32[0] = tmp2;
  cycles.uint32[1] = tmp3;

  return cycles.uint64;
}

/**********************************************************************//**
 * Set current system time.
 *
 * @note The MTIME timer increments with the primary processor clock.
 *
 * @param[in] time New system time (uint64_t)
 **************************************************************************/
void neorv32_mtime_set_time(uint64_t time) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } cycles;

  cycles.uint64 = time;

  NEORV32_MTIME->TIME_LO = 0;
  NEORV32_MTIME->TIME_HI = cycles.uint32[1];
  NEORV32_MTIME->TIME_LO = cycles.uint32[0];

  __asm__ volatile("nop"); // delay due to write buffer
}

/**********************************************************************//**
 * Due to the porting issue.
 *
 * I created my own cycle counter in vhdl.
 **************************************************************************/
unsigned int cpu_get_mycycle(unsigned int latencyStart, unsigned int latencyEnd){

  uint32_t max = -1;

  uint32_t latency = 0;

  if(latencyEnd < latencyStart){
    latency = max - latencyStart + latencyEnd;
  }
  else{
    latency = latencyEnd - latencyStart;
  }

  return latency;
}



/**********************************************************************//**
 * @file neorv32_gptmr.h
 * @brief General purpose timer (GPTMR) HW driver header file.
 *
 * @note These functions should only be used if the GPTMR unit was synthesized (IO_GPTMR_EN = true).
 **************************************************************************/

#define NEORV32_GPTMR_BASE     (0xFFFFFF60U) /**< General Purpose Timer (GPTMR) */

/**********************************************************************//**
 * @name IO Device: General Purpose Timer (GPTMR)
 **************************************************************************/
/**@{*/
/** GPTMR module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;           /**< offset  0: control register (#NEORV32_GPTMR_CTRL_enum) */
  uint32_t THRES;          /**< offset  4: threshold register */
  uint32_t COUNT;          /**< offset  8: counter register */
  const uint32_t reserved; /**< offset 12: reserved */
} neorv32_gptmr_t;

/** GPTMR module hardware access (#neorv32_gptmr_t) */
#define NEORV32_GPTMR ((neorv32_gptmr_t*) (NEORV32_GPTMR_BASE))

/** GPTMR control/data register bits */
enum NEORV32_GPTMR_CTRL_enum {
  GPTMR_CTRL_EN    = 0, /**< GPTIMR control register(0) (r/w): Timer unit enable */
  GPTMR_CTRL_PRSC0 = 1, /**< GPTIMR control register(1) (r/w): Clock prescaler select bit 0 */
  GPTMR_CTRL_PRSC1 = 2, /**< GPTIMR control register(2) (r/w): Clock prescaler select bit 1 */
  GPTMR_CTRL_PRSC2 = 3, /**< GPTIMR control register(3) (r/w): Clock prescaler select bit 2 */
  GPTMR_CTRL_MODE  = 4  /**< GPTIMR control register(4) (r/w): Timer mode: 0=single-shot mode, 1=continuous mode */
};
/**@}*/

/**********************************************************************//**
 * Enable and configure general purpose timer.
 *
 * @param[in] prsc Clock prescaler select (0..7). See #NEORV32_CLOCK_PRSC_enum.
 * @param[in] mode 0=single-shot mode, 1=continuous mode
 * @param[in] threshold Threshold value to trigger interrupt.
 **************************************************************************/
void neorv32_gptmr_setup(int prsc, int mode, uint32_t threshold) {

  NEORV32_GPTMR->CTRL  = 0; // reset
  NEORV32_GPTMR->THRES = threshold;
  NEORV32_GPTMR->COUNT = 0; // reset counter

  uint32_t tmp = 0;
  tmp |= (uint32_t)(1    & 0x01) << GPTMR_CTRL_EN;
  tmp |= (uint32_t)(prsc & 0x07) << GPTMR_CTRL_PRSC0;
  tmp |= (uint32_t)(mode & 0x01) << GPTMR_CTRL_MODE;
  NEORV32_GPTMR->CTRL = tmp;
}


/**********************************************************************//**
 * Disable general purpose timer.
 **************************************************************************/
void neorv32_gptmr_disable(void) {

  NEORV32_GPTMR->CTRL &= ~((uint32_t)(1 << GPTMR_CTRL_EN));
}


/**********************************************************************//**
 * Enable general purpose timer.
 **************************************************************************/
void neorv32_gptmr_enable(void) {

  NEORV32_GPTMR->CTRL |= ((uint32_t)(1 << GPTMR_CTRL_EN));
}


/**********************************************************************//**
 * Reset general purpose timer's counter register.
 **************************************************************************/
void neorv32_gptmr_restart(void) {

  NEORV32_GPTMR->COUNT = 0;
}


/**********************************************************************//**
 * Get the current time. (stevez)
 **************************************************************************/
int neorv32_gptmr_curtime(void){

  // neorv32_uart0_printf("NEORV32_CURTIME: \n", NEORV32_GPTMR->COUNT); // (NEEDED)
  return NEORV32_GPTMR->COUNT;

}