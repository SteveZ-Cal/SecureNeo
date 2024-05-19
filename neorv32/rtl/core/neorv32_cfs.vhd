-- #################################################################################################
-- # << NEORV32 - Custom Functions Subsystem (CFS) >>                                              #
-- # ********************************************************************************************* #
-- # Intended for tightly-coupled, application-specific custom co-processors. This module provides #
-- # 64x 32-bit memory-mapped interface registers, one interrupt request signal and custom IO      #
-- # conduits for processor-external or chip-external interface.                                   #
-- #                                                                                               #
-- # NOTE: This is just an example/illustration template. Modify/replace this file to implement    #
-- #       your own custom design logic.                                                           #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cfs is
  generic (
    CFS_CONFIG   : std_ulogic_vector(31 downto 0); -- custom CFS configuration generic
    CFS_IN_SIZE  : natural; -- size of CFS input conduit in bits
    CFS_OUT_SIZE : natural  -- size of CFS output conduit in bits
  );
  port (
    clk_i       : in  std_ulogic; -- global clock line
    rstn_i      : in  std_ulogic; -- global reset line, low-active, use as async
    bus_req_i   : in  bus_req_t;  -- bus request
    bus_rsp_o   : out bus_rsp_t;  -- bus response
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(07 downto 0); -- "clock" inputs
    irq_o       : out std_ulogic; -- interrupt request
    cfs_in_i    : in  std_ulogic_vector(CFS_IN_SIZE-1 downto 0); -- custom inputs
    cfs_out_o   : out std_ulogic_vector(CFS_OUT_SIZE-1 downto 0); -- custom outputs
	 -- task_block_o: out std_ulogic; -- (stevez) 9/26/2023 mainly used for testing whether the current task should be blocked 
    cpu_i_req   : in  bus_req_t   -- (stevez) 10/28/2023 added cpu_fetch (connect from neorv32_cpu -> neorv23_top -> neorv32_cfs) 
  );
end neorv32_cfs;

architecture neorv32_cfs_rtl of neorv32_cfs is

	-- (stevez) 9/29/2023 define the number of register in the cfs -- 
	constant REG_NUM: natural := 64; -- register number 64
	
	constant MAX_NUM_TASkS: natural := 10; -- maximum number of task

  -- IO space: module base address --
  -- WARNING: Do not modify the CFS base address or the CFS' occupied address
  -- space as this might cause access collisions with other processor modules.
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(cfs_size_c); -- low address boundary bit

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- default CFS interface registers --
  -- type cfs_regs_t is array (0 to 3) of std_ulogic_vector(31 downto 0); -- just implement 4 registers for this example
  -- (stevez) 9/29/2023 open up all its availabel register
  type cfs_regs_t is array(0 to REG_NUM-1) of std_ulogic_vector(31 downto 0); -- implement all 64 registers
  signal cfs_reg_wr : cfs_regs_t; -- interface registers for WRITE accesses
  signal cfs_reg_rd : cfs_regs_t; -- interface registers for READ accesses

  -- (stevez) 11/27/2023 CFS_REG helper parameters
  constant REG_PER_TASK : integer := 5;
  constant START_REG_FOR_TASK : integer := 10; -- (stevez) 1/9/2024 changed to since need up to 10 cfs_reg for configurations
  constant END_REG_FOR_TASK : integer := 55;
  constant ACCESS_OFFSET : integer := 1;
  constant TIME_OFFSET : integer := 3;
  
  -- (stevez) 12/1/2023 CFS REG buffer to store task_status_table
  type cfs_regs_t2 is array (0 to END_REG_FOR_TASK - START_REG_FOR_TASK) of std_ulogic_vector(31 downto 0);
  signal cfs_reg_buffer : cfs_regs_t2; -- buffer CFS REG

  -- (stevez) 9/22/2023 CFS Interrupt One Pulse Wave Capture -- 
  signal irq_cfs : std_ulogic; 
  signal irq_cfs_dly : std_ulogic; -- one pulse delay of actual irq_cfs
  
  -- (stevez) 9/27/2023 for testing whether the current task should be blocked
  signal thread_block	: std_ulogic := '0';
  
  -- (stevez) 9/29/2023 defining constant thread ID for testing purpose
--  signal thread0_id_reg : std_ulogic_vector(31 downto 0) := std_ulogic_vector(to_unsigned(2147483720, 32));
--  signal thread1_id_reg : std_ulogic_vector(31 downto 0) := std_ulogic_vector(to_unsigned(2147484128, 32));
--  signal thread2_id_reg : std_ulogic_vector(31 downto 0) := std_ulogic_vector(to_unsigned(2147483992, 32));
--  signal thread3_id_reg : std_ulogic_vector(31 downto 0) := std_ulogic_vector(to_unsigned(2147483856, 32));
  -- constant thread1_id : natural := 2147483856;
  -- constant thread2_id : natural := 2147483720;

  -- (stevez) 10/28/2023 added cpu_fetch (connect from neorv32_cpu -> neorv23_top -> neorv32_cfs)
  signal fetch_pc    : std_ulogic_vector(XLEN-1 downto 0); -- pc for instruction fetch from neorv32_cpu
  
  -- (stevez) 10/30/2023 added PC_boundary for checking whether that task is within that range
  signal LOWER_BOUND_T1	:  std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"0000f000"); 	-- lower bound of task1
  signal UPPER_BOUND_T1 : 	std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"0000f3ff");	-- upper bound of task1
  signal LOWER_BOUND_T2	:  std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"0000f400");	-- lower bound of task2
  signal UPPER_BOUND_T2 : 	std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"0000f7ff");	-- upper bound of task2
  signal LOWER_BOUND_T3	:  std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"0000f800");	-- lower bound of task3
  signal UPPER_BOUND_T3 : 	std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"0000fBff");	-- upper bound of task3
   -- (stevez) 11/27/2023 I-O-Guard (at most 10 tasks)
	signal LOWER_BOUND_T4	:  std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"0000fC00");	-- lower bound of task4
  signal UPPER_BOUND_T4 : 	std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"0000ffff");	-- upper bound of task4
    signal LOWER_BOUND_T5	:  std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"00010000");	-- lower bound of task5
  signal UPPER_BOUND_T5 : 	std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"000103ff");	-- upper bound of task5
    signal LOWER_BOUND_T6	:  std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"00010400");	-- lower bound of task6
  signal UPPER_BOUND_T6 : 	std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"000107ff");	-- upper bound of task6
    signal LOWER_BOUND_T7	:  std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"00010800");	-- lower bound of task7
  signal UPPER_BOUND_T7 : 	std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"00010Bff");	-- upper bound of task7
    signal LOWER_BOUND_T8	:  std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"00010C00");	-- lower bound of task8
  signal UPPER_BOUND_T8 : 	std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"00010fff");	-- upper bound of task8
    signal LOWER_BOUND_T9	:  std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"00011000");	-- lower bound of task9
  signal UPPER_BOUND_T9 : 	std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"000113ff");	-- upper bound of task9
    signal LOWER_BOUND_T10	:  std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"00011400");	-- lower bound of task10
  signal UPPER_BOUND_T10 : std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"000117ff");	-- upper bound of task10
  
  -- (stevez) 11/27/2023 simplied version for above (max 10 tasks)
	type std_ulogic_aoa_32 is array (0 to MAX_NUM_TASKS-1) of std_ulogic_vector(31 downto 0);
	signal LOWER_BOUND_T : std_ulogic_aoa_32	:= (LOWER_BOUND_T1, LOWER_BOUND_T2, LOWER_BOUND_T3, LOWER_BOUND_T4, LOWER_BOUND_T5, LOWER_BOUND_T6, LOWER_BOUND_T7, LOWER_BOUND_T8, LOWER_BOUND_T9, LOWER_BOUND_T10);
	signal UPPER_BOUND_T : std_ulogic_aoa_32	:= (UPPER_BOUND_T1, UPPER_BOUND_T2, UPPER_BOUND_T3, UPPER_BOUND_T4, UPPER_BOUND_T5, UPPER_BOUND_T6, UPPER_BOUND_T7, UPPER_BOUND_T8, UPPER_BOUND_T9, UPPER_BOUND_T10);
  
  -- (stevez) 12/2/2023 determine zephyr interrupt boundary
  signal LOWER_BOUND_FIRQ1	:   std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"00012000");	-- lower bound of FIRQ1
  signal UPPER_BOUND_FIRQ1  :   std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"000123ff");	-- upper bound of FIRQ1
  
  -- (stevez) 10/30/2023 indicate what's the current running thread
  signal thread1	: std_ulogic := '0'; -- Initial thread 1 running status
  signal thread2	: std_ulogic := '0'; -- Initial thread 2 running status
  signal thread3	: std_ulogic := '0'; -- Initial thread 3 running status
     -- (stevez) 11/27/2023 I-O-Guard (at most 10 tasks)
    signal thread4	: std_ulogic := '0'; -- Initial thread 4 running status
  signal thread5	: std_ulogic := '0'; -- Initial thread 5 running status
  signal thread6	: std_ulogic := '0'; -- Initial thread 6 running status
    signal thread7	: std_ulogic := '0'; -- Initial thread 7 running status
  signal thread8	: std_ulogic := '0'; -- Initial thread 8 running status
  signal thread9	: std_ulogic := '0'; -- Initial thread 9 running status
    signal thread10	: std_ulogic := '0'; -- Initial thread 10 running status
	 
	 -- (stevez) 11/27/2023 simplied version for above (max 10 tasks)
	 signal threads : std_ulogic_vector(0 to MAX_NUM_TASKS - 1) := (thread1, thread2, thread3, thread4, thread5, thread6, thread7, thread8, thread9, thread10);
  
  -- (stevez) 11/8/2023 indicate how many tasks in total is needed (may subjected to input change later)
--  constant NUM_THREAD: natural := 4; -- task0 (trusted thread), task1, task2, task3
  
  -- (stevez) 11/8/2023 indicate which thread now has the control
  -- differ from thread1, thread2, thread3, etc (if memory address != desired thread)
--  signal thread_in_control : std_ulogic_vector(NUM_THREAD-1 downto 0);

-- (stevez) 11/27/2023 only need 4 bits for 10 tasks
	constant NUM_BITS: natural := 4;
	signal thread_in_control: std_ulogic_vector(NUM_BITS-1 downto 0);
  
  -- (stevez) 11/18/2023 correponding of cfs_out used for I-O-Guard
  constant gpio_o_num : natural := 7; -- set manually for the number of physical output gpio pins
  
  -- Interal Testing Purpose -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- (stevez) 11/18/2023 created to testing PC
  -- asm volatile ("li t0, %[input_i]; jr t0" :  : [input_i] "i" (BOOTLOADER_BASE_ADDRESS)); // jump to beginning of boot ROM
  -- jump to address other than BOOLOADER_BASE_ADDRESS or ISPACE_BASE
  
  signal LOWER_BOUND_SECURE_TASK0	:  std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"00020000"); -- lower bound
  signal UPPER_BOUND_SECURE_TASK0 :  std_ulogic_vector(31 downto 0) := To_StdULogicVector(x"00022000");	-- upper bound

  -- (stevez) 11/20/2023 define cfs_reg_masking variable
  --signal masking : std_ulogic_vector(31 downto 0);
  constant secure_masking_c : std_ulogic_vector(31 downto 0) := (others => '1');
  constant insecure_masking_c : std_ulogic_vector(31 downto 0) := (others => '0'); 


  -- (stevez) 11/27/2023 number of tasks currently on run
  signal NUM_TASKS : integer := 0;

  signal CURRENT_TASK_NUM : integer := 0;

  -- (stevez) 1/10/2024 whether or not my_isr or task0 is on run
  -- task0 -> 1; my_isr -> 2
  signal SECURE_CFS_REGIONS : integer := 0;
  
--  -- (stevez) 11/27/2023 CFS_REG helper parameters
--  constant REG_PER_TASK : integer := 5;
--  constant START_REG_FOR_TASK : integer := 5;
--  constant END_REG_FOR_TASK : integer := 55;
--  constant ACCESS_OFFSET : integer := 1;
--  constant TIME_OFFSET : integer := 3;

  -- (stevez) 11/27/2023 CFS_REG time counter for each tasks (max 10 tasks)
  constant STARTING_TIME : std_ulogic_vector(63 downto 0) := (others => '0');
  type std_ulogic_aoa_64 is array (0 to MAX_NUM_TASKS-1) of std_ulogic_vector(63 downto 0);
  signal TIMER_FOR_TASKS : std_ulogic_aoa_64	:= (STARTING_TIME, STARTING_TIME, STARTING_TIME, STARTING_TIME, STARTING_TIME, STARTING_TIME, STARTING_TIME, STARTING_TIME, STARTING_TIME, STARTING_TIME);
  -- our unit for time in each task is millsec, we want to convert clk to millsec, 50000 clk cycle = 1 millsec
  constant CLK_PER_MILLSEC : integer := 50000;
  type std_ulogic_aoa_int is array (0 to MAX_NUM_TASKS-1) of integer;
  signal COUNTING_TO_MILLSEC : std_ulogic_aoa_int := (0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  -- (stevez) 12/20/2023 equivalent as irq_o
  signal irq_cfs_int : std_ulogic := '0';

  -- Define CFS_REG_NUM
  constant CFS_CTRL_REG : integer := 2;
  constant CFS_UPDATE_REG : integer := 3;

  ------------------------------
  --- HybridAttestation Section
  ------------------------------

  constant WORD_SIZE : natural := 32;
  -- constant REG_NUM: natural := 63; -- register number 64

  constant data_out_reg_start : natural := 48;
  constant data_out_reg_end : natural := 63;

  -- Define CFS_REG_NUM for HybridAttestation
  constant DATA_READY_REG : integer := 10; -- Data Ready Flag for HybridAttestation
  constant NUM_BLOCKS_REG : integer := 11; -- Number of Blocks for HybridAttestation
  constant FINISH_FLAG_REG : integer := 12; -- Finish Signal for HybridAttestation

  -- (stevez) CFS Hash Function Signal --
  signal data_ready:    std_logic;
  signal n_blocks:      natural;
  signal msg_block_in:  std_logic_vector(0 to (16 * WORD_SIZE)-1);
  signal finished:      std_logic;
  signal data_out:      std_logic_vector((WORD_SIZE*8)-1 downto 0);

begin
  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- This logic is required to handle the CPU accesses - DO NOT MODIFY!
  acc_en <= '1' when (bus_req_i.addr(hi_abb_c downto lo_abb_c) = cfs_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= cfs_base_c(31 downto lo_abb_c) & bus_req_i.addr(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and bus_req_i.we; -- only full-word write accesses are supported
  rden   <= acc_en and bus_req_i.re; -- read accesses always return a full 32-bit word

  -- (stevez) 11/27/2023 assign number of tasks based on its corresponding cfs_reg
  NUM_TASKS <= to_integer(unsigned(cfs_reg_rd(1)));

  -- CFS Generics ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- In it's default version the CFS provides three configuration generics:
  -- > CFS_IN_SIZE  - configures the size (in bits) of the CFS input conduit cfs_in_i
  -- > CFS_OUT_SIZE - configures the size (in bits) of the CFS output conduit cfs_out_o
  -- > CFS_CONFIG   - is a blank 32-bit generic. It is intended as a "generic conduit" to propagate
  --                  custom configuration flags from the top entity down to this module.


  -- CFS IOs --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- By default, the CFS provides two IO signals (cfs_in_i and cfs_out_o) that are available at the processor's top entity.
  -- These are intended as "conduits" to propagate custom signals from this module and the processor top entity.

 cfs_out_o <= (others => '0'); -- not used for this minimal example


  -- Reset System ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- The CFS can be reset using the global rstn_i signal. This signal should be used as asynchronous reset and is active-low.
  -- Note that rstn_i can be asserted by a processor-external reset, the on-chip debugger and also by the watchdog.
  --
  -- Most default peripheral devices of the NEORV32 do NOT use a dedicated hardware reset at all. Instead, these units are
  -- reset by writing ZERO to a specific "control register" located right at the beginning of the device's address space
  -- (so this register is cleared at first). The crt0 start-up code writes ZERO to every single address in the processor's
  -- IO space - including the CFS. Make sure that this initial clearing does not cause any unintended CFS actions.


  -- Clock System ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- The processor top unit implements a clock generator providing 8 "derived clocks".
  -- Actually, these signals should not be used as direct clock signals, but as *clock enable* signals.
  -- clkgen_i is always synchronous to the main system clock (clk_i).
  --
  -- The following clock dividers are available:
  -- > clkgen_i(clk_div2_c)    -> MAIN_CLK/2
  -- > clkgen_i(clk_div4_c)    -> MAIN_CLK/4
  -- > clkgen_i(clk_div8_c)    -> MAIN_CLK/8
  -- > clkgen_i(clk_div64_c)   -> MAIN_CLK/64
  -- > clkgen_i(clk_div128_c)  -> MAIN_CLK/128
  -- > clkgen_i(clk_div1024_c) -> MAIN_CLK/1024
  -- > clkgen_i(clk_div2048_c) -> MAIN_CLK/2048
  -- > clkgen_i(clk_div4096_c) -> MAIN_CLK/4096
  --
  -- For instance, if you want to drive a clock process at MAIN_CLK/8 clock speed you can use the following construct:
  --
  --   if (rstn_i = '0') then -- async and low-active reset (if required at all)
  --   ...
  --   elsif rising_edge(clk_i) then -- always use the main clock for all clock processes
  --     if (clkgen_i(clk_div8_c) = '1') then -- the div8 "clock" is actually a clock enable
  --       ...
  --     end if;
  --   end if;
  --
  -- The clkgen_i input clocks are available when at least one IO/peripheral device (for example UART0) requires the clocks
  -- generated by the clock generator. The CFS can enable the clock generator by itself by setting the clkgen_en_o signal high.
  -- The CFS cannot ensure to deactivate the clock generator by setting the clkgen_en_o signal low as other peripherals might
  -- still keep the generator activated. Make sure to deactivate the CFS's clkgen_en_o if no clocks are required in here to
  -- reduce dynamic power consumption.

  clkgen_en_o <= '0'; -- not used for this minimal example


  -- Interrupt ------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- The CFS features a single interrupt signal, which is connected to the CPU's "fast interrupt" channel 1 (FIRQ1).
  -- The interrupt is triggered by a one-cycle high-level. After triggering, the interrupt appears as "pending" in the CPU's
  -- mip CSR ready to trigger execution of the according interrupt handler. It is the task of the application to programmer
  -- to enable/clear the CFS interrupt using the CPU's mie and mip registers when required.

  -- irq_o <= '0'; -- not used for this minimal example 
  -- used in (stevez) 9/22/2023 CFS Interrupt One Pulse Wave Capture

  
  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Here we are reading/writing from/to the interface registers of the module and generate the CPU access handshake (bus response).
  --
  -- The CFS provides up to 64 memory-mapped 32-bit interface registers. For instance, these could be used to provide a
  -- <control register> for global control of the unit, a <data register> for reading/writing from/to a data FIFO, a
  -- <command register> for issuing commands and a <status register> for status information.
  --
  -- Following the interface protocol, each read or write access has to be acknowledged in the following cycle using the ack_o
  -- signal (or even later if the module needs additional time). If no ACK is generated at all, the bus access will time out
  -- and cause a bus access fault exception. The current CPU privilege level is available via the 'priv_i' signal (0 = user mode,
  -- 1 = machine mode), which can be used to constrain access to certain registers or features to privileged software only.
  --
  -- This module also provides an optional ERROR signal to indicate a faulty access operation (for example when accessing an
  -- unused, read-only or "locked" CFS register address). This signal may only be set when the module is actually accessed
  -- and is set INSTEAD of the ACK signal. Setting the ERR signal will raise a bus access exception with a "Device Error" qualifier
  -- that can be handled by the application software. Note that the current privilege level should not be exposed to software to
  -- maintain full virtualization. Hence, CFS-based "privilege escalation" should trigger a bus access exception (e.g. by setting 'err_o').

  bus_rsp_o.err <= '0'; -- Tie to zero if not explicitly used.


  -- Host access example: Read and write access to the interface registers + bus transfer acknowledge. This example only
  -- implements four physical r/w register (the four lowest CFS registers). The remaining addresses of the CFS are not associated
  -- with any physical registers - any access to those is simply ignored but still acknowledged. Only full-word write accesses are
  -- supported (and acknowledged) by this example. Sub-word write access will not alter any CFS register state and will cause
  -- a "bus store access" exception (with a "Device Timeout" qualifier as not ACK is generated in that case).
  
  -- (stevez) 10/29/2023 instant update for PC_Fetch value
  fetch_pc <= cpu_i_req.addr;
  
  -- (stevez) 10/30/2023 check which thread is currenlty running in hardware
--  boundary_checking: process(rstn_i, clk_i)
--  	begin
--		if (rstn_i = '0') then
--			thread1 <= '0';
--			thread2 <= '0';
--			thread3 <= '0';
--			thread_in_control <= "0000";
--		elsif rising_edge(clk_i) then
--			
--			if(fetch_pc >= LOWER_BOUND_T1) and (fetch_pc <= UPPER_BOUND_T1) then
--				thread1 <= '1';
--				thread2 <= '0';
--				thread3 <= '0';
--				thread_in_control <= "0010";
--				-- (stevez) pc jump after firq, store in cfs_reg_rd(61)
--				cfs_reg_rd(REG_NUM-2) <= fetch_pc;
--			elsif (fetch_pc >= LOWER_BOUND_T2) and (fetch_pc <= UPPER_BOUND_T2) then
--				thread1 <= '0';
--				thread2 <= '1';
--				thread3 <= '0';
--				thread_in_control <= "0100";
--				-- (stevez) pc jump after firq, store in cfs_reg_rd(61)
--				cfs_reg_rd(REG_NUM-2) <= fetch_pc;
--			elsif (fetch_pc >= LOWER_BOUND_T3) and (fetch_pc <= UPPER_BOUND_T3) then
--				thread1 <= '0';
--				thread2 <= '0';
--				thread3 <= '1';
--				thread_in_control <= "1000";
--				-- (stevez) pc jump after firq, store in cfs_reg_rd(61)
--				cfs_reg_rd(REG_NUM-2) <= fetch_pc;
--			else
--				thread1 <= '0';
--				thread2 <= '0';
--				thread3 <= '0';
--			end if;
--			
--		end if;  
--	end process boundary_checking;

-- (stevez) 11/27/2023 (max 10 tasks) REMAKE of Boundary_Checking
  boundary_checking: process(rstn_i, clk_i)
  	begin
		if (rstn_i = '0') then
			
			for i in 0 to MAX_NUM_TASKS-1 loop
				threads(i) <= '0';
			end loop;

			thread_in_control <= "0000";
		elsif rising_edge(clk_i) then
			
			for i in 0 to MAX_NUM_TASKS-1 loop
				
				-- first identify for the on run tasks
				if(NUM_TASKS > i) then
					
					-- check for which task is on run
					if(fetch_pc >= LOWER_BOUND_T(i)) and (fetch_pc <= UPPER_BOUND_T(i)) then

						threads(i) <= '1';
						
            CURRENT_TASK_NUM <= i + 1;

						thread_in_control <= std_ulogic_vector(to_unsigned(i+1, NUM_BITS));

            SECURE_CFS_REGIONS <= 0;

					else
						
						threads(i) <= '0';
						
					end if;
				end if;
			end loop;
      
      -- (stevez) 1/10/2024 check for whether PC in my_isr or task0 region
      if (fetch_pc >= LOWER_BOUND_SECURE_TASK0) and (fetch_pc <= UPPER_BOUND_SECURE_TASK0) then

          SECURE_CFS_REGIONS <= 1;
      
      elsif (fetch_pc >= LOWER_BOUND_FIRQ1) and (fetch_pc <= UPPER_BOUND_FIRQ1) then

          SECURE_CFS_REGIONS <= 2;

      end if;

		end if; 
	end process boundary_checking;
	
	
	-- (stevez) 10/30/2023 checking current thread validation
--	thread_checking: process(rstn_i, clk_i)
--  	begin
--		if (rstn_i = '0') then
--			thread_block <= '0';
--		elsif rising_edge(clk_i) then
--
--			if (thread1 = '1' or thread_in_control = "0010") then
--				thread_block <= cfs_reg_rd(1)(0);
--			elsif (thread2 = '1' or thread_in_control = "0100") then
--				thread_block <= cfs_reg_rd(2)(0);
--			elsif (thread3 = '1' or thread_in_control = "1000") then
--				thread_block <= cfs_reg_rd(3)(0);
--			else
--				thread_block <= '0';
--			end if;
--			
--		end if;  
--	end process thread_checking;
  
  -- (stevez) 9/29/2023 rewrote cfs register wiring
  -- (stevez) 11/28/2023 Only task0 allow cfs_reg write/read
  host_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
--      cfs_reg_wr(0) <= (others => '0');
--      cfs_reg_wr(1) <= (others => '0');
--      cfs_reg_wr(2) <= (others => '0');
--      cfs_reg_wr(3) <= (others => '0');
		cfs_reg_wr_rstn: for j in 0 to REG_NUM-1 loop 
			cfs_reg_wr(j) <= (others => '0');
		end loop;

      bus_rsp_o.ack  <= '0';
      bus_rsp_o.data <= (others => '0');
    elsif rising_edge(clk_i) then -- synchronous interface for read and write accesses
      -- transfer/access acknowledge --
      -- default: required for the CPU to check the CFS is answering a bus read OR write request;
      -- all read and write accesses (to any cfs_reg, even if there is no according physical register implemented) will succeed.
      bus_rsp_o.ack <= rden or wren;
		
		-- (stevez) 9/29/2023 below commented are lagacy code for read/write access below 

      -- write access --
--      if (wren = '1') then -- full-word write access, high for one cycle if there is an actual write access
--        if (addr = cfs_reg0_addr_c) then -- make sure to use the internal "addr" signal for the read/write interface
--          cfs_reg_wr(0) <= bus_req_i.data; -- some physical register, for example: control register
--        end if;
--        if (addr = cfs_reg1_addr_c) then
--          cfs_reg_wr(1) <= bus_req_i.data; -- some physical register, for example: data in/out fifo
--        end if;
--        if (addr = cfs_reg2_addr_c) then
--          cfs_reg_wr(2) <= bus_req_i.data; -- some physical register, for example: command fifo
--        end if;
--        if (addr = cfs_reg3_addr_c) then
--          cfs_reg_wr(3) <= bus_req_i.data; -- some physical register, for example: status register
--        end if;
--      end if;
		
		-- (stevez) 9/29/2023 newly write access
		if (wren = '1') then
      -- (stevez) 12/31/2023 if it's in TASK0, it will disable the barrier, thus written 0 to any cfs_reg is accept
      if(to_integer(unsigned(cfs_reg_rd(CFS_CTRL_REG))) = 2) then

        if(SECURE_CFS_REGIONS = 1) then
          cfs_reg_wr(to_integer(unsigned(bus_req_i.addr(lo_abb_c-1 downto 2)))) <= bus_req_i.data; -- and masking;
        elsif (SECURE_CFS_REGIONS = 2) then
          
          -- only allow zephyr interrupt handler to access index 0 to 9 cfs_reg
          if(to_integer(unsigned(bus_req_i.addr(lo_abb_c-1 downto 2))) >= 0) and (to_integer(unsigned(bus_req_i.addr(lo_abb_c-1 downto 2))) < 10 ) then
            cfs_reg_wr(to_integer(unsigned(bus_req_i.addr(lo_abb_c-1 downto 2)))) <= bus_req_i.data;
          end if;
        
        end if;

      else
      -- (stevez) 12/31/2023 if written data is 0 to any cfs_reg, it's ignored (mainly bc of some zephyr specific issue)
        if(to_integer(unsigned(bus_req_i.data)) > 0) then

          -- cfs_reg_wr(to_integer(unsigned(bus_req_i.addr(lo_abb_c-1 downto 2)))) <= bus_req_i.data; -- and masking;

          if(SECURE_CFS_REGIONS = 1) then
            cfs_reg_wr(to_integer(unsigned(bus_req_i.addr(lo_abb_c-1 downto 2)))) <= bus_req_i.data; -- and masking;
          elsif (SECURE_CFS_REGIONS = 2) then
            
            -- only allow zephyr interrupt handler to access index 0 to 9 cfs_reg
            if(to_integer(unsigned(bus_req_i.addr(lo_abb_c-1 downto 2))) >= 0) and (to_integer(unsigned(bus_req_i.addr(lo_abb_c-1 downto 2))) < 10 ) then
              cfs_reg_wr(to_integer(unsigned(bus_req_i.addr(lo_abb_c-1 downto 2)))) <= bus_req_i.data;
            end if;
          
          end if;
         

        end if;        
      end if;

    end if;
      -- read access --
		bus_rsp_o.data <= (others => '0'); -- the output HAS TO BE ZERO if there is no actual read access
		
--      if (rden = '1') then -- the read access is always 32-bit wide, high for one cycle if there is an actual read access
--        case addr is -- make sure to use the internal 'addr' signal for the read/write interface
--          when cfs_reg0_addr_c => bus_rsp_o.data <= cfs_reg_rd(0);
--          when cfs_reg1_addr_c => bus_rsp_o.data <= cfs_reg_rd(1);
--          when cfs_reg2_addr_c => bus_rsp_o.data <= cfs_reg_rd(2);
--          when cfs_reg3_addr_c => bus_rsp_o.data <= cfs_reg_rd(3);
--          when others          => bus_rsp_o.data <= (others => '0'); -- the remaining registers are not implemented and will read as zero
--        end case;
--      end if;
						
		-- (stevez) 9/29/2023 newly read access
		if (rden = '1') then
			bus_rsp_o.data <= cfs_reg_rd(to_integer(unsigned(bus_req_i.addr(lo_abb_c-1 downto 2)))); -- and masking;	-- (stevez) 11/20/2023 added masking
		end if;
		 -- (stevez) 10/28/2023 read fetch_pc from user side for testing purpose


    -- INPUT TO HASH
    data_ready <= cfs_reg_wr(DATA_READY_REG)(0);
    n_blocks <= to_integer(unsigned(cfs_reg_wr(NUM_BLOCKS_REG)));

    cfs_reg_rd(FINISH_FLAG_REG) <= cfs_reg_wr(FINISH_FLAG_REG); 

    -- DATA_OUT_REG_WRITE
    if (finished = '1') then 
      -- report "finished";  
      if (bus_req_i.we = '0') then
        cfs_reg_wr(FINISH_FLAG_REG)(0) <= '1'; -- only assign when finished = '1', else not since finished will be manully set to 0 in sha256 right after set to 1
      end if; -- we want user later to manully set this register to 0 
      cfs_reg_rd(data_out_reg_start+7)  <= to_stdulogicvector(data_out(31 downto 0));
      cfs_reg_rd(data_out_reg_start+6)  <= to_stdulogicvector(data_out(63 downto 32));
      cfs_reg_rd(data_out_reg_start+5)  <= to_stdulogicvector(data_out(95 downto 64));
      cfs_reg_rd(data_out_reg_start+4)  <= to_stdulogicvector(data_out(127 downto 96));
      cfs_reg_rd(data_out_reg_start+3)  <= to_stdulogicvector(data_out(159 downto 128));
      cfs_reg_rd(data_out_reg_start+2)  <= to_stdulogicvector(data_out(191 downto 160));
      cfs_reg_rd(data_out_reg_start+1)  <= to_stdulogicvector(data_out(223 downto 192));
      cfs_reg_rd(data_out_reg_start)    <= to_stdulogicvector(data_out(255 downto 224));
    end if;

      if (cfs_reg_rd(FINISH_FLAG_REG)(0) = '1') then
        cfs_reg_rd(FINISH_FLAG_REG)(0) <= '0';
      end if;

		--  cfs_reg_rd(REG_NUM-1) <= fetch_pc;

    end if;
  end process host_access;


  -- CFS Function Core ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- This is where the actual functionality can be implemented.
  -- The logic below is just a very simple example that transforms data
  -- from an input register into data in an output register.

  -- cfs_reg_rd(0) <= bin_to_gray_f(cfs_reg_wr(0)); -- convert binary to gray code
  -- cfs_reg_rd(1) <= gray_to_bin_f(cfs_reg_wr(1)); -- convert gray to binary code
  -- cfs_reg_rd(2) <= bit_rev_f(cfs_reg_wr(2)); -- bit reversal
  -- cfs_reg_rd(3) <= bswap32_f(cfs_reg_wr(3)); -- byte swap (endianness conversion)
  
  -- -- (stevez) CFS REG READ -------------------------------------
  -- --------------------------------------------------------------
	-- cfs_reg: for i in 0 to (REG_NUM-9) generate
	-- 		cfs_reg_rd(i) <= cfs_reg_wr(i);
	-- end generate;

  -- (stevez) 11/20/2023 I-O-Guarding cfs_reg masking
--  cfs_reg_masking: process(thread_block)
--  begin
--    if (thread_block = '1') then 
--      masking <= (others => '0');
--    else
--      masking <= (others => '1');
--    end if;
--  end process cfs_reg_masking;
  
  -- (stevez) 11/27/2023 I-O-Guarding cfs_reg masking REMAKE
  -- cfs_reg_masking: process(thread_block)
  -- begin
  --   -- if pc is within the TASK0 execution region, then allow read/write cfs_reg; else (other tasks) not allowed
  --   if (SECURE_CFS_REGIONS = 2) or (SECURE_CFS_REGIONS = 1) then 
  --     masking <= (others => '1');
  --   else
  --     masking <= (others => '0');
  --   end if;
  -- end process cfs_reg_masking;


	-- (stevez) 9/22/2023 CFS Interrupt One Pulse Wave Capture --
	-- Wave Diagram Pls on PPT ----------------------------------
	-- ----------------------------------------------------------
	
	irq_process: process(rstn_i, clk_i)
	begin
		if (rstn_i ='0') then 
			irq_cfs <= '0';
			irq_cfs_dly <= '0';
		elsif rising_edge(clk_i) then -- synchronous interface for read and write accesses
			
			irq_cfs_dly <= irq_cfs;
			
			if cfs_in_i(0) = '1' then   -- set cfs_in_i(0)(0) as the trigger to cfs interrupt (stevez) 10/12/2023
				irq_cfs <= '1';
--				cfs_reg_rd(REG_NUM-1) <= To_StdULogicVector(x"00000001");
--				cfs_out_o(gpio_o_num+2) <= irq_cfs;
			else
				irq_cfs <= '0';
--				cfs_reg_rd(REG_NUM-1) <= To_StdULogicVector(x"00000000");
--				cfs_out_o(gpio_o_num+2) <= irq_cfs;
			end if;
	
		end if;
	end process irq_process;
	
	irq_cfs_int <= irq_cfs and (not irq_cfs_dly); -- gettting CFS Interrupt One Pulses
  irq_o <= irq_cfs_int;
		

 -- (stevez) CFS REG READ ---------------------------------------
  ----------------------------------------------------------------
  cfs_reg: for i in 0 to 9 generate
    cfs_reg_rd(i) <= cfs_reg_wr(i);
-- report "R-> CFS_VHDL REG_[5]: " & integer'image(to_integer(unsigned(cfs_reg_wr(i))));
  end generate;

-- (stevez) CFS Hash Function-----------------------------------------------------------------
-- -- -------------------------------------------------------------------------------------------
CFS_Functions_Generator:
if(true) generate

neorv32_sha256: entity neorv32.neorv32_sha_256_core
generic map (
  RESET_VALUE => '0'
)
port map(
  clk                    => clk_i,
  rst                    => rstn_i,
  data_ready             => data_ready,
  n_blocks               => n_blocks,
  msg_block_in           => msg_block_in,
  finished               => finished,
  data_out               => data_out
);

end generate;


msg_block_in <= to_stdlogicvector(cfs_reg_wr(16) & cfs_reg_wr(17) & cfs_reg_wr(18) & cfs_reg_wr(19) & 
                                  cfs_reg_wr(20) & cfs_reg_wr(21) & cfs_reg_wr(22) & cfs_reg_wr(23) & 
                                  cfs_reg_wr(24) & cfs_reg_wr(25) & cfs_reg_wr(26) & cfs_reg_wr(27) & 
                                  cfs_reg_wr(28) & cfs_reg_wr(29) & cfs_reg_wr(30) & cfs_reg_wr(31));

end neorv32_cfs_rtl;




-- I-O-Guard CFS_REG Assign Base on Tasks
