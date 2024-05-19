-- #################################################################################################
-- # << NEORV32 - General Purpose Parallel Input/Output Port (GPIO) >>                             #
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

entity neorv32_gpio is
  generic (
    GPIO_NUM : natural -- number of GPIO input/output pairs (0..64)
  );
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- global reset line, low-active, async
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t;  -- bus response
    gpio_o    : out std_ulogic_vector(63 downto 0); -- parallel output
    gpio_i    : in  std_ulogic_vector(63 downto 0)  -- parallel input
  );
end neorv32_gpio;

architecture neorv32_gpio_rtl of neorv32_gpio is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(gpio_size_c); -- low address boundary bit

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- accessible regs --
  signal din, din_rd, dout, dout_rd : std_ulogic_vector(63 downto 0);

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not ((GPIO_NUM < 0) or (GPIO_NUM > 64)) report
    "NEORV32 PROCESSOR CONFIG ERROR! Invalid GPIO pin number configuration (0..64)." severity error;


  -- Host Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- access control --
  acc_en <= '1' when (bus_req_i.addr(hi_abb_c downto lo_abb_c) = gpio_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= gpio_base_c(31 downto lo_abb_c) & bus_req_i.addr(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and bus_req_i.we;
  rden   <= acc_en and bus_req_i.re;

  -- write access --
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      dout <= (others => '0');
    elsif rising_edge(clk_i) then
      if (wren = '1') then
        if (addr = gpio_out_lo_addr_c) then
          dout(31 downto 00) <= bus_req_i.data;
        end if;
        if (addr = gpio_out_hi_addr_c) then
          dout(63 downto 32) <= bus_req_i.data;
        end if;
      end if;
    end if;
  end process write_access;

  -- read access --
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- bus handshake --
      bus_rsp_o.ack <= wren or rden;
      -- read data --
      bus_rsp_o.data <= (others => '0');
      if (rden = '1') then
        case addr(3 downto 2) is
          when "00"   => bus_rsp_o.data <= din_rd(31 downto 00);
          when "01"   => bus_rsp_o.data <= din_rd(63 downto 32);
          when "10"   => bus_rsp_o.data <= dout_rd(31 downto 00);
          when others => bus_rsp_o.data <= dout_rd(63 downto 32);
        end case;
      end if;
    end if;
  end process read_access;

  -- no access error possible --
  bus_rsp_o.err <= '0';


  -- Physical Pin Mapping -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  pin_mapping: process(din, dout)
  begin
    -- defaults --
    din_rd  <= (others => '0');
    dout_rd <= (others => '0');
    for i in 0 to GPIO_NUM-1 loop
      din_rd(i)  <= din(i);
      dout_rd(i) <= dout(i);
    end loop;
  end process pin_mapping;

  -- IO --
  gpio_o <= dout_rd;
  din    <= gpio_i when rising_edge(clk_i); -- sample buffer to prevent metastability


end neorv32_gpio_rtl;
