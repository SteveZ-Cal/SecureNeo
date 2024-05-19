library ieee;
use ieee.std_logic_1164.all;

-- Added Libaray (stevez)
use ieee.numeric_std.all;
-- use std.textio.all;

library neorv32;
use neorv32.neorv32_package.all;

-- function is just output = input + 1;

entity neorv32_sha256 is
    port (
        clk_i     : in  std_ulogic; -- global clock line
        rstn_i    : in  std_ulogic; -- global reset line, low-active, async
        bus_req_i : in  bus_req_t;  -- bus request
        bus_rsp_o : out bus_rsp_t   -- bus response
        -- irq_o     : out std_ulogic  -- CPU interrupt
    );
end entity neorv32_sha256;

architecture neorv32_sha256_rtl of neorv32_sha256 is

    -- control register bits for msb & lsb--
    constant ctrl_data_lsb_c      : natural :=  0; -- r/-: SHA256 input byte LSB
    constant ctrl_data_msb_c      : natural :=  31; -- r/-: SHA256 input byte MSB
    signal counter_up: integer := 1;
    signal slv : std_ulogic_vector(23 downto 0) := x"ABCDEF";
    signal hash_reg: std_ulogic_vector(31 downto 0);

begin

    RWaccess: process(clk_i, rstn_i)
    begin

        if(rstn_i = '0') then
            counter_up <= 0;
            hash_reg <= (others => '0');
        elsif rising_edge(clk_i) then
            -- write access --
            if (bus_req_i.we = '1') then -- full-word write access, high for one cycle if there is an actual write access
                hash_reg <= bus_req_i.data;
                report "Writing Hash Register Data " & integer'image(to_integer(unsigned(hash_reg)));
            end if;
            -- reading access -- 
            counter_up <= counter_up + 1;
            bus_rsp_o.ack  <= bus_req_i.we or bus_req_i.re; -- host bus acknowledge
            -- report "The value of 'c' is "& integer'image(counter_up);
            if (bus_req_i.re = '1') then
                report "Reading Hash Register Data " & integer'image(to_integer(unsigned(hash_reg)));
                report "Counter: " & integer'image(counter_up);
                -- report "sha256 bus_req_t data: " & to_hstring(slv);
                -- bus_rsp_o.data(ctrl_data_msb_c downto ctrl_data_lsb_c) <= bus_req_i.data(ctrl_data_msb_c downto ctrl_data_lsb_c); -- std_logic_vector: output the input
            end if;
        end if;

    end process RWaccess;

    bus_rsp_o.data <= hash_reg;

end neorv32_sha256_rtl;