library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Bitmap Writer
--
-- This module plugs into one port of a true dual port BRAM. It reads-modify-writes the
-- counter of the incoming edge from the edge extractor.
--
-- An extension to avoid clearing the whole bitmap would be to use 8-bit epochs written
-- next to the actual counter. The epoch is expected to be cleared through the other port
-- of the BRAM that handles the AXI-Stream DMA writing.

entity bitmap_writer_bram is
    generic (
        ADDR_WIDTH   : integer := 16 -- makes a 64KB map, AFL++ default
    );
    port (
        aclk        : in  std_logic;
        aresetn     : in  std_logic;

        -- FIFO interface from edge extractor
        i_fifo_index  : in  std_logic_vector(63 downto 0);
        i_fifo_valid  : in  std_logic;
        o_fifo_ready  : out std_logic;

        i_fifo_freeze_req : in std_logic;

        -- BRAM interface
        bram_addr     : out std_logic_vector(ADDR_WIDTH-1 downto 0);
        bram_din      : out std_logic_vector(7 downto 0);
        bram_dout     : in  std_logic_vector(7 downto 0);
        bram_en       : out std_logic;
        bram_we       : out std_logic
    );
end entity;

architecture Behavioral of bitmap_writer_bram is
    signal fifo_ready : std_logic;
begin

    -- FIFO output
    o_fifo_ready <= fifo_ready and not i_fifo_freeze_req;

    -- BRAM edge writing process
    edge_update_process: process(aclk)
        variable v_counter : unsigned(7 downto 0);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                bram_en    <= '0';
                bram_we    <= '0';
                fifo_ready <= '1';
            else
                if i_fifo_valid = '1' and fifo_ready = '1' then
                    bram_en   <= '1';
                    bram_addr <= i_fifo_index(ADDR_WIDTH-1 downto 0);

                    -- latch old word
                    v_counter := unsigned(bram_dout);


                    if v_counter < 255 then
                        v_counter := v_counter + 1;
                    end if;

                    bram_din <= std_logic_vector(v_counter);
                    bram_we  <= '1';
                    fifo_ready <= '1';
                else
                    bram_en <= '0';
                    bram_we <= '0';
                end if;

                fifo_ready <= '1';
            end if;
        end if;
    end process;

end Behavioral;