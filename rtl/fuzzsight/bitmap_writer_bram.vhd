library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Bitmap Writer
--
-- This module plugs into one port of a true dual port BRAM. It reads-modify-writes the
-- counter of the incoming edge from the edge extractor. It uses a simple FSM to account
-- for the BRAM one cycle read latency and uses a forwarder when a repeated address is
-- used. The whole read-modify-write takes 3 cycles when new addresses are coming but
-- forwards previous counter if the same address is repeated, coming down to 2 cycles in
-- that case.
--
-- An extension to avoid clearing the whole bitmap would be to use 8-bit epochs written
-- next to the actual counter. The epoch is expected to be cleared through the other port
-- of the BRAM that handles the AXI-Stream DMA writing.

entity bitmap_writer_bram is
    generic (
        ADDR_WIDTH : integer := 16
    );
    port (
        aclk      : in  std_logic;
        aresetn   : in  std_logic;

        -- FIFO interface from edge extractor
        i_fifo_index     : in  std_logic_vector(63 downto 0);
        i_fifo_valid     : in  std_logic;
        o_fifo_ready     : out std_logic;

        -- BRAM interface
        bram_addr : out std_logic_vector(ADDR_WIDTH-1 downto 0);
        bram_din  : out std_logic_vector(7 downto 0);
        bram_dout : in  std_logic_vector(7 downto 0);
        bram_en   : out std_logic;
        bram_we   : out std_logic
    );
end entity;

architecture Behavioral of bitmap_writer_bram is

    type state_t is (IDLE, READING, WRITING);
    signal state : state_t := IDLE;

    -- Latched edge address from IDLE
    signal latched_addr : std_logic_vector(ADDR_WIDTH-1 downto 0);

    -- Forwarding register
    signal fwd_valid : std_logic := '0';
    signal fwd_addr  : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal fwd_val   : unsigned(7 downto 0);

begin

    o_fifo_ready <= '1' when state = IDLE else '0';

    process(aclk)
        variable v_addr    : std_logic_vector(ADDR_WIDTH-1 downto 0);
        variable v_counter : unsigned(7 downto 0);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                state      <= IDLE;
                fwd_valid  <= '0';
                bram_en    <= '0';
                bram_we    <= '0';
            else
                -- Defaults
                bram_en <= '0';
                bram_we <= '0';

                case state is

                    -- IDLE: waiting for a valid fifo input
                    when IDLE =>
                        if i_fifo_valid = '1' then
                            v_addr := i_fifo_index(ADDR_WIDTH-1 downto 0);
                            latched_addr <= v_addr;

                            if fwd_valid = '1' and fwd_addr = v_addr then
                                -- Forwarding hit: skip READING, go straight to WRITING
                                state <= WRITING;
                            else
                                -- Forwarding miss: issue BRAM read
                                bram_en   <= '1';
                                bram_addr <= v_addr;
                                state     <= READING;
                            end if;
                        end if;

                    -- READING: BRAM has a 1-cycle read latency
                    when READING =>
                        -- BRAM read latency cycle, bram_dout valid next cycle
                        state <= WRITING;

                    -- WRITING: Increment counter, either from the read BRAM or forwarded counter
                    when WRITING =>
                        -- Select source: forwarding register or BRAM
                        if fwd_valid = '1' and fwd_addr = latched_addr then
                            v_counter := fwd_val;
                        else
                            v_counter := unsigned(bram_dout);
                        end if;

                        -- Saturating increment
                        if v_counter < 255 then
                            v_counter := v_counter + 1;
                        end if;

                        -- Write back to BRAM
                        bram_en   <= '1';
                        bram_addr <= latched_addr;
                        bram_din  <= std_logic_vector(v_counter);
                        bram_we   <= '1';

                        -- Update forwarding register
                        fwd_valid <= '1';
                        fwd_addr  <= latched_addr;
                        fwd_val   <= v_counter;

                        state <= IDLE;

                end case;
            end if;
        end if;
    end process;

end Behavioral;