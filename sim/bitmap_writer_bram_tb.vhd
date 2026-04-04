library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity bitmap_writer_bram_tb is
-- Port ( );
end bitmap_writer_bram_tb;

architecture Simulation of bitmap_writer_bram_tb is

    -- Constants
    constant ADDR_WIDTH : integer := 16; -- could be reduced for simulation (256B)

    -- Types
    type ram_type is array (0 to (2**ADDR_WIDTH) - 1)  of std_logic_vector(7 downto 0);

    -- Component definition
    component bitmap_writer_bram
    port (
        aclk        : in  std_logic;
        aresetn     : in  std_logic;

        o_idle      : out std_logic;

        i_fifo_index      : in  std_logic_vector(63 downto 0);
        i_fifo_valid      : in  std_logic;
        o_fifo_ready      : out std_logic;

        bram_addr : out std_logic_vector(ADDR_WIDTH-1 downto 0);
        bram_din  : out std_logic_vector(7 downto 0);
        bram_dout : in  std_logic_vector(7 downto 0);
        bram_en   : out std_logic;
        bram_we   : out std_logic
    );
    end component;

    -- Clock and reset
    signal clock  : std_logic := '1';
    signal reset  : std_logic := '1';

    -- Idle info
    signal writer_idle : std_logic;

    -- FIFO signals
    signal fifo_index  : std_logic_vector(63 downto 0);
    signal fifo_valid  : std_logic;
    signal fifo_ready  : std_logic;

    -- BRAM signals
    signal bram_addr : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal bram_din  : std_logic_vector(7 downto 0);
    signal bram_dout : std_logic_vector(7 downto 0) := (others=>'0');
    signal bram_en   : std_logic;
    signal bram_we   : std_logic;

    -- Simulation RAM
    signal ram : ram_type := (others => (others => '0'));

begin
    -- Clock and reset
    clock <= not clock after 1 ns;
    reset <= '0', '1' after 6 ns;

    DUT: bitmap_writer_bram
    port map (
        aclk     => clock,
        aresetn  => reset,

        -- Idle info
        o_idle       => writer_idle,

        -- FIFO interface from edge extractor
        i_fifo_index => fifo_index,
        i_fifo_valid => fifo_valid,
        o_fifo_ready => fifo_ready,

        -- BRAM interface
        bram_addr => bram_addr,
        bram_din  => bram_din,
        bram_dout => bram_dout,
        bram_en   => bram_en,
        bram_we   => bram_we
    );

    -- BRAM process, 1-cycle read latency in read-first mode
    bram_process: process(clock)
    begin
        if rising_edge(clock) then
            if bram_en = '1' then
                bram_dout <= ram(to_integer(unsigned(bram_addr)));  -- always, read-first
                if bram_we = '1' then
                    ram(to_integer(unsigned(bram_addr))) <= bram_din;
                end if;
            end if;
        end if;
    end process;
    -- Simulation process
    simulation_process: process
    begin
        -- clear inputs
        fifo_valid <= '0';
        fifo_index <= (others => '0');

        -- wait for reset
        wait until reset = '1';
        wait for 6 ns;

        -------------------------------------------------------------------------
        -- Test 1: Continuous stream valid stays high

        -- 0x10 first hit
        fifo_index <= x"0000000000000010";
        fifo_valid <= '1';
        wait until rising_edge(clock) and fifo_ready = '1';

        -- 0x20 first hit (valid never drops)
        fifo_index <= x"0000000000000020";
        wait until rising_edge(clock) and fifo_ready = '1';

        -- 0x10 x4 increments
        fifo_index <= x"0000000000000010";
        wait until rising_edge(clock) and fifo_ready = '1';


        -- 0x30 one hit
        fifo_index <= x"0000000000000030";
        wait until rising_edge(clock) and fifo_ready = '1';

        -- 0x20 second hit
        fifo_index <= x"0000000000000020";
        wait until rising_edge(clock) and fifo_ready = '1';

        fifo_valid <= '0';

        -- drain pipeline
        wait until rising_edge(clock) and fifo_ready = '1';
        wait until rising_edge(clock);

        assert ram(16#10#) = std_logic_vector(to_unsigned(2, 8))
            report "0x10: expected 2 got " & integer'image(to_integer(unsigned(ram(16#10#))));
        assert ram(16#20#) = std_logic_vector(to_unsigned(2, 8))
            report "0x20: expected 2 got " & integer'image(to_integer(unsigned(ram(16#20#))));
        assert ram(16#30#) = std_logic_vector(to_unsigned(1, 8))
            report "0x30: expected 1 got " & integer'image(to_integer(unsigned(ram(16#30#))));

        -------------------------------------------------------------------------
        -- Test 2: forwarding by sending same address 3 times back-to-back

        fifo_index <= x"0000000000000040";
        fifo_valid <= '1';
        wait until rising_edge(clock) and fifo_ready = '1';
        wait until rising_edge(clock) and fifo_ready = '1';
        wait until rising_edge(clock) and fifo_ready = '1';
        fifo_valid <= '0';
        wait until rising_edge(clock) and fifo_ready = '1';
        wait until rising_edge(clock);

        assert ram(16#40#) = std_logic_vector(to_unsigned(3, 8))
            report "Forwarding test: 0x40 expected 3 got "
                & integer'image(to_integer(unsigned(ram(16#40#))));

        -------------------------------------------------------------------------
        -- Test 3: saturation, pre-fill 254, send 3 hits, expect 255

        fifo_index <= x"0000000000000050";
        fifo_valid <= '1';
        for i in 0 to 255 loop
            wait until rising_edge(clock) and fifo_ready = '1';
        end loop;

        wait until rising_edge(clock) and fifo_ready = '1';
        wait until rising_edge(clock) and fifo_ready = '1';
        wait until rising_edge(clock) and fifo_ready = '1';
        fifo_valid <= '0';
        wait until rising_edge(clock) and fifo_ready = '1';
        wait until rising_edge(clock);

        assert ram(16#50#) = x"FF"
            report "Saturation test: expected 255 got "
                & integer'image(to_integer(unsigned(ram(16#50#))));

        -------------------------------------------------------------------------
        -- Test 4: o_idle deasserts during transaction and reasserts only
        -- after the BRAM write completes

        fifo_index <= x"0000000000000060";
        fifo_valid <= '1';
        wait until rising_edge(clock) and fifo_ready = '1';
        -- writer just accepted the entry, now in READING or WRITING
        fifo_valid <= '0';

        -- o_idle must be low while processing
        assert writer_idle = '0'
            report "Test 4: writer_idle should be low during transaction";

        -- wait for idle
        wait until rising_edge(clock) and writer_idle = '1';

        -- BRAM must be written by now
        assert ram(16#60#) = std_logic_vector(to_unsigned(1, 8))
            report "Test 4: 0x60 expected 1 got "
                & integer'image(to_integer(unsigned(ram(16#60#))));

        wait;
    end process;
end Simulation;