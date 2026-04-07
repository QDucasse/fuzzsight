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

        o_idle           : out std_logic;
        i_freeze_request : in  std_logic;

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

    procedure clear_bram_addr (
        constant addr     : in  integer;
        signal tb_bram_addr : out std_logic_vector(ADDR_WIDTH-1 downto 0);
        signal tb_bram_din  : out std_logic_vector(7 downto 0);
        signal tb_bram_en   : out std_logic;
        signal tb_bram_we   : out std_logic;
        signal clock        : in  std_logic
    ) is
    begin
        tb_bram_addr <= std_logic_vector(to_unsigned(addr, ADDR_WIDTH));
        tb_bram_din  <= (others => '0');
        tb_bram_en   <= '1';
        tb_bram_we   <= '1';
        wait until rising_edge(clock);
        tb_bram_en   <= '0';
        tb_bram_we   <= '0';
    end procedure;

    -- Clock and reset
    signal clock  : std_logic := '1';
    signal reset  : std_logic := '1';

    -- Idle info
    signal writer_idle : std_logic;

    signal freeze_req : std_logic := '0';

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

    -- TB-side BRAM port (simulates DMA clearing via port B)
    signal tb_bram_addr : std_logic_vector(ADDR_WIDTH-1 downto 0) := (others => '0');
    signal tb_bram_din  : std_logic_vector(7 downto 0)            := (others => '0');
    signal tb_bram_en   : std_logic := '0';
    signal tb_bram_we   : std_logic := '0';

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

        -- Freeze request from reader
        i_freeze_request => freeze_req,

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
            -- Port B: testbench (DMA simulation)
            if tb_bram_en = '1' and tb_bram_we = '1' then
                ram(to_integer(unsigned(tb_bram_addr))) <= tb_bram_din;
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

        -- 0x10 increments
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
        -- Test 3: o_idle deasserts during transaction and reasserts only
        -- after the BRAM write completes

        fifo_index <= x"0000000000000050";
        fifo_valid <= '1';
        wait until rising_edge(clock) and fifo_ready = '1';
        -- writer just accepted the entry, now in READING or WRITING
        fifo_valid <= '0';
        wait until rising_edge(clock);

        -- o_idle must be low while processing
        assert writer_idle = '0'
            report "Test 3: writer_idle should be low during transaction";

        -- wait for idle
        wait until rising_edge(clock) and writer_idle = '1';
        wait until rising_edge(clock);

        -- BRAM must be written by now
        assert ram(16#50#) = std_logic_vector(to_unsigned(1, 8))
            report "Test 3: 0x50 expected 1 got "
                & integer'image(to_integer(unsigned(ram(16#50#))));

        -------------------------------------------------------------------------
        -- Test 4: fwd_valid cleared on falling edge of freeze_request
        -- After a forwarding hit is established, simulate a DMA cycle
        -- (freeze high then low) and verify the next access goes to BRAM

        -- Hit 0x60 twice to establish forwarding
        fifo_index <= x"0000000000000060";
        fifo_valid <= '1';
        wait until rising_edge(clock) and fifo_ready = '1';
        wait until rising_edge(clock) and fifo_ready = '1';
        fifo_valid <= '0';
        wait until rising_edge(clock) and fifo_ready = '1';
        wait until rising_edge(clock);
        wait until rising_edge(clock);

        assert ram(16#60#) = std_logic_vector(to_unsigned(2, 8))
            report "Test 4 setup: 0x60 expected 2 got "
                & integer'image(to_integer(unsigned(ram(16#60#))));

        -- Simulate freeze (DMA cycle): assert then deassert freeze
        freeze_req <= '1';
        wait until rising_edge(clock);
        wait until rising_edge(clock);
        -- Simulate BRAM being cleared (as DMA would do)
        clear_bram_addr(16#60#, tb_bram_addr, tb_bram_din, tb_bram_en, tb_bram_we, clock);
        freeze_req <= '0';
        wait until rising_edge(clock);

        -- Hit 0x60 again - fwd_valid should be cleared, so BRAM read must occur
        -- BRAM was zeroed, so result should be 1, not 3 (which fwd_val would give)
        fifo_index <= x"0000000000000060";
        fifo_valid <= '1';
        wait until rising_edge(clock) and fifo_ready = '1';
        fifo_valid <= '0';
        wait until rising_edge(clock) and fifo_ready = '1';
        wait until rising_edge(clock);

        assert ram(16#60#) = std_logic_vector(to_unsigned(1, 8))
            report "Test 4: fwd not cleared, expected 1 got "
                & integer'image(to_integer(unsigned(ram(16#60#))));

        -------------------------------------------------------------------------
        -- Test 5: saturation, pre-fill 254, send 3 hits, expect 255

        fifo_index <= x"0000000000000070";
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

        assert ram(16#70#) = x"FF"
            report "Saturation test: expected 255 got "
                & integer'image(to_integer(unsigned(ram(16#70#))));

        wait;
    end process;
end Simulation;