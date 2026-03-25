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

        i_fifo_index      : in  std_logic_vector(63 downto 0);
        i_fifo_valid      : in  std_logic;
        o_fifo_ready      : out std_logic;
        i_fifo_freeze_req : in  std_logic;

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

    -- FIFO signals
    signal fifo_index  : std_logic_vector(63 downto 0);
    signal fifo_valid  : std_logic;
    signal fifo_ready  : std_logic;
    signal fifo_freeze_req  : std_logic := '0';

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

        -- FIFO interface from edge extractor
        i_fifo_index => fifo_index,
        i_fifo_valid => fifo_valid,
        o_fifo_ready => fifo_ready,

        i_fifo_freeze_req => fifo_freeze_req,

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
            -- read-first: sample address before write
            if bram_en = '1' and bram_we = '0' then
                bram_dout <= ram(to_integer(unsigned(bram_addr)));
            end if;

            -- write happens after read sample
            if bram_en = '1' and bram_we = '1' then
                ram(to_integer(unsigned(bram_addr))) <= bram_din;
            end if;
        end if;
    end process;

    -- Simulation process
    simulation_process: process
    begin
        -- clear inputs
        fifo_valid <= '0';
        fifo_index <= (others => '0');
        fifo_freeze_req <= '0';

        -- wait for reset
        wait until reset = '1';
        wait for 6 ns;

        ----------------------------------------------------------------
        -- Continuous stream, valid stays high between transactions
        ----------------------------------------------------------------

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
        wait until rising_edge(clock) and fifo_ready = '1';
        wait until rising_edge(clock) and fifo_ready = '1';
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


        ----------------------------------------------------------------
        -- Freeze test
        ----------------------------------------------------------------
        fifo_freeze_req <= '1';
        for i in 0 to 3 loop
            fifo_index <= x"0000000000000030";
            fifo_valid <= '1';
            wait until rising_edge(clock);
            fifo_valid <= '0';
            wait until rising_edge(clock);
        end loop;
        fifo_freeze_req <= '0';

        wait;
    end process;
end Simulation;