library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity bitmap_reader_bram_tb is
-- Port ( );
end bitmap_reader_bram_tb;

architecture Simulation of bitmap_reader_bram_tb is
    --------------------
    -- Constants/Types
    --------------------

    -- Constants
    constant ADDR_WIDTH : integer := 8; -- could be smaller for simulation (256B)
    constant AXIL_WIDTH : integer := 8;
    constant AXIS_WIDTH : integer := 32;
    constant MAP_SIZE   : integer := 2**ADDR_WIDTH;
    constant WORD_COUNT : integer := MAP_SIZE / 4;

    -- Types
    type ram_type is array (0 to WORD_COUNT - 1)  of std_logic_vector(31 downto 0);
    -- Note: BRAM on the read port has a 32-bit width

    --------------------
    -- Component
    --------------------

    -- Component definition
    component bitmap_reader_bram
        generic (
            ADDR_WIDTH: integer;
            AXIL_WIDTH: integer;
            AXIS_WIDTH: integer
        );
        port (
        aclk    : in std_logic;
        aresetn : in std_logic;

        -- FIFO info
        i_fifo_empty      : in  std_logic;
        o_fifo_freeze_req : out std_logic;

        -- BRAM interface
        bram_addr : out std_logic_vector(ADDR_WIDTH-1 downto 0);
        bram_din  : out std_logic_vector(31 downto 0);
        bram_dout : in  std_logic_vector(31 downto 0);
        bram_en   : out std_logic;
        bram_we   : out std_logic;

        -- AXI-Stream interface
        m_axis_tready : in  std_logic;
        m_axis_tdata  : out std_logic_vector(AXIS_WIDTH-1 downto 0);
        m_axis_tvalid : out std_logic;
        m_axis_tlast  : out std_logic;

        -- AXI4-Lite interface
        -- write address channel
        s_axi_awaddr  : in  std_logic_vector(AXIL_WIDTH-1 downto 0);
        s_axi_awvalid : in  std_logic;
        s_axi_awready : out std_logic;
        -- write data channel
        s_axi_wdata   : in  std_logic_vector(31 downto 0);
        s_axi_wvalid  : in  std_logic;
        s_axi_wready  : out std_logic;
        -- write response channel
        s_axi_bresp   : out std_logic_vector(1 downto 0);
        s_axi_bvalid  : out std_logic;
        s_axi_bready  : in  std_logic;
        -- read address channel
        s_axi_araddr  : in  std_logic_vector(AXIL_WIDTH-1 downto 0);
        s_axi_arvalid : in  std_logic;
        s_axi_arready : out std_logic;
        -- read data channel
        s_axi_rdata   : out std_logic_vector(31 downto 0);
        s_axi_rresp   : out std_logic_vector(1 downto 0);
        s_axi_rvalid  : out std_logic;
        s_axi_rready  : in  std_logic
    );
    end component;

    --------------------
    -- Procedures
    --------------------

    -- Helper procedure for AXI-Lite write
    procedure axi_lite_write(
        signal awaddr  : out std_logic_vector;
        signal awvalid : inout std_logic;
        signal wdata   : out std_logic_vector;
        signal wvalid  : inout std_logic;
        signal awready : in std_logic;
        signal wready  : in std_logic;
        signal bvalid  : in std_logic;
        signal clock   : in std_logic;
        addr           : in std_logic_vector;
        data           : in std_logic_vector
    ) is
        variable aw_done : boolean := false;
        variable w_done  : boolean := false;
    begin
        -- Assert write address and write data
        awaddr  <= addr;
        awvalid <= '1';
        wdata   <= data;
        wvalid  <= '1';

        -- Wait for both write channel and address write channel to be done
        while not (aw_done and w_done) loop
            wait until rising_edge(clock);
            if awvalid = '1' and awready = '1' then
                awvalid <= '0';
                aw_done := true;
            end if;
            if wvalid = '1' and wready = '1' then
                wvalid <= '0';
                w_done := true;
            end if;
        end loop;

        -- Wait for response
        wait until rising_edge(clock) and bvalid = '1';
    end procedure;

    -- Helper procedure for AXI-Lite read
    procedure axi_lite_read(
        signal araddr  : out std_logic_vector;
        signal arvalid : inout std_logic;
        signal rdata   : in std_logic_vector;
        signal arready : in std_logic;
        signal rvalid  : in std_logic;
        signal clock   : in std_logic;
        addr           : in std_logic_vector;
        variable data  : out std_logic_vector(31 downto 0)
    ) is
        variable ar_done : boolean := false;
    begin
        -- Assert read address
        araddr  <= addr;
        arvalid <= '1';

        -- Wait for both read channel and address read channel to be done
        while not ar_done loop
            wait until rising_edge(clock);
            if arvalid = '1' and arready = '1' then
                arvalid <= '0';
                ar_done := true;
            end if;
        end loop;

        -- Wait for valid data
        wait until rising_edge(clock) and rvalid = '1';
        data := rdata;
    end procedure;

    --------------------
    -- Signals
    --------------------

    -- Clock and reset
    signal clock  : std_logic := '1';
    signal reset  : std_logic := '1';

    -- FIFO signals
    signal fifo_empty : std_logic;
    signal fifo_freeze_req : std_logic := '0';

    -- BRAM signals
    signal bram_addr : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal bram_din  : std_logic_vector(31 downto 0);
    signal bram_dout : std_logic_vector(31 downto 0) := (others=>'0');
    signal bram_en   : std_logic;
    signal bram_we   : std_logic;

    -- AXIS signals
    signal axis_tready : std_logic;
    signal axis_tdata  : std_logic_vector(AXIS_WIDTH-1 downto 0);
    signal axis_tvalid : std_logic;
    signal axis_tlast  : std_logic;

    -- AXI4-Lite signals
    signal axi_awaddr  : std_logic_vector(AXIL_WIDTH-1 downto 0) := (others=>'0');
    signal axi_awvalid : std_logic := '0';
    signal axi_awready : std_logic;
    signal axi_wdata   : std_logic_vector(31 downto 0) := (others=>'0');
    signal axi_wvalid  : std_logic := '0';
    signal axi_wready  : std_logic;
    signal axi_bresp   : std_logic_vector(1 downto 0);
    signal axi_bvalid  : std_logic;
    signal axi_bready  : std_logic := '1';
    signal axi_araddr  : std_logic_vector(AXIL_WIDTH-1 downto 0) := (others=>'0');
    signal axi_arvalid : std_logic := '0';
    signal axi_arready : std_logic;
    signal axi_rdata   : std_logic_vector(31 downto 0) := (others=>'0');
    signal axi_rresp   : std_logic_vector(1 downto 0);
    signal axi_rvalid  : std_logic;
    signal axi_rready  : std_logic := '1';

    -- Simulation RAM
    shared variable ram : ram_type := (others => (others => '0'));

begin
    -- Clock and reset
    clock <= not clock after 1 ns;
    reset <= '0', '1' after 6 ns;

    DUT: bitmap_reader_bram
    generic map (
        ADDR_WIDTH => ADDR_WIDTH,
        AXIL_WIDTH => AXIL_WIDTH,
        AXIS_WIDTH => AXIS_WIDTH
    )
    port map (
        aclk    => clock,
        aresetn => reset,

        -- FIFO info
        i_fifo_empty      => fifo_empty,
        o_fifo_freeze_req => fifo_freeze_req,

        -- BRAM interface
        bram_addr => bram_addr,
        bram_din  => bram_din,
        bram_dout => bram_dout,
        bram_en   => bram_en,
        bram_we   => bram_we,

        -- AXI-Stream interface
        m_axis_tready => axis_tready,
        m_axis_tdata  => axis_tdata,
        m_axis_tvalid => axis_tvalid,
        m_axis_tlast  => axis_tlast,

        -- AXI4-Lite interface
        -- write address channel
        s_axi_awaddr  => axi_awaddr,
        s_axi_awvalid => axi_awvalid,
        s_axi_awready => axi_awready,
        -- write data channel
        s_axi_wdata   => axi_wdata,
        s_axi_wvalid  => axi_wvalid,
        s_axi_wready  => axi_wready,
        -- write response channel
        s_axi_bresp   => axi_bresp,
        s_axi_bvalid  => axi_bvalid,
        s_axi_bready  => axi_bready,
        -- read address channel
        s_axi_araddr  => axi_araddr,
        s_axi_arvalid => axi_arvalid,
        s_axi_arready => axi_arready,
        -- read data channel
        s_axi_rdata   => axi_rdata,
        s_axi_rresp   => axi_rresp,
        s_axi_rvalid  => axi_rvalid,
        s_axi_rready  => axi_rready
    );

    -- BRAM process, 1-cycle read latency in read-first mode
    bram_process: process(clock)
    begin
        if rising_edge(clock) then
            if bram_en = '1' then
                bram_dout <= ram(to_integer(unsigned(bram_addr)));  -- always, read-first
                if bram_we = '1' then
                    ram(to_integer(unsigned(bram_addr))) := bram_din;
                end if;
            end if;
        end if;
    end process;

    -- Simulation process
    simulation_process: process
        variable status : std_logic_vector(31 downto 0);
    begin
        -- clear inputs

        -- wait for reset
        wait until reset = '1';

        -------------------------------------------------------------------------
        -- Test 1: basic DMA flow, prefill RAM, trigger DMA, look for the
        -- pattern on the output AXI-Stream

        fifo_empty  <= '1';
        axis_tready <= '1';


        -- Pre-fill RAM with known pattern
        for i in 0 to WORD_COUNT-1 loop
            ram(i) := std_logic_vector(to_unsigned(i + 1, 32));
        end loop;

        wait until rising_edge(clock);

        -- Trigger DMA
        axi_lite_write(axi_awaddr, axi_awvalid, axi_wdata, axi_wvalid,
               axi_awready, axi_wready, axi_bvalid, clock, x"00", x"00000001");

        -- Collect and verify AXI-Stream output for the first inputs
        for i in 0 to WORD_COUNT-1 loop
            wait until rising_edge(clock) and axis_tvalid = '1';
            assert axis_tdata = std_logic_vector(to_unsigned(i + 1, 32))
                report "Data mismatch at word " & integer'image(i);
            if i = WORD_COUNT-1 then
                assert axis_tlast = '1' report "tlast not asserted on last word";
            else
                assert axis_tlast = '0' report "tlast asserted too early";
            end if;
        end loop;

        -- Poll status register until dma_done is set (bit 1)
        loop
            axi_lite_read(axi_araddr, axi_arvalid, axi_rdata, axi_arready,
                          axi_rvalid, clock, x"04", status);
            exit when status(1) = '1';
        end loop;
        assert status(1) = '1' report "dma_done not set after transfer";

        -- Verify dma_done is cleared after read (it was just read in the loop)
        axi_lite_read(axi_araddr, axi_arvalid, axi_rdata, axi_arready,
                      axi_rvalid, clock, x"04", status);
        assert status(1) = '0' report "dma_done not cleared after read";

        for i in 0 to WORD_COUNT-1 loop
            assert ram(i) = x"00000000"
                report "RAM not zeroed at word " & integer'image(i)
                    & " got " & integer'image(to_integer(unsigned(ram(i))));
        end loop;

        -------------------------------------------------------------------------
        -- Test 2: trigger clear, verify all RAM locations zeroed

        -- Verify RAM is zeroed via another DMA readout
        axi_lite_write(axi_awaddr, axi_awvalid, axi_wdata, axi_wvalid,
            axi_awready, axi_wready, axi_bvalid, clock, x"00", x"00000001");

        for i in 0 to WORD_COUNT-1 loop
            wait until rising_edge(clock) and axis_tvalid = '1';
            assert axis_tdata = x"00000000"
                report "Clear failed at word " & integer'image(i)
                    & " got " & integer'image(to_integer(unsigned(axis_tdata)));
        end loop;

        wait;
    end process;
end Simulation;