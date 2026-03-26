library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.decoder_constants.all;

entity fuzzsight_tb is
end fuzzsight_tb;

architecture Simulation of fuzzsight_tb is

    --------------------
    -- Constants
    --------------------
    constant ADDR_WIDTH : integer := 8;
    constant AXIL_WIDTH : integer := 8;
    constant AXIS_WIDTH : integer := 32;
    constant MAP_SIZE   : integer := 2**ADDR_WIDTH;
    constant WORD_COUNT : integer := MAP_SIZE / 4;

    --------------------
    -- Types
    --------------------
    type ram_byte_t is array(0 to MAP_SIZE-1) of std_logic_vector(7 downto 0);

    --------------------
    -- Components
    --------------------
    component edge_extractor
        generic (
            FIFO_DEPTH : integer;
            AXIL_WIDTH : integer
        );
        port (
            aclk     : in  std_logic;
            aresetn  : in  std_logic;

            i_atom_valid0     : in std_logic;
            i_address_reg_0_0 : in std_logic_vector(63 downto 0);
            i_atom_elements0  : in std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
            i_atom_nb0        : in unsigned(ATOM_NB_SIZE-1 downto 0);

            i_atom_valid1     : in std_logic;
            i_address_reg_0_1 : in std_logic_vector(63 downto 0);
            i_atom_elements1  : in std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
            i_atom_nb1        : in unsigned(ATOM_NB_SIZE-1 downto 0);

            i_atom_valid2     : in std_logic;
            i_address_reg_0_2 : in std_logic_vector(63 downto 0);
            i_atom_elements2  : in std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
            i_atom_nb2        : in unsigned(ATOM_NB_SIZE-1 downto 0);

            i_atom_valid3     : in std_logic;
            i_address_reg_0_3 : in std_logic_vector(63 downto 0);
            i_atom_elements3  : in std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
            i_atom_nb3        : in unsigned(ATOM_NB_SIZE-1 downto 0);

            o_index   : out std_logic_vector(63 downto 0);
            o_valid   : out std_logic;
            o_empty   : out std_logic;
            i_ready   : in  std_logic;

            s_axi_awaddr  : in  std_logic_vector(AXIL_WIDTH-1 downto 0);
            s_axi_awvalid : in  std_logic;
            s_axi_awready : out std_logic;
            s_axi_wdata   : in  std_logic_vector(31 downto 0);
            s_axi_wvalid  : in  std_logic;
            s_axi_wready  : out std_logic;
            s_axi_bresp   : out std_logic_vector(1 downto 0);
            s_axi_bvalid  : out std_logic;
            s_axi_bready  : in  std_logic;
            s_axi_araddr  : in  std_logic_vector(AXIL_WIDTH-1 downto 0);
            s_axi_arvalid : in  std_logic;
            s_axi_arready : out std_logic;
            s_axi_rdata   : out std_logic_vector(31 downto 0);
            s_axi_rresp   : out std_logic_vector(1 downto 0);
            s_axi_rvalid  : out std_logic;
            s_axi_rready  : in  std_logic
        );
    end component;

    component bitmap_writer_bram
        generic (
            ADDR_WIDTH : integer
        );
        port (
            aclk    : in  std_logic;
            aresetn : in  std_logic;

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

    component bitmap_reader_bram
        generic (
            ADDR_WIDTH : integer;
            AXIL_WIDTH : integer;
            AXIS_WIDTH : integer
        );
        port (
            aclk    : in std_logic;
            aresetn : in std_logic;

            i_fifo_empty      : in  std_logic;
            o_fifo_freeze_req : out std_logic;

            bram_addr : out std_logic_vector(ADDR_WIDTH-1 downto 0);
            bram_din  : out std_logic_vector(31 downto 0);
            bram_dout : in  std_logic_vector(31 downto 0);
            bram_en   : out std_logic;
            bram_we   : out std_logic;

            m_axis_tready : in  std_logic;
            m_axis_tdata  : out std_logic_vector(AXIS_WIDTH-1 downto 0);
            m_axis_tvalid : out std_logic;
            m_axis_tlast  : out std_logic;

            s_axi_awaddr  : in  std_logic_vector(AXIL_WIDTH-1 downto 0);
            s_axi_awvalid : in  std_logic;
            s_axi_awready : out std_logic;
            s_axi_wdata   : in  std_logic_vector(31 downto 0);
            s_axi_wvalid  : in  std_logic;
            s_axi_wready  : out std_logic;
            s_axi_bresp   : out std_logic_vector(1 downto 0);
            s_axi_bvalid  : out std_logic;
            s_axi_bready  : in  std_logic;
            s_axi_araddr  : in  std_logic_vector(AXIL_WIDTH-1 downto 0);
            s_axi_arvalid : in  std_logic;
            s_axi_arready : out std_logic;
            s_axi_rdata   : out std_logic_vector(31 downto 0);
            s_axi_rresp   : out std_logic_vector(1 downto 0);
            s_axi_rvalid  : out std_logic;
            s_axi_rready  : in  std_logic
        );
    end component;

    --------------------
    -- Procedures
    --------------------

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
        awaddr  <= addr;
        awvalid <= '1';
        wdata   <= data;
        wvalid  <= '1';

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

        wait until rising_edge(clock) and bvalid = '1';
    end procedure;

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
        araddr  <= addr;
        arvalid <= '1';

        while not ar_done loop
            wait until rising_edge(clock);
            if arvalid = '1' and arready = '1' then
                arvalid <= '0';
                ar_done := true;
            end if;
        end loop;

        wait until rising_edge(clock) and rvalid = '1';
        data := rdata;
    end procedure;

    --------------------
    -- Signals
    --------------------

    -- Clock and reset
    signal clock  : std_logic := '1';
    signal reset  : std_logic := '0';

    -- Edge extractor inputs
    signal atom_valid0     : std_logic := '0';
    signal address_reg_0_0 : std_logic_vector(63 downto 0) := (others => '0');
    signal atom_elements0  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0) := (others => '0');
    signal atom_nb0        : unsigned(ATOM_NB_SIZE-1 downto 0) := (others => '0');

    signal atom_valid1     : std_logic := '0';
    signal address_reg_0_1 : std_logic_vector(63 downto 0) := (others => '0');
    signal atom_elements1  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0) := (others => '0');
    signal atom_nb1        : unsigned(ATOM_NB_SIZE-1 downto 0) := (others => '0');

    signal atom_valid2     : std_logic := '0';
    signal address_reg_0_2 : std_logic_vector(63 downto 0) := (others => '0');
    signal atom_elements2  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0) := (others => '0');
    signal atom_nb2        : unsigned(ATOM_NB_SIZE-1 downto 0) := (others => '0');

    signal atom_valid3     : std_logic := '0';
    signal address_reg_0_3 : std_logic_vector(63 downto 0) := (others => '0');
    signal atom_elements3  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0) := (others => '0');
    signal atom_nb3        : unsigned(ATOM_NB_SIZE-1 downto 0) := (others => '0');

    -- Edge extractor <-> bitmap writer
    signal edge_index    : std_logic_vector(63 downto 0);
    signal edge_valid    : std_logic;
    signal edge_ready    : std_logic;
    signal edge_empty    : std_logic;

    -- Freeze request: reader -> writer
    signal freeze_req    : std_logic;

    -- BRAM port A (writer, 8-bit)
    signal bram_addr_a   : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal bram_din_a    : std_logic_vector(7 downto 0);
    signal bram_dout_a   : std_logic_vector(7 downto 0);
    signal bram_en_a     : std_logic;
    signal bram_we_a     : std_logic;

    -- BRAM port B (reader, 32-bit)
    signal bram_addr_b   : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal bram_din_b    : std_logic_vector(31 downto 0);
    signal bram_dout_b   : std_logic_vector(31 downto 0);
    signal bram_en_b     : std_logic;
    signal bram_we_b     : std_logic;

    -- AXI-Stream output
    signal axis_tready   : std_logic := '1';
    signal axis_tdata    : std_logic_vector(AXIS_WIDTH-1 downto 0);
    signal axis_tvalid   : std_logic;
    signal axis_tlast    : std_logic;

    -- AXI-Lite for edge extractor
    signal ee_axi_awaddr  : std_logic_vector(AXIL_WIDTH-1 downto 0) := (others => '0');
    signal ee_axi_awvalid : std_logic := '0';
    signal ee_axi_awready : std_logic;
    signal ee_axi_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal ee_axi_wvalid  : std_logic := '0';
    signal ee_axi_wready  : std_logic;
    signal ee_axi_bresp   : std_logic_vector(1 downto 0);
    signal ee_axi_bvalid  : std_logic;
    signal ee_axi_bready  : std_logic := '1';
    signal ee_axi_araddr  : std_logic_vector(AXIL_WIDTH-1 downto 0) := (others => '0');
    signal ee_axi_arvalid : std_logic := '0';
    signal ee_axi_arready : std_logic;
    signal ee_axi_rdata   : std_logic_vector(31 downto 0);
    signal ee_axi_rresp   : std_logic_vector(1 downto 0);
    signal ee_axi_rvalid  : std_logic;
    signal ee_axi_rready  : std_logic := '1';

    -- AXI-Lite for bitmap reader
    signal br_axi_awaddr  : std_logic_vector(AXIL_WIDTH-1 downto 0) := (others => '0');
    signal br_axi_awvalid : std_logic := '0';
    signal br_axi_awready : std_logic;
    signal br_axi_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal br_axi_wvalid  : std_logic := '0';
    signal br_axi_wready  : std_logic;
    signal br_axi_bresp   : std_logic_vector(1 downto 0);
    signal br_axi_bvalid  : std_logic;
    signal br_axi_bready  : std_logic := '1';
    signal br_axi_araddr  : std_logic_vector(AXIL_WIDTH-1 downto 0) := (others => '0');
    signal br_axi_arvalid : std_logic := '0';
    signal br_axi_arready : std_logic;
    signal br_axi_rdata   : std_logic_vector(31 downto 0);
    signal br_axi_rresp   : std_logic_vector(1 downto 0);
    signal br_axi_rvalid  : std_logic;
    signal br_axi_rready  : std_logic := '1';

    -- Shared BRAM backing store
    shared variable bram_byte : ram_byte_t := (others => (others => '0'));

begin

    clock <= not clock after 1 ns;
    reset <= '0', '1' after 6 ns;

    --------------------
    -- DUT instantiations
    --------------------

    EE: edge_extractor
        generic map (
            FIFO_DEPTH => 4,
            AXIL_WIDTH => AXIL_WIDTH
        )
        port map (
            aclk    => clock,
            aresetn => reset,

            i_atom_valid0     => atom_valid0,
            i_address_reg_0_0 => address_reg_0_0,
            i_atom_elements0  => atom_elements0,
            i_atom_nb0        => atom_nb0,

            i_atom_valid1     => atom_valid1,
            i_address_reg_0_1 => address_reg_0_1,
            i_atom_elements1  => atom_elements1,
            i_atom_nb1        => atom_nb1,

            i_atom_valid2     => atom_valid2,
            i_address_reg_0_2 => address_reg_0_2,
            i_atom_elements2  => atom_elements2,
            i_atom_nb2        => atom_nb2,

            i_atom_valid3     => atom_valid3,
            i_address_reg_0_3 => address_reg_0_3,
            i_atom_elements3  => atom_elements3,
            i_atom_nb3        => atom_nb3,

            o_index => edge_index,
            o_valid => edge_valid,
            o_empty => edge_empty,
            i_ready => edge_ready,

            s_axi_awaddr  => ee_axi_awaddr,
            s_axi_awvalid => ee_axi_awvalid,
            s_axi_awready => ee_axi_awready,
            s_axi_wdata   => ee_axi_wdata,
            s_axi_wvalid  => ee_axi_wvalid,
            s_axi_wready  => ee_axi_wready,
            s_axi_bresp   => ee_axi_bresp,
            s_axi_bvalid  => ee_axi_bvalid,
            s_axi_bready  => ee_axi_bready,
            s_axi_araddr  => ee_axi_araddr,
            s_axi_arvalid => ee_axi_arvalid,
            s_axi_arready => ee_axi_arready,
            s_axi_rdata   => ee_axi_rdata,
            s_axi_rresp   => ee_axi_rresp,
            s_axi_rvalid  => ee_axi_rvalid,
            s_axi_rready  => ee_axi_rready
        );

    BW: bitmap_writer_bram
        generic map (
            ADDR_WIDTH => ADDR_WIDTH
        )
        port map (
            aclk    => clock,
            aresetn => reset,

            i_fifo_index      => edge_index,
            i_fifo_valid      => edge_valid,
            o_fifo_ready      => edge_ready,
            i_fifo_freeze_req => freeze_req,

            bram_addr => bram_addr_a,
            bram_din  => bram_din_a,
            bram_dout => bram_dout_a,
            bram_en   => bram_en_a,
            bram_we   => bram_we_a
        );

    BR: bitmap_reader_bram
        generic map (
            ADDR_WIDTH => ADDR_WIDTH,
            AXIL_WIDTH => AXIL_WIDTH,
            AXIS_WIDTH => AXIS_WIDTH
        )
        port map (
            aclk    => clock,
            aresetn => reset,

            i_fifo_empty      => edge_empty,
            o_fifo_freeze_req => freeze_req,

            bram_addr => bram_addr_b,
            bram_din  => bram_din_b,
            bram_dout => bram_dout_b,
            bram_en   => bram_en_b,
            bram_we   => bram_we_b,

            m_axis_tready => axis_tready,
            m_axis_tdata  => axis_tdata,
            m_axis_tvalid => axis_tvalid,
            m_axis_tlast  => axis_tlast,

            s_axi_awaddr  => br_axi_awaddr,
            s_axi_awvalid => br_axi_awvalid,
            s_axi_awready => br_axi_awready,
            s_axi_wdata   => br_axi_wdata,
            s_axi_wvalid  => br_axi_wvalid,
            s_axi_wready  => br_axi_wready,
            s_axi_bresp   => br_axi_bresp,
            s_axi_bvalid  => br_axi_bvalid,
            s_axi_bready  => br_axi_bready,
            s_axi_araddr  => br_axi_araddr,
            s_axi_arvalid => br_axi_arvalid,
            s_axi_arready => br_axi_arready,
            s_axi_rdata   => br_axi_rdata,
            s_axi_rresp   => br_axi_rresp,
            s_axi_rvalid  => br_axi_rvalid,
            s_axi_rready  => br_axi_rready
        );

    --------------------
    -- BRAM model
    --------------------

    -- Port A: 8-bit writer port
    bram_port_a: process(clock)
    begin
        if rising_edge(clock) then
            if bram_en_a = '1' then
                bram_dout_a <= bram_byte(to_integer(unsigned(bram_addr_a)));
                if bram_we_a = '1' then
                    bram_byte(to_integer(unsigned(bram_addr_a))) := bram_din_a;
                end if;
            end if;
        end if;
    end process;

    -- Port B: 32-bit reader port, addr indexes 4-byte words
    bram_port_b: process(clock)
    begin
        if rising_edge(clock) then
            if bram_en_b = '1' then
                bram_dout_b <= bram_byte(to_integer(unsigned(bram_addr_b)) * 4 + 3)
                             & bram_byte(to_integer(unsigned(bram_addr_b)) * 4 + 2)
                             & bram_byte(to_integer(unsigned(bram_addr_b)) * 4 + 1)
                             & bram_byte(to_integer(unsigned(bram_addr_b)) * 4 + 0);
                if bram_we_b = '1' then
                    bram_byte(to_integer(unsigned(bram_addr_b)) * 4 + 3) := bram_din_b(31 downto 24);
                    bram_byte(to_integer(unsigned(bram_addr_b)) * 4 + 2) := bram_din_b(23 downto 16);
                    bram_byte(to_integer(unsigned(bram_addr_b)) * 4 + 1) := bram_din_b(15 downto  8);
                    bram_byte(to_integer(unsigned(bram_addr_b)) * 4 + 0) := bram_din_b( 7 downto  0);
                end if;
            end if;
        end if;
    end process;

    --------------------
    -- Simulation
    --------------------

    simulation_process: process
        variable status      : std_logic_vector(31 downto 0);
        variable dma_data    : std_logic_vector(AXIS_WIDTH-1 downto 0);
        variable non_zero    : boolean;
    begin
        wait until reset = '1';
        wait until rising_edge(clock);

        -----------------------------------------------------------------------
        -- Test 1: send some atom packets, let the writer fill the bitmap,
        -- then trigger a DMA readout and verify at least some words are non-zero

        -- Port 0: single E atom at address 0x100
        atom_valid0     <= '1';
        address_reg_0_0 <= x"0000000000000100";
        atom_elements0  <= (others => '1');  -- all E, no N atoms
        atom_nb0        <= "00001";
        wait until rising_edge(clock);
        atom_valid0 <= '0';

        -- Port 0: single E atom at address 0x200
        atom_valid0     <= '1';
        address_reg_0_0 <= x"0000000000000200";
        atom_elements0  <= (others => '1');
        atom_nb0        <= "00001";
        wait until rising_edge(clock);
        atom_valid0 <= '0';

        -- Port 0 and 1 simultaneously
        atom_valid0     <= '1';
        address_reg_0_0 <= x"0000000000000300";
        atom_elements0  <= (others => '1');
        atom_nb0        <= "00001";

        atom_valid1     <= '1';
        address_reg_0_1 <= x"0000000000000400";
        atom_elements1  <= (others => '1');
        atom_nb1        <= "00001";
        wait until rising_edge(clock);
        atom_valid0 <= '0';
        atom_valid1 <= '0';

        -- Wait a few cycles for the writer to process and write to BRAM
        for i in 0 to 7 loop
            wait until rising_edge(clock);
        end loop;

        -----------------------------------------------------------------------
        -- Test 2: trigger DMA readout, verify non-zero words in bitmap

        -- Poll until edge extractor FIFO is empty (safe to DMA)
        loop
            axi_lite_read(br_axi_araddr, br_axi_arvalid, br_axi_rdata, br_axi_arready,
                          br_axi_rvalid, clock, x"04", status);
            exit when status(0) = '1';  -- i_fifo_empty
        end loop;

        -- Trigger DMA
        axi_lite_write(br_axi_awaddr, br_axi_awvalid, br_axi_wdata, br_axi_wvalid,
                       br_axi_awready, br_axi_wready, br_axi_bvalid, clock,
                       x"00", x"00000001");

        -- Collect DMA output, check at least one word is non-zero
        non_zero := false;
        for i in 0 to WORD_COUNT-1 loop
            wait until rising_edge(clock) and axis_tvalid = '1';
            if axis_tdata /= x"00000000" then
                non_zero := true;
                report "Non-zero word at index " & integer'image(i)
                     & " = 0x" & integer'image(to_integer(unsigned(axis_tdata)));
            end if;
            if i = WORD_COUNT-1 then
                assert axis_tlast = '1' report "tlast not asserted on last word";
            end if;
        end loop;
        assert non_zero report "All DMA words are zero - bitmap was not written";

        -- Poll until dma_done
        loop
            axi_lite_read(br_axi_araddr, br_axi_arvalid, br_axi_rdata, br_axi_arready,
                          br_axi_rvalid, clock, x"04", status);
            exit when status(1) = '1';  -- dma_done
        end loop;
        report "DMA complete";

        -----------------------------------------------------------------------
        -- Test 3: second DMA readout should show bitmap cleared by first DMA

        -- Wait for freeze to deassert and FIFO to drain
        loop
            axi_lite_read(br_axi_araddr, br_axi_arvalid, br_axi_rdata, br_axi_arready,
                          br_axi_rvalid, clock, x"04", status);
            exit when status(0) = '1';
        end loop;

        -- Trigger second DMA
        axi_lite_write(br_axi_awaddr, br_axi_awvalid, br_axi_wdata, br_axi_wvalid,
                       br_axi_awready, br_axi_wready, br_axi_bvalid, clock,
                       x"00", x"00000001");

        for i in 0 to WORD_COUNT-1 loop
            wait until rising_edge(clock) and axis_tvalid = '1';
            assert axis_tdata = x"00000000"
                report "Bitmap not cleared at word " & integer'image(i)
                     & " got 0x" & integer'image(to_integer(unsigned(axis_tdata)));
        end loop;
        report "Bitmap correctly cleared by read-back DMA";

        -----------------------------------------------------------------------
        -- Test 4: read edge extractor stats

        axi_lite_read(ee_axi_araddr, ee_axi_arvalid, ee_axi_rdata, ee_axi_arready,
                      ee_axi_rvalid, clock, x"00", status);
        report "edges_total = " & integer'image(to_integer(unsigned(status)));
        assert unsigned(status) = 4 report "Expected 4 edges total";

        axi_lite_read(ee_axi_araddr, ee_axi_arvalid, ee_axi_rdata, ee_axi_arready,
                      ee_axi_rvalid, clock, x"04", status);
        report "fifo_overflow_count = " & integer'image(to_integer(unsigned(status)));
        assert unsigned(status) = 0 report "Expected no overflows";

        report "Integration test complete";
        wait;
    end process;

end Simulation;