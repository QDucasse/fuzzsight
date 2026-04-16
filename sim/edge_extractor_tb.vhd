library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.decoder_constants.all;


entity edge_extractor_tb is
-- Port ( );
end edge_extractor_tb;

architecture Simulation of edge_extractor_tb is

    --------------------
    -- Constants
    --------------------

    constant ATOM_ELTS_SIZE : integer := 24;
    constant ATOM_NB_SIZE   : integer := 5;
    constant AXIL_WIDTH     : integer := 8;
    constant FIFO_DEPTH     : integer := 4;

    --------------------
    -- Component
    --------------------

    component edge_extractor
    generic (
        FIFO_DEPTH: integer;
        AXIL_WIDTH: integer
    );
    port (
        aclk     : in  std_logic;
        aresetn  : in  std_logic;

        -- Inputs from ETM decoder (4 sequential ports)
        i_atom_valid0         : in std_logic;
        i_address_reg_0_0     : in std_logic_vector(63 downto 0);
        i_atom_elements0      : in std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
        i_atom_nb0            : in unsigned(ATOM_NB_SIZE-1 downto 0);
        i_exception_valid0    : in std_logic;
        i_exception_type0     : in std_logic_vector(EXC_TYPE_SIZE-1 downto 0);
        i_exception_pending0  : in std_logic;
        i_pre_exception_addr0 : in std_logic_vector(63 downto 0);

        i_atom_valid1         : in std_logic;
        i_address_reg_0_1     : in std_logic_vector(63 downto 0);
        i_atom_elements1      : in std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
        i_atom_nb1            : in unsigned(ATOM_NB_SIZE-1 downto 0);
        i_exception_valid1    : in std_logic;
        i_exception_type1     : in std_logic_vector(EXC_TYPE_SIZE-1 downto 0);
        i_exception_pending1  : in std_logic;
        i_pre_exception_addr1 : in std_logic_vector(63 downto 0);

        i_atom_valid2         : in std_logic;
        i_address_reg_0_2     : in std_logic_vector(63 downto 0);
        i_atom_elements2      : in std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
        i_atom_nb2            : in unsigned(ATOM_NB_SIZE-1 downto 0);
        i_exception_valid2    : in std_logic;
        i_exception_type2     : in std_logic_vector(EXC_TYPE_SIZE-1 downto 0);
        i_exception_pending2  : in std_logic;
        i_pre_exception_addr2 : in std_logic_vector(63 downto 0);

        i_atom_valid3         : in std_logic;
        i_address_reg_0_3     : in std_logic_vector(63 downto 0);
        i_atom_elements3      : in std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
        i_atom_nb3            : in unsigned(ATOM_NB_SIZE-1 downto 0);
        i_exception_valid3    : in std_logic;
        i_exception_type3     : in std_logic_vector(EXC_TYPE_SIZE-1 downto 0);
        i_exception_pending3  : in std_logic;
        i_pre_exception_addr3 : in std_logic_vector(63 downto 0);

        -- Freeze request
        i_freeze_request  : in std_logic;

        -- Output edge index
        o_index   : out std_logic_vector(63 downto 0);

        -- Output fifo info
        o_valid   : out std_logic;
        o_empty   : out std_logic;
        i_ready   : in  std_logic;

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

    -- Decoder outputs
    signal atom_valid0     : std_logic;
    signal address_reg_0_0 : std_logic_vector(63 downto 0);
    signal atom_elements0  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal atom_nb0        : unsigned(ATOM_NB_SIZE-1 downto 0);
    signal exc_valid0      : std_logic;
    signal exc_type0       : std_logic_vector(4 downto 0);
    signal exc_pending0    : std_logic;
    signal pre_exc_addr0   : std_logic_vector(63 downto 0);

    signal atom_valid1     : std_logic;
    signal address_reg_0_1 : std_logic_vector(63 downto 0);
    signal atom_elements1  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal atom_nb1        : unsigned(ATOM_NB_SIZE-1 downto 0);
    signal exc_valid1      : std_logic;
    signal exc_type1       : std_logic_vector(4 downto 0);
    signal exc_pending1    : std_logic;
    signal pre_exc_addr1   : std_logic_vector(63 downto 0);

    signal atom_valid2     : std_logic;
    signal address_reg_0_2 : std_logic_vector(63 downto 0);
    signal atom_elements2  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal atom_nb2        : unsigned(ATOM_NB_SIZE-1 downto 0);
    signal exc_valid2      : std_logic;
    signal exc_type2       : std_logic_vector(4 downto 0);
    signal exc_pending2    : std_logic;
    signal pre_exc_addr2   : std_logic_vector(63 downto 0);

    signal atom_valid3     : std_logic;
    signal address_reg_0_3 : std_logic_vector(63 downto 0);
    signal atom_elements3  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal atom_nb3        : unsigned(ATOM_NB_SIZE-1 downto 0);
    signal exc_valid3      : std_logic;
    signal exc_type3       : std_logic_vector(4 downto 0);
    signal exc_pending3    : std_logic;
    signal pre_exc_addr3   : std_logic_vector(63 downto 0);

    -- Freeze request
    signal freeze_request  : std_logic := '0';

    -- FIFO handshake
    signal index : std_logic_vector(63 downto 0);
    signal valid : std_logic;
    signal empty : std_logic;
    signal ready : std_logic;

        -- AXI4-Lite signals
    signal axi_awaddr  : std_logic_vector(AXIL_WIDTH-1 downto 0) := (others=>'0');
    signal axi_awvalid : std_logic := '0';
    signal axi_awready : std_logic := '0';
    signal axi_wdata   : std_logic_vector(31 downto 0) := (others=>'0');
    signal axi_wvalid  : std_logic := '0';
    signal axi_wready  : std_logic := '0';
    signal axi_bresp   : std_logic_vector(1 downto 0);
    signal axi_bvalid  : std_logic := '0';
    signal axi_bready  : std_logic := '1';
    signal axi_araddr  : std_logic_vector(AXIL_WIDTH-1 downto 0) := (others=>'0');
    signal axi_arvalid : std_logic := '0';
    signal axi_arready : std_logic := '0';
    signal axi_rdata   : std_logic_vector(31 downto 0) := (others=>'0');
    signal axi_rresp   : std_logic_vector(1 downto 0);
    signal axi_rvalid  : std_logic := '0';
    signal axi_rready  : std_logic := '1';
begin
    -- Clock and reset
    clock <= not clock after 1 ns;
    reset <= '0', '1' after 6 ns;

    -- DUT instantiation WITH SIGNALS
    DUT: edge_extractor
        generic map (
            FIFO_DEPTH => FIFO_DEPTH,
            AXIL_WIDTH => AXIL_WIDTH
        ) port map (
        aclk     => clock,
        aresetn  => reset,

        -- Inputs from ETM decoder (4 sequential ports)
        i_atom_valid0         => atom_valid0,
        i_address_reg_0_0     => address_reg_0_0,
        i_atom_elements0      => atom_elements0,
        i_atom_nb0            => atom_nb0,
        i_exception_valid0    => exc_valid0,
        i_exception_type0     => exc_type0,
        i_exception_pending0  => exc_pending0,
        i_pre_exception_addr0 => pre_exc_addr0,

        i_atom_valid1         => atom_valid1,
        i_address_reg_0_1     => address_reg_0_1,
        i_atom_elements1      => atom_elements1,
        i_atom_nb1            => atom_nb1,
        i_exception_valid1    => exc_valid1,
        i_exception_type1     => exc_type1,
        i_exception_pending1  => exc_pending1,
        i_pre_exception_addr1 => pre_exc_addr1,

        i_atom_valid2         => atom_valid2,
        i_address_reg_0_2     => address_reg_0_2,
        i_atom_elements2      => atom_elements2,
        i_atom_nb2            => atom_nb2,
        i_exception_valid2    => exc_valid2,
        i_exception_type2     => exc_type2,
        i_exception_pending2  => exc_pending2,
        i_pre_exception_addr2 => pre_exc_addr2,

        i_atom_valid3         => atom_valid3,
        i_address_reg_0_3     => address_reg_0_3,
        i_atom_elements3      => atom_elements3,
        i_atom_nb3            => atom_nb3,
        i_exception_valid3    => exc_valid3,
        i_exception_type3     => exc_type3,
        i_exception_pending3  => exc_pending3,
        i_pre_exception_addr3 => pre_exc_addr3,

        -- Freeze request
        i_freeze_request  => freeze_request,

        -- Output edge index
        o_index => index,

        -- Output fifo info
        o_valid => valid,
        o_empty => empty,
        i_ready => ready,

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

    -- Simulation process
    simulation_process: process
        variable status : std_logic_vector(31 downto 0);
    begin

        -- clear inputs
        atom_valid0      <= '0';
        address_reg_0_0  <= (others => '0');
        atom_elements0   <= (others => '0');
        atom_nb0         <= (others => '0');

        atom_valid1      <= '0';
        address_reg_0_1  <= (others => '0');
        atom_elements1   <= (others => '0');
        atom_nb1         <= (others => '0');

        atom_valid2      <= '0';
        address_reg_0_2  <= (others => '0');
        atom_elements2   <= (others => '0');
        atom_nb2         <= (others => '0');

        atom_valid3      <= '0';
        address_reg_0_3  <= (others => '0');
        atom_elements3   <= (others => '0');
        atom_nb3         <= (others => '0');

        freeze_request   <= '0';
        ready <= '0';

        -- wait for reset
        wait until reset = '1';
        wait until rising_edge(clock);

        -----------------------------------------------------------------------
        -- Test 1: Normal operations
        -- Hold ready high and let several atom come

        ready <= '1';

        -- Single atom packet on port 0
        atom_valid0     <= '1';
        address_reg_0_0 <= x"0000000000000100";
        atom_elements0  <= "000000000000000000000001"; -- first atom is E
        atom_nb0        <= "00001";                    -- 1 element
        ready           <= '1';
        wait until rising_edge(clock);
        atom_valid0 <= '0';

        -- Simultaneous atoms on ports 1 and 2
        atom_valid1     <= '1';
        address_reg_0_1 <= x"0000000000000200";
        atom_elements1  <= "000000000000000000000001";
        atom_nb1        <= "00010";

        atom_valid2     <= '1';
        address_reg_0_2 <= x"0000000000000300";
        atom_elements2  <= "000000000000000000000101";
        atom_nb2        <= "00010";

        wait until rising_edge(clock);
        atom_valid1 <= '0';
        atom_valid2 <= '0';

        -- Wait for existing edges to drain
        wait until rising_edge(clock);
        wait until rising_edge(clock);
        wait until rising_edge(clock);

        -- After the drain waits in the normal operations test:
        axi_lite_read(axi_araddr, axi_arvalid, axi_rdata, axi_arready,
                    axi_rvalid, clock, x"04", status);
        report "edges_total after normal ops = " & integer'image(to_integer(unsigned(status)));
        assert unsigned(status) = 3
            report "Expected 3 edges: 1 from port0, 2 from simultaneous ports 1+2" severity error;


        -----------------------------------------------------------------------
        -- Test: freeze_drop counter

        freeze_request <= '1';
        ready          <= '1';

        for i in 0 to 3 loop
            atom_valid0     <= '1';
            address_reg_0_0 <= std_logic_vector(to_unsigned(i * 16#100#, 64));
            atom_elements0  <= "000000000000000000000001";
            atom_nb0        <= "00011";
            wait until rising_edge(clock);
        end loop;
        atom_valid0    <= '0';
        freeze_request <= '0';

        wait until rising_edge(clock);

        -- Read freeze_drop_count (0x0C)
        axi_lite_read(axi_araddr, axi_arvalid, axi_rdata, axi_arready,
                    axi_rvalid, clock, x"0C", status);

        report "freeze_drop_count = " & integer'image(to_integer(unsigned(status)));
        assert unsigned(status) = 4 report "Expected 4 freeze drops";

        -- overflow should be zero since FIFO was empty and ready was high
        axi_lite_read(axi_araddr, axi_arvalid, axi_rdata, axi_arready,
                    axi_rvalid, clock, x"08", status);
        assert status = x"00000000" report "overflow_count should be zero during freeze test";


        -----------------------------------------------------------------------
        -- Test: overflow the FIFO
        -- Hold ready low and fire all 4 ports for more cycles than FIFO_DEPTH

        -- Reset stats leftover from freeze test
        axi_lite_write(axi_awaddr, axi_awvalid, axi_wdata, axi_wvalid,
               axi_awready, axi_wready, axi_bvalid, clock,
               x"00", x"00000001");

        ready <= '0';

        for i in 0 to FIFO_DEPTH loop  -- one extra to guarantee overflow
            atom_valid0     <= '1';
            address_reg_0_0 <= std_logic_vector(to_unsigned(i * 16#100#, 64));
            atom_elements0  <= "000000000000000000000001";  -- NNE
            atom_nb0        <= "00011";

            atom_valid1     <= '1';
            address_reg_0_1 <= std_logic_vector(to_unsigned(i * 16#200#, 64));
            atom_elements1  <= "000000000000000000000001";
            atom_nb1        <= "00011";

            atom_valid2     <= '1';
            address_reg_0_2 <= std_logic_vector(to_unsigned(i * 16#300#, 64));
            atom_elements2  <= "000000000000000000000001";
            atom_nb2        <= "00011";

            atom_valid3     <= '1';
            address_reg_0_3 <= std_logic_vector(to_unsigned(i * 16#400#, 64));
            atom_elements3  <= "000000000000000000000001";
            atom_nb3        <= "00011";

            wait until rising_edge(clock);
        end loop;

        atom_valid0 <= '0';
        atom_valid1 <= '0';
        atom_valid2 <= '0';
        atom_valid3 <= '0';

        wait until rising_edge(clock);

        --------------
        -- Read stats

        -- Read edges_total (0x04)
        axi_lite_read(axi_araddr, axi_arvalid, axi_rdata, axi_arready,
                        axi_rvalid, clock, x"04", status);
        report "edges_total = " & integer'image(to_integer(unsigned(status)));
        assert unsigned(status) > 0 report "edges_total should be non-zero";

        -- Read fifo_overflow_count (0x08)
        axi_lite_read(axi_araddr, axi_arvalid, axi_rdata, axi_arready,
                        axi_rvalid, clock, x"08", status);
        report "fifo_overflow_count = " & integer'image(to_integer(unsigned(status)));
        assert unsigned(status) > 0 report "overflow should have occurred";

        -- Reset stats (0x00, bit 0)
        axi_lite_write(axi_awaddr, axi_awvalid, axi_wdata, axi_wvalid,
                        axi_awready, axi_wready, axi_bvalid, clock,
                        x"00", x"00000001");

        -- Verify both counters cleared
        axi_lite_read(axi_araddr, axi_arvalid, axi_rdata, axi_arready,
                        axi_rvalid, clock, x"04", status);
        assert status = x"00000000" report "edges_total not cleared after reset";

        axi_lite_read(axi_araddr, axi_arvalid, axi_rdata, axi_arready,
                        axi_rvalid, clock, x"08", status);
        assert status = x"00000000" report "fifo_overflow_count not cleared after reset";

        -----------------------------------------------------------------------
        -- Test: prev_slice reset
        -- Fire an atom, record the edge index, reset prev_slice, fire the
        -- same atom again - should produce the same index as the first time

        ready <= '1';

        -- First atom
        atom_valid0     <= '1';
        address_reg_0_0 <= x"0000000000000ABC";
        atom_elements0  <= "000000000000000000000001";
        atom_nb0        <= "00001";
        wait until rising_edge(clock);
        atom_valid0 <= '0';

        -- Capture the first edge index
        wait until rising_edge(clock) and valid = '1';
        report "first edge index = " & to_hstring(to_bitvector(index));

        -- Fire a second atom to advance prev_slice
        atom_valid0     <= '1';
        address_reg_0_0 <= x"0000000000000DEF";
        atom_elements0  <= "000000000000000000000001";
        atom_nb0        <= "00001";
        wait until rising_edge(clock);
        atom_valid0 <= '0';

        wait until rising_edge(clock);

        -- Reset prev_slice by setting freeze request
        freeze_request <= '1';
        wait until rising_edge(clock);

        freeze_request <= '0';
        wait until rising_edge(clock);

        -- Fire the same first atom again - should produce the same index as the first
        atom_valid0     <= '1';
        address_reg_0_0 <= x"0000000000000ABC";
        atom_elements0  <= "000000000000000000000001";
        atom_nb0        <= "00001";
        wait until rising_edge(clock);
        atom_valid0 <= '0';

        -- Capture and compare
        wait until rising_edge(clock) and valid = '1';
        report "post-reset edge index = " & to_hstring(to_bitvector(index));
        assert index = x"0000000000000ABC"
            report "prev_slice reset failed: index should match first-ever edge" severity error;


        wait;
    end process;
end Simulation;