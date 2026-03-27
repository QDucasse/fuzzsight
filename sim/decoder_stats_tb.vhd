library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity decoder_stats_tb is
-- Port ( );
end decoder_stats_tb;

architecture Simulation of decoder_stats_tb is

    -- -----------------
    -- Constants
    -- -----------------

    -- decoder
    constant ATOM_ELTS_SIZE : integer := 24;
    constant ATOM_NB_SIZE : integer := 5;
    -- AXI4-Lite interface
    constant DATA_WIDTH : integer := 64;
    constant ADDR_WIDTH : integer := 8;

    -- Component definition
    component decoder_stats_lut
    port (
        aclk     : in  std_logic;
        aresetn  : in  std_logic;

        -- CoreSight decoder interface
        -- Inputs from ETM decoder (4 sequential ports)
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

        -- Outputs for passthrough
        o_atom_valid0     : out std_logic;
        o_address_reg_0_0 : out std_logic_vector(63 downto 0);
        o_atom_elements0  : out std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
        o_atom_nb0        : out unsigned(ATOM_NB_SIZE-1 downto 0);

        o_atom_valid1     : out std_logic;
        o_address_reg_0_1 : out std_logic_vector(63 downto 0);
        o_atom_elements1  : out std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
        o_atom_nb1        : out unsigned(ATOM_NB_SIZE-1 downto 0);

        o_atom_valid2     : out std_logic;
        o_address_reg_0_2 : out std_logic_vector(63 downto 0);
        o_atom_elements2  : out std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
        o_atom_nb2        : out unsigned(ATOM_NB_SIZE-1 downto 0);

        o_atom_valid3     : out std_logic;
        o_address_reg_0_3 : out std_logic_vector(63 downto 0);
        o_atom_elements3  : out std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
        o_atom_nb3        : out unsigned(ATOM_NB_SIZE-1 downto 0);

        -- AXI4-Lite interface
        -- write address channel
        s_axi_awaddr  : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
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
        s_axi_araddr  : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
        s_axi_arvalid : in  std_logic;
        s_axi_arready : out std_logic;
        -- read data channel
        s_axi_rdata   : out std_logic_vector(31 downto 0);
        s_axi_rresp   : out std_logic_vector(1 downto 0);
        s_axi_rvalid  : out std_logic;
        s_axi_rready  : in  std_logic
    );
    end component;

    -- -----------------
    -- Procedures
    -- -----------------

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

    signal atom_valid1     : std_logic;
    signal address_reg_0_1 : std_logic_vector(63 downto 0);
    signal atom_elements1  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal atom_nb1        : unsigned(ATOM_NB_SIZE-1 downto 0);

    signal atom_valid2     : std_logic;
    signal address_reg_0_2 : std_logic_vector(63 downto 0);
    signal atom_elements2  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal atom_nb2        : unsigned(ATOM_NB_SIZE-1 downto 0);

    signal atom_valid3     : std_logic;
    signal address_reg_0_3 : std_logic_vector(63 downto 0);
    signal atom_elements3  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal atom_nb3        : unsigned(ATOM_NB_SIZE-1 downto 0);

    -- outs
    signal o_atom_valid0     : std_logic;
    signal o_address_reg_0_0 : std_logic_vector(63 downto 0);
    signal o_atom_elements0  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal o_atom_nb0        : unsigned(ATOM_NB_SIZE-1 downto 0);

    signal o_atom_valid1     : std_logic;
    signal o_address_reg_0_1 : std_logic_vector(63 downto 0);
    signal o_atom_elements1  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal o_atom_nb1        : unsigned(ATOM_NB_SIZE-1 downto 0);

    signal o_atom_valid2     : std_logic;
    signal o_address_reg_0_2 : std_logic_vector(63 downto 0);
    signal o_atom_elements2  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal o_atom_nb2        : unsigned(ATOM_NB_SIZE-1 downto 0);

    signal o_atom_valid3     : std_logic;
    signal o_address_reg_0_3 : std_logic_vector(63 downto 0);
    signal o_atom_elements3  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal o_atom_nb3        : unsigned(ATOM_NB_SIZE-1 downto 0);

        -- AXI-Lite
    signal s_axi_awaddr  : std_logic_vector(ADDR_WIDTH-1 downto 0) := (others=>'0');
    signal s_axi_awvalid : std_logic := '0';
    signal s_axi_awready : std_logic;

    signal s_axi_wdata   : std_logic_vector(31 downto 0) := (others=>'0');
    signal s_axi_wvalid  : std_logic := '0';
    signal s_axi_wready  : std_logic;

    signal s_axi_bresp   : std_logic_vector(1 downto 0);
    signal s_axi_bvalid  : std_logic;
    signal s_axi_bready  : std_logic := '1';

    signal s_axi_araddr  : std_logic_vector(ADDR_WIDTH-1 downto 0) := (others=>'0');
    signal s_axi_arvalid : std_logic := '0';
    signal s_axi_arready : std_logic;

    signal s_axi_rdata   : std_logic_vector(31 downto 0);
    signal s_axi_rresp   : std_logic_vector(1 downto 0);
    signal s_axi_rvalid  : std_logic;
    signal s_axi_rready  : std_logic := '1';

begin
    -- Clock and reset
    clock <= not clock after 1 ns;
    reset <= '0', '1' after 6 ns;

    -- DUT instantiation WITH SIGNALS
    DUT: decoder_stats_lut port map (
        aclk     => clock,
        aresetn  => reset,

        -- Inputs from ETM decoder (4 sequential ports)
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

        -- Outputs for passthrough
        o_atom_valid0     => o_atom_valid0,
        o_address_reg_0_0 => o_address_reg_0_0,
        o_atom_elements0  => o_atom_elements0,
        o_atom_nb0        => o_atom_nb0,

        o_atom_valid1     => o_atom_valid1,
        o_address_reg_0_1 => o_address_reg_0_1,
        o_atom_elements1  => o_atom_elements1,
        o_atom_nb1        => o_atom_nb1,

        o_atom_valid2     => o_atom_valid2,
        o_address_reg_0_2 => o_address_reg_0_2,
        o_atom_elements2  => o_atom_elements2,
        o_atom_nb2        => o_atom_nb2,

        o_atom_valid3     => o_atom_valid3,
        o_address_reg_0_3 => o_address_reg_0_3,
        o_atom_elements3  => o_atom_elements3,
        o_atom_nb3        => o_atom_nb3,

        -- AXI4-Lite interface
        s_axi_awaddr  => s_axi_awaddr,
        s_axi_awvalid => s_axi_awvalid,
        s_axi_awready => s_axi_awready,

        s_axi_wdata   => s_axi_wdata,
        s_axi_wvalid  => s_axi_wvalid,
        s_axi_wready  => s_axi_wready,

        s_axi_bresp   => s_axi_bresp,
        s_axi_bvalid  => s_axi_bvalid,
        s_axi_bready  => s_axi_bready,

        s_axi_araddr  => s_axi_araddr,
        s_axi_arvalid => s_axi_arvalid,
        s_axi_arready => s_axi_arready,

        s_axi_rdata   => s_axi_rdata,
        s_axi_rresp   => s_axi_rresp,
        s_axi_rvalid  => s_axi_rvalid,
        s_axi_rready  => s_axi_rready
    );

    -- Simulation process
    simulation_process: process
        --------------------
        -- Helpers
        --------------------

        -- Helper to reset stats between tests
        procedure reset_stats is
        begin
            axi_lite_write(s_axi_awaddr, s_axi_awvalid, s_axi_wdata, s_axi_wvalid,
                            s_axi_awready, s_axi_wready, s_axi_bvalid, clock,
                            x"00", x"00000002");
            wait until rising_edge(clock);
            wait until rising_edge(clock);
        end procedure;

        -- Helper to check the value of a stat
        procedure check(name : string; got : std_logic_vector(31 downto 0); expected : integer) is
        begin
            assert to_integer(unsigned(got)) = expected
                report name & ": expected " & integer'image(expected)
                           & " got " & integer'image(to_integer(unsigned(got)))
                severity error;
        end procedure;

        -- Helper to enable/disable stats
        procedure set_stats_en(en : std_logic) is
        begin
            if en = '1' then
                axi_lite_write(s_axi_awaddr, s_axi_awvalid, s_axi_wdata, s_axi_wvalid,
                               s_axi_awready, s_axi_wready, s_axi_bvalid, clock,
                               x"00", x"00000001");
            else
                axi_lite_write(s_axi_awaddr, s_axi_awvalid, s_axi_wdata, s_axi_wvalid,
                               s_axi_awready, s_axi_wready, s_axi_bvalid, clock,
                               x"00", x"00000000");
            end if;
            -- wait until rising_edge(clock);
        end procedure;

        --------------------
        -- Variables
        --------------------
        variable v : std_logic_vector(31 downto 0);

    begin
        -- wait for reset
        wait until reset = '1';

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

        -----------------------------------------------------------------------
        -- Test 1: Single atom on port 0 only
        -- "1000" for one cycle
        -- Expected:
        --   atom_count=1, atom_burst_count=1,
        --   max_atom_burst=1 (still open), ongoing=0
        -- Note: burst only closes when a gap follows
        -----------------------------------------------------------------------
        report "Test 1: Single atom port 0";
        set_stats_en('1');

        atom_valid0 <= '1'; atom_nb0 <= "00001"; atom_elements0 <= (0 => '1', others => '0');
        wait until rising_edge(clock);
        atom_valid0 <= '0';

        -- gap cycle to close the burst
        wait until rising_edge(clock);

        set_stats_en('0');

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"30", v);
        check("T1 atom_count", v, 1);

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"34", v);
        check("T1 atom_burst_count", v, 1);

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"3C", v);
        check("T1 max_atom_burst_length", v, 1);

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"38", v);
        check("T1 min_atom_burst_length", v, 1);

        reset_stats;

        -----------------------------------------------------------------------
        -- Test 2: Two consecutive cycles, port 0 only
        -- "1000" "1000" then gap
        -- Expected: atom_count=2, burst_count=1, max_burst=1
        -----------------------------------------------------------------------
        report "Test 2: Two consecutive atoms port 0";
        set_stats_en('1');

        atom_valid0 <= '1'; atom_nb0 <= "00001"; atom_elements0 <= (0 => '1', others => '0');
        wait until rising_edge(clock);
        wait until rising_edge(clock);
        atom_valid0 <= '0';

        -- gap to close burst
        wait until rising_edge(clock);


        set_stats_en('0');

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"30", v);
        check("T2 atom_count", v, 2);

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"34", v);
        check("T2 atom_burst_count", v, 2);

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"3C", v);
        check("T2 max_atom_burst_length", v, 1);

        reset_stats;

        -----------------------------------------------------------------------
        -- Test 3: Atom on port 3 followed by atom on port 0 next cycle
        -- This tests prev_atom_valid transition: port3=1 -> prev=1
        -- "0001" then "1100"
        -- Expected: one burst of length 3, and 3 atoms
        -----------------------------------------------------------------------
        report "Test 3: Port 3 then port 0 and 1 next cycle";
        set_stats_en('1');

        atom_valid3 <= '1'; atom_nb3 <= "00001"; atom_elements3 <= (0 => '1', others => '0');
        wait until rising_edge(clock);
        atom_valid3 <= '0';
        atom_valid0 <= '1'; atom_nb0 <= "00001"; atom_elements0 <= (0 => '1', others => '0');
        atom_valid1 <= '1'; atom_nb0 <= "00001"; atom_elements0 <= (0 => '1', others => '0');
        wait until rising_edge(clock);
        atom_valid0 <= '0';
        atom_valid1 <= '0';

        -- gap to close last burst
        wait until rising_edge(clock);

        set_stats_en('0');

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"30", v);
        check("T3 atom_count", v, 3);

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"34", v);
        check("T3 atom_burst_count", v, 1);

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"3C", v);
        check("T3 max_atom_burst_length", v, 3);

        reset_stats;

        -----------------------------------------------------------------------
        -- Test 4: Atom-gap-atom pattern, testing gap measurement
        -- "1000" gap(2 cycles) "0001"
        -- Expected: burst_count=2, gap_count=2,
        --           gap_length=3 (1000) + 2*4 (0000) + 3 (0001) = 14
        -----------------------------------------------------------------------
        report "Test 4: Gap counting";
        set_stats_en('1');

        atom_valid0 <= '1'; atom_nb0 <= "00001"; atom_elements0 <= (0 => '1', others => '0');
        wait until rising_edge(clock);
        atom_valid0 <= '0';

        -- 2 idle cycles
        wait until rising_edge(clock);
        wait until rising_edge(clock);

        atom_valid3 <= '1';
        wait until rising_edge(clock);
        atom_valid3 <= '0';

        -- close last burst
        wait until rising_edge(clock);

        set_stats_en('0');

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"34", v);
        check("T4 atom_burst_count", v, 2);

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"44", v);
        check("T4 atom_gap_count", v, 1);

        -- first cycle  (atom 0 valid) - 1000 - gap 3 (ongoing)
        -- second cycle (idle)         - 0000 - gap 4 (ongoing)
        -- third cycle  (idle)         - 0000 - gap 4 (ongoing)
        -- fourth cycle (atom 3 valid) - 0001 - gap 3 (closed)
        -- then 3 full idle cycles = 3*4 = 12, total = 15
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"4C", v);
        check("T4 max_atom_gap_length", v, 14);

        reset_stats;

        -----------------------------------------------------------------------
        -- Test 5: verify no spurious gaps from set_stats_en itself
        -----------------------------------------------------------------------
        report "Test 5: Spurious gap check";
        set_stats_en('1');

        atom_valid2 <= '1'; atom_nb2 <= "00001"; atom_elements2 <= (0 => '1', others => '0');
        wait until rising_edge(clock);
        atom_valid2 <= '0';

        set_stats_en('0');

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"44", v);
        check("T5 spurious gap_count", v, 0);

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"50", v);
        check("T5 spurious sum_gaps", v, 0);

        reset_stats;

        -----------------------------------------------------------------------
        -- Test 6: Inners
        -- "1010", "1001", "0110", "0101"
        -- Expected: burst_count=7, gap_count=7,
        --           max_burst_length=2, max_gap_length=2
        -----------------------------------------------------------------------

        report "Test 6: Inners";
        set_stats_en('1');

        -- Packet 1 - 1010
        atom_valid0 <= '1'; atom_nb0 <= "00001"; atom_elements0 <= (0 => '1', others => '0');
        atom_valid2 <= '1'; atom_nb2 <= "00001"; atom_elements2 <= (0 => '1', others => '0');
        wait until rising_edge(clock);
        atom_valid0 <= '0';
        atom_valid2 <= '0';

        -- Packet 2 - 1001
        atom_valid0 <= '1'; atom_nb0 <= "00001"; atom_elements0 <= (0 => '1', others => '0');
        atom_valid3 <= '1'; atom_nb3 <= "00001"; atom_elements3 <= (0 => '1', others => '0');
        wait until rising_edge(clock);
        atom_valid0 <= '0';
        atom_valid3 <= '0';

        -- Packet 3 - 0110
        atom_valid1 <= '1'; atom_nb1 <= "00001"; atom_elements1 <= (0 => '1', others => '0');
        atom_valid2 <= '1'; atom_nb2 <= "00001"; atom_elements2 <= (0 => '1', others => '0');
        wait until rising_edge(clock);
        atom_valid1 <= '0';
        atom_valid2 <= '0';

        -- Packet 4 - 0101
        atom_valid1 <= '1'; atom_nb1 <= "00001"; atom_elements1 <= (0 => '1', others => '0');
        atom_valid3 <= '1'; atom_nb3 <= "00001"; atom_elements3 <= (0 => '1', others => '0');
        wait until rising_edge(clock);
        atom_valid1 <= '0';
        atom_valid3 <= '0';

        set_stats_en('0');

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"34", v);
        check("T6 atom_burst_count", v, 7);

        -- 6 inners + 1 previous
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"44", v);
        check("T6 atom_gap_count", v, 6);

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"4C", v);
        check("T6 max_atom_gap_length", v, 2);

        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata, s_axi_arready, s_axi_rvalid, clock, x"3C", v);
        check("T6 max_atom_burst_length", v, 2);

        reset_stats;


        wait;
    end process;
end Simulation;