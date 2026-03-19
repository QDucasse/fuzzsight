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
    component decoder_stats
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

    -- -----------------
    -- Signals
    -- -----------------

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
    DUT: decoder_stats port map (
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
        -- Variables to store read results
        variable total_cycles : std_logic_vector(31 downto 0);
        variable packet_count : std_logic_vector(31 downto 0);
        variable idle_cycles  : std_logic_vector(31 downto 0);
        variable burst_count  : std_logic_vector(31 downto 0);
        variable max_burst    : std_logic_vector(31 downto 0);
        variable min_gap      : std_logic_vector(31 downto 0);
        variable max_gap      : std_logic_vector(31 downto 0);
        variable gap_events   : std_logic_vector(31 downto 0);
        variable sum_burst    : std_logic_vector(31 downto 0);
        variable sum_gaps     : std_logic_vector(31 downto 0);
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

        ----------------------------------------------------------------
        -- DECODER PASSTHROUGH
        ----------------------------------------------------------------

        -- Decoder info without stats_en, they should be transparently sent
        atom_valid0     <= '1';
        address_reg_0_0 <= x"0000000000000100";
        atom_elements0  <= "000000000000000000101011"; -- ENENEE
        atom_nb0        <= "00110";                    -- 6 elements
        atom_valid1     <= '1';
        address_reg_0_1 <= x"0000000000000200";
        atom_elements1  <= "000000000000000000101011"; -- ENENEE
        atom_nb1        <= "00110";                    -- 6 elements
        atom_valid2     <= '1';
        address_reg_0_2 <= x"0000000000000300";
        atom_elements2  <= "000000000000000000101011"; -- ENENEE
        atom_nb2        <= "00110";                    -- 6 elements
        atom_valid3     <= '1';
        address_reg_0_3 <= x"0000000000000400";
        atom_elements3  <= "000000000000000000101011"; -- ENENEE
        atom_nb3        <= "00110";                    -- 6 elements

        wait for 2 ns;
        atom_valid0 <= '0';
        atom_valid1 <= '0';
        atom_valid2 <= '0';
        atom_valid3 <= '0';

        ----------------------------------------------------------------
        -- AXI-LITE WRITE ENABLE
        ----------------------------------------------------------------

        axi_lite_write(s_axi_awaddr, s_axi_awvalid, s_axi_wdata, s_axi_wvalid,
               s_axi_awready, s_axi_wready, s_axi_bvalid, clock, x"00", x"00000001");

        ----------------------------------------------------------------
        -- DECODER INCOMING DATA
        ----------------------------------------------------------------

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

        wait for 2 ns;

        -- Single atom packet on port 0
        atom_valid0     <= '1';
        address_reg_0_0 <= x"0000000000000100";
        atom_elements0  <= "000000000000000000000001"; -- first atom is E
        atom_nb0        <= "00001";                    -- 1 element

        wait for 2 ns;

        atom_valid0 <= '0';

        -- Simultaneous atoms on ports 1 and 2
        atom_valid1     <= '1';
        address_reg_0_1 <= x"0000000000000200";
        atom_elements1  <= "000000000000000000000011";
        atom_nb1        <= "00010";

        atom_valid2     <= '1';
        address_reg_0_2 <= x"0000000000000300";
        atom_elements2  <= "000000000000000000000101";
        atom_nb2        <= "00010";

        wait for 2 ns;

        atom_valid1 <= '0';
        atom_valid2 <= '0';

        -- Gap
        wait for 10 ns;

        -- Single atom packet on port 3
        atom_valid3     <= '1';
        address_reg_0_3 <= x"0000000000000400";
        atom_elements3  <= "000000000000000000000001"; -- first atom is E
        atom_nb3        <= "00001";                    -- 1 element

        wait for 2 ns;

        atom_valid3 <= '0';

        wait for 2 ns;

        ----------------------------------------------------------------
        -- AXI-LITE WRITE DISABLE
        ----------------------------------------------------------------

        axi_lite_write(s_axi_awaddr, s_axi_awvalid, s_axi_wdata, s_axi_wvalid,
               s_axi_awready, s_axi_wready, s_axi_bvalid, clock, x"00", x"00000000");

        ----------------------------------------------------------------
        -- AXI-LITE READ: dump stats registers
        ----------------------------------------------------------------

        -- Read total_cycles (0x04)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"04", total_cycles);

        -- Read packet_count (0x08)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"08", packet_count);

        -- Read idle_cycles (0x0C)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"0C", idle_cycles);

        -- Read burst_count (0x10)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"10", burst_count);

        -- Read max_burst (0x14)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"14", max_burst);

        -- Read min_gap (0x18)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"18", min_gap);

        -- Read max_gap (0x1C)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"1C", max_gap);

        -- Read gap_events (0x20)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"20", gap_events);

        -- Read sum_burst (0x24)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"24", sum_burst);

        -- Read sum_gaps (0x28)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"28", sum_gaps);

        -- Print all stats in a simple report
        report "=== Decoder Stats ===";
        report "Total cycles  : " & integer'image(to_integer(unsigned(total_cycles)));
        report "Packet count  : " & integer'image(to_integer(unsigned(packet_count)));
        report "Idle cycles   : " & integer'image(to_integer(unsigned(idle_cycles)));
        report "Burst count   : " & integer'image(to_integer(unsigned(burst_count)));
        report "Max burst     : " & integer'image(to_integer(unsigned(max_burst)));
        report "Min gap       : " & integer'image(to_integer(unsigned(min_gap)));
        report "Max gap       : " & integer'image(to_integer(unsigned(max_gap)));
        report "Gap events    : " & integer'image(to_integer(unsigned(gap_events)));
        report "Sum burst     : " & integer'image(to_integer(unsigned(sum_burst)));
        report "Sum gaps      : " & integer'image(to_integer(unsigned(sum_gaps)));
        report "=====================";


        ----------------------------------------------------------------
        -- AXI-LITE WRITE RESET
        ----------------------------------------------------------------

        axi_lite_write(s_axi_awaddr, s_axi_awvalid, s_axi_wdata, s_axi_wvalid,
               s_axi_awready, s_axi_wready, s_axi_bvalid, clock,
               x"00", x"00000002");  -- bit 1 = stats_reset

        -- Wait a few cycles for the self-clearing reset
        wait for 4 ns;

        ----------------------------------------------------------------
        -- AXI-LITE READ: confirm counters reset
        ----------------------------------------------------------------

                -- Read total_cycles (0x04)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"04", total_cycles);

        -- Read packet_count (0x08)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"08", packet_count);

        -- Read idle_cycles (0x0C)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"0C", idle_cycles);

        -- Read burst_count (0x10)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"10", burst_count);

        -- Read max_burst (0x14)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"14", max_burst);

        -- Read min_gap (0x18)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"18", min_gap);

        -- Read max_gap (0x1C)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"1C", max_gap);

        -- Read gap_events (0x20)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"20", gap_events);

        -- Read sum_burst (0x24)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"24", sum_burst);

        -- Read sum_gaps (0x28)
        axi_lite_read(s_axi_araddr, s_axi_arvalid, s_axi_rdata,
              s_axi_arready, s_axi_rvalid, clock,
              x"28", sum_gaps);

        -- Print all stats in a simple report
        report "=== AXI Stats ====";
        report "Total cycles   : " & integer'image(to_integer(unsigned(total_cycles)));
        report "Transfer count : " & integer'image(to_integer(unsigned(packet_count)));
        report "Idle cycles    : " & integer'image(to_integer(unsigned(idle_cycles)));
        report "Burst count    : " & integer'image(to_integer(unsigned(burst_count)));
        report "Max burst      : " & integer'image(to_integer(unsigned(max_burst)));
        report "Min gap        : " & integer'image(to_integer(unsigned(min_gap)));
        report "Max gap        : " & integer'image(to_integer(unsigned(max_gap)));
        report "Gap events     : " & integer'image(to_integer(unsigned(gap_events)));
        report "Sum burst      : " & integer'image(to_integer(unsigned(sum_burst)));
        report "Sum gaps       : " & integer'image(to_integer(unsigned(sum_gaps)));
        report "==================";


        wait;
    end process;
end Simulation;