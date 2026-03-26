library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.decoder_constants.all;

-- Decoder stats
--
-- Module that derives stats from a decoded CoreSight stream:
--   global idle cycles,
--   gap/burst details for packets (32-bits inputs),
--   gap/burst details for atoms,
--
-- Presents an AXI-Lite interface to activate the module and read the stats:
--   0x00: Control (bit 0 = stats enable, bit 1 = reset)
--   0x04: Total cycles
--   0x08: Idle cycles
--   0x0C: Packet count
--   0x10: Packet burst count
--   0x14: Min packet burst length
--   0x18: Max packet burst length
--   0x1C: Sum packet bursts
--   0x20: Packet gap count
--   0x24: Min packet gap length
--   0x28: Max packet gap length
--   0x2C: Sum packet gaps
--   0x30: Atom count
--   0x34: Atom burst count
--   0x38: Min atom burst length
--   0x3C: Max atom burst length
--   0x40: Sum atom bursts
--   0x44: Atom gap count
--   0x48: Min atom gap length
--   0x4C: Max atom gap length
--   0x50: Sum atom gaps

entity decoder_stats is
    generic (
        DATA_WIDTH : integer := 64;
        COUNTER_W  : integer := 64;
        ADDR_WIDTH : integer := 8
    );
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

        -- Output
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
end entity;

architecture Behavioral of decoder_stats is

    ------------------
    -- Procedures
    ------------------

    procedure process_atom(
        i_valid           : in  std_logic;
        has_seen_transfer : in  std_logic;
        -- vars
        variable v_prev_atom_valid : inout std_logic;
        variable v_atom_count      : inout unsigned(COUNTER_W-1 downto 0);

        variable v_atom_burst_length     : inout unsigned(COUNTER_W-1 downto 0);
        variable v_atom_burst_count      : inout unsigned(COUNTER_W-1 downto 0);
        variable v_max_atom_burst_length : inout unsigned(COUNTER_W-1 downto 0);
        variable v_min_atom_burst_length : inout unsigned(COUNTER_W-1 downto 0);
        variable v_sum_atom_bursts       : inout unsigned(COUNTER_W-1 downto 0);

        variable v_atom_gap_length     : inout unsigned(COUNTER_W-1 downto 0);
        variable v_atom_gap_count      : inout unsigned(COUNTER_W-1 downto 0);
        variable v_min_atom_gap_length : inout unsigned(COUNTER_W-1 downto 0);
        variable v_max_atom_gap_length : inout unsigned(COUNTER_W-1 downto 0);
        variable v_sum_atom_gaps       : inout unsigned(COUNTER_W-1 downto 0)
    ) is
    begin
        if i_valid = '1' then
            v_atom_count := v_atom_count + 1;

            -- Continuation of current burst
            if v_prev_atom_valid = '1' then
                v_atom_burst_length := v_atom_burst_length + 1;
            else
                -- Update gaps info
                if has_seen_transfer = '1' then
                    v_atom_gap_count := v_atom_gap_count + 1;
                    v_sum_atom_gaps  := v_sum_atom_gaps + v_atom_gap_length;

                    if v_atom_gap_length < v_min_atom_gap_length or v_min_atom_gap_length = 0 then
                        v_min_atom_gap_length := v_atom_gap_length;
                    end if;
                    if v_atom_gap_length > v_max_atom_gap_length then
                        v_max_atom_gap_length := v_atom_gap_length;
                    end if;
                end if;

                v_atom_burst_length := to_unsigned(1, COUNTER_W);
            end if;

            v_atom_gap_length := (others=>'0');
            v_prev_atom_valid := '1';

        else -- i_valid = '0'
            -- Update burst info
            if v_prev_atom_valid = '1' then
                v_atom_burst_count := v_atom_burst_count + 1;
                v_sum_atom_bursts  := v_sum_atom_bursts  + v_atom_burst_length;

                if v_atom_burst_length > v_max_atom_burst_length then
                    v_max_atom_burst_length := v_atom_burst_length;
                end if;

                if v_atom_burst_length < v_min_atom_burst_length or v_min_atom_burst_length = 0 then
                    v_min_atom_burst_length := v_atom_burst_length;
                end if;

                v_atom_burst_length := (others=>'0');
            end if;

            v_atom_gap_length := v_atom_gap_length + 1;
            v_prev_atom_valid := '0';
        end if;
    end procedure;

    ------------------
    -- Signals
    ------------------

    -- Enable/Reset
    signal stats_en      : std_logic := '0';
    signal stats_reset   : std_logic := '0';

    -- Reset handle, AXI write capture
    signal stats_reset_req   : std_logic := '0';

    -- Stats for the AXI transfer
    signal total_cycles            : unsigned(COUNTER_W-1 downto 0);
    signal idle_cycles             : unsigned(COUNTER_W-1 downto 0);

    -- Packets-related
    signal packet_count            : unsigned(COUNTER_W-1 downto 0);

    signal packet_burst_count      : unsigned(COUNTER_W-1 downto 0);
    signal max_packet_burst_length : unsigned(COUNTER_W-1 downto 0);
    signal min_packet_burst_length : unsigned(COUNTER_W-1 downto 0);
    signal sum_packet_bursts       : unsigned(COUNTER_W-1 downto 0);

    signal packet_gap_count        : unsigned(COUNTER_W-1 downto 0);
    signal min_packet_gap_length   : unsigned(COUNTER_W-1 downto 0);
    signal max_packet_gap_length   : unsigned(COUNTER_W-1 downto 0);
    signal sum_packet_gaps         : unsigned(COUNTER_W-1 downto 0);

    -- Atom-related
    signal atom_count              : unsigned(COUNTER_W-1 downto 0);

    signal atom_burst_count        : unsigned(COUNTER_W-1 downto 0);
    signal min_atom_burst_length   : unsigned(COUNTER_W-1 downto 0);
    signal max_atom_burst_length   : unsigned(COUNTER_W-1 downto 0);
    signal sum_atom_bursts         : unsigned(COUNTER_W-1 downto 0);

    signal atom_gap_count          : unsigned(COUNTER_W-1 downto 0);
    signal min_atom_gap_length     : unsigned(COUNTER_W-1 downto 0);
    signal max_atom_gap_length     : unsigned(COUNTER_W-1 downto 0);
    signal sum_atom_gaps           : unsigned(COUNTER_W-1 downto 0);

    -- Current info

    signal packet_gap_length       : unsigned(COUNTER_W-1 downto 0);
    signal packet_burst_length     : unsigned(COUNTER_W-1 downto 0);
    signal prev_packet_valid       : std_logic;

    signal atom_burst_length       : unsigned(COUNTER_W-1 downto 0);
    signal atom_gap_length         : unsigned(COUNTER_W-1 downto 0);
    signal prev_atom_valid         : std_logic;

    signal has_seen_transfer : std_logic;

    -- AXI-Lite signals
    signal awready : std_logic := '0';
    signal wready  : std_logic := '0';
    signal bvalid  : std_logic := '0';
    signal bresp   : std_logic_vector(1 downto 0) := (others=>'0');

    signal arready : std_logic := '0';
    signal rvalid  : std_logic := '0';
    signal rresp   : std_logic_vector(1 downto 0) := (others=>'0');

    signal awaddr_reg : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal araddr_reg : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal wdata_reg  : std_logic_vector(31 downto 0);

    signal aw_seen : std_logic := '0';
    signal w_seen  : std_logic := '0';
    signal read_in_progress : std_logic := '0';

begin
    -- Passthrough of the decoder info
    o_atom_valid0     <= i_atom_valid0;
    o_address_reg_0_0 <= i_address_reg_0_0;
    o_atom_elements0  <= i_atom_elements0;
    o_atom_nb0        <= i_atom_nb0;

    o_atom_valid1     <= i_atom_valid1;
    o_address_reg_0_1 <= i_address_reg_0_1;
    o_atom_elements1  <= i_atom_elements1;
    o_atom_nb1        <= i_atom_nb1;

    o_atom_valid2     <= i_atom_valid2;
    o_address_reg_0_2 <= i_address_reg_0_2;
    o_atom_elements2  <= i_atom_elements2;
    o_atom_nb2        <= i_atom_nb2;

    o_atom_valid3     <= i_atom_valid3;
    o_address_reg_0_3 <= i_address_reg_0_3;
    o_atom_elements3  <= i_atom_elements3;
    o_atom_nb3        <= i_atom_nb3;

    -- AXI-Lite outputs
    s_axi_awready <= awready;
    s_axi_wready  <= wready;
    s_axi_bvalid  <= bvalid;
    s_axi_bresp   <= bresp;

    s_axi_arready <= arready;
    s_axi_rvalid  <= rvalid;
    s_axi_rresp   <= rresp;

    -- Counting logic, unrolled to the 4 ports
    process(aclk)
        -- Atom stats, kept through ports
        variable v_prev_atom_valid       : std_logic;
        variable v_atom_count            : unsigned(COUNTER_W-1 downto 0);

        variable v_atom_burst_count      : unsigned(COUNTER_W-1 downto 0);
        variable v_atom_burst_length     : unsigned(COUNTER_W-1 downto 0);
        variable v_max_atom_burst_length : unsigned(COUNTER_W-1 downto 0);
        variable v_min_atom_burst_length : unsigned(COUNTER_W-1 downto 0);
        variable v_sum_atom_bursts       : unsigned(COUNTER_W-1 downto 0);

        variable v_atom_gap_count        : unsigned(COUNTER_W-1 downto 0);
        variable v_atom_gap_length       : unsigned(COUNTER_W-1 downto 0);
        variable v_max_atom_gap_length   : unsigned(COUNTER_W-1 downto 0);
        variable v_min_atom_gap_length   : unsigned(COUNTER_W-1 downto 0);
        variable v_sum_atom_gaps         : unsigned(COUNTER_W-1 downto 0);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' or stats_reset = '1' then
                total_cycles <= (others=>'0');
                idle_cycles  <= (others=>'0');

                -- Packets-related
                packet_count  <= (others=>'0');

                packet_burst_count      <= (others=>'0');
                max_packet_burst_length <= (others=>'0');
                min_packet_burst_length <= (others=>'0');
                sum_packet_bursts       <= (others=>'0');

                packet_gap_count        <= (others=>'0');
                min_packet_gap_length   <= (others=>'0');
                max_packet_gap_length   <= (others=>'0');
                sum_packet_gaps         <= (others=>'0');

                -- Atom-related
                atom_count              <= (others=>'0');

                atom_burst_count        <= (others=>'0');
                min_atom_burst_length   <= (others=>'0');
                max_atom_burst_length   <= (others=>'0');
                sum_atom_bursts         <= (others=>'0');

                atom_gap_count          <= (others=>'0');
                min_atom_gap_length     <= (others=>'0');
                max_atom_gap_length     <= (others=>'0');
                sum_atom_gaps           <= (others=>'0');

                -- Current info
                packet_gap_length       <= (others=>'0');
                packet_burst_length     <= (others=>'0');
                prev_packet_valid       <= '0';

                atom_burst_length       <= (others=>'0');
                atom_gap_length         <= (others=>'0');
                prev_atom_valid         <= '0';

                -- Latch
                has_seen_transfer       <= '0';

                -- Variables
                v_atom_burst_count      := (others=>'0');
                v_atom_burst_length     := (others=>'0');
                v_min_atom_burst_length := (others=>'0');
                v_max_atom_burst_length := (others=>'0');
                v_sum_atom_bursts       := (others=>'0');

                v_atom_gap_count        := (others=>'0');
                v_atom_gap_length       := (others=>'0');
                v_max_atom_gap_length   := (others=>'0');
                v_min_atom_gap_length   := (others=>'0');
                v_sum_atom_gaps         := (others=>'0');

                v_prev_atom_valid       := '0';

            elsif stats_en = '1' then
                total_cycles <= total_cycles + 1;

                ----------------------------------------------------------------
                -- PACKET-LEVEL INFO

                if (i_atom_valid0 or i_atom_valid1 or i_atom_valid2 or i_atom_valid3) = '1' then
                    packet_count <= packet_count + 1;
                    has_seen_transfer <= '1';

                    if prev_packet_valid = '1' then
                        packet_burst_length <= packet_burst_length + 1;
                    else
                        packet_burst_count <= packet_burst_count + 1;

                        if has_seen_transfer = '1' then
                            packet_gap_count <= packet_gap_count + 1;
                            sum_packet_gaps  <= sum_packet_gaps + packet_gap_length;

                            if packet_gap_length < min_packet_gap_length  then min_packet_gap_length  <= packet_gap_length; end if;
                            if packet_gap_length > max_packet_gap_length  then max_packet_gap_length  <= packet_gap_length; end if;
                        end if;

                        packet_burst_length <= to_unsigned(1, COUNTER_W);
                        packet_gap_length   <= (others=>'0');
                    end if;

                    prev_packet_valid <= '1';

                else
                    idle_cycles <= idle_cycles + 1;
                    packet_gap_length <= packet_gap_length + 1;

                    if prev_packet_valid = '1' then
                        sum_packet_bursts <= sum_packet_bursts + packet_burst_length;

                        if packet_burst_length > max_packet_burst_length then
                            max_packet_burst_length <= packet_burst_length;
                        end if;

                        if packet_burst_length < min_packet_burst_length or min_packet_burst_length = 0 then
                            min_packet_burst_length <= packet_burst_length;
                        end if;

                        packet_burst_length <= (others=>'0');
                    end if;

                    prev_packet_valid <= '0';
                end if;

                ----------------------------------------------------------------
                -- ATOM-LEVEL INFO
                ----------------------------------------------------------------

                -- Initialize variables with current signal values
                v_atom_count            := atom_count;
                v_prev_atom_valid       := prev_atom_valid;

                v_atom_burst_count      := atom_burst_count;
                v_atom_burst_length     := atom_burst_length;
                v_max_atom_burst_length := max_atom_burst_length;
                v_min_atom_burst_length := min_atom_burst_length;
                v_sum_atom_bursts       := sum_atom_bursts;

                v_atom_gap_count       := atom_gap_count;
                v_atom_gap_length      := atom_gap_length;
                v_max_atom_gap_length  := max_atom_gap_length;
                v_min_atom_gap_length  := min_atom_gap_length;
                v_sum_atom_gaps        := sum_atom_gaps;

                process_atom(
                    i_atom_valid0, has_seen_transfer, v_prev_atom_valid, v_atom_count,
                    v_atom_burst_length, v_atom_burst_count, v_max_atom_burst_length, v_min_atom_burst_length, v_sum_atom_bursts,
                    v_atom_gap_length, v_atom_gap_count, v_min_atom_gap_length, v_max_atom_gap_length, v_sum_atom_gaps
                );
                process_atom(
                    i_atom_valid1, has_seen_transfer, v_prev_atom_valid, v_atom_count,
                    v_atom_burst_length, v_atom_burst_count, v_max_atom_burst_length, v_min_atom_burst_length, v_sum_atom_bursts,
                    v_atom_gap_length, v_atom_gap_count, v_min_atom_gap_length, v_max_atom_gap_length, v_sum_atom_gaps
                );
                process_atom(
                    i_atom_valid2, has_seen_transfer, v_prev_atom_valid, v_atom_count,
                    v_atom_burst_length, v_atom_burst_count, v_max_atom_burst_length, v_min_atom_burst_length, v_sum_atom_bursts,
                    v_atom_gap_length, v_atom_gap_count, v_min_atom_gap_length, v_max_atom_gap_length, v_sum_atom_gaps
                );
                process_atom(
                    i_atom_valid3, has_seen_transfer, v_prev_atom_valid, v_atom_count,
                    v_atom_burst_length, v_atom_burst_count, v_max_atom_burst_length, v_min_atom_burst_length, v_sum_atom_bursts,
                    v_atom_gap_length, v_atom_gap_count, v_min_atom_gap_length, v_max_atom_gap_length, v_sum_atom_gaps
                );

                -- Commit back to signals
                atom_count            <= v_atom_count;
                prev_atom_valid       <= v_prev_atom_valid;

                atom_burst_count      <= v_atom_burst_count;
                atom_burst_length     <= v_atom_burst_length;
                max_atom_burst_length <= v_max_atom_burst_length;
                min_atom_burst_length <= v_min_atom_burst_length;
                sum_atom_bursts       <= v_sum_atom_bursts;

                atom_gap_count        <= v_atom_gap_count;
                atom_gap_length       <= v_atom_gap_length;
                max_atom_gap_length   <= v_max_atom_gap_length;
                min_atom_gap_length   <= v_min_atom_gap_length;
                sum_atom_gaps         <= v_sum_atom_gaps;
            end if;
        end if;
    end process;

    -- AXI-Lite interface for control and reads
    process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                awready <= '0';
                wready  <= '0';
                bvalid  <= '0';
                arready <= '0';
                rvalid  <= '0';

                awaddr_reg       <= (others => '0');
                araddr_reg       <= (others => '0');
                wdata_reg        <= (others => '0');
                s_axi_rdata      <= (others => '0');

                aw_seen <= '0';
                w_seen  <= '0';
                read_in_progress <= '0';
                stats_en <= '0';

            else
                -- WRITE CHANNEL

                -- Accept address
                if (awready = '0' and s_axi_awvalid = '1') then
                    awready    <= '1';
                    awaddr_reg <= s_axi_awaddr;
                    aw_seen    <= '1';
                else
                    awready    <= '0';
                end if;

                -- Accept data
                if (wready = '0' and s_axi_wvalid = '1') then
                    wready    <= '1';
                    wdata_reg <= s_axi_wdata;
                    w_seen    <= '1';
                else
                    wready    <= '0';
                end if;

                -- Generate write response
                if (aw_seen = '1' and w_seen = '1' and bvalid = '0') then
                    bvalid <= '1';
                    bresp  <= "00";

                    -- Register write
                    if awaddr_reg = x"00" then
                        stats_en        <= wdata_reg(0);
                        stats_reset_req <= wdata_reg(1);
                    end if;

                elsif (bvalid = '1' and s_axi_bready = '1') then
                    bvalid  <= '0';
                    aw_seen <= '0';
                    w_seen  <= '0';
                end if;

                ----------------------------------------------------------------
                -- READ CHANNEL

                -- Accept address
                if (arready = '0' and s_axi_arvalid = '1' and read_in_progress = '0') then
                    arready          <= '1';
                    araddr_reg       <= s_axi_araddr;
                    read_in_progress <= '1';
                else
                    arready          <= '0';
                end if;

                -- Provide data
                if (read_in_progress = '1' and rvalid = '0') then
                    rvalid <= '1';
                    rresp  <= "00";

                    case araddr_reg is
                        -- Offsets:
                        --   0x00: Control (bit 0 = stats enable, bit 1 = reset)
                        --   0x04: Total cycles
                        --   0x08: Idle cycles
                        --   0x0C: Packet count
                        --   0x10: Packet burst count
                        --   0x14: Min packet burst length
                        --   0x18: Max packet burst length
                        --   0x1C: Sum packet bursts
                        --   0x20: Packet gap count
                        --   0x24: Min packet gap length
                        --   0x28: Max packet gap length
                        --   0x2C: Sum packet gaps
                        --   0x30: Atom count
                        --   0x34: Atom burst count
                        --   0x38: Min atom burst length
                        --   0x3C: Max atom burst length
                        --   0x40: Sum atom bursts
                        --   0x44: Atom gap count
                        --   0x48: Min atom gap length
                        --   0x4C: Max atom gap length
                        --   0x50: Sum atom gaps

                        -- reset bit not readable as it is self-clearing
                        when x"00" => s_axi_rdata <= (31 downto 1=>'0') & stats_en;
                        when x"04" => s_axi_rdata <= std_logic_vector(total_cycles(31 downto 0));
                        when x"08" => s_axi_rdata <= std_logic_vector(idle_cycles(31 downto 0));
                        -- Packet
                        when x"0C" => s_axi_rdata <= std_logic_vector(packet_count(31 downto 0));
                        when x"10" => s_axi_rdata <= std_logic_vector(packet_burst_count(31 downto 0));
                        when x"14" => s_axi_rdata <= std_logic_vector(min_packet_burst_length(31 downto 0));
                        when x"18" => s_axi_rdata <= std_logic_vector(max_packet_burst_length(31 downto 0));
                        when x"1C" => s_axi_rdata <= std_logic_vector(sum_packet_bursts(31 downto 0));
                        when x"20" => s_axi_rdata <= std_logic_vector(packet_gap_count(31 downto 0));
                        when x"24" => s_axi_rdata <= std_logic_vector(min_packet_gap_length(31 downto 0));
                        when x"28" => s_axi_rdata <= std_logic_vector(max_packet_gap_length(31 downto 0));
                        when x"2C" => s_axi_rdata <= std_logic_vector(sum_packet_gaps(31 downto 0));
                        -- Atom
                        when x"30" => s_axi_rdata <= std_logic_vector(atom_count(31 downto 0));
                        when x"34" => s_axi_rdata <= std_logic_vector(atom_burst_count(31 downto 0));
                        when x"38" => s_axi_rdata <= std_logic_vector(min_atom_burst_length (31 downto 0));
                        when x"3C" => s_axi_rdata <= std_logic_vector(max_atom_burst_length (31 downto 0));
                        when x"40" => s_axi_rdata <= std_logic_vector(sum_atom_bursts(31 downto 0));
                        when x"44" => s_axi_rdata <= std_logic_vector(atom_gap_count(31 downto 0));
                        when x"48" => s_axi_rdata <= std_logic_vector(min_atom_gap_length(31 downto 0));
                        when x"4C" => s_axi_rdata <= std_logic_vector(max_atom_gap_length(31 downto 0));
                        when x"50" => s_axi_rdata <= std_logic_vector(sum_atom_gaps(31 downto 0));

                        when others => s_axi_rdata <= (others=>'0');
                    end case;

                elsif (rvalid = '1' and s_axi_rready = '1') then
                    rvalid <= '0';
                    read_in_progress <= '0';
                end if;
            end if;
            -- Self-clearing reset logic
            if stats_reset_req = '1' then
                stats_reset <= '1';
                stats_reset_req <= '0'; -- clear the request
            else
                stats_reset <= '0';
            end if;
        end if;
    end process;


end architecture;