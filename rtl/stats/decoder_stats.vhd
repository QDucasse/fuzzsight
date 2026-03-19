library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.decoder_constants.all;

-- Decoder stats
--
-- Module that derives stats from a decoded CoreSight stream:
--   global idle cycles,
--   gap details,
--   burst details,
--
-- Presents an AXI-Lite interface to activate the module and read the stats:
--   0x0: Control (bit 0 = stats enable, bit 1 = reset)
--   0x4: Total cycles [31:0]
--   0x8: Packet count [31:0]
--   0xC: Idle cycles [31:0]
--   0x10: Burst count [31:0]
--   0x14: Max burst [31:0]
--   0x18: Min gap [31:0]
--   0x1C: Max gap [31:0]
--   0x20: Gap events [31:0]
--   0x24: Sum burst [31:0]
--   0x28: Sum gaps [31:0]

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
    -- Enable/Reset
    signal stats_en      : std_logic := '0';
    signal stats_reset   : std_logic := '0';

    -- Reset handle, AXI write capture
    signal stats_reset_req   : std_logic := '0';

    -- Stats for the AXI transfer
    signal total_cycles : unsigned(COUNTER_W-1 downto 0);
    signal packet_count : unsigned(COUNTER_W-1 downto 0);
    signal idle_cycles  : unsigned(COUNTER_W-1 downto 0);

    signal gap_counter  : unsigned(COUNTER_W-1 downto 0);
    signal min_gap      : unsigned(COUNTER_W-1 downto 0);
    signal max_gap      : unsigned(COUNTER_W-1 downto 0);
    signal sum_gaps     : unsigned(COUNTER_W-1 downto 0);
    signal gap_events   : unsigned(COUNTER_W-1 downto 0);

    signal burst_len    : unsigned(COUNTER_W-1 downto 0);
    signal burst_count  : unsigned(COUNTER_W-1 downto 0);
    signal max_burst    : unsigned(COUNTER_W-1 downto 0);
    signal sum_burst    : unsigned(COUNTER_W-1 downto 0);

    signal consecutive_packet_cycles : unsigned(COUNTER_W-1 downto 0);

    signal prev_valid        : std_logic;
    signal has_seen_transfer : std_logic;

    -- AXI-Lite signals
    signal awready_i : std_logic := '0';
    signal wready_i  : std_logic := '0';
    signal bvalid_i  : std_logic := '0';
    signal bresp_i   : std_logic_vector(1 downto 0) := (others=>'0');

    signal arready_i : std_logic := '0';
    signal rvalid_i  : std_logic := '0';
    signal rresp_i   : std_logic_vector(1 downto 0) := (others=>'0');

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
    s_axi_awready <= awready_i;
    s_axi_wready  <= wready_i;
    s_axi_bvalid  <= bvalid_i;
    s_axi_bresp   <= bresp_i;

    s_axi_arready <= arready_i;
    s_axi_rvalid  <= rvalid_i;
    s_axi_rresp   <= rresp_i;

    -- Counting logic
    process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' or stats_reset = '1' then
                -- Cycle info
                total_cycles   <= (others=>'0');
                packet_count <= (others=>'0');
                idle_cycles    <= (others=>'0');

                -- Gap info (cycles between packets)
                gap_counter   <= (others=>'0');
                min_gap       <= (others=>'1');
                max_gap       <= (others=>'0');
                sum_gaps      <= (others=>'0');
                gap_events    <= (others=>'0');

                -- Burst info (packs of valid packets)
                burst_len     <= (others=>'0');
                burst_count   <= (others=>'0');
                max_burst     <= (others=>'0');
                sum_burst     <= (others=>'0');

                -- Current values
                consecutive_packet_cycles <= (others=>'0');
                has_seen_transfer           <= '0';
                prev_valid                  <= '0';
            else
                 if stats_en = '1' then
                    -- Global cycle count
                    total_cycles <= total_cycles + 1;

                    ----------------------------------------------
                    -- Port 0
                    if i_atom_valid0 = '1' then

                        packet_count <= packet_count + 1;
                        burst_len    <= burst_len + 1;

                        has_seen_transfer <= '1';  -- monotonic latch

                        if prev_valid = '1' then
                            consecutive_packet_cycles <= consecutive_packet_cycles + 1;
                        else
                            -- New burst
                            burst_count <= burst_count + 1;

                            -- Gap is valid only after first-ever transfer
                            if has_seen_transfer = '1' then
                                gap_events <= gap_events + 1;
                                sum_gaps   <= sum_gaps + gap_counter;

                                if gap_counter < min_gap then
                                    min_gap <= gap_counter;
                                end if;

                                if gap_counter > max_gap then
                                    max_gap <= gap_counter;
                                end if;
                            end if;

                            gap_counter <= (others=>'0');
                        end if;

                    else -- i_atom_valid0
                        idle_cycles <= idle_cycles + 1;
                        gap_counter <= gap_counter + 1;

                        -- Updating burst events
                        if prev_valid = '1' then
                            sum_burst <= sum_burst + burst_len;

                            if burst_len > max_burst then
                                max_burst <= burst_len;
                            end if;

                            burst_len <= (others=>'0');
                        end if;
                    end if;

                    ----------------------------------------------
                    -- Port 1
                    if i_atom_valid1 = '1' then

                        packet_count <= packet_count + 1;
                        burst_len    <= burst_len + 1;

                        has_seen_transfer <= '1';  -- monotonic latch

                        if i_atom_valid0 = '1' then
                            consecutive_packet_cycles <= consecutive_packet_cycles + 1;
                        else
                            -- New burst
                            burst_count <= burst_count + 1;

                            -- Gap is valid only after first-ever transfer
                            if has_seen_transfer = '1' then
                                gap_events <= gap_events + 1;
                                sum_gaps   <= sum_gaps + gap_counter;

                                if gap_counter < min_gap then
                                    min_gap <= gap_counter;
                                end if;

                                if gap_counter > max_gap then
                                    max_gap <= gap_counter;
                                end if;
                            end if;

                            gap_counter <= (others=>'0');
                        end if;

                    else -- i_atom_valid0
                        idle_cycles <= idle_cycles + 1;
                        gap_counter <= gap_counter + 1;

                        -- Updating burst events
                        if i_atom_valid0 = '1' then
                            sum_burst <= sum_burst + burst_len;

                            if burst_len > max_burst then
                                max_burst <= burst_len;
                            end if;

                            burst_len <= (others=>'0');
                        end if;
                    end if;

                    ----------------------------------------------
                    -- Port 2
                    if i_atom_valid2 = '1' then

                        packet_count <= packet_count + 1;
                        burst_len    <= burst_len + 1;

                        has_seen_transfer <= '1';  -- monotonic latch

                        if i_atom_valid1 = '1' then
                            consecutive_packet_cycles <= consecutive_packet_cycles + 1;
                        else
                            -- New burst
                            burst_count <= burst_count + 1;

                            -- Gap is valid only after first-ever transfer
                            if has_seen_transfer = '1' then
                                gap_events <= gap_events + 1;
                                sum_gaps   <= sum_gaps + gap_counter;

                                if gap_counter < min_gap then
                                    min_gap <= gap_counter;
                                end if;

                                if gap_counter > max_gap then
                                    max_gap <= gap_counter;
                                end if;
                            end if;

                            gap_counter <= (others=>'0');
                        end if;

                    else -- i_atom_valid0
                        idle_cycles <= idle_cycles + 1;
                        gap_counter <= gap_counter + 1;

                        -- Updating burst events
                        if i_atom_valid1 = '1' then
                            sum_burst <= sum_burst + burst_len;

                            if burst_len > max_burst then
                                max_burst <= burst_len;
                            end if;

                            burst_len <= (others=>'0');
                        end if;
                    end if;

                    ----------------------------------------------
                    -- Port 3
                    if i_atom_valid3 = '1' then

                        packet_count <= packet_count + 1;
                        burst_len    <= burst_len + 1;

                        has_seen_transfer <= '1';  -- monotonic latch

                        if i_atom_valid2 = '1' then
                            consecutive_packet_cycles <= consecutive_packet_cycles + 1;
                        else
                            -- New burst
                            burst_count <= burst_count + 1;

                            -- Gap is valid only after first-ever transfer
                            if has_seen_transfer = '1' then
                                gap_events <= gap_events + 1;
                                sum_gaps   <= sum_gaps + gap_counter;

                                if gap_counter < min_gap then
                                    min_gap <= gap_counter;
                                end if;

                                if gap_counter > max_gap then
                                    max_gap <= gap_counter;
                                end if;
                            end if;

                            gap_counter <= (others=>'0');
                        end if;

                    else -- i_atom_valid0
                        idle_cycles <= idle_cycles + 1;
                        gap_counter <= gap_counter + 1;

                        -- Updating burst events
                        if i_atom_valid2 = '1' then
                            sum_burst <= sum_burst + burst_len;

                            if burst_len > max_burst then
                                max_burst <= burst_len;
                            end if;

                            burst_len <= (others=>'0');
                        end if;
                    end if;

                    prev_valid <= i_atom_valid3;
                end if;
            end if;
        end if;
    end process;

    -- AXI-Lite interface for control and reads
    process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                awready_i <= '0';
                wready_i  <= '0';
                bvalid_i  <= '0';
                arready_i <= '0';
                rvalid_i  <= '0';
                aw_seen   <= '0';
                w_seen    <= '0';
                read_in_progress  <= '0';
                stats_en <= '0';

            else
                -- WRITE CHANNEL

                -- Accept address
                if (awready_i = '0' and s_axi_awvalid = '1') then
                    awready_i  <= '1';
                    awaddr_reg <= s_axi_awaddr;
                    aw_seen    <= '1';
                else
                    awready_i  <= '0';
                end if;

                -- Accept data
                if (wready_i = '0' and s_axi_wvalid = '1') then
                    wready_i  <= '1';
                    wdata_reg <= s_axi_wdata;
                    w_seen    <= '1';
                else
                    wready_i  <= '0';
                end if;

                -- Generate write response
                if (aw_seen = '1' and w_seen = '1' and bvalid_i = '0') then
                    bvalid_i <= '1';
                    bresp_i  <= "00";

                    -- Register write
                    if awaddr_reg = x"00" then
                        stats_en        <= wdata_reg(0);
                        stats_reset_req <= wdata_reg(1);
                    end if;

                elsif (bvalid_i = '1' and s_axi_bready = '1') then
                    bvalid_i <= '0';
                    aw_seen  <= '0';
                    w_seen   <= '0';
                end if;

                ----------------------------------------------------------------
                -- READ CHANNEL

                -- Accept address
                if (arready_i = '0' and s_axi_arvalid = '1' and read_in_progress = '0') then
                    arready_i        <= '1';
                    araddr_reg       <= s_axi_araddr;
                    read_in_progress <= '1';
                else
                    arready_i        <= '0';
                end if;

                -- Provide data
                if (read_in_progress = '1' and rvalid_i = '0') then
                    rvalid_i <= '1';
                    rresp_i  <= "00";

                    case araddr_reg is
                        -- reset bit not readable as it is self-clearing
                        when x"00" => s_axi_rdata <= (31 downto 1=>'0') & stats_en;
                        when x"04" => s_axi_rdata <= std_logic_vector(total_cycles(31 downto 0));
                        when x"08" => s_axi_rdata <= std_logic_vector(packet_count(31 downto 0));
                        when x"0C" => s_axi_rdata <= std_logic_vector(idle_cycles(31 downto 0));
                        when x"10" => s_axi_rdata <= std_logic_vector(burst_count(31 downto 0));
                        when x"14" => s_axi_rdata <= std_logic_vector(max_burst(31 downto 0));
                        when x"18" => s_axi_rdata <= std_logic_vector(min_gap(31 downto 0));
                        when x"1C" => s_axi_rdata <= std_logic_vector(max_gap(31 downto 0));
                        when x"20" => s_axi_rdata <= std_logic_vector(gap_events(31 downto 0));
                        when x"24" => s_axi_rdata <= std_logic_vector(sum_burst(31 downto 0));
                        when x"28" => s_axi_rdata <= std_logic_vector(sum_gaps(31 downto 0));
                        when others => s_axi_rdata <= (others=>'0');
                    end case;

                elsif (rvalid_i = '1' and s_axi_rready = '1') then
                    rvalid_i <= '0';
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