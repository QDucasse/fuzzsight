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

entity decoder_stats_lut is
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

architecture Behavioral of decoder_stats_lut is

    type atom_stats_t is record
        previous_burst_ended : std_logic;
        previous_gap_ended   : std_logic;
        bursts_ended         : unsigned(3 downto 0);
        gaps_ended           : unsigned(3 downto 0);
        inner_bursts         : unsigned(3 downto 0);
        inner_gaps           : unsigned(3 downto 0);
        ongoing_burst        : unsigned(3 downto 0);
        ongoing_gap          : unsigned(3 downto 0);
    end record;

    type atom_lut_t is array (0 to 15, 0 to 1) of atom_stats_t;

    -- Instead of running a heavy combinational procedure on each of the four ports
    -- sequentially, a look-up table can hardcode the events directly.
    --
    -- Each sequence of incoming 4 valid bits (1 per port) is qualified using:
    --       - previous burst/gap ended: the last signal from the cycle before is different from the first one here
    --       - bursts/gaps ended: number of trailing 1s/0s
    --       - inner bursts/gaps: burst/gap starting AND ending in the same group
    --       - ongoing bursts/gaps: trailing 1s/0s
    -- This representation helps covering all cases directly.
    --
    -- Example:
    --     previous (1), 1110
    --        previous gap ended    - no, there were no gap before
    --        previous burst eneded - no, the burst is continuing in this cycle
    --        burst ended           - yes, 3 trailing 1s!
    --        gaps ended            - no, trailing 1s
    --        inner bursts/gaps     - no
    --        ongoing burst         - no, trailing 0
    --        ongoing gap           - yes, trailing 0!


    constant atom_lut : atom_lut_t := (
        -- For each index, prev_valid is 0/1
        0 => ( -- 0000
            0 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0000",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0000", ongoing_gap => "0100"), -- prev_valid = 0
            1 => (previous_burst_ended => '1', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0000",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0000", ongoing_gap => "0100")  -- prev_valid = 1
        ),
        1 => ( -- 0001
            0 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0011",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0001", ongoing_gap => "0000"), -- prev_valid = 0
            1 => (previous_burst_ended => '1', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0011",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0001", ongoing_gap => "0000")  -- prev_valid = 1
        ),
        2 => ( -- 0010
            0 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0010",
                  inner_bursts  => "0001", inner_gaps  => "0000",
                  ongoing_burst => "0000", ongoing_gap => "0001"), -- prev_valid = 0
            1 => (previous_burst_ended => '0', previous_gap_ended => '1',
                  bursts_ended  => "0000", gaps_ended  => "0010",
                  inner_bursts  => "0001", inner_gaps  => "0000",
                  ongoing_burst => "0000", ongoing_gap => "0001")  -- prev_valid = 1
        ),
        3 => ( -- 0011
            0 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0010",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0010", ongoing_gap => "0000"), -- prev_valid = 0
            1 => (previous_burst_ended => '1', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0010",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0010", ongoing_gap => "0000")  -- prev_valid = 1
        ),
        4 => ( -- 0100
            0 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0001",
                  inner_bursts  => "0001", inner_gaps  => "0000",
                  ongoing_burst => "0000", ongoing_gap => "0010"), -- prev_valid = 0
            1 => (previous_burst_ended => '1', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0001",
                  inner_bursts  => "0001", inner_gaps  => "0000",
                  ongoing_burst => "0000", ongoing_gap => "0010")  -- prev_valid = 1
        ),
        5 => ( -- 0101
            0 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0001",
                  inner_bursts  => "0001", inner_gaps  => "0001",
                  ongoing_burst => "0001", ongoing_gap => "0000"), -- prev_valid = 0
            1 => (previous_burst_ended => '1', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0001",
                  inner_bursts  => "0001", inner_gaps  => "0001",
                  ongoing_burst => "0001", ongoing_gap => "0000")  -- prev_valid = 1
        ),
        6 => ( -- 0110
            0 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0001",
                  inner_bursts  => "0010", inner_gaps  => "0000",
                  ongoing_burst => "0000", ongoing_gap => "0001"), -- prev_valid = 0
            1 => (previous_burst_ended => '1', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0001",
                  inner_bursts  => "0010", inner_gaps  => "0000",
                  ongoing_burst => "0000", ongoing_gap => "0001")  -- prev_valid = 1
        ),
        7 => ( -- 0111
            0 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0001",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0011", ongoing_gap => "0000"), -- prev_valid = 0
            1 => (previous_burst_ended => '1', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0001",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0011", ongoing_gap => "0000")  -- prev_valid = 1
        ),
        8 => ( -- 1000
            0 => (previous_burst_ended => '0', previous_gap_ended => '1',
                  bursts_ended  => "0001", gaps_ended  => "0000",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0000", ongoing_gap => "0011"), -- prev_valid = 0
            1 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0001", gaps_ended  => "0000",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0000", ongoing_gap => "0011")  -- prev_valid = 1
        ),
        9 =>  ( -- 1001
            0 => (previous_burst_ended => '0', previous_gap_ended => '1',
                  bursts_ended  => "0001", gaps_ended  => "0010",
                  inner_bursts  => "0000", inner_gaps  => "0010",
                  ongoing_burst => "0001", ongoing_gap => "0000"), -- prev_valid = 0
            1 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0001", gaps_ended  => "0010",
                  inner_bursts  => "0000", inner_gaps  => "0010",
                  ongoing_burst => "0001", ongoing_gap => "0000")  -- prev_valid = 1
        ),
        10 => ( -- 1010
            0 => (previous_burst_ended => '0', previous_gap_ended => '1',
                  bursts_ended  => "0001", gaps_ended  => "0000",
                  inner_bursts  => "0001", inner_gaps  => "0001",
                  ongoing_burst => "0000", ongoing_gap => "0001"), -- prev_valid = 0
            1 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0001", gaps_ended  => "0000",
                  inner_bursts  => "0001", inner_gaps  => "0001",
                  ongoing_burst => "0000", ongoing_gap => "0001")  -- prev_valid = 1
        ),
        11 => ( -- 1011
            0 => (previous_burst_ended => '0', previous_gap_ended => '1',
                  bursts_ended  => "0001", gaps_ended  => "0000",
                  inner_bursts  => "0000", inner_gaps  => "0001",
                  ongoing_burst => "0010", ongoing_gap => "0000"), -- prev_valid = 0
            1 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0001", gaps_ended  => "0000",
                  inner_bursts  => "0000", inner_gaps  => "0001",
                  ongoing_burst => "0010", ongoing_gap => "0000")  -- prev_valid = 1
        ),
        12 => ( -- 1100
            0 => (previous_burst_ended => '0', previous_gap_ended => '1',
                  bursts_ended  => "0010", gaps_ended  => "0000",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0000", ongoing_gap => "0010"), -- prev_valid = 0
            1 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0010", gaps_ended  => "0000",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0000", ongoing_gap => "0010")  -- prev_valid = 1
        ),
        13 => ( -- 1101
            0 => (previous_burst_ended => '0', previous_gap_ended => '1',
                  bursts_ended  => "0010", gaps_ended  => "0000",
                  inner_bursts  => "0000", inner_gaps  => "0001",
                  ongoing_burst => "0001", ongoing_gap => "0000"), -- prev_valid = 0
            1 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0010", gaps_ended  => "0000",
                  inner_bursts  => "0000", inner_gaps  => "0001",
                  ongoing_burst => "0001", ongoing_gap => "0000")  -- prev_valid = 1
        ),
        14 => ( -- 1110
            0 => (previous_burst_ended => '0', previous_gap_ended => '1',
                  bursts_ended  => "0011", gaps_ended  => "0000",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0000", ongoing_gap => "0001"), -- prev_valid = 0
            1 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0011", gaps_ended  => "0000",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0000", ongoing_gap => "0001")  -- prev_valid = 1
        ),
        15 => ( -- 1111
            0 => (previous_burst_ended => '0', previous_gap_ended => '1',
                  bursts_ended  => "0000", gaps_ended  => "0000",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0100", ongoing_gap => "0000"), -- prev_valid = 0
            1 => (previous_burst_ended => '0', previous_gap_ended => '0',
                  bursts_ended  => "0000", gaps_ended  => "0000",
                  inner_bursts  => "0000", inner_gaps  => "0000",
                  ongoing_burst => "0100", ongoing_gap => "0000")  -- prev_valid = 1
        )
    );

    ------------------
    -- Functions
    ------------------

    function popcount4(slv : std_logic_vector(3 downto 0)) return unsigned is
        variable cnt : integer range 0 to 4 := 0;
    begin
        for i in slv'range loop
            if slv(i) = '1' then
                cnt := cnt + 1;
            end if;
        end loop;
        return to_unsigned(cnt, 3); -- 3 bits enough for 0..4
    end function;

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
    signal atom_valids             : std_logic_vector(3 downto 0);
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
    signal prev_atom_valid         : integer := 0;

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
    -- Valid atom vector
    atom_valids       <= i_atom_valid3 & i_atom_valid2 & i_atom_valid1 & i_atom_valid0;

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

    -- Counting logic - packets
    packet_counting_process: process(aclk)
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

            elsif stats_en = '1' then
                total_cycles <= total_cycles + 1;

                ----------------------------------------------------------------
                -- PACKET-LEVEL INFO

                if (i_atom_valid0 or i_atom_valid1 or i_atom_valid2 or i_atom_valid3) = '1' then
                    packet_count <= packet_count + 1;

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

                        has_seen_transfer <= '1';
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
            end if;
        end if;
    end process;

    -- Atom counting logic, based on the LUT
    atom_counting_process: process(aclk)
        variable lut_entry : atom_stats_t;

        -- Atom stats
        variable v_atom_count            : unsigned(COUNTER_W-1 downto 0);

        variable v_atom_burst_count      : unsigned(COUNTER_W-1 downto 0);
        variable v_max_atom_burst_length : unsigned(COUNTER_W-1 downto 0);
        variable v_min_atom_burst_length : unsigned(COUNTER_W-1 downto 0);
        variable v_sum_atom_bursts       : unsigned(COUNTER_W-1 downto 0);

        variable v_atom_gap_count        : unsigned(COUNTER_W-1 downto 0);
        variable v_max_atom_gap_length   : unsigned(COUNTER_W-1 downto 0);
        variable v_min_atom_gap_length   : unsigned(COUNTER_W-1 downto 0);
        variable v_sum_atom_gaps         : unsigned(COUNTER_W-1 downto 0);

        -- Cross-cycle
        variable v_atom_burst_length          : unsigned(COUNTER_W-1 downto 0);
        variable v_atom_gap_length            : unsigned(COUNTER_W-1 downto 0);
        variable v_atom_burst_length_resolved : unsigned(COUNTER_W-1 downto 0);
        variable v_atom_gap_length_resolved   : unsigned(COUNTER_W-1 downto 0);

    begin
        if rising_edge(aclk) then
            if aresetn = '0' or stats_reset = '1' then
                atom_count              <= (others=>'0');

                atom_burst_count        <= (others=>'0');
                min_atom_burst_length   <= (others=>'0');
                max_atom_burst_length   <= (others=>'0');
                sum_atom_bursts         <= (others=>'0');

                atom_gap_count          <= (others=>'0');
                min_atom_gap_length     <= (others=>'0');
                max_atom_gap_length     <= (others=>'0');
                sum_atom_gaps           <= (others=>'0');

                atom_burst_length       <= (others=>'0');
                atom_gap_length         <= (others=>'0');
                prev_atom_valid         <= 0;

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

            elsif stats_en = '1' then
                -- Initialize variables with current signal values
                v_atom_count            := atom_count;

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

                -- LUT lookup
                lut_entry := atom_lut(
                    to_integer(unsigned(atom_valids)),
                    prev_atom_valid
                );

                ----------------------------------------------------------------
                -- Handle previous-crossing segments
                ----------------------------------------------------------------
                -- Case:
                -- previous cycle: atom_valid3 = 1
                -- this cycle:     atom_valid0 = 0
                --  -> update burst info
                if lut_entry.previous_burst_ended = '1' then
                    v_atom_burst_count := v_atom_burst_count + 1;
                    v_sum_atom_bursts  := v_sum_atom_bursts + v_atom_burst_length;

                    -- Check min/max
                    if v_atom_burst_length > v_max_atom_burst_length then
                        v_max_atom_burst_length := v_atom_burst_length;
                    end if;

                    if v_atom_burst_length < v_min_atom_burst_length or v_min_atom_burst_length = 0 then
                        v_min_atom_burst_length := v_atom_burst_length;
                    end if;

                    -- Reset ongoing length
                    v_atom_burst_length := (others => '0');
                end if;

                -- Case:
                -- previous cycle: atom_valid3 = 0
                -- this cycle:     atom_valid0 = 1
                -- -> update gap info
                if lut_entry.previous_gap_ended = '1' then
                    v_atom_gap_count := v_atom_gap_count + 1;
                    v_sum_atom_gaps  := v_sum_atom_gaps + v_atom_gap_length;

                    -- Check min/max
                    if v_atom_gap_length > v_max_atom_gap_length then
                        v_max_atom_gap_length := v_atom_gap_length;
                    end if;

                    if v_atom_gap_length < v_min_atom_gap_length or v_min_atom_gap_length = 0 then
                        v_min_atom_gap_length := v_atom_gap_length;
                    end if;

                    -- Reset ongoing length
                    v_atom_gap_length := (others => '0');
                end if;

                ----------------------------------------------------------------
                -- Handle leading segments
                ----------------------------------------------------------------

                -- A burst ends in these valids -> update burst info
                if lut_entry.bursts_ended /= 0 then
                    v_atom_burst_count := v_atom_burst_count + 1;

                    -- Continuation from the previous cycle
                    if lut_entry.previous_burst_ended = '0' then
                        v_atom_burst_length_resolved := v_atom_burst_length + resize(lut_entry.bursts_ended, COUNTER_W);
                    else -- Independent burst
                        v_atom_burst_length_resolved := resize(lut_entry.bursts_ended, COUNTER_W);
                    end if;
                    v_sum_atom_bursts  := v_sum_atom_bursts + v_atom_burst_length_resolved;

                    -- Check min/max
                    if v_atom_burst_length_resolved > v_max_atom_burst_length then
                        v_max_atom_burst_length := v_atom_burst_length_resolved;
                    end if;

                    if v_atom_burst_length_resolved < v_min_atom_burst_length or v_min_atom_burst_length = 0 then
                        v_min_atom_burst_length := v_atom_burst_length_resolved;
                    end if;

                    -- Reset ongoing length
                    v_atom_burst_length := (others => '0');
                end if;

                -- A gap ends in these valids -> update gap info
                if lut_entry.gaps_ended /= 0 then
                    v_atom_gap_count := v_atom_gap_count + 1;

                    -- Continuation from the previous cycle
                    if lut_entry.previous_gap_ended = '0' then
                        v_atom_gap_length_resolved := v_atom_gap_length + resize(lut_entry.gaps_ended, COUNTER_W);
                    else -- Independent gap
                        v_atom_gap_length_resolved := resize(lut_entry.gaps_ended, COUNTER_W);
                    end if;
                    v_sum_atom_gaps  := v_sum_atom_gaps + v_atom_gap_length_resolved;

                    -- Check min/max
                    if v_atom_gap_length_resolved > v_max_atom_gap_length then
                        v_max_atom_gap_length := v_atom_gap_length_resolved;
                    end if;

                    if v_atom_gap_length_resolved < v_min_atom_gap_length or v_min_atom_gap_length = 0 then
                        v_min_atom_gap_length := v_atom_gap_length_resolved;
                    end if;

                    -- Reset ongoing length
                    v_atom_gap_length := (others => '0');
                end if;

                ----------------------------------------------------------------
                -- Handle inner segments
                ----------------------------------------------------------------
                -- Inner burst found -> update burst info
                if lut_entry.inner_bursts /= 0 then
                    v_atom_burst_count := v_atom_burst_count + 1;  -- one burst per inner segment
                    v_sum_atom_bursts  := v_sum_atom_bursts + resize(lut_entry.inner_bursts, COUNTER_W);

                    -- Check min/max
                    if resize(lut_entry.inner_bursts, COUNTER_W) > v_max_atom_burst_length then
                        v_max_atom_burst_length := resize(lut_entry.inner_bursts, COUNTER_W);
                    end if;

                    if resize(lut_entry.inner_bursts, COUNTER_W) < v_min_atom_burst_length or v_min_atom_burst_length = 0 then
                        v_min_atom_burst_length := resize(lut_entry.inner_bursts, COUNTER_W);
                    end if;

                    -- Reset ongoing length
                    v_atom_burst_length := (others => '0');
                end if;

                -- Inner gap found -> update gap info
                if lut_entry.inner_gaps /= 0 then
                    v_atom_gap_count := v_atom_gap_count + 1;  -- one gap per inner segment
                    v_sum_atom_gaps  := v_sum_atom_gaps + resize(lut_entry.inner_gaps, COUNTER_W);

                    -- Check min/max
                    if resize(lut_entry.inner_gaps, COUNTER_W) > v_max_atom_gap_length then
                        v_max_atom_gap_length := resize(lut_entry.inner_gaps, COUNTER_W);
                    end if;

                    if resize(lut_entry.inner_gaps, COUNTER_W) < v_min_atom_gap_length or v_min_atom_gap_length = 0 then
                        v_min_atom_gap_length := resize(lut_entry.inner_gaps, COUNTER_W);
                    end if;

                    -- Reset ongoing length
                    v_atom_gap_length := (others => '0');
                end if;

                ----------------------------------------------------------------
                -- Accumulate ongoing segments
                ----------------------------------------------------------------
                v_atom_burst_length := v_atom_burst_length + resize(lut_entry.ongoing_burst, COUNTER_W);
                v_atom_gap_length   := v_atom_gap_length   + resize(lut_entry.ongoing_gap, COUNTER_W);

                ----------------------------------------------------------------
                -- Count valid atoms
                ----------------------------------------------------------------
                v_atom_count := v_atom_count + resize(popcount4(atom_valids), COUNTER_W);

                ----------------------------------------------------------------
                -- Update state
                ----------------------------------------------------------------
                if atom_valids(3) = '0' then
                    prev_atom_valid <= 0;
                else
                    prev_atom_valid <= 1;
                end if;

                -- Commit back to signals
                atom_count            <= v_atom_count;

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
    axi_lite_process: process(aclk)
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