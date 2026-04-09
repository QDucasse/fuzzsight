library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Frame generator
--
-- From TPIU 32-bit packets, constructs a 128-bit frame (so every 4 cycles).
-- Removes HALF-SYNCH packets
--
-- Important IOs:
-- i_data: 32 bit TPIU packet
-- o_frame: 128 bit frame

entity frame_generator is
    generic (
        AXIL_WIDTH : integer := 8
    );
    port (
        aclk    : in std_logic;
        aresetn : in std_logic;
        i_data  : in std_logic_vector (31 downto 0);

        i_freeze_request : in std_logic;

        o_frame       : out std_logic_vector(127 downto 0);
        o_valid_frame : out std_logic;
        o_frame_error : out std_logic;

        --AXI4-Lite interface (read only)
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
end frame_generator;

architecture Behavioral of frame_generator is

    constant SYNCH_PACKET : std_logic_vector(31 downto 0) := X"7FFFFFFF";
    constant HALF_SYNCH_PACKET : std_logic_vector(31 downto 0) := X"7FFF7FFF";
    constant EMPTY_FRAME : std_logic_vector(127 downto 0) := X"00000000000000000000000000000000";

    -- 4 cycle counter
    signal tpiu_packets_handled : integer range 0 to 3;

    -- Stabilized after seeing the first SYNC packet, cleared on freeze request
    signal synchronized : std_logic := '0';

    -- Latching freeze to clear synchronization on falling edge
    signal latched_freeze_req : std_logic := '0';

    -- AXI-Lite
    signal arready : std_logic;
    signal rresp   : std_logic_vector(1 downto 0);
    signal rvalid  : std_logic;

    signal read_in_progress : std_logic := '0';
    signal araddr_reg : std_logic_vector(AXIL_WIDTH-1 downto 0);

begin

    -- AXI4-Lite
    -- write
    s_axi_awready <= '0';
    s_axi_wready  <= '0';
    s_axi_bresp   <= "00";
    s_axi_bvalid  <= '0';
    -- read
    s_axi_arready <= arready;
    s_axi_rresp   <= rresp;
    s_axi_rvalid  <= rvalid;

    generator : process (ACLK) begin
    if rising_edge(ACLK) then
        -- Active low
        if aresetn = '0' then
            tpiu_packets_handled <= 0;

            synchronized       <= '0';
            latched_freeze_req <= '0';

            o_frame       <= EMPTY_FRAME;
            o_valid_frame <= '0';
            o_frame_error <= '0';
        else
            -- Latch incoming freeze request
            latched_freeze_req <= i_freeze_request;

            -- Rising edge of freeze_request, reset synchronization
            if latched_freeze_req = '0' and i_freeze_request = '1' then
                synchronized         <= '0';
                tpiu_packets_handled <= 0;
                o_valid_frame        <= '0';
            -- New synch packet encountered while a frame was processing
            -- output an error
            elsif i_data = SYNCH_PACKET then
                if tpiu_packets_handled /= 0 then
                    o_frame_error <= '1';
                else
                    o_frame_error <= '0';
                end if;
                tpiu_packets_handled <= 0;
                o_valid_frame <= '0';
                synchronized  <= '1';
            else
                o_frame_error <= '0';
                -- Half-synch packet encountered, breaking the current frame
                if synchronized = '0' then
                    o_valid_frame <= '0';
                elsif i_data = HALF_SYNCH_PACKET then
                    o_valid_frame <= '0';
                -- Extract frame fata
                else
                    case tpiu_packets_handled is
                        when 0 =>
                            o_frame(31 downto 0) <= i_data;
                            tpiu_packets_handled <= 1;
                            o_valid_frame <= '0';
                        when 1 =>
                            o_frame(63 downto 32) <= i_data;
                            tpiu_packets_handled <= 2;
                            o_valid_frame <= '0';
                        when 2 =>
                            o_frame(95 downto 64) <= i_data;
                            tpiu_packets_handled <= 3;
                            o_valid_frame <= '0';
                        when 3 =>
                            o_frame(127 downto 96) <= i_data;
                            tpiu_packets_handled <= 0;
                            o_valid_frame <= '1';
                    end case;
            end if;
            end if;
        end if;
    end if;
    end process;


    axi_lite_process: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                arready <= '0';
                rresp   <= (others => '0');
                rvalid  <= '0';

                read_in_progress <= '0';
            else

                ----------------------------------------------------------------
                -- READ CHANNEL
                if arready = '0' and s_axi_arvalid = '1' and read_in_progress = '0' then
                    arready          <= '1';
                    araddr_reg       <= s_axi_araddr;
                    read_in_progress <= '1';
                else
                    arready <= '0';
                end if;

                if read_in_progress = '1' and rvalid = '0' then
                    rvalid <= '1';
                    rresp  <= "00";

                    case araddr_reg is
                        -- 0x00: synchronized
                        --   synchronized bit (1 done, 0 no sync packet found yet)
                        when x"00" =>
                            s_axi_rdata <= (31 downto 1 => '0') & synchronized;

                        when others =>
                            s_axi_rdata <= (others => '0');
                    end case;

                elsif rvalid = '1' and s_axi_rready = '1' then
                    rvalid           <= '0';
                    read_in_progress <= '0';
                end if;

            end if;
        end if;
    end process;


end Behavioral;
