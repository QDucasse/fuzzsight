library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- AXI Interface
--
-- AXI-Lite interface with a control register, able to launch an 8-cycle long soft reset.
-- Status register with error counts from the decoder.

entity axi_interface is
    generic (
        COUNTER_W  : integer := 32;
        ADDR_WIDTH : integer := 8
    );
    port (
        aclk    : in std_logic;
        aresetn : in std_logic;

        -- Errors from members of the decoder
        i_frame_error               : in  std_logic;
        i_bytestream_gen_error      : in  std_logic;
        i_bytestream_demux_id_error : in  std_logic;

        -- Soft reset
        o_soft_reset : out std_logic;

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
end axi_interface;

architecture Behavioral of axi_interface is
    -- Error counters
    signal frame_error_count              : std_logic_vector(COUNTER_W-1 downto 0);
    signal bytestream_gen_error_count     : std_logic_vector(COUNTER_W-1 downto 0);
    signal bytesteam_demux_id_error_count : std_logic_vector(COUNTER_W-1 downto 0);
    signal stats_reset : std_logic := '0';

    signal soft_reset_sr : std_logic_vector(7 downto 0) := (others => '0');
    signal soft_reset_in_progress : std_logic := '0';
    signal soft_reset_done_sticky : std_logic := '1';  -- starts 1, no reset pending


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

    s_axi_awready <= awready;
    s_axi_wready  <= wready;
    s_axi_bvalid  <= bvalid;
    s_axi_bresp   <= bresp;

    s_axi_arready <= arready;
    s_axi_rvalid  <= rvalid;
    s_axi_rresp   <= rresp;

    -- Soft reset is active while any bit in the shift register is set
    o_soft_reset <= soft_reset_sr(0);

    counter_process: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                frame_error_count              <= (others => '0');
                bytestream_gen_error_count     <= (others => '0');
                bytesteam_demux_id_error_count <= (others => '0');
            else
                -- Reset takes priority over increment
                if stats_reset = '1' then
                    frame_error_count              <= (others => '0');
                    bytestream_gen_error_count     <= (others => '0');
                    bytesteam_demux_id_error_count <= (others => '0');
                else
                    if i_frame_error = '1' and frame_error_count /= x"FFFFFFFF" then
                        frame_error_count <= std_logic_vector(unsigned(frame_error_count) + 1);
                    end if;
                    if i_bytestream_gen_error = '1' and bytestream_gen_error_count /= x"FFFFFFFF" then
                        bytestream_gen_error_count <= std_logic_vector(unsigned(bytestream_gen_error_count) + 1);
                    end if;
                    if i_bytestream_demux_id_error = '1' and bytesteam_demux_id_error_count /= x"FFFFFFFF" then
                        bytesteam_demux_id_error_count <= std_logic_vector(unsigned(bytesteam_demux_id_error_count) + 1);
                    end if;
                end if;
            end if;
        end if;
    end process;

    axi_lite_process: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                awready          <= '0';
                wready           <= '0';
                bvalid           <= '0';
                arready          <= '0';
                rvalid           <= '0';

                awaddr_reg       <= (others => '0');
                araddr_reg       <= (others => '0');
                wdata_reg        <= (others => '0');
                s_axi_rdata      <= (others => '0');

                aw_seen          <= '0';
                w_seen           <= '0';
                read_in_progress <= '0';

                stats_reset      <='0';

                soft_reset_sr          <= (others => '0');
                soft_reset_in_progress <= '0';
                soft_reset_done_sticky <= '1';
            else

                stats_reset <= '0'; -- Default

                -- Shift the reset register down every cycle
                soft_reset_sr <= '0' & soft_reset_sr(7 downto 1);

                ----------------------------------------------------------------
                -- WRITE CHANNEL
                if awready = '0' and s_axi_awvalid = '1' and bvalid = '0' then
                    awready    <= '1';
                    awaddr_reg <= s_axi_awaddr;
                    aw_seen    <= '1';
                else
                    awready <= '0';
                end if;

                if wready = '0' and s_axi_wvalid = '1' and bvalid = '0' then
                    wready    <= '1';
                    wdata_reg <= s_axi_wdata;
                    w_seen    <= '1';
                else
                    wready <= '0';
                end if;

                if aw_seen = '1' and w_seen = '1' and bvalid = '0' then
                    bvalid <= '1';
                    bresp  <= "00";

                    case awaddr_reg is
                        -- 0x00: Control register
                        --   bit 0 = stats_reset: clear all error counts
                        --   bit 1 = soft_reset: reset decoder
                        when x"00" =>
                            if wdata_reg(0) = '1' then
                                stats_reset <= '1';
                            end if;
                            if wdata_reg(1) = '1' then
                                soft_reset_sr <= (others => '1'); -- load 8 cycles
                                soft_reset_done_sticky <= '0';
                                soft_reset_in_progress <= '1';
                            end if;
                        when others => null;
                    end case;

                elsif bvalid = '1' and s_axi_bready = '1' then
                    bvalid  <= '0';
                    aw_seen <= '0';
                    w_seen  <= '0';
                end if;

                ----------------------------------------------------------------
                -- READ CHANNEL

                if arready = '0' and s_axi_arvalid = '1' and read_in_progress = '0' then
                    araddr_reg       <= s_axi_araddr;
                    read_in_progress <= '1';
                    arready          <= '1';
                elsif rvalid = '1' and s_axi_rready = '1' then
                    arready          <= '0';
                    rvalid           <= '0';
                    read_in_progress <= '0';
                else
                    arready <= '0';
                end if;

                if read_in_progress = '1' and rvalid = '0' then
                    rvalid <= '1';
                    rresp  <= "00";

                    case araddr_reg is
                        -- 0x04: frame_error_count
                        --   32-bit saturating count of frames dropped while synchronizing
                        when x"04" =>
                            s_axi_rdata <= std_logic_vector(frame_error_count);

                        -- 0x08: bytestream_gen_error_count
                        --   32-bit saturating count of valid frames coming while the previous has not
                        --   been processed
                        when x"08" =>
                            s_axi_rdata <= std_logic_vector(bytestream_gen_error_count);

                        -- 0x0C: bytesteam_demux_error_count
                        --   32-bit saturating count of unknown IDs when demuxing packets
                        when x"0C" =>
                            s_axi_rdata <= std_logic_vector(bytesteam_demux_id_error_count);

                        -- 0x10: status register
                        --   bit 0 = soft_reset_done: '1' when reset complete
                        when x"10" =>
                            s_axi_rdata <= (31 downto 1 => '0') & soft_reset_done_sticky;
                        when others =>
                            s_axi_rdata <= (others => '0');
                    end case;
                end if;

                if soft_reset_in_progress = '1' and soft_reset_sr = "00000000" then
                    soft_reset_done_sticky  <= '1';
                    soft_reset_in_progress  <= '0';
                end if;

            end if;
        end if;
    end process;



end Behavioral;
