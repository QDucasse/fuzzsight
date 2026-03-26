library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity bitmap_reader_bram is
    generic (
        ADDR_WIDTH : integer := 16; -- makes a 64KB map, AFL++ default
        AXIL_WIDTH : integer := 8;  -- address length for AXI-Lite register
        AXIS_WIDTH : integer := 32  -- 32-bit AXI-Stream transfer
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
end entity;

architecture Behavioral of bitmap_reader_bram is

    ---------------
    -- Constants
    ---------------
    constant MAP_SIZE   : integer := 2**ADDR_WIDTH;
    constant WORD_COUNT : integer := MAP_SIZE / 4;

    ---------------
    -- Types
    ---------------
    type dma_state_t   is (IDLE, READING, STREAM, DONE);
    type clear_state_t is (C_IDLE, C_BUSY, C_DONE);

    ---------------
    -- Signals
    ---------------

    -- AXIS
    signal tvalid : std_logic := '0';

    -- DMA
    signal dma_busy       : std_logic := '0';
    signal dma_done       : std_logic := '0'; -- sticky dma done pulse for AXI-Lite
    signal dma_done_pulse : std_logic := '0';
    signal dma_state      : dma_state_t := IDLE;
    -- handshake
    signal dma_req_set    : std_logic := '0'; -- driven by AXI-Lite process
    signal dma_req_clr    : std_logic := '0'; -- driven by DMA process
    signal dma_request    : std_logic := '0'; -- driven by handshake process

    -- BRAM read register (1-cycle latency)
    signal addr_reg : unsigned(ADDR_WIDTH-1 downto 0) := (others=>'0');

    -- AXI4-Lite signals
    signal awready : std_logic := '0';
    signal wready  : std_logic := '0';
    signal bvalid  : std_logic := '0';
    signal bresp   : std_logic_vector(1 downto 0) := (others=>'0');

    signal arready : std_logic := '0';
    signal rvalid  : std_logic := '0';
    signal rresp   : std_logic_vector(1 downto 0) := (others=>'0');

    signal awaddr_reg : std_logic_vector(AXIL_WIDTH-1 downto 0);
    signal araddr_reg : std_logic_vector(AXIL_WIDTH-1 downto 0);
    signal wdata_reg  : std_logic_vector(31 downto 0);

    -- AXI4-Lite flags
    signal read_in_progress : std_logic := '0';
    signal w_seen           : std_logic := '0';
    signal aw_seen          : std_logic := '0';
begin

    -- Freeze request sent to the updstream fifo
    o_fifo_freeze_req <= '1' when dma_state /= IDLE else '0';

    -- AXI-Stream
    m_axis_tvalid <= tvalid; -- Needed because we read it too!

    -- AXI4-Lite
    s_axi_awready <= awready;
    s_axi_wready  <= wready;
    s_axi_bvalid  <= bvalid;
    s_axi_bresp   <= bresp;

    s_axi_arready <= arready;
    s_axi_rvalid  <= rvalid;
    s_axi_rresp   <= rresp;

    -- BRAM combinatorial
    -- Mux to use DMA or CLEAR process
    bram_en   <= '1' when (dma_state = READING or (dma_state = STREAM and addr_reg < WORD_COUNT)) else '0';
    bram_addr <=  std_logic_vector(addr_reg);

    -- Clear needs to write 0s
    bram_we <= '1' when (dma_state = READING or (dma_state = STREAM and m_axis_tready = '1')) else '0';
    bram_din  <= (others => '0');

    -- Handshake between the DMA and AXI-Lite process
    dma_req_process: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                dma_request <= '0';
            elsif dma_req_set = '1' then
                dma_request <= '1';
            elsif dma_req_clr = '1' then
                dma_request <= '0';
            end if;
        end if;
    end process;

    -- DMA FSM
    dma_process: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                dma_busy       <= '0';
                dma_done_pulse <= '0';
                dma_state      <= IDLE;
                tvalid         <= '0';
                m_axis_tlast   <= '0';
                m_axis_tdata   <= (others => '0');
                addr_reg       <= (others => '0');
            else
                -- Default reset of the pulse, kept in the AXI-Lite exposed sticky reg
                dma_done_pulse <= '0';

                -- Default reset of the request clear
                dma_req_clr <= '0';

                case dma_state is

                    -- IDLE: Waiting for DMA request
                    when IDLE =>
                        dma_busy     <= '0';
                        tvalid       <= '0';
                        m_axis_tlast <= '0';
                        addr_reg     <= (others => '0');

                        -- Start if requested, no ongoing clear, and upstream fifo drained
                        if dma_request = '1' and i_fifo_empty = '1' then
                            dma_busy    <= '1';
                            dma_req_clr <= '1';
                            dma_state   <= READING;
                        end if;

                    -- READING: Account for the 1-cycle BRAM read latency. Issue the
                    -- first read here, for the data to be valid in STREAM.
                    when READING =>
                        addr_reg  <= addr_reg + 1;
                        dma_state <= STREAM;
                        dma_busy  <= '1';

                    -- STREAM: AXI-Stream handshake with the read data. Latency of
                    -- 1 cycle is accounted with the READING state.
                    when STREAM =>
                        dma_busy  <= '1';

                        -- bram_dout is valid here due to READING cycle
                        m_axis_tdata <= bram_dout;
                        tvalid       <= '1';

                        if addr_reg = WORD_COUNT then
                            m_axis_tlast <= '1';
                        else
                            m_axis_tlast <= '0';
                        end if;

                        -- Advance only when downstream consumes, issue next
                        -- read speculatively to maintain full throughput
                        if m_axis_tready = '1' then
                            if addr_reg = WORD_COUNT then
                                dma_state <= DONE;
                            else
                                addr_reg  <= addr_reg + 1;
                            end if;
                        end if;

                    -- DONE: Transfer completed
                    when DONE =>
                        dma_busy       <= '0';
                        dma_done_pulse <= '1';
                        tvalid         <= '0';
                        m_axis_tlast   <= '0';
                        dma_state      <= IDLE;
                end case;
            end if;
        end if;
    end process;

    -- AXI-Lite
    axi_lite_process: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                awready          <= '0';
                wready           <= '0';
                bvalid           <= '0';
                arready          <= '0';
                rvalid           <= '0';
                aw_seen          <= '0';
                w_seen           <= '0';
                read_in_progress <= '0';
            else
                -- clear requests
                dma_req_set   <= '0';

                -- Sticky pulses, cleared on PS read of status register
                if dma_done_pulse = '1' then
                    dma_done <= '1';
                end if;

                ----------------------------------------------------------------
                -- WRITE CHANNEL

                -- Accept address
                if awready = '0' and s_axi_awvalid = '1' then
                    awready    <= '1';
                    awaddr_reg <= s_axi_awaddr;
                    aw_seen    <= '1';
                else
                    awready    <= '0';
                end if;

                -- Accept data
                if wready = '0' and s_axi_wvalid = '1' then
                    wready    <= '1';
                    wdata_reg <= s_axi_wdata;
                    w_seen    <= '1';
                else
                    wready    <= '0';
                end if;

                -- Generate write response
                if aw_seen = '1' and w_seen = '1' and bvalid = '0' then
                    bvalid <= '1';
                    bresp  <= "00";

                    case awaddr_reg is
                        -- 0x00: Control register
                        --   bit 0 = dma_req_set: trigger DMA readout
                        when x"00" =>
                            dma_req_set   <= wdata_reg(0);
                        when others => null;
                    end case;

                elsif (bvalid = '1' and s_axi_bready = '1') then
                    bvalid  <= '0';
                    aw_seen <= '0';
                    w_seen  <= '0';
                end if;

                ----------------------------------------------------------------
                -- READ CHANNEL

                -- Accept address
                if arready = '0' and s_axi_arvalid = '1' and read_in_progress = '0' then
                    arready          <= '1';
                    araddr_reg       <= s_axi_araddr;
                    read_in_progress <= '1';
                else
                    arready          <= '0';
                end if;

                -- Provide data
                if read_in_progress = '1' and rvalid = '0' then
                    rvalid <= '1';
                    rresp  <= "00";

                    case araddr_reg is
                        -- 0x04: Status register
                        --   bit 0 = i_fifo_empty:  FIFO drained, safe to DMA
                        --   bit 1 = dma_done:      DMA readout complete
                        --                          (sticky, cleared on read)
                        --   bit 2 = dma_busy:      DMA in progress
                        --   bit 3 = clear_done:    bitmap clear complete
                        --                          (sticky, cleared on read)
                        --   bit 4 = clear_busy:    clear in progress
                        when x"04" =>
                            s_axi_rdata <= (31 downto 3 => '0')
                                         & dma_busy
                                         & dma_done
                                         & i_fifo_empty;

                            -- Clear out the values when read
                            dma_done   <= '0';

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