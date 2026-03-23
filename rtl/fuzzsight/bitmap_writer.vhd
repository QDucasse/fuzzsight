library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Bitmap Writer
--
-- This module is composed of three processes:
-- edge_update_process:
-- Keeps the bitmap of the covered edges in BRAM, updating counters based on the
-- incoming edges from the edge_extractor.
--
-- axi_stream_process:
-- Once the coverage extraction is done, it exposes the bitmap over an AXI-Stream
-- interface to be written back into PS through DMA.
--
-- axi_lite_process:
-- Exposes AXI-Lite registers to read and write, gathering information from the
-- fuzzer to know when the child exists (stop updating edges or waiting for new
-- packets) and return back when the edges are all cleared from the pipeline.
-- The PS is then expected to DMA the bitmap.

entity bitmap_writer is
    generic (
        ADDR_WIDTH   : integer := 16; -- makes a 64KB map, AFL++ default
        AXIS_WIDTH   : integer := 32  -- 32-bit AXI-Stream transfer
    );
    port (
        aclk        : in  std_logic;
        aresetn     : in  std_logic;

        -- FIFO interface from edge extractor
        i_fifo_index  : in  std_logic_vector(63 downto 0);
        i_fifo_valid  : in  std_logic;
        i_fifo_empty  : in  std_logic;
        o_fifo_ready  : out std_logic;

        -- AXI-Stream interface
        m_axis_tready : in std_logic;
        m_axis_tdata  : out std_logic_vector(AXIS_WIDTH-1 downto 0);
        m_axis_tvalid : out std_logic;
        m_axis_tlast  : out std_logic;

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

architecture Behavioral of bitmap_writer is

    ---------------
    -- Constants
    ---------------
    constant MAP_SIZE: integer := 2**ADDR_WIDTH;

    ---------------
    -- Types
    ---------------
    type bram_t is array(0 to MAP_SIZE-1) of std_logic_vector(7 downto 0);
    type dma_state_t is (IDLE, STREAM, DONE);

    ---------------
    -- Signals
    ---------------
    signal bram_mem : bram_t := (others => (others => '0'));

    -- FIFO
    signal fifo_ready : std_logic;

    -- AXIS
    signal tvalid : std_logic;

    -- DMA
    signal dma_start : std_logic := '0';
    signal dma_busy  : std_logic := '0';
    signal dma_done  : std_logic := '0';

    -- AXI-Lite requests/actions
    signal clear_bitmap     : std_logic := '0';

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

    -- AXI Lite flags
    signal read_in_progress : std_logic := '0';
    signal w_seen           : std_logic := '0';
    signal aw_seen          : std_logic := '0';

begin

    -- FIFO output
    o_fifo_ready <= fifo_ready;

    -- AXI-Stream output
    m_axis_tvalid <= tvalid; -- Needed because we read it too!

    -- AXI-Lite outputs
    s_axi_awready <= awready;
    s_axi_wready  <= wready;
    s_axi_bvalid  <= bvalid;
    s_axi_bresp   <= bresp;

    s_axi_arready <= arready;
    s_axi_rvalid  <= rvalid;
    s_axi_rresp   <= rresp;

    -- BRAM edge writing process
    edge_update_process: process(aclk)
        variable v_addr : integer;
        variable v_curr : unsigned(7 downto 0);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                fifo_ready <= '1';
                bram_mem   <= (others => (others => '0'));
            else
                -- clear the bitmap if requested
                if clear_bitmap = '1' and dma_busy = '0' then
                    bram_mem <= (others => (others => '0'));
                    -- stall FIFO while clearing
                    fifo_ready <= '0';

                elsif i_fifo_valid = '1' and fifo_ready = '1' then
                    -- compute address (truncate to ADDR_WIDTH)
                    v_addr := to_integer(unsigned(i_fifo_index(ADDR_WIDTH-1 downto 0)));

                    -- read-modify-write increment, saturating counters to 256
                    v_curr := unsigned(bram_mem(v_addr));
                    if v_curr = 255 then
                        bram_mem(v_addr) <= std_logic_vector(v_curr);
                    else
                        bram_mem(v_addr) <= std_logic_vector(v_curr + 1);
                    end if;

                    fifo_ready <= '1';
                else
                    -- No valid input, keep ready high
                    fifo_ready <= '1';
                end if;
            end if;
        end if;
    end process;

    -- BRAM AXI-Stream interface for DMA
    axi_stream_process: process(aclk)
        variable state     : dma_state_t := IDLE;
        variable word_idx  : integer range 0 to MAP_SIZE-1 := 0;
        variable next_last : std_logic;
        variable i         : integer;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                tvalid        <= '0';
                m_axis_tlast  <= '0';
                dma_busy      <= '0';
                dma_done      <= '0';

                word_idx      := 0;
                state         := IDLE;
            else
                case state is
                    -- Waiting for transfer
                    when IDLE =>
                        tvalid <= '0';
                        m_axis_tlast  <= '0';
                        dma_done      <= '0';
                        dma_busy      <= '0';
                        word_idx      := 0;

                        if dma_start = '1' then
                            dma_busy <= '1';
                            state    := STREAM;
                        end if;

                    -- Streaming BRAM to AXI-DMA
                    when STREAM =>
                        -- Load data if not valid or downstream ready
                        if (tvalid = '0') or (m_axis_tready = '1') then
                            -- Pack bytes into AXIS WIDTH
                            for i in 0 to (AXIS_WIDTH/8)-1 loop
                                if (word_idx + i) < MAP_SIZE then
                                    m_axis_tdata((i+1)*8-1 downto i*8) <= bram_mem(word_idx + i);
                                else
                                    m_axis_tdata((i+1)*8-1 downto i*8) <= (others => '0');
                                end if;
                            end loop;

                            -- Compute TLAST
                            if (word_idx + (AXIS_WIDTH/8)) >= MAP_SIZE then
                                next_last := '1';
                            else
                                next_last := '0';
                            end if;

                            m_axis_tlast  <= next_last;
                            tvalid <= '1';

                            -- Advance only if downstream ready
                            if m_axis_tready = '1' then
                                word_idx := word_idx + (AXIS_WIDTH/8);

                                if next_last = '1' then
                                    state    := DONE;
                                end if;
                            end if;
                        end if;

                    -- Transfer completed
                    when DONE =>
                        tvalid        <= '0';
                        m_axis_tlast  <= '0';
                        dma_busy      <= '0';
                        dma_done      <= '1';
                        word_idx      := 0;
                        state         := IDLE;
                end case;
            end if;
        end if;
    end process;


    -- AXI-Lite interface
    axi_lite_process: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                awready <= '0';
                wready  <= '0';
                bvalid  <= '0';
                arready <= '0';
                rvalid  <= '0';
                aw_seen <= '0';
                w_seen  <= '0';
                read_in_progress <= '0';
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
                        dma_start    <= wdata_reg(0);
                        clear_bitmap <= wdata_reg(1);
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
                        --   0x00: Control (write only)
                        --   0x04: Status register (bit 0 = dma_busy, bit 1 = dma_done, bit 2 = fifo_empty)

                        -- reset bit not readable as it is self-clearing
                        when x"04" => s_axi_rdata <= (31 downto 3 => '0') & dma_busy & dma_done & i_fifo_empty;

                        when others => s_axi_rdata <= (others=>'0');
                    end case;

                elsif (rvalid = '1' and s_axi_rready = '1') then
                    rvalid <= '0';
                    read_in_progress <= '0';
                end if;
            end if;
        end if;
    end process;

end Behavioral;