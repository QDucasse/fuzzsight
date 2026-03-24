library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Bitmap Writer
--
-- This module plugs into one port of a true dual port BRAM. It supports two main
-- roles: exposing an AXI-Lite to the PS for control and setup, interfacing the
-- BRAM port to AXI-Stream, enabling DMA of the BRAM.
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

entity bitmap_reader_bram is
    generic (
        ADDR_WIDTH   : integer := 16; -- makes a 64KB map, AFL++ default
        AXIS_WIDTH   : integer := 32  -- 32-bit AXI-Stream transfer
    );
    port (
        aclk        : in  std_logic;
        aresetn     : in  std_logic;

        -- FIFO info
        i_fifo_empty      : in std_logic;
        o_fifo_freeze_req : out std_logic;

        -- BRAM interface
        bram_addr : out std_logic_vector(ADDR_WIDTH-1 downto 0);
        bram_din  : out std_logic_vector(31 downto 0);
        bram_dout : in  std_logic_vector(31 downto 0);
        bram_en   : out std_logic;
        bram_we   : out std_logic;

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

architecture Behavioral of bitmap_reader_bram is

    ---------------
    -- Constants
    ---------------
    constant MAP_SIZE   : integer := 2**ADDR_WIDTH;
    constant WORD_COUNT : integer := MAP_SIZE / 4;

    ---------------
    -- Types
    ---------------
    type dma_state_t   is (IDLE, STREAM, DONE);
    type clear_state_t is (C_IDLE, C_BUSY, C_DONE);

    ---------------
    -- Signals
    ---------------

    -- FIFO
    signal fifo_ready : std_logic;
    signal fifo_empty : std_logic;
    signal fifo_freeze_req : std_logic;

    -- AXIS
    signal tvalid : std_logic;

    -- Clear logic
    signal clear_busy       : std_logic := '0';
    signal clear_done_pulse : std_logic := '0';
    signal clear_done       : std_logic := '0'; -- sticky clear done pulse for AXI-Lite
    signal clear_state      : clear_state_t := C_IDLE;
    signal clear_addr       : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');

    -- DMA
    signal dma_start      : std_logic := '0';
    signal dma_busy       : std_logic := '0';
    signal dma_done       : std_logic := '0'; -- sticky dma done pulse for AXI-Lite
    signal dma_done_pulse : std_logic := '0';
    signal dma_state      : dma_state_t := IDLE;

    -- BRAM read pipeline
    signal addr_reg : unsigned(ADDR_WIDTH-1 downto 0);
    signal data_reg : std_logic_vector(31 downto 0);

    -- AXI-Lite requests/actions
    signal clear_bitmap : std_logic := '0';

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
    -- FIFO
    fifo_freeze_req <= dma_start;

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

    -- FIFO empty latch
    fifo_latch_process: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                fifo_empty <= '0';
            else
                if dma_start = '1' and fifo_empty = '0' then
                    fifo_empty <= i_fifo_empty;
                end if;
            end if;
        end if;
    end process;

    -- BRAM clear process and FSM
    bitram_clear_process: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                clear_busy  <= '0';
                clear_done  <= '0';
                clear_state <= C_IDLE;
                clear_addr  <= (others => '0');
                bram_en     <= '0';
                bram_we     <= '0';
            else
                case clear_state is
                    -- Waiting for clear_bitmap
                    when C_IDLE =>
                        clear_busy <= '0';
                        clear_done_pulse <= '0';

                        if (clear_bitmap = '1') and (dma_busy = '0') then
                            clear_addr  <= (others => '0');
                            clear_state <= C_BUSY;
                        end if;

                        bram_en <= '0';
                        bram_we <= '0';

                    -- Writes 0 then increments address
                    when C_BUSY =>
                        clear_busy <= '1';
                        clear_done_pulse <= '0';

                        bram_en    <= '1';
                        bram_we    <= '1';
                        bram_addr  <= std_logic_vector(clear_addr);
                        bram_din   <= (others => '0');

                        -- Advance address
                        if clear_addr = (2**ADDR_WIDTH - 1) then
                            clear_state <= C_DONE;
                        else
                            clear_addr <= clear_addr + 1;
                        end if;
                    -- BRAM completely cleared
                    when C_DONE =>
                        clear_done_pulse <= '1';
                        clear_busy       <= '0';

                        bram_en      <= '0';
                        bram_we      <= '0';
                        clear_state  <= C_IDLE;
                        clear_bitmap <= '0'; -- Auto-clear once done

                end case;
            end if;
        end if;
    end process;

    -- BRAM AXI-Stream interface for DMA
    axi_stream_process: process(aclk)
        variable first_read : std_logic := '1';
        variable v_data_valid : std_logic := '0';
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                tvalid        <= '0';
                m_axis_tlast  <= '0';
                dma_busy      <= '0';
                dma_done      <= '0';
                dma_state     <= IDLE;

                addr_reg <= (others=>'0');
                data_reg <= (others=>'0');
                v_data_valid := '0';
            else
                case dma_state is
                    -- Waiting for transfer
                    when IDLE =>
                        dma_busy       <= '0';
                        dma_done_pulse <= '0';

                        tvalid         <= '0';
                        m_axis_tlast   <= '0';
                        addr_reg       <= (others => '0');
                        v_data_valid   := '0';

                        --  Start if asked for and no clears are ongoing
                        if (dma_start = '1') and (clear_state = C_IDLE) and (fifo_empty = '1') then
                            dma_busy  <= '1';
                            dma_state <= STREAM;
                        end if;
                    -- Streaming BRAM to AXI-DMA
                    when STREAM =>
                        dma_busy       <= '1';
                        dma_done_pulse <= '0';

                        bram_en <= '1';
                        bram_we <= '0';

                        -- Issue read address (data read next cycle)
                        bram_addr <= std_logic_vector(addr_reg);

                        -- Register read data for AXIS output
                        if v_data_valid = '0' then
                            -- Read 4 bytes and pack into 32-bit word
                            data_reg     <= bram_dout;
                            v_data_valid := '1';
                        end if;

                        -- Drive AXIS
                        if v_data_valid = '1' then
                            m_axis_tdata <= data_reg;
                            tvalid       <= '1';

                            -- Check tlast
                            if addr_reg = WORD_COUNT-1 then
                                m_axis_tlast <= '1';
                            else
                                m_axis_tlast <= '0';
                            end if;
                        else
                            tvalid       <= '0';
                            m_axis_tlast <= '0';
                        end if;

                        -- Advance address when downstream consumes
                        if (v_data_valid = '0') or (m_axis_tready = '1') then
                            addr_reg <= addr_reg + 1;
                            -- Clear valid once consumed
                            v_data_valid := '0';

                            if addr_reg = WORD_COUNT then
                                dma_state <= DONE;
                            end if;
                        end if;

                    -- Transfer completed
                    when DONE =>
                        dma_busy       <= '0';
                        dma_done_pulse <= '1';

                        tvalid        <= '0';
                        m_axis_tlast  <= '0';
                        dma_state     <= IDLE;
                        clear_bitmap  <= '1'; -- Clear once DMA done
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
                -- Propagate pulse to sticky signals, only cleared after read
                if dma_done_pulse = '1' then
                    dma_done <= '1';
                end if;
                if clear_done_pulse = '1' then
                    clear_done <= '1';
                end if;

                ----------------------------------------------------------------
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
                        when x"04" => s_axi_rdata <= (31 downto 5 => '0') & clear_busy & clear_done & dma_busy &  dma_done & i_fifo_empty;

                        -- Fire clear status request to 0 clear_done/dma_done in their respective processes
                        dma_done   <= '1';
                        clear_done <= '1';

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