library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.decoder_constants.all;

-- Edge Extractor
--
-- This module gets the outputs of the CoreSight packet decoder and produces edge coverage indexes
-- as used by AFL++. To integrate CoreSight information into this edge coverage, it follows the
-- methodology defined by the paper "Efficiently rebuilding coverage in hardware-assisted greybox fuzzing."
-- presented at RAID'24: it adds the count of N atoms into the previous location address to
-- differentiate between the edges.
--
-- Due to the decoder presenting four ports (potentially 1 packet per port), an internal FIFO is used
-- and a simple ready/valid is provided to empty it.

entity edge_extractor is
    generic (
        FIFO_DEPTH : integer := 8;
        AXIL_WIDTH : integer := 8
    );
    port (
        aclk     : in  std_logic;
        aresetn  : in  std_logic;

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

        -- Freeze request
        i_freeze_request  : in std_logic;

        -- Output edge index
        o_index   : out std_logic_vector(63 downto 0);

        -- Output fifo info
        o_valid   : out std_logic;
        o_empty   : out std_logic;
        i_ready   : in  std_logic;

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

architecture Behavioral of edge_extractor is

    ---------------
    -- Types
    ---------------

    type fifo_t is array(0 to FIFO_DEPTH-1) of std_logic_vector(63 downto 0);

    ---------------
    -- Functions
    ---------------

    function count_n_atoms(bits : std_logic_vector; nb : unsigned) return unsigned is
        variable count : unsigned(ATOM_NB_SIZE-1 downto 0) := (others => '0'); -- max 24
    begin
        for i in 0 to ATOM_NB_SIZE-1 loop
            if i < to_integer(nb) and bits(i) = '0' then
                count := count + 1;
            end if;
        end loop;
        return count;
    end function;

    ---------------
    -- Procedures
    ---------------

    procedure process_atom(
        atom_valid : in std_logic;
        addr       : in std_logic_vector(63 downto 0);
        atom_elts  : in std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
        atom_nb    : in unsigned(ATOM_NB_SIZE-1 downto 0);
        freeze_req : in std_logic;

        variable v_prev_slice        : inout std_logic_vector(63 downto 0);
        variable v_fifo              : inout fifo_t;
        variable v_fifo_wr_ptr       : inout integer;
        variable v_fifo_count        : inout integer;
        variable v_index             : inout std_logic_vector(63 downto 0);
        variable v_edge_pulse        : inout std_logic;
        variable v_overflow_pulse    : inout std_logic;
        variable v_freeze_drop_pulse : inout std_logic
    ) is
        variable n_atom_count  : unsigned(ATOM_NB_SIZE-1 downto 0);
        variable addr_extended : unsigned(63 downto 0);
    begin
        if freeze_req = '1' then
            if atom_valid = '1' then
                v_freeze_drop_pulse := '1';
            end if;
        elsif atom_valid = '1' then
            if v_fifo_count < FIFO_DEPTH then
                v_edge_pulse := '1';
                -- Add the number of N elements to the address
                n_atom_count := count_n_atoms(atom_elts, atom_nb);
                addr_extended := unsigned(addr) + n_atom_count;
                -- Compute the index by XORing the address to the previous slice
                v_index := std_logic_vector(addr_extended xor unsigned(v_prev_slice));
                -- Update the previous slice with the new address >> 1
                v_prev_slice := std_logic_vector(shift_right(addr_extended, 1));
                -- Update FIFO
                v_fifo(v_fifo_wr_ptr) := v_index;
                v_fifo_wr_ptr := (v_fifo_wr_ptr + 1) mod FIFO_DEPTH;
                v_fifo_count  := v_fifo_count + 1;
            else -- FIFO full, edge dropped
                v_overflow_pulse := '1';
            end if;
        end if;
    end procedure;

    ---------------
    -- Signals
    ---------------

    signal prev_slice : std_logic_vector(63 downto 0) := (others => '0');
    signal fifo        : fifo_t;
    signal fifo_wr_ptr : integer range 0 to FIFO_DEPTH-1 := 0;
    signal fifo_rd_ptr : integer range 0 to FIFO_DEPTH-1 := 0;
    signal fifo_count  : integer range 0 to FIFO_DEPTH := 0;

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

    -- Stats
    -- owned by process_atom
    signal edge_pulse          : std_logic := '0';
    signal overflow_pulse      : std_logic := '0';
    signal freeze_drop_pulse   : std_logic := '0';
    -- owned by axi-lite
    signal edges_total         : std_logic_vector(31 downto 0) := (others => '0');
    signal fifo_overflow_count : std_logic_vector(31 downto 0) := (others => '0');
    signal freeze_drop_count   : std_logic_vector(31 downto 0) := (others => '0');

begin

    -- AXI4-Lite
    s_axi_awready <= awready;
    s_axi_wready  <= wready;
    s_axi_bvalid  <= bvalid;
    s_axi_bresp   <= bresp;

    s_axi_arready <= arready;
    s_axi_rvalid  <= rvalid;
    s_axi_rresp   <= rresp;


    process_packet: process(aclk)
        variable v_prev_slice        : std_logic_vector(63 downto 0);
        variable v_fifo              : fifo_t;
        variable v_fifo_wr_ptr       : integer range 0 to FIFO_DEPTH-1;
        variable v_fifo_rd_ptr       : integer range 0 to FIFO_DEPTH-1;
        variable v_fifo_count        : integer range 0 to FIFO_DEPTH;
        variable v_index             : std_logic_vector(63 downto 0);
        variable v_edge_pulse        : std_logic;
        variable v_overflow_pulse    : std_logic;
        variable v_freeze_drop_pulse : std_logic;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                -- Signals
                prev_slice  <= (others => '0');
                fifo_wr_ptr <= 0;
                fifo_rd_ptr <= 0;
                fifo_count  <= 0;

                -- Outputs
                o_index     <= (others => '0');
                o_valid     <= '0';
                o_empty     <= '1';

                -- Variables
                v_prev_slice  := (others=>'0');
                v_fifo_wr_ptr := 0;
                v_fifo_rd_ptr := 0;
                v_fifo_count  := 0;
                v_index       := (others=>'0');

                v_edge_pulse        := '0';
                v_overflow_pulse    := '0';
                v_freeze_drop_pulse := '0';
            else

                v_fifo        := fifo;
                v_fifo_wr_ptr := fifo_wr_ptr;
                v_fifo_rd_ptr := fifo_rd_ptr;
                v_fifo_count  := fifo_count;
                v_prev_slice  := prev_slice;

                v_edge_pulse        := '0';
                v_overflow_pulse    := '0';
                v_freeze_drop_pulse := '0';

                -- Sequentially process ports 0..3
                process_atom(i_atom_valid0, i_address_reg_0_0, i_atom_elements0, i_atom_nb0, i_freeze_request,
                             v_prev_slice, v_fifo, v_fifo_wr_ptr, v_fifo_count, v_index,
                             v_edge_pulse, v_overflow_pulse, v_freeze_drop_pulse);
                process_atom(i_atom_valid1, i_address_reg_0_1, i_atom_elements1, i_atom_nb1, i_freeze_request,
                             v_prev_slice, v_fifo, v_fifo_wr_ptr, v_fifo_count, v_index,
                             v_edge_pulse, v_overflow_pulse, v_freeze_drop_pulse);
                process_atom(i_atom_valid2, i_address_reg_0_2, i_atom_elements2, i_atom_nb2, i_freeze_request,
                             v_prev_slice, v_fifo, v_fifo_wr_ptr, v_fifo_count, v_index,
                             v_edge_pulse, v_overflow_pulse, v_freeze_drop_pulse);
                process_atom(i_atom_valid3, i_address_reg_0_3, i_atom_elements3, i_atom_nb3, i_freeze_request,
                             v_prev_slice, v_fifo, v_fifo_wr_ptr, v_fifo_count, v_index,
                             v_edge_pulse, v_overflow_pulse, v_freeze_drop_pulse);

                -- Output FIFO to downstream
                if v_fifo_count > 0 and i_ready = '1' then
                    o_index <= v_fifo(v_fifo_rd_ptr);
                    o_valid <= '1';
                    -- Update read pointer/count
                    v_fifo_rd_ptr := (v_fifo_rd_ptr + 1) mod FIFO_DEPTH;
                    v_fifo_count  := v_fifo_count - 1;
                else
                    o_valid <= '0';
                end if;

                -- Update empty flag
                if v_fifo_count = 0 then
                    o_empty <= '1';
                else
                    o_empty <= '0';
                end if;

                -- Commit variables to the signals
                fifo        <= v_fifo;
                fifo_wr_ptr <= v_fifo_wr_ptr;
                fifo_rd_ptr <= v_fifo_rd_ptr;
                fifo_count  <= v_fifo_count;
                prev_slice  <= v_prev_slice;

                edge_pulse        <= v_edge_pulse;
                overflow_pulse    <= v_overflow_pulse;
                freeze_drop_pulse <= v_freeze_drop_pulse;

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
                edges_total         <= (others => '0');
                fifo_overflow_count <= (others => '0');
                freeze_drop_count   <= (others => '0');
            else
                -- Saturating counters
                if edge_pulse = '1' then
                    if edges_total /= x"FFFFFFFF" then
                        edges_total <= std_logic_vector(unsigned(edges_total) + 1);
                    end if;
                end if;

                if overflow_pulse = '1' then
                    if fifo_overflow_count /= x"FFFFFFFF" then
                        fifo_overflow_count <= std_logic_vector(unsigned(fifo_overflow_count) + 1);
                    end if;
                end if;

                if freeze_drop_pulse = '1' then
                    if freeze_drop_count /= x"FFFFFFFF" then
                        freeze_drop_count <= std_logic_vector(unsigned(freeze_drop_count) + 1);
                    end if;
                end if;

                ----------------------------------------------------------------
                -- WRITE CHANNEL
                if awready = '0' and s_axi_awvalid = '1' then
                    awready    <= '1';
                    awaddr_reg <= s_axi_awaddr;
                    aw_seen    <= '1';
                else
                    awready <= '0';
                end if;

                if wready = '0' and s_axi_wvalid = '1' then
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
                        --   bit 0 = stats_reset: clear edges_total, fifo_overflow_count and freeze_drop_count
                        when x"00" =>
                            if wdata_reg(0) = '1' then
                                edges_total         <= (others => '0');
                                fifo_overflow_count <= (others => '0');
                                freeze_drop_count   <= (others => '0');
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
                        -- 0x04: edges_total
                        --   32-bit saturating count of edges produced since reset
                        when x"04" =>
                            s_axi_rdata <= edges_total;

                        -- 0x08: fifo_overflow_count
                        --   32-bit saturating count of edges dropped due to FIFO full
                        when x"08" =>
                            s_axi_rdata <= fifo_overflow_count;

                        -- 0x0C: freeze_drop_count
                        --   32-bit saturating count of edges dropped due to freeze request
                        when x"0C" =>
                            s_axi_rdata <= freeze_drop_count;

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