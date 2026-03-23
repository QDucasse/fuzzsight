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
        FIFO_DEPTH : integer := 4
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

        -- Output edge index
        o_index   : out std_logic_vector(63 downto 0);

        -- Output fifo info
        o_valid   : out std_logic;
        o_empty   : out std_logic;
        i_ready   : in  std_logic
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
            if bits(i) = '0' then
                count := count + 1;
            end if;
        end loop;
        return count;
    end function;

    ---------------
    -- Procedures
    ---------------

    procedure process_atom(
        atom_valid   : in std_logic;
        addr         : in std_logic_vector(63 downto 0);
        atom_elts    : in std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
        atom_nb      : in unsigned(ATOM_NB_SIZE-1 downto 0);

        variable v_prev_slice  : inout std_logic_vector(63 downto 0);
        variable v_fifo        : inout fifo_t;
        variable v_fifo_wr_ptr : inout integer;
        variable v_fifo_count  : inout integer;
        variable v_index       : inout std_logic_vector(63 downto 0)
    ) is
        variable n_atom_count  : unsigned(ATOM_NB_SIZE-1 downto 0);
        variable addr_extended : unsigned(63 downto 0);
    begin
        if atom_valid = '1' and v_fifo_count < FIFO_DEPTH then
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

begin

    process(aclk)
        variable v_prev_slice  : std_logic_vector(63 downto 0);
        variable v_fifo        : fifo_t;
        variable v_fifo_wr_ptr : integer range 0 to FIFO_DEPTH-1;
        variable v_fifo_rd_ptr : integer range 0 to FIFO_DEPTH-1;
        variable v_fifo_count  : integer range 0 to FIFO_DEPTH;
        variable v_index       : std_logic_vector(63 downto 0);
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
            else

                v_fifo        := fifo;
                v_fifo_wr_ptr := fifo_wr_ptr;
                v_fifo_rd_ptr := fifo_rd_ptr;
                v_fifo_count  := fifo_count;
                v_prev_slice  := prev_slice;

                -- Sequentially process ports 0..3
                process_atom(i_atom_valid0, i_address_reg_0_0, i_atom_elements0, i_atom_nb0,
                             v_prev_slice, v_fifo, v_fifo_wr_ptr, v_fifo_count, v_index);
                process_atom(i_atom_valid1, i_address_reg_0_1, i_atom_elements1, i_atom_nb1,
                             v_prev_slice, v_fifo, v_fifo_wr_ptr, v_fifo_count, v_index);
                process_atom(i_atom_valid2, i_address_reg_0_2, i_atom_elements2, i_atom_nb2,
                             v_prev_slice, v_fifo, v_fifo_wr_ptr, v_fifo_count, v_index);
                process_atom(i_atom_valid3, i_address_reg_0_3, i_atom_elements3, i_atom_nb3,
                             v_prev_slice, v_fifo, v_fifo_wr_ptr, v_fifo_count, v_index);

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

            end if;
        end if;
    end process;
end Behavioral;