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
        o_valid   : out std_logic;
        i_ready   : in  std_logic
    );
end entity;

architecture Behavioral of edge_extractor is

    signal prev_slice : std_logic_vector(63 downto 0) := (others => '0');

    type fifo_t is array(0 to FIFO_DEPTH-1) of std_logic_vector(63 downto 0);
    signal fifo        : fifo_t;
    signal fifo_wr_ptr : integer range 0 to FIFO_DEPTH-1 := 0;
    signal fifo_rd_ptr : integer range 0 to FIFO_DEPTH-1 := 0;
    signal fifo_count  : integer range 0 to FIFO_DEPTH := 0;

    signal index : std_logic_vector(63 downto 0);
    signal valid : std_logic := '0';

    function count_n_atoms(bits : std_logic_vector; nb : unsigned) return unsigned is
        variable count : unsigned(ATOM_NB_SIZE-1 downto 0) := (others => '0'); -- max 24
    begin
        for i in 0 to to_integer(nb)-1 loop
            if bits(i) = '0' then
                count := count + 1;
            end if;
        end loop;
        return count;
    end function;

begin

    process(aclk)
        variable n_atom_count : unsigned(ATOM_NB_SIZE-1 downto 0);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                prev_slice <= (others => '0');
                fifo_wr_ptr <= 0;
                fifo_rd_ptr <= 0;
                fifo_count <= 0;
                o_index <= (others => '0');
                o_valid <= '0';
            else
                valid <= '0';

                -- Sequentially process ports 0..3
                if i_atom_valid0 = '1' and fifo_count < FIFO_DEPTH then
                    n_atom_count := count_n_atoms(i_atom_elements0, i_atom_nb0);
                    index             <= std_logic_vector((unsigned(i_address_reg_0_0) + n_atom_count) xor unsigned(prev_slice));
                    prev_slice        <= std_logic_vector(shift_right(unsigned(i_address_reg_0_0) + n_atom_count, 1));
                    fifo(fifo_wr_ptr) <= index;
                    fifo_wr_ptr       <= (fifo_wr_ptr + 1) mod FIFO_DEPTH;
                    fifo_count        <= fifo_count + 1;
                end if;

                if i_atom_valid1 = '1' and fifo_count < FIFO_DEPTH then
                    n_atom_count := count_n_atoms(i_atom_elements1, i_atom_nb1);
                    index             <= std_logic_vector((unsigned(i_address_reg_0_1) + n_atom_count) xor unsigned(prev_slice));
                    prev_slice        <= std_logic_vector(shift_right(unsigned(i_address_reg_0_1) + n_atom_count, 1));
                    fifo(fifo_wr_ptr) <= index;
                    fifo_wr_ptr       <= (fifo_wr_ptr + 1) mod FIFO_DEPTH;
                    fifo_count        <= fifo_count + 1;
                end if;

                if i_atom_valid2 = '1' and fifo_count < FIFO_DEPTH then
                    n_atom_count := count_n_atoms(i_atom_elements2, i_atom_nb2);
                    index             <= std_logic_vector((unsigned(i_address_reg_0_2) + n_atom_count) xor unsigned(prev_slice));
                    prev_slice        <= std_logic_vector(shift_right(unsigned(i_address_reg_0_2) + n_atom_count, 1));
                    fifo(fifo_wr_ptr) <= index;
                    fifo_wr_ptr       <= (fifo_wr_ptr + 1) mod FIFO_DEPTH;
                    fifo_count        <= fifo_count + 1;
                end if;

                if i_atom_valid3 = '1' and fifo_count < FIFO_DEPTH then
                    n_atom_count := count_n_atoms(i_atom_elements3, i_atom_nb3);
                    index             <= std_logic_vector((unsigned(i_address_reg_0_3) + n_atom_count) xor unsigned(prev_slice));
                    prev_slice        <= std_logic_vector(shift_right(unsigned(i_address_reg_0_3) + n_atom_count, 1));
                    fifo(fifo_wr_ptr) <= index;
                    fifo_wr_ptr       <= (fifo_wr_ptr + 1) mod FIFO_DEPTH;
                    fifo_count        <= fifo_count + 1;
                end if;

                -- Output FIFO to downstream
                if fifo_count > 0 and i_ready = '1' then
                    o_index <= fifo(fifo_rd_ptr);
                    o_valid <= '1';
                    -- Update read pointer/count
                    fifo_rd_ptr <= (fifo_rd_ptr + 1) mod FIFO_DEPTH;
                    fifo_count  <= fifo_count - 1;
                else
                    o_valid <= '0';
                end if;
            end if;
        end if;
    end process;
end Behavioral;