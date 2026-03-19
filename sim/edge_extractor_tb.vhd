library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity edge_extractor_tb is
-- Port ( );
end edge_extractor_tb;

architecture Simulation of edge_extractor_tb is

    -- constants
    constant ATOM_ELTS_SIZE : integer := 24;
    constant ATOM_NB_SIZE : integer := 5;

    -- Component definition

    component edge_extractor
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
    end component;

    -- Clock and reset
    signal clock  : std_logic := '1';
    signal reset  : std_logic := '1';

    -- Decoder outputs
    signal atom_valid0     : std_logic;
    signal address_reg_0_0 : std_logic_vector(63 downto 0);
    signal atom_elements0  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal atom_nb0        : unsigned(ATOM_NB_SIZE-1 downto 0);

    signal atom_valid1     : std_logic;
    signal address_reg_0_1 : std_logic_vector(63 downto 0);
    signal atom_elements1  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal atom_nb1        : unsigned(ATOM_NB_SIZE-1 downto 0);

    signal atom_valid2     : std_logic;
    signal address_reg_0_2 : std_logic_vector(63 downto 0);
    signal atom_elements2  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal atom_nb2        : unsigned(ATOM_NB_SIZE-1 downto 0);

    signal atom_valid3     : std_logic;
    signal address_reg_0_3 : std_logic_vector(63 downto 0);
    signal atom_elements3  : std_logic_vector(ATOM_ELTS_SIZE-1 downto 0);
    signal atom_nb3        : unsigned(ATOM_NB_SIZE-1 downto 0);

    -- FIFO handshake
    signal index : std_logic_vector(63 downto 0);
    signal valid : std_logic;
    signal ready : std_logic;

begin
    -- Clock and reset
    clock <= not clock after 1 ns;
    reset <= '0', '1' after 6 ns;

    -- DUT instantiation WITH SIGNALS
    DUT: edge_extractor port map (
        aclk     => clock,
        aresetn  => reset,

        -- Inputs from ETM decoder (4 sequential ports)
        i_atom_valid0     => atom_valid0,
        i_address_reg_0_0 => address_reg_0_0,
        i_atom_elements0  => atom_elements0,
        i_atom_nb0        => atom_nb0,

        i_atom_valid1     => atom_valid1,
        i_address_reg_0_1 => address_reg_0_1,
        i_atom_elements1  => atom_elements1,
        i_atom_nb1        => atom_nb1,

        i_atom_valid2     => atom_valid2,
        i_address_reg_0_2 => address_reg_0_2,
        i_atom_elements2  => atom_elements2,
        i_atom_nb2        => atom_nb2,

        i_atom_valid3     => atom_valid3,
        i_address_reg_0_3 => address_reg_0_3,
        i_atom_elements3  => atom_elements3,
        i_atom_nb3        => atom_nb3,

        -- Output edge index
        o_index => index,
        o_valid => valid,
        i_ready => ready
    );

    -- Simulation process
    simulation_process: process
    begin

        -- clear inputs
        atom_valid0      <= '0';
        address_reg_0_0  <= (others => '0');
        atom_elements0   <= (others => '0');
        atom_nb0         <= (others => '0');

        atom_valid1      <= '0';
        address_reg_0_1  <= (others => '0');
        atom_elements1   <= (others => '0');
        atom_nb1         <= (others => '0');

        atom_valid2      <= '0';
        address_reg_0_2  <= (others => '0');
        atom_elements2   <= (others => '0');
        atom_nb2         <= (others => '0');

        atom_valid3      <= '0';
        address_reg_0_3  <= (others => '0');
        atom_elements3   <= (others => '0');
        atom_nb3         <= (others => '0');

        -- wait for reset
        wait until reset = '1';

        -- Single atom packet on port 0
        atom_valid0     <= '1';
        address_reg_0_0 <= x"0000000000000100";
        atom_elements0  <= "000000000000000000000001"; -- first atom is E
        atom_nb0        <= "00001";                    -- 1 element
        ready           <= '1';
        wait for 2 ns;
        atom_valid0 <= '0';

        -- Simultaneous atoms on ports 1 and 2
        atom_valid1     <= '1';
        address_reg_0_1 <= x"0000000000000200";
        atom_elements1  <= "000000000000000000000011";
        atom_nb1        <= "00010";

        atom_valid2     <= '1';
        address_reg_0_2 <= x"0000000000000300";
        atom_elements2  <= "000000000000000000000101";
        atom_nb2        <= "00010";

        wait for 4 ns;
        atom_valid1 <= '0';
        atom_valid2 <= '0';

        wait;
    end process;
end Simulation;