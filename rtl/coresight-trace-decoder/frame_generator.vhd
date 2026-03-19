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
    port (
        aclk    : in std_logic;
        aresetn : in std_logic;
        i_data  : in std_logic_vector (31 downto 0);

        o_frame       : out std_logic_vector(127 downto 0);
        o_valid_frame : out std_logic;
        o_frame_error : out std_logic
    );
end frame_generator;

architecture Behavioral of frame_generator is

    constant SYNCH_PACKET : std_logic_vector(31 downto 0) := X"7FFFFFFF";
    constant HALF_SYNCH_PACKET : std_logic_vector(31 downto 0) := X"7FFF7FFF";
    constant EMPTY_FRAME : std_logic_vector(127 downto 0) := X"00000000000000000000000000000000";

    -- 4 cycle counter
    signal tpiu_packets_handled : integer range 0 to 3;

    begin

    generator : process (ACLK) begin
    if rising_edge(ACLK) then
        -- Active low
        if aresetn = '0' then
            tpiu_packets_handled <= 0;

            o_frame       <= EMPTY_FRAME;
            o_valid_frame <= '0';
            o_frame_error <= '0';
        else
            -- New synch packet encountered while a frame was processing
            -- output an error
            if i_data = SYNCH_PACKET then
                if tpiu_packets_handled /= 0 then
                    o_frame_error <= '1';
                else
                    o_frame_error <= '0';
                end if;
                tpiu_packets_handled <= 0;
                o_valid_frame <= '0';
            else
                o_frame_error <= '0';
                -- Half-synch packet encountered, breaking the current frame
                if i_data = HALF_SYNCH_PACKET then
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


end Behavioral;
