library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Byte Stream Compactor
--
-- Removes "holes" (invalid bytes) from the stream generator output, when OUT_KEEP(i) = 0.
-- Repacks the remaining valid bytes into a continuous 32-bit stream. Stage one compacts
-- within each 32-bit cycle while stage 2 compacts across cycles, emitting aligned 4-byte
-- outputs.
--
-- Example:
--  Cycle | IN_KEEP | IN_DATA bytes | Output (stage 1) | Output (stage 2)
--  ----------------------------------------------------------------------
--    1   |  0011   |  A B C D      |      C D         |        -
--    2   |  1100   |  A B C D      |      A B         |     C D A B
--    3   |  0001   |  A B C D      |       D          |        -
--    4   |  1110   |  A B C D      |     A B C        |     D A B C
--
-- Important IOs:
-- i_data: actual raw bytes
-- i_keep: valid packet?
-- o_data: raw bytes
-- o_valid: valid output bytes

entity byte_stream_compactor is
    port (
        aclk    : in std_logic;
        aresetn : in std_logic;

        i_data  : in std_logic_vector (31 downto 0);
        i_keep  : in std_logic_vector(3 downto 0);

        o_data  : out std_logic_vector (31 downto 0);
        o_valid : out std_logic
    );
end byte_stream_compactor;

architecture Behavioral of byte_stream_compactor is

    -- Helper functions

    -- stdlogic to int
    pure function toInt(x : std_logic) return integer is
    begin
        if x = '1' then
            return 1;
        else
            return 0;
        end if;
    end;

    -- Computes how many bytes are marked as kept before index
    -- Used to determine the byte position after compaction
    pure function getShiftValue(keep : std_logic_vector(3 downto 0); index  : integer) return integer is
    begin
        case index is
        when 0 =>
            return 0;
        when 1 =>
            return toInt(keep(0));
        when 2 =>
            return toInt(keep(0)) + toInt(keep(1));
        when others  =>
            return toInt(keep(0)) + toInt(keep(1)) + toInt(keep(2));
        end case;
    end;

    -- Computes the number of kept bytes
    pure function size(keep : std_logic_vector(3 downto 0)) return integer is
    begin
       return toInt(keep(0)) + toInt(keep(1)) + toInt(keep(2)) + toInt(keep(3));
    end;

    -- Signals

    signal data_valid : std_logic;
    signal data_buffer : std_logic_vector(63 downto 0);
    signal data_buffer_size : integer range 0 to 8;

    signal out_stage1 : std_logic_vector(31 downto 0);
    signal out_size_stage1 : integer;

    signal out_stage2 : std_logic_vector(63 downto 0);
    signal out_size_stage2 : integer;


-- Compacts byte from an incoming word, sending over the valid
-- compacted bytes to stage 2.
begin
stage1 : process(aclk) begin
    if rising_edge(aclk) then
        if aresetn = '0' then
            out_stage1 <= X"00000000";
            out_size_stage1 <= 0;
        else
            -- Iterates over the 4 bytes. For each byte where keep is high, store the input
            -- byte to the first available position (found through getShiftValue)
            for i in 0 to 3 loop
                if i_keep(i) = '1' then
                    out_stage1((getShiftValue(i_keep, i) + 1) * 8 - 1 downto getShiftValue(i_keep, i) * 8) <= i_data((i + 1) * 8 - 1  downto  i * 8);
                end if;
            end loop;
            out_size_stage1 <= size(i_keep);
        end if;
    end if;
end process;


-- Buffers output from stage 1, sending to the overall output if a complete word
-- of compacted bytes is available.
stage2 : process(aclk) begin
    if rising_edge(aclk) then
        if aresetn = '0' then
            out_stage2 <= X"0000000000000000";
            out_size_stage2 <= 0;
            o_valid <= '0';
        else
            -- Valid word of compacted bytes, send it
            if out_size_stage2 >= 4 then
                o_valid <= '1';
                o_data <= out_stage2(31 downto 0);
                out_stage2(31 downto 0) <= out_stage2(63 downto 32);
                out_stage2((out_size_stage2 - 4) * 8 + 31 downto (out_size_stage2 - 4) * 8) <=  out_stage1;
                out_size_stage2 <= out_size_stage2 - 4 + out_size_stage1;
            -- No valid word yet, wait for stage 1
            else
                o_valid <= '0';
                out_stage2((out_size_stage2) * 8 + 31 downto (out_size_stage2) * 8) <=  out_stage1;
                out_size_stage2 <= out_size_stage2 + out_size_stage1;
            end if;
        end if;
    end if;
end process;

end Behavioral;
