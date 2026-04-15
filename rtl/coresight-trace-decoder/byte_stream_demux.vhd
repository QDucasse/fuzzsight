library ieee;
use ieee.std_logic_1164.all;

-- Byte stream demultiplexer
--
-- Forwards the input data and sets the corresponding keep bit
-- for a given ETM.
--
-- Important IOs:
-- i_data     - The raw 32 bit packet
-- i_ids      - Concatenation of the four 7 bit IDs (one per ETM)
-- o_data    - Forwarded input data

entity byte_stream_demux is
    port (
        aclk : in std_logic;
        aresetn : in std_logic;

        i_soft_reset : in std_logic;

        i_data : in std_logic_vector(31 downto 0);
        i_ids  : in std_logic_vector(27 downto 0);
        i_keep: in std_logic_vector(3 downto 0);
        i_valid : in std_logic;

        o_data  : out std_logic_vector(31 downto 0);
        o_keep0 : out std_logic_vector(3 downto 0);
        o_keep1 : out std_logic_vector(3 downto 0);
        o_keep2 : out std_logic_vector(3 downto 0);
        o_keep3 : out std_logic_vector(3 downto 0);
        o_keeps : out std_logic_vector(3 downto 0);
        o_bytestream_demux_id_error : out std_logic
    );
end byte_stream_demux;

architecture Behavioral of byte_stream_demux is

constant ETM0_ID : std_logic_vector(6 downto 0) := "0010000";
constant ETM1_ID : std_logic_vector(6 downto 0) := "0010001";
constant ETM2_ID : std_logic_vector(6 downto 0) := "0010010";
constant ETM3_ID : std_logic_vector(6 downto 0) := "0010011";
constant STM_ID  : std_logic_vector(6 downto 0) := "0100000"; -- 0x20
constant EMPTY_KEEP : std_logic_vector(3 downto 0) := "0000";

type OutDataFormat is record
    bytes : std_logic_vector(31 downto 0);
    valids : std_logic_vector(3 downto 0);
end record;


-- Creates an ID mask from a 7 bit mask, the input IDs and keep.
-- Basically does the same thing as the main entity but checks if the ID correspond.
pure function createIdMask(masking_id: std_logic_vector(6 downto 0); in_ids : std_logic_vector(27 downto 0); in_keep : std_logic_vector(3 downto 0 )) return std_logic_vector is
    variable out_mask : std_logic_vector(3 downto 0);
begin
    -- Check byte 0
    if in_keep(0) = '1' and in_ids(6 downto 0) = masking_id then
        out_mask(0) :=  '1';
    else
        out_mask(0) := '0';
    end if;
    -- Check byte 1
    if in_keep(1) = '1' and in_ids(13 downto 7) = masking_id then
        out_mask(1) :=  '1';
    else
        out_mask(1) := '0';
    end if;
    -- Check byte 2
    if in_keep(2) = '1' and in_ids(20 downto 14) = masking_id then
        out_mask(2) :=  '1';
    else
        out_mask(2) := '0';
    end if;
    -- Check byte 3
    if in_keep(3) = '1' and in_ids(27 downto 21) = masking_id then
        out_mask(3) :=  '1';
    else
        out_mask(3) := '0';
    end if;
    return out_mask;
end;

-- From the ID masks, route the input data to their correct outputs
begin
demux : process(aclk) begin
    if rising_edge(aclk) then
        if aresetn = '0' or i_soft_reset = '1' then
            o_keep0 <= EMPTY_KEEP;
            o_keep1 <= EMPTY_KEEP;
            o_keep2 <= EMPTY_KEEP;
            o_keep3 <= EMPTY_KEEP;
            o_bytestream_demux_id_error <= '0';
        else
            if i_valid = '1' then
                o_data <= i_data;
                o_keep0 <= createIdMask(ETM0_ID,i_ids,i_keep);
                o_keep1 <= createIdMask(ETM1_ID,i_ids,i_keep);
                o_keep2 <= createIdMask(ETM2_ID,i_ids,i_keep);
                o_keep3 <= createIdMask(ETM3_ID,i_ids,i_keep);
                o_keeps <= createIdMask(STM_ID,i_ids,i_keep);
                -- Check if no mask matches it, raising an error
                if i_keep /= "0000" and
                    createIdMask(ETM0_ID,i_ids,i_keep) = EMPTY_KEEP and
                    createIdMask(ETM1_ID,i_ids,i_keep) = EMPTY_KEEP and
                    createIdMask(ETM2_ID,i_ids,i_keep) = EMPTY_KEEP and
                    createIdMask(ETM3_ID,i_ids,i_keep) = EMPTY_KEEP and
                    createIdMask(STM_ID,i_ids,i_keep) = EMPTY_KEEP then
                        o_bytestream_demux_id_error <= '1';
                else
                    o_bytestream_demux_id_error <= '0';
                end if;
            else
                o_keep0 <= EMPTY_KEEP;
                o_keep1 <= EMPTY_KEEP;
                o_keep2 <= EMPTY_KEEP;
                o_keep3 <= EMPTY_KEEP;
                o_keeps <= EMPTY_KEEP;
                o_bytestream_demux_id_error <= '0';
            end if;
        end if;
    end if;
end process;


end Behavioral;
