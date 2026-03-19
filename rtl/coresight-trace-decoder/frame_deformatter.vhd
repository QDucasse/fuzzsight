library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Frame Deformatter
--
-- > Protocol from Arm Coresight Architecture Specification v3.0 Chapter D4
-- TPIU Interface is a frame based protocol. A frame is 127 bits, containing 7 bytes of data,
-- 8 bytes of mixed use data (containing either source ID or more Data) and 1 auxiliary byte.
-- The frame format essentially provides a single stream of data from multiple sources (multiplexed).
--
-- This byte_stream generator parses data from the TPIU running in continuous mode. Trace stream
-- protocol is a byte-base packet protocol. Each packet contains one or more bytes. A Frame has
-- no context of packets, only data bytes. The frames exist solely to demux packet streams from
-- different ETMs (max 6, 4 from A53s and 2 from R5s) . This means the packet generator must
-- generate a stream of bytes for each trace stream.
--
-- We see a new frame AT MOST every 4 cycles. The maximum number of bytes containing data for one
-- ETM per frame is 15 bytes. (When all ID or Data bytes contain data). Minimum is 0 bytes.
--
-- ==> Packet generator must demux into 6 output wires with 4 bytes (32 bits) per cycle. The demuxing
-- is handled by the byte_stream_demux module. The frame_deformatter only sends out 4 bytes of
-- data, 4 IDs (7 bits) and 4 data_valid_bits. We use an additional 4 bits to denote which bytes contain
-- data in the output of 4 bytes. We process a frame in 4 cycles. At each cycle we output (at most)
-- 4 bytes to the relevant stream.
--
-- The processes are the following:
-- - 1. Synchronization (meta)
--    Sets up a 4-cycle counter to emit the packets one by one from the input frame
-- - 2. Data processing (data_process)
--    Converts the frame into data outputs
-- - 3. ID Processing (id_process)
--    Tracks and propagate the IDs
--
-- Important IOs:
-- i_frame: input 128 bit frame
-- o_data: output packet
-- o_ids: output ids
-- o_keep: output source identification


-- Frame format
-- -----------------
--             31            24 23         17 16 15            8 7            1 0
--             ┌───────────────┬──────────────┬─┬───────────────┬──────────────┬─┐
-- Bytes 3-0   │     Data      │ ID/Data (B)  │F│     Data      │ ID/Data (A)  │F│
--             ├───────────────┼──────────────┼─┼───────────────┼──────────────┼─┤
-- Bytes 7-4   │     Data      │ ID/Data (D)  │F│     Data      │ ID/Data (C)  │F│
--             ├───────────────┼──────────────┼─┼───────────────┼──────────────┼─┤
-- Bytes 11-8  │     Data      │ ID/Data (G)  │F│     Data      │ ID/Data (E)  │F│
--             ├─┬─┬─┬─┬─┬─┬─┬─┼──────────────┼─┼───────────────┼──────────────┼─┤
-- Bytes 15-12 │J│H│G│E│D│C│B│A│ ID/Data (J)  │F│     Data      │ ID/Data (H)  │F│
--             └─┴─┴─┴─┴─┴─┴─┴─┴──────────────┴─┴───────────────┴──────────────┴─┘
--
-- The F bit for each ID/Data indicates if it is a data (0) or new id (1).
-- It should then be replaced by the corresponding auxiliary bit in the auxiliary byte.


entity frame_deformatter is
  port (
    aclk : in std_logic;
    aresetn : in std_logic;
    i_frame: in std_logic_vector(127 downto 0);
    i_valid_frame : in std_logic;

    o_data : out std_logic_vector(31 downto 0);
    o_ids  : out std_logic_vector(27 downto 0);
    o_keep: out std_logic_vector(3 downto 0);
    o_valid : out std_logic;
    o_bytestream_gen_error : out std_logic
   );
end frame_deformatter;

architecture Behavioral of frame_deformatter is

-- 7 bit ID or DATA, the missing bit comes from the auxiliary byte at the end
-- of the frame
type IdOrData is record
   f : std_logic;
   value : std_logic_vector(6 downto 0);
end record;

-- 7 bit Data + auxiliary bit
type Data is record
    value : std_logic_vector(7 downto 0);
end record;

-- Auxiliary byte
-- Note: byte F is not used
type Auxiliary is record
    a : std_logic;
    b : std_logic;
    c : std_logic;
    d : std_logic;
    e : std_logic;
    g : std_logic;
    h : std_logic;
    j : std_logic;
end record;

-- Complete frame format, even byte are IDs or Data
-- odd bytes are always data, last byte is auxiliary
type FrameFormat is record
   id_data0 : IdOrData;
   data0 : Data;
   id_data1: IdOrData;
   data1 : Data;
   id_data2 : IdOrData;
   data2 : Data;
   id_data3 : IdOrData;
   data3 : Data;
   id_data4 : IdOrData;
   data4 : Data;
   id_data5 : IdOrData;
   data5 : Data;
   id_data6 : IdOrData;
   data6 : Data;
   id_data7 : IdOrData;
   aux : Auxiliary;
end record;


pure function createData(value : std_logic_vector(7 downto 0)) return Data is
    variable result: Data;
begin
    result.value := value;
    return result;
end;

pure function createIdOrData(value : std_logic_vector(7 downto 0)) return IdOrData is
    variable result: IdOrData;
begin
    result.f     := value(0);
    result.value := value(7 downto 1);
    return result;
end;

pure function createAuxiliary(value : std_logic_vector(7 downto 0)) return Auxiliary is
    variable result: Auxiliary;
begin
    result.a := value(0);
    result.b := value(1);
    result.c := value(2);
    result.d := value(3);
    result.e := value(4);
    result.g := value(5);
    result.h := value(6);
    result.j := value(7);
    return result;
end;

pure function getAuxBit(aux : Auxiliary; inIdOrDataIndex : integer range 0 to 7 ) return std_logic is
begin
    case inIdOrDataIndex is
        when 0 =>
            return aux.a;
        when 1 =>
            return aux.b;
        when 2 =>
            return aux.c;
        when 3 =>
            return aux.d;
        when 4 =>
            return aux.e;
        when 5 =>
            return aux.g;
        when 6 =>
            return aux.h;
        when others =>
            return aux.j;
     end case;
end;

pure function isNewID(idorData : IdOrData) return boolean is
begin
    return IdOrData.f = '1';
end;

pure function extractDataFromIdOrData(idOrData : IdOrData; aux : Auxiliary; index : integer) return std_logic_vector is
    variable result : std_logic_vector(7 downto 0);
begin
    result(7 downto 1) := idOrData.value;
    result(0) := getAuxBit(aux, index);
    return result;
end;


-- Following the frame format presented earlier, fills the fields
-- of the structure
pure function createFrame(in_frame : std_logic_vector(127 downto 0)) return FrameFormat is
    variable result: FrameFormat;
begin
    result.id_data0 :=  createIdOrData(in_frame(7 downto 0));
    result.data0    :=  createData(in_frame(15 downto 8));
    result.id_data1 :=  createIdOrData(in_frame(23 downto 16));
    result.data1    :=  createData(in_frame(31 downto 24));
    result.id_data2 :=  createIdOrData(in_frame(39 downto 32));
    result.data2    :=  createData(in_frame(47 downto 40));
    result.id_data3 :=  createIdOrData(in_frame(55 downto 48));
    result.data3    :=  createData(in_frame(63 downto 56));
    result.id_data4 :=  createIdOrData(in_frame(71 downto 64));
    result.data4    :=  createData(in_frame(79 downto 72));
    result.id_data5 :=  createIdOrData(in_frame(87 downto 80));
    result.data5    :=  createData(in_frame(95 downto 88));
    result.id_data6 :=  createIdOrData(in_frame(103 downto 96));
    result.data6    :=  createData(in_frame(111 downto 104));
    result.id_data7 :=  createIdOrData(in_frame(119 downto 112));
    result.aux      :=  createAuxiliary(in_frame(127 downto 120));
    return result;
end;


-- From the two ID/Data bytes of a given word, extract them if needed (otherwise last id)
-- Constructs the combination of the four IDs in the output IDs
pure function extractOutIds(aux : Auxiliary; first_id_or_data : IdOrData; second_id_or_data : IdOrData; last_id : std_logic_vector(6 downto 0)) return std_logic_vector is
    variable out_ids_res : std_logic_vector(27 downto 0);
begin
    out_ids_res := X"0000000";
    -- Checking first byte
    if not isNewID(first_id_or_data) then
        -- Propagate last id to the first ID/Data byte and next (Data only) byte
        out_ids_res(6 downto 0) := last_id;
        out_ids_res(13 downto 7) := last_id;
        -- Same check for second one
        if not isNewID(second_id_or_data) then
            out_ids_res(20 downto 14) :=  last_id;
            out_ids_res(27 downto 21) :=  last_id;
        else
            -- second id or data contains a newId
            -- Auxiliary bit high means new id takes effect AFTER byte 1
            if(aux.b = '1') then
                out_ids_res(27 downto 21) :=  last_id;
            else
                out_ids_res(27 downto 21) :=  second_id_or_data.value;
            end if;
        end if;
    else
        -- first id or data contains newId
        if(aux.a = '1') then
            out_ids_res(13 downto 7) := last_id;
        else
            out_ids_res(13 downto 7) := first_id_or_data.value;
        end if;

        if not isNewID(second_id_or_data) then
            out_ids_res(20 downto 14) := first_id_or_data.value;
            out_ids_res(27 downto 21) := first_id_or_data.value;
        else
            -- second id or data contains a newId
            -- Auxiliary bit high means new id takes effect AFTER byte 3
            if(aux.b = '1') then
                out_ids_res(27 downto 21) := first_id_or_data.value;
            else
                out_ids_res(27 downto 21) := second_id_or_data.value;
            end if;
        end if;
    end if;
    return out_ids_res;
end;


-- Update the last id with the latest seen for a word
pure function updateLastId(first_id_or_data : IdOrData; second_id_or_data : IdOrData; last_id : std_logic_vector(6 downto 0)) return std_logic_vector is
begin
    if isNewId(second_id_or_data) then
        return second_id_or_data.value;
    elsif isNewId(first_id_or_data) then
        return first_id_or_data.value;
    else
        return last_id;
    end if;
end;

signal frame : FrameFormat;
signal processing_stage : integer range 0 to 4;
signal last_id : std_logic_vector(6 downto 0);

-- Counter process, counts 4 cycles, verifying if a valid frame comes before,
-- raising an error
begin
meta : process(aclk) begin
if rising_edge(aclk) then
    if aresetn = '0' then
        processing_stage <= 0;
        o_valid <= '0';
        o_bytestream_gen_error <= '0';
    else
        -- New frame, create it!
        if i_valid_frame = '1'then
            processing_stage <= 4;
            frame <= createFrame(i_frame);
        else
            if processing_stage > 0 then
                processing_stage <= processing_stage - 1;
            end if;
        end if;
        -- New valid frame before complete processing
        if i_valid_frame = '1' and processing_stage > 1 then
            o_bytestream_gen_error <= '1';
        else
            o_bytestream_gen_error <= '0';
        end if;
        -- Valid bytes when the frame is being processed
        if processing_stage /= 0 then
            o_valid <= '1';
        else
            o_valid <= '0';
        end if;
    end if;
 end if;
end process;

-- Extract data from the bytes, one word at a time.
-- Checks for the F bit on mixed bytes and extracts data if needed.
data_process : process(aclk) begin
    if aresetn = '0' then
        o_keep <= "0000";
        o_data <= X"00000000";
    end if;
    if rising_edge(aclk) then
        case processing_stage is
            -- Process byte 0 to 3
            when 4 =>
                if isNewID(frame.id_data0) then
                    o_keep(0) <= '0';
                else
                    o_data(7 downto 0) <= extractDataFromIdOrData(frame.id_data0, frame.aux, 0);
                    o_keep(0) <= '1';
                end if;
                if isNewID(frame.id_data1) then
                    o_keep(2) <= '0';
                else
                    o_data(23 downto 16) <= extractDataFromIdOrData(frame.id_data1, frame.aux, 1);
                    o_keep(2) <= '1';
                end if;
                -- these bytes always contain Data
                o_keep(1) <= '1';
                o_data(15 downto 8) <= frame.data0.value;
                o_keep(3) <= '1';
                o_data(31 downto 24) <= frame.data1.value;
            -- Process byte 4 to 7
            when 3 =>
                if isNewID(frame.id_data2) then
                    o_keep(0) <= '0';
                else
                    o_data(7 downto 0) <= extractDataFromIdOrData(frame.id_data2, frame.aux, 2);
                    o_keep(0) <= '1';
                end if;
                if isNewID(frame.id_data3) then
                    o_keep(2) <= '0';
                else
                    o_data(23 downto 16) <= extractDataFromIdOrData(frame.id_data3, frame.aux, 3);
                    o_keep(2) <= '1';
                end if;
                -- these bytes always contain Data
                o_keep(1) <= '1';
                o_data(15 downto 8) <= frame.data2.value;
                o_keep(3) <= '1';
                o_data(31 downto 24) <= frame.data3.value;
            -- Process byte 8 to 11
            when 2 =>
                if isNewID(frame.id_data4) then
                    o_keep(0) <= '0';
                else
                    o_data(7 downto 0) <= extractDataFromIdOrData(frame.id_data4, frame.aux, 4);
                    o_keep(0) <= '1';
                end if;
                if isNewID(frame.id_data5) then
                    o_keep(2) <= '0';
                else
                    o_data(23 downto 16) <= extractDataFromIdOrData(frame.id_data5, frame.aux, 5);
                    o_keep(2) <= '1';
                end if;
                -- these bytes always contain Data
                o_keep(1) <= '1';
                o_data(15 downto 8) <= frame.data4.value;
                o_keep(3) <= '1';
                o_data(31 downto 24) <= frame.data5.value;
            -- Process byte 12-15
            when 1 =>
                if isNewID(frame.id_data6) then
                    o_keep(0) <= '0';
                else
                    o_data(7 downto 0) <= extractDataFromIdOrData(frame.id_data6, frame.aux, 6);
                    o_keep(0) <= '1';
                end if;
                if isNewID(frame.id_data7) then
                    o_keep(2) <= '0';
                else
                    o_data(23 downto 16) <= extractDataFromIdOrData(frame.id_data7, frame.aux, 7);
                    o_keep(2) <= '1';
                end if;
                -- these bytes always contain Data
                o_keep(1) <= '1';
                o_data(15 downto 8) <= frame.data6.value;
                -- last byte is auxiliary byte, we ignore this here.
                o_keep(3) <= '0';
             when others =>
                -- do nothing
        end case;
    end if;
end process;


-- Process the frame word by word based on the current processing stage
id_process : process(aclk) begin
    if aresetn = '0' then
        last_id <= "0000000";
        o_ids <= X"0000000";
    end if;
    if rising_edge(aclk) then
        case processing_stage is
            when 4 =>
                o_ids <= extractOutIds(frame.aux, frame.id_data0, frame.id_data1, last_id);
                last_id <= updateLastId(frame.id_data0, frame.id_data1, last_id);
             when 3 =>
                o_ids <= extractOutIds(frame.aux, frame.id_data2, frame.id_data3, last_id);
                last_id <= updateLastId(frame.id_data2, frame.id_data3, last_id);
             when 2 =>
                o_ids <= extractOutIds(frame.aux, frame.id_data4, frame.id_data5, last_id);
                last_id <= updateLastId(frame.id_data4, frame.id_data5, last_id);
             when 1 =>
                o_ids <= extractOutIds(frame.aux, frame.id_data6, frame.id_data7, last_id);
                last_id <= updateLastId(frame.id_data6, frame.id_data7, last_id);
             when others =>
                -- do nothing
       end case;
   end if;
end process;

end Behavioral;