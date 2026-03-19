library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.decoder_constants.all;

-- ============================================================
-- STM Decoder (32-bit input, nibble-aligned, vectorized)
-- ============================================================

entity stm_decoder is
    Generic(
        STM_MAX_BOUNDED_PAYLOAD_SIZE : integer := STM_MAX_BOUNDED_PAYLOAD_SIZE
    );
    port (
        aclk    : in  std_logic;
        aresetn : in  std_logic;

        i_data  : in  std_logic_vector(31 downto 0);
        i_valid : in  std_logic;

        o_valid0   : out std_logic;
        o_payload0 : out std_logic_vector(63 downto 0);
        o_valid1   : out std_logic;
        o_payload1 : out std_logic_vector(63 downto 0);
        o_valid2   : out std_logic;
        o_payload2 : out std_logic_vector(63 downto 0);
        o_valid3   : out std_logic;
        o_payload3 : out std_logic_vector(63 downto 0);
        o_valid4   : out std_logic;
        o_payload4 : out std_logic_vector(63 downto 0);
        o_valid5   : out std_logic;
        o_payload5 : out std_logic_vector(63 downto 0);
        o_valid6   : out std_logic;
        o_payload6 : out std_logic_vector(63 downto 0);
        o_valid7   : out std_logic;
        o_payload7 : out std_logic_vector(63 downto 0)
);
end entity;

architecture Behavioral of stm_decoder is

    -- ==========================
    --          Types
    -- ==========================

    type HeaderType is (
        -- Wronf decoding
        UNDEFINED,
        -- Misc
        NUL, VERSION, FREQ,
        -- Synchronization
        ASYNC,
        -- Master channel
        M8,
        -- Channel selection
        C8, C16,
        -- Data
        D4, D8, D16, D32, D64,
        -- Data with marker
        D4M, D8M, D16M, D32M, D64M,
        -- Data with timestamp
        D4TS, D8TS, D16TS, D32TS, D64TS,
        -- Data with marker & timestamp
        D4MTS, D8MTS, D16MTS, D32MTS, D64MTS,
        -- Errors
        MERR, GERR,
        -- Flags
        FLAG, FLAG_TS,
        -- Triggers
        TRIG, TRIG_TS,
        -- Currently decoding
        CONTINUE
    );

    type PreprocessingState is record
        header : HeaderType;
        next_header_index : integer range 0 to STM_MAX_HEADER_SIZE;
        nibble : std_logic_vector(3 downto 0);
    end record;

    type OutputState is record
        data_valid   : std_logic;
        data_payload : std_logic_vector(63 downto 0);
    end record;

    type TraceState is record
        output_state : OutputState;
    end record;

    type StreamMode is (
        next_nibble_must_be_header,  -- initial state, just processed a packet
        payload_fixed_size           -- processing a payload
    );

    type StreamState is record
        -- Stream mode coming in
        stream_mode : StreamMode;

        -- Current payload and max index
        payload_index     : integer range 0 to STM_MAX_BOUNDED_PAYLOAD_SIZE;
        payload_max_index : integer range 0 to STM_MAX_BOUNDED_PAYLOAD_SIZE;

        -- Header and current nibble
        header_t : HeaderType;
        nibble   : std_logic_vector(3 downto 0);

        -- Pipeline for the next nibbles
        next_stream_mode0       : StreamMode;
        next_header_t0          : HeaderType;
        next_payload_max_index0 : integer range 0 to STM_MAX_BOUNDED_PAYLOAD_SIZE;

        next_stream_mode1       : StreamMode;
        next_header_t1          : HeaderType;
        next_payload_max_index1 : integer range 0 to STM_MAX_BOUNDED_PAYLOAD_SIZE;

        next_stream_mode2       : StreamMode;
        next_header_t2          : HeaderType;
        next_payload_max_index2 : integer range 0 to STM_MAX_BOUNDED_PAYLOAD_SIZE;

        next_stream_mode3       : StreamMode;
        next_header_t3          : HeaderType;
        next_payload_max_index3 : integer range 0 to STM_MAX_BOUNDED_PAYLOAD_SIZE;

        next_stream_mode4       : StreamMode;
        next_header_t4          : HeaderType;
        next_payload_max_index4 : integer range 0 to STM_MAX_BOUNDED_PAYLOAD_SIZE;

        next_stream_mode5       : StreamMode;
        next_header_t5          : HeaderType;
        next_payload_max_index5 : integer range 0 to STM_MAX_BOUNDED_PAYLOAD_SIZE;

        next_stream_mode6       : StreamMode;
        next_header_t6          : HeaderType;
        next_payload_max_index6 : integer range 0 to STM_MAX_BOUNDED_PAYLOAD_SIZE;

        next_stream_mode7       : StreamMode;
        next_header_t7          : HeaderType;
        next_payload_max_index7 : integer range 0 to STM_MAX_BOUNDED_PAYLOAD_SIZE;
    end record;


    -- 1. Header decoding
    -- \____________________

    -- Decoder based on the nibble index and value
    pure function headerLookupTable(
        nibble : std_logic_vector(3 downto 0);
        prev_nibble : std_logic_vector(3 downto 0);
        n_index : integer range 0 to STM_MAX_HEADER_SIZE
    ) return HeaderType is
        variable nibble_u : unsigned(3 downto 0);
    begin
        nibble_u := unsigned(nibble);
        case n_index is
            -- Nibble 0
            when 0 =>
                if nibble = "0000" then
                    return NUL;
                elsif nibble = "0001" then
                    return M8;
                elsif nibble = "0010" then
                    return MERR;
                elsif nibble = "0011" then
                    return C8;
                elsif nibble = "0100" then
                    return D8;
                elsif nibble = "0101" then
                    return D16;
                elsif nibble = "0110" then
                    return D32;
                elsif nibble = "0111" then
                    return D64;
                elsif nibble = "1000" then
                    return D8MTS;
                elsif nibble = "1001" then
                    return D16MTS;
                elsif nibble = "1010" then
                    return D32MTS;
                elsif nibble = "1011" then
                    return D64MTS;
                elsif nibble = "1100" then
                    return D4;
                elsif nibble = "1101" then
                    return D4MTS;
                elsif nibble = "1110" then
                    return FLAG_TS;
                elsif nibble = "1111" then
                    return CONTINUE;
                else
                    return UNDEFINED;
                end if;
            -- Nibble 1
            when 1 =>
                if nibble = "0000" then
                    return CONTINUE;
                -- 0001 - M16 Unsupported by CoreSight
                elsif nibble = "0010" then
                    return GERR;
                elsif nibble = "0011" then
                    return C16;
                elsif nibble = "0100" then
                    return D8TS;
                elsif nibble = "0101" then
                    return D16TS;
                elsif nibble = "0110" then
                    return D32TS;
                elsif nibble = "0111" then
                    return D64TS;
                elsif nibble = "1000" then
                    return D8M;
                elsif nibble = "1001" then
                    return D16M;
                elsif nibble = "1010" then
                    return D32M;
                elsif nibble = "1011" then
                    return D64M;
                elsif nibble = "1100" then
                    return D4TS;
                elsif nibble = "1101" then
                    return D4M;
                elsif nibble = "1110" then
                    return FLAG;
                elsif nibble = "1111" then
                    return CONTINUE;  -- 22 Fs then one 0
                else
                    return UNDEFINED;
                end if;
            -- Nibble 2
            when 2 =>
                -- Might come from 0 or F, each having their own decodes
                case prev_nibble is
                    when "0000" =>
                        if nibble = "0000" then
                            return VERSION;
                        -- 0001 - NUL_TS Unsupported by CoreSight
                        -- 0010 - USER    Unsupported by CoreSight
                        -- 0011 - USER_TS Unsupported by CoreSight
                        -- 0100 - TIME    Unsupported by CoreSight
                        -- 0101 - TIME_TS Unsupported by CoreSight
                        elsif nibble = "0110" then
                            return TRIG;
                        elsif nibble = "0111" then
                            return TRIG_TS;
                        elsif nibble = "1000" then
                            return FREQ;
                        -- 1001 - FREQ_TS  Unsupported by CoreSight
                        -- 1010 - XSYNC    Unsupported by CoreSight
                        -- 1011 - XSYNC_TS Unsupported by CoreSight
                        -- Rest is reserved
                        else
                            return UNDEFINED;
                        end if;
                    when "1111" =>
                        if nibble = "1111" then
                            return CONTINUE;
                        else
                            return UNDEFINED;
                        end if;
                    when others =>
                        return UNDEFINED;
                end case;
            -- The rest is for ASYNC, 21Fs and a 0
            when 3 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 4 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 5 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 6 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 7 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 8 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 9 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 10 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 11 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 12 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 13 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 14 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 15 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 16 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 17 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 18 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 19 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 20 =>
                if nibble = "1111" then return CONTINUE;
                else return UNDEFINED;
                end if;
            when 21 =>
                if nibble = "0000" then return ASYNC;
                else return UNDEFINED;
                end if;
            when others =>
                return UNDEFINED;
        end case;
    end;

    -- Update header index if needed
    pure function preprocess(
        in_preprocessing_state : PreprocessingState;
        nibble : std_logic_vector(3 downto 0)
    ) return PreprocessingState is
        variable out_preprocessing_state : PreprocessingState;
    begin
        out_preprocessing_state.header := headerLookupTable(nibble, in_preprocessing_state.nibble, in_preprocessing_state.next_header_index);
        out_preprocessing_state.nibble := nibble;
        -- If the header is CONTINUE, increment the header index, otherwise clear it
        if out_preprocessing_state.header = CONTINUE then
            out_preprocessing_state.next_header_index := in_preprocessing_state.next_header_index + 1;
        else
            out_preprocessing_state.next_header_index := 0;
        end if;
        return out_preprocessing_state;
    end;


    -- 2. Stream State Processing
    -- \______________________________________

    -- Payload fixed size mode, if the index is max, shifts to the next mode,
    -- Otherwise, increase index, stay in this mode and propagate lookahead
    pure function handleStreamStatePayloadFixedSize(
        in_stream_state : StreamState;
        preprocessing_state : PreprocessingState;
        nibble : std_logic_vector(3 downto 0)
    ) return StreamState is
        variable out_stream_state : StreamState;
    begin
        if in_stream_state.payload_index = in_stream_state.payload_max_index then
            -- Payload index is the max, we switch to the next stream mode,
            -- update the header, and other values.
            out_stream_state.stream_mode       := in_stream_state.next_stream_mode0;
            out_stream_state.header_t          := in_stream_state.next_header_t0;
            out_stream_state.payload_index     := 0;
            out_stream_state.payload_max_index := in_stream_state.next_payload_max_index0;

            -- Shift lookahead
            out_stream_state.next_stream_mode0       := in_stream_state.next_stream_mode0;
            out_stream_state.next_header_t0          := in_stream_state.next_header_t0;
            out_stream_state.next_payload_max_index0 := in_stream_state.next_payload_max_index0;
        else
            -- Payload index is not the max, we stay in the payload_fixed_size mode
            -- and increment the payload index.
            out_stream_state.stream_mode       := payload_fixed_size;
            out_stream_state.payload_index     := in_stream_state.payload_index + 1;
            out_stream_state.payload_max_index := in_stream_state.payload_max_index;
            out_stream_state.header_t          := in_stream_state.header_t;

            -- Propagate stream lookahead knowledge
            out_stream_state.next_stream_mode0       := in_stream_state.next_stream_mode0;
            out_stream_state.next_header_t0          := in_stream_state.next_header_t0;
            out_stream_state.next_payload_max_index0 := in_stream_state.next_payload_max_index0;

            out_stream_state.next_stream_mode1       := in_stream_state.next_stream_mode1;
            out_stream_state.next_header_t1          := in_stream_state.next_header_t1;
            out_stream_state.next_payload_max_index1 := in_stream_state.next_payload_max_index1;

            out_stream_state.next_stream_mode2       := in_stream_state.next_stream_mode2;
            out_stream_state.next_header_t2          := in_stream_state.next_header_t2;
            out_stream_state.next_payload_max_index2 := in_stream_state.next_payload_max_index2;

            out_stream_state.next_stream_mode3       := in_stream_state.next_stream_mode3;
            out_stream_state.next_header_t3          := in_stream_state.next_header_t3;
            out_stream_state.next_payload_max_index3 := in_stream_state.next_payload_max_index3;

            out_stream_state.next_stream_mode4       := in_stream_state.next_stream_mode4;
            out_stream_state.next_header_t4          := in_stream_state.next_header_t4;
            out_stream_state.next_payload_max_index4 := in_stream_state.next_payload_max_index4;

            out_stream_state.next_stream_mode5       := in_stream_state.next_stream_mode5;
            out_stream_state.next_header_t5          := in_stream_state.next_header_t5;
            out_stream_state.next_payload_max_index5 := in_stream_state.next_payload_max_index5;

            out_stream_state.next_stream_mode6       := in_stream_state.next_stream_mode6;
            out_stream_state.next_header_t6          := in_stream_state.next_header_t6;
            out_stream_state.next_payload_max_index6 := in_stream_state.next_payload_max_index6;

            out_stream_state.next_stream_mode7       := in_stream_state.next_stream_mode7;
            out_stream_state.next_header_t7          := in_stream_state.next_header_t7;
            out_stream_state.next_payload_max_index7 := in_stream_state.next_payload_max_index7;
        end if;
        return out_stream_state;
    end;

    --
    pure function handleStreamStateHeader(
        in_stream_state : StreamState;
        preprocessing_state : PreprocessingState;
        nibble : std_logic_vector(3 downto 0)
    ) return StreamState is
        variable out_stream_state : StreamState;
    begin
        case preprocessing_state.header is

            -- Header only, no payload
            when NUL | FLAG =>
                out_stream_state.stream_mode  := next_nibble_must_be_header;
                out_stream_state.header_t     := preprocessing_state.header;

            -- 4-bit payload
            when D4 | D4M | VERSION =>
                out_stream_state.stream_mode       := payload_fixed_size;
                out_stream_state.header_t          := preprocessing_state.header;
                out_stream_state.payload_max_index := 0;

                out_stream_state.next_stream_mode0 := next_nibble_must_be_header;
                out_stream_state.next_header_t0    := UNDEFINED;

            -- 8-bit payload
            when M8 | C8 | D8 | D8M | TRIG | MERR | GERR =>
                out_stream_state.stream_mode       := payload_fixed_size;
                out_stream_state.header_t          := preprocessing_state.header;
                out_stream_state.payload_max_index := 1;

                out_stream_state.next_stream_mode0 := next_nibble_must_be_header;
                out_stream_state.next_header_t0    := UNDEFINED;

            -- 16-bit payload
            when C16 | D16 | D16M =>
                out_stream_state.stream_mode       := payload_fixed_size;
                out_stream_state.header_t          := preprocessing_state.header;
                out_stream_state.payload_max_index := 3;

                out_stream_state.next_stream_mode0 := next_nibble_must_be_header;
                out_stream_state.next_header_t0    := UNDEFINED;

            -- 32-bit payload
            when D32 | D32M =>
                out_stream_state.stream_mode       := payload_fixed_size;
                out_stream_state.header_t          := preprocessing_state.header;
                out_stream_state.payload_max_index := 7;

                out_stream_state.next_stream_mode0 := next_nibble_must_be_header;
                out_stream_state.next_header_t0    := UNDEFINED;

            -- 64-bit payload
            when D64 | D64M =>
                out_stream_state.stream_mode       := payload_fixed_size;
                out_stream_state.header_t          := preprocessing_state.header;
                out_stream_state.payload_max_index := 15;

                out_stream_state.next_stream_mode0 := next_nibble_must_be_header;
                out_stream_state.next_header_t0    := UNDEFINED;

            -- Continue header
            when CONTINUE =>
                out_stream_state.stream_mode      := next_nibble_must_be_header;
                out_stream_state.header_t         := preprocessing_state.header;

            -- TODO: Missing timestamp packets:
            --    D4TS, D8TS, D16TS, D32TS, D64TS, D4MTS, D8MTS, D16MTS, D32MTS, D64MTS,
            --    TRIG_TS, FLAG_TS
            -- TODO: Version packet set up with a fixed payload, but it may be more complex

            when others =>
                out_stream_state.stream_mode  := next_nibble_must_be_header;
                out_stream_state.header_t     := preprocessing_state.header;

        end case;

        -- Reset payload index
        out_stream_state.payload_index := 0;
        return out_stream_state;
    end;

    -- TODO: Handle the version (and timestamps through those)
    -- pure function handleStreamStateVersionMode
    -- pure function handleStreamStateVersionMode

    -- Dispatch function based on the stream mode
    pure function handleStreamState(
        in_stream_state : StreamState;
        preprocessing_state: PreprocessingState;
        nibble : std_logic_vector(3 downto 0)
    ) return StreamState is
        variable out_stream_state : StreamState;
    begin
        case in_stream_state.stream_mode is
            when payload_fixed_size => -- special packet mode
                return handleStreamStatePayloadFixedSize(in_stream_state, preprocessing_state, nibble);
            when next_nibble_must_be_header =>
                return handleStreamStateHeader(in_stream_state, preprocessing_state, nibble);
            when others =>
                -- do nothing
        end case;
        return out_stream_state;
    end;

    -- 3. Action Preprocessing
    -- \___________________________________

    type TraceStateAction is (
        nop,
        -- Base updates
        update_payload_0_3,
        update_payload_4_7,
        update_payload_8_11,
        update_payload_12_15,
        update_payload_16_19,
        update_payload_20_23,
        update_payload_24_27,
        update_payload_28_31,
        update_payload_32_35,
        update_payload_36_39,
        update_payload_40_43,
        update_payload_44_47,
        update_payload_48_51,
        update_payload_52_55,
        update_payload_56_59,
        update_payload_60_63,
        -- Updates marking payload validity
        update_payload_0_3_valid,
        update_payload_4_7_valid,
        update_payload_12_15_valid,
        update_payload_28_31_valid
        -- update_payload_60_63_valid,
        -- Note: not needed since it is the max value
    );

    -- TODO: Upper than the actual update should be zeroed out

    pure function get_update_payload_0_3(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(3 downto 0)  := nibble;
        return res;
    end;

    pure function get_update_payload_4_7(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(3 downto 0)  := payload(3 downto 0);
        res(7 downto 4)  := nibble;
        return res;
    end;


    pure function get_update_payload_8_11(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(7 downto 0)  := payload(7 downto 0);
        res(11 downto 8) := nibble;
        return res;
    end;


    pure function get_update_payload_12_15(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(11 downto 0)  := payload(11 downto 0);
        res(15 downto 12) := nibble;
        return res;
    end;


    pure function get_update_payload_16_19(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(15 downto 0)  := payload(15 downto 0);
        res(19 downto 16) := nibble;
        return res;
    end;


    pure function get_update_payload_20_23(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(19 downto 0)  := payload(19 downto 0);
        res(23 downto 20) := nibble;
        return res;
    end;


    pure function get_update_payload_24_27(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(23 downto 0)  := payload(23 downto 0);
        res(27 downto 24) := nibble;
        return res;
    end;


    pure function get_update_payload_28_31(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(27 downto 0)  := payload(27 downto 0);
        res(31 downto 28) := nibble;
        return res;
    end;


    pure function get_update_payload_32_35(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(31 downto 0)  := payload(31 downto 0);
        res(35 downto 32) := nibble;
        return res;
    end;


    pure function get_update_payload_36_39(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(35 downto 0)  := payload(35 downto 0);
        res(39 downto 36) := nibble;
        return res;
    end;


    pure function get_update_payload_40_43(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(39 downto 0)  := payload(39 downto 0);
        res(43 downto 40) := nibble;
        return res;
    end;


    pure function get_update_payload_44_47(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(43 downto 0)  := payload(43 downto 0);
        res(47 downto 44) := nibble;
        return res;
    end;


    pure function get_update_payload_48_51(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(47 downto 0)  := payload(47 downto 0);
        res(51 downto 48) := nibble;
        return res;
    end;


    pure function get_update_payload_52_55(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(51 downto 0)  := payload(51 downto 0);
        res(55 downto 52) := nibble;
        return res;
    end;


    pure function get_update_payload_56_59(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res := (others => '0');
        res(55 downto 0)  := payload(55 downto 0);
        res(59 downto 56) := nibble;
        return res;

    end;


    pure function get_update_payload_60_63(
        payload : std_logic_vector(63 downto 0);
        nibble : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(59 downto 0)  := payload(59 downto 0);
        res(63 downto 60) := nibble;
        return res;
    end;


    -- Action plan creation, simple routing to the correct routine based
    -- on the payload index
    pure function createActionPlan(
        stream_state : StreamState;
        preprocessing_state : PreprocessingState
    ) return TraceStateAction is
    begin
        case stream_state.header_t is
            -- Non-data
            when M8 | C8 | TRIG | MERR | GERR =>
                case stream_state.payload_index is
                    when 0      => return update_payload_0_3;
                    when others => return update_payload_4_7;
                end case;

            when C16 =>
                case stream_state.payload_index is
                    when 0      => return update_payload_0_3;
                    when 1      => return update_payload_4_7;
                    when 2      => return update_payload_8_11;
                    when others => return update_payload_12_15;
                end case;

            -- Data packets
            when D4 | D4M =>
                return update_payload_0_3_valid;

            when D8 | D8M =>
                case stream_state.payload_index is
                    when 0      => return update_payload_0_3;
                    when others => return update_payload_4_7_valid;
                end case;

            when D16 | D16M =>
                case stream_state.payload_index is
                    when 0      => return update_payload_0_3;
                    when 1      => return update_payload_4_7;
                    when 2      => return update_payload_8_11;
                    when others => return update_payload_12_15_valid;
                end case;

            when D32 | D32M =>
                case stream_state.payload_index is
                    when 0      => return update_payload_0_3;
                    when 1      => return update_payload_4_7;
                    when 2      => return update_payload_8_11;
                    when 3      => return update_payload_12_15;
                    when 4      => return update_payload_16_19;
                    when 5      => return update_payload_20_23;
                    when 6      => return update_payload_24_27;
                    when others => return update_payload_28_31_valid;
                end case;

            when D64 | D64M =>
                case stream_state.payload_index is
                    when 0      => return update_payload_0_3;
                    when 1      => return update_payload_4_7;
                    when 2      => return update_payload_8_11;
                    when 3      => return update_payload_12_15;
                    when 4      => return update_payload_16_19;
                    when 5      => return update_payload_20_23;
                    when 6      => return update_payload_24_27;
                    when 7      => return update_payload_28_31;
                    when 8      => return update_payload_32_35;
                    when 9      => return update_payload_36_39;
                    when 10     => return update_payload_40_43;
                    when 11     => return update_payload_44_47;
                    when 12     => return update_payload_48_51;
                    when 13     => return update_payload_52_55;
                    when 14     => return update_payload_56_59;
                    when others => return update_payload_60_63;
                end case;

            when others =>
                return nop;
        end case;
    end;

    -- 4. Trace State Processing
    -- \___________________________

    pure function handleActionPlan(
        in_trace_state : TraceState;
        action : TraceStateAction;
        nibble : std_logic_vector(3 downto 0)
    ) return TraceState is
        variable out_trace_state : TraceState;
    begin
        -- Start by mapping the input trace state to the output THEN applying changes:
        out_trace_state := in_trace_state;
        -- Reset the valid signal
        out_trace_state.output_state.data_valid := '0';

        case action is
            when update_payload_0_3 =>
                out_trace_state.output_state.data_payload := get_update_payload_0_3(
                    in_trace_state.output_state.data_payload, nibble
                );

            when update_payload_4_7 =>
                out_trace_state.output_state.data_payload := get_update_payload_4_7(
                    in_trace_state.output_state.data_payload, nibble
                );

            when update_payload_8_11 =>
                out_trace_state.output_state.data_payload := get_update_payload_8_11(
                    in_trace_state.output_state.data_payload, nibble
                );

            when update_payload_12_15 =>
                out_trace_state.output_state.data_payload := get_update_payload_12_15(
                    in_trace_state.output_state.data_payload, nibble
                );

            when update_payload_16_19 =>
                out_trace_state.output_state.data_payload := get_update_payload_16_19(
                    in_trace_state.output_state.data_payload, nibble
                );

            when update_payload_20_23 =>
                out_trace_state.output_state.data_payload := get_update_payload_20_23(
                    in_trace_state.output_state.data_payload, nibble
                );

            when update_payload_24_27 =>
                out_trace_state.output_state.data_payload := get_update_payload_24_27(
                    in_trace_state.output_state.data_payload, nibble
                );

            when update_payload_28_31 =>
                out_trace_state.output_state.data_payload := get_update_payload_28_31(
                    in_trace_state.output_state.data_payload, nibble
                );

            when update_payload_32_35 =>
                out_trace_state.output_state.data_payload := get_update_payload_32_35(
                    in_trace_state.output_state.data_payload, nibble
                );

            when update_payload_36_39 =>
                out_trace_state.output_state.data_payload := get_update_payload_36_39(
                    in_trace_state.output_state.data_payload, nibble
                );

            when update_payload_40_43 =>
                out_trace_state.output_state.data_payload := get_update_payload_40_43(
                    in_trace_state.output_state.data_payload, nibble
                );

            when update_payload_44_47 =>
                out_trace_state.output_state.data_payload := get_update_payload_44_47(
                    in_trace_state.output_state.data_payload, nibble
                );

            when update_payload_48_51 =>
                out_trace_state.output_state.data_payload := get_update_payload_48_51(
                    in_trace_state.output_state.data_payload, nibble
                );

            when update_payload_52_55 =>
                out_trace_state.output_state.data_payload := get_update_payload_52_55(
                    in_trace_state.output_state.data_payload, nibble
                );

            when update_payload_56_59 =>
                out_trace_state.output_state.data_payload := get_update_payload_56_59(
                    in_trace_state.output_state.data_payload, nibble
                );

            -- With valid signal, marking the end of the payload
            when update_payload_0_3_valid =>
                out_trace_state.output_state.data_payload := get_update_payload_0_3(
                    in_trace_state.output_state.data_payload, nibble
                );
                out_trace_state.output_state.data_valid   := '1';

            when update_payload_4_7_valid =>
                out_trace_state.output_state.data_payload := get_update_payload_4_7(
                    in_trace_state.output_state.data_payload, nibble
                );
                out_trace_state.output_state.data_valid   := '1';

            when update_payload_12_15_valid =>
                out_trace_state.output_state.data_payload := get_update_payload_12_15(
                    in_trace_state.output_state.data_payload, nibble
                );
                out_trace_state.output_state.data_valid   := '1';

            when update_payload_28_31_valid =>
                out_trace_state.output_state.data_payload := get_update_payload_28_31(
                    in_trace_state.output_state.data_payload, nibble
                );
                out_trace_state.output_state.data_valid   := '1';

            when update_payload_60_63 =>
                out_trace_state.output_state.data_payload := get_update_payload_60_63(
                    in_trace_state.output_state.data_payload, nibble
                );
                out_trace_state.output_state.data_valid   := '1';

            when nop =>
                -- do nothing
            when others =>
                -- do nothing
        end case;
        return out_trace_state;
    end;

    -- ==========================
    --          Signals
    -- ==========================

    signal data_stream_state_stage : std_logic_vector(31 downto 0);
    signal data_valid_stream_state_stage : std_logic;

    signal data_action_plan_stage : std_logic_vector(31 downto 0);
    signal data_valid_action_plan_stage : std_logic;

    signal data_action_stage : std_logic_vector(31 downto 0);
    signal data_valid_action_stage: std_logic;

    signal data_valid_output_stage: std_logic;

    signal preprocessing_state_stage0 : PreprocessingState;
    signal preprocessing_state_stage1 : PreprocessingState;
    signal preprocessing_state_stage2 : PreprocessingState;
    signal preprocessing_state_stage3 : PreprocessingState;
    signal preprocessing_state_stage4 : PreprocessingState;
    signal preprocessing_state_stage5 : PreprocessingState;
    signal preprocessing_state_stage6 : PreprocessingState;
    signal preprocessing_state_stage7 : PreprocessingState;

    signal preprocessing_state_stage7_prop : PreprocessingState;


    signal preprocessing_action_plan_stage0 : PreprocessingState;
    signal preprocessing_action_plan_stage1 : PreprocessingState;
    signal preprocessing_action_plan_stage2 : PreprocessingState;
    signal preprocessing_action_plan_stage3 : PreprocessingState;
    signal preprocessing_action_plan_stage4 : PreprocessingState;
    signal preprocessing_action_plan_stage5 : PreprocessingState;
    signal preprocessing_action_plan_stage6 : PreprocessingState;
    signal preprocessing_action_plan_stage7 : PreprocessingState;

    signal trace_state0 : TraceState;
    signal trace_state1 : TraceState;
    signal trace_state2 : TraceState;
    signal trace_state3 : TraceState;
    signal trace_state4 : TraceState;
    signal trace_state5 : TraceState;
    signal trace_state6 : TraceState;
    signal trace_state7 : TraceState;

    signal stream_state0 : StreamState;
    signal stream_state1 : StreamState;
    signal stream_state2 : StreamState;
    signal stream_state3 : StreamState;
    signal stream_state4 : StreamState;
    signal stream_state5 : StreamState;
    signal stream_state6 : StreamState;
    signal stream_state7 : StreamState;

    signal stream_state7_prop : StreamState;

    signal action0 : TraceStateAction;
    signal action1 : TraceStateAction;
    signal action2 : TraceStateAction;
    signal action3 : TraceStateAction;
    signal action4 : TraceStateAction;
    signal action5 : TraceStateAction;
    signal action6 : TraceStateAction;
    signal action7 : TraceStateAction;

begin


-- ==========================
--         Processes
-- ==========================

-- 1. Header Preprocessing
preprocessing_process : process(aclk)
begin
     if rising_edge(aclk) then
        if aresetn = '0'then
           data_valid_stream_state_stage <= '0';
           preprocessing_state_stage7.header <= UNDEFINED;
           preprocessing_state_stage7.next_header_index <= 0;
           preprocessing_state_stage7.nibble <= "0000";

        else
            -- TODO: Deassert by default?
            data_valid_stream_state_stage <= '0';
            if i_valid = '1' then
                preprocessing_state_stage7_prop <= preprocessing_state_stage7;

                preprocessing_state_stage0 <=
                    preprocess(preprocessing_state_stage7, i_data(3 downto 0));
                preprocessing_state_stage1 <=
                    preprocess(
                    preprocess(preprocessing_state_stage7, i_data(3 downto 0)),
                        i_data(7 downto 4));
                preprocessing_state_stage2 <=
                    preprocess(
                    preprocess(
                    preprocess(preprocessing_state_stage7, i_data(3 downto 0)),
                        i_data(7 downto 4)),
                        i_data(11 downto 8));
                preprocessing_state_stage3 <=
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(preprocessing_state_stage7, i_data(3 downto 0)),
                        i_data(7 downto 4)),
                        i_data(11 downto 8)),
                        i_data(15 downto 12));
                preprocessing_state_stage4 <=
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(preprocessing_state_stage7, i_data(3 downto 0)),
                        i_data(7 downto 4)),
                        i_data(11 downto 8)),
                        i_data(15 downto 12)),
                        i_data(19 downto 16));
                preprocessing_state_stage5 <=
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(preprocessing_state_stage7, i_data(3 downto 0)),
                        i_data(7 downto 4)),
                        i_data(11 downto 8)),
                        i_data(15 downto 12)),
                        i_data(19 downto 16)),
                        i_data(23 downto 20));
                preprocessing_state_stage6 <=
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(preprocessing_state_stage7, i_data(3 downto 0)),
                        i_data(7 downto 4)),
                        i_data(11 downto 8)),
                        i_data(15 downto 12)),
                        i_data(19 downto 16)),
                        i_data(23 downto 20)),
                        i_data(27 downto 24));
                preprocessing_state_stage7 <=
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(
                    preprocess(preprocessing_state_stage7, i_data(3 downto 0)),
                        i_data(7 downto 4)),
                        i_data(11 downto 8)),
                        i_data(15 downto 12)),
                        i_data(19 downto 16)),
                        i_data(23 downto 20)),
                        i_data(27 downto 24)),
                        i_data(31 downto 28));
                data_stream_state_stage <= i_data;
                data_valid_stream_state_stage <= '1';
           end if;
        end if;
     end if;
end process;

-- 2. Stream State Processing
stream_state_process : process(aclk)
begin
     if rising_edge(aclk) then
        if aresetn = '0' then
            -- TODO: Deassert in reset?
            data_valid_action_plan_stage <= '0';
            stream_state7.stream_mode <= next_nibble_must_be_header;
        else
            -- TODO: Deassert by default?
            data_valid_action_plan_stage <= '0';

            if data_valid_stream_state_stage = '1' then
                preprocessing_action_plan_stage0 <= preprocessing_state_stage0;
                preprocessing_action_plan_stage1 <= preprocessing_state_stage1;
                preprocessing_action_plan_stage2 <= preprocessing_state_stage2;
                preprocessing_action_plan_stage3 <= preprocessing_state_stage3;
                preprocessing_action_plan_stage4 <= preprocessing_state_stage4;
                preprocessing_action_plan_stage5 <= preprocessing_state_stage5;
                preprocessing_action_plan_stage6 <= preprocessing_state_stage6;
                preprocessing_action_plan_stage7 <= preprocessing_state_stage7;

                stream_state7_prop <= stream_state7;

                stream_state0 <= handleStreamState(stream_state7, preprocessing_state_stage0, data_stream_state_stage(3 downto 0));

                stream_state1 <=
                    handleStreamState(
                    handleStreamState(stream_state7, preprocessing_state_stage0, data_stream_state_stage(3 downto 0)),
                    preprocessing_state_stage1, data_stream_state_stage(7 downto 4));

                stream_state2 <=
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(stream_state7, preprocessing_state_stage0, data_stream_state_stage(3 downto 0)),
                    preprocessing_state_stage1, data_stream_state_stage(7 downto 4)),
                    preprocessing_state_stage2, data_stream_state_stage(11 downto 8));

                stream_state3 <=
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(stream_state7, preprocessing_state_stage0, data_stream_state_stage(3 downto 0)),
                    preprocessing_state_stage1, data_stream_state_stage(7 downto 4)),
                    preprocessing_state_stage2, data_stream_state_stage(11 downto 8)),
                    preprocessing_state_stage3, data_stream_state_stage(15 downto 12));

                stream_state4 <=
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(stream_state7, preprocessing_state_stage0, data_stream_state_stage(3 downto 0)),
                    preprocessing_state_stage1, data_stream_state_stage(7 downto 4)),
                    preprocessing_state_stage2, data_stream_state_stage(11 downto 8)),
                    preprocessing_state_stage3, data_stream_state_stage(15 downto 12)),
                    preprocessing_state_stage4, data_stream_state_stage(19 downto 16));

                stream_state5 <=
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(stream_state7, preprocessing_state_stage0, data_stream_state_stage(3 downto 0)),
                    preprocessing_state_stage1, data_stream_state_stage(7 downto 4)),
                    preprocessing_state_stage2, data_stream_state_stage(11 downto 8)),
                    preprocessing_state_stage3, data_stream_state_stage(15 downto 12)),
                    preprocessing_state_stage4, data_stream_state_stage(19 downto 16)),
                    preprocessing_state_stage5, data_stream_state_stage(23 downto 20));

                stream_state6 <=
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(stream_state7, preprocessing_state_stage0, data_stream_state_stage(3 downto 0)),
                    preprocessing_state_stage1, data_stream_state_stage(7 downto 4)),
                    preprocessing_state_stage2, data_stream_state_stage(11 downto 8)),
                    preprocessing_state_stage3, data_stream_state_stage(15 downto 12)),
                    preprocessing_state_stage4, data_stream_state_stage(19 downto 16)),
                    preprocessing_state_stage5, data_stream_state_stage(23 downto 20)),
                    preprocessing_state_stage6, data_stream_state_stage(27 downto 24));

                stream_state7 <=
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(stream_state7, preprocessing_state_stage0, data_stream_state_stage(3 downto 0)),
                    preprocessing_state_stage1, data_stream_state_stage(7 downto 4)),
                    preprocessing_state_stage2, data_stream_state_stage(11 downto 8)),
                    preprocessing_state_stage3, data_stream_state_stage(15 downto 12)),
                    preprocessing_state_stage4, data_stream_state_stage(19 downto 16)),
                    preprocessing_state_stage5, data_stream_state_stage(23 downto 20)),
                    preprocessing_state_stage6, data_stream_state_stage(27 downto 24)),
                    preprocessing_state_stage7, data_stream_state_stage(31 downto 28));

                data_action_plan_stage <= data_stream_state_stage;
                data_valid_action_plan_stage <= '1';
            end if;
        end if;
     end if;
end process;

-- 3. Action Preprocessing
action_plan_process : process(aclk)
begin
     if rising_edge(aclk) then
        if aresetn = '0' then
            -- TODO: Deassert in reset?
            data_valid_action_stage <= '0';
        else
            -- TODO: Deassert by default?
            data_valid_action_stage <= '0';
            if data_valid_action_plan_stage = '1' then

                action0 <= createActionPlan(stream_state7_prop, preprocessing_action_plan_stage0);
                action1 <= createActionPlan(stream_state0, preprocessing_action_plan_stage1);
                action2 <= createActionPlan(stream_state1, preprocessing_action_plan_stage2);
                action3 <= createActionPlan(stream_state2, preprocessing_action_plan_stage3);
                action4 <= createActionPlan(stream_state3, preprocessing_action_plan_stage4);
                action5 <= createActionPlan(stream_state4, preprocessing_action_plan_stage5);
                action6 <= createActionPlan(stream_state5, preprocessing_action_plan_stage6);
                action7 <= createActionPlan(stream_state6, preprocessing_action_plan_stage7);

                data_action_stage <= data_action_plan_stage;
                data_valid_action_stage <= '1';
            end if;
        end if;
     end if;
end process;

-- 4. Trace State Processing
trace_state_action_process : process(aclk)
begin
     if rising_edge(aclk) then
        if aresetn = '0' then
            -- TODO: Deassert in reset?
            data_valid_output_stage <= '0';
        else
            -- TODO: Deassert by default?
            data_valid_output_stage <= '0';
            if data_valid_action_stage = '1' then
                trace_state0 <= handleActionPlan(trace_state7, action0, data_action_stage(3 downto 0));

                trace_state1 <= handleActionPlan(
                                handleActionPlan(trace_state7, action0, data_action_stage(3 downto 0)),
                                                action1, data_action_stage(7 downto 4));

                trace_state2 <= handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(trace_state7, action0, data_action_stage(3 downto 0)),
                                                 action1, data_action_stage(7 downto 4)),
                                                 action2, data_action_stage(11 downto 8));

                trace_state3 <= handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(trace_state7, action0, data_action_stage(3 downto 0)),
                                                 action1, data_action_stage(7 downto 4)),
                                                 action2, data_action_stage(11 downto 8)),
                                                 action3, data_action_stage(15 downto 12));

                trace_state4 <= handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(trace_state7, action0, data_action_stage(3 downto 0)),
                                                 action1, data_action_stage(7 downto 4)),
                                                 action2, data_action_stage(11 downto 8)),
                                                 action3, data_action_stage(15 downto 12)),
                                                 action4, data_action_stage(19 downto 16));

                trace_state5 <= handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(trace_state7, action0, data_action_stage(3 downto 0)),
                                                 action1, data_action_stage(7 downto 4)),
                                                 action2, data_action_stage(11 downto 8)),
                                                 action3, data_action_stage(15 downto 12)),
                                                 action4, data_action_stage(19 downto 16)),
                                                 action5, data_action_stage(23 downto 20));

                trace_state6 <= handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(trace_state7, action0, data_action_stage(3 downto 0)),
                                                 action1, data_action_stage(7 downto 4)),
                                                 action2, data_action_stage(11 downto 8)),
                                                 action3, data_action_stage(15 downto 12)),
                                                 action4, data_action_stage(19 downto 16)),
                                                 action5, data_action_stage(23 downto 20)),
                                                 action6, data_action_stage(27 downto 24));

                trace_state7 <= handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(trace_state7, action0, data_action_stage(3 downto 0)),
                                                 action1, data_action_stage(7 downto 4)),
                                                 action2, data_action_stage(11 downto 8)),
                                                 action3, data_action_stage(15 downto 12)),
                                                 action4, data_action_stage(19 downto 16)),
                                                 action5, data_action_stage(23 downto 20)),
                                                 action6, data_action_stage(27 downto 24)),
                                                 action7, data_action_stage(31 downto 28));

                data_valid_output_stage <= '1';
            end if;
        end if;
     end if;
end process;


-- Output values
output_process : process(aclk)
begin
     if rising_edge(aclk) then
        if aresetn = '0'then
            o_valid0 <= '0';
            o_valid1 <= '0';
            o_valid2 <= '0';
            o_valid3 <= '0';
            o_valid4 <= '0';
            o_valid5 <= '0';
            o_valid6 <= '0';
            o_valid7 <= '0';
        else
            -- TODO: Deassert by default?
            o_valid0 <= '0';
            o_valid1 <= '0';
            o_valid2 <= '0';
            o_valid3 <= '0';
            o_valid4 <= '0';
            o_valid5 <= '0';
            o_valid6 <= '0';
            o_valid7 <= '0';

            if data_valid_output_stage = '1' then
                o_payload0  <= trace_state0.output_state.data_payload;
                o_payload1  <= trace_state1.output_state.data_payload;
                o_payload2  <= trace_state2.output_state.data_payload;
                o_payload3  <= trace_state3.output_state.data_payload;
                o_payload4  <= trace_state4.output_state.data_payload;
                o_payload5  <= trace_state5.output_state.data_payload;
                o_payload6  <= trace_state6.output_state.data_payload;
                o_payload7  <= trace_state7.output_state.data_payload;

                o_valid0  <= trace_state0.output_state.data_valid;
                o_valid1  <= trace_state1.output_state.data_valid;
                o_valid2  <= trace_state2.output_state.data_valid;
                o_valid3  <= trace_state3.output_state.data_valid;
                o_valid4  <= trace_state4.output_state.data_valid;
                o_valid5  <= trace_state5.output_state.data_valid;
                o_valid6  <= trace_state6.output_state.data_valid;
                o_valid7  <= trace_state7.output_state.data_valid;
            end if;
        end if;
     end if;
end process;

end Behavioral;
