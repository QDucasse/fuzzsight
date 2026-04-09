library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.decoder_constants.all;

-- ETM Decoder
--
-- Pipelined CoreSight trace packet decoder, transforming the raw bytes into
-- the decoded architectural state (addresses, timestamps, VMIDs, context IDs).
-- Note: This is the unrolled version of the packet_parser_function, ready to
--       process 4 bytes per cycle
-- Stages:
-- 1. Header Preprocessing
--   Runs on the 32-bit inputs, one word per cycle, calls preprocess onece per byte.
--   Precompute the classification per byte, giving the type of packet. Calls the
--   preprocess() function that in turn calls headerLookupTable() on the first byte.
-- 2. Stream State Processing
--   In the same cycle, the dispatch function handleStreamState() is called on the stream
--   state, redirecting to the actual handling of the next packets.
-- 3. Action Preprocessing
--   Plan the next actions that should be done on the trace state.
-- 4. Trace State Processing
--   Executes up to four actions sequentially in the same cycle, based on the planning done
--   in the previous step.
--
-- MW: maximal load on packet stream is 32 bits/cycle if all trace data is from one ETM.

entity etm_decoder is
    -- The decoding algorithm depends on these values to properly decode the trace.
    -- These are implementation defined (can be checked by reading register TRCIDR2).
    Generic(
        -- Bunch of generics that we don't need at the moment(hardcoded on ultrascale+) but can be important
        -- for porting to different hardware (aka thunderx).
        -- > See Chapter 7.3.32 in TRM ETMv4.0 to ETM v4.4
        VMID_SIZE_BYTE : integer := VMID_SIZE_BYTE; -- On ultrascale plus etms for a53 support only 1 byte of VMID.
        IA_SIZE_BYTE   : integer := IA_SIZE_BYTE;   -- Instruction address size in bytes, for a53 is 8.
        CID_SIZE_BYTE  : integer := CID_SIZE_BYTE;
        DA_SIZE_BYTE   : integer := DA_SIZE_BYTE;
        -- This number is determined to by the largest possible payload that has a bounded size (atm its the
        -- a-sync packet), for example timestamp packet with 9. This size is important to be kept small, as it
        -- can be a bottleneck to make timing constraints.
        ETM_MAX_BOUNDED_PAYLOAD_SIZE : integer := ETM_MAX_BOUNDED_PAYLOAD_SIZE
    );
    port (
        aclk : in std_logic;
        aresetn : in std_logic;

        i_data : in std_logic_vector(31 downto 0);
        i_valid: in std_logic;

        -- From the bram reader
        i_freeze_request : in std_logic;

        o_atom_valid0      : out std_logic;
        o_exception_valid0 : out std_logic;
        o_ts0              : out std_logic_vector(63 downto 0);
        o_address_reg_0_0  : out std_logic_vector(63 downto 0);
        o_ctxt_id0         : out std_logic_vector(CID_SIZE_BYTE * 8 - 1 downto 0);
        o_vmid0            : out std_logic_vector(VMID_SIZE_BYTE * 8 - 1 downto 0);
        o_atom_elements0   : out std_logic_vector(ATOM_ELTS_SIZE - 1 downto 0);
        o_atom_nb0         : out unsigned(ATOM_NB_SIZE - 1 downto 0);
        o_exception_type0  : out std_logic_vector(EXC_TYPE_SIZE - 1 downto 0);

        o_atom_valid1      : out std_logic;
        o_exception_valid1 : out std_logic;
        o_ts1              : out std_logic_vector(63 downto 0);
        o_address_reg_0_1  : out std_logic_vector(63 downto 0);
        o_ctxt_id1         : out std_logic_vector(CID_SIZE_BYTE * 8 - 1  downto 0);
        o_vmid1            : out std_logic_vector(VMID_SIZE_BYTE * 8 - 1 downto 0);
        o_atom_elements1   : out std_logic_vector(ATOM_ELTS_SIZE - 1 downto 0);
        o_atom_nb1         : out unsigned(ATOM_NB_SIZE - 1 downto 0);
        o_exception_type1  : out std_logic_vector(EXC_TYPE_SIZE - 1 downto 0);

        o_atom_valid2      : out std_logic;
        o_exception_valid2 : out std_logic;
        o_ts2              : out std_logic_vector(63 downto 0);
        o_address_reg_0_2  : out std_logic_vector(63 downto 0);
        o_ctxt_id2         : out std_logic_vector(CID_SIZE_BYTE * 8 - 1 downto 0);
        o_vmid2            : out std_logic_vector(VMID_SIZE_BYTE * 8 - 1 downto 0);
        o_atom_elements2   : out std_logic_vector(ATOM_ELTS_SIZE - 1 downto 0);
        o_atom_nb2         : out unsigned(ATOM_NB_SIZE - 1 downto 0);
        o_exception_type2  : out std_logic_vector(EXC_TYPE_SIZE - 1 downto 0);

        o_atom_valid3      : out std_logic;
        o_exception_valid3 : out std_logic;
        o_ts3              : out std_logic_vector(63 downto 0);
        o_address_reg_0_3  : out std_logic_vector(63 downto 0);
        o_ctxt_id3         : out std_logic_vector(CID_SIZE_BYTE * 8 - 1 downto 0);
        o_vmid3            : out std_logic_vector(VMID_SIZE_BYTE * 8 - 1 downto 0);
        o_atom_elements3   : out std_logic_vector(ATOM_ELTS_SIZE - 1 downto 0);
        o_atom_nb3         : out unsigned(ATOM_NB_SIZE - 1 downto 0);
        o_exception_type3  : out std_logic_vector(EXC_TYPE_SIZE - 1 downto 0)
    );
end etm_decoder;

architecture Behavioral of etm_decoder is

    -- ==========================
    --     Type Definitions
    -- ==========================

    -- TODO: Unused?
    type address_reg_t is record
        address : std_logic_vector(63 downto 0);
        instruction_set : std_logic_vector(1 downto 0);
    end record;

    -- The trace maintains three address registers for efficient
    -- reuse of recently seen addresses
    type AddressState is record
        reg0 : std_logic_vector(63 downto 0);
        reg1 : std_logic_vector(63 downto 0);
        reg2 : std_logic_vector(63 downto 0);
    end record;

    type security_level is (
        Secure,
        NonSecure
    );

    -- Complete context state, all elements may require activation
    -- and configuration through CSAL
    type ContextState is record
        context_id : std_logic_vector((CID_SIZE_BYTE) * 8 - 1 downto 0);
        vmid       : std_logic_vector((VMID_SIZE_BYTE) * 8 - 1 downto 0);
        ex_level   : std_logic_vector(1 downto 0);
        security   : security_level;
        AArch64    : boolean;

        curr_spec_depth   : integer;
        p0_key            : integer;
        cond_c_key        : integer;
        cond_r_key        : integer;
        p0_key_max        : integer;
        cond_key_max_incr : integer;
        max_spec_depth    : integer;
        cc_threshold      : integer;
    end record;

    type OutputState is record
        timestamp       : std_logic_vector(63 downto 0);
        atom_valid      : std_logic;
        atom_elements   : std_logic_vector(ATOM_ELTS_SIZE - 1 downto 0);
        atom_nb         : integer;
        exception_type  : std_logic_vector(EXC_TYPE_SIZE - 1 downto 0);
        exception_valid : std_logic;
    end record;

    -- Trace State from Arm Embedded Trace Macrocell Architecture Specification ETMv4.0 to ETMv4.4, chapter 6.2
    type TraceState is record
        output_state  : OutputState;
        address_state : AddressState;
        context_state : ContextState;
        synchronized  : std_logic; -- reset between runs, waits for TraceOn
        -- This denotes whether the tracestate is valid. The trace state is INVALID,
        -- for example, if we are in the process of updating the timestamp.
        -- This takes multiple packets to update.
    end record;

    -- Headers from Arm Embedded Trace Macrocell Architecture Specification ETMv4.0 to ETMv4.4, chapter 6.3
    type HeaderType is (
        Undefined,
        Extension,
        TraceInfo,
        Timestamp,
        TimestampF1,
        TimestampF2,
        TraceOn,
        FunctionReturn, -- only in Armv8-M PE if data tracing is enabled.
        Exception,
        ExceptionReturn,
        CycleCountingF2,
        CycleCountingF1,
        CycleCountingF3,
        NumDataSyncMark,
        UnnumDataSyncMark,
        Commit,
        CancelF1,
        Mispredict,
        CancelF2,
        CancelF3,
        CondInstrF2,
        CondFlush,
        CondResF4,
        CondResF2,
        CondResF3,
        CondResF1,
        CondInstrF1, -- multiple value entries for this header!
        CondInstrF3,
        Ignore,
        TraceEvent,
        Context,
        ContextF1,
        ContextF2,
        AddressWithContext_32bit_IS0,
        AddressWithContext_32bit_IS1,
        AddressWithContext_64bit_IS0,
        AddressWithContext_64bit_IS1,
        ExactMatchAddress,
        -- Two different types of ShortAddress Format
        ShortAddress_IS0,
        ShortAddress_IS1,
        -- Four different types of LongAddressFormat that need to be handled in different ways.
        LongAddress_32bit_IS0,
        LongAddress_32bit_IS1,
        LongAddress_64bit_IS0,
        LongAddress_64bit_IS1,
        QEvent,
        AtomF6,     -- multiple value entries for this header!
        AtomF5,     -- multiple value entries for this header!
        AtomF2,
        AtomF4,
        AtomF1,
        AtomF3,

        -- The following Header types are only seen INSIDE specific Packet types.
        -- For example Trace Info has SpeculationDepthSection (Spec).

        Async,

        TraceInfoPlctl, -- internal traceinfo packet
        TraceInfoInfo,  -- internal traceinfo packet
        TraceInfoKey,   -- internal traceinfo packet
        TraceInfoSpec,  -- internal traceinfo packet
        TraceInfoCyct,  -- internal traceinfo packet

        InternalCycleCount, -- internal Timestamp packet

        ContextVMID,         -- internal Context packet
        ContextContextID,    -- internal Context packet
        ContextInfo          -- internal Context packet

    );


    -- ==========================
    --         Functions
    -- ==========================

    -- Streaming state and mode of the decoder
    -- \__________________________________________

    -- Streaming mode of the current packet We are either parsing a header or a payload.
    -- And for a payload, it is either fixed size or continuous. Continuous requires to check
    -- for the continuation bit (bit 7, continues if high).

    type StreamMode is (
        next_byte_must_be_header, -- initial state of StreamMode
        extension_mode,
        payload_continuous,
        payload_continuous_unbounded, -- we differentiate continuous payloads with a maximum size (see timestamp to unbounded payloads, see commit)
        payload_fixed_size,

        -- special mode for INFO packets, see Context or TraceInfo packets. This category is for packets that
        -- have non header bytes that affect the stream state.
        context_info_mode,
        trace_info_plctl_mode
    );

    type StreamState is record
        -- Current streaming mode
        stream_mode       :  StreamMode;
        -- Current payload index
        payload_index     : integer range 0 to ETM_MAX_BOUNDED_PAYLOAD_SIZE;
        payload_max_index : integer range 0 to ETM_MAX_BOUNDED_PAYLOAD_SIZE;
        -- Note: payload must be exactly this for fixed size and at maximum this for continuous

        -- Stored when processing a payload
        header_t : HeaderType;
        header : std_logic_vector(7 downto 0);
        -- Note: sometimes we need to access the header to decide process state correctly

        -- Pipeline for the next bytes
        next_stream_mode0 : StreamMode;
        next_header_t0 : HeaderType;
        next_payload_max_index0 : integer range 0 to ETM_MAX_BOUNDED_PAYLOAD_SIZE;

        next_stream_mode1 : StreamMode;
        next_header_t1 : HeaderType;
        next_payload_max_index1 : integer range 0 to ETM_MAX_BOUNDED_PAYLOAD_SIZE;

        next_stream_mode2 : StreamMode;
        next_header_t2 : HeaderType;
        next_payload_max_index2 : integer range 0 to ETM_MAX_BOUNDED_PAYLOAD_SIZE;

        next_stream_mode3 : StreamMode;
        next_header_t3 : HeaderType;
        next_payload_max_index3 : integer range 0 to ETM_MAX_BOUNDED_PAYLOAD_SIZE;
    end record;


    -- 1. Linked to packet preprocessing
    -- \___________________________________

    type PreprocessingState is record
        header : HeaderType;
    end record;

        -- Resolves the complete set of headers from Embedded Trace Macrocell
    -- Architecture Specification ETMv4.0 to ETMv4.4, chapter 6.3.1
    pure function headerLookupTable(header : std_logic_vector(7 downto 0)) return HeaderType is
        variable header_u : unsigned (7 downto 0);
    begin
        header_u := unsigned(header);
        if header = "00000000" then
            return Extension;
        elsif header = "00000001" then
            return TraceInfo;
        elsif header = "00000010" then
            return TimestampF1;
        elsif header = "00000011" then
            return TimestampF2;
        elsif header = "00000100" then
            return TraceOn;
        elsif header = "00000101" then
            return FunctionReturn;
        elsif header = "00000110" then
            return Exception;
        elsif header(7 downto 1) = "0000110" then
            return CycleCountingF2;
        elsif header(7 downto 1) = "0000111" then
            return CycleCountingF1;
        elsif header(7 downto 4) = "0001" then
            return CycleCountingF3;
        elsif header(7 downto 3) = "00100" then
            return NumDataSyncMark;
        elsif header(7 downto 2) = "001010" or header = "00101100" then
            return UnnumDataSyncMark;
        elsif header = "00101101" then
            return Commit;
        elsif header(7 downto 1) = "0010111" then
            return CancelF1;
        elsif header(7 downto 2) = "001100" then
            return Mispredict;
        elsif header(7 downto 2) = "001101" then
            return CancelF2;
        elsif header(7 downto 3) = "00111" then
            return CancelF3;
        elsif header(7 downto 1) = "0100000" or header = "01000010" then
            return CondInstrF2;
        elsif header = "01000011" then
            return CondFlush;
        elsif header(7 downto 1) = "0100010" or header = "01000110" then
            return CondResF4;
        elsif header(7 downto 1) = "0100100" or header = "01001010" then
            return CondResF2;
        elsif header(7 downto 1) = "0100110" or header = "01001110" then
            return CondResF2; -- twice CondResF2, not a typo
        elsif header(7 downto 4) = "0101" then
            return CondResF3;
        elsif header(7 downto 2) = "011010" then
            return CondResF1;
        elsif header = "01101100" then
            return CondInstrF1;
        elsif header = "01101101" then
            return CondInstrF3;
        elsif header(7 downto 1) = "0110111" then
            return CondResF1;
        elsif header = "01110000" then
            return Ignore;
        elsif header_u >= "01110001" and header_u <= "01111111"  then
            return TraceEvent;
        elsif header= "10000000" then
            return ContextF1;
        elsif header = "10000001" then
            return ContextF2;
        elsif header = "10000010" then
            return AddressWithContext_32bit_IS0;
        elsif header = "10000011" then
            return AddressWithContext_32bit_IS1;
        elsif header = "10000101" then
            return AddressWithContext_64bit_IS0;
        elsif header = "10000110" then
            return AddressWithContext_64bit_IS1;
        elsif header_u >= "10010000" and header_u <= "10010010" then
            return ExactMatchAddress;
        elsif header_u >= "10010011" and header_u <= "10010100" then
            return ExactMatchAddress;
        elsif header = "10010101" then
            return ShortAddress_IS0;
        elsif header = "10010110" then
            return ShortAddress_IS1;
        elsif header = "10011010" then
            return LongAddress_32bit_IS0;
        elsif header = "10011011" then
            return LongAddress_32bit_IS1;
        elsif header = "10011101" then
            return LongAddress_64bit_IS0;
        elsif header = "10011110" then
            return LongAddress_64bit_IS1;
        elsif header(7 downto 4) = "1010" then
            return QEvent;
        elsif header_u >= "11000000" and header_u <= "11010100" then
            return AtomF6;
        elsif header_u >= "11010101" and header_u <= "11010111" then
            return AtomF5;
        elsif header(7 downto 2) = "110110" then
            return AtomF2;
        elsif header(7 downto 2) = "110111" then
            return AtomF4;
        elsif header_u >= "11100000" and header_u <= "11110100" then
            return AtomF6;
        elsif header = "11110101" then
            return AtomF5;
        elsif header(7 downto 1) = "1111011" then
            return AtomF1;
        elsif header(7 downto 3) = "11111" then
            return AtomF3;
        else
            return Undefined;
        end if;
    end;

    -- Simply looks for the header
    pure function preprocess(byte : std_logic_vector(7 downto 0)) return PreprocessingState is
    variable preprocessing_state : PreprocessingState;
    begin
        preprocessing_state.header := headerLookupTable(byte);
        return preprocessing_state;
    end;


    -- 2. Linked to Stream State Processing
    -- \______________________________________

    -- Extension mode, only async/discard/overflow packets
    pure function handleStreamStateExtension(in_stream_state : StreamState; preprocessing_state : PreprocessingState; byte : std_logic_vector(7 downto 0)) return StreamState is
    variable out_stream_state : StreamState;
    begin
        case byte is
            when "00000000" => -- async, fixed size of max index
                out_stream_state.header            := byte;
                out_stream_state.payload_max_index := 9;
                out_stream_state.payload_index     := 0;
                out_stream_state.header_t          := Extension;
                out_stream_state.stream_mode       := payload_fixed_size;
                out_stream_state.next_stream_mode0 := next_byte_must_be_header;
                return out_stream_state;
            when "00000011" =>  -- discard
                out_stream_state.stream_mode := next_byte_must_be_header;
                return out_stream_state;
            when "00000101" =>  -- overflow
                out_stream_state.stream_mode := next_byte_must_be_header;
                return out_stream_state;
            when others =>
                return in_stream_state;
        end case;
        -- should not reach here,  we are in error state...
    end;

    -- Payload fixed size mode, if the index is max, shifts to the next mode,
    -- Otherwise, increase index, stay in this mode and propagate lookahead
    pure function handleStreamStatePayloadFixedSize(in_stream_state : StreamState; preprocessing_state : PreprocessingState; byte : std_logic_vector(7 downto 0)) return StreamState is
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
            out_stream_state.next_stream_mode0       := in_stream_state.next_stream_mode1;
            out_stream_state.next_header_t0          := in_stream_state.next_header_t1;
            out_stream_state.next_payload_max_index0 := in_stream_state.next_payload_max_index1;

            out_stream_state.next_stream_mode1       := in_stream_state.next_stream_mode2;
            out_stream_state.next_header_t1          := in_stream_state.next_header_t2;
            out_stream_state.next_payload_max_index1 := in_stream_state.next_payload_max_index2;

            out_stream_state.next_stream_mode2       := in_stream_state.next_stream_mode3;
            out_stream_state.next_header_t2          := in_stream_state.next_header_t3;
            out_stream_state.next_payload_max_index2 := in_stream_state.next_payload_max_index3;
        else
            -- Payload index is not the max, we stay in the payload_fixed_size mode
            -- and increment the payload index.
            out_stream_state.stream_mode       := payload_fixed_size;
            out_stream_state.payload_index     := in_stream_state.payload_index + 1;
            out_stream_state.payload_max_index := in_stream_state.payload_max_index;
            out_stream_state.header            := in_stream_state.header;
            out_stream_state.header_t          := in_stream_state.header_t;

            -- Propagate stream lookahead knowledge
            out_stream_state.next_stream_mode0 := in_stream_state.next_stream_mode0;
            out_stream_state.next_stream_mode1 := in_stream_state.next_stream_mode1;
            out_stream_state.next_stream_mode2 := in_stream_state.next_stream_mode2;
            out_stream_state.next_stream_mode3 := in_stream_state.next_stream_mode3;

            out_stream_state.next_header_t0 := in_stream_state.next_header_t0;
            out_stream_state.next_header_t1 := in_stream_state.next_header_t1;
            out_stream_state.next_header_t2 := in_stream_state.next_header_t2;
            out_stream_state.next_header_t3 := in_stream_state.next_header_t3;

            out_stream_state.next_payload_max_index0 := in_stream_state.next_payload_max_index0;
            out_stream_state.next_payload_max_index1 := in_stream_state.next_payload_max_index1;
            out_stream_state.next_payload_max_index2 := in_stream_state.next_payload_max_index2;
            out_stream_state.next_payload_max_index3 := in_stream_state.next_payload_max_index3;
        end if;
        return out_stream_state;
    end;


    -- Payload continuous mode, if the index is max or the continuation bit is low, shifts to the next mode,
    -- Otherwise, increase index, stay in this mode and propagate lookahead
    pure function handleStreamStatePayloadContinuous(in_stream_state : StreamState; preprocessing_state : PreprocessingState; byte : std_logic_vector(7 downto 0)) return StreamState is
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
            out_stream_state.next_stream_mode0       := in_stream_state.next_stream_mode1;
            out_stream_state.next_header_t0          := in_stream_state.next_header_t1;
            out_stream_state.next_payload_max_index0 := in_stream_state.next_payload_max_index1;

            out_stream_state.next_stream_mode1       := in_stream_state.next_stream_mode2;
            out_stream_state.next_header_t1          := in_stream_state.next_header_t2;
            out_stream_state.next_payload_max_index1 := in_stream_state.next_payload_max_index2;

            out_stream_state.next_stream_mode2       := in_stream_state.next_stream_mode3;
            out_stream_state.next_header_t2          := in_stream_state.next_header_t3;
            out_stream_state.next_payload_max_index2 := in_stream_state.next_payload_max_index3;
        elsif byte(7) = '0' then -- Continuation bit low, this is the last byte of the payload
            -- /!\ DUPLICATED FOR TIMING REASONS /!\
            -- Last byte reached, we switch to the next stream mode,
            -- update the header, and other values.
            out_stream_state.stream_mode       := in_stream_state.next_stream_mode0;
            out_stream_state.header_t          := in_stream_state.next_header_t0;
            out_stream_state.payload_index     := 0;
            out_stream_state.payload_max_index := in_stream_state.next_payload_max_index0;

            -- Shift lookahead
            out_stream_state.next_stream_mode0       := in_stream_state.next_stream_mode1;
            out_stream_state.next_header_t0          := in_stream_state.next_header_t1;
            out_stream_state.next_payload_max_index0 := in_stream_state.next_payload_max_index1;

            out_stream_state.next_stream_mode1       := in_stream_state.next_stream_mode2;
            out_stream_state.next_header_t1          := in_stream_state.next_header_t2;
            out_stream_state.next_payload_max_index1 := in_stream_state.next_payload_max_index2;

            out_stream_state.next_stream_mode2       := in_stream_state.next_stream_mode3;
            out_stream_state.next_header_t2          := in_stream_state.next_header_t3;
            out_stream_state.next_payload_max_index2 := in_stream_state.next_payload_max_index3;

        else
            -- Payload index is not the max, or the byte is not the last one (bit 7 is high),
            -- we stay in the payload_continuous and increment the payload index.
            out_stream_state.stream_mode       :=  payload_continuous;
            out_stream_state.payload_index     := in_stream_state.payload_index + 1;
            out_stream_state.payload_max_index := in_stream_state.payload_max_index;
            out_stream_state.header            := in_stream_state.header;
            out_stream_state.header_t          := in_stream_state.header_t;

            -- Propagate stream lookahead knowledge
            out_stream_state.next_stream_mode0 := in_stream_state.next_stream_mode0;
            out_stream_state.next_stream_mode1 := in_stream_state.next_stream_mode1;
            out_stream_state.next_stream_mode2 := in_stream_state.next_stream_mode2;
            out_stream_state.next_stream_mode3 := in_stream_state.next_stream_mode3;

            out_stream_state.next_header_t0 := in_stream_state.next_header_t0;
            out_stream_state.next_header_t1 := in_stream_state.next_header_t1;
            out_stream_state.next_header_t2 := in_stream_state.next_header_t2;
            out_stream_state.next_header_t3 := in_stream_state.next_header_t3;

            out_stream_state.next_payload_max_index0 := in_stream_state.next_payload_max_index0;
            out_stream_state.next_payload_max_index1 := in_stream_state.next_payload_max_index1;
            out_stream_state.next_payload_max_index2 := in_stream_state.next_payload_max_index2;
        out_stream_state.next_payload_max_index3 := in_stream_state.next_payload_max_index3;

        end if;
        return out_stream_state;
    end;


    -- Payload continuous mode, if the index is max or the continuation bit is low, shifts to the next mode,
    -- Otherwise, increase index, stay in this mode and propagate lookahead
    pure function handleStreamStatePayloadContinuousUnbounded(in_stream_state : StreamState; preprocessing_state : PreprocessingState; byte : std_logic_vector(7 downto 0)) return StreamState is
    variable out_stream_state : StreamState;
    begin
        -- Note: Same as the bounded version but the continuation bit is the ONLY stop reason
        if byte(7) = '0' then -- Continuation bit low, this is the last byte of the payload
            -- /!\ DUPLICATED FOR TIMING REASONS /!\
            -- Last byte reached, we switch to the next stream mode,
            -- update the header, and other values.
            out_stream_state.stream_mode       := in_stream_state.next_stream_mode0;
            out_stream_state.header_t          := in_stream_state.next_header_t0;
            out_stream_state.payload_index     := 0;
            out_stream_state.payload_max_index := in_stream_state.next_payload_max_index0;

            -- Shift lookahead
            out_stream_state.next_stream_mode0       := in_stream_state.next_stream_mode1;
            out_stream_state.next_header_t0          := in_stream_state.next_header_t1;
            out_stream_state.next_payload_max_index0 := in_stream_state.next_payload_max_index1;

            out_stream_state.next_stream_mode1       := in_stream_state.next_stream_mode2;
            out_stream_state.next_header_t1          := in_stream_state.next_header_t2;
            out_stream_state.next_payload_max_index1 := in_stream_state.next_payload_max_index2;

            out_stream_state.next_stream_mode2       := in_stream_state.next_stream_mode3;
            out_stream_state.next_header_t2          := in_stream_state.next_header_t3;
            out_stream_state.next_payload_max_index2 := in_stream_state.next_payload_max_index3;

        else
            -- This byte is not the last one (bit 7 is high), we stay in the payload_continuous_unbounded
            -- mode and increment the payload index.
            out_stream_state.stream_mode   := payload_continuous_unbounded;
            out_stream_state.payload_index := in_stream_state.payload_index + 1;
            -- out_stream_state.payload_max_index := in_stream_state.payload_max_index; -- we don't care in unbounded

            out_stream_state.header   := in_stream_state.header;
            out_stream_state.header_t := in_stream_state.header_t;

            -- Propagate stream lookahead knowledge
            out_stream_state.next_stream_mode0 := in_stream_state.next_stream_mode0;
            out_stream_state.next_stream_mode1 := in_stream_state.next_stream_mode1;
            out_stream_state.next_stream_mode2 := in_stream_state.next_stream_mode2;
            out_stream_state.next_stream_mode3 := in_stream_state.next_stream_mode3;

            out_stream_state.next_header_t0 := in_stream_state.next_header_t0;
            out_stream_state.next_header_t1 := in_stream_state.next_header_t1;
            out_stream_state.next_header_t2 := in_stream_state.next_header_t2;
            out_stream_state.next_header_t3 := in_stream_state.next_header_t3;

            out_stream_state.next_payload_max_index0 := in_stream_state.next_payload_max_index0;
            out_stream_state.next_payload_max_index1 := in_stream_state.next_payload_max_index1;
            out_stream_state.next_payload_max_index2 := in_stream_state.next_payload_max_index2;
            out_stream_state.next_payload_max_index3 := in_stream_state.next_payload_max_index3;

        end if;
        return out_stream_state;
    end;

    -- Header mode, sets the next modes and payload size based on the header
    pure function handleStreamStateHeader(in_stream_state : StreamState; preprocessing_state : PreprocessingState; byte : std_logic_vector(7 downto 0)) return StreamState is
    variable out_stream_state : StreamState;
    begin
        case preprocessing_state.header is

        when TimestampF1 =>
            out_stream_state.stream_mode       := payload_continuous;
            out_stream_state.header_t          := preprocessing_state.header;
            out_stream_state.payload_max_index := 8;

            out_stream_state.next_stream_mode0 := next_byte_must_be_header;
            out_stream_state.next_header_t0    := Undefined;

        when TimestampF2 =>
            out_stream_state.stream_mode             := payload_continuous;
            out_stream_state.header_t                := preprocessing_state.header;
            out_stream_state.payload_max_index       := 8;

            out_stream_state.next_stream_mode0       := payload_continuous;
            out_stream_state.next_header_t0          := InternalCycleCount;
            out_stream_state.next_payload_max_index0 := 2;

            out_stream_state.next_stream_mode1       := next_byte_must_be_header;
            out_stream_state.next_header_t1          := Undefined;

        when LongAddress_32bit_IS0|LongAddress_32bit_IS1 =>
            out_stream_state.stream_mode       := payload_fixed_size;
            out_stream_state.header_t          := preprocessing_state.header;
            out_stream_state.payload_max_index := 3;

            out_stream_state.next_stream_mode0 := next_byte_must_be_header;
            out_stream_state.next_header_t0    := Undefined;

        when AddressWithContext_32bit_IS0|AddressWithContext_32bit_IS1 =>
            out_stream_state.stream_mode       := payload_fixed_size;
            out_stream_state.header_t          := preprocessing_state.header;
            out_stream_state.payload_max_index := 3;

            out_stream_state.next_stream_mode0 := context_info_mode;
            out_stream_state.next_header_t0    := ContextInfo;
            -- size doens't matter here
            out_stream_state.next_payload_max_index0 := 0;

        when LongAddress_64bit_IS0|LongAddress_64bit_IS1 =>
            out_stream_state.stream_mode       := payload_fixed_size;
            out_stream_state.payload_max_index := 7;
            out_stream_state.header_t          := preprocessing_state.header;

            out_stream_state.next_stream_mode0 := next_byte_must_be_header;
            out_stream_state.next_header_t0    := Undefined;

        when AddressWithContext_64bit_IS0|AddressWithContext_64bit_IS1 =>
            out_stream_state.stream_mode       := payload_fixed_size;
            out_stream_state.payload_max_index := 7;
            out_stream_state.header_t          := preprocessing_state.header;

            out_stream_state.next_stream_mode0 := context_info_mode;
            out_stream_state.next_header_t0    := ContextInfo;
            -- size doens't matter here
            out_stream_state.next_payload_max_index0 := 0;

        when ShortAddress_IS0|ShortAddress_IS1 =>
            out_stream_state.stream_mode       := payload_continuous;
            out_stream_state.payload_max_index := 1;
            out_stream_state.header_t          := preprocessing_state.header;

            out_stream_state.next_stream_mode0 := next_byte_must_be_header;
            out_stream_state.next_header_t0    := Undefined;

        when ContextF1 =>
            out_stream_state.stream_mode       := next_byte_must_be_header;
            out_stream_state.header_t          := preprocessing_state.header;

            out_stream_state.next_stream_mode0 := next_byte_must_be_header;
            out_stream_state.next_header_t0    := Undefined;

        when ContextF2 =>
            out_stream_state.stream_mode       := context_info_mode;
            out_stream_state.payload_max_index := 0;
            out_stream_state.header_t          := ContextInfo;

            -- next stream State doesn't matter, the info byte will have the complete overview.
            out_stream_state.next_stream_mode0 := next_byte_must_be_header;
            out_stream_state.next_header_t0    := Undefined;

        when TraceInfo =>
            out_stream_state.stream_mode := trace_info_plctl_mode;
            out_stream_state.header_t    := preprocessing_state.header;

            -- next stream State doesn't matter, the info byte will have the complete overview.
            out_stream_state.next_stream_mode0 := next_byte_must_be_header;
            out_stream_state.next_header_t0    := Undefined;

        when Exception =>
            out_stream_state.stream_mode       := payload_continuous;
            out_stream_state.payload_max_index := 2;
            out_stream_state.header_t          := preprocessing_state.header;

            -- Next packet is an address
            out_stream_state.next_stream_mode0 := next_byte_must_be_header;
            out_stream_state.next_header_t0    := Undefined;


        -- All these packets consist only of a header
        when TraceOn|FunctionReturn|ExceptionReturn|CycleCountingF3|NumDataSyncMark
            |UnnumDataSyncMark|Mispredict|CancelF2|CancelF3|CondInstrF2
            |CondFlush|CondResF4|CondResF2|Ignore|TraceEvent|ExactMatchAddress|AtomF1
            |AtomF2|AtomF3|AtomF4|AtomF5|AtomF6 =>
            out_stream_state.stream_mode := next_byte_must_be_header;
            out_stream_state.header_t    := preprocessing_state.header;

        when Extension =>
            out_stream_state.stream_mode := extension_mode;

        when others =>
            out_stream_state.stream_mode := next_byte_must_be_header;
            out_stream_state.header_t    := preprocessing_state.header;
            -- do nothing for now
        end case;

        out_stream_state.payload_index := 0;
        return out_stream_state;
    end;

    -- Context Info Mode, the context information byte looks like this:
    --   7    6    5    4   3      2 1      0
    -- ┌────┬────┬────┬────┬────────┬────────┐
    -- │ C  │ V  │ NS │ SF │  SBZ   │   EL   │
    -- └────┴────┴────┴────┴────────┴────────┘
    --
    -- V: Contains VMID bytes (right after this byte for VMID size)
    -- C: Contains CID bytes  (right after VMID if present for CID size)
    pure function handleStreamStateContextInfoMode(in_stream_state : StreamState; preprocessing_state : PreprocessingState; byte : std_logic_vector(7 downto 0)) return StreamState is
    variable out_stream_state : StreamState;
    begin
        if byte(7 downto 6) = "00" then -- C low and V low, CID and VMID absent
            out_stream_state.stream_mode  := next_byte_must_be_header;
            out_stream_state.header_t :=  Undefined;
            out_stream_state.payload_max_index := 0;

        elsif byte(7 downto 6) = "10" then -- C high, CID present
            out_stream_state.stream_mode             := payload_fixed_size;
            out_stream_state.payload_max_index       := CID_SIZE_BYTE - 1;
            out_stream_state.header_t                := ContextContextID;

            out_stream_state.next_stream_mode0       := next_byte_must_be_header;
            out_stream_state.next_payload_max_index0 := 0;
            out_stream_state.next_header_t0          := Undefined;

        elsif byte(7 downto 6) = "01" then -- V high, VMID Present
            out_stream_state.stream_mode             := payload_fixed_size;
            out_stream_state.payload_max_index       := VMID_SIZE_BYTE - 1;
            out_stream_state.header_t                := ContextVMID;

            out_stream_state.next_stream_mode0       := next_byte_must_be_header;
            out_stream_state.next_payload_max_index0 := 0;
            out_stream_state.next_header_t0          := Undefined;

        elsif byte(7 downto 6) = "11" then -- C high and V high, CID and VMID present
            out_stream_state.stream_mode             := payload_fixed_size;
            out_stream_state.payload_max_index       := VMID_SIZE_BYTE - 1;
            out_stream_state.header_t                := ContextVMID;

            out_stream_state.next_stream_mode0       := payload_fixed_size;
            out_stream_state.next_payload_max_index0 := CID_SIZE_BYTE - 1;
            out_stream_state.next_header_t0          := ContextContextID;

            out_stream_state.next_stream_mode1       := next_byte_must_be_header;
            out_stream_state.next_payload_max_index1 := 0;
            out_stream_state.next_header_t1          := Undefined;

        end if;
        out_stream_state.payload_index := 0;
        return out_stream_state;
    end;

    -- PLCTL Mode, the PLCTL byte looks like this:
    --   7   6   4   3      2      1      0
    -- ┌────┬─────┬──────┬──────┬──────┬──────┐
    -- │ C  │  U  │ INFO │ KEY  │ SPEC │ CYCT │
    -- └────┴─────┴──────┴──────┴──────┴──────┘
    --
    -- C: Continuation bit, not linked to plctl info
    -- U: Unused
    --
    -- INFO, KEY, SPEC, CYCT signal the presence of their respective sections when high

    pure function handleStreamStateTraceInfoPlctlMode(in_stream_state : StreamState; preprocessing_state : PreprocessingState; byte : std_logic_vector(7 downto 0)) return StreamState is
    variable out_stream_state : StreamState;
    begin
        -- the docs are a little weird on this one: it suggest that there can be multiple bytes of PLCTL, but at the same time say:
        -- "A trace unit must not output more than 1 PLCTL field in a Trace Info packet" - architecture specification ETVMv4.0 to ETMv4.4,Ch. 6.4
        -- ==> we assume PLTCL is always 1 byte.

        -- The code we write in the name of optimization... (and meeting timing constraints)

        if byte(3 downto 0) = "0000" then -- No additional section
            out_stream_state.stream_mode := next_byte_must_be_header;
            out_stream_state.header_t := Undefined;

        elsif byte(3 downto 0) = "1111" then -- All sections
            -- Info
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoInfo;
            -- Key
            out_stream_state.next_stream_mode0 := payload_continuous_unbounded;
            out_stream_state.next_header_t0    := TraceInfoKey;
            -- Spec
            out_stream_state.next_stream_mode1 := payload_continuous_unbounded;
            out_stream_state.next_header_t1    := TraceInfoSpec;
            -- Cyct
            out_stream_state.next_stream_mode2 := payload_continuous_unbounded;
            out_stream_state.next_header_t2    := TraceInfoCyct;

            out_stream_state.next_stream_mode3       := next_byte_must_be_header;
            out_stream_state.next_header_t2          := Undefined;
            out_stream_state.next_payload_max_index3 := 0;

        elsif byte(3 downto 0) = "0001" then
            -- Info
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoInfo;

            out_stream_state.next_stream_mode0       := next_byte_must_be_header;
            out_stream_state.next_header_t0          := Undefined;
            out_stream_state.next_payload_max_index0 := 0;

        elsif byte(3 downto 0) = "0010" then
            -- Key
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoKey;

            out_stream_state.next_stream_mode0       := next_byte_must_be_header;
            out_stream_state.next_header_t0          := Undefined;
            out_stream_state.next_payload_max_index0 := 0;

        elsif byte(3 downto 0) = "0100" then
            -- Spec
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoSpec;

            out_stream_state.next_stream_mode0       := next_byte_must_be_header;
            out_stream_state.next_header_t0          := Undefined;
            out_stream_state.next_payload_max_index0 := 0;

        elsif byte(3 downto 0) = "1000" then
            -- Cyct
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoCyct;

            out_stream_state.next_stream_mode0       := next_byte_must_be_header;
            out_stream_state.next_header_t0          := Undefined;
            out_stream_state.next_payload_max_index0 := 0;

        elsif byte(3 downto 0) = "0011" then
            -- TODO: QD, was cyct then key where reverted
            -- Info
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoInfo;
            -- Key
            out_stream_state.next_stream_mode0 := payload_continuous_unbounded;
            out_stream_state.next_header_t0    := TraceInfoKey;

            out_stream_state.next_stream_mode1       := next_byte_must_be_header;
            out_stream_state.next_header_t1          := Undefined;
            out_stream_state.next_payload_max_index1 := 0;

        elsif byte(3 downto 0) = "0110" then
            -- Key
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoKey;
            -- Spec
            out_stream_state.next_stream_mode0 := payload_continuous;
            out_stream_state.next_header_t0    := TraceInfoSpec;

            out_stream_state.next_stream_mode1       := next_byte_must_be_header;
            out_stream_state.next_header_t1          := Undefined;
            out_stream_state.next_payload_max_index1 := 0;

        elsif byte(3 downto 0) = "1100" then
            -- Spec
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoSpec;
            -- Cyct
            out_stream_state.next_stream_mode0 := payload_continuous_unbounded;
            out_stream_state.next_header_t0    := TraceInfoCyct;

            out_stream_state.next_stream_mode1       := next_byte_must_be_header;
            out_stream_state.next_header_t1          := Undefined;
            out_stream_state.next_payload_max_index1 := 0;

        elsif byte(3 downto 0) = "1001" then
            -- Info
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoInfo;
            -- Cyct
            out_stream_state.next_stream_mode0 := payload_continuous_unbounded;
            out_stream_state.next_header_t0    := TraceInfoCyct;

            out_stream_state.next_stream_mode1       := next_byte_must_be_header;
            out_stream_state.next_header_t1          := Undefined;
            out_stream_state.next_payload_max_index1 := 0;

        elsif byte(3 downto 0) = "0101" then
            -- Info
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoInfo;
            -- Spec
            out_stream_state.next_stream_mode0 := payload_continuous_unbounded;
            out_stream_state.next_header_t0    := TraceInfoSpec;

            out_stream_state.next_stream_mode1       := next_byte_must_be_header;
            out_stream_state.next_header_t1          := Undefined;
            out_stream_state.next_payload_max_index1 := 0;

        elsif byte(3 downto 0) = "1010" then
            -- Key
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoKey;
            -- Cyct
            out_stream_state.next_stream_mode0 := payload_continuous_unbounded;
            out_stream_state.next_header_t0    := TraceInfoCyct;

            out_stream_state.next_stream_mode1       := next_byte_must_be_header;
            out_stream_state.next_header_t1          := Undefined;
            out_stream_state.next_payload_max_index1 := 0;

        elsif byte(3 downto 0) = "0111" then
            -- Info
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoInfo;
            -- Key
            out_stream_state.next_stream_mode0 := payload_continuous_unbounded;
            out_stream_state.next_header_t1    := TraceInfoKey;
            -- Spec
            out_stream_state.next_stream_mode1 := payload_continuous_unbounded;
            out_stream_state.next_header_t1    := TraceInfoSpec;

            out_stream_state.next_stream_mode2       := next_byte_must_be_header;
            out_stream_state.next_header_t2          := Undefined;
            out_stream_state.next_payload_max_index2 := 0;

        elsif byte(3 downto 0) = "1110" then
            -- Key
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoKey;
            -- Spec
            out_stream_state.next_stream_mode0 := payload_continuous_unbounded;
            out_stream_state.next_header_t1    := TraceInfoSpec;
            -- Cyct
            out_stream_state.next_stream_mode1 := payload_continuous_unbounded;
            out_stream_state.next_header_t1    := TraceInfoCyct;

            out_stream_state.next_stream_mode2       := next_byte_must_be_header;
            out_stream_state.next_header_t2          := Undefined;
            out_stream_state.next_payload_max_index2 := 0;

        elsif byte(3 downto 0) = "1011" then
            -- Info
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoInfo;
            -- Key
            out_stream_state.next_stream_mode0 := payload_continuous_unbounded;
            out_stream_state.next_header_t1    := TraceInfoKey;
            -- Cyct
            out_stream_state.next_stream_mode1 := payload_continuous_unbounded;
            out_stream_state.next_header_t1    := TraceInfoCyct;

            out_stream_state.next_stream_mode2 := next_byte_must_be_header;
            out_stream_state.next_header_t2    := Undefined;
            out_stream_state.next_payload_max_index2 := 0;

        elsif byte(3 downto 0) = "1101" then
            -- Info
            out_stream_state.stream_mode := payload_continuous_unbounded;
            out_stream_state.header_t    := TraceInfoInfo;
            -- Spec
            out_stream_state.next_stream_mode0 := payload_continuous_unbounded;
            out_stream_state.next_header_t1    := TraceInfoSpec;
            -- Cyct
            out_stream_state.next_stream_mode1 := payload_continuous_unbounded;
            out_stream_state.next_header_t1    := TraceInfoCyct;

            out_stream_state.next_stream_mode2       := next_byte_must_be_header;
            out_stream_state.next_header_t2          := Undefined;
            out_stream_state.next_payload_max_index2 := 0;
        end if;

        out_stream_state.payload_index := 0;
        return out_stream_state;
    end;


    -- Dispatch function based on the stream mode
    pure function handleStreamState(in_stream_state : StreamState; preprocessing_state : PreprocessingState; byte : std_logic_vector(7 downto 0)) return StreamState is
    variable out_stream_state : StreamState;
    begin
        case in_stream_state.stream_mode is
            when extension_mode =>
                return handleStreamStateExtension(in_stream_state, preprocessing_state, byte);
            when payload_fixed_size => -- special packet mode
                return handleStreamStatePayloadFixedSize(in_stream_state, preprocessing_state, byte);
            when payload_continuous =>
                return handleStreamStatePayloadContinuous(in_stream_state, preprocessing_state, byte);
            when payload_continuous_unbounded =>
                return handleStreamStatePayloadContinuousUnbounded(in_stream_state, preprocessing_state, byte);
            when next_byte_must_be_header =>
                return handleStreamStateHeader(in_stream_state, preprocessing_state, byte);
            when context_info_mode =>
                return handleStreamStateContextInfoMode(in_stream_state, preprocessing_state, byte);
            when trace_info_plctl_mode =>
                return handleStreamStateTraceInfoPlctlMode(in_stream_state, preprocessing_state, byte);
            when others =>
                -- do nothing
        end case;
        return out_stream_state;
    end;


    -- 3. Linked to Action Preprocessing
    -- \___________________________________

    type TraceStateAction is (
        nop,
        shift_address,

        trace_on,

        update_address_7_1_short,
        update_address_8_2_short,
        update_address_15_8_short,
        update_address_15_9_short,
        update_address_16_9_short,
        update_address_23_16_short,
        update_address_31_24_short,

        update_address_7_1_long,
        update_address_8_2_long,
        update_address_15_8_long,
        update_address_15_9_long,
        update_address_23_16_long,
        update_address_31_24_long,
        update_address_39_32_long,
        update_address_47_40_long,
        update_address_55_48_long,
        update_address_63_56_long,

        update_timestamp_6_0,
        update_timestamp_13_7,
        update_timestamp_20_14,
        update_timestamp_27_21,
        update_timestamp_34_28,
        update_timestamp_41_35,
        update_timestamp_48_42,
        update_timestamp_55_49,
        update_timestamp_63_56,

        update_contextid_7_0,
        update_contextid_15_8,
        update_contextid_23_16,
        update_contextid_31_24,

        update_vmid_0,

        exact_match_address_0,
        exact_match_address_1,
        exact_match_address_2,

        scan_1_atom_element,
        scan_2_atom_elements,
        scan_3_atom_elements,
        scan_4_atom_elements,
        scan_5_atom_elements,
        scan_6_atom_elements,

        update_exception_type_4_0,

        trace_event -- Undefined
    );

    -------------------
    -- Context IDs
    -------------------

    pure function get_update_contextid_7_0(cid : std_logic_vector(31 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(31 downto 0);
    begin
        res(7 downto 0)  := byte;
        res(31 downto 8) := cid(31 downto 8);
        return res;
    end;

    pure function get_update_contextid_15_8(cid : std_logic_vector(31 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
    variable res : std_logic_vector(31 downto 0);
    begin
        res(7 downto 0)   := cid(7 downto 0);
        res(15 downto 8)  := byte;
        res(31 downto 16) := cid(31 downto 16);
        return res;
    end;

    pure function get_update_contextid_23_16(cid : std_logic_vector(31 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(31 downto 0);
    begin
        res(23 downto 16) := byte;
        res(31 downto 24) := cid(31 downto 24);
        res(15 downto 0)  := cid(15 downto 0);
        return res;
    end;

    pure function get_update_contextid_31_24(cid : std_logic_vector(31 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(31 downto 0);
    begin
        res(31 downto 24) := byte;
        res(23 downto 0)  := cid(23 downto 0);
        return res;
    end;

    -------------------
    -- Timestamps
    -------------------

    pure function get_update_timestamp_6_0(ts : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(6 downto 0)  := byte(6 downto 0);
        res(63 downto 7) := ts(63 downto 7);
        return res;
    end;

    pure function get_update_timestamp_13_7(ts : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(13 downto 7)  := byte(6 downto 0);
        res(6 downto 0)   := ts(6 downto 0);
        res(63 downto 14) := ts(63 downto 14);
        return res;
    end;

    pure function get_update_timestamp_20_14(ts : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 21) := ts(63 downto 21);
        res(20 downto 14) := byte(6 downto 0);
        res(13 downto 0)  := ts(13 downto 0);
        return res;
    end;

    pure function get_update_timestamp_27_21(ts : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 28) := ts(63 downto 28);
        res(27 downto 21) := byte(6 downto 0);
        res(20 downto 0)  := ts(20 downto 0);
        return res;
    end;

    pure function get_update_timestamp_34_28(ts : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 35) := ts(63 downto 35);
        res(34 downto 28) := byte(6 downto 0);
        res(27 downto 0)  := ts(27 downto 0);
        return res;
    end;

    pure function get_update_timestamp_41_35(ts : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 42) := ts(63 downto 42);
        res(41 downto 35) := byte(6 downto 0);
        res(34 downto 0)  := ts(34 downto 0);
        return res;
    end;

    pure function get_update_timestamp_48_42(ts : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 49) := ts(63 downto 49);
        res(48 downto 42) := byte(6 downto 0);
        res(41 downto 0)  := ts(41 downto 0);
        return res;
    end;

    pure function get_update_timestamp_55_49(ts : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 56) := ts(63 downto 56);
        res(55 downto 49) := byte(6 downto 0);
        res(48 downto 0)  := ts(48 downto 0);
        return res;
    end;

    pure function get_update_timestamp_63_56(ts : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 56) := byte;
        res(55 downto 0)  := ts(55 downto 0);
        return res;
    end;

    -------------------
    -- Short address
    -------------------

    pure function get_update_address_7_1_short(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 8) := addr(63 downto 8);
        res(7 downto 1)  := byte(6 downto 0);
        res(0)           := addr(0);
        return res;
    end;

    pure function get_update_address_8_2_short(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 9) := addr(63 downto 9);
        res(8 downto 2)  := byte(6 downto 0);
        res(1 downto 0)  := addr(1 downto 0);
        return res;
    end;

    pure function get_update_address_15_8_short(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 16) := addr(63 downto 16);
        res(15 downto 8)  := byte;
        res(7 downto 0)   := addr(7 downto 0);
        return res;
    end;

    pure function get_update_address_15_9_short(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 16) := addr(63 downto 16);
        res(15 downto 9)  := byte(6 downto 0);
        res(8 downto 0)   := addr(8 downto 0);
        return res;
    end;

    pure function get_update_address_16_9_short(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 17) := addr(63 downto 17);
        res(16 downto 9)  := byte;
        res(8 downto 0)   := addr(8 downto 0);
        return res;
    end;

    pure function get_update_address_23_16_short(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 24) := addr(63 downto 24);
        res(23 downto 16) := byte;
        res(15 downto 0)  := addr(15 downto 0);
        return res;
    end;

    pure function get_update_address_31_24_short(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 32) := addr(63 downto 32);
        res(31 downto 24) := byte;
        res(23 downto 0)  := addr(23 downto 0);
        return res;
    end;

    -------------------
    -- Long Address
    -------------------

    pure function get_update_address_7_1_long(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
    variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 8) := (others => '0');
        res(7 downto 1)  := byte(6 downto 0);
        res(0)           := '0';
        return res;
    end;

    pure function get_update_address_8_2_long(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
    variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 9) := (others => '0');
        res(8 downto 2)  := byte(6 downto 0);
        res(1 downto 0)  := (others => '0');
        return res;
    end;

    pure function get_update_address_15_8_long(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
    variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 16) := (others => '0');
        res(15 downto 8)  := byte;
        res(7 downto 0)   := addr(7 downto 0);
        return res;
    end;

    pure function get_update_address_15_9_long(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
    variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 16) := (others => '0');
        res(15 downto 9)  := byte(6 downto 0);
        res(8 downto 0)   := addr(8 downto 0);
        return res;
    end;

    pure function get_update_address_23_16_long(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
    variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 24) := (others => '0');
        res(23 downto 16) := byte;
        res(15 downto 0)  := addr(15 downto 0);
        return res;
    end;

    pure function get_update_address_31_24_long(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
    variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 32) := (others => '0');
        res(31 downto 24) := byte;
        res(23 downto 0)  := addr(23 downto 0);
        return res;
    end;

    pure function get_update_address_39_32_long(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
    variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 40) := (others => '0');
        res(39 downto 32) := byte;
        res(31 downto 0)  := addr(31 downto 0);
        return res;
    end;

    pure function get_update_address_47_40_long(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
    variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 48) := (others => '0');
        res(47 downto 40) := byte;
        res(39 downto 0)  := addr(39 downto 0);
        return res;
    end;

    pure function get_update_address_55_48_long(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
    variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 56) := (others => '0');
        res(55 downto 48) := byte;
        res(47 downto 0)  := addr(47 downto 0);
        return res;
    end;

    pure function get_update_address_63_56_long(addr : std_logic_vector(63 downto 0); byte : std_logic_vector(7 downto 0)) return std_logic_vector is
    variable res : std_logic_vector(63 downto 0);
    begin
        res(63 downto 56) := byte;
        res(55 downto 0)  := addr(55 downto 0);
        return res;
    end;


    -- 3. From the preprocessing state, the byte itself, return a trace state action depending
    --    on where we are currently in the payload
    pure function createActionPlan(stream_state : StreamState; preprocessing_state : PreprocessingState; byte : std_logic_vector(7 downto 0) ) return TraceStateAction is
    begin
        -- Check if we are expecting a header and match with the current byte. In the case
        -- of address headers, handle the internal address registers updates.
        case stream_state.stream_mode is
            when next_byte_must_be_header =>
                case preprocessing_state.header is
                    -- In case of an exact match, the address registers become:
                    --       t-1     t0-match0    t0-match1    t0-match2
                    -- A0   addr0      addr0        addr1        addr2
                    -- A1   addr1      addr0        addr0        addr0
                    -- A2   addr2      addr1        addr1        addr1
                    when ExactMatchAddress =>
                        if byte(1 downto 0) = "00" then
                            return exact_match_address_0;
                        elsif byte(1 downto 0) = "01" then
                            return exact_match_address_1;
                        elsif byte(1 downto 0) = "10" then
                            return exact_match_address_2;
                        else
                            return nop;
                        end if;

                    -- In case of other address packets, the address registers become:
                    --       t-1        t0
                    -- A0   addr0     new addr (in payload)
                    -- A1   addr1      addr0
                    -- A2   addr2      addr1
                    when ShortAddress_IS0|ShortAddress_IS1|LongAddress_32bit_IS0|LongAddress_32bit_IS1|
                        LongAddress_64bit_IS0|LongAddress_64bit_IS1|AddressWithContext_32bit_IS0|AddressWithContext_32bit_IS1|
                        AddressWithContext_64bit_IS0|AddressWithContext_64bit_IS1 => return shift_address;

                    when AtomF1 =>
                        return scan_1_atom_element;

                    when AtomF2 =>
                        return scan_2_atom_elements;

                    when AtomF3 =>
                        return scan_3_atom_elements;

                    when AtomF4 =>
                        return scan_4_atom_elements;

                    when AtomF5 =>
                        return scan_5_atom_elements;

                    when AtomF6 =>
                        return scan_6_atom_elements;

                    when TraceOn =>
                        return trace_on;
                    when TraceEvent => return trace_event;
                    when others => return nop;
                end case;
            when others =>
                -- continue;
        end case;

        -- If we are not expecting a header, we have a payload to handle, create the plan based on the
        -- current payload index
        case stream_state.header_t is
            when ShortAddress_IS0 =>
                case stream_state.payload_index is
                    when 0 => return update_address_8_2_short;
                    when others => return update_address_16_9_short;
                end case;

            when ShortAddress_IS1 =>
                case stream_state.payload_index is
                    when 0 => return update_address_7_1_short;
                    when others => return update_address_15_8_short;
                end case;

            when LongAddress_32bit_IS0|AddressWithContext_32bit_IS0 =>
                case stream_state.payload_index is
                    when 0 => return update_address_8_2_short;
                    when 1 => return update_address_15_9_short;
                    when 2 => return update_address_23_16_short;
                    when others => return update_address_31_24_short;
                end case;

            when LongAddress_32bit_IS1|AddressWithContext_32bit_IS1 =>
                case stream_state.payload_index is
                    when 0 => return update_address_7_1_short;
                    when 1 => return update_address_15_8_short;
                    when 2 => return update_address_23_16_short;
                    when others => return update_address_31_24_short;
                end case;

            when LongAddress_64bit_IS0|AddressWithContext_64bit_IS0 =>
                case stream_state.payload_index is
                    when 0 => return update_address_8_2_long;
                    when 1 => return update_address_15_9_long;
                    when 2 => return update_address_23_16_long;
                    when 3 => return update_address_31_24_long;
                    when 4 => return update_address_39_32_long;
                    when 5 => return update_address_47_40_long;
                    when 6 => return update_address_55_48_long;
                    when others => return update_address_63_56_long;
                end case;

            when LongAddress_64bit_IS1|AddressWithContext_64bit_IS1 =>
                case stream_state.payload_index is
                    when 0 => return update_address_7_1_long;
                    when 1 => return update_address_15_8_long;
                    when 2 => return update_address_23_16_long;
                    when 3 => return update_address_31_24_long;
                    when 4 => return update_address_39_32_long;
                    when 5 => return update_address_47_40_long;
                    when 6 => return update_address_55_48_long;
                    when others => return update_address_63_56_long;
                end case;
            when TimestampF1|TimestampF2 =>
                case stream_state.payload_index is
                    when 0 => return update_timestamp_6_0;
                    when 1 => return update_timestamp_13_7;
                    when 2 => return update_timestamp_20_14;
                    when 3 => return update_timestamp_27_21;
                    when 4 => return update_timestamp_34_28;
                    when 5 => return update_timestamp_41_35;
                    when 6 => return update_timestamp_48_42;
                    when 7 => return update_timestamp_55_49;
                    when others => return update_timestamp_63_56;
                end case;
            when ContextVMID => return update_vmid_0;
            when ContextContextID =>
                case stream_state.payload_index is
                    when 0 => return update_contextid_7_0;
                    when 1 => return update_contextid_15_8;
                    when 2 => return update_contextid_23_16;
                    when others => return update_contextid_31_24;
                end case;
            when Exception =>
                case stream_state.payload_index is
                    when 0 => return update_exception_type_4_0;
                    -- Discard the other exception types as not supported on Cortex-As
                    when others => return nop;
                end case;
            when others =>
                return nop;
                -- continue
        end case;
    end;


    -- 4. Linked to Trace State Processing
    -- \_______________________________________

    -- 4. Trace State Processing
    --   Executes up to four actions sequentially in the same cycle, based on the planning done
    --   in the previous step.
    --
    -- MW: maximal load on packet stream is 32 bits/cycle if all trace data is from one ETM.
    pure function handleActionPlan(in_trace_state : TraceState; action : TraceStateAction; byte : std_logic_vector(7 downto 0)) return TraceState is
    variable out_trace_state : TraceState;
    -- Atom related
    variable atom_f5_code : std_logic_vector(2 downto 0);
    variable atom_size : integer;
    variable atom_elements : std_logic_vector(23 downto 0);
    begin
        -- Start by mapping the input trace state to the output THEN applying changes:
        out_trace_state := in_trace_state;
        -- Reset the valid signals
        out_trace_state.output_state.atom_valid      := '0';
        out_trace_state.output_state.exception_valid := '0';


        case action is
            when shift_address =>
                out_trace_state.address_state.reg0 := in_trace_state.address_state.reg0; -- keep register here, will be written to
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg0;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg1;

            when exact_match_address_0 =>
                out_trace_state.address_state.reg0 := in_trace_state.address_state.reg0;
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg0;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg1;

            when exact_match_address_1 =>
                out_trace_state.address_state.reg0 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg0;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg1;

            when exact_match_address_2 =>
                out_trace_state.address_state.reg0 := in_trace_state.address_state.reg2;
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg0;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg1;

            when update_address_7_1_short =>
                out_trace_state.address_state.reg0 := get_update_address_7_1_short(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_8_2_short =>
                out_trace_state.address_state.reg0 := get_update_address_8_2_short(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_15_8_short =>
                out_trace_state.address_state.reg0 := get_update_address_15_8_short(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_15_9_short =>
                out_trace_state.address_state.reg0 := get_update_address_15_9_short(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_16_9_short =>
                out_trace_state.address_state.reg0 := get_update_address_16_9_short(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_23_16_short =>
                out_trace_state.address_state.reg0 := get_update_address_23_16_short(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_31_24_short =>
                out_trace_state.address_state.reg0 := get_update_address_31_24_short(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_7_1_long =>
                out_trace_state.address_state.reg0 := get_update_address_7_1_long(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_8_2_long =>
                out_trace_state.address_state.reg0 := get_update_address_8_2_long(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_15_8_long =>
                out_trace_state.address_state.reg0 := get_update_address_15_8_long(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_15_9_long =>
                out_trace_state.address_state.reg0 := get_update_address_15_9_long(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_23_16_long =>
                out_trace_state.address_state.reg0 := get_update_address_23_16_long(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_31_24_long =>
                out_trace_state.address_state.reg0 := get_update_address_31_24_long(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_39_32_long =>
                out_trace_state.address_state.reg0 := get_update_address_39_32_long(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_47_40_long =>
                out_trace_state.address_state.reg0 := get_update_address_47_40_long(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_55_48_long =>
                out_trace_state.address_state.reg0 := get_update_address_55_48_long(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_address_63_56_long =>
                out_trace_state.address_state.reg0 := get_update_address_63_56_long(in_trace_state.address_state.reg0, byte);
                out_trace_state.address_state.reg1 := in_trace_state.address_state.reg1;
                out_trace_state.address_state.reg2 := in_trace_state.address_state.reg2;

            when update_timestamp_6_0 =>
                out_trace_state.output_state.timestamp := get_update_timestamp_6_0(in_trace_state.output_state.timestamp, byte);

            when update_timestamp_13_7 =>
                out_trace_state.output_state.timestamp := get_update_timestamp_13_7(in_trace_state.output_state.timestamp, byte);

            when update_timestamp_20_14 =>
                out_trace_state.output_state.timestamp := get_update_timestamp_20_14(in_trace_state.output_state.timestamp, byte);

            when update_timestamp_27_21 =>
                out_trace_state.output_state.timestamp := get_update_timestamp_27_21(in_trace_state.output_state.timestamp, byte);

            when update_timestamp_34_28 =>
                out_trace_state.output_state.timestamp := get_update_timestamp_34_28(in_trace_state.output_state.timestamp, byte);

            when update_timestamp_41_35 =>
                out_trace_state.output_state.timestamp := get_update_timestamp_41_35(in_trace_state.output_state.timestamp, byte);

            when update_timestamp_48_42 =>
                out_trace_state.output_state.timestamp := get_update_timestamp_48_42(in_trace_state.output_state.timestamp, byte);

            when update_timestamp_55_49 =>
                out_trace_state.output_state.timestamp := get_update_timestamp_55_49(in_trace_state.output_state.timestamp, byte);

            when update_timestamp_63_56 =>
                out_trace_state.output_state.timestamp := get_update_timestamp_63_56(in_trace_state.output_state.timestamp, byte);

            when update_vmid_0 =>
                out_trace_state.context_state.vmid              := byte;

            when update_contextid_7_0 =>
                out_trace_state.context_state.context_id        := get_update_contextid_7_0(in_trace_state.context_state.context_id, byte);

            when update_contextid_15_8 =>
                out_trace_state.context_state.context_id        := get_update_contextid_15_8(in_trace_state.context_state.context_id, byte);

            when update_contextid_23_16 =>
                out_trace_state.context_state.context_id        := get_update_contextid_23_16(in_trace_state.context_state.context_id, byte);

            when update_contextid_31_24 =>
                out_trace_state.context_state.context_id        := get_update_contextid_31_24(in_trace_state.context_state.context_id, byte);

            when scan_1_atom_element =>
                if in_trace_state.synchronized = '1' then
                    out_trace_state.output_state.atom_elements := (ATOM_ELTS_SIZE - 1 downto 1 => '0') & byte(0);
                    out_trace_state.output_state.atom_nb       := 1;
                    out_trace_state.output_state.atom_valid    := '1';
                end if;

            when scan_2_atom_elements =>
                if in_trace_state.synchronized = '1' then
                    out_trace_state.output_state.atom_elements := (ATOM_ELTS_SIZE - 1 downto 2 => '0') & byte(1 downto 0);
                    out_trace_state.output_state.atom_nb       := 2;
                    out_trace_state.output_state.atom_valid    := '1';
                end if;

            when scan_3_atom_elements =>
                if in_trace_state.synchronized = '1' then
                    out_trace_state.output_state.atom_elements := (ATOM_ELTS_SIZE - 1 downto 3 => '0') & byte(2 downto 0);
                    out_trace_state.output_state.atom_nb       := 3;
                    out_trace_state.output_state.atom_valid    := '1';
                end if;

            when scan_4_atom_elements =>
                if in_trace_state.synchronized = '1' then
                    case byte(1 downto 0) is
                        when "00" =>
                            out_trace_state.output_state.atom_elements := (ATOM_ELTS_SIZE - 1 downto 4 => '0') & "0111";
                            out_trace_state.output_state.atom_nb       := 4;
                            out_trace_state.output_state.atom_valid    := '1';
                        when "01" =>
                            out_trace_state.output_state.atom_elements := (ATOM_ELTS_SIZE - 1 downto 4 => '0') & "0000";
                            out_trace_state.output_state.atom_nb       := 4;
                            out_trace_state.output_state.atom_valid    := '1';
                        when "10" =>
                            out_trace_state.output_state.atom_elements := (ATOM_ELTS_SIZE - 1 downto 4 => '0') & "0101";
                            out_trace_state.output_state.atom_nb       := 4;
                            out_trace_state.output_state.atom_valid    := '1';
                        when "11" =>
                            out_trace_state.output_state.atom_elements := (ATOM_ELTS_SIZE - 1 downto 4 => '0') & "1010";
                            out_trace_state.output_state.atom_nb       := 4;
                            out_trace_state.output_state.atom_valid    := '1';
                        when others =>
                            -- Invalid
                            out_trace_state.output_state.atom_elements := (others => '0');
                            out_trace_state.output_state.atom_nb       := 0;
                            out_trace_state.output_state.atom_valid    := '0';
                    end case;
                end if;

            when scan_5_atom_elements =>
                if in_trace_state.synchronized = '1' then
                    atom_f5_code := (byte(5) & byte(1) & byte(0));
                    case atom_f5_code is
                        when "101" =>
                            out_trace_state.output_state.atom_elements := (ATOM_ELTS_SIZE - 1 downto 5 => '0') & "01111";
                            out_trace_state.output_state.atom_nb       := 5;
                            out_trace_state.output_state.atom_valid    := '1';
                        when "001" =>
                            out_trace_state.output_state.atom_elements := (ATOM_ELTS_SIZE - 1 downto 5 => '0') & "00000";
                            out_trace_state.output_state.atom_nb       := 5;
                            out_trace_state.output_state.atom_valid    := '1';
                        when "010" =>
                            out_trace_state.output_state.atom_elements := (ATOM_ELTS_SIZE - 1 downto 5 => '0') & "01010";
                            out_trace_state.output_state.atom_nb       := 5;
                            out_trace_state.output_state.atom_valid    := '1';
                        when "011" =>
                            out_trace_state.output_state.atom_elements := (ATOM_ELTS_SIZE - 1 downto 5 => '0') & "10101";
                            out_trace_state.output_state.atom_nb       := 5;
                            out_trace_state.output_state.atom_valid    := '1';
                        when others =>
                            out_trace_state.output_state.atom_elements := (others => '0');
                            out_trace_state.output_state.atom_nb       := 0;
                            out_trace_state.output_state.atom_valid    := '0';
                    end case;
                end if;

            when scan_6_atom_elements =>
                if in_trace_state.synchronized = '1' then
                    atom_size := to_integer(unsigned(byte(4 downto 0))) + 3;

                    -- Check for max size, 0b10100 or 20 + 3 (1 comes from the A bit)
                    if atom_size > ATOM_ELTS_SIZE - 1 then
                        atom_size := ATOM_ELTS_SIZE - 1;
                    end if;

                    out_trace_state.output_state.atom_elements := (others => '0');

                    -- Count of Es
                    out_trace_state.output_state.atom_elements(atom_size - 1 downto 0) := (others => '1');
                    -- Inverted A bit, 1->N, 0->E
                    out_trace_state.output_state.atom_elements(atom_size - 1) := not byte(5);

                    out_trace_state.output_state.atom_nb := atom_size;
                    out_trace_state.output_state.atom_valid    := '1';
                end if;

            when update_exception_type_4_0 =>
                out_trace_state.output_state.exception_type  := byte(5 downto 1);
                out_trace_state.output_state.exception_valid := '1';

            when trace_on =>
                out_trace_state.synchronized       := '1';
                out_trace_state.address_state.reg0 := (others => '0');
                out_trace_state.address_state.reg1 := (others => '0');
                out_trace_state.address_state.reg2 := (others => '0');

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


    signal preprocessing_state_stream_state_stage0 : PreprocessingState;
    signal preprocessing_state_stream_state_stage1 : PreprocessingState;
    signal preprocessing_state_stream_state_stage2 : PreprocessingState;
    signal preprocessing_state_stream_state_stage3 : PreprocessingState;

    signal preprocessing_action_plan_stage0 : PreprocessingState;
    signal preprocessing_action_plan_stage1 : PreprocessingState;
    signal preprocessing_action_plan_stage2 : PreprocessingState;
    signal preprocessing_action_plan_stage3 : PreprocessingState;

    signal trace_state0 : TraceState;
    signal trace_state1 : TraceState;
    signal trace_state2 : TraceState;
    signal trace_state3 : TraceState;


    signal stream_state0 : StreamState;
    signal stream_state1 : StreamState;
    signal stream_state2 : StreamState;
    signal stream_state3 : StreamState;

    signal stream_state3_prop : StreamState;

    signal action0 : TraceStateAction;
    signal action1 : TraceStateAction;
    signal action2 : TraceStateAction;
    signal action3 : TraceStateAction;

    -- Latching freeze to clear synchronization on falling edge
    signal latched_freeze_req : std_logic := '0';
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
        else
            -- TODO: Deassert by default?
            data_valid_stream_state_stage <= '0';
            if i_valid = '1' then
               preprocessing_state_stream_state_stage0 <=  preprocess(i_data(7 downto 0));
               preprocessing_state_stream_state_stage1 <=  preprocess(i_data(15 downto 8));
               preprocessing_state_stream_state_stage2 <=  preprocess(i_data(23 downto 16));
               preprocessing_state_stream_state_stage3 <=  preprocess(i_data(31 downto 24));
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
            stream_state3.stream_mode <= extension_mode;
        else
            -- TODO: Deassert by default?
            data_valid_action_plan_stage <= '0';

            if data_valid_stream_state_stage = '1' then
                preprocessing_action_plan_stage0 <= preprocessing_state_stream_state_stage0;
                preprocessing_action_plan_stage1 <= preprocessing_state_stream_state_stage1;
                preprocessing_action_plan_stage2 <= preprocessing_state_stream_state_stage2;
                preprocessing_action_plan_stage3 <= preprocessing_state_stream_state_stage3;

                stream_state3_prop <= stream_state3;

                stream_state0 <= handleStreamState(stream_state3, preprocessing_state_stream_state_stage0, data_stream_state_stage(7 downto 0));

                stream_state1 <=
                    handleStreamState(
                    handleStreamState(stream_state3, preprocessing_state_stream_state_stage0, data_stream_state_stage(7 downto 0)),
                    preprocessing_state_stream_state_stage1, data_stream_state_stage(15 downto 8));

                stream_state2 <=
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(stream_state3, preprocessing_state_stream_state_stage0, data_stream_state_stage(7 downto 0)),
                    preprocessing_state_stream_state_stage1, data_stream_state_stage(15 downto 8)),
                    preprocessing_state_stream_state_stage2, data_stream_state_stage(23 downto 16));

                stream_state3 <=
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(
                    handleStreamState(stream_state3, preprocessing_state_stream_state_stage0, data_stream_state_stage(7 downto 0)),
                    preprocessing_state_stream_state_stage1, data_stream_state_stage(15 downto 8)),
                    preprocessing_state_stream_state_stage2, data_stream_state_stage(23 downto 16)),
                    preprocessing_state_stream_state_stage3, data_stream_state_stage(31 downto 24));

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

                action0 <= createActionPlan(stream_state3_prop, preprocessing_action_plan_stage0, data_action_plan_stage(7 downto 0));
                action1 <= createActionPlan(stream_state0, preprocessing_action_plan_stage1, data_action_plan_stage(15 downto 8));
                action2 <= createActionPlan(stream_state1, preprocessing_action_plan_stage2, data_action_plan_stage(23 downto 16));
                action3 <= createActionPlan(stream_state2, preprocessing_action_plan_stage3, data_action_plan_stage(31 downto 24));

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
            trace_state3.synchronized <= '0';
        else
            -- TODO: Deassert by default?
            data_valid_output_stage <= '0';

            -- Latch freeze request
            latched_freeze_req <= i_freeze_request;

            -- Rising edge of freeze = end of iteration
            if latched_freeze_req = '0' and i_freeze_request = '1' then
                trace_state3.synchronized          <= '0';
                trace_state3.address_state.reg0    <= (others => '0');
                trace_state3.address_state.reg1    <= (others => '0');
                trace_state3.address_state.reg2    <= (others => '0');
            elsif data_valid_action_stage = '1' then
                trace_state0 <= handleActionPlan(trace_state3, action0, data_action_stage(7 downto 0));

                trace_state1 <= handleActionPlan(
                                handleActionPlan(trace_state3, action0, data_action_stage(7 downto 0)),
                                                action1, data_action_stage(15 downto 8));

                trace_state2 <= handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(trace_state3, action0, data_action_stage(7 downto 0)),
                                                 action1, data_action_stage(15 downto 8)),
                                                 action2, data_action_stage(23 downto 16));

                trace_state3 <= handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(
                                handleActionPlan(trace_state3, action0, data_action_stage(7 downto 0)),
                                                 action1, data_action_stage(15 downto 8)),
                                                 action2, data_action_stage(23 downto 16)),
                                                 action3, data_action_stage(31 downto 24));

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
            o_atom_valid0 <= '0';
            o_atom_valid1 <= '0';
            o_atom_valid2 <= '0';
            o_atom_valid3 <= '0';
            o_exception_valid0 <= '0';
            o_exception_valid1 <= '0';
            o_exception_valid2 <= '0';
            o_exception_valid3 <= '0';
        else
            -- TODO: Deassert by default?
            o_atom_valid0 <= '0';
            o_atom_valid1 <= '0';
            o_atom_valid2 <= '0';
            o_atom_valid3 <= '0';
            o_exception_valid0 <= '0';
            o_exception_valid1 <= '0';
            o_exception_valid2 <= '0';
            o_exception_valid3 <= '0';

            if data_valid_output_stage = '1' then
                o_address_reg_0_0  <= trace_state0.address_state.reg0;
                o_ts0              <= trace_state0.output_state.timestamp;
                o_vmid0            <= trace_state0.context_state.vmid;
                o_ctxt_id0         <= trace_state0.context_state.context_id;
                o_atom_elements0   <= trace_state0.output_state.atom_elements;
                o_atom_nb0         <= to_unsigned(trace_state0.output_state.atom_nb, ATOM_NB_SIZE);
                o_exception_type0  <= trace_state0.output_state.exception_type;
                o_atom_valid0      <= trace_state0.output_state.atom_valid;
                o_exception_valid0 <= trace_state0.output_state.exception_valid;

                o_address_reg_0_1  <= trace_state1.address_state.reg0;
                o_ts1              <= trace_state1.output_state.timestamp;
                o_vmid1            <= trace_state1.context_state.vmid;
                o_ctxt_id1         <= trace_state1.context_state.context_id;
                o_atom_elements1   <= trace_state1.output_state.atom_elements;
                o_atom_nb1         <= to_unsigned(trace_state1.output_state.atom_nb, ATOM_NB_SIZE);
                o_exception_type1  <= trace_state1.output_state.exception_type;
                o_atom_valid1      <= trace_state1.output_state.atom_valid;
                o_exception_valid1 <= trace_state1.output_state.exception_valid;

                o_address_reg_0_2  <= trace_state2.address_state.reg0;
                o_ts2              <= trace_state2.output_state.timestamp;
                o_vmid2            <= trace_state2.context_state.vmid;
                o_ctxt_id2         <= trace_state2.context_state.context_id;
                o_atom_elements2   <= trace_state2.output_state.atom_elements;
                o_atom_nb2         <= to_unsigned(trace_state2.output_state.atom_nb, ATOM_NB_SIZE);
                o_exception_type2  <= trace_state2.output_state.exception_type;
                o_atom_valid2      <= trace_state2.output_state.atom_valid;
                o_exception_valid2 <= trace_state2.output_state.exception_valid;

                o_address_reg_0_3  <= trace_state3.address_state.reg0;
                o_ts3              <= trace_state3.output_state.timestamp;
                o_vmid3            <= trace_state3.context_state.vmid;
                o_ctxt_id3         <= trace_state3.context_state.context_id;
                o_atom_elements3   <= trace_state3.output_state.atom_elements;
                o_atom_nb3         <= to_unsigned(trace_state3.output_state.atom_nb, ATOM_NB_SIZE);
                o_exception_type3  <= trace_state3.output_state.exception_type;
                o_atom_valid3      <= trace_state3.output_state.atom_valid;
                o_exception_valid3 <= trace_state3.output_state.exception_valid;
            end if;
        end if;
     end if;
end process;
end Behavioral;
