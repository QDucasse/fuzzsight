library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


package decoder_constants is
    -- ETM-related
    constant VMID_SIZE_BYTE               : integer := 1; -- On ultrascale plus etms for a53 support only 1 byte of VMID.
    constant IA_SIZE_BYTE                 : integer := 8; -- Instruction address size in bytes, for a53 is 8.
    constant CID_SIZE_BYTE                : integer := 4;
    constant DA_SIZE_BYTE                 : integer := 0;
    constant ETM_MAX_BOUNDED_PAYLOAD_SIZE : integer := 9;

    constant ATOM_ELTS_SIZE               : integer := 24; -- One-bit per atom encoding
    constant ATOM_NB_SIZE                 : integer := 5;  -- Actual number (should go up to 24)
    constant EXC_TYPE_SIZE                : integer := 5;

    -- STM-related
    constant STM_MAX_BOUNDED_PAYLOAD_SIZE : integer := 16; -- 64-bit data (in nibbles)
    constant STM_MAX_HEADER_SIZE          : integer := 22; -- Async is 21 Fs then a 0
end package decoder_constants;