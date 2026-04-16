library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

entity trace_replay_tb is
end trace_replay_tb;

architecture Behavioral of trace_replay_tb is

    constant CLK_PERIOD : time := 10 ns;

    -- Clock / reset
    signal aclk    : std_logic := '0';
    signal aresetn : std_logic := '0';

    -- frame_generator ? frame_deformatter
    signal tpiu_data       : std_logic_vector(31 downto 0) := (others => '0');
    signal tpiu_valid      : std_logic := '0';

    signal frame_data      : std_logic_vector(127 downto 0);
    signal frame_valid     : std_logic;
    signal frame_error     : std_logic;

    -- frame_deformatter ? byte_stream_demux
    signal deform_data     : std_logic_vector(31 downto 0);
    signal deform_ids      : std_logic_vector(27 downto 0);
    signal deform_keep     : std_logic_vector(3 downto 0);
    signal deform_valid    : std_logic;
    signal deform_error    : std_logic;

    -- byte_stream_demux outputs ? one keep per ETM/STM
    signal demux_data      : std_logic_vector(31 downto 0);
    signal demux_keep0     : std_logic_vector(3 downto 0);
    signal demux_keep1     : std_logic_vector(3 downto 0);
    signal demux_keep2     : std_logic_vector(3 downto 0);
    signal demux_keep3     : std_logic_vector(3 downto 0);
    signal demux_keeps     : std_logic_vector(3 downto 0);
    signal demux_id_error  : std_logic;

    -- byte_stream_compactor (one per ETM, wire up the one you care about ? ETM0 here)
    signal compact_data    : std_logic_vector(31 downto 0);
    signal compact_valid   : std_logic;

    -- etm_decoder outputs (ETM0)
    signal atom_valid0     : std_logic;
    signal exc_valid0      : std_logic;
    signal ts0             : std_logic_vector(63 downto 0);
    signal addr0           : std_logic_vector(63 downto 0);
    signal ctxt0           : std_logic_vector(31 downto 0);
    signal vmid0           : std_logic_vector(7 downto 0);
    signal atom_elts0      : std_logic_vector(23 downto 0);
    signal atom_nb0        : unsigned(4 downto 0);
    signal exc_type0       : std_logic_vector(4 downto 0);
    signal exc_pending0    : std_logic;
    signal pre_exc_addr0   : std_logic_vector(63 downto 0);

    signal atom_valid1     : std_logic;
    signal exc_valid1      : std_logic;
    signal ts1             : std_logic_vector(63 downto 0);
    signal addr1           : std_logic_vector(63 downto 0);
    signal ctxt1           : std_logic_vector(31 downto 0);
    signal vmid1           : std_logic_vector(7 downto 0);
    signal atom_elts1      : std_logic_vector(23 downto 0);
    signal atom_nb1        : unsigned(4 downto 0);
    signal exc_type1       : std_logic_vector(4 downto 0);
    signal exc_pending1    : std_logic;
    signal pre_exc_addr1   : std_logic_vector(63 downto 0);

    signal atom_valid2     : std_logic;
    signal exc_valid2      : std_logic;
    signal ts2             : std_logic_vector(63 downto 0);
    signal addr2           : std_logic_vector(63 downto 0);
    signal ctxt2           : std_logic_vector(31 downto 0);
    signal vmid2           : std_logic_vector(7 downto 0);
    signal atom_elts2      : std_logic_vector(23 downto 0);
    signal atom_nb2        : unsigned(4 downto 0);
    signal exc_type2       : std_logic_vector(4 downto 0);
    signal exc_pending2    : std_logic;
    signal pre_exc_addr2   : std_logic_vector(63 downto 0);

    signal atom_valid3     : std_logic;
    signal exc_valid3      : std_logic;
    signal ts3             : std_logic_vector(63 downto 0);
    signal addr3           : std_logic_vector(63 downto 0);
    signal ctxt3           : std_logic_vector(31 downto 0);
    signal vmid3           : std_logic_vector(7 downto 0);
    signal atom_elts3      : std_logic_vector(23 downto 0);
    signal atom_nb3        : unsigned(4 downto 0);
    signal exc_type3       : std_logic_vector(4 downto 0);
    signal exc_pending3    : std_logic;
    signal pre_exc_addr3   : std_logic_vector(63 downto 0);

    -- edge_extractor
    signal edge_index      : std_logic_vector(63 downto 0);
    signal edge_valid      : std_logic;
    signal edge_empty      : std_logic;
    signal edge_ready      : std_logic := '1'; -- always drain

    -- Tie-off unused AXI4-Lite ports
    signal axi_awaddr  : std_logic_vector(7 downto 0)  := (others => '0');
    signal axi_awvalid : std_logic := '0';
    signal axi_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal axi_wvalid  : std_logic := '0';
    signal axi_bready  : std_logic := '1';
    signal axi_araddr  : std_logic_vector(7 downto 0)  := (others => '0');
    signal axi_arvalid : std_logic := '0';
    signal axi_rready  : std_logic := '1';

    -- Main sim signal for the observer
    signal sim_done : boolean := false;
begin

    aclk <= not aclk after CLK_PERIOD / 2;

    -- ----------------------------------------------------------------
    -- DUT chain
    -- ----------------------------------------------------------------

    u_frame_gen : entity work.frame_generator
        port map (
            aclk          => aclk,
            aresetn       => aresetn,
            i_soft_reset  => '0',
            i_data        => tpiu_data,
            o_frame       => frame_data,
            o_valid_frame => frame_valid,
            o_frame_error => frame_error
        );

    u_deformatter : entity work.frame_deformatter
        port map (
            aclk                   => aclk,
            aresetn                => aresetn,
            i_soft_reset           => '0',
            i_frame                => frame_data,
            i_valid_frame          => frame_valid,
            o_data                 => deform_data,
            o_ids                  => deform_ids,
            o_keep                 => deform_keep,
            o_valid                => deform_valid,
            o_bytestream_gen_error => deform_error
        );

    u_demux : entity work.byte_stream_demux
        port map (
            aclk                        => aclk,
            aresetn                     => aresetn,
            i_soft_reset                => '0',
            i_data                      => deform_data,
            i_ids                       => deform_ids,
            i_keep                      => deform_keep,
            i_valid                     => deform_valid,
            o_data                      => demux_data,
            o_keep0                     => demux_keep0,
            o_keep1                     => demux_keep1,
            o_keep2                     => demux_keep2,
            o_keep3                     => demux_keep3,
            o_keeps                     => demux_keeps,
            o_bytestream_demux_id_error => demux_id_error
        );

    -- Only instantiate compactor + decoder for ETM0 for now;
    -- duplicate with demux_keep1/2/3 as needed.
    u_compactor0 : entity work.byte_stream_compactor
        port map (
            aclk         => aclk,
            aresetn      => aresetn,
            i_soft_reset => '0',
            i_data       => demux_data,
            i_keep       => demux_keep0,
            o_data       => compact_data,
            o_valid      => compact_valid
        );

    u_etm_decoder : entity work.etm_decoder
        port map (
            aclk                  => aclk,
            aresetn               => aresetn,
            i_soft_reset          => '0',
            i_data                => compact_data,
            i_valid               => compact_valid,

            o_atom_valid0         => atom_valid0,
            o_exception_valid0    => exc_valid0,
            o_ts0                 => ts0,
            o_address_reg_0_0     => addr0,
            o_ctxt_id0            => ctxt0,
            o_vmid0               => vmid0,
            o_atom_elements0      => atom_elts0,
            o_atom_nb0            => atom_nb0,
            o_exception_type0     => exc_type0,
            o_exception_pending0  => exc_pending0,
            o_pre_exception_addr0 => pre_exc_addr0,

            o_atom_valid1         => atom_valid1,
            o_exception_valid1    => exc_valid1,
            o_ts1                 => ts1,
            o_address_reg_0_1     => addr1,
            o_ctxt_id1            => ctxt1,
            o_vmid1               => vmid1,
            o_atom_elements1      => atom_elts1,
            o_atom_nb1            => atom_nb1,
            o_exception_type1     => exc_type1,
            o_exception_pending1  => exc_pending1,
            o_pre_exception_addr1 => pre_exc_addr1,

            o_atom_valid2         => atom_valid2,
            o_exception_valid2    => exc_valid2,
            o_ts2                 => ts2,
            o_address_reg_0_2     => addr2,
            o_ctxt_id2            => ctxt2,
            o_vmid2               => vmid2,
            o_atom_elements2      => atom_elts2,
            o_atom_nb2            => atom_nb2,
            o_exception_type2     => exc_type2,
            o_exception_pending2  => exc_pending2,
            o_pre_exception_addr2 => pre_exc_addr2,

            o_atom_valid3         => atom_valid3,
            o_exception_valid3    => exc_valid3,
            o_ts3                 => ts3,
            o_address_reg_0_3     => addr3,
            o_ctxt_id3            => ctxt3,
            o_vmid3               => vmid3,
            o_atom_elements3      => atom_elts3,
            o_atom_nb3            => atom_nb3,
            o_exception_type3     => exc_type3,
            o_exception_pending3  => exc_pending3,
            o_pre_exception_addr3 => pre_exc_addr3

        );

    u_edge_extractor : entity work.edge_extractor
        port map (
            aclk              => aclk,
            aresetn           => aresetn,

            i_atom_valid0         => atom_valid0,
            i_address_reg_0_0     => addr0,
            i_atom_elements0      => atom_elts0,
            i_atom_nb0            => atom_nb0,
            i_exception_valid0    => exc_valid0,
            i_exception_type0     => exc_type0,
            i_exception_pending0  => exc_pending0,
            i_pre_exception_addr0 => pre_exc_addr0,

            i_atom_valid1         => atom_valid1,
            i_address_reg_0_1     => addr1,
            i_atom_elements1      => atom_elts1,
            i_atom_nb1            => atom_nb1,
            i_exception_valid1    => exc_valid1,
            i_exception_type1     => exc_type1,
            i_exception_pending1  => exc_pending1,
            i_pre_exception_addr1 => pre_exc_addr1,

            i_atom_valid2         => atom_valid2,
            i_address_reg_0_2     => addr2,
            i_atom_elements2      => atom_elts2,
            i_atom_nb2            => atom_nb2,
            i_exception_valid2    => exc_valid2,
            i_exception_type2     => exc_type2,
            i_exception_pending2  => exc_pending2,
            i_pre_exception_addr2 => pre_exc_addr2,

            i_atom_valid3         => atom_valid3,
            i_address_reg_0_3     => addr3,
            i_atom_elements3      => atom_elts3,
            i_atom_nb3            => atom_nb3,
            i_exception_valid3    => exc_valid3,
            i_exception_type3     => exc_type3,
            i_exception_pending3  => exc_pending3,
            i_pre_exception_addr3 => pre_exc_addr3,

            i_freeze_request  => '0',

            o_index           => edge_index,
            o_valid           => edge_valid,
            o_empty           => edge_empty,
            i_ready           => edge_ready,

            s_axi_awaddr      => axi_awaddr,
            s_axi_awvalid     => axi_awvalid,
            s_axi_awready     => open,
            s_axi_wdata       => axi_wdata,
            s_axi_wvalid      => axi_wvalid,
            s_axi_wready      => open,
            s_axi_bresp       => open,
            s_axi_bvalid      => open,
            s_axi_bready      => axi_bready,
            s_axi_araddr      => axi_araddr,
            s_axi_arvalid     => axi_arvalid,
            s_axi_arready     => open,
            s_axi_rdata       => open,
            s_axi_rresp       => open,
            s_axi_rvalid      => open,
            s_axi_rready      => axi_rready
        );

    -- ----------------------------------------------------------------
    -- Stimulus: read binary, feed one 32-bit word per cycle
    -- ----------------------------------------------------------------
    stimulus : process
        -- Each line in the hex file is one 32-bit word in hex,
        -- produced by: xxd -p cstrace.bin | tr -d '\n' | fold -w 8 > capture.hex
        file     trace_file  : text open read_mode is "readelf.hex";
        variable line_buf    : line;
        variable word_hex    : string(1 to 8);
        variable word_val    : std_logic_vector(31 downto 0);
        variable nibble      : std_logic_vector(3 downto 0);

        -- Convert a single hex character to 4 bits
        function hex_char_to_slv(c : character) return std_logic_vector is
        begin
            case c is
                when '0' => return "0000";
                when '1' => return "0001";
                when '2' => return "0010";
                when '3' => return "0011";
                when '4' => return "0100";
                when '5' => return "0101";
                when '6' => return "0110";
                when '7' => return "0111";
                when '8' => return "1000";
                when '9' => return "1001";
                when 'a'|'A' => return "1010";
                when 'b'|'B' => return "1011";
                when 'c'|'C' => return "1100";
                when 'd'|'D' => return "1101";
                when 'e'|'E' => return "1110";
                when 'f'|'F' => return "1111";
                when others  => return "0000";
            end case;
        end function;

    begin
        -- Hold reset for a few cycles
        aresetn   <= '0';
        tpiu_data <= X"7FFFFFFF";
        wait for CLK_PERIOD * 5;
        wait until rising_edge(aclk);
        aresetn <= '1';
        wait until rising_edge(aclk);

        -- Feed words from the hex file
        while not endfile(trace_file) loop
            readline(trace_file, line_buf);
            read(line_buf, word_hex);

            -- Parse 8 hex characters into 32 bits
            for i in 0 to 7 loop
                word_val(31 - i*4 downto 28 - i*4) := hex_char_to_slv(word_hex(i+1));
            end loop;

            tpiu_data <= word_val;
            wait until rising_edge(aclk);
        end loop;

        -- Flush pipeline (enough cycles to drain all stages)
        tpiu_data <= X"7FFFFFFF";  -- back to sync after done
        wait for CLK_PERIOD * 64;

        report "Replay complete" severity note;
        sim_done <= true;
        std.env.finish;
    end process;

    -- ----------------------------------------------------------------
    -- Observer: log edges and key pipeline signals to file
    -- ----------------------------------------------------------------
    observer : process
        file     out_file   : text open write_mode is "edges_out.txt";
        variable out_line   : line;
        variable cycle_cnt  : integer := 0;

        function to_hex_string(slv : std_logic_vector) return string is
            constant hex_chars : string(1 to 16) := "0123456789abcdef";
            variable result    : string(1 to slv'length / 4);
            variable nibble    : integer;
        begin
            for i in result'range loop
                nibble := to_integer(unsigned(
                    slv(slv'length - (i-1)*4 - 1 downto slv'length - i*4)));
                result(i) := hex_chars(nibble + 1);
            end loop;
            return result;
        end function;
    begin
        wait until aresetn = '1';
        loop
            wait until rising_edge(aclk);

            cycle_cnt := cycle_cnt + 1;

            if edge_valid = '1' and edge_ready = '1' then
                write(out_line, string'("EDGE cycle="));
                write(out_line, cycle_cnt);
                write(out_line, string'(" index=0x"));
                write(out_line, to_hex_string(edge_index));
                writeline(out_file, out_line);
            end if;

            if frame_error = '1' then
                write(out_line, string'("FRAME_ERROR cycle="));
                write(out_line, cycle_cnt);
                writeline(out_file, out_line);
            end if;

            if deform_error = '1' then
                write(out_line, string'("DEFORM_ERROR cycle="));
                write(out_line, cycle_cnt);
                writeline(out_file, out_line);
            end if;

            if demux_id_error = '1' then
                write(out_line, string'("DEMUX_ID_ERROR cycle="));
                write(out_line, cycle_cnt);
                writeline(out_file, out_line);
            end if;
        end loop;
    end process;

end Behavioral;