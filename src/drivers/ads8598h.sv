// vim: set sw=4 et ts=4 ai ff=unix:
//-------------------------------------------------------------------------------------------------|
//
//  Driver for TI ADS8598H 18-Bit 500kSPS 8-Channel Simultaneous-Sampling ADC
//      https://www.ti.com/product/ADS8598H
//      https://www.ti.com/lit/ds/symlink/ads8598h.pdf
//
//-------------------------------------------------------------------------------------------------|
//
// Copyright 2019 Marsh Ray
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//-------------------------------------------------------------------------------------------------|

`default_nettype none

//-------------------------------------------------------------------------------------------------|

package ADS8598H;

    //---------------------------------------------------------------------------------------------|

    typedef enum logic [2:0] {
        //                          samples/s
        oversamprat_1  = 3'b000, // 500,000
        oversamprat_2  = 3'b001, // 250,000
        oversamprat_4  = 3'b010, // 125,000
        oversamprat_8  = 3'b011, //  62,500
        oversamprat_16 = 3'b100, //  31,250
        oversamprat_32 = 3'b101, //  15,625
        oversamprat_64 = 3'b110  //   7,812.5
    } oversamprat_t;

    //---------------------------------------------------------------------------------------------|

    typedef enum logic {
        ref_source_external = 1'b0,
        ref_source_internal = 1'b1
    } ref_source_t;

    //---------------------------------------------------------------------------------------------|

    typedef enum logic {
        range_5v  = 1'b0,
        range_10v = 1'b1
    } range_t;

    //---------------------------------------------------------------------------------------------|

    // A channel index, 0 through 7.
    localparam time chan_cnt    = 8;
    localparam time chan_ix_max_u64 = chan_cnt - 1;
    localparam time chan_ix_wid = $clog2(chan_ix_max_u64 + 1);

    typedef logic [chan_ix_wid-1 : 0] chan_ix_t;

    localparam chan_ix_t chan_ix_max = chan_ix_t'(chan_ix_max_u64);

    //---------------------------------------------------------------------------------------------|

    localparam time sample_wid = 18;

    // A single sample from one 18-bit channel.
    typedef logic [sample_wid-1 : 0] sample_t;

    // A simultaneous sample from eight 18-bit channels.
    // TODO Find a better name for this concept?
    typedef sample_t octosample_t[0 : chan_ix_max];

    //---------------------------------------------------------------------------------------------|

endpackage : ADS8598H

//-------------------------------------------------------------------------------------------------|
//
//  Drives an ADS8598H in parallel interface mode.
//  The '/PAR SER BYTESEL' pin must be tied low at power-on.
//
module ADS8598H_driver
    import ADS8598H::*;
#(
    parameter real          clk_freq_hz
    ,parameter oversamprat_t oversamprat
    ,parameter ref_source_t  ref_source
    ,parameter range_t       param_range
) (
    input clk,
    input reset_sn,

    // The low bit of the count of conversions initiated by the ADC.
    // A change indicates analog channels have just been latched for
    // a new sampling event. This is intended to allow recording of
    // timing information.
    output logic conv_cnt_m2,

    // Sample data out.
    // octosample_valid goes high with data from the first sample event,
    // then stays high, so is technically redunant with samp_cnt_m2.
    output logic        octosample_valid,
    output octosample_t octosample,

    // The low bit of the count of valid samples presented on output.
    // A change indicates valid data from a new sampling event.
    output logic samp_cnt_m2,

    // Things have gone horribly wrong.
    output logic fail,

    // Probable glitch
    output logic latch_msbs,
    output logic glitch,

    // ADS8598H signals (in the order listed on the datasheet).
    input         busy_asy,
    output        convsta,
    output        convstb,
    output        cs_n,
    input  [15:0] db_asy, // includes overloaded signals
//    input         frstdata,
    output [2:0]  os,
    output        range,
    output        rd_n_sclk,
    output        refsel,
    output        reset,
    output        stby_n
);
    //------------- Timing calcs

    localparam real clk_period_ns = 1.0e9/clk_freq_hz;

    // Exit shutdown mode: stby_n rising to reset rising
    localparam real t_D_SDRST_ns = 50_000_000.0;
    //localparam real t_D_SDRST_ns =      1_000.0; //? test value
    //localparam real t_D_SDRST_ns   =    100_000.0; //? test value

    // Min time reset high
    //localparam real t_PH_RST_ns = 50_000.0; // Value from datasheet
    localparam real t_PH_RST_ns    = 1_000.0; //? test value

    // Min time reset falling to convst rising
    localparam real t_D_RSTCN_ns = 25_000.0;
    //localparam real t_D_RSTCN_ns   =  1_000.0; //? test value

    // Real (not enum) oversampling ratio.
    initial assert(0 <= oversamprat && oversamprat < 7) else $error("unexpected oversamprat");
    localparam time oversampling_ratio = time'(1 << oversamprat);

    // Sample cycle frequency and period.
    localparam real theo_f_CYC_hz = 500_000.0/oversampling_ratio;
    localparam real theo_t_CYC_ns = 1.0e9/theo_f_CYC_hz;
    localparam time  t_CYC_clks = `ceil_ti(theo_t_CYC_ns/clk_period_ns);
    localparam real t_CYC_ns = t_CYC_clks*clk_period_ns;

    // On power-up, observe the timing specified for leaving shutdown mode even though it's
    // much greater than the min reset timing. This allows the analog reference to settle.
    localparam real theo_reset_high_ns = `max(t_D_SDRST_ns, t_PH_RST_ns);
    localparam real theo_postreset_ns  = t_D_RSTCN_ns;

    localparam time reset_high_clks = `ceil_ti(theo_reset_high_ns/clk_period_ns);
    localparam real reset_high_ns = reset_high_clks*clk_period_ns;
    initial assert(0 < reset_high_clks) else $error("reset_high_clks too small");
    localparam time reset_high_count_u64 = reset_high_clks - 1;

    localparam time postreset_clks  = `ceil_ti(theo_postreset_ns/clk_period_ns);
    localparam real postreset_ns  = postreset_clks*clk_period_ns;
    initial assert(0 < postreset_clks) else $error("postreset_clks too small");
    localparam time postreset_count_u64  = postreset_clks - 1;

    initial assert(0 < t_CYC_clks) else $error("t_CYC_clks too small");
    localparam time sample_cyc_count_u64 = t_CYC_clks - 1;

    // Clk counter used for st_reset, post-reset, and sample period.
    localparam time sampcyc_ctr_max = `max( reset_high_count_u64,   `max(
                                            postreset_count_u64,     
                                            sample_cyc_count_u64    ));

    localparam int sampcyc_ctr_wid = $clog2(sampcyc_ctr_max + 1);
    typedef logic [sampcyc_ctr_wid-1 : 0] sampcyc_ctr_t;
    localparam sampcyc_ctr_t reset_high_count = sampcyc_ctr_t'(reset_high_count_u64);
    localparam sampcyc_ctr_t postreset_count  = sampcyc_ctr_t'(postreset_count_u64);
    localparam sampcyc_ctr_t sample_cyc_count = sampcyc_ctr_t'(sample_cyc_count_u64);

    initial assert(reset_high_count_u64 < (time'(1) << sampcyc_ctr_wid));
    initial assert(postreset_count_u64  < (time'(1) << sampcyc_ctr_wid));
    initial assert(sample_cyc_count_u64 < (time'(1) << sampcyc_ctr_wid));

    // Min pulse for CONVST[AB]
    localparam real theo_t_PHL_convst_ns =   25.0; // Datasheet value
    localparam time t_PHL_convst_clks = `ceil_ti(theo_t_PHL_convst_ns/clk_period_ns);
    localparam real t_PHL_convst_ns = t_PHL_convst_clks*clk_period_ns;

    initial assert(0 < t_PHL_convst_clks) else $error("t_PHL_convst_clks too small");
    localparam time t_PHL_convst_count_u64 = t_PHL_convst_clks - 1;

    // Rather than refer to "/CS and /RD asserted" everywhere, we'll just combine them into one
    // active-high signal called 'RD'.

    // Max delay from us asserting edge of RD to ADC DB[15:0] valid.
    localparam real theo_t_D_RDDB_max_ns =   12.0  // Datasheet value
                                           + 10.0; // Determined experimentally

    // Minimum setup time from ADC driving db[15:0] pins to latching clk edge.
    localparam real t_db_internal_setup_ns = 10.0;  // TODO get this from FPGA data

    localparam real t_rd_latch_db = theo_t_D_RDDB_max_ns + t_db_internal_setup_ns;
    localparam time t_D_rddb_clks = `ceil_ti(t_rd_latch_db/clk_period_ns);
    localparam real t_D_rddb_ns = t_D_rddb_clks*clk_period_ns;

    // Minimum hold time from clk edge latching db[15:0] before deasserting RD.
    localparam real t_db_internal_hold_ns =  10.0; // TODO get this from FPGA data

    localparam time t_db_hold_clks = `max(1, `ceil_ti(t_db_internal_hold_ns/clk_period_ns));
    localparam real t_db_hold_ns = t_db_hold_clks*clk_period_ns;

    // Min pulse for /CS and /RD
    localparam real theo_t_PH_rd_min_ns =   15.0; // Datasheet value, min /CS and /RD asserted time
    localparam real theo_t_PL_rd_min_ns =   15.0; // Datasheet value, min /CS and /RD asserted time

    // Assert 'RD' for t_D_rddb and t_db_hold, and at least theo_t_PHL_rd_min_ns
    localparam time t_PH_rd_clks = `max( t_D_rddb_clks + t_db_hold_clks,
                                         `ceil_ti(theo_t_PH_rd_min_ns/clk_period_ns) );
    localparam real t_PH_rd_ns   = t_PH_rd_clks*clk_period_ns;

    // De-assert 'RD' for at least theo_t_PHL_rd_min_ns
    localparam time t_PL_rd_clks = `ceil_ti(theo_t_PL_rd_min_ns/clk_period_ns);
    localparam real t_PL_rd_ns   = t_PL_rd_clks*clk_period_ns;

    //--- Counts

    initial assert(0 < t_D_rddb_clks) else $error("t_D_rddb_clks too small");
    localparam time t_D_rddb_count_u64 = t_D_rddb_clks - 1;

    initial assert(0 < t_db_hold_clks) else $error("t_db_hold_clks too small");
    localparam time t_db_rd_hold_count_u64 = t_db_hold_clks - 1;

    initial assert(0 < t_PL_rd_clks) else $error("t_PL_rd_clks too small");
    localparam time t_PL_rd_count_u64 = t_PL_rd_clks - 1;

    // Clk counter used for:
    //     t_P[HL]_CN      convst[ab] min pulse (high|low)
    //     t_P[HL]_(CS|RD) cs_n, rd_n min pulse (high|low)
    localparam time phl_ctr_max = `max( `max( t_PHL_convst_count_u64,    
                                                  t_D_rddb_count_u64      ),
                                            `max( t_db_rd_hold_count_u64,
                                                  t_PL_rd_count_u64       ) );
    localparam int phl_ctr_wid = $clog2(phl_ctr_max + 1);
    typedef logic [phl_ctr_wid-1 : 0] phl_ctr_t;
    localparam t_PHL_convst_count = phl_ctr_t'(t_PHL_convst_count_u64);
    localparam t_D_rddb_count     = phl_ctr_t'(t_D_rddb_count_u64);
    localparam t_db_rd_hold_count = phl_ctr_t'(t_db_rd_hold_count_u64);
    localparam t_PL_rd_count      = phl_ctr_t'(t_PL_rd_count_u64);

    initial assert(t_PHL_convst_count_u64 < (time'(1) << phl_ctr_wid));
    initial assert(t_D_rddb_count_u64     < (time'(1) << phl_ctr_wid));
    initial assert(t_db_rd_hold_count_u64 < (time'(1) << phl_ctr_wid));
    initial assert(t_PL_rd_count_u64      < (time'(1) << phl_ctr_wid));

    //localparam real theo_t_D_rdDB_ns = 12.0; // cs_n/rd_n falling edge to db[15:0] valid
    // Because theo_t_D_rdDB_ns <= theo_t_PHL_rd_min_ns < 2*theo_t_D_rdDB_ns, we can
    // just read db[15:0] and de-assert rd on the same cycle.

    // Read op index - eight channels of two rd ops each.
    //? TODO - shift-register?
    localparam time cnt_read_ops = chan_cnt*2;
    initial assert(0 < cnt_read_ops) else $error("cnt_read_ops too small");
    localparam time read_op_ix_max_u64 = cnt_read_ops - 1;
    localparam time read_op_ix_wid = $clog2(read_op_ix_max_u64 + 1);
    typedef logic [read_op_ix_wid-1 : 0] read_op_ix_t;
    localparam read_op_ix_t read_op_ix_max = read_op_ix_t'(read_op_ix_max_u64);

    initial assert(read_op_ix_max_u64 < (time'(1) << read_op_ix_wid));

    // Asserting edge of CONVST[AB] to rising edge of BUSY.
    localparam real theo_t_cn_bsy_max_ns = 15.0; // Datasheet value
    localparam time theo_t_cn_bsy_max_clks = `ceil_ti(theo_t_cn_bsy_max_ns/clk_period_ns);

    // Sample conversion time
    localparam real theo_t_conv_max_ns = oversamprat == 0 ?   1_290.0 : // Datasheet values
                                         oversamprat == 1 ?   3_290.0 :
                                         oversamprat == 2 ?   7_250.0 :
                                         oversamprat == 3 ?  15_180.0 :
                                         oversamprat == 4 ?  31_050.0 :
                                         oversamprat == 5 ?  62_770.0 :
                                                            126_200.0;
    localparam time t_conv_max_clks = `ceil_ti(theo_t_conv_max_ns/clk_period_ns);
    localparam real t_conv_max_ns = t_conv_max_clks*clk_period_ns;

    // Total time to read out the data is 
    //     rd_phase_clks*((two phases)*(two words)*(eight channels) - (last phase)).
    localparam time t_readout_clks = (t_PL_rd_clks + t_PH_rd_clks)*(2*8 - 1);
    localparam real t_readout_ns = t_readout_clks*clk_period_ns;

    // Sample cycle critical path: acquisition time and readout time
    localparam time sampcycle_critpath_max_clks =   theo_t_cn_bsy_max_clks
                                                  + t_conv_max_clks
                                                  + t_readout_clks;
    //localparam real sampcyc_min_hz = clk_freq_hz/sampcycle_critpath_max_clks;
    localparam time expected_sampcyc_clks = `max(sampcycle_critpath_max_clks, t_CYC_clks);
    localparam real expected_sampcyc_freq_hz = clk_freq_hz/expected_sampcyc_clks;

    initial begin
        $info("Clk: %f MHz", clk_freq_hz/1.0e6);
        $info("oversampling_ratio: %0d", oversampling_ratio);
        $info("reset high:   %0d clks = %f ns", reset_high_clks, reset_high_ns);
        $info("post-reset:   %0d clks = %f ns", postreset_clks,  postreset_ns);
        $info("sample cycle: theo min %0d clks = %f ns = theo max %f KHz", t_CYC_clks, t_CYC_ns, 1.0e6/t_CYC_ns);
        $info("sampcyc_ctr: max %0d, width %0d", sampcyc_ctr_max, sampcyc_ctr_wid);
        $info("t_P[HL]_convst: %0d clks = %f ns", t_PHL_convst_clks, t_PHL_convst_ns);
        $info("t_D_rddb:  %0d clks = %f ns", t_D_rddb_clks, t_D_rddb_ns);
        $info("t_db_hold: %0d clks = %f ns", t_db_hold_clks, t_db_hold_ns);
        $info("t_PH_rd: %0d clks = %f ns", t_PH_rd_clks, t_PH_rd_ns);
        $info("t_PL_rd: %0d clks = %f ns", t_PL_rd_clks, t_PL_rd_ns);
        $info("t_conv: max %0d clks = max %f ns", t_conv_max_clks, t_conv_max_ns);
        $info("t_readout: %0d clks = %f ns", t_readout_clks, t_readout_ns);
        $info("phl_ctr: max %0d, width %0d", phl_ctr_max, phl_ctr_wid);
        $info("read_op_ix: %0d max, width %0d", read_op_ix_max, read_op_ix_wid);

        $info("sample cycle crit path: %0d clks = %f ns",
            sampcycle_critpath_max_clks, sampcycle_critpath_max_clks*clk_period_ns);

        if (sampcycle_critpath_max_clks <= t_CYC_clks)
            $info("sample cycle crit path has %0d clks = %0f ns slack",
                t_CYC_clks - sampcycle_critpath_max_clks,
                (t_CYC_clks - sampcycle_critpath_max_clks)*clk_period_ns);
        else
            $info("sample cycle crit path has %0d clks = %0f ns overrun",
                sampcycle_critpath_max_clks - t_CYC_clks,
                (sampcycle_critpath_max_clks - t_CYC_clks)*clk_period_ns);

        $info("expected sample rate: %f KHz", expected_sampcyc_freq_hz/1.0e3);
        $info("expected sample freq %f %s", expected_sampcyc_freq_hz/theo_f_CYC_hz*100.0, "% of theoretical max");
    end

    //------------- Constant output wires

    // Standby pin
    localparam stdby_n_POWER_UP   = 1'b1;
    //localparam stdby_n_POWER_DOWN = 1'b0;

    always_comb begin
        os[2:0] = oversamprat;
        range   = param_range;
        refsel  = ref_source;
        stby_n  = stdby_n_POWER_UP;
    end
    
    //------------- Flopped state.

    // I'd rather use a typdef here and make it part of state_t, but Quartus 17.0
    // doesn't recognize it as a state machine and it ends up being 32 bits wide.
    enum int unsigned
    {
            st_initial       ,  // =  0,
            st_reset         ,  // =  1,
            st_postreset     ,  // =  2,
            st_preconvst     ,  // =  3,
            st_convst        ,  // =  4,
            st_t_PH_convst   ,  // =  5,
            st_wait_notbusy  ,  // =  6,
            st_t_D_rddb      ,  // =  7,
            st_t_db_rd_hold  ,  // =  8,
            st_read_rd0      ,  // =  9,
            st_fail             // = 10
    } state_prev_ff, state_ff, state_nxt;

    typedef struct {
        // Enregistered module outputs
        logic             conv_cnt_m2_;
        logic             octosample_valid_;
        octosample_t      octosample_;
        logic             samp_cnt_m2_;

        // Internal-only state
        sampcyc_ctr_t  sampcyc_ctr;
        phl_ctr_t      phl_ctr;
        octosample_t   partial;
        read_op_ix_t   read_op_ix;
    } state_t;

    const state_t ff_initial = '{                // Initial/reset state
                        conv_cnt_m2_      : '0,
                        octosample_valid_ : '0,
                        samp_cnt_m2_      : '0,
                        default           : 'x };

    const state_t ff_fail = '{                 // Fail state
                        conv_cnt_m2_      : '0,
                        octosample_valid_ : '0,
                        samp_cnt_m2_      : '0,
                        default           : 'x };

    state_t ff = ff_initial;
    state_t ff_nxt;

    //------------- Synchronize inputs

    logic busy;
    sync sync_busy( .clk(clk), .async( busy_asy ), .sync( busy ) );

    // db_asy doesn't need sync because we explicitly observe setup and hold

    //------------- State update

    always_ff @(posedge clk) begin
        if (~reset_sn) begin
            state_prev_ff <= st_initial;
            state_ff      <= st_initial;
            ff            <= ff_initial;
        end else begin
            state_prev_ff <= state_ff;
            state_ff      <= state_nxt;
            ff            <= ff_nxt;
        end
    end // always_ff

    //------------- Combinationals

    logic convst; // drives convst[ab]
    logic rd;     // drives cs_n and rd_n

    always_comb begin
        state_nxt      = state_ff;
        ff_nxt         = ff;
        convst         = 1'b0;
        rd             = 1'b0;
        reset          = 1'b0;
        fail           = 1'b0;
        latch_msbs     = '0;

        // Counters count down
        ff_nxt.sampcyc_ctr = ff.sampcyc_ctr + (ff.sampcyc_ctr ? sampcyc_ctr_t'(-1) : '0);
        ff_nxt.phl_ctr     = ff.phl_ctr     + (ff.phl_ctr     ?     phl_ctr_t'(-1) : '0);

        //--------- State initial entry actions
        //
        // If this is the first clock in the current state, do 'on entry' actions.

        if (state_prev_ff != state_ff) begin
            case (state_ff)
                st_t_db_rd_hold: begin    //----------------------------- on entering st_t_db_rd_hold
                    latch_msbs = '1;
                end

                st_read_rd0: begin        //--------------------------------- on entering st_read_rd0

                    // If the bits we just read completed a channel value
                    if (ff.read_op_ix[0]) begin

                        // If there are still more channel values in this octosample
                        if (ff.read_op_ix != read_op_ix_max) begin

                            // Shift data in channels [1:7] into channels [0:6]
                            ff_nxt.partial[0 : chan_ix_max-1] = ff.partial[1 : chan_ix_max];

                        end else begin // We have completed the octosample

                            // Emit completed octosample.
                            ff_nxt.octosample_valid_ = '1;
                            ff_nxt.octosample_ = ff.partial;
                            ff_nxt.samp_cnt_m2_ = ~ff.samp_cnt_m2_;
                        end
                    end

                end // st_read_rd0

                default: begin end
            endcase
        end

        //--------- Current state evaluations

        case (state_ff)
            st_initial: begin              //--------------------------------------------- st_initial
                    reset = 1'b1;

                    state_nxt = st_reset;
                end

            st_reset: begin                   //-------------------------------------------- st_reset
                    reset = 1'b1;

                    if (!ff.sampcyc_ctr)
                        state_nxt = st_postreset;
                end

            st_postreset: begin               //---------------------------------------- st_postreset
                    if (!ff.sampcyc_ctr)
                        state_nxt = st_preconvst;
                end

            st_preconvst: begin               //---------------------------------------- st_preconvst
                    // Waiting until we the counter says we can start the next sample.

                    if (!ff.sampcyc_ctr)
                        state_nxt = st_convst;
                end

            st_convst: begin                  //------------------------------------------- st_convst

                    // Waiting until we get the 'busy' signal from the chip

                    convst = 1'b1;

                    //if (!ff.sampcyc_ctr) // Something really bad happened
                    //    ff_nxt = ff_fail;
                    //else
                    if (busy)
                        state_nxt = st_t_PH_convst;
                end

            st_t_PH_convst: begin             //-------------------------------------- st_t_PH_convst

                    // We have 'busy', continue asserting convst until t_PH_convst has elapsed              

                    convst = 1'b1;

                    //if (!ff.sampcyc_ctr) // Something really bad happened
                    //    ff_nxt = ff_fail;
                    //else
                    if (!ff_nxt.phl_ctr)
                        state_nxt = st_wait_notbusy;
                end

            st_wait_notbusy: begin             //------------------------------------ st_wait_notbusy

                    // Drop convst, wait until we get not-busy (at least a us)

                    //if (!ff.sampcyc_ctr) // Something really bad happened
                    //    ff_nxt = ff_fail;
                    //else
                    if (!busy)
                        state_nxt = st_t_D_rddb;
                end

            st_t_D_rddb: begin      //--------------------------------------------------- st_t_D_rddb

                    // assert rd to request the next output              

                    rd = 1'b1;

                    if (!ff_nxt.phl_ctr)
                        state_nxt = st_t_db_rd_hold;
                end

            st_t_db_rd_hold: begin        //----------------------------------------- st_t_db_rd_hold

                    // Continue asserting rd during hold time

                    rd = 1'b1;

                    if (!ff_nxt.phl_ctr) begin
                        state_nxt = st_read_rd0;
                    end
                end

            st_read_rd0: begin                //----------------------------------------- st_read_rd0

                    // De-assert rd

                    if (!ff_nxt.phl_ctr) begin
                        // Set up for either the next read op or the next cycle.
                        state_nxt = ff.read_op_ix != read_op_ix_max ? st_t_D_rddb : st_preconvst;
                    end
                end

            st_fail: begin                        //----------------------------------------- st_fail
                    ff_nxt = ff_fail;
                end

            default: begin end
        endcase

        //----------------------------------------------------
        //
        // If this is the last clock in the current state, the next rising edge will
        // put us in a different state.

        if (state_nxt != state_ff) begin

            //--------- state pre-exit actions

            case (state_ff)
                st_wait_notbusy: begin      //------------------------ before exiting st_wait_notbusy
                        // Start with first read_op_ix.
                        ff_nxt.read_op_ix = read_op_ix_t'(0);
                    end

                st_read_rd0: begin              //------------------------ before exiting st_read_rd0
                        // Advance to next read_op_ix, if any.
                        ff_nxt.read_op_ix = ff.read_op_ix + read_op_ix_t'(1);
                    end

                default: begin end
            endcase

            //--------- state pre-entry actions

            case (state_nxt)
                st_reset: begin                   //------------------------ before entering st_reset
                        ff_nxt.sampcyc_ctr = reset_high_count;
                    end

                st_postreset: begin           //------------------------ before entering st_postreset
                        ff_nxt.sampcyc_ctr = postreset_count;
                    end

                st_preconvst: begin           //------------------------ before entering st_preconvst
                        if (state_ff == st_postreset) begin
                            // If entering st_preconvst from st_reset, sampcyc_ctr will
                            // correctly be 0.
                        end else begin
                            // Sampcyc_ctr should be counting down until we can start the next
                            // conversion. But if it's already at 0, we have probably overrun our
                            // cycle period.
                            //if (!ff.sampcyc_ctr)
                            //    ff_nxt.overrun = 1'b1;
                        end
                    end

                st_convst: begin                 //------------------------ before entering st_convst
                        ff_nxt.conv_cnt_m2_ = ~ff.conv_cnt_m2_;    // Report beginning of new acquisition
                        ff_nxt.sampcyc_ctr = sample_cyc_count; // Restart sample cycle period counter
                        ff_nxt.phl_ctr = t_PHL_convst_count;   // Restart counter for t_PH_convst
                    end

                st_t_D_rddb: begin             //------------------------ before entering st_t_D_rddb
                        ff_nxt.phl_ctr = t_D_rddb_count; // Reset counter for rd_rddb_clks
                    end

                st_t_db_rd_hold: begin     //------------------------ before entering st_t_db_rd_hold
                        ff_nxt.phl_ctr = t_db_rd_hold_count; // Reset counter for rd_hold_clks

                        latch_msbs = '1;

                        // Read data into channel [7]
                        if (!ff.read_op_ix[0]) begin
                            // Read op 1 of 2: read top 16 bits
                            ff_nxt.partial[chan_ix_max] = { db_asy[15:0], 1'b0, 1'b0 };
                            //ff_nxt.partial[chan_ix_max][ sample_wid-1 : 2 ] = db_asy[15:0];

                        end else begin
                            // Read op 2 of 2: read bottom 2 bits
                            ff_nxt.partial[chan_ix_max][1:0] = db_asy[15:14];
                        end
                    end

                st_read_rd0: begin             //------------------------ before entering st_read_rd0
                        ff_nxt.phl_ctr = t_PL_rd_count; // Reset counter for t_PH_rd
                    end

                default: begin end
            endcase
        end // if
    end // always_comb

    //------------- assign outputs

    always_comb begin
        conv_cnt_m2      = ff.conv_cnt_m2_;
        octosample_valid = ff.octosample_valid_;
        octosample       = ff.octosample_;
        samp_cnt_m2      = ff.samp_cnt_m2_;

        // If the top two bits are unequal the clock before or after the MSB edge
        // there may be a problem
        glitch = latch_msbs & (db_asy[15]^db_asy[14]);

        convsta = convst;
        convstb = convst;
        cs_n      = ~rd;
        rd_n_sclk = ~rd;
    end
 
endmodule // driver

//-------------------------------------------------------------------------------------------------|
