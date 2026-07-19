// leland_dac_mixer -- WP7 of docs/planning_80186_sound.md ("DAC/mixer
// datapath + audio integration stub").
//
// Takes WP6's (leland_sound_board.sv) digital DAC register outputs --
// 6x 8-bit sample+volume pairs and one 10-bit sample -- and produces a
// single mono 16-bit signed PCM stream (MiSTer's AUDIO_L/AUDIO_R
// convention; leland_a.cpp's own mconfig is a single SPEAKER
// "front_center", so mono duplicated to both channels is the faithful
// mapping, done by the WP10 caller).
//
// Analog-path model: each 8-bit channel is a real AD7524 multiplying
// DAC whose reference is itself driven by a second DAC
// (DAC_8BIT_BINARY_WEIGHTED, the "volume" register) --
// `m_dacvol[i]->add_route(0, m_dac[i], 1.0, DAC_INPUT_RANGE_HI)` /
// `(..., -1.0, DAC_INPUT_RANGE_LO)` in leland_a.cpp's mconfig is MAME's
// idiom for "volume acts as a VCA on the sample", i.e. the analog
// output is proportional to `sample * volume`. Both `sample` and
// `volume` are unsigned 8-bit game-authored PCM/gain codes centered at
// 128 (standard 8-bit-unsigned-PCM convention) -- this module treats
// `sample` as offset-binary (subtracts 128 to recover the signed
// waveform) and `volume` as a plain 0-255 unsigned gain multiplier.
// `dac9` (AD7533, no separate volume register -- it IS the 10-bit
// sample) is likewise offset-binary, centered at 512.
//
// Gains: leland_a.cpp's mconfig routes each 8-bit channel at 0.2 and
// dac9 at 1.0 (`add_route(ALL_OUTPUTS, "speaker", 0.2)` / `1.0`).
// Implemented here as Q8 fixed-point multipliers (GAIN_8BIT=51/256 ~=
// 0.199, GAIN_10BIT=255/256 ~= 0.996) applied after each channel's own
// sample*volume (or sample-only, for dac9) product -- close enough for
// an integration stub (this WP's own acceptance criterion is "Stage C
// WAV criterion" + synthetic test tones, not bit-exact gain matching).
// dac9 has no volume register, so its raw 11-bit signed range (-512..
// 511) is first scaled by DAC9_SCALE (matching one 8-bit channel's
// full-scale product magnitude, ~32640) before the same gain stage, so
// the two channel types land in a comparable numeric range prior to
// summing -- an explicit, documented normalization choice, not a
// literal MAME quantity.
module leland_dac_mixer(
    input  logic        clk,
    input  logic        reset,

    input  logic [7:0]  dac_sample [0:5],
    input  logic [7:0]  dac_vol    [0:5],
    input  logic [9:0]  dac9_sample,

    output logic signed [15:0] audio_out
);

localparam signed [8:0] GAIN_8BIT_Q8  = 9'sd51;  // 0.2  * 256, ~0.199
localparam signed [8:0] GAIN_10BIT_Q8 = 9'sd255; // 1.0  * 256, ~0.996
localparam signed [10:0] DAC9_SCALE   = 11'sd64; // 511*64=32704, comparable to an 8-bit channel's max product (127*255=32385)

logic signed [8:0]  sdiff  [0:5]; // sample - 128
logic signed [16:0] prod   [0:5]; // sdiff * volume
logic signed [24:0] gprod  [0:5]; // prod * GAIN_8BIT_Q8 (pre-shift)
logic signed [16:0] gained [0:5]; // gprod >>> 8

logic signed [10:0] sdiff9;       // dac9_sample - 512
logic signed [17:0] dac9_full;    // sdiff9 * DAC9_SCALE
logic signed [26:0] gprod9;       // dac9_full * GAIN_10BIT_Q8 (pre-shift)
logic signed [17:0] gained9;      // gprod9 >>> 8

logic signed [20:0] sum_all;
logic signed [15:0] sat;

genvar gi;
generate
    for (gi = 0; gi < 6; gi = gi + 1) begin : g_ch
        always_comb begin
            sdiff[gi]  = $signed({1'b0, dac_sample[gi]}) - 9'sd128;
            prod[gi]   = sdiff[gi] * $signed({1'b0, dac_vol[gi]});
            gprod[gi]  = prod[gi] * GAIN_8BIT_Q8;
            gained[gi] = gprod[gi] >>> 8;
        end
    end
endgenerate

always_comb begin
    sdiff9    = $signed({1'b0, dac9_sample}) - 11'sd512;
    dac9_full = sdiff9 * DAC9_SCALE;
    gprod9    = dac9_full * GAIN_10BIT_Q8;
    gained9   = gprod9 >>> 8;
end

// Pipeline stage (Quartus timing closure, docs/WP10_PROGRESS.md): the
// full chain from dac_sample/dac_vol through both multiplies, the
// 7-way sum, and saturation was too deep to fit in one clk_sys cycle
// (48 MHz, 20.833ns period) -- confirmed by TimeQuest as this design's
// single worst path once the unrelated WP10 arbiter combinational
// loops were fixed (-2.135ns worst-case setup slack, entirely within
// this module, from dac_sample's register through Mult2/Mult3/Add9/
// Add10 to audio_out). Splitting the per-channel gain stage (DSP
// multiplies) from the summation+saturation stage with one register
// gives each half its own cycle. Adds exactly one clk_sys cycle of
// audio-pipeline latency (~21ns at 48MHz) -- inaudible, and
// leland_dac_mixer_tb.sv already waits 3 cycles before sampling
// audio_out per change, so this needs no test-side timing update.
logic signed [16:0] gained_r  [0:5];
logic signed [17:0] gained9_r;

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        gained_r[0] <= 17'sd0; gained_r[1] <= 17'sd0; gained_r[2] <= 17'sd0;
        gained_r[3] <= 17'sd0; gained_r[4] <= 17'sd0; gained_r[5] <= 17'sd0;
        gained9_r   <= 18'sd0;
    end else begin
        gained_r[0] <= gained[0]; gained_r[1] <= gained[1]; gained_r[2] <= gained[2];
        gained_r[3] <= gained[3]; gained_r[4] <= gained[4]; gained_r[5] <= gained[5];
        gained9_r   <= gained9;
    end
end

always_comb begin
    sum_all = gained_r[0] + gained_r[1] + gained_r[2] + gained_r[3] + gained_r[4] + gained_r[5] + gained9_r;

    // Saturate to signed 16-bit (MiSTer's AUDIO_L/AUDIO_R width).
    if (sum_all > 21'sd32767)
        sat = 16'sd32767;
    else if (sum_all < -21'sd32768)
        sat = -16'sd32768;
    else
        sat = sum_all[15:0];
end

always_ff @(posedge clk or posedge reset) begin
    if (reset) audio_out <= 16'sd0;
    else       audio_out <= sat;
end

endmodule
