//============================================================================
//  Super Off Road — MiSTer FPGA Core
//  Leland / Tradewest 1989
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

//----------------------------------------------------------------
// Unused port defaults
//----------------------------------------------------------------
assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
// SDRAM pins are driven by sor_board (sdram controller inside)
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = '0;

assign VGA_SL       = 0;
assign VGA_F1       = 0;
assign VGA_SCALER   = 0;
assign VGA_DISABLE  = 0;
assign HDMI_FREEZE  = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

// Signed 16-bit audio (WP10): leland_dac_mixer's mono output, via
// sor_board's audio_out port, duplicated to both channels -- real
// hardware's own mconfig is mono (leland_a.cpp SPEAKER(config,
// "speaker").front_center()), so this is the complete, faithful signal,
// not a placeholder stereo split.
assign AUDIO_S   = 1;
assign AUDIO_L   = audio_out;
assign AUDIO_R   = audio_out;
assign AUDIO_MIX = 0;

assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;

//----------------------------------------------------------------
// OSD / HPS configuration
//----------------------------------------------------------------
wire [1:0] ar = status[122:121];
assign VIDEO_ARX = (!ar) ? 12'd4 : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? 12'd3 : 12'd0;

`include "build_id.v"
localparam CONF_STR = {
	"SuperOffRoad;;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[2],Orientation,Landscape,Portrait;",
	"-;",
	"O[4],Service,Off,On;",
	"O[5],Free Play,Off,On;",
	"-;",
	"O[7:6],Lives,3,4,5,2;",
	"O[9:8],Difficulty,Normal,Easy,Hard,Very Hard;",
	"-;",
	"O[10],D-Pad Steering,Velocity,Position;",
	"-;",
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	"J1,Nitro,Coin,Gas;",
	"v,0;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire  [1:0] buttons;
wire [127:0] status;
wire  [10:0] ps2_key;

// ROM loading
wire        ioctl_download;
wire [15:0] ioctl_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;
// ioctl_wait driven by sor_board (stalls HPS during SDRAM writes / init)
wire        ioctl_wait;

// Three-player digital buttons ([3]=coin, [2]=btn2, [1]=btn1, [0]=btn0)
wire [31:0] joy1, joy2, joy3;
// Analog: signed -127..+127, [15:8]=Y (unused -- gas is digital, see p1_pedal
// below), [7:0]=X (one of three steering inputs combined by steering_input.sv)
wire [15:0] joy1_ana, joy2_ana, joy3_ana;
// Spinner: [8]=toggle (flips on every host update), [7:0]=signed delta
wire  [8:0] spinner1, spinner2, spinner3;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),

	.forced_scandoubler(forced_scandoubler),
	.buttons(buttons),
	.status(status),
	.status_menumask(0),
	.ps2_key(ps2_key),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),

	.joystick_0(joy1),
	.joystick_1(joy2),
	.joystick_2(joy3),
	.joystick_l_analog_0(joy1_ana),
	.joystick_l_analog_1(joy2_ana),
	.joystick_l_analog_2(joy3_ana),

	.spinner_0(spinner1),
	.spinner_1(spinner2),
	.spinner_2(spinner3),

	// Unused rumble outputs
	.joystick_0_rumble(16'd0),
	.joystick_1_rumble(16'd0),
	.joystick_2_rumble(16'd0)
);

//----------------------------------------------------------------
// Clock  (PLL: 50 MHz → 48 MHz system clock)
// 48 MHz gives clean dividers:
//   Z80  @ 6 MHz  → CE every 8 cycles
//   80186@ 8 MHz  → CE every 6 cycles
//   Pixel@ ~7.16MHz → fractional phase accumulator
//----------------------------------------------------------------
wire clk_sys;
wire clk_sdram; // phase-shifted 48 MHz PLL output feeding SDRAM_CLK (docs/sdram_plan.md Section 3a)
wire pll_locked;

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(clk_sdram),
	.locked(pll_locked)
);

wire reset      = RESET | status[0] | buttons[1] | ioctl_download | ~pll_locked;
wire sdram_init = RESET | ~pll_locked;

//----------------------------------------------------------------
// Video signals from board
//----------------------------------------------------------------
wire       ce_pix;
wire       HBlank, HSync, VBlank, VSync;
wire [3:0] pen;         // 4-bit pixel pen value from video RAM
wire [23:0] rgb;        // 24-bit colour after palette lookup

wire signed [15:0] audio_out; // WP10: leland_dac_mixer's mono output

//----------------------------------------------------------------
// Steering — combine analog stick, digital d-pad, and spinner into the
// free-running "virtual dial" position sor_board's p*_wheel ports expect
// (see steering_input.sv and sor_master.sv's wheel port comment for why
// this can't just be the raw analog stick value any more: the real wheel
// is a free-spinning rotary encoder, not an absolute position).
//----------------------------------------------------------------
reg vblank_d;
always @(posedge clk_sys) vblank_d <= VBlank;
wire ce_frame = VBlank & ~vblank_d; // once-per-frame pulse for the steering ramp

wire [7:0] p1_wheel_pos, p2_wheel_pos, p3_wheel_pos;

steering_input steer1
(
	.clk_sys(clk_sys), .reset(reset), .ce_frame(ce_frame),
	.dpad_pos_mode(status[10]),
	.analog_x(joy1_ana[7:0]),
	.dpad_left(joy1[1]), .dpad_right(joy1[0]),
	.spinner(spinner1),
	.wheel_pos(p1_wheel_pos)
);

steering_input steer2
(
	.clk_sys(clk_sys), .reset(reset), .ce_frame(ce_frame),
	.dpad_pos_mode(status[10]),
	.analog_x(joy2_ana[7:0]),
	.dpad_left(joy2[1]), .dpad_right(joy2[0]),
	.spinner(spinner2),
	.wheel_pos(p2_wheel_pos)
);

steering_input steer3
(
	.clk_sys(clk_sys), .reset(reset), .ce_frame(ce_frame),
	.dpad_pos_mode(status[10]),
	.analog_x(joy3_ana[7:0]),
	.dpad_left(joy3[1]), .dpad_right(joy3[0]),
	.spinner(spinner3),
	.wheel_pos(p3_wheel_pos)
);

// Gas — MiSTer has no analog-trigger support, so this is a plain digital
// button (3rd J1 entry, "Gas") mapped like Nitro/Coin, driving the pedal
// straight to its two real endpoints (0=released, 255=full) rather than
// through the now-unused analog Y axis (WP-controls: that Y-axis pedal
// wiring was dead code before this change anyway -- see sor_board.sv's
// p1_pedal port comment).
wire [7:0] p1_gas = joy1[6] ? 8'hFF : 8'h00;
wire [7:0] p2_gas = joy2[6] ? 8'hFF : 8'h00;
wire [7:0] p3_gas = joy3[6] ? 8'hFF : 8'h00;

//----------------------------------------------------------------
// Board
//----------------------------------------------------------------
sor_board board
(
	.clk_sys(clk_sys),
	.clk_sdram(clk_sdram),
	.reset(reset),
	.sdram_init(sdram_init),

	// ROM loading
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_data(ioctl_dout),
	.ioctl_wait(ioctl_wait),

	// SDRAM
	.SDRAM_DQ  (SDRAM_DQ),
	.SDRAM_A   (SDRAM_A),
	.SDRAM_BA  (SDRAM_BA),
	.SDRAM_CLK (SDRAM_CLK),
	.SDRAM_CKE (SDRAM_CKE),
	.SDRAM_nCS (SDRAM_nCS),
	.SDRAM_nRAS(SDRAM_nRAS),
	.SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_nWE (SDRAM_nWE),
	.SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH),

	// Video output
	.ce_pix(ce_pix),
	.HBlank(HBlank),
	.HSync(HSync),
	.VBlank(VBlank),
	.VSync(VSync),
	.rgb(rgb),

	// Player inputs (digital buttons + analog)
	//
	// sor_board/sor_master's p1_btn[1]=nitro, p1_btn[3]=coin (that
	// internal meaning is validated against MAME's leland_m.cpp GIN0/
	// GIN1 bit layout, unchanged). What changed is which PHYSICAL
	// input feeds those bits: joystick_0[3:0] are MiSTer's standard
	// D-pad bits, auto-mapped with no CONF_STR label and no entry in
	// the OSD's "define buttons" UI -- there was no way for a user to
	// discover or remap Coin at all (confirmed: sticky I/O-read probes
	// on real hardware showed the Master CPU polling both the nitro
	// and coin ports in what turned out to be the normal, correct
	// attract-mode "wait for coin" idle loop, not a hang -- but coin
	// had no reachable input). Sourced from joystick_0[5:4] instead,
	// the standard first-two-named-buttons position (see the "J1,..."
	// CONF_STR line above), so Nitro/Coin get real, labeled, mappable
	// buttons like every other MiSTer arcade core.
	.p1_btn({joy1[5], 1'b0, joy1[4], 1'b0}),
	.p2_btn({joy2[5], 1'b0, joy2[4], 1'b0}),
	.p3_btn({joy3[5], 1'b0, joy3[4], 1'b0}),
	// Wheel: free-running virtual dial position from steering_input.sv
	// (analog stick + digital d-pad + spinner already combined -- see the
	// "Steering" block above and sor_master.sv's p1_wheel port comment).
	.p1_wheel(p1_wheel_pos),
	.p2_wheel(p2_wheel_pos),
	.p3_wheel(p3_wheel_pos),
	// Gas: digital button (J1's 3rd entry), 0/255 -- see p1_gas comment above.
	.p1_pedal(p1_gas),
	.p2_pedal(p2_gas),
	.p3_pedal(p3_gas),

	// Service / free play from OSD
	.service(status[4]),
	.free_play(status[5]),

	.audio_out(audio_out)
);

//----------------------------------------------------------------
// Video output to MiSTer framework
//----------------------------------------------------------------
assign CLK_VIDEO = clk_sys;
assign CE_PIXEL  = ce_pix;

assign VGA_DE = ~(HBlank | VBlank);
assign VGA_HS = HSync;
assign VGA_VS = VSync;
assign VGA_R  = rgb[23:16];
assign VGA_G  = rgb[15:8];
assign VGA_B  = rgb[7:0];

//----------------------------------------------------------------
// Activity LED — heartbeat while board is running
//----------------------------------------------------------------
reg [26:0] act_cnt;
always @(posedge clk_sys) act_cnt <= act_cnt + 1'd1;
assign LED_USER = act_cnt[26] ? act_cnt[25:18] > act_cnt[7:0]
                               : act_cnt[25:18] <= act_cnt[7:0];

endmodule
