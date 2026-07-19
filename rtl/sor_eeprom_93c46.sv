//============================================================================
//  Super Off Road — 93C46 3-wire serial EEPROM (64 x 16-bit)
//
//  Standard Microwire protocol, validated against MAME leland.cpp/leland_m.cpp:
//    EEPROM_93C46_16BIT(config, m_eeprom) -- 64 words x 16 bits.
//    Wiring (leland_master_output_w, /MCONT write, port 0x09):
//      bit4 = DI (data in), bit5 = CLK, bit6 = CS
//    Wiring (leland_master_input_r, /GIN3 read): bit0 = DO (data out),
//      via PORT_READ_LINE_DEVICE_MEMBER("eeprom", ...::do_read).
//
//  Command format (shifted in MSB-first while CS=1, sampled on CLK rising
//  edge): START(1) + OPCODE(2) + ADDRESS(6), then 16 data bits for WRITE,
//  or 16 clocks of DO output for READ (MSB first). EWEN/EWDS/ERASE
//  opcodes are accepted (bit pattern decoded) but are no-ops here --
//  this core doesn't need write-protect semantics to boot, only correct
//  READ/WRITE of the 64 words.
//============================================================================

module sor_eeprom_93c46
(
	input        clk_sys,
	input        reset,
	input        cs,      // chip select (MCONT bit6)
	input        clk_in,  // serial clock (MCONT bit5)
	input        di,      // data in (MCONT bit4)
	output reg   do_out   // data out (GIN3 bit0)
);

reg [15:0] mem [0:63];
initial begin
	// Default content == sim/eeprom-offroad.bin (128 bytes, CRC 57955248),
	// the same default image documented in mra/SuperOffRoad.mra. Big-
	// endian byte pairs (matches leland_m.cpp's eeprom_data[o*2+0]=hi,
	// [o*2+1]=lo convention).
	mem[0]=16'h07BB; mem[1]=16'hF483; mem[2]=16'hFFFF; mem[3]=16'hFFFF;
	mem[4]=16'hFFFF; mem[5]=16'hFFFF; mem[6]=16'hFFFF; mem[7]=16'hFFFF;
	mem[8]=16'hFFFF; mem[9]=16'hFEFE; mem[10]=16'hFFFB; mem[11]=16'hFFFF;
	mem[12]=16'hFFFF; mem[13]=16'h00FF; mem[14]=16'hFFFB; mem[15]=16'hFFFF;
	mem[16]=16'hFFFF; mem[17]=16'hFFFF; mem[18]=16'hFFFF; mem[19]=16'hFFFF;
	mem[20]=16'hFFFF; mem[21]=16'hFFFF; mem[22]=16'hFFFF; mem[23]=16'hFFFF;
	mem[24]=16'hFFFF; mem[25]=16'hFFFF; mem[26]=16'hFFFF; mem[27]=16'hFFFF;
	mem[28]=16'hFFFF; mem[29]=16'hFFFF; mem[30]=16'hFFFF; mem[31]=16'hFFFF;
	mem[32]=16'hFFFF; mem[33]=16'hFFFF; mem[34]=16'hFFFF; mem[35]=16'hFFFF;
	mem[36]=16'hFFFF; mem[37]=16'hFFFF; mem[38]=16'hFFFF; mem[39]=16'hFFFF;
	mem[40]=16'hFFFF; mem[41]=16'hFFFF; mem[42]=16'hFFFF; mem[43]=16'hFFFF;
	mem[44]=16'hFFFF; mem[45]=16'hFFFF; mem[46]=16'hFFFF; mem[47]=16'hFFFF;
	mem[48]=16'hFFFF; mem[49]=16'hFFFF; mem[50]=16'hFFFF; mem[51]=16'hFFFF;
	mem[52]=16'hFFFF; mem[53]=16'hFFFF; mem[54]=16'hFEFF; mem[55]=16'hFEFE;
	mem[56]=16'hFFFE; mem[57]=16'h50FF; mem[58]=16'h976C; mem[59]=16'hFFAD;
	mem[60]=16'hFFFF; mem[61]=16'hFFFF; mem[62]=16'hFFFF; mem[63]=16'hFFFF;
end

localparam S_WAIT_START = 0, S_CMD = 1, S_DATA = 2;
reg [1:0]  state;
reg [7:0]  cmd_shift;   // {op[1:0], addr[5:0]} accumulated after the start bit
reg [3:0]  cmd_bits;    // 0..8
reg [15:0] data_shift;
reg [4:0]  data_bits;   // 0..16
reg [1:0]  op;
reg [5:0]  eaddr;
reg        clk_prev;

wire clk_rise = clk_in & ~clk_prev;

always @(posedge clk_sys) begin
	clk_prev <= clk_in;

	if (reset || !cs) begin
		state    <= S_WAIT_START;
		cmd_bits <= 4'd0;
		data_bits<= 5'd0;
		do_out   <= 1'b1; // idle/ready high
	end else if (clk_rise) begin
		case (state)
			S_WAIT_START: begin
				if (di) state <= S_CMD; // start bit seen; ignore leading zeros
			end

			S_CMD: begin
				cmd_shift <= {cmd_shift[6:0], di};
				cmd_bits  <= cmd_bits + 4'd1;
				if (cmd_bits == 4'd7) begin
					// This shift completes the 8th bit after start: op[1:0]
					// then addr[5:0], MSB-first -> {cmd_shift[6:0],di}.
					op     <= {cmd_shift[6], cmd_shift[5]};
					eaddr  <= {cmd_shift[4:0], di};
					state  <= S_DATA;
					data_bits <= 5'd0;
				end
			end

			S_DATA: begin
				if (op == 2'b10) begin // READ
					// MAME eepromser.cpp STATE_READING_DATA: on the first data
					// clock, load the word and drive its MSB; on later clocks,
					// shift. The load and the shift are exclusive per edge --
					// two nonblocking assigns to data_shift on the same edge
					// would let the shift silently override the load (the bug
					// that returned 0x7FFF for every word: first bit right,
					// fifteen stale bits after it).
					if (data_bits == 5'd0) begin
						do_out     <= mem[eaddr][15];
						data_shift <= mem[eaddr] << 1;
					end else begin
						do_out     <= data_shift[15];
						data_shift <= data_shift << 1;
					end
					data_bits  <= data_bits + 5'd1;
				end else if (op == 2'b01) begin // WRITE
					data_shift <= {data_shift[14:0], di};
					data_bits  <= data_bits + 5'd1;
					if (data_bits == 5'd15)
						mem[eaddr] <= {data_shift[14:0], di};
				end
				// EWEN/EWDS/ERASE (op==2'b11 with addr-encoded sub-op, or
				// op==2'b00 extended commands): no data phase needed for
				// this core to boot -- accepted but functionally no-ops.
			end
		endcase
	end
end

endmodule
