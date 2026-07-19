#!/bin/sed -f

# ================================================================
# Fix sor_board.sv for 16‑bit ROM writes
# 
# Usage: sed -f fix_sor_board.sed sor_board.sv > sor_board_fixed.sv
# ================================================================

# -----------------------------------------------------------------
# 1. DELETE the original 8-bit checksum block (lines that start with
#    "always @(posedge clk_sys) begin" and contain "wr_chk <= 8'h00"
#    followed by the 8-bit condition)
# -----------------------------------------------------------------
/^always @(posedge clk_sys) begin$/ {
    :checksum_block
    N
    /end$/! b checksum_block
    # If this block contains the 8-bit condition, delete it
    /sdram_wr_ack && !latched_ioctl_slave && !bt_write_active/ {
        d
    }
}

# -----------------------------------------------------------------
# 2. Find the 16-bit checksum block and add latch logic before it
# -----------------------------------------------------------------
/^always @(posedge clk_sys) begin$/ {
    :find_16bit
    N
    /sdram_wr_ack && sdram_wr_we16/ {
        # Insert latch logic before this block
        i\
\
// ----- Latch bytes at write issue time -----\
reg [7:0] latched_byte_even, latched_byte_odd;\
reg [23:0] latched_wr_addr;\
\
always @(posedge clk_sys) begin\
    if (sdram_wr_ack) begin\
        // Clear after ack (optional)\
    end else if (wr_pending && have_even) begin\
        // Latch bytes and address when the write is issued\
        latched_byte_even <= byte_even;\
        latched_byte_odd  <= byte_odd;\
        latched_wr_addr   <= {4'b0, even_addr};\
    end\
end\
\
// ----- 16-bit checksum (single block) -----\
reg  [7:0] wr_chk, wr_chk_even, wr_chk_odd;\
reg  [7:0] wr_chk_q0, wr_chk_q1, wr_chk_q2, wr_chk_q3;\
reg  [7:0] dbg_wr_b0, dbg_wr_b1, dbg_wr_b2, dbg_wr_b3;\
\
always @(posedge clk_sys) begin\
    if (sdram_init) begin\
        wr_chk      <= 8'h00;\
        wr_chk_even <= 8'h00;\
        wr_chk_odd  <= 8'h00;\
        wr_chk_q0   <= 8'h00;\
        wr_chk_q1   <= 8'h00;\
        wr_chk_q2   <= 8'h00;\
        wr_chk_q3   <= 8'h00;\
        dbg_wr_b0   <= 8'h00;\
        dbg_wr_b1   <= 8'h00;\
        dbg_wr_b2   <= 8'h00;\
        dbg_wr_b3   <= 8'h00;\
    end else if (sdram_wr_ack && sdram_wr_we16 && !even_is_slave) begin\
        // Use latched values\
        wr_chk <= wr_chk + latched_byte_even + latched_byte_odd;\
        wr_chk_even <= wr_chk_even + latched_byte_even;\
        wr_chk_odd  <= wr_chk_odd + latched_byte_odd;\
        \
        // Quarter selection: word address bits [16:15] (since word quarter = 32K words)\
        // Byte quarter 0 = word address 0x00000-0x07FFF\
        // Byte quarter 1 = word address 0x08000-0x0FFFF\
        // Byte quarter 2 = word address 0x10000-0x17FFF\
        // Byte quarter 3 = word address 0x18000-0x1FFFF\
        case (latched_wr_addr[16:15])\
            2'd0: wr_chk_q0 <= wr_chk_q0 + latched_byte_even + latched_byte_odd;\
            2'd1: wr_chk_q1 <= wr_chk_q1 + latched_byte_even + latched_byte_odd;\
            2'd2: wr_chk_q2 <= wr_chk_q2 + latched_byte_even + latched_byte_odd;\
            2'd3: wr_chk_q3 <= wr_chk_q3 + latched_byte_even + latched_byte_odd;\
        endcase\
        \
        // Spot checks for bytes 0-3\
        if (latched_wr_addr == 24'd0) begin\
            dbg_wr_b0 <= latched_byte_even;\
            dbg_wr_b1 <= latched_byte_odd;\
        end\
        if (latched_wr_addr == 24'd1) begin\
            dbg_wr_b2 <= latched_byte_even;\
            dbg_wr_b3 <= latched_byte_odd;\
        end\
    end\
end
        # Now delete the old 16-bit block (we've replaced it)
        d
    }
}

# -----------------------------------------------------------------
# 3. Remove any remaining duplicate declarations of wr_chk etc.
#    (they'll be declared in the new block above)
# -----------------------------------------------------------------
/^reg  \[7:0\] wr_chk[,;]/d
/^reg  \[7:0\] wr_chk_even[,;]/d
/^reg  \[7:0\] wr_chk_odd[,;]/d
/^reg  \[7:0\] wr_chk_q0[,;]/d
/^reg  \[7:0\] wr_chk_q1[,;]/d
/^reg  \[7:0\] wr_chk_q2[,;]/d
/^reg  \[7:0\] wr_chk_q3[,;]/d
/^reg  \[7:0\] dbg_wr_b0[,;]/d
/^reg  \[7:0\] dbg_wr_b1[,;]/d
/^reg  \[7:0\] dbg_wr_b2[,;]/d
/^reg  \[7:0\] dbg_wr_b3[,;]/d

# -----------------------------------------------------------------
# 4. Fix the existing latch that uses latched_ioctl_addr
#    (ensure it's consistent with the new logic)
# -----------------------------------------------------------------
/^reg \[23:0\] latched_ioctl_addr;/d

# -----------------------------------------------------------------
# 5. End of script
# -----------------------------------------------------------------
