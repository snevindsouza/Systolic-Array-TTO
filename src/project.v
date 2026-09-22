// Copyright (c) 2024 Your Name
// SPDX-License-Identifier: Apache-2.0
// 
// tt_um_systolic_mm.v -- Tiny Tapeout wrapper for the 4x4 systolic
//                        matrix-multiply accelerator.
//
// Verilog-2001 only.  Async active-low reset (TT supplies rst_n).
//
// Pin budget actually available on a Tiny Tapeout tile:
//   ui_in  [7:0]  dedicated inputs
//   uo_out [7:0]  dedicated outputs
//   uio    [7:0]  bidirectional, direction chosen per bit by uio_oe
//   clk, rst_n, ena
//
// Assignment (see interface.md):
//   ui_in [7:0]  = DATA_IN      8-bit operand byte stream
//   uo_out[7:0]  = DATA_OUT     8-bit result byte stream
//   uio [0]  in  = LOAD_A
//   uio [1]  in  = LOAD_B
//   uio [2]  in  = START
//   uio [3]  in  = READ
//   uio [4] out  = BUSY
//   uio [5] out  = DONE
//   uio [6] out  = 0 (reserved)
//   uio [7] out  = 0 (reserved)
//
// uio_oe = 8'b1111_0000 : bits 7..4 drive out, bits 3..0 are inputs.
// The direction vector is a constant, so no bus turnaround logic is needed.

`default_nettype none

module tt_um_systolic_mm (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

//--------------------------------------------------------------------------
  // CONFIGURATION -- this is the only place the size is chosen.
  // Whatever is set here MUST match the tile count in info.yaml.
  //
  //   MN = matrix dimension (the problem size)
  //   PN = PE array dimension (the parallelism).  Must divide MN.
  //
  // TWO measured sky130 runs anchor this, both fully parallel (PN = MN = 4):
  //   DW=8 ACCW=32 -> 123 248 um2, 774.219 % on 1x1
  //   DW=4 ACCW=10 ->  37 014 um2, 116.257 % on 1x2
  // area_model.py is fitted to both within 0.8 %, and gives for a 4x4 product
  // in 1x2 tiles (core 31 838 um2):
  //
  //   PN  DW  ACCW  PEs  pass  area um2  cycles  1x2 util
  //    4   4   10    16    1      36 800     79    116 %  <- does not fit
  //    4   3    9    16    1      27 700     79     87 %  tight, operands 0..7
  //    4   2    9    16    1      22 000     79     69 %  operands 0..3, toy
  //    2   4   10     4    4      20 100     96     63 %  <-- selected
  //    2   3    9     4    4      16 500     96     52 %  safe fallback
  //    1   4   10     1   16      14 900    148     47 %  not systolic
  //
  // Keeping 16 parallel multipliers forces DW <= 2, which is not a useful
  // matrix multiplier.  PN = 2 computes the SAME 4x4 product on 4 PEs over 4
  // passes and keeps 4-bit operands, for 1.8x less area.  That trade is nearly
  // free because the design is I/O bound: compute is 29 of 96 cycles either
  // way, and the byte-serial port sets the throughput.
  //
  // If routing struggles at 63 %, set DW = 3 and ACCW = 9 for 52 %.  ACCW must
  // always be the exact width 2*DW + ceil(log2(MN)) -- anything wider is
  // provably dead logic, anything narrower overflows -- floored at 9 so a
  // result element is at least two bytes on the readout port.
  // See area_analysis.md.
  //--------------------------------------------------------------------------
  localparam MN   = 4;       // 4x4 matrix product
  localparam PN   = 2;       // on a 2x2 PE array, 4 passes
  localparam DW   = 4;
  localparam ACCW = 10;      // exact: 4 * 15 * 15 = 900 < 2^10

  wire [7:0] DATA_OUT;
  wire       BUSY;
  wire       DONE;

  systolic_array #(
    .MN       (MN),
    .PN       (PN),
    .DW       (DW),
    .ACCW     (ACCW)
  ) U_MM (
    .CLK      (clk),
    .RESET_N  (rst_n),
    .DATA_IN  (ui_in),
    .LOAD_A   (uio_in[0]),
    .LOAD_B   (uio_in[1]),
    .START    (uio_in[2]),
    .READ     (uio_in[3]),
    .DATA_OUT (DATA_OUT),
    .BUSY     (BUSY),
    .DONE     (DONE)
  );

  assign uo_out  = DATA_OUT;
  assign uio_out = {2'b00, DONE, BUSY, 4'b0000};
  assign uio_oe  = 8'b1111_0000;

  // ena is documented by Tiny Tapeout as always 1 for the selected design;
  // tie it off so lint does not flag an unused input.
  wire _UNUSED = ena;

endmodule
