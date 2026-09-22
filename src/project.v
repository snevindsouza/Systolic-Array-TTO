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
  // A measured sky130 synthesis run of N=4 DW=8 ACCW=32 came out at
  // 123 248 um2 = 774 % utilisation on a 1x1 tile.  area_model.py is calibrated
  // against that run (-0.4 % error) and gives, for a 1x1/1x2 budget:
  //
  //   N  DW  ACCW  PEs  area um2  cycles  1x1    1x2    note
  //   2   2    9     4      6 250     24   39 %   20 %  operands 0..3, toy
  //   2   3    9     4      7 700     24   49 %   24 %  smallest 1x1 candidate
  //   2   4    9     4      9 700     24   61 %   30 %  <-- selected
  //   2   5   11     4     12 800     24   80 %   40 %
  //   2   6   13     4     16 400     24  103 %   52 %
  //   3   3    9     9     16 700     47  105 %   52 %  9 PEs, 3x3 mesh
  //   3   4   10     9     22 000     47  138 %   69 %
  //   4   4   10    16     38 700     78  243 %  122 %  full 4x4, needs 3x2
  //   4   8   32    16    122 800    110  774 %  387 %  as originally submitted
  //
  // Selected N=2 DW=4 ACCW=9: 30 % on 1x2, which survives even a 1.8x model
  // error.  The same configuration is 61 % on a 1x1 tile, so 1x1 is worth
  // trying -- change info.yaml to tiles: "1x1" and nothing here needs to move.
  // Note the multiplier coefficient was fitted at DW=8, and small multipliers
  // carry proportionally more overhead than DW^2 predicts, so treat the 4-bit
  // areas as a floor.  See area_analysis.md.
  //
  // ACCW must be the exact width 2*DW + ceil(log2(N)), floored at 9 so that a
  // result element is at least two bytes on the readout port.  Anything wider
  // is provably dead logic; anything narrower overflows.
  //--------------------------------------------------------------------------
  localparam N    = 2;
  localparam DW   = 4;
  localparam ACCW = 9;       // exact: 2 * 15 * 15 = 450 < 2^9

  wire [7:0] DATA_OUT;
  wire       BUSY;
  wire       DONE;

  systolic_array #(
    .N        (N),
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
