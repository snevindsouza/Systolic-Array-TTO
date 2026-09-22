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

  // ACCW = 32 matches the specified datapath.  Set to 18 for the
  // area-optimised build; the external protocol is unchanged because the
  // readout always presents 4 zero-extended bytes per element.
  localparam ACCW = 32;

  wire [7:0] DATA_OUT;
  wire       BUSY;
  wire       DONE;

  systolic_array #(
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
