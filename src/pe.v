//============================================================================
// pe.v -- Processing element for a 4x4 output-stationary systolic array
//
// Verilog-2001 only. Asynchronous active-low reset.
//
// The PE is output-stationary: the accumulator for one C element lives here
// permanently, while the A and B operands stream through it.
//
//   A_IN  ---> [ MAC ] ---> A_OUT   (A travels to the right)
//   B_IN  ---> [ MAC ] ---> B_OUT   (B travels downward)
//
// On every enabled clock edge the PE performs
//
//   ACC <= ACC + A_IN * B_IN
//   A_OUT <= A_IN
//   B_OUT <= B_IN
//
// Note that the MAC consumes the *incoming* (combinational) operands in the
// same cycle in which it registers them for forwarding.  A value therefore
// takes exactly one cycle per hop, which is what the skewed injection
// schedule in systolic_array.v assumes.
//
// Datapath  : PROD, ACC_REG, A_REG, B_REG
// Control   : CLR (synchronous accumulator/pipeline clear), EN (MAC enable)
//
// ACCW notes:
//   * ACCW must be in the range 18..32.
//   * 18 bits is the provably exact width for 4x4 8-bit unsigned inputs:
//     max C element = 4 * 255 * 255 = 260_100 < 2**18 = 262_144, so the
//     accumulator can never overflow.  ACCW = 32 is the default because the
//     project specification asks for a 32-bit accumulator; the upper 14 bits
//     are always zero and synthesis will not remove them (they are visible on
//     ACC_OUT), so ACCW = 18 is the recommended area optimisation.
//============================================================================

module PE #(
  parameter ACCW = 32
) (
  input  wire             CLK,
  input  wire             RESET_N,   // asynchronous, active low
  input  wire             CLR,       // synchronous clear of ACC and pipeline
  input  wire             EN,        // MAC / forward enable
  input  wire [7:0]       A_IN,
  input  wire [7:0]       B_IN,
  output wire [7:0]       A_OUT,
  output wire [7:0]       B_OUT,
  output wire [ACCW-1:0]  ACC_OUT
);

  // --------------------------------------------------------------------
  // Datapath registers
  // --------------------------------------------------------------------
  reg  [7:0]      A_REG;
  reg  [7:0]      B_REG;
  reg  [ACCW-1:0] ACC_REG;

  // --------------------------------------------------------------------
  // Unsigned 8 x 8 -> 16 multiply.
  // Both operands are unsigned nets, and the product is assigned to an
  // explicitly 16-bit net, so the multiply is evaluated at 16 bits.  The
  // maximum product 255*255 = 65_025 fits in 16 bits, so there is no
  // truncation and no sign extension anywhere in this expression.
  // --------------------------------------------------------------------
  wire [15:0]     PROD;
  wire [ACCW-1:0] PROD_EXT;

  assign PROD     = A_IN * B_IN;
  assign PROD_EXT = PROD;            // zero extension to ACCW (ACCW >= 18)

  // --------------------------------------------------------------------
  // Sequential MAC.  Nonblocking assignments only.
  // --------------------------------------------------------------------
  always @(posedge CLK or negedge RESET_N) begin
    if (!RESET_N) begin
      A_REG   <= 8'h00;
      B_REG   <= 8'h00;
      ACC_REG <= {ACCW{1'b0}};
    end
    else if (CLR) begin
      // Clearing the forwarding registers as well as the accumulator is
      // mandatory: at compute step T = 0 the interior PEs must see a zero
      // operand rather than a stale value left over from a previous run.
      A_REG   <= 8'h00;
      B_REG   <= 8'h00;
      ACC_REG <= {ACCW{1'b0}};
    end
    else if (EN) begin
      A_REG   <= A_IN;
      B_REG   <= B_IN;
      ACC_REG <= ACC_REG + PROD_EXT;
    end
  end

  assign A_OUT   = A_REG;
  assign B_OUT   = B_REG;
  assign ACC_OUT = ACC_REG;

endmodule
