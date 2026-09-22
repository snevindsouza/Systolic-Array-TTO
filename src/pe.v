//============================================================================
// pe.v -- Processing element for an N x N output-stationary systolic array
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
// The MAC consumes the *incoming* (combinational) operands in the same cycle
// in which it registers them for forwarding.  A value therefore takes exactly
// one cycle per hop, which is what the skewed injection schedule in
// systolic_array.v assumes.  Do not change this to ACC <= ACC + A_REG * B_REG:
// that would make each hop cost two cycles for the operand but one for the
// forwarded copy, and the derived skew would no longer line up the k indices.
//
// Datapath  : PROD, ACC_REG, A_REG, B_REG
// Control   : CLR (synchronous accumulator/pipeline clear), EN (MAC enable)
//
// Parameters
//   DW    operand width (unsigned).  Product width is exactly 2*DW.
//   ACCW  accumulator width.  Must be >= 2*DW + ceil(log2(N)) for the
//         accumulation to be exact; see systolic_array.v, which computes the
//         exact value and passes it down.
//============================================================================

module PE #(
  parameter DW   = 8,
  parameter ACCW = 32
) (
  input  wire             CLK,
  input  wire             RESET_N,   // asynchronous, active low
  input  wire             CLR,       // synchronous clear of ACC and pipeline
  input  wire             EN,        // MAC / forward enable
  input  wire [DW-1:0]    A_IN,
  input  wire [DW-1:0]    B_IN,
  output wire [DW-1:0]    A_OUT,
  output wire [DW-1:0]    B_OUT,
  output wire [ACCW-1:0]  ACC_OUT
);

  localparam PW = 2 * DW;            // exact product width

  // --------------------------------------------------------------------
  // Datapath registers
  // --------------------------------------------------------------------
  reg  [DW-1:0]   A_REG;
  reg  [DW-1:0]   B_REG;
  reg  [ACCW-1:0] ACC_REG;

  // --------------------------------------------------------------------
  // Unsigned DW x DW -> 2*DW multiply.
  // Both operands are unsigned nets and the product is assigned to an
  // explicitly 2*DW-bit net, so the multiply is evaluated at 2*DW bits.  The
  // maximum product (2^DW - 1)^2 fits in 2*DW bits, so there is no truncation
  // and no sign extension anywhere in this expression.
  // --------------------------------------------------------------------
  wire [PW-1:0]   PROD;
  wire [ACCW-1:0] PROD_EXT;

  assign PROD     = A_IN * B_IN;
  assign PROD_EXT = PROD;            // zero extension to ACCW (ACCW >= PW)

  // --------------------------------------------------------------------
  // Sequential MAC.  Nonblocking assignments only.
  // --------------------------------------------------------------------
  always @(posedge CLK or negedge RESET_N) begin
    if (!RESET_N) begin
      A_REG   <= {DW{1'b0}};
      B_REG   <= {DW{1'b0}};
      ACC_REG <= {ACCW{1'b0}};
    end
    else if (CLR) begin
      // Clearing the forwarding registers as well as the accumulator is
      // mandatory: at compute step T = 0 the interior PEs must see a zero
      // operand rather than a stale value left over from a previous run.
      A_REG   <= {DW{1'b0}};
      B_REG   <= {DW{1'b0}};
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
