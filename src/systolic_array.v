//============================================================================
// systolic_array.v -- N x N unsigned integer matrix multiply accelerator
//
//   C = A * B      A: N x N, DW-bit unsigned
//                  B: N x N, DW-bit unsigned
//                  C: N x N, ACCW-bit unsigned (exact, no overflow)
//
// Verilog-2001 only.  Asynchronous active-low reset.  No SystemVerilog
// constructs, no unpacked array ports, no logic/always_ff/always_comb.
//
//----------------------------------------------------------------------------
// CONFIGURATION
//----------------------------------------------------------------------------
// N, DW and ACCW set the area.  Cell area scales roughly as
//
//   N^2 * (56 * DW^2  +  59 * ACCW)  +  26 * flops
//
// so the multiplier term dominates and is quadratic in *both* N and DW.  See
// area_model.py, which is calibrated against a measured sky130 synthesis run,
// and area_analysis.md for the tile budget of each configuration.
//
// ACCW must be at least 2*DW + ceil(log2(N)) for the accumulation to be exact;
// ACC_EXACT below computes that value, and ACCW is checked against it at
// elaboration.  ACCW must also be at least 9 so that a result element occupies
// at least two bytes on the readout port.
//
//----------------------------------------------------------------------------
// ARCHITECTURE
//----------------------------------------------------------------------------
// N*N output-stationary PEs in a mesh.  A flows left-to-right, B flows
// top-to-bottom, PE(i,j) owns the accumulator for C[i][j].
//
//        B col0   B col1   B col2   B col3
//           |        |        |        |
//  Arow0 -> PE00 --> PE01 --> PE02 --> PE03
//           |        |        |        |
//  Arow1 -> PE10 --> PE11 --> PE12 --> PE13
//           |        |        |        |
//  Arow2 -> PE20 --> PE21 --> PE22 --> PE23
//           |        |        |        |
//  Arow3 -> PE30 --> PE31 --> PE32 --> PE33
//
// Each hop costs one clock cycle, so an operand injected at the left edge of
// row i reaches PE(i,j) j cycles later, and an operand injected at the top of
// column j reaches PE(i,j) i cycles later.  The injection is therefore skewed
// in time:
//
//   left edge of row i at step T    : A[i][T-i]  for i <= T <= i+N-1, else 0
//   top edge of column j at step T  : B[T-j][j]  for j <= T <= j+N-1, else 0
//
// With that schedule PE(i,j) sees the operand pair (A[i][k], B[k][j]) with
//
//   k = T - i - j
//
// which is exactly the required inner product.  Out-of-window steps inject
// zeros, so no per-PE valid signal is needed: a zero operand contributes
// nothing to the accumulator.  The last useful step is T = 3*(N-1), so the
// computation takes 3*N-2 clock cycles.  See timing.md for the full table.
//
//----------------------------------------------------------------------------
// SKEW WITHOUT SKEW REGISTERS
//----------------------------------------------------------------------------
// A and B live in flop arrays and the skewed edge streams are generated
// combinationally from the step counter, so no delay-line registers are
// needed.  For row i the candidate operands are exactly the N words of row i
// of A, selected by k = T-i, so each edge needs an N:1 mux rather than an
// N*N:1 mux.  Likewise each B column selects among the N words of column j.
//
//----------------------------------------------------------------------------
// EXTERNAL PROTOCOL (see interface.md)
//----------------------------------------------------------------------------
//   Load A : hold LOAD_A for exactly N*N consecutive cycles, presenting
//            A00,A01,...  row-major on DATA_IN[DW-1:0].  Deassert LOAD_A.
//   Load B : same with LOAD_B, B00,B01,...
//   Start  : pulse START for one cycle.  DONE rises 3*N-1 cycles later.
//   Read   : hold READ for N*N*OBYTES consecutive cycles; DATA_OUT presents
//            each C element little-endian, OBYTES bytes per element, and the
//            read pointer post-increments.
//============================================================================

module systolic_array #(
  parameter N    = 4,        // matrix / array dimension
  parameter DW   = 8,        // operand width, unsigned
  parameter ACCW = 32        // accumulator width, >= 2*DW+clog2(N) and >= 9
) (
  input  wire       CLK,
  input  wire       RESET_N,   // asynchronous, active low
  input  wire [7:0] DATA_IN,
  input  wire       LOAD_A,
  input  wire       LOAD_B,
  input  wire       START,
  input  wire       READ,
  output wire [7:0] DATA_OUT,
  output wire       BUSY,
  output wire       DONE
);

  //--------------------------------------------------------------------------
  // Verilog-2001 constant function (there is no $clog2 before Verilog-2005).
  //--------------------------------------------------------------------------
  // Explicitly sized rather than "function integer" so that every tool in the
  // chain accepts it.  VALUE must be >= 1, which holds for all uses below.
  function [31:0] CLOG2;
    input [31:0] VALUE;
    reg [31:0] V;
    begin
      V = VALUE - 32'd1;
      for (CLOG2 = 32'd0; V > 32'd0; CLOG2 = CLOG2 + 32'd1)
        V = V >> 1;
    end
  endfunction

  //--------------------------------------------------------------------------
  // Derived sizes
  //--------------------------------------------------------------------------
  localparam NN        = N * N;                  // elements per matrix
  localparam PTRW      = CLOG2(NN);              // load pointer width
  localparam T_LAST    = 3 * (N - 1);            // last systolic step
  localparam TW        = CLOG2(3 * N - 1);       // step counter width
  localparam OB_RAW    = 1 << CLOG2((ACCW + 7) / 8);
  localparam OBYTES    = (OB_RAW < 2) ? 2 : OB_RAW;   // power-of-2 bytes/elem
  localparam BSEL_W    = CLOG2(OBYTES);          // byte select width, >= 1
  localparam RDW       = CLOG2(NN * OBYTES);     // readout pointer width
  localparam OW        = OBYTES * 8;             // readout word width
  localparam ACC_EXACT = 2 * DW + CLOG2(N);      // minimum exact accumulator

  // Configuration checks.  Verilog-2001 has no elaboration-time assertion,
  // so these are reported at time 0 in simulation and ignored by synthesis.
`ifndef SYNTHESIS
  initial begin
    if (ACCW < ACC_EXACT) begin
      $display("FATAL systolic_array: ACCW=%0d is smaller than the exact width %0d for N=%0d DW=%0d",
               ACCW, ACC_EXACT, N, DW);
      $finish;
    end
    if (ACCW < 9 || ACCW > OW) begin
      $display("FATAL systolic_array: ACCW=%0d must be in 9..%0d", ACCW, OW);
      $finish;
    end
    if (DW < 2 || DW > 8) begin
      $display("FATAL systolic_array: DW=%0d must be in 2..8 (DATA_IN is one byte)", DW);
      $finish;
    end
    if (N < 2) begin
      $display("FATAL systolic_array: N=%0d must be at least 2", N);
      $finish;
    end
  end
`endif

  //--------------------------------------------------------------------------
  // FSM encoding
  //--------------------------------------------------------------------------
  localparam [2:0] S_IDLE    = 3'd0;
  localparam [2:0] S_LOAD_A  = 3'd1;
  localparam [2:0] S_LOAD_B  = 3'd2;
  localparam [2:0] S_COMPUTE = 3'd3;
  localparam [2:0] S_DONE    = 3'd4;
  localparam [2:0] S_READ    = 3'd5;

  //--------------------------------------------------------------------------
  // Control state
  //--------------------------------------------------------------------------
  reg  [2:0]      STATE;
  reg  [2:0]      NEXT_STATE;
  reg  [PTRW-1:0] A_PTR;      // load pointer for A
  reg  [PTRW-1:0] B_PTR;      // load pointer for B
  reg  [TW-1:0]   T_CNT;      // systolic step counter
  reg  [RDW-1:0]  RD_PTR;     // readout byte pointer

  //--------------------------------------------------------------------------
  // Matrix storage: NN words each, row-major.  A_MEM[N*i+k] = A[i][k],
  // B_MEM[N*k+j] = B[k][j].  C is not stored separately: the NN PE
  // accumulators *are* the C storage and are muxed out during readout.
  //--------------------------------------------------------------------------
  reg  [DW-1:0] A_MEM [0:NN-1];
  reg  [DW-1:0] B_MEM [0:NN-1];

  integer       I;            // reset loop index

  //--------------------------------------------------------------------------
  // Control decode (combinational)
  //--------------------------------------------------------------------------
  wire WR_A;
  wire WR_B;
  wire CLR_ACC;
  wire EN_MAC;

  always @(*) begin
    NEXT_STATE = STATE;
    case (STATE)
      S_IDLE: begin
        if      (LOAD_A) NEXT_STATE = S_LOAD_A;
        else if (LOAD_B) NEXT_STATE = S_LOAD_B;
        else if (START)  NEXT_STATE = S_COMPUTE;
      end
      S_LOAD_A: begin
        if (!LOAD_A) NEXT_STATE = S_IDLE;
      end
      S_LOAD_B: begin
        if (!LOAD_B) NEXT_STATE = S_IDLE;
      end
      S_COMPUTE: begin
        if (T_CNT == T_LAST) NEXT_STATE = S_DONE;
      end
      S_DONE: begin
        if      (LOAD_A) NEXT_STATE = S_LOAD_A;
        else if (LOAD_B) NEXT_STATE = S_LOAD_B;
        else if (START)  NEXT_STATE = S_COMPUTE;
        else if (READ)   NEXT_STATE = S_READ;
      end
      S_READ: begin
        if (!READ) NEXT_STATE = S_DONE;
      end
      default: NEXT_STATE = S_IDLE;
    endcase
  end

  // A write happens on every edge at which the machine is in (or entering)
  // the matching load state.  Using NEXT_STATE means the very first byte of a
  // burst -- presented in the same cycle in which LOAD_A first goes high -- is
  // captured, so an NN-cycle burst transfers exactly NN operands.
  assign WR_A    = (NEXT_STATE == S_LOAD_A);
  assign WR_B    = (NEXT_STATE == S_LOAD_B);

  // One-cycle clear pulse immediately before the first systolic step.
  assign CLR_ACC = (NEXT_STATE == S_COMPUTE) && (STATE != S_COMPUTE);
  assign EN_MAC  = (STATE == S_COMPUTE);

  assign BUSY    = (STATE == S_COMPUTE);
  assign DONE    = (STATE == S_DONE) || (STATE == S_READ);

  //--------------------------------------------------------------------------
  // Sequential control / storage.  Nonblocking assignments only.
  //--------------------------------------------------------------------------
  always @(posedge CLK or negedge RESET_N) begin
    if (!RESET_N) begin
      STATE  <= S_IDLE;
      A_PTR  <= {PTRW{1'b0}};
      B_PTR  <= {PTRW{1'b0}};
      T_CNT  <= {TW{1'b0}};
      RD_PTR <= {RDW{1'b0}};
      for (I = 0; I < NN; I = I + 1) begin
        A_MEM[I] <= {DW{1'b0}};
        B_MEM[I] <= {DW{1'b0}};
      end
    end
    else begin
      STATE <= NEXT_STATE;

      // ---- A load pointer and store ----
      if (!LOAD_A)   A_PTR <= {PTRW{1'b0}};   // pointer rearms between bursts
      else if (WR_A) A_PTR <= A_PTR + 1'b1;

      if (WR_A) A_MEM[A_PTR] <= DATA_IN[DW-1:0];

      // ---- B load pointer and store ----
      if (!LOAD_B)   B_PTR <= {PTRW{1'b0}};
      else if (WR_B) B_PTR <= B_PTR + 1'b1;

      if (WR_B) B_MEM[B_PTR] <= DATA_IN[DW-1:0];

      // ---- systolic step counter ----
      if (CLR_ACC)                  T_CNT <= {TW{1'b0}};
      else if (STATE == S_COMPUTE)  T_CNT <= T_CNT + 1'b1;

      // ---- readout pointer: rearmed by every new computation ----
      // The wrap is explicit rather than relying on the counter overflowing,
      // because NN*OBYTES is only a power of two when N is.  Without this the
      // pointer would run past the last element for e.g. N = 3, addressing a
      // non-existent accumulator and breaking the re-read guarantee.
      if (CLR_ACC)
        RD_PTR <= {RDW{1'b0}};
      else if (NEXT_STATE == S_READ)
        RD_PTR <= (RD_PTR == (NN*OBYTES - 1)) ? {RDW{1'b0}} : RD_PTR + 1'b1;
    end
  end

  //--------------------------------------------------------------------------
  // PE mesh interconnect.
  //
  // Flat 1-D net arrays are used instead of multidimensional arrays so that
  // the netlist is legal in the most conservative Verilog dialect.
  //   AH[(N+1)*i + j] = A entering PE(i,j);  j = N is the row's unused spill.
  //   BV[N*i + j]     = B entering PE(i,j);  i = N is the column's spill.
  //
  // The spill nets are intentionally left unread: the A forwarding register of
  // the last column and the B forwarding register of the last row drive
  // nothing, so synthesis removes them (2*N*DW flops).  Keeping the mesh
  // uniform is worth more than special-casing the boundary.
  //--------------------------------------------------------------------------
  wire [DW-1:0]      AH [0:N*(N+1)-1];
  wire [DW-1:0]      BV [0:N*(N+1)-1];
  wire [NN*ACCW-1:0] ACC_BUS;

  genvar GI, GJ, GK;

  generate
    //----------------------------------------------------------------------
    // Skewed A injection on the left edge.
    // Row i needs A[i][T-i].  The candidates are the N words of row i, so
    // pack them into a bus and select with k = T-i.  KA wraps to a value
    // >= N when T < i, which selects the zero default.
    //----------------------------------------------------------------------
    for (GI = 0; GI < N; GI = GI + 1) begin : A_INJECT
      wire [N*DW-1:0] ROWBUS;
      wire [TW-1:0]   KA;
      wire [TW-1:0]   KA_SEL;

      for (GK = 0; GK < N; GK = GK + 1) begin : PACK
        assign ROWBUS[GK*DW +: DW] = A_MEM[N*GI + GK];
      end

      assign KA     = T_CNT - GI;
      assign KA_SEL = (KA < N) ? KA : {TW{1'b0}};   // keep the select in range
      assign AH[(N+1)*GI] = (KA < N) ? ROWBUS[KA_SEL*DW +: DW] : {DW{1'b0}};
    end

    //----------------------------------------------------------------------
    // Skewed B injection on the top edge.
    // Column j needs B[T-j][j].  The candidates are the N words of column j,
    // i.e. B_MEM[N*k + j] for k = 0..N-1.
    //----------------------------------------------------------------------
    for (GJ = 0; GJ < N; GJ = GJ + 1) begin : B_INJECT
      wire [N*DW-1:0] COLBUS;
      wire [TW-1:0]   KB;
      wire [TW-1:0]   KB_SEL;

      for (GK = 0; GK < N; GK = GK + 1) begin : PACK
        assign COLBUS[GK*DW +: DW] = B_MEM[N*GK + GJ];
      end

      assign KB     = T_CNT - GJ;
      assign KB_SEL = (KB < N) ? KB : {TW{1'b0}};
      assign BV[GJ] = (KB < N) ? COLBUS[KB_SEL*DW +: DW] : {DW{1'b0}};
    end

    //----------------------------------------------------------------------
    // The N*N processing elements.
    //----------------------------------------------------------------------
    for (GI = 0; GI < N; GI = GI + 1) begin : PE_ROW
      for (GJ = 0; GJ < N; GJ = GJ + 1) begin : PE_COL
        wire [ACCW-1:0] ACC_LOCAL;

        PE #(
          .DW      (DW),
          .ACCW    (ACCW)
        ) U_PE (
          .CLK     (CLK),
          .RESET_N (RESET_N),
          .CLR     (CLR_ACC),
          .EN      (EN_MAC),
          .A_IN    (AH[(N+1)*GI + GJ]),
          .B_IN    (BV[N*GI + GJ]),
          .A_OUT   (AH[(N+1)*GI + GJ + 1]),
          .B_OUT   (BV[N*(GI+1) + GJ]),
          .ACC_OUT (ACC_LOCAL)
        );

        // Flatten the accumulators into one bus for the readout mux.
        assign ACC_BUS[(N*GI + GJ)*ACCW +: ACCW] = ACC_LOCAL;
      end
    end
  endgenerate

  //--------------------------------------------------------------------------
  // Byte-serial result readout.
  //   RD_PTR[RDW-1:BSEL_W] selects the C element (row-major C00..)
  //   RD_PTR[BSEL_W-1:0]   selects the byte within the element (little-endian)
  // DATA_OUT is combinational from the registered pointer, so the host sees
  // byte n while asserting READ and the pointer advances on that same edge.
  //--------------------------------------------------------------------------
  wire [PTRW-1:0]   RD_ELEM;
  wire [BSEL_W-1:0] RD_BYTE;
  wire [ACCW-1:0]   ACC_SEL;
  wire [OW-1:0]     ACC_PAD;

  assign RD_ELEM  = RD_PTR[RDW-1:BSEL_W];
  assign RD_BYTE  = RD_PTR[BSEL_W-1:0];
  assign ACC_SEL  = ACC_BUS[RD_ELEM*ACCW +: ACCW];
  assign ACC_PAD  = ACC_SEL;                     // zero extend to OBYTES*8
  assign DATA_OUT = ACC_PAD[RD_BYTE*8 +: 8];

endmodule
