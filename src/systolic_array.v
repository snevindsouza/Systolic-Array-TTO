//============================================================================
// systolic_array.v -- 4x4 unsigned integer matrix multiply accelerator
//
//   C = A * B      A: 4x4 8-bit unsigned
//                  B: 4x4 8-bit unsigned
//                  C: 4x4 ACCW-bit unsigned (exact, no overflow)
//
// Verilog-2001 only.  Asynchronous active-low reset.  No SystemVerilog
// constructs, no unpacked array ports, no logic/always_ff/always_comb.
//
//----------------------------------------------------------------------------
// ARCHITECTURE
//----------------------------------------------------------------------------
// 16 output-stationary PEs in a 4x4 mesh.  A flows left-to-right, B flows
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
//   left edge of row i at step T    : A[i][T-i]  for i <= T <= i+3, else 0
//   top edge of column j at step T  : B[T-j][j]  for j <= T <= j+3, else 0
//
// With that schedule PE(i,j) sees the operand pair (A[i][k], B[k][j]) with
//
//   k = T - i - j
//
// which is exactly the required inner product.  Out-of-window steps inject
// zeros, so no per-PE valid signal is needed: a zero operand contributes
// nothing to the accumulator.  The last useful step is T = 3+3+3 = 9, so the
// whole computation takes 10 clock cycles.  See timing.md for the full table.
//
//----------------------------------------------------------------------------
// SKEW WITHOUT SKEW REGISTERS
//----------------------------------------------------------------------------
// A and B live in flop arrays and the skewed edge streams are generated
// combinationally from the step counter T_CNT, so no delay-line registers are
// needed.  Because A is stored row-major, the flat address for the row-i edge
// stream reduces to 4*i + (T-i) = 3*i + T, i.e. only four addresses per row
// are ever selected: a 4:1 mux, not a 16:1 mux.  Likewise for B (column-major
// traversal of a row-major store): 4*(T-j) + j.
//
//----------------------------------------------------------------------------
// EXTERNAL PROTOCOL (see interface.md)
//----------------------------------------------------------------------------
//   Load A : hold LOAD_A for exactly 16 consecutive cycles, presenting
//            A00,A01,A02,A03,A10,...,A33 on DATA_IN.  Deassert LOAD_A.
//   Load B : same with LOAD_B, B00,B01,...,B33.
//   Start  : pulse START for one cycle.  DONE rises 11 cycles later.
//   Read   : hold READ for 64 consecutive cycles; DATA_OUT presents
//            C00[7:0],C00[15:8],C00[23:16],C00[31:24],C01[7:0],... and the
//            read pointer post-increments.  Little-endian, 4 bytes per
//            element regardless of ACCW.
//============================================================================

module systolic_array #(
  parameter ACCW = 32        // accumulator width, legal range 18..32
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
  // FSM encoding
  //--------------------------------------------------------------------------
  localparam [2:0] S_IDLE    = 3'd0;
  localparam [2:0] S_LOAD_A  = 3'd1;
  localparam [2:0] S_LOAD_B  = 3'd2;
  localparam [2:0] S_COMPUTE = 3'd3;
  localparam [2:0] S_DONE    = 3'd4;
  localparam [2:0] S_READ    = 3'd5;

  localparam [3:0] T_LAST    = 4'd9;   // last systolic step (3+3+3)

  //--------------------------------------------------------------------------
  // Control state
  //--------------------------------------------------------------------------
  reg  [2:0] STATE;
  reg  [2:0] NEXT_STATE;
  reg  [3:0] A_PTR;      // load pointer for A, 0..15
  reg  [3:0] B_PTR;      // load pointer for B, 0..15
  reg  [3:0] T_CNT;      // systolic step counter, 0..9
  reg  [5:0] RD_PTR;     // readout byte pointer, 0..63

  //--------------------------------------------------------------------------
  // Matrix storage: 16 bytes each, row-major.  A_MEM[4*i+k] = A[i][k],
  // B_MEM[4*k+j] = B[k][j].  C is not stored separately: the 16 PE
  // accumulators *are* the C storage and are muxed out during readout.
  //--------------------------------------------------------------------------
  reg  [7:0] A_MEM [0:15];
  reg  [7:0] B_MEM [0:15];

  integer    I;          // reset loop index (elaboration/reset only)

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
  // captured, so a 16-cycle burst transfers exactly 16 bytes.
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
      A_PTR  <= 4'd0;
      B_PTR  <= 4'd0;
      T_CNT  <= 4'd0;
      RD_PTR <= 6'd0;
      for (I = 0; I < 16; I = I + 1) begin
        A_MEM[I] <= 8'h00;
        B_MEM[I] <= 8'h00;
      end
    end
    else begin
      STATE <= NEXT_STATE;

      // ---- A load pointer and store ----
      if (!LOAD_A)   A_PTR <= 4'd0;          // pointer rearms between bursts
      else if (WR_A) A_PTR <= A_PTR + 4'd1;

      if (WR_A) A_MEM[A_PTR] <= DATA_IN;

      // ---- B load pointer and store ----
      if (!LOAD_B)   B_PTR <= 4'd0;
      else if (WR_B) B_PTR <= B_PTR + 4'd1;

      if (WR_B) B_MEM[B_PTR] <= DATA_IN;

      // ---- systolic step counter ----
      if (CLR_ACC)                  T_CNT <= 4'd0;
      else if (STATE == S_COMPUTE)  T_CNT <= T_CNT + 4'd1;

      // ---- readout pointer: rearmed by every new computation ----
      if (CLR_ACC)                      RD_PTR <= 6'd0;
      else if (NEXT_STATE == S_READ)    RD_PTR <= RD_PTR + 6'd1;
    end
  end

  //--------------------------------------------------------------------------
  // PE mesh interconnect.
  //
  // Flat 1-D net arrays are used instead of multidimensional arrays so that
  // the netlist is legal in the most conservative Verilog dialect.
  //   AH[5*i + j] = A entering PE(i,j);  j = 4 is the row's unused spill.
  //   BV[4*i + j] = B entering PE(i,j);  i = 4 is the column's unused spill.
  //
  // The spill nets AH[5*i+4] and BV[16+j] are intentionally left unread: the
  // A forwarding register of column 3 and the B forwarding register of row 3
  // drive nothing, so synthesis removes them (64 of the 256 forwarding flops).
  // Keeping the mesh uniform is worth more than special-casing the boundary.
  //--------------------------------------------------------------------------
  wire [7:0]           AH [0:19];
  wire [7:0]           BV [0:19];
  wire [16*ACCW-1:0]   ACC_BUS;

  genvar GI, GJ;

  generate
    //----------------------------------------------------------------------
    // Skewed A injection on the left edge.
    // Row i needs A[i][T-i], i.e. A_MEM[3*i + T], valid for i <= T <= i+3.
    // KA wraps to a large value when T < i, which selects the zero default.
    //----------------------------------------------------------------------
    for (GI = 0; GI < 4; GI = GI + 1) begin : A_INJECT
      wire [3:0] KA;
      assign KA = T_CNT - GI;
      assign AH[5*GI] = (KA == 4'd0) ? A_MEM[4*GI + 0] :
                        (KA == 4'd1) ? A_MEM[4*GI + 1] :
                        (KA == 4'd2) ? A_MEM[4*GI + 2] :
                        (KA == 4'd3) ? A_MEM[4*GI + 3] : 8'h00;
    end

    //----------------------------------------------------------------------
    // Skewed B injection on the top edge.
    // Column j needs B[T-j][j], i.e. B_MEM[4*(T-j) + j].
    //----------------------------------------------------------------------
    for (GJ = 0; GJ < 4; GJ = GJ + 1) begin : B_INJECT
      wire [3:0] KB;
      assign KB = T_CNT - GJ;
      assign BV[GJ] = (KB == 4'd0) ? B_MEM[ 0 + GJ] :
                      (KB == 4'd1) ? B_MEM[ 4 + GJ] :
                      (KB == 4'd2) ? B_MEM[ 8 + GJ] :
                      (KB == 4'd3) ? B_MEM[12 + GJ] : 8'h00;
    end

    //----------------------------------------------------------------------
    // The 16 processing elements.
    //----------------------------------------------------------------------
    for (GI = 0; GI < 4; GI = GI + 1) begin : PE_ROW
      for (GJ = 0; GJ < 4; GJ = GJ + 1) begin : PE_COL
        wire [ACCW-1:0] ACC_LOCAL;

        PE #(
          .ACCW    (ACCW)
        ) U_PE (
          .CLK     (CLK),
          .RESET_N (RESET_N),
          .CLR     (CLR_ACC),
          .EN      (EN_MAC),
          .A_IN    (AH[5*GI + GJ]),
          .B_IN    (BV[4*GI + GJ]),
          .A_OUT   (AH[5*GI + GJ + 1]),
          .B_OUT   (BV[4*(GI+1) + GJ]),
          .ACC_OUT (ACC_LOCAL)
        );

        // Flatten the 16 accumulators into one bus for the readout mux.
        assign ACC_BUS[(4*GI + GJ)*ACCW +: ACCW] = ACC_LOCAL;
      end
    end
  endgenerate

  //--------------------------------------------------------------------------
  // Byte-serial result readout.
  //   RD_PTR[5:2] selects the C element (row-major C00..C33)
  //   RD_PTR[1:0] selects the byte within the 32-bit word (little-endian)
  // DATA_OUT is combinational from the registered pointer, so the host sees
  // byte n while asserting READ and the pointer advances on that same edge.
  //--------------------------------------------------------------------------
  wire [3:0]      RD_ELEM;
  wire [1:0]      RD_BYTE;
  wire [ACCW-1:0] ACC_SEL;
  wire [31:0]     ACC_PAD;

  assign RD_ELEM  = RD_PTR[5:2];
  assign RD_BYTE  = RD_PTR[1:0];
  assign ACC_SEL  = ACC_BUS[RD_ELEM*ACCW +: ACCW];
  assign ACC_PAD  = ACC_SEL;                     // zero extend to 32 bits
  assign DATA_OUT = ACC_PAD[RD_BYTE*8 +: 8];

endmodule
