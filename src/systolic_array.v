//============================================================================
// systolic_array.v -- MN x MN unsigned integer matrix multiply accelerator,
//                     computed on a PN x PN output-stationary systolic array.
//
//   C = A * B      A: MN x MN, DW-bit unsigned
//                  B: MN x MN, DW-bit unsigned
//                  C: MN x MN, ACCW-bit unsigned (exact, no overflow)
//
// CONFIGURATION
//   MN   matrix dimension -- the problem size
//   PN   PE array dimension -- the amount of parallelism.  Must divide MN.
//   DW   operand width
//   ACCW accumulator width, must be exactly 2*DW + ceil(log2(MN))
//
// PN == MN is the fully parallel array: MN*MN PEs, one pass, lowest latency.
// PN <  MN time-multiplexes a PN x PN array over (MN/PN)^2 passes.  This is
// the lever that matters for area, because the multiplier count is PN^2 while
// the problem size stays MN.
//
// Cell area scales roughly as
//
//   PN^2 * (37.2*DW^2.194 + 59.4*ACCW)  +  26.3 * flops  +  muxes
//
// See area_model.py, which is calibrated against two measured sky130 runs to
// within 0.8 %, and area_analysis.md for the tile budget of each setting.
//
// ARCHITECTURE
// PN*PN output-stationary PEs in a mesh.  A flows left-to-right, B flows
// top-to-bottom, and during pass (pi,pj) the PE at mesh position (i,j) owns
// the accumulator for C[PN*pi+i][PN*pj+j].
//
//        B col0   B col1
//           |        |
//  Arow0 -> PE00 --> PE01
//           |        |
//  Arow1 -> PE10 --> PE11
//
// Each hop costs one clock cycle, so an operand injected at the left edge of
// row i reaches PE(i,j) j cycles later, and one injected at the top of column
// j reaches PE(i,j) i cycles later.  The injection is therefore skewed:
//
//   left edge of row i, step T   : A[PN*pi+i][T-i]  for 0 <= T-i <= MN-1
//   top edge of col j,  step T   : B[T-j][PN*pj+j]  for 0 <= T-j <= MN-1
//
// With that schedule PE(i,j) sees the operand pair
//
//   ( A[PN*pi+i][k] , B[k][PN*pj+j] )   with   k = T - i - j
//
// which is exactly the required inner product.  Out-of-window steps inject
// zeros, so no per-PE valid signal is needed: a zero operand contributes
// nothing.  The last useful step is
//
//   T_LAST = MN - 1 + 2*(PN - 1)
//
// so each pass takes T_LAST+1 systolic steps plus one writeback cycle.
// See timing.md for the full tables.
//
// WHY A C BUFFER IS NEEDED WHEN PN < MN
// Only PN*PN accumulators exist, but the result has MN*MN elements, so each
// pass must copy its block out before the accumulators are cleared for the
// next one.  That is the S_WB state and the CBUF register file.  When
// PN == MN there is a single pass, the accumulators *are* the result storage,
// and the buffer is omitted entirely by a generate-if.
//
// SKEW WITHOUT SKEW REGISTERS
// A and B live in flop arrays and the skewed edge streams are generated
// combinationally from the step and pass counters, so no delay-line registers
// are needed.  Each edge selects one of BLK*MN stored words -- MN choices of k
// times BLK choices of block -- rather than one of MN*MN, so the mux is
// (BLK*MN):1 instead of (MN*MN):1.
//
// EXTERNAL PROTOCOL (see interface.md)
//   Load A : hold LOAD_A for exactly MN*MN consecutive cycles, presenting
//            A00,A01,... row-major on DATA_IN[DW-1:0].  Deassert LOAD_A.
//   Load B : same with LOAD_B.
//   Start  : pulse START for one cycle.  DONE rises
//            1 + NPASS*(T_LAST+2) cycles later.
//   Read   : hold READ for MN*MN*OBYTES consecutive cycles; DATA_OUT presents
//            each C element little-endian, OBYTES bytes per element.
//============================================================================

module systolic_array #(
  parameter MN   = 4,        // matrix dimension (problem size)
  parameter PN   = 2,        // PE array dimension, must divide MN
  parameter DW   = 4,        // operand width, unsigned
  parameter ACCW = 10        // accumulator width, = 2*DW+clog2(MN), >= 9
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

  // Derived sizes
  localparam MNN       = MN * MN;                // elements per matrix
  localparam PNN       = PN * PN;                // processing elements
  localparam BLK       = MN / PN;                // blocks per dimension
  localparam NPASS     = BLK * BLK;              // passes per computation
  localparam T_LAST    = MN - 1 + 2 * (PN - 1);  // last systolic step of a pass
  localparam TW        = $clog2(T_LAST + 2);     // step counter width
  localparam PTRW      = $clog2(MNN);            // load / element pointer width
  localparam BW        = (BLK < 2) ? 1 : $clog2(BLK);  // pass counter width
  localparam SELW      = $clog2(BLK * MN);       // injection select width
  localparam OB_RAW    = 1 << $clog2((ACCW + 7) / 8);
  localparam OBYTES    = (OB_RAW < 2) ? 2 : OB_RAW;   // power-of-2 bytes/elem
  localparam BSEL_W    = $clog2(OBYTES);         // byte select width, >= 1
  localparam RDW       = $clog2(MNN * OBYTES);   // readout pointer width
  localparam OW        = OBYTES * 8;             // readout word width
  localparam ACC_EXACT = 2 * DW + $clog2(MN);    // minimum exact accumulator

  // FSM encoding
  localparam [2:0] S_IDLE    = 3'd0;
  localparam [2:0] S_LOAD_A  = 3'd1;
  localparam [2:0] S_LOAD_B  = 3'd2;
  localparam [2:0] S_COMPUTE = 3'd3;
  localparam [2:0] S_WB      = 3'd4;   // copy this pass's block into CBUF
  localparam [2:0] S_DONE    = 3'd5;
  localparam [2:0] S_READ    = 3'd6;

  // Control state
  reg  [2:0]      STATE;
  reg  [2:0]      NEXT_STATE;
  reg  [PTRW-1:0] A_PTR;      // load pointer for A
  reg  [PTRW-1:0] B_PTR;      // load pointer for B
  reg  [TW-1:0]   T_CNT;      // systolic step counter, within a pass
  reg  [BW-1:0]   PASS_I;     // row block index
  reg  [BW-1:0]   PASS_J;     // column block index
  reg  [RDW-1:0]  RD_PTR;     // readout byte pointer

  // Matrix storage: MNN words each, row-major.  A_MEM[MN*i+k] = A[i][k],
  // B_MEM[MN*k+j] = B[k][j].
  reg  [DW-1:0] A_MEM [0:MNN-1];
  reg  [DW-1:0] B_MEM [0:MNN-1];

  integer       I;            // reset loop index for this always block only

  // Control decode (combinational)
  wire WR_A;
  wire WR_B;
  wire CLR_ACC;
  wire EN_MAC;
  wire WB_NOW;
  wire START_RUN;
  wire LAST_PASS;

  assign LAST_PASS = (PASS_I == (BLK - 1)) && (PASS_J == (BLK - 1));

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
        if (T_CNT == T_LAST) NEXT_STATE = S_WB;
      end
      S_WB: begin
        // one cycle to copy the block out, then either the next pass or done
        if (LAST_PASS) NEXT_STATE = S_DONE;
        else           NEXT_STATE = S_COMPUTE;
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

  // A write happens on every edge at which the machine is in (or entering) the
  // matching load state.  Using NEXT_STATE means the first byte of a burst --
  // presented in the same cycle LOAD_A first goes high -- is captured, so an
  // MNN-cycle burst transfers exactly MNN operands.
  assign WR_A      = (NEXT_STATE == S_LOAD_A);
  assign WR_B      = (NEXT_STATE == S_LOAD_B);

  // Clear pulse before the first step of *every* pass, including the first.
  assign CLR_ACC   = (NEXT_STATE == S_COMPUTE) && (STATE != S_COMPUTE);
  assign EN_MAC    = (STATE == S_COMPUTE);
  assign WB_NOW    = (STATE == S_WB);

  // True only when a whole new computation begins, not between passes.
  assign START_RUN = (NEXT_STATE == S_COMPUTE) &&
                     ((STATE == S_IDLE) || (STATE == S_DONE));

  assign BUSY      = (STATE == S_COMPUTE) || (STATE == S_WB);
  assign DONE      = (STATE == S_DONE) || (STATE == S_READ);

  // Sequential control / storage.  Nonblocking assignments only.
  always @(posedge CLK or negedge RESET_N) begin
    if (!RESET_N) begin
      STATE  <= S_IDLE;
      A_PTR  <= {PTRW{1'b0}};
      B_PTR  <= {PTRW{1'b0}};
      T_CNT  <= {TW{1'b0}};
      PASS_I <= {BW{1'b0}};
      PASS_J <= {BW{1'b0}};
      RD_PTR <= {RDW{1'b0}};
      for (I = 0; I < MNN; I = I + 1) begin
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

      // ---- systolic step counter, restarted for every pass ----
      if (CLR_ACC)                 T_CNT <= {TW{1'b0}};
      else if (STATE == S_COMPUTE) T_CNT <= T_CNT + 1'b1;

      // ---- pass counters: zeroed per computation, advanced per writeback ----
      if (START_RUN) begin
        PASS_I <= {BW{1'b0}};
        PASS_J <= {BW{1'b0}};
      end
      else if (WB_NOW) begin
        if (PASS_J == (BLK - 1)) begin
          PASS_J <= {BW{1'b0}};
          if (PASS_I != (BLK - 1)) PASS_I <= PASS_I + 1'b1;
        end
        else begin
          PASS_J <= PASS_J + 1'b1;
        end
      end

      // ---- readout pointer ----
      // The wrap is explicit rather than relying on the counter overflowing,
      // because MNN*OBYTES is only a power of two when MN is.  Without this the
      // pointer would run past the last element for e.g. MN = 3.
      if (START_RUN)
        RD_PTR <= {RDW{1'b0}};
      else if (NEXT_STATE == S_READ)
        RD_PTR <= (RD_PTR == (MNN*OBYTES - 1)) ? {RDW{1'b0}} : RD_PTR + 1'b1;
    end
  end

  // PE mesh interconnect.
  //
  // Flat 1-D net arrays are used instead of multidimensional arrays so that
  // the netlist is legal in the most conservative Verilog dialect.
  //   AH[(PN+1)*i + j] = A entering PE(i,j);  j = PN is the row's spill.
  //   BV[PN*i + j]     = B entering PE(i,j);  i = PN is the column's spill.
  //
  // The spill nets are intentionally left unread: the A forwarding register of
  // the last column and the B forwarding register of the last row drive
  // nothing, so synthesis removes them (2*PN*DW flops).  Keeping the mesh
  // uniform is worth more than special-casing the boundary.
  wire [DW-1:0]       AH [0:PN*(PN+1)-1];
  wire [DW-1:0]       BV [0:PN*(PN+1)-1];
  wire [PNN*ACCW-1:0] ACC_BUS;

  genvar GI, GJ, GK, GB;

  generate
    // Skewed A injection on the left edge.
    // Row i of the active block needs A[PN*PASS_I + i][T-i].  The candidates
    // are BLK rows (one per row block) times MN values of k, packed at bus
    // index (block*MN + k), so the select is {PASS_I, KA}.  KA wraps to a
    // value >= MN when T < i, which selects the zero default.
    for (GI = 0; GI < PN; GI = GI + 1) begin : A_INJECT
      wire [BLK*MN*DW-1:0] ABUS;
      wire [TW-1:0]        KA;
      wire [SELW-1:0]      ASEL;

      for (GB = 0; GB < BLK; GB = GB + 1) begin : BLOCK
        for (GK = 0; GK < MN; GK = GK + 1) begin : PACK
          assign ABUS[(GB*MN + GK)*DW +: DW] = A_MEM[MN*(PN*GB + GI) + GK];
        end
      end

      assign KA   = T_CNT - GI;
      assign ASEL = (KA < MN) ? (PASS_I*MN + KA) : {SELW{1'b0}};
      assign AH[(PN+1)*GI] = (KA < MN) ? ABUS[ASEL*DW +: DW] : {DW{1'b0}};
    end

    // Skewed B injection on the top edge.
    // Column j of the active block needs B[T-j][PN*PASS_J + j].  Candidates
    // are BLK columns times MN values of k, packed at (block*MN + k).
    for (GJ = 0; GJ < PN; GJ = GJ + 1) begin : B_INJECT
      wire [BLK*MN*DW-1:0] BBUS;
      wire [TW-1:0]        KB;
      wire [SELW-1:0]      BSEL;

      for (GB = 0; GB < BLK; GB = GB + 1) begin : BLOCK
        for (GK = 0; GK < MN; GK = GK + 1) begin : PACK
          assign BBUS[(GB*MN + GK)*DW +: DW] = B_MEM[MN*GK + PN*GB + GJ];
        end
      end

      assign KB   = T_CNT - GJ;
      assign BSEL = (KB < MN) ? (PASS_J*MN + KB) : {SELW{1'b0}};
      assign BV[GJ] = (KB < MN) ? BBUS[BSEL*DW +: DW] : {DW{1'b0}};
    end

    // The PN*PN processing elements.
    for (GI = 0; GI < PN; GI = GI + 1) begin : PE_ROW
      for (GJ = 0; GJ < PN; GJ = GJ + 1) begin : PE_COL
        wire [ACCW-1:0] ACC_LOCAL;

        PE #(
          .DW      (DW),
          .ACCW    (ACCW)
        ) U_PE (
          .CLK     (CLK),
          .RESET_N (RESET_N),
          .CLR     (CLR_ACC),
          .EN      (EN_MAC),
          .A_IN    (AH[(PN+1)*GI + GJ]),
          .B_IN    (BV[PN*GI + GJ]),
          .A_OUT   (AH[(PN+1)*GI + GJ + 1]),
          .B_OUT   (BV[PN*(GI+1) + GJ]),
          .ACC_OUT (ACC_LOCAL)
        );

        // Flatten the accumulators into one bus for writeback and readout.
        assign ACC_BUS[(PN*GI + GJ)*ACCW +: ACCW] = ACC_LOCAL;
      end
    end
  endgenerate

  // Result storage and readout source.
  //
  // When PN < MN the PN*PN accumulators hold only one block at a time, so each
  // pass is copied into CBUF during S_WB.  The copy reads the accumulators
  // through ACC_BUS on the same edge that CLR_ACC zeroes them; nonblocking
  // assignment means the old value is captured, which is what is wanted.
  //
  // When PN == MN there is one pass and the accumulators are the result, so the
  // buffer and its write decoder are omitted entirely.
  wire [PTRW-1:0]   RD_ELEM;
  wire [BSEL_W-1:0] RD_BYTE;
  wire [ACCW-1:0]   C_RD;
  wire [OW-1:0]     ACC_PAD;

  assign RD_ELEM = RD_PTR[RDW-1:BSEL_W];
  assign RD_BYTE = RD_PTR[BSEL_W-1:0];

  generate
    if (NPASS > 1) begin : C_BUFFERED
      reg [ACCW-1:0] CBUF [0:MNN-1];
      // Loop indices are local to this always block.  Sharing the module-level
      // integers with the control block would make two clocked processes
      // mutate the same variable on the same edge.
      integer        CI;
      integer        WI;
      integer        WJ;

      always @(posedge CLK or negedge RESET_N) begin
        if (!RESET_N) begin
          for (CI = 0; CI < MNN; CI = CI + 1)
            CBUF[CI] <= {ACCW{1'b0}};
        end
        else if (WB_NOW) begin
          for (WI = 0; WI < PN; WI = WI + 1)
            for (WJ = 0; WJ < PN; WJ = WJ + 1)
              CBUF[MN*(PN*PASS_I + WI) + PN*PASS_J + WJ]
                <= ACC_BUS[(PN*WI + WJ)*ACCW +: ACCW];
        end
      end

      assign C_RD = CBUF[RD_ELEM];
    end
    else begin : C_DIRECT
      assign C_RD = ACC_BUS[RD_ELEM*ACCW +: ACCW];
    end
  endgenerate

  // Byte-serial result readout.
  //   RD_PTR[RDW-1:BSEL_W] selects the C element (row-major C00..)
  //   RD_PTR[BSEL_W-1:0]   selects the byte within the element (little-endian)
  // DATA_OUT is combinational from the registered pointer, so the host sees
  // byte n while asserting READ and the pointer advances on that same edge.
  assign ACC_PAD  = C_RD;                        // zero extend to OBYTES*8
  assign DATA_OUT = ACC_PAD[RD_BYTE*8 +: 8];

endmodule
