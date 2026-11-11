![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# Systolic Array - 4×4 Matrix Multiplier

A Tiny Tapeout digital design that computes the product of two **4×4 matrices of unsigned 4-bit integers** using a **2×2 output-stationary systolic array**. The four processing elements are reused across four computation passes, allowing the complete 4×4 operation to fit the project's **1×2 Tiny Tapeout tile allocation**.

<img width="303" height="79" alt="image" src="https://github.com/user-attachments/assets/cd44bca6-03c6-469a-87e6-95bbb09b2464" />


The result is exact: each output element is 10 bits wide, with no truncation, saturation, or overflow for legal operands.

## Design configuration

| Parameter | Value | Description |
|---|---:|---|
| `MN` | 4 | Matrix dimension (`4×4`) |
| `PN` | 2 | Systolic-array dimension (`2×2`) |
| `DW` | 4 | Unsigned operand width; legal values are `0`–`15` |
| `ACCW` | 10 | Accumulator/result width |
| Processing elements | 4 | `PN × PN` multiply-accumulate cells |
| Computation passes | 4 | `(MN / PN)²` output blocks |
| Result bytes | 32 | 16 elements × 2 bytes per element |
| Tiny Tapeout tiles | `1×2` | As declared in `info.yaml` |
| Top module | `tt_um_systolic_mm` | Tiny Tapeout wrapper |
| Process target | Sky130 | Tiny Tapeout SKY130 project template / shuttle target |
| Declared clock | 50 MHz | `clock_hz` in `info.yaml`; final timing capability depends on timing analysis |

The maximum output value is:

<img width="316" height="42" alt="image" src="https://github.com/user-attachments/assets/9081473b-2809-4587-a2dd-2cc02d039181" />


Ten bits are sufficient to represent this value exactly. The accumulator width is derived from the operand width and matrix dimension:

<img width="355" height="41" alt="image" src="https://github.com/user-attachments/assets/bfd01627-96fc-4a21-aa46-65599d93a7df" />


## Architecture

The design uses an **output-stationary** dataflow. Each processing element (PE) owns an accumulator for one output element in the active `2×2` output block. Operand A moves from left to right; operand B moves from top to bottom. Each PE multiplies its incoming operands, adds the product to its accumulator, and forwards both operands by one hop per clock.

```text
                 B column 0       B column 1
                     |                 |
                     v                 v
A row 0  ------->  +------+  ------> +------+
                   | PE00 |          | PE01 |
                   +------+          +------+
                      |                 |
                      v                 v
A row 1  ------->  +------+  ------> +------+
                   | PE10 |          | PE11 |
                   +------+          +------+
```

The input streams are **time-skewed** so that PE `(i,j)` receives the matching pair `A[i][k]` and `B[k][j]`. The inner-product index at systolic step `T` is `k = T - i - j`. Zero operands are injected outside each PE's active window.

Because the hardware has four PEs but the result contains sixteen elements, the controller computes the four `2×2` blocks of C in separate passes:

1. Top-left block: `C[0:1][0:1]`
2. Top-right block: `C[0:1][2:3]`
3. Bottom-left block: `C[2:3][0:1]`
4. Bottom-right block: `C[2:3][2:3]`

After each pass, the four accumulated values are copied to the result buffer. The PE accumulators and forwarding registers are then cleared before the next pass. The result buffer retains all sixteen output elements for readout.

For the shipped configuration, `T_LAST = MN - 1 + 2(PN - 1) = 5`. Each pass takes six systolic steps and one writeback cycle. Including the start-acceptance cycle, `DONE` is expected **29 clock cycles after `START` is accepted**.

## Tiny Tapeout interface

All input strobes are sampled on the rising edge of `clk`. `rst_n` is an asynchronous, active-low reset. The `uio_oe` value is constant: `8'b1111_0000` (the lower four bidirectional pins are inputs and the upper four are outputs).

| Tiny Tapeout signal | Direction | Function |
|---|---|---|
| `clk` | Input | Single clock domain; rising-edge operation |
| `rst_n` | Input | Asynchronous active-low reset |
| `ena` | Input | Not used by the datapath; Tiny Tapeout asserts it when this design is selected |
| `ui_in[3:0]` | Input | `DATA_IN`; one 4-bit matrix operand per cycle |
| `ui_in[7:4]` | Input | Unused by the arithmetic datapath |
| `uo_out[7:0]` | Output | `DATA_OUT`; result byte stream |
| `uio_in[0]` | Input | `LOAD_A`; load matrix A |
| `uio_in[1]` | Input | `LOAD_B`; load matrix B |
| `uio_in[2]` | Input | `START`; one-cycle compute request |
| `uio_in[3]` | Input | `READ`; read-result strobe |
| `uio_out[4]` | Output | `BUSY`; computation in progress |
| `uio_out[5]` | Output | `DONE`; result available |
| `uio_out[7:6]` | Output | Reserved; driven low |

### Transaction protocol

#### 1. Reset

Assert `rst_n = 0` to asynchronously clear the controller, operand memories, result storage, pointers, PE accumulators, forwarding registers, and status outputs. After reset, `BUSY = 0` and `DONE = 0`.

#### 2. Load matrix A

Hold `LOAD_A` high for **16 consecutive clock cycles** and present one operand on `ui_in[3:0]` per cycle in row-major order:

```text
A00, A01, A02, A03,
A10, A11, A12, A13,
A20, A21, A22, A23,
A30, A31, A32, A33
```

The first operand is captured on the first clock edge for which `LOAD_A` is high. Deassert `LOAD_A` for at least one clock cycle to rearm the load pointer.

#### 3. Load matrix B

Repeat the same 16-cycle sequence using `LOAD_B`, presenting B in row-major order:

```text
B00, B01, B02, B03,
B10, B11, B12, B13,
B20, B21, B22, B23,
B30, B31, B32, B33
```

#### 4. Compute

Pulse `START` high for one clock cycle. `BUSY` indicates that the four systolic passes are in progress. For the configured `MN=4`, `PN=2` build, `DONE` rises 29 clock cycles after the `START` acceptance cycle. `BUSY` is low when the result is ready.

The operands remain stored after computation. A new `START` recomputes the product; it does not accumulate on top of the previous result.

#### 5. Read matrix C

Hold `READ` high for **32 consecutive clock cycles**. The sixteen 10-bit result elements are transferred in row-major order, with the least-significant byte first:

```text
C00[7:0], C00[9:8], C01[7:0], C01[9:8], ...,
C33[7:0], C33[9:8]
```

Each element occupies two output bytes. In the second byte, only bits `[1:0]` can be nonzero; the upper six bits are zero. Reassemble an element as:

```text
C[i][j] = DATA_OUT_LOW | (DATA_OUT_HIGH << 8)
```

The read pointer wraps after the final byte, allowing the result to be read again without recomputing. `DONE` remains asserted while the result is available, including during readout.

## Repository layout

```text
.
├── .github/
│   └── workflows/
│       └── test.yaml          # GitHub Actions RTL regression
├── docs/
│   └── info.md                # Tiny Tapeout datasheet source
├── src/
│   ├── project.v              # Tiny Tapeout top-level wrapper
│   ├── systolic_array.v       # Matrix storage, controller, skewing, PE mesh and readout
│   └── pe.v                   # Parameterized processing element (MAC and forwarding registers)
├── test/
│   ├── Makefile               # Icarus Verilog / cocotb simulation setup
│   ├── requirements.txt       # Python test dependencies
│   ├── tb.v                   # Cocotb simulation wrapper and waveform dump
│   ├── test.py                # Directed, randomized and protocol tests
│   └── README.md               # Tiny Tapeout testbench instructions
├── info.yaml                  # Tiny Tapeout project metadata and pin configuration
└── README.md
```

## Configuration notes

The Tiny Tapeout wrapper in `src/project.v` is the configuration point for `MN`, `PN`, `DW`, and `ACCW`. The `tiles` field in `info.yaml` must remain consistent with the selected hardware configuration. The current checked-in settings are `MN=4`, `PN=2`, `DW=4`, `ACCW=10`, and `tiles: "1x2"`.

If changing the parameters:

- `PN` must divide `MN`.
- `ACCW` must be wide enough for the exact sum of `MN` unsigned `DW`-bit products. For this configuration, `ACCW=10`.
- Update the parameter values in `test/test.py` to match the RTL.
- Re-evaluate tile allocation and update `info.yaml` accordingly.
- Re-run RTL simulation and the Tiny Tapeout hardening flow; do not infer timing closure from RTL simulation.

> **Documentation maintenance:** `docs/info.md` is the source used for the Tiny Tapeout datasheet. Its currently checked-in text describes an earlier 2×2-matrix / 9-bit-result configuration, while the RTL and `info.yaml` describe the current 4×4-matrix / 10-bit-result configuration. Update `docs/info.md` before treating the generated datasheet as documentation for this build.

## Project metadata

- **Project:** Systolic Array — 4×4 Matrix Multiplier
- **Author:** Snevin Leoneel Dsouza
- **HDL:** Verilog
- **Tiny Tapeout top module:** `tt_um_systolic_mm`
- **Tile allocation:** `1×2`

## References

- [Tiny Tapeout](https://tinytapeout.com/)
- [Tiny Tapeout HDL testing guide](https://tinytapeout.com/hdl/testing/)
- [Testbench instructions](test/README.md)
- [Tiny Tapeout project metadata](info.yaml)

