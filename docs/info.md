<!---

This file is used to generate your project datasheet. Please fill in the
information below and delete any unused sections.

You can also include images in this folder and reference them in the markdown.
Each image must be less than 512 kb in size, and the combined size of all images
must be less than 1 MB.

-->

## How it works

This is a **2×2 output-stationary systolic array** that computes the matrix
product `C = A × B` for 2×2 matrices of 4-bit unsigned integers. The result is
exact — 9 bits per element, no overflow, no saturation.

A systolic array computes a matrix product by streaming the operands through a
mesh of small multiply-accumulate cells, each of which only ever talks to its
immediate neighbours. There is no global broadcast and no shared bus, which is
what makes the structure scale well in silicon.

```
            B col 0     B col 1
               |           |
  A row 0 --> PE00 -----> PE01
               |           |
  A row 1 --> PE10 -----> PE11
               |           |
```

Each `PE` owns one accumulator and holds it for the whole computation — that is
what "output-stationary" means. `PE(i,j)` accumulates `C[i][j]`. On every clock
cycle it multiplies the two operands arriving at its inputs, adds the product to
its accumulator, and passes the `A` operand one step right and the `B` operand
one step down:

```
ACC   <= ACC + A_IN * B_IN
A_OUT <= A_IN
B_OUT <= B_IN
```

### The interesting part: why the operands are skewed

Each hop through the mesh costs exactly one clock cycle. So an operand injected
at the left edge of row `i` arrives at `PE(i,j)` after `j` cycles, and an operand
injected at the top of column `j` arrives at `PE(i,j)` after `i` cycles. For the
right pair of numbers to meet in the right cell, the injection has to be **skewed
in time**:

```
left edge of row i,    step T :  A[i][T-i]    for i <= T <= i+N-1, else 0
top edge of column j,  step T :  B[T-j][j]    for j <= T <= j+N-1, else 0
```

With that schedule, `PE(i,j)` sees the operand pair `(A[i][k], B[k][j])` where

```
k = T - i - j
```

which is exactly the inner product `C[i][j] = sum_k A[i][k]*B[k][j]`. Steps
outside a cell's window inject **zeros**, so no per-cell "valid" signal is
needed — a zero operand simply contributes nothing to the accumulator. That trick
removes a control wire from all four cells.

The skew is generated combinationally from the step counter reading out of the
operand storage, so there are no delay-line registers anywhere in the design.

The last useful step is `T = 3(N-1) = 3`, so the multiply takes **4 clock
cycles** and `DONE` rises **5 cycles** after `START`.

### Configurability

The RTL is parameterised in array size (`N`), operand width (`DW`) and
accumulator width (`ACCW`), all set in one place in `tt_um_systolic_mm.v`. The
same source builds anything from this 2×2 4-bit version up to a 4×4 8-bit
version; the larger builds simply need more tiles. This submission is the
configuration that fits the tile budget — see the *area* note at the end.

## How to test

All communication is byte-serial over `ui_in` (in) and `uo_out` (out), with four
strobes and two status bits on the bidirectional pins. Every strobe is sampled on
the rising edge of the clock.

| Signal | Pin | Direction |
|---|---|---|
| `DATA_IN[3:0]` | `ui_in[3:0]` | in (`ui_in[7:4]` unused) |
| `DATA_OUT[7:0]` | `uo_out[7:0]` | out |
| `LOAD_A` | `uio[0]` | in |
| `LOAD_B` | `uio[1]` | in |
| `START` | `uio[2]` | in |
| `READ` | `uio[3]` | in |
| `BUSY` | `uio[4]` | out |
| `DONE` | `uio[5]` | out |

Hold `rst_n` low for at least one clock to reset. After reset both matrices read
as zero and `BUSY = DONE = 0`.

### 1. Load matrix A — 4 cycles

Hold `LOAD_A` high for **exactly 4 consecutive clock cycles**, presenting one
operand per cycle on `ui_in[3:0]` in row-major order:

```
cycle : 0    1    2    3
A     : A00  A01  A10  A11
```

Then drop `LOAD_A`. The internal pointer rearms whenever `LOAD_A` is low, so a
partial or aborted burst cannot leave the pointer stranded.

### 2. Load matrix B — 4 cycles

Identical, using `LOAD_B`:

```
cycle : 0    1    2    3
B     : B00  B01  B10  B11
```

### 3. Compute — 5 cycles

Pulse `START` high for one cycle. `BUSY` goes high immediately and `DONE` rises
**5 cycles** after the `START` cycle. Either poll `DONE` or just wait 5 cycles.

The accumulators are cleared automatically at the start of every run, so pulsing
`START` again without reloading recomputes the same product rather than doubling
it.

### 4. Read the result — 8 cycles

Hold `READ` high for **8 consecutive clock cycles**. Each of the four `C`
elements is 9 bits, so it is returned as **2 bytes, least-significant byte
first**, in row-major order:

```
cycle    : 0        1         2        3        4        5        6        7
DATA_OUT : C00[7:0] C00[8]    C01[7:0] C01[8]   C10[7:0] C10[8]   C11[7:0] C11[8]
```

The high byte only ever contains bit 8, so the unused bits read as zero.
Reassemble each element as `low | (high << 8)`.

The read pointer wraps back to the start, so you can re-read the whole result set
as many times as you like without recomputing. `DONE` stays high throughout.

### Worked example

```
A = [ 1  2 ]      B = [ 5  6 ]      C = [ 1*5 + 2*7   1*6 + 2*8 ]   = [ 19  22 ]
    [ 3  4 ]          [ 7  8 ]          [ 3*5 + 4*7   3*6 + 4*8 ]     [ 43  50 ]
```

Drive `1, 2, 3, 4` during `LOAD_A`, then `5, 6, 7, 8` during `LOAD_B`, pulse
`START`, wait for `DONE`, then read 8 bytes. You should see
`19, 0, 22, 0, 43, 0, 50, 0`.

### Things worth testing on silicon

| Test | Expected |
|---|---|
| A = identity `[1 0; 0 1]` | `C = B` exactly |
| A or B all zero | `C = 0` |
| A = B = all ones | every element = 2 |
| A = B = all 15 (maximum) | every element = 450 = `0x1C2`, i.e. bytes `C2 01` |
| `START` twice without reloading | identical result, not doubled |
| Reload A only, then `START` | new A against the retained B |
| `READ` twice in a row | same 8 bytes both times |
| `rst_n` low mid-computation | `BUSY` and `DONE` clear immediately; storage zeroed |

The maximum-value case is the one worth checking carefully: `2 × 15 × 15 = 450`
is the largest result the accumulator is sized for, so it proves there is no
overflow at the top of the range.

A full RP2040 / MicroPython bring-up sequence and cycle-level waveforms are in
`interface.md` and `timing.md` in the project repository.

## External hardware

**None.** The design needs only the clock, reset and GPIO provided by the Tiny
Tapeout demo board; the RP2040 on the board is sufficient to drive the whole
protocol. No external memory, level shifters, ADC or display are required.

A complete load-compute-read transaction is **24 clock cycles**:

```
load A  5   +   load B  5   +   compute  5   +   read  9   =  24
```

At the nominal 10 MHz that is 2.4 µs per matrix product.

## A note on area

The first version of this project was a 4×4 array with 8-bit operands. It
synthesised to 123 248 µm², which OpenROAD reported as **774 % utilisation** —
roughly eight tiles' worth of logic in a one-tile hole — and the build failed
during global placement.

The dominant cost is the multipliers, and their area grows as the *square* of
both the array dimension and the operand width:

```
area ~ N^2 * (56 * DW^2 + 59 * ACCW) + 26 * flops
```

Narrowing the operands is therefore just as strong a lever as shrinking the
array. Going from a 4×4 array of 8-bit cells to a 2×2 array of 4-bit cells cuts
the logic by about 13×, down to a modelled 9 670 µm², which is a comfortable
30 % utilisation on the two tiles requested here.

The schedule, the protocol and the RTL are identical to the large version — this
is the same design, built at a size that fits.
