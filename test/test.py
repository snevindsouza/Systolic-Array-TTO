# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0
#
# Cocotb tests for tt_um_systolic_mm -- a 4x4 unsigned matrix product computed
# on a 2x2 output-stationary systolic array over four passes.
#
# The configuration under test is fixed by the localparams in
# tt_um_systolic_mm.v.  If you change MN/PN/DW/ACCW there, change CONFIG below
# to match; everything else in this file derives from it.
#
# Pin assignment (see interface.md):
#     ui_in [DW-1:0]  DATA_IN    operand stream; ui_in[7:DW] is ignored
#     uo_out[7:0]     DATA_OUT   result stream, OBYTES per element
#     uio_in [0]      LOAD_A
#     uio_in [1]      LOAD_B
#     uio_in [2]      START
#     uio_in [3]      READ
#     uio_out[4]      BUSY
#     uio_out[5]      DONE
#
# Sampling convention: every strobe is driven and then a rising edge is awaited,
# so the DUT samples what was set before that edge.  Outputs are read on the
# falling edge, mid-cycle, where the combinational paths from the registered
# pointers are settled.

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge

# ---------------------------------------------------------------- configuration
MN = 4       # matrix dimension (problem size)
PN = 2       # PE array dimension
DW = 4       # operand width
ACCW = 10    # accumulator width, exact = 2*DW + clog2(MN)

MNN = MN * MN
BLK = MN // PN
NPASS = BLK * BLK
T_LAST = MN - 1 + 2 * (PN - 1)
OBYTES = 2 if ACCW <= 16 else 4
RDBYTES = MNN * OBYTES
MAXV = (1 << DW) - 1

# One accept cycle, then per pass T_LAST+1 systolic steps and 1 writeback cycle.
EXP_LATENCY = 1 + NPASS * (T_LAST + 2)

LOAD_A, LOAD_B, START, READ = 0, 1, 2, 3
BUSY_BIT, DONE_BIT = 4, 5

CLK_PERIOD = 10          # microseconds, matching the Tiny Tapeout template


# ------------------------------------------------------------------- utilities
def _bits(sig):
    """Bit string, MSB first, tolerating x/z.  Works on cocotb 1.x and 2.x."""
    return str(sig.value)


def _bit(sig, idx):
    """Bit `idx` of `sig` as a character, counting from the LSB."""
    return _bits(sig)[-(idx + 1)]


def _int(sig, name):
    s = _bits(sig)
    if any(c not in "01" for c in s):
        raise AssertionError("%s is not fully driven: %s" % (name, s))
    return int(s, 2)


def busy(dut):
    return _bit(dut.uio_out, BUSY_BIT)


def done(dut):
    return _bit(dut.uio_out, DONE_BIT)


def golden(a, b):
    """Reference C = A * B, row-major, exact."""
    return [sum(a[MN * i + k] * b[MN * k + j] for k in range(MN))
            for i in range(MN) for j in range(MN)]


def fmt(m):
    return "[" + " ".join("%4d" % v for v in m) + "]"


def make_clock(sig, period, unit):
    # cocotb 2.x renamed the keyword from `units` to `unit`.
    try:
        return Clock(sig, period, unit=unit)
    except TypeError:
        return Clock(sig, period, units=unit)


async def setup(dut):
    """Start the clock and apply an asynchronous active-low reset."""
    cocotb.start_soon(make_clock(dut.clk, CLK_PERIOD, "us").start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    # uio_oe is a constant: bits 7..4 drive out, bits 3..0 are inputs.
    assert _int(dut.uio_oe, "uio_oe") == 0xF0, (
        "uio_oe should be 0xF0, got 0x%02X" % _int(dut.uio_oe, "uio_oe"))
    assert busy(dut) == "0", "BUSY should be low after reset"
    assert done(dut) == "0", "DONE should be low after reset"


# -------------------------------------------------------------------- protocol
async def load(dut, strobe, values):
    """Hold `strobe` for exactly len(values) cycles, one operand per cycle."""
    for v in values:
        dut.ui_in.value = v & 0xFF
        dut.uio_in.value = 1 << strobe
        await RisingEdge(dut.clk)
    dut.uio_in.value = 0
    dut.ui_in.value = 0
    await RisingEdge(dut.clk)


async def load_a(dut, a):
    await load(dut, LOAD_A, a)


async def load_b(dut, b):
    await load(dut, LOAD_B, b)


async def start_and_wait(dut, limit=8 * EXP_LATENCY):
    """Pulse START, then count cycles until DONE.  Returns the latency."""
    dut.uio_in.value = 1 << START
    await RisingEdge(dut.clk)              # cycle 1: accept, clear accumulators
    dut.uio_in.value = 0

    cycles = 1
    while True:
        await FallingEdge(dut.clk)
        if done(dut) == "1":
            break
        assert busy(dut) == "1", (
            "BUSY dropped at cycle %d while DONE was still low" % cycles)
        if cycles >= limit:
            raise AssertionError("DONE never rose within %d cycles" % limit)
        await RisingEdge(dut.clk)
        cycles += 1

    assert busy(dut) == "0", "BUSY should be low once DONE is high"
    return cycles


async def read_result(dut):
    """Hold READ for RDBYTES cycles and reassemble the little-endian elements.

    This is the one helper whose correctness depends on where in the clock
    period it starts, so it synchronises itself rather than trusting the caller.
    DATA_OUT is combinational from RD_PTR, and RD_PTR advances on every rising
    edge for which READ was high.  Byte 0 is therefore only visible *before* the
    first such edge: entering this function mid-period would burn that edge and
    silently shift the whole stream one byte early.
    """
    dut.uio_in.value = 0
    await RisingEdge(dut.clk)              # land just after a rising edge

    dut.uio_in.value = 1 << READ
    raw = []
    for _ in range(RDBYTES):
        await FallingEdge(dut.clk)         # mid-period: RD_PTR still points here
        raw.append(_int(dut.uo_out, "uo_out"))
        await RisingEdge(dut.clk)          # this edge advances the pointer
    dut.uio_in.value = 0
    await RisingEdge(dut.clk)

    out = []
    for e in range(MNN):
        word = 0
        for byte in range(OBYTES):
            word |= raw[e * OBYTES + byte] << (8 * byte)
        out.append(word)
    return out


async def multiply(dut, a, b):
    """Full transaction: load both matrices, compute, read the result back."""
    await load_a(dut, a)
    await load_b(dut, b)
    latency = await start_and_wait(dut)
    assert latency == EXP_LATENCY, (
        "latency was %d cycles, expected %d" % (latency, EXP_LATENCY))
    return await read_result(dut)


async def check(dut, name, a, b):
    am = [v & MAXV for v in a]
    bm = [v & MAXV for v in b]
    got = await multiply(dut, a, b)
    exp = golden(am, bm)
    if got != exp:
        dut._log.error("%s FAILED", name)
        dut._log.error("  A   = %s", fmt(am))
        dut._log.error("  B   = %s", fmt(bm))
        dut._log.error("  got = %s", fmt(got))
        dut._log.error("  exp = %s", fmt(exp))
        for i, (g, e) in enumerate(zip(got, exp)):
            if g != e:
                dut._log.error("  first mismatch at C[%d][%d]: %d != %d",
                               i // MN, i % MN, g, e)
                break
        raise AssertionError("%s: result mismatch" % name)
    dut._log.info("%-22s OK", name)
    return got


# ----------------------------------------------------------------------- tests
@cocotb.test()
async def test_reset(dut):
    """Reset clears the status flags and both operand memories."""
    dut._log.info("MN=%d PN=%d DW=%d ACCW=%d : %d PEs, %d passes, %d-cycle compute",
                  MN, PN, DW, ACCW, PN * PN, NPASS, EXP_LATENCY)
    await setup(dut)

    # Both matrices read as zero after reset, so the product is zero without
    # loading anything.
    latency = await start_and_wait(dut)
    assert latency == EXP_LATENCY
    got = await read_result(dut)
    assert got == [0] * MNN, "expected an all-zero product after reset, got %s" % fmt(got)
    dut._log.info("reset state             OK")


@cocotb.test()
async def test_directed(dut):
    """Identity, zeros, ones, the accumulator maximum, and a worked example."""
    await setup(dut)

    identity = [1 if i // MN == i % MN else 0 for i in range(MNN)]
    ramp = [(1 + i) % (MAXV + 1) for i in range(MNN)]

    # A = I  =>  C = B exactly.  This is the cleanest check that the k index and
    # the per-pass block offsets line up.
    got = await check(dut, "identity", identity, ramp)
    assert got == ramp, "A = I should give C = B"

    await check(dut, "zero A", [0] * MNN, ramp)
    await check(dut, "zero B", ramp, [0] * MNN)

    # Every element must be exactly MN.
    got = await check(dut, "all ones", [1] * MNN, [1] * MNN)
    assert got == [MN] * MNN

    # The largest value the accumulator is sized for: MN * MAXV^2.  With MN=4
    # and DW=4 that is 900 = 0x384, which needs all 10 bits.  This is the test
    # that proves there is no overflow at the top of the range, and it exercises
    # all four passes since every element must come out identical.
    top = MN * MAXV * MAXV
    got = await check(dut, "maximum", [MAXV] * MNN, [MAXV] * MNN)
    assert got == [top] * MNN, "expected %d everywhere" % top
    assert top < (1 << ACCW), "ACCW is too narrow for MN=%d DW=%d" % (MN, DW)
    dut._log.info("maximum element = %d = 0x%03X (fits %d bits)", top, top, ACCW)

    # The worked example from docs/info.md.
    a = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1]
    b = [2, 1, 3, 1, 1, 2, 1, 3, 3, 1, 2, 1, 1, 3, 1, 2]
    got = await check(dut, "worked example", a, b)
    assert got == [17, 20, 15, 18, 45, 48, 43, 46,
                   73, 76, 71, 74, 86, 59, 84, 72], (
        "the datasheet worked example does not match: %s" % fmt(got))

    # Only the low DW bits of ui_in are wired to DATA_IN, so setting the unused
    # high bits must change nothing.
    await check(dut, "ui_in[7:DW] ignored",
                [v | 0xF0 for v in ramp], [v | 0xF0 for v in ramp])


@cocotb.test()
async def test_random(dut):
    """Randomised operands across the full 0..MAXV range."""
    await setup(dut)
    random.seed(1)
    for n in range(16):
        a = [random.randrange(MAXV + 1) for _ in range(MNN)]
        b = [random.randrange(MAXV + 1) for _ in range(MNN)]
        await check(dut, "random %d" % n, a, b)


@cocotb.test()
async def test_protocol(dut):
    """Operational edge cases around the load/compute/read handshake."""
    await setup(dut)

    a = [(2 + i) % (MAXV + 1) for i in range(MNN)]
    b = [(5 + i) % (MAXV + 1) for i in range(MNN)]
    exp = golden(a, b)

    first = await check(dut, "baseline", a, b)

    # Re-reading must return the same bytes: the pointer wraps and the result
    # buffer is not consumed by reading.
    again = await read_result(dut)
    assert again == first, "re-read gave a different result: %s" % fmt(again)
    dut._log.info("re-read                 OK")

    # Pulsing START again without reloading must recompute, not accumulate on
    # top of the previous result.  The accumulators are cleared at the start of
    # every pass, so this is the check that the clear is per-pass and not
    # once-per-transaction.
    latency = await start_and_wait(dut)
    assert latency == EXP_LATENCY
    repeat = await read_result(dut)
    assert repeat == exp, (
        "repeated START doubled the result: %s vs %s" % (fmt(repeat), fmt(exp)))
    dut._log.info("repeated START          OK")

    # Reloading A alone must leave B intact.
    a2 = [(7 + i) % (MAXV + 1) for i in range(MNN)]
    await load_a(dut, a2)
    latency = await start_and_wait(dut)
    assert latency == EXP_LATENCY
    got = await read_result(dut)
    assert got == golden(a2, b), (
        "B did not survive an A-only reload: %s vs %s" % (fmt(got), fmt(golden(a2, b))))
    dut._log.info("partial reload          OK")

    # An aborted load must not leave the write pointer stranded: the pointer
    # rearms whenever the strobe is low, so a short burst followed by a full one
    # still lands the full matrix.
    await load(dut, LOAD_A, a2[:MNN // 2])
    await check(dut, "aborted then full load", a, b)


@cocotb.test()
async def test_reset_mid_compute(dut):
    """Asynchronous reset asserted during a pass clears the machine."""
    await setup(dut)

    a = [(1 + i) % (MAXV + 1) for i in range(MNN)]
    b = [(2 + i) % (MAXV + 1) for i in range(MNN)]
    await load_a(dut, a)
    await load_b(dut, b)

    dut.uio_in.value = 1 << START
    await RisingEdge(dut.clk)
    dut.uio_in.value = 0

    # Land inside the first pass, then pull reset.
    await ClockCycles(dut.clk, 3)
    await FallingEdge(dut.clk)
    assert busy(dut) == "1", "expected BUSY during compute"

    dut.rst_n.value = 0
    await FallingEdge(dut.clk)
    assert busy(dut) == "0", "BUSY should clear immediately on reset"
    assert done(dut) == "0", "DONE should clear immediately on reset"
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    # Reset zeroes the operand memories, so the next product is zero.
    latency = await start_and_wait(dut)
    assert latency == EXP_LATENCY
    got = await read_result(dut)
    assert got == [0] * MNN, (
        "operand memories should be zero after reset, got %s" % fmt(got))
    dut._log.info("reset during compute    OK")


@cocotb.test()
async def test_latency(dut):
    """START -> DONE is data independent."""
    await setup(dut)
    random.seed(7)
    seen = set()
    for _ in range(6):
        a = [random.randrange(MAXV + 1) for _ in range(MNN)]
        b = [random.randrange(MAXV + 1) for _ in range(MNN)]
        await load_a(dut, a)
        await load_b(dut, b)
        seen.add(await start_and_wait(dut))
        await read_result(dut)
    assert seen == {EXP_LATENCY}, (
        "latency varied with data: saw %s, expected only %d"
        % (sorted(seen), EXP_LATENCY))
    dut._log.info("latency fixed at %d cycles over %d transactions",
                  EXP_LATENCY, 6)
