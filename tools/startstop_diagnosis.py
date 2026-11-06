"""Offline replay of the analysis chain in src/main.cpp.

Two modes, so the start/stop fix can be verified without hardware:

  before : fixed OFF threshold 120, non overlapping 32 ms blocks,
           rate taken from the last interval only, 3.0 Hz ceiling,
           3 s stale zeroing, FREQ gated by the envelope state of the
           last block of the capture.

  after  : relative OFF threshold, 8 ms block hop, rate averaged over
           the rising edges of the observation window, 8 Hz clamp,
           FREQ gated by whether the capture contains vibration at all.

Everything else (Hann window, FFT, background estimate, parabolic peak)
is identical in both modes and is ported 1:1 from main.cpp.

Run:  python tools/startstop_diagnosis.py
"""

import math

import numpy as np

# ------------------------------------------------------------------
# Constants from main.cpp
# ------------------------------------------------------------------
N = 1024
FS = 1000.0
FREQ_MIN = 80.0
FREQ_MAX = 180.0
SEARCH_FREQ_MAX = 300.0

ENV_BLOCK_SIZE = 32
ENV_BLOCK_HOP = 8

VIB_ON_THRESHOLD = 180.0
VIB_OFF_FLOOR = 120.0
VIB_OFF_RATIO = 0.55
ENV_PEAK_DECAY = 0.995
VIB_MIN_ON_BLOCKS = 6

MIN_PEAK_RATIO = 5.0

STARTSTOP_MAX_HZ = 8.0
STARTSTOP_EDGE_HISTORY = 8
STARTSTOP_AVG_WINDOW_MS = 4000
STARTSTOP_MIN_EDGES = 3
STARTSTOP_SLOW_SPAN_MS = 20000
STARTSTOP_HOLD_MS = 5000

# before-mode constants
OLD_OFF_THRESHOLD = 120.0
OLD_MAX_HZ = 3.0
OLD_TIMEOUT_MS = 3000

MIN_BIN = int(math.ceil(FREQ_MIN * N / FS))
MAX_BIN = int(math.floor(SEARCH_FREQ_MAX * N / FS))

WINDOW = 0.5 * (1.0 - np.cos(2.0 * np.pi * np.arange(N) / (N - 1))).astype(np.float32)


def analyze_axis(samples):
    """analyzeAxis() from main.cpp -> (peak_frequency, peak_ratio)."""
    buf = np.asarray(samples, dtype=np.float32)
    ac = buf - np.float32(buf.mean(dtype=np.float64))
    spec = np.fft.rfft((ac * WINDOW).astype(np.float32))
    mag = np.abs(spec).astype(np.float32)

    band = mag[MIN_BIN:MAX_BIN + 1]
    peak_bin = MIN_BIN + int(np.argmax(band))
    peak_mag = float(mag[peak_bin])

    mask = np.ones(band.shape[0], dtype=bool)
    lo = max(MIN_BIN, peak_bin - 3) - MIN_BIN
    hi = min(MAX_BIN, peak_bin + 3) - MIN_BIN
    mask[lo:hi + 1] = False
    bg = float(band[mask].mean()) if mask.any() else 0.0
    ratio = peak_mag / bg if bg > 1e-6 else 0.0

    delta = 0.0
    if MIN_BIN < peak_bin < MAX_BIN:
        left = float(mag[peak_bin - 1])
        center = float(mag[peak_bin])
        right = float(mag[peak_bin + 1])
        den = left - 2.0 * center + right
        if abs(den) > 1e-6:
            delta = 0.5 * (left - right) / den
        delta = max(-0.5, min(0.5, delta))

    return (peak_bin + delta) * FS / N, ratio


def envelopes(axes, hop):
    """calculateEnvelopeBlock(): strongest-axis AC RMS per window."""
    out = []
    for start in range(0, N - ENV_BLOCK_SIZE + 1, hop):
        block = axes[:, start:start + ENV_BLOCK_SIZE].astype(np.float64)
        ac = block - block.mean(axis=1, keepdims=True)
        out.append(float(np.sqrt((ac * ac).mean(axis=1)).max()))
    return out


def gate_wave(rate, duty, rise_tau, decay_tau, total, on_ms=None):
    """Start/stop gate with first order spin-up / coast-down."""
    t = np.arange(total) / FS
    if rate <= 0.0:
        ideal = np.ones(total)
    elif on_ms is not None:
        ideal = (np.mod(t * 1000.0, 1000.0 / rate) < on_ms).astype(float)
    else:
        ideal = (np.mod(t * rate, 1.0) < duty).astype(float)

    out = np.empty(total)
    state = 0.0
    for i in range(total):
        tau = rise_tau if ideal[i] > state else decay_tau
        state += (ideal[i] - state) * (1000.0 / FS / tau)
        out[i] = state
    return out


def replay(rate, fixed, duty=0.5, rise_tau=80.0, decay_tau=150.0, on_ms=None,
           loop_ms=1200.0, carrier=120.0, amp=400.0, sigma=80.0,
           buffers=160, seed=7):
    rng = np.random.default_rng(seed)
    total = int((buffers + 4) * loop_ms * FS / 1000.0) + N
    gate = gate_wave(rate, duty, rise_tau, decay_tau, total, on_ms=on_ms)

    hop = ENV_BLOCK_HOP if fixed else ENV_BLOCK_SIZE

    state = False
    env_peak = 0.0
    last_edge = 0
    start_stop = 0.0
    edges = []          # before mode: previous/last pair
    prev_edge = 0
    ring = [0] * STARTSTOP_EDGE_HISTORY
    ring_write = 0
    ring_count = 0

    lines = []
    warmup = int(STARTSTOP_HOLD_MS / loop_ms) + 2

    for k in range(buffers):
        cap_ms = k * loop_ms
        off = int(cap_ms * FS / 1000.0)
        axes = rng.normal(0.0, sigma, size=(3, N))
        t = np.arange(N) / FS
        axes[0] += amp * gate[off:off + N] * np.sin(2.0 * np.pi * carrier * t)

        env = envelopes(axes, hop)
        blocks_on = 0

        for b, level in enumerate(env):
            index = b * hop

            if fixed:
                env_peak *= ENV_PEAK_DECAY
                if level > env_peak:
                    env_peak = level
                off_level = max(VIB_OFF_FLOOR, VIB_OFF_RATIO * env_peak)
            else:
                off_level = OLD_OFF_THRESHOLD

            if level >= VIB_ON_THRESHOLD:
                blocks_on += 1

            old = state
            if not state:
                if level >= VIB_ON_THRESHOLD:
                    state = True
            else:
                if level <= off_level:
                    state = False

            if (not old) and state:
                edge_ms = cap_ms + index
                if fixed:
                    ring[ring_write] = edge_ms
                    ring_write = (ring_write + 1) % STARTSTOP_EDGE_HISTORY
                    ring_count = min(ring_count + 1, STARTSTOP_EDGE_HISTORY)
                    last_edge = edge_ms
                else:
                    prev_edge = last_edge
                    last_edge = edge_ms
                    if prev_edge != 0:
                        period = last_edge - prev_edge
                        if period > 0:
                            freq = 1000.0 / period
                            if freq <= OLD_MAX_HZ:
                                start_stop = freq

        now_ms = cap_ms + N

        if fixed:
            first = last = 0
            count = 0
            for i in range(ring_count):
                slot = (ring_write - ring_count + i + STARTSTOP_EDGE_HISTORY) % STARTSTOP_EDGE_HISTORY
                edge_ms = ring[slot]
                if now_ms - edge_ms > STARTSTOP_AVG_WINDOW_MS:
                    continue
                if count == 0:
                    first = edge_ms
                last = edge_ms
                count += 1

            if count >= STARTSTOP_MIN_EDGES and last > first:
                start_stop = min((count - 1) * 1000.0 / (last - first), STARTSTOP_MAX_HZ)
            elif (ring_count >= 2 and last_edge != 0
                  and now_ms - last_edge <= STARTSTOP_HOLD_MS):
                period = last_edge - ring[(ring_write - 2 + STARTSTOP_EDGE_HISTORY) % STARTSTOP_EDGE_HISTORY]
                if 0 < period <= STARTSTOP_SLOW_SPAN_MS:
                    start_stop = min(1000.0 / period, STARTSTOP_MAX_HZ)
            elif last_edge == 0 or now_ms - last_edge > STARTSTOP_HOLD_MS:
                start_stop = 0.0

            present = blocks_on >= VIB_MIN_ON_BLOCKS
        else:
            if last_edge != 0 and now_ms - last_edge > OLD_TIMEOUT_MS:
                start_stop = 0.0
            present = state

        best_f, best_ratio = None, -1.0
        for ax in range(3):
            f, r = analyze_axis(axes[ax])
            if r > best_ratio:
                best_f, best_ratio = f, r

        valid = present and best_ratio >= MIN_PEAK_RATIO and FREQ_MIN <= best_f <= FREQ_MAX
        main_freq = best_f if valid else 0.0

        if k >= warmup:
            lines.append((present, main_freq, start_stop, best_ratio, best_f))

    vib = np.array([1 if l[0] else 0 for l in lines])
    freq = np.array([l[1] for l in lines])
    ss = np.array([l[2] for l in lines])
    ratio = np.array([l[3] for l in lines])
    peak_f = np.array([l[4] for l in lines])

    err = np.abs(ss - rate) / rate if rate > 0 else np.zeros_like(ss)

    return {
        "vib_zero": 100.0 * (vib == 0).mean(),
        "freq_zero": 100.0 * (freq == 0).mean(),
        "all_zero": 100.0 * ((vib == 0) & (freq == 0) & (ss == 0)).mean(),
        "ratio_med": float(np.median(ratio)),
        "ratio_lt5": 100.0 * (ratio < MIN_PEAK_RATIO).mean(),
        "ss_zero": 100.0 * (ss == 0).mean(),
        "ss_med": float(np.median(ss)),
        "ss_err": 100.0 * float(np.median(err)),
        "peak_med": float(np.median(peak_f)),
    }


HEADER = ("rate", "VIB=0%", "FREQ=0%", "all0%", "RATIO", "R<5%", "SS=0%",
          "SS med", "SSerr%", "peak Hz")


def table(title, rates, **kwargs):
    print(f"--- {title} ---")
    print("{:>5} {:>8} {:>8} {:>6} {:>7} {:>6} {:>7} {:>8} {:>7} {:>8}".format(*HEADER))
    for rate in rates:
        for fixed, tag in ((False, "before"), (True, "after")):
            res = replay(rate, fixed, **kwargs)
            print("{:>5} {:>8.0f} {:>8.0f} {:>6.0f} {:>7.1f} {:>6.0f} {:>7.0f} {:>8.2f} {:>7.0f} {:>8.1f}  {}"
                  .format(rate, res["vib_zero"], res["freq_zero"], res["all_zero"],
                          res["ratio_med"], res["ratio_lt5"], res["ss_zero"],
                          res["ss_med"], res["ss_err"], res["peak_med"], tag))
        print()


def main():
    print("carrier 120 Hz, amp 400 LSB (ON envelope ~280), quiet noise 80 LSB")
    print("loop 1200 ms -> 176 ms per capture is not sampled\n")
    rates = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    table("50% duty, rise 80 ms, coast-down 150 ms", rates,
          duty=0.5, rise_tau=80.0, decay_tau=150.0)
    table("50% duty, rise 40 ms, coast-down 60 ms", rates,
          duty=0.5, rise_tau=40.0, decay_tau=60.0)
    table("50% duty, rise 80 ms, coast-down 150 ms, no unsampled gap (loop 1044 ms)",
          rates, duty=0.5, rise_tau=80.0, decay_tau=150.0, loop_ms=1044.0)


if __name__ == "__main__":
    main()
