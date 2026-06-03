# P1 ACQUISITION make-or-break results (full acquire+sync+decode SIM)

Harness: MERCURY_P1_ACQ=1 — fills the whole ring buffer with AWGN, overlays the
burst at an unknown lead-in, and LEAVES ofdm_forced_delay=-1 so the real
Schmidl-Cox detection + ±30 Hz coarse search + Moose fine CFO run. CFO injected
via -f <Hz> (test_tx_carrier_offset). Contrast: the normal BER harness forces
the delay + skips freq sync (perfect-sync DECODE floor, phase4 §7.4).

SNR3k(Nc=5) = EsN0 - 5.1 dB   [EsN0 + 6.0 + 10log10(46.875*Nc/3000)]

## A. Acquisition floor vs perfect-sync decode floor (zero CFO, Nc=5 QPSK r4)
| Es/N0 | PRE=4 BER | PRE=8 BER | PRE=16 BER | SNR3k |
|------:|----------:|----------:|-----------:|------:|
|   +6  |   0       |   0       |   0        | +0.9  |
|   +4  |   0       |   0       |   0        | -1.1  |
|   +3  |   -       |   -       |   0        | -2.1  |
|   +2  |  0.095    |  0.051    |  0.009     | -3.1  |
|    0  |  0.36     |  0.30     |  0.25      | -5.1  |
|   -2  |  0.50     |  0.49     |  0.49      | -7.1  |
|   -3  |  0.49     |  0.49     |  0.49      | -8.1  |

Perfect-sync DECODE floor (phase4 §7.4): Es/N0 = -3 (SNR3k = -8.1, BER=0).
ACQUISITION floor (PRE=16, BER=0): Es/N0 = +3 (SNR3k = -2.1).
=> Acquisition penalty ~6 dB. Longer preamble (4->16) barely moves the clean
   cliff (+4 stays +4); only shaves the marginal region (+2: 0.095->0.009).

## B. Control — baseline Nc=50 CONFIG_0 (BPSK r1/16) through the SAME harness
| Es/N0 | BER |   (zero CFO)
|------:|----:|
|   +2  |  0  |
|    0  |  0  |
|   -2  |  0  |
|   -4  | 0.49|
Baseline Nc=50 acquires to Es/N0=-2; Nc=5 to +4. The ~6 dB gap is FEW-CARRIER-
SPECIFIC (weak Schmidl-Cox metric: 5-carrier averaging vs 50). NOT a harness
artifact.

## C. CFO tolerance (Es/N0=8, well above acq floor, PRE=16)
| CFO Hz | Nc=5 QPSK | Nc=5 BPSK | Nc=50 baseline |
|-------:|----------:|----------:|---------------:|
|   0    |    0      |    0      |     0          |
|   1-2  |    0      |    -      |     0          |
|   5    |  0.49     |  0.48     |   0.49         |
|  10    |  0.49     |  0.48     |   0.48         |
CFO budget ~±2 Hz for ALL arms incl. baseline Nc=50 => NOT few-carrier-specific.
Moose MEASURES the offset accurately (10 Hz injected -> Moose=9.39 Hz) but the
offline-harness correction leaves a large residual (trial-1 residual -2.88 Hz,
SKIP-VAR var>1) => pre-existing CONFIG_0 CFO-correction weakness in the BER+acq
path (sign §23 + coarse/fine interaction), inherited not introduced.

## Verdict
ACQUISITION GATES the few-carrier coherent tier. It acquires to SNR3k ~ -2 dB
(PRE=16), NOT the -8 dB perfect-sync decode floor. ~6 dB short. The timing/burst
detection (Schmidl-Cox on 5 carriers + the 0.15 metric threshold over a long
buffer) is the dominant new bottleneck; preamble LENGTH alone does not close it.
