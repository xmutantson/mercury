# Mercury

Mercury is an OFDM HF data modem written in C++, designed as a free-software
replacement for VARA HF on Winlink and Pat workflows. It targets life-critical
communication over noisy, fading HF channels — robustness is prioritized over
peak throughput. The modem combines coherent OFDM (BPSK through 32-QAM), MFSK
weak-signal modes, LDPC forward error correction, end-to-end encryption, and a
selective-ACK ARQ protocol. Licensed under AGPL v3.

## Project Status

Mercury is in **active development**. Core ARQ, gearshift, compression, and
encryption are production-ready for VARA HF replacement on Winlink/Pat-style
workflows. The newer link-layer features (SACK Design A piggyback retransmits,
the effective-rate optimizer) are beta — shipped and stable on the development
testbed (VB-Cable + IONOS fading simulator) but still accumulating field hours.

The current link-layer is organized as four cooperating mechanisms:

1. **Gearshift** — chooses an initial modulation/coding config from measured SNR.
2. **SACK Design A** — partial-batch recovery so any viable config keeps making
   progress; piggyback retransmits ride alongside new data instead of stalling.
3. **Effective-rate optimizer** — a calibration-driven Q-table that, given the
   recent batch history, nudges the gearshift toward the config with the highest
   *effective* throughput for the channel (not just the highest PHY rate).
4. **BREAK** — safety net. When the link genuinely cannot make progress at the
   current config, BREAK falls back to the last known good config rather than
   timing out.

Headline result on a clean loopback at CONFIG_16: **8898 bps on the wire,
~22 kbps effective with streaming compression on text** (roughly 46× VARA HF
L1 on the same payload). Real HF channels are slower; this number is a ceiling,
not a typical operating point.

## Features

- **17 OFDM configurations.** CONFIG_0 (BPSK, ~71 bps WB / ~14 bps NB) through
  CONFIG_16 (32-QAM 14/16, ~5665 bps WB / ~1133 bps NB). 50 data subcarriers
  WB, 10 NB. All modes use LDPC with up to 50 SPA iterations + CRC16.
- **3 MFSK weak-signal modes.** ROBUST_0/1/2 (config IDs 100/101/102) operate
  below the OFDM threshold — ROBUST_0 decodes at roughly −13 dB Es/N0.
- **Narrowband (468.75 Hz) and wideband (2343.75 Hz) operation.** NB ↔ WB
  upgrade is negotiated at link setup; both sides must agree.
- **Adaptive gearshift** with SNR-based SUPERSHIFT for fast initial climb,
  ladder gearshift for fine adaptation, and a verification probe at the top
  config so the link never settles on a mode that doesn't actually decode.
- **SACK partial-batch recovery (Design A).** Selective ACKs with piggyback
  retransmits — the responder re-sends only the missing frames, mixed into the
  next outbound batch instead of as a separate retransmit phase. Negotiated via
  `CAP_SACK` (0x20) and `CAP_SACK_V2`; default ON.
- **Effective-rate optimizer (Phase 3c).** Loads a calibration table at startup
  and steers gearshift toward the highest effective-throughput config for the
  current channel conditions. Auto-disables on missing/invalid table.
- **Streaming batch compression.** PPMd8 (order 6, 2 MB) and zstd (level 3)
  compete per batch; best ratio wins. Streaming context (PPMd carry + zstd
  32 KB prefix) is preserved across batches and self-corrects on CRC mismatch.
  Negotiated via `CAP_COMPRESSION` / `CAP_STREAMING`.
- **End-to-end encryption.** X25519 ECDH key exchange with HKDF-Blake2b,
  ChaCha20-Poly1305 AEAD, direction-bound nonces. Optional pre-shared key via
  `-K <hex>`. Monocypher 4.0.2 vendored. Negotiated via `CAP_ENCRYPTION`.
- **Winlink B2F unroll/reroll.** Strips Winlink LZHUF on TX so Mercury's better
  PPMd/zstd compressor can do its job; rerolls on RX so the receiving Winlink
  client sees an unmodified message. `CAP_B2F_UNROLL` (0x04).
- **BREAK recovery.** Failsafe that releases stuck `messages_control` state and
  falls back to a known-good config when the link cannot make progress.
- **Passive monitor mode.** Decode both sides of a third-party session without
  transmitting (`--monitor` / `-m MONITOR`).
- **GUI** (Dear ImGui + GLFW): waterfall, constellation, level meters,
  callsign/SSID, encryption status, gearshift controls. Run headless with `-n`.

## Mode Reference

PHY rates sorted by WB throughput. NB PHY is roughly WB/5.

```
Mode          Modulation    LDPC    WB PHY    NB PHY    Es/N0
                            Rate     (bps)     (bps)    Waterfall
-----------   -----------   ----    ------    ------    ---------
ROBUST_0      32-MFSK       1/16       14        28    -13.0 dB
ROBUST_1      16-MFSK x2    1/16       22        37    -11.0 dB
CONFIG_0      BPSK          1/16       71        14    -10.0 dB
ROBUST_2      16-MFSK x2     1/4       87       149     -8.0 dB
CONFIG_1..6   BPSK         2..8/16    156..665   31..133
CONFIG_7..9   QPSK         5..8/16    763..1235  153..247
CONFIG_10/11  8PSK         6,8/16    1354,1818  271,364
CONFIG_12     QPSK         14/16     2261       452     +6.5 dB
CONFIG_13/14  8PSK         12,14/16  2471,3390  494,678
CONFIG_15     16-QAM       14/16     4361       872    +12.5 dB
CONFIG_16     32-QAM       14/16     5665      1133    +13.5 dB
```

## Build

Mercury builds on Linux (gcc/glibc) and Windows (MinGW64 POSIX via MSYS2).

### Native build

```
bash build.sh o3
```

Output is `mercury.exe` on Windows (installed to `C:\Program Files\Mercury\`)
or `mercury` on Linux. Other build modes: `release`, `debug`, `o0`/`o1`/`o2`,
`asan`, `ubsan`.

Linux dependencies:
```
apt-get install libasound2-dev libpulse-dev libglfw3-dev
```

### Cross-build for Raspberry Pi (aarch64)

```
MERCURY_CROSS_BUILD=1 MERCURY_SYSROOT=/path/to/pi/sysroot bash build.sh o3
```

Output is `mercury_aarch64`. See `build.sh` for the full sysroot layout.

### Tests

```
mercury.exe --test          # built-in unit tests; must pass before deployment
```

Loopback benchmark:
```
python tools/mercury_benchmark.py sweep --configs 0,8,16 --measure-duration 60
```

BER simulation for a single config:
```
mercury.exe -m PLOT_PASSBAND -s 10        # WB
mercury.exe -m PLOT_PASSBAND -s 10 -N     # NB
```

## Quick Usage

Basic ARQ session over WASAPI (Windows virtual cable):

```
mercury.exe -m ARQ -x wasapi -i "CABLE Output (VB-Audio Virtual Cable)" \
                            -o "CABLE Input (VB-Audio Virtual Cable)" \
                            -s 11 -g
```

ARQ on Linux ALSA with a stock HF radio + adaptive gearshift + MFSK hailing:

```
mercury -m ARQ -g -R -r stockhf -i "plughw:0,0" -o "plughw:0,0"
```

Encrypted session with PSK:

```
mercury -m ARQ -g -R -E strict -K 0123456789abcdef0123456789abcdef
```

Passive monitor:

```
mercury -m MONITOR --stdout
```

## CLI Flags (Selected)

| Flag                  | Purpose                                                       |
|-----------------------|---------------------------------------------------------------|
| `-m <mode>`           | `ARQ`, `MONITOR`, `PLOT_PASSBAND`, `TX_TEST`, `RX_TEST`, etc. |
| `-s <config>`         | Modulation: 0–16 (OFDM), 100/101/102 (ROBUST_0/1/2). `-l` lists. |
| `-M {auto,nb}`        | Bandwidth: `auto` (NB hail, WB upgrade) or `nb` (500 Hz only). |
| `-N` / `-W`           | Force narrowband / wideband.                                  |
| `-g`                  | Enable adaptive gearshift.                                    |
| `-R`                  | Enable ROBUST (MFSK) hailing.                                 |
| `-x <api>`            | Sound API: `wasapi`, `alsa`, `pulse`, `dsound`.               |
| `-i <dev>` / `-o <dev>` | Audio capture / playback device.                            |
| `-A <ch>`             | Audio channel index (multichannel mode).                      |
| `-z`                  | List available sound devices.                                 |
| `-T <dB>` / `-G <dB>` | TX / RX gain override.                                        |
| `-F {on,off,auto}`    | Compression. Default `auto` (B2F-triggered).                  |
| `-E {strict,fast}`    | Encryption: require / classical-first.                        |
| `-K <hex>`            | Pre-shared key (hex, up to 64 bytes).                         |
| `--gi <ms>`           | OFDM guard interval, 1.0–8.0 (default 3.0). Both sides must match. |
| `--no-sack`           | Opt out of SACK (default is ON).                              |
| `--enable-sack`       | No-op; kept for harness compatibility.                        |
| `--disable-sack-v2`   | Force CAP_SACK_V2 off (legacy v1 ACK behavior).               |
| `--no-optimizer`      | Disable effective-rate optimizer (for calibration runs).      |
| `-I <5..50>`          | LDPC decoder max iterations (default 50).                     |
| `-p <port>`           | TCP base port (control = port, data = port+1). Default 7002.  |
| `-n`                  | Headless (no GUI).                                            |
| `--log <file>`        | Redirect output to a log file (HF field debugging).           |
| `--stdout`            | Emit decoded plaintext on stdout (monitor / debug).           |
| `-v`                  | Verbose debug output.                                         |

`mercury -h` prints the full help.

## Environment Variables

Operational overrides and opt-in diagnostics. The LDPC selector below changes
decoder behavior only when explicitly armed; its production default is SPA.

| Variable                  | Effect                                                  |
|---------------------------|---------------------------------------------------------|
| `MERCURY_LDPC_MINSUM=0\|1\|scoped` | LDPC decoder policy: unset (and `scoped`) selects fixed-point NMS for cfg15/16/17 with exact SPA everywhere else — the default; `0` forces all-SPA; `1` preserves global min-sum behavior. |
| `MERCURY_LDPC_FIXEDPOINT=1` | With `MERCURY_LDPC_MINSUM=1`, select the historical global fixed-point min-sum arm; ignored by `0` and unnecessary for `scoped`. |
| `MERCURY_FIR_AVX2_EXACT=0` | Disable the byte-exact AVX2 FIR kernels and run the scalar authority (default-on on AVX2-capable x86-64; the output is byte-identical either way — the lever selects the implementation, not the result). |
| `MERCURY_HAIL_POLL=1`     | Emit `[HAIL-POLL]` trace lines for hail-poll cadence.   |
| `MERCURY_SACK_RX_TRACE=1` | Emit per-message SACK RX trace (commander + common).   |
| `MERCURY_RATE_TABLE=<path>` | Override the effective-rate optimizer's calibration table path. Optimizer auto-disables on miss. |
| `MERCURY_CROSS_BUILD=1`   | Build script: cross-compile for aarch64.                |
| `MERCURY_SYSROOT=<path>`  | Build script: target sysroot for cross-build.           |

## Documentation Pointers

- `mercury/fact-documents/` — investigation records, design docs, and
  cross-session context. Start with `fact-documents/README.md`.
- `mercury/fact-documents/EFFECTIVE_RATE_OPTIMIZER_DESIGN.md` — design of the
  Q-table optimizer (SACK + Q-table + BREAK architecture rationale).
- `B2F_UNROLL_REROLL.md` (workspace root) — Winlink B2F integration policy.
- `IONOS_BUTLER.md` (workspace root) — IONOS fading-channel testbed access
  protocol. All hardware access goes through the butler on `localhost:7700`.

## Compatible Clients

Mercury's ARQ mode uses a VARA-compatible TCP control protocol (control on
base port, data on base+1; default 7002).

- **mercury-connector** — simple ARQ client with hamlib:
  <https://github.com/Rhizomatica/mercury-connector>
- **HERMES-BROADCAST** — RaptorQ-coded broadcast:
  <https://github.com/Rhizomatica/hermes-broadcast>
- **hermes-net UUCP** — UUCP over Mercury:
  <https://github.com/Rhizomatica/hermes-net/tree/main/uucpd>
- Any VARA-compatible client should work against Mercury's TCP interface.

For the sBitx integrated radio stack, see HERMES:
<https://github.com/Rhizomatica/hermes-net>.

## License

GNU Affero General Public License v3.0. See `LICENSE` for the full text.

## Contact

Mailing list: <https://lists.riseup.net/www/info/hermes-general>

Maintainer / project contact: _TODO — add your preferred contact here._
