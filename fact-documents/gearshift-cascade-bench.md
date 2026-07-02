# Gearshift Cascade Benchmark — Design

**Created**: 2026-05-28
**Branch**: monitor
**Owner**: gearshift_cascade_bench.py harness
**Status**: design v1, awaiting hardware run

> **CORRECTION (2026-07-01):** This benchmark is an **announcement-keyed climb
> scorer** — it grades the cascade by parsing `[GEARSHIFT]`/`[POLICY-MOVE]`
> log-line *announcements* (§5/§6). Announcement-keyed climb scorers were
> **DEPRECATED** by commit `fc9153e` ("scoring: deprecate announcement-keyed
> climb scorers; canonical audit record") in favor of the canonical audit
> record, because a logged announcement does not prove the RSP followed the
> config (the CMD-vs-RSP lag flagged in §7 is exactly the blind spot). Prefer the
> canonical audit record for climb scoring; keep this doc as the log-marker
> reference. Anchors below have also drifted — see the re-anchor note in §6.

## §1. Purpose & Thesis

Validate the strategic claim made by the mini-Moose / Phase 0 stack
(MEMORY.md §"MFSK vs VARA HF parity audit"): **deepening the ROBUST_0
floor benefits higher-rate modes via the gearshift cascade**.

The mechanism: Mercury's commander starts at ROBUST_0 after `HAIL` and
walks UP to the channel's working point via two paths:
1. **SUPERSHIFT** — fast jump to `get_configuration(SNR - SUPERSHIFT_MARGIN_DB)`
   immediately after HAIL/turbo-reverse (`arq_commander.cc:1978,3787,3981,4241`)
2. **LADDER UP** — per-block ACK-rate climb after data starts flowing
   (`arq_commander.cc:4711`, `[POLICY-MOVE] reason=ladder_up`)

The thesis says: if ROBUST_0 is the entry floor and that floor is robust
enough to survive HAIL at a given SNR, the cascade can then reach the
correct working config. We need a single-purpose benchmark that
measures this — not a per-config cliff sweep (`axis_walk_sweep.py`
already does that) but a "hold WGN, watch config climb" measurement.

## §2. What this is NOT

- Not a comparison vs a "baseline" build. Single sweep, single arm. We
  are measuring an absolute property: "given WGN:X, does cascade reach
  CFG_N within T seconds?"
- Not a throughput benchmark. We do report bps (sanity check that the
  link is alive) but cannot grade post-cascade throughput here without a
  paired pinned-mode reference run, which is `axis_walk_sweep.py`'s job.
- Not a per-config debug tool. Use `axis_walk_sweep.py --pin-config` for
  that.

## §3. Mercury Flag Set (audited 2026-05-28)

From `mercury/source/main.cc` + `arq_commander.cc`:

```
mercury -m ARQ -x alsa -i plughw:Audio -o plughw:Audio --rx-channel 1 \
        -Q 0 -M auto -g -R -n -v -F off \
        --enable-sack --enable-sack-v2
```

| Flag | Source line | Semantic | Why we use this value |
|---|---|---|---|
| `-m ARQ` | main.cc:?? | ARQ mode | Gearshift only fires in ARQ mode |
| `-g` | main.cc:1022 | `gear_shift_mode = GEAR_SHIFT_ENABLED` | Required for Axis-1 to move; the cascade IS Axis-1 |
| `-R` | main.cc:1080 | `robust_mode = 1` | Enables MFSK hailing + lets BREAK descend into ROBUST tier (otherwise BREAK floors at CONFIG_0 and the link can deadlock at low SNR — see `BREAK_FAILSAFE_INVESTIGATION.md`) |
| no `-s` | main.cc:2038-2044 | `mod_config = ROBUST_0` when `-g` and no explicit config | THE cascade-start condition. This is the production behaviour we're validating |
| `-Q 0` | main.cc:1117,2052 | `nb_probe_max = 0`, sets `narrowband_enabled = NO` | Skip NB probes → straight to WB HAIL → cleaner cascade timeline (no ~30s NB-probe noise at top of log) |
| `-M auto` | main.cc:1132 | `bandwidth_mode = BW_AUTO` | Required for `-Q 0` to take effect (main.cc:2052 conditional) |
| no `--skip-turbo-reverse` | main.cc | `skip_turbo_reverse = 0` (default) | We *want* reverse turbo — it's part of the SUPERSHIFT path that walks UP from ROBUST_0 |
| `-F off` | main.cc | compression off | Apples-to-apples PHY throughput; compression amplifies bps and obscures the cascade |
| `-v` | main.cc | verbose | Required to get `[GEARSHIFT]`/`[TURBO]`/`[POLICY-MOVE]` lines |
| `--enable-sack --enable-sack-v2` | main.cc | match steady-state | Matches axis_walk + f2_ab harness — capability set matters for upper-tier reachability |

## §4. Recommended Channel Conditions

Two single-cell runs, ~10 minutes each (60s setup + 60s settle + 300s dwell + 60s teardown).

### §4.1. WGN:+6 — "easy" cell

**Expectation**: Channel can hold CFG_10 confidently, CFG_15 plausibly
(see MEMORY.md "Verified Throughput": CONFIG_15 WB ~3785 bps historical).
At WGN:+6 the Phase 0 stack has plenty of margin over the ROBUST_0
floor (-12 to -8 dB Eb/N0).

**Pass criterion**: `max_config >= CFG_10 AND time_to_max <= 60s`

If this fails, the cascade is broken — channel margin is ample. Look at
SUPERSHIFT line: did it fire? did it aim high enough? did SET_CONFIG ACK?

### §4.2. WGN:+0 — "harder" cell

**Expectation**: CFG_10 is cliff-edge per pre-Phase-0 data (CONFIG_12
fails at 0-45% per MEMORY.md). Mini-Moose's deeper floor SHOULD let
the cascade still escape ROBUST_0 and at least reach CFG_4.

**Pass criterion**: `max_config >= CFG_4 AND time_to_max <= 90s`

If max_config stays at ROBUST_0 here but escapes at WGN:+6, we have a
clear "where does the cascade start working" data point — useful to
compare against Phase 0's cliff move (WGN:0 → WGN:-4 to -6).

### §4.3. Why not WGN:-4 / lower?

Below WGN:0 the channel itself starts pinning ROBUST_0 (mini-Moose
floor). If the cascade can't escape at WGN:-4, we can't tell whether
that's "cascade bug" or "channel doesn't support CFG_4 here". Save
sub-zero runs for a follow-up where we compare against a pinned-mode
reference (post-Phase-0 axis_walk cliff data).

## §5. Interpretation Rules

Applied by `tools/gearshift_cascade_bench.py` `interpret()` and
restated here as the authoritative source.

The script parses `[GEARSHIFT]`/`[TURBO]`/`[POLICY-MOVE]` lines from
the CMD-side log, computes a step-function `config(t)`, and reports
`max_config` (highest reached) and `time_to_max_s` (when first
reached, relative to CONNECT anchor).

| Max config reached | Bucket | Verdict | Action |
|---|---|---|---|
| ≥ CFG_15 | `CFG_15_OR_HIGHER` | **cascade_full_pass** | Mercury can reach near-peak throughput on this channel; mini-Moose stack functioning end-to-end. |
| CFG_10..14 | `CFG_10_TO_14` | **cascade_functional** | Gearshift works but ceiling is rate-0.625 LDPC. Acceptable post-Phase-0; cascade is healthy. |
| CFG_4..9 | `CFG_4_TO_9` | **cascade_partial** | Cascade works to CFG_4 only. Check `[TURBO] SNR-SUPERSHIFT` target — if it aimed higher than reached, LADDER UP is stuck. |
| CFG_0..3 | `CFG_0_TO_3` | **cascade_minimal** | Reached OFDM tier but barely. Likely SUPERSHIFT didn't fire or LADDER UP rejecting blocks. Inspect raw transitions for SUPERSHIFT/SET_CONFIG presence. |
| ROBUST_0/1/2 (100/101/102) | `ROBUST_TIER_ONLY` | **cascade_not_firing** | Stayed in ROBUST tier — gearshift cascade BUG. Check that `[TURBO] SUPERSHIFT` and `[GEARSHIFT] SET_CONFIG` lines appear at all. If they don't: cascade is silent. If they do but config_loaded never advances: SET_CONFIG ACK path is broken. |

**Time-to-max secondary criterion**: clean channels should reach max
within 30s (one SUPERSHIFT + a few LADDER UP blocks). > 60s on a clean
channel means the cascade is climbing one rung at a time when it
should be jumping — investigate SUPERSHIFT path (commit history around
`turboshift_last_good`).

## §6. Log Markers Parsed

All sourced from `mercury/source/datalink_layer/`:

| Regex | Source | Semantic |
|---|---|---|
| `\[GEARSHIFT\] SET_CONFIG: forward=(\d+) reverse=(\d+)` | ~~`arq_commander.cc:545`~~ `arq_commander.cc:1201` (re-anchored 2026-07-01) | Authoritative "next config" announcement on commander |
| `\[TURBO\] (?:SNR-)?SUPERSHIFT[^\n]*config (\d+) -> (\d+)` | `arq_commander.cc:1978,1985,3787,3794,3981,3988,4241,4255` [?] verify (drifted) | Fast-jump path (post-HAIL or post-reverse-turbo) |
| `\[POLICY-MOVE\] axis=1 from=(\d+) to=(\d+) reason=ladder_up` | ~~`arq_commander.cc:4715`~~ `arq_commander.cc:8057` (re-anchored 2026-07-01) | Per-block ACK-rate climb (slower than SUPERSHIFT) |
| `\[POLICY-MOVE\] axis=1 from=(\d+) to=(\d+) reason=ladder_down` | ~~`arq_commander.cc:4791`~~ `arq_commander.cc:8135` (re-anchored 2026-07-01) | Per-block ACK-rate descent |
| `\[GEARSHIFT\] FRAME UP[^\n]*config (\d+)[^\n]*BREAK to (\d+)` | `arq_commander.cc:2210,3011,3181` | Frame failure → BREAK to lower config |

Wall-time anchor: we inject `### CASCADE_MARK t=<epoch>` into both logs
every 10s during the run, plus a `### CASCADE_CONNECT_ANCHOR` line
right after the CONNECTED ack. `parse_transitions()` linearly
interpolates file-position → wall-time between marks. Resolution ≈ 1
second over a 5-minute window; sufficient since cascade events are
seconds-apart at worst.

## §7. Open Questions

- **[?]** Does the CMD-vs-RSP config-state lag matter for the timeline?
  `[GEARSHIFT] Received SET_CONFIG` on the RSP side trails CMD by ~1
  batch. We report CMD-side timeline; RSP could lag by 1-2 seconds.
  Not significant at our resolution.
- **[?]** Should we report TIME-IN-FLIGHT bps per config (transition i+1.t
  minus transition i.t)? Useful for comparing rate-vs-config but adds
  complexity. Deferred to v2.
- **[?]** Should we run a third cell at WGN:+12 to confirm CFG_16 is
  reachable? Adds 10 minutes. Decision: skip on first run; add if WGN:+6
  hits CFG_15 cleanly and we want to verify CFG_16 cap.

## §8. What This Doesn't Test

- Does NOT verify post-cascade steady-state bps matches pinned-mode bps.
  For that, pair with `axis_walk_sweep.py --pin-config --config N` at
  the same WGN and compare.
- Does NOT verify the RSP-side cascade — only commander. The RSP follows
  via SET_CONFIG; if SET_CONFIG ACK is broken, the RSP sticks and the
  link dies. But that failure mode shows up as bps=0 with config climbed
  on CMD side — visible in the JSON output.
- Does NOT measure how fast the cascade RECOVERS from a BREAK. That's a
  drop-and-recover test, separate scope.

## §9. Companion Doc

- `axis_walk_sweep.py` — per-config cliff sweep (holds config, walks
  WGN). Complementary; this benchmark holds WGN and watches config.
- `mfsk-vara-parity-plan.md` §Phase 0 — the cascade hypothesis under test.
- `BREAK_FAILSAFE_INVESTIGATION.md` — why `-R` is required to avoid the
  ROBUST_0-floor BREAK deadlock.
