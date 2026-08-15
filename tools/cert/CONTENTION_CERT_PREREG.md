# Per-host contention certification preregistration

Status: immutable build contract; launch withheld. Box 31 is occupied during this
build window. No cohort, smoke, broker lease, or synthetic load was started.

Protocol version: `contention-cert-v1`  
Contract SHA-256: `6d0c7cc3fad0a3d26461c96997b4889414053cf68db93ea8f9d1a1be062c61b8`

## Purpose and unit of certification

This experiment measures a safe concurrent real-audio cell limit separately on
each fleet host. A result applies only to the host named in its manifest. It may
not be copied to another host, pooled across hosts, or used to infer a fleet-wide
cap.

The width sweep is an A/A load experiment: every scored cell uses the same
binary, harness, real-audio recipe, environment, channel dial, payload, and
seed bank. Width is the only planned change. The fixed-width synthetic-load arm
tests whether a width association is explained by host contention rather than
width itself.

## Immutable design

- Target widths: `2, 4, 8, 12, 16` concurrent live real-audio cells.
- Scored seeds at every width: `1..16`, exactly once each.
- Scored denominator: exactly 16 cells per width. No result may be substituted.
- Channel: WGN, commanded `--snr 28 --snr3k 28`, steady P-signal axis.
- Recipe: config 100, auto mode, warm start, random-binary traffic, 262144-byte
  payload, 800-second run and score horizon, two-second launch stagger.
- Measurement environment: `MERCURY_SIM_PSIG_MODE=steady`,
  `MERCURY_TURN_TRACE=1`, and unbuffered Python output. Turn trace is
  measurement-only and is not a scoring source.
- Width order: `2, 4, 8, 12, 16`, followed by the nonzero synthetic-load cells.
- Fixed-width synthetic-load arm: width 8 at stress-ng CPU load 25%, 50%, and
  75%. The unloaded width-8 A/A block is the registered 0% baseline. Each load
  level uses scored seeds `1..16` exactly once.
- stress-ng method: `--cpu-method all`, pinned to the preregistered site CPU
  list, one stressor per listed CPU, with a 15-second warmup. The CPU list is a
  site parameter recorded in the manifest and may not change after preflight.
- Outcome retries: zero. There is no resume mode. Any stale run directory or
  archive directory is a hard failure.

Every wave must realize its named width. The driver therefore requires exactly
W simultaneous native one-card leases before launching a width-W wave; a short
grant is not run as W. Widths 2, 4, 8, and 16 divide the scored seed bank.
Width 12 does not, so it runs two complete width-12 waves: seeds 1–12 in the
first, then scored seeds 13–16 plus eight preregistered load-only companions in
the second. The companions use deterministic seeds 10001–10008. They do not
enter the immutable 16-cell score denominator, but they are conservative safety
sentinels: a primary-meter exceedance on any companion trips width 12. They can
only make certification stricter.

The complete plan is 8 jobs, 23 exact-width waves, 128 scored cells, and 136 live
cells. `python3 contention_cert_driver.py plan --box N` emits the canonical plan.

## Primary rule

Only two native per-cell result fields determine the knee:

1. `bridge_underruns.total`, with its native `fwd` and `rev` components required
   to be nonnegative integers satisfying `total = fwd + rev`.
2. `rx_overrun_total`, the session-cumulative RX-overrun count introduced at
   the source. The registered form is a nonnegative integer. A later
   producer-preserving object is acceptable only if it contains a nonnegative
   integer `total`.

Both epsilons are preregistered at zero:

```text
cell_hot = (bridge_underruns.total > 0) OR (rx_overrun_total > 0)
width_hot(W) = any live cell at unloaded width W is cell_hot
knee = min W in {2,4,8,12,16} for which width_hot(W)
```

No log grep, storm count, timing signature, throughput, completion state, or
post-hoc classifier may alter this primary rule. A missing, null, malformed, or
internally inconsistent primary field invalidates the run; it is never read as
zero.

If no width is hot, the knee is right-censored above 16. That is reported as
“knee not observed through width 16,” not as a measured knee of 18 or infinity.

## Secondary rule

`connected`, `delivered_full`, and `byte_integrity_ok` are reported for every
scored cell. They do not move the primary knee. A broker policy is derivable
only if all scored cells connected, fully delivered, and passed byte integrity.
Failure of this secondary gate yields an invalid policy result and no cap,
without retrying the affected outcome.

Load-only companions must also produce complete, parseable result JSON with
both primary meters. Their completion and integrity values are retained but do
not enter the 16-cell secondary denominator.

## Fixed-width discriminator

The width-8 unloaded block and width-8 25/50/75% stress blocks are reduced with
the identical primary rule.

| Unloaded width knee | Loaded width-8 event | Registered interpretation |
|---|---|---|
| observed | observed | contention supported |
| observed | not observed | width per se supported |
| not observed | observed | contention-sensitive without a width knee |
| not observed | not observed | no primary event observed |

This is a mechanism discriminator, not a license to replace the primary knee
with a stress threshold. Completion and integrity remain secondary.

## Broker policy derivation

The preregistered safety margin is two concurrent cells. When a finite knee is
observed and the secondary gate passes:

```text
broker_width_cap_cells = knee_cells - 2
```

The driver writes `CERT_MEASUREMENTS.json` and `BROKER_POLICY.json` side by
side. The policy record contains the measured knee, margin, literal formula,
derived per-host cap, target box, contract hash, and measurement hash.

If the knee is not observed through 16, the formula cannot produce a measured
cap. Extend the preregistered width range in a new protocol or retain the
existing cap. If the knee is 2, no positive cap is certified. If the secondary
gate fails, no cap is emitted. No clipping, interpolation, cross-host borrowing,
or post-hoc margin change is allowed.

## Lease and wave contract

The driver uses the native broker shape once per live cell:

```text
REQ <agent> box=<N> cards=1 cpu=<site_cpu_per_cell> ttl=<s>
GRANT <lease> box=<N> cards=<card_id> slot=<0|1>
```

Each lease has its own heartbeat and is released in a `finally` path. The
granted `(card_id, slot)` maps to the explicit spawner plan as card
`Loopback[_HEX]` and substreams `slot*4 + {0,1,2,3}`. Duplicate grants, a grant
on another box, an invalid slot, heartbeat failure, non-OK release, nonzero
spawner return, missing result, unexpected result, or schema mismatch fails the
campaign. There is no partial-width adaptation.

Certification may require a temporary, recorded broker capacity window capable
of granting 16 simultaneous one-card leases on the dedicated target box. This
window is an experiment-enablement override, not the resulting policy. Capture
broker `STATUS` before and after, run no other workload on the box, and restore
the standard broker configuration before applying the derived cap.

## Landmine preflight

Before a launch review, run the mechanical gate under
`/mnt/c/mercury_codex_stage/cohort_preflight/`. The driver emits the gate's exact
experiment spec and invokes `preflight.py <spec>`. That gate queries live broker
caps and runs the one-cell exact-recipe smoke, so it is intentionally part of
launch preflight rather than offline package verification. Any nonzero result is
binding. The gate became available during this build, after the no-launch hold
was established; its live smoke was therefore not run and no passing fleet gate
is claimed.

The launch checklist remains binding even after the mechanical gate passes:

- Confirm the target box is dedicated and idle; `.31` is explicitly unavailable
  during this build window.
- Confirm `--snr` and `--snr3k` are both passed and the result attests commanded
  SNR 28 on the steady axis.
- Do not add `--no-gearshift`; it does not weld BREAK demotion. No behavioral
  pin is part of this A/A contract.
- Confirm every `MERCURY_*` value reaches both cells. Do not assume an absent
  lever is off or invert default-on `=0 disables` semantics.
- Query broker status and capacity; never infer per-lease or per-box caps from a
  driver constant. Confirm exact-width 16 is grantable only inside the recorded
  certification window.
- Compare builds with functional and provenance gates. Do not require SHA
  equality across independently built boxes.
- Keep active work under `/dev/shm`, not WSL `/tmp`; retain final archives on a
  persistent filesystem.
- Verify a one-cell exact-recipe smoke contains every reducer field, including
  native `bridge_underruns` and `rx_overrun_total`, plus arm/environment and
  channel attestation.
- Verify staged executable bits and imports after archive extraction.
- Verify every failure reaches the top-level return code. A success marker is
  forbidden when any wave, cell census, schema, cleanup, archive, or policy
  gate fails.
- Record the committed harness identity. Do not stage an uncommitted harness
  snapshot or selectively flatten its tree.
- Confirm stress-ng and taskset exist, every pinned CPU is online and within the
  driver's affinity, and the selected CPUs are the intended contention domain.
- Confirm `/dev/shm` and persistent archive capacity, ALSA loopback topology,
  TCP ports 7100–7554, heartbeat TTL, and uninterrupted campaign time.
- Move old paths aside; never delete or reuse them as a fresh campaign.

The generated preflight record is host-bound, box-bound, contract-hashed, and
valid for at most 24 hours. The run command additionally requires the literal
approval token `CONTENTION_CERT_APPROVED`. These gates prevent an accidental
launch from this build package.

## Analysis and retention

Raw `res_*.json`, aggregate spawner JSON, spawn logs, stress logs, event lines,
broker grants, hashes, plan, and compact cell rows are retained. The driver does
not rewrite raw result JSON. `CELL_RESULTS.jsonl` records the raw result hash and
the normalized primary totals. `EVENTS.jsonl` timestamps lease, stress, wave,
and meter-read observations with monotonic milliseconds; observation time is not
an emitter timestamp.

The reducer is deterministic and may be rerun with:

```sh
python3 contention_cert_driver.py reduce --run-dir /path/to/run --box N
```

The persistent tar archive and its SHA-256 sidecar must verify before a policy
change. After collection, verify no owned campaign process or stress-ng process
remains, the broker shows no campaign lease, the temporary capacity window is
restored, and the target host is idle.

## Final committed-capability addendum — 2026-08-14

The text above is preserved verbatim and its immutable experiment semantics
remain binding. Monitor commit
`41554479c0bc56639be32a094f04ba34073bda96` commits the complete campaign
dialect: warm start, deterministic random-binary traffic with byte-integrity
oracle, fixed score horizon, independent `--snr` and `--snr3k` dials, seed
offsets, explicit `--spawn-plan` injection, and `byte_integrity_ok` results.

The final driver passes the full recipe directly to the committed spawner:
`--warm-start --traffic random-binary --score-horizon-s 800 --snr 28
--snr3k 28 --seed-offset 0`, plus the broker-derived explicit spawn plan. The
spawner's `--dry-run` is the binding committed-dialect check because it runs
argparse, validates every plan, and constructs every child command without
starting a cell. Automatic gearshift remains the committed default;
`--no-gearshift` is still omitted.

The round-3 corrections remain binding. Broker `STATUS` is decoded with
`json.loads` after its optional `OK ` envelope and every box must report integer
`active_leases == 0`. The driver and preregistration remain tracked at
`tools/cert/`. Bridge underruns are read from `bridge_<tag>_stats.json`, and a
scalar `rx_overrun_total` of zero is valid scored data. A simulated preflight
record is explicitly marked non-launch-eligible; only a fresh live smoke and
live broker preflight can authorize the approved campaign entry point.
