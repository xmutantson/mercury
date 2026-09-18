# Quicksilver runtime feature-gate audit

Status: first production-readiness pass on `gearshift-v2` after CONFIG_TAG was restored as the Gearshift-v2 ACTIVE transition transport.

This document exists because Quicksilver has accumulated several proven fixes, experiments, fail-before knobs, and protocol migrations behind similarly named environment gates. A gate being present is **not** evidence that it should remain off, and a comment saying "experimental" is **not** evidence that it is safe to enable. Classify from measured behavior and protocol role.

## Classification

- **A — proven / production baseline:** should be on by default. Keep an explicit escape hatch when useful for A/B or interoperability.
- **B — useful but legitimately opt-in:** credible mechanism, but deployment evidence or protocol migration is incomplete.
- **C — diagnostic / test-only:** traces, simulators, fail-before, and defeat knobs. They are not production features.
- **D — superseded / removal candidate:** no longer the live mechanism; retain only while needed for compatibility or forensic replay.
- **E — dangerous / keep off:** measured harmful, deliberately quarantined, or intentionally reproduces a broken state.

## Rate selection and transition transport

| Gate / mechanism | Current default | Class | Evidence / decision |
| --- | --- | --- | --- |
| CONFIG_TAG transport under Gearshift-v2 ACTIVE | ON whenever `rate_opt.controls_link()` | **A** | ACTIVE owns *what config to choose*; CONFIG_TAG owns intra-tier transport and peer-follow confirmation. CONFIG_0→CONFIG_16 is OFDM→OFDM and must not depend on the ACK-fragile legacy SET_CONFIG handshake. The ACTIVE path therefore ignores an omitted `MERCURY_INBAND_RATE`. |
| `MERCURY_INBAND_RATE` under legacy selectors | opt-in | **B** | This legacy gate enables more than the CONFIG_TAG codec: D1 re-tag, D2 NACK, no-BREAK demotion/recovery, periodic re-announce, and related lifecycle. Do **not** globally flip the legacy gate merely to make the transport default-on. If legacy also needs unconditional CONFIG_TAG, split transport enablement from the broader legacy policy first. |
| `MERCURY_GEARSHIFT_V2=active` | opt-in | **B** | ACTIVE is the branch under physical knee validation. Do not make it the universal default until WGN/MPP/MPD regression evidence is clean. |
| `MERCURY_CLIMB_TIER2` | opt-in | **B** | Source explicitly says C2/C3 are deferred pending a decode-margin gate. Tier-1 remains the production baseline. |
| `MERCURY_SCREAM_WAKE` | opt-in | **B** | Separate wake/re-entry behavior that uses CONFIG_TAG for handoff. CONFIG_TAG being production transport does not imply SCREAM wake itself is ready to become baseline. |

## Timing, ACK, and recovery

| Gate / mechanism | Current default | Class | Evidence / decision |
| --- | --- | --- | --- |
| `MERCURY_LINKPHASE_OPTCLOCK` | ON; `=0` disables | **A** | Uses emitted DATA-keydown timing instead of the frozen 1800 ms optimizer clock. Source already marks it default-on. |
| `MERCURY_TURNAROUND_REPHASE` | ON; `=0` disables | **A** | Source records a hardware A/B CFG15 whole-window ~3.4x win and deliberately scopes the change to CFG15 so measured CFG16 harm remains excluded. |
| `MERCURY_CUMULATIVE_ACK` | ON; `=0` disables | **A** | Code already defaults the capability bit on and engages wire behavior only when both peers advertise support. The nearby stale "default-off" comment was corrected in the same audit commit. |
| `MERCURY_MFSK_ROBUST_PREAMBLE` | robust/default mode ON; `short|off` disables advertisement | **A** | RX carries both sequence sets and advertises capability by default; explicit legacy modes suppress the promise. |
| `MERCURY_LINKPHASE_CONFIG_CONTRACT` | opt-in | **B** | Source explicitly calls this a Phase-0 protocol-contract migration that remains default-off for live traffic. Finish migration evidence before changing default. |
| `MERCURY_RECOVERY_ACK_ROBUST` | opt-in | **B** | Repeats and noncoherently combines the recovery control ACK to lift marginal 6–7/16 matches without lowering the acceptance bar. Mechanism is plausible, but the source does not document a production closed-loop verdict sufficient to promote it here. |
| `MERCURY_RECOVERY_ACK_REPHASE` | opt-in | **B** | Forensics identify capture-window misplacement (silent-absent / late-truncated) and the lever is RX-only, but it remains a recovery-specific experimental correction pending a clean live verdict. |
| `MERCURY_L1_BLOCKACK_PIPELINE` | opt-in | **B** | Pipeline timing is meaningful only when block-ACK is armed. Keep coupled/opt-in until the complete block-ACK mode is production baseline. |
| `MERCURY_KARN_RETX_ONLY` | opt-in | **B** | Corrects retransmit ambiguity in the measured-turnaround estimator, but remains an estimator-policy migration rather than a prerequisite for the current CONFIG_TAG root fix. |
| `MERCURY_SPEC_SACK` | opt-in | **B** | Advances the partial-SACK decision at a deadline while preserving the existing ACK gate. It is explicitly speculative and not needed for the WGN25 transition regression. |
| `MERCURY_RECOVERY_ACK_FINE` | OFF | **E** | Source records the deployed hardened fine detector as **falsified on 51 real HW buffers** and says live ON delivered **zero bytes**. Offline rescoring only recovered a small detector-only subset. Keep off unless new end-to-end evidence supersedes that verdict. |

## Known-broken / quarantine

| Gate / mechanism | Current default | Class | Evidence / decision |
| --- | --- | --- | --- |
| `MERCURY_LOSSY_DEMOTE` | OFF | **E** | Source labels it **QUARANTINED**, **proven-broken**, and **do-not-enable**; it intentionally reproduces the pre-fix non-contiguous re-present / GAP-ABORT behavior. |
| `*_DEFEAT`, `*_FAILBEFORE`, `*_DISABLE` regression knobs | normally unset | **C** unless specifically documented otherwise | These exist to reproduce a historical failure or disable a shipped fix on the same binary. Do not interpret their existence as a production feature choice. |
| `MERCURY_SACK_RX_TRACE`, `MERCURY_GS2_PRIMITIVE_TRACE` and similar trace knobs | OFF | **C** | Observability only; keep out of production hot paths unless collecting evidence. |
| simulator-only / in-process stepper gates | OFF outside tests | **C** | Harness controls, not modem feature policy. |

## Removal / split candidates

There is no feature in this first pass that should be deleted blindly. The strongest **D-candidate is conceptual**: the name `MERCURY_INBAND_RATE` conflates **transport enablement** with a broad legacy in-band recovery policy. Gearshift-v2 ACTIVE now bypasses that ambiguity by making CONFIG_TAG transport unconditional. If legacy mode is later migrated to CONFIG_TAG by default, split the narrow transport capability from legacy selection/recovery gates rather than turning the entire old bundle on at once.

## Debugging rule

When a modem behavior appears inexplicably primitive or regresses to a known old failure mode:

1. Identify the decision owner and the transport/recovery mechanism separately.
2. Run `python3 tools/audit_default_off_features.py` and inspect candidate gates near that path.
3. Check source comments, fail-before tests, commit history, simulator evidence, and physical A/B evidence before changing a default.
4. Promote only **A** features. Keep **B** opt-in, **C** test-only, and **E** off.
5. Add a regression that fails if a future patch silently routes around the promoted mechanism.

This audit is intentionally conservative: "we have code for it" is not the same as "ship it."
