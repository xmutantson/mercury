# RRO visible-machine telemetry (incremental source producer)

Set `MERCURY_RRO_TELEMETRY=1` to emit a complete
`mercury.telemetry.snapshot` v1 datagram every 125 ms to `127.0.0.1:38429`.
`MERCURY_RRO_UDP_PORT` changes the loopback destination port. The producer is
off by default; it never listens for commands or binds a non-loopback address.
Serialization and UDP I/O run on a dedicated thread. The receive and ARQ paths
only publish bounded atomic observations.

The v1 envelope always includes all 43 established RRO metrics so an absent
measurement cannot silently become zero. This patch sources all 43 v1 fields: capture-buffer
occupancy/fill, processing load, four acquisition observations, configured FFT size,
four selected OFDM carrier-lattice fields,
LDPC iteration count and limit, an actually evaluated CRC result, two lifetime
CRC check/reject counters, ten ACK/HAIL control-correlator fields, and all
sixteen existing Gearshift controller states.
Source-backed event-like values expire to `inactive` after two seconds without
a new source write. This distinguishes an idle or stopped receive path from a
fresh measured zero. The receiver separately handles transport staleness and
disconnection.

The CRC counters increment only when the outer CRC was actually evaluated;
they are not a count of all receive attempts or a recent fault rate. They stay
available after the receive path becomes idle because they are lifetime counts
for this modem process. The paired FFT-size/LDPC-limit publication and the
multi-field Gearshift publication are read coherently by the sender; the
Gearshift check is bounded and emits `inactive` for a collided sample rather
than mixing controller moments.

Gearshift's last-batch classification and partial streak are updated once per
completed optimizer transaction from the actual clean/SACK/failed outcome;
`UNKNOWN` is reserved for a transaction with no sent-frame count. The streak
counts consecutive partial transactions and resets on any other outcome or
session reset. Neither value is inferred from the rolling optimizer rate.
`gearshift.target_config` is the controller's negotiated configuration, not an
inferred optimizer proposal. `gearshift.optimizer_target_config` is the
currently pending actionable optimizer switch; `NONE` is a real no-pending
state, not an extrapolation from the last decision. The Gearshift snapshot is
taken from `cl_arq_controller::process_main()` on
the controller's own thread at most eight times per second. Backoff remaining
is the largest unexpired floor-probe deadline, in the policy clock's
milliseconds; BREAK total counts actually emitted `send_break_pattern()`
bursts. `gearshift.optimizer_enabled` means the optimizer is enabled and
gearshifting is on, not that the optimizer currently owns the link. The
target values are not inferred from neighboring states. In-process two-peer sim
is intentionally excluded from the Gearshift hook because this v1 envelope
identifies one modem process, not two controllers.

Carrier geometry counts unique columns that carry at least one DATA or PILOT
cell in the selected OFDM grid. A column can appear in both category counts if
its role changes between symbols; the active count is their union. These are
configured carrier roles, **not** `nData`/`nPilots` cell totals or evidence that
the FFT, channel estimator, or demapper executed. MFSK and invalid grids clear
the observation, rather than reusing a stale OFDM lattice.

Acquisition's `timing_offset_samples` is the signed observed preamble-versus-
prediction residual on a verified batch/re-pin; initial full searches without a
prediction leave it unavailable. `frequency_offset_hz` is the accepted raw
OFDM fine-residual estimator output, not a reused last-good value, MFSK
estimate, or a claim about total RF offset. `coarse_metric` is emitted only
from actual correlator results, not the forced-delay/prediction sentinels in
`receive_stats`. These values expire independently after two seconds.

The control-correlator window counter counts passband submissions to the ACK
and HAIL detector entry points, not every distinct FFT symbol window inside a
submission. Its invocation counter counts actual calls to the common sliding
detector within those submissions, including CFO refinement and diagnostic
re-search on the same submitted input. Prekey, connect, configuration-tag and
BREAK uses of that common detector are excluded. The invocation delta is the
publisher's eight-Hz interval difference, never a second lifetime counter.
Memo reuses count coarse symbol-power cache reads after the first use of each
cached symbol in that invocation; the fine search and non-memoized combining
path are not counted as reuses. `memo_enabled` reports the detector's effective
configuration (environment setting or test override), even when a particular
repetition-combining invocation cannot use the cache. Cumulative counts remain
available across idle intervals, while the interval delta correctly falls to
zero. The source snapshots submission and invocation counters as one stable
observation, so an in-flight first call cannot fabricate a calls/window spike.
These fields do not measure FFT execution time, queue depth, or signal
match quality.

All 43 existing wire fields being sourced is not full visible-machine coverage.
In particular, RRO's Channel
Estimator, Demapper, and ARQ modules still have no v1 wire bindings, and the
remaining stateful cues require source and scene-contract extensions. Do not
replace their `NOT INSTRUMENTED` indication with illustrative motion or assume
that this PR makes the lab machine diagnostically complete.

Validation on Windows: the telemetry unit executable compiles with
`-Wall -Wextra -Werror`, all touched Mercury translation units pass focused
syntax checks in the supported GUI configuration, and the emitted JSON passes
RRO's 43-binding adapter dry run, including stale/disconnected transport and
validated recording/replay. This is not a full modem performance qualification;
measure receive-loop and render cost before enabling it in VR.
