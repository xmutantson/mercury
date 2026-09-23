# RRO visible-machine telemetry (incremental source producer)

Set `MERCURY_RRO_TELEMETRY=1` to emit a complete
`mercury.telemetry.snapshot` v1 datagram every 125 ms to `127.0.0.1:38429`.
`MERCURY_RRO_UDP_PORT` changes the loopback destination port. The producer is
off by default; it never listens for commands or binds a non-loopback address.
Serialization and UDP I/O run on a dedicated thread. The receive and ARQ paths
only publish bounded atomic observations.

The v1 envelope always includes all 43 established RRO metrics so an absent
measurement cannot silently become zero. This patch sources 20: capture-buffer
occupancy/fill, processing load, OFDM candidate admission, configured FFT size,
LDPC iteration count and limit, an actually evaluated CRC result, and twelve
Gearshift controller states. The remaining 23 report `not_instrumented`.
Source-backed event-like values expire to `inactive` after two seconds without
a new source write. This distinguishes an idle or stopped receive path from a
fresh measured zero. The receiver separately handles transport staleness and
disconnection.

The Gearshift snapshot is taken from `cl_arq_controller::process_main()` on
the controller's own thread at most eight times per second. Backoff remaining
is the largest unexpired floor-probe deadline, in the policy clock's
milliseconds; BREAK total counts actually emitted `send_break_pattern()`
bursts. `gearshift.optimizer_enabled` means the optimizer is enabled and
gearshifting is on, not that the optimizer currently owns the link. The
currently omitted target, batch classification, partial streak, and optimizer
target are **not** inferred from neighboring states. In-process two-peer sim
is intentionally excluded from the Gearshift hook because this v1 envelope
identifies one modem process, not two controllers.

This is not full visible-machine coverage. In particular, RRO's Channel
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
