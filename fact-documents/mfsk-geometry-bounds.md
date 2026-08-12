# MFSK carrier-grid geometry bounds

## Scope and defect signature

Base: `26c82b3cb459bd33c9cce4e3d22b4e3f1d426d80`.

MFSK gives each stream `M` adjacent carrier bins. For a valid initialized
geometry, every local carrier index must satisfy:

```text
0 <= stream_offsets[st] + tone < Nc
0 <= st < nStreams
0 <= tone < M
```

The prior sweep accepted `M32 x 2` with `Nc=50`. Because an overfull grid is
left-aligned, its offsets were `[0,32]`. The first invalid carrier is index 50
at stream 1, tone 18; the maximum generated index is 63. Narrowband `M8 x 2`
with `Nc=10` similarly spans indices 0 through 15. Both configurations reached
TX, RX, and correlator indexing sites despite arrays being allocated for `Nc`
carriers.

The necessary and sufficient packed-grid bound is:

```text
M * nStreams <= Nc
```

The implementation tests it without multiplication overflow as
`M <= Nc / nStreams`, after requiring positive dimensions and
`nStreams <= MAX_STREAMS`.

## Producers

- `source/physical_layer/telecom_system.cc:12593-12608` reads the test/search
  overrides for `M`, `nStreams`, and wideband `Nc`.
- `source/physical_layer/telecom_system.cc:12832-12847` derives the requested
  tuple and rejects it before publishing `current_configuration` or starting
  subsystem initialization.
- `source/physical_layer/telecom_system.cc:13216-13237` performs the
  pre-initialization call with the selected carrier count rather than the
  `AUTO_SELLECT` sentinel; `:13274-13286` repeats initialization after OFDM
  finalizes `Nc`.
- `source/physical_layer/mfsk.cc:30-88` and `:619-627` produce the inert state.
  `source/physical_layer/mfsk.cc:95-174` is the only producer of a live direct
  geometry and of `stream_offsets[]`.
- `include/physical_layer/mfsk.h:509-518` and
  `source/physical_layer/telecom_system.cc:13507-13510,13908` copy complete
  geometries into and out of precooked bundles. Those bundles originate from
  the guarded configuration path.
- `source/physical_layer/telecom_system.cc:13394-13403,13608-13614` initializes
  the separate ACK geometry. Its fixed WB `M16 x 1 / Nc50` and NB
  `M8 x 1 / Nc10` tuples satisfy the same direct-initialization guard.

## Consumers

- MFSK TX, control-pattern, and preamble writers consume the offsets at
  `source/physical_layer/mfsk.cc:658-674,683-699,713-729,737-753,762-779,
  1008-1028,1043-1061,1074-1086,1095-1120,1132-1161,1178-1203,
  1265-1279,1287-1306,1315-1365`.
- MFSK demodulation reads every tone bin at
  `source/physical_layer/mfsk.cc:1374-1431`.
- The telecom data path reads the same grid directly at
  `source/physical_layer/telecom_system.cc:1122-1155,3769-3801` and publishes
  it to the OFDM detector mirror at `:12378-12387`.
- OFDM time-sync, control, ACK, and suffix detectors consume the supplied
  offsets at `source/physical_layer/ofdm.cc:830,1021,4267,4516,4558,
  4572,4581,4668,4693,4707,4715,4912,4936,5071,5083,5196,5228,5304,5375`.
- Telecom control-plane call sites pass the fixed ACK geometry to those OFDM
  consumers at `source/physical_layer/telecom_system.cc:5142-6753`.

All consumers assume a live geometry's generated carrier indices are within
one `Nc`-carrier row. None owns extra overflow capacity.

## States and invariants

Valid states are:

1. Inert/default: `M == Nc == nStreams == 0`. Generation and demodulation
   return before indexing.
2. Live: all dimensions are positive, `nStreams <= 4`, and
   `M <= Nc / nStreams`. Centered offsets then make the last used carrier
   strictly less than `Nc`.
3. Configuration rejection: an invalid selected tuple leaves the telecom
   system at its prior configuration; from startup it remains `CONFIG_NONE`.

Direct invalid initialization first deinitializes the geometry, then returns.
This makes retries, demotion/climb, BREAK/control generation, reconnect, and
session reset fail inert if a malformed tuple ever bypasses selection. Normal
stock and exact-boundary geometries are unchanged. No waveform parameter,
threshold, tone map, or valid-grid offset changed.

## Deterministic evidence

The focused regression is `mfsk_geometry_guard` in
`source/physical_layer/mfsk_ctrl_codec_tests.cc`. Before the guard it reported
three geometry failures: both invalid active configurations and the direct
initializer, with the WB first-invalid witness `carrier=50, stream=1, tone=18`.

After the guard:

```text
MERCURY_MFSK_GEOMETRY_ONLY=1 MERCURY_CAP_CODEC_ONLY=1 ./mercury_debug --test
[OK] mfsk_geometry_guard
1 passed, 0 failed
```

The same focused test passes under AddressSanitizer with leak detection off and
a 64 MiB process stack (the test binary's existing main-frame footprint exceeds
the default ASan stack):

```text
ulimit -s 65536
ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
  MERCURY_MFSK_GEOMETRY_ONLY=1 MERCURY_CAP_CODEC_ONLY=1 \
  ./mercury_asan --test
[OK] mfsk_geometry_guard
1 passed, 0 failed
```

Both binaries were built from the base above plus the scoped working diff with
`build.sh debug` and `build.sh asan`, respectively. Their SHA-256 values were:

```text
mercury_debug  fe69cb39711407b68cfeaa9c52203dfbbf8f11a6200f8abe766196bd24ea298d
mercury_asan   4dfd004f96f3a8b5387bffac394831075d2a050ee6f9ee448f80021bb2a4293b
```

The repository-wide `./mercury_debug --test` smoke was also started with the
heavy sweep unset. It did not complete within the 304-second command bound and
was terminated with `SIGTERM`; that run is inconclusive and is not counted as
pass evidence.
