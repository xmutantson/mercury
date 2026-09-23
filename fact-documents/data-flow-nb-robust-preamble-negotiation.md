# data-flow: NB robust-preamble negotiation to MFSK acquisition

Audit date: 2026-09-23.

## Producers

The datalink handshake derives `both = local-capable && peer-capable` and calls
the PHY (`source/datalink_layer/arq_common.cc`). Responder and commander receipt
paths both call this shared update. The PHY stores the session verdict, selects
the active MFSK set, and preserves the verdict across configuration changes.

## Consumers

TX uses the active `mfsk.preamble_*` tables. RX mirrors the active tables as its
primary and may mirror the other set as an alternate. The detector searches the
primary first, then the alternate only after a primary miss. Extraction uses the
matched preamble length.

## Valid states

- Before negotiation or after reset, legacy is primary and sidelnikov may be an
  alternate transition aid.
- After both peers advertise support, sidelnikov is the only legal TX preamble
  and therefore the only legal RX detector set.
- Forced legacy or sidelnikov modes remain authoritative and are unchanged.

## Invariants

1. Active TX and primary RX tables have the same sequence, length, and threshold.
2. An alternate may cover handshake transition, but must not accept a wire form
   the negotiated peer is forbidden to transmit.
3. Session reset restores legacy before any legacy data can be sent.
4. Configuration swaps reapply the session verdict rather than cached init state.
5. A short alternate match must not shadow a real long primary after negotiation.

## Change

`rebuild_mfsk_preamble_runtime()` now closes the legacy alternate after symmetric
negotiation. `MERCURY_NB_POSTNEG_ALT_CLOSE=0` restores the old decision. The
`nb_robust_preamble_capneg` regression fails against the old decision (or with
the switch set to zero) and requires the default path to reject the false
alternate while retaining primary sidelnikov acquisition and reset behavior.

The archived root cohort transmitted the 48-symbol negotiated preamble and no
8-symbol legacy preambles, while stalled responders selected the short alternate.
The independent live rerun found zero post-negotiation short-alternate matches and
zero not-ACKed switch stalls with the fix enabled.
