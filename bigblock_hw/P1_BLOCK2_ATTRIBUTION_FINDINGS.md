# P1-block2-attribution findings (read-only analysis, branch fix/bigblock-whiten-align @af5f012)

## VERDICT: block-2 wrong-window is a SIM single-thread pacing artifact, NOT a production bug.

### Evidence chain
1. af5f012 touches ZERO production sim-pump code. The sim wire pump (`sim2_drain_to_wire`,
   `sim2_deliver_from_wire`, single-symbol pacing, `sim_inproc_pump_2`, arq_commander.cc
   ~9700-9960) is PRE-EXISTING and unmodified by this commit. The commit's arq_commander.cc
   changes are ALL test-harness (`test_sim_inproc_bigblock_fullpath`, capture statics,
   `MERCURY_SIM2_STOP_AFTER_FIRST_BLOCK`).

2. The green regression only exercises ONE block. `test_sim_inproc_bigblock_fullpath` pins
   PAYLOAD=1200 with the explicit comment "1200 app bytes fit in ONE block" (K=8, sub_len=175,
   cap ~1400 B). So the multi-block "block 2" path is OUTSIDE the passing regression.

3. The single-thread sim STRUCTURALLY cannot complete multi-batch — INDEPENDENTLY documented
   BEFORE the big-block work in data-flow-sim2-time-domain-faithfulness.md §9 (F3 + DEPENDENCY):
   "the responder->commander SACK/ACK turnaround delivery does not complete under the
   single-thread pump's depth-0-gated delivery, so the commander LINK-TIMEOUTs and re-HAILs
   instead of advancing to batch 2." Block 2 only TXes after A accepts B's clean ACK for block 1.
   The big-block clean ACK uses the SAME path: bigblock_block_to_arq CLEAN branch
   (test_bigblock_arq_unit.cc:184-248) marks slots ACKED + batch_data_delivered, then the stock
   process_messages_acknowledging_data() ACK-GATE (arq_responder.cc:1431,1140) sends the ACK over
   the rx->tx wire — exactly the turnaround §9 says deadlocks. So in the sim, block 2 is fed late /
   misaligned / after a re-HAIL, and acquires a garbage window. This is the documented sim limit.

4. The cross-block re-arm/wipe/bsi-advance is CORRECT for HW:
   - Ring wipe (arq_common.cc:6918-6945): signal_period at :6738 = Nofdm*buffer_Nsymb*interp =
     FULL double-mapped ring, so the `for k in 0..sp` wipe clears the entire ring (both copies).
     This is needed because the stock OK-decode zeroing (:6979) is gated on
     message_decoded==YES, which the carve clears to NO (:6906). Consistent.
   - ring_write_index is deliberately NOT reset (:6937) — the capture-prep feed keeps advancing
     it; the next snapshot (:6756-6757) reads signal_period samples from the live rwi, a correct
     rolling window. Block 2 re-accumulates from a wiped ring into a fresh frame-aligned window.
   - bsi advance (test_bigblock_arq_unit.cc:239-242): +1 per CLEAN block; seeded from the WIRE bsi
     on the first block. Block 2 carve uses use_wire_header=true (arq_common.cc:6901-6902), so the
     wire cw0 header bsi is authoritative — drift-proof; rsp_current_expected_batch_seq_id is only
     a FALLBACK. A bsi error could NOT move the PHY acquisition WINDOW anyway (window = Schmidl-Cox
     on the snapshot in bigblock_rx_passband; bsi is ARQ-layer, post-decode).

5. REAL HARDWARE already proves back-to-back multi-block PHY acquisition. bigblock_hw/
   PHASE2_HW_RESULTS.md: 8/8 byte-correct over "multiple independent block periods" and
   "consecutive periods in the SAME loop", using 4x back-to-back blocks played in a continuous
   loop (RPi1 -> RPi2, clean channel, real Fe-Pi crystals + 13 ppm SFO). The only first-block
   caveat is a key-up/throwaway-lead (line 61), not a block-2 defect. HW PHY = two real timelines,
   no single-thread turnaround deadlock -> block 2 acquires correctly there.

### HW multi-block prediction: WILL-WORK (clean/slow-fade regime).
PHY back-to-back acquisition is HW-proven. The af5f012 RX-arming (full-block frames_to_read wait
+ full-ring wipe + non-reset rwi + wire-bsi advance) is HW-correct and is exactly what a real
concurrent RX needs. The sim's block-2 garbage is the depth-0 SACK-turnaround deadlock (§9), which
does NOT exist on HW (real concurrent threads). Recommend: validate multi-block on HW through the
full ARQ loop (not just the PHY WAV loop), since the HW proof so far is PHY-only.

### Retry-heap risk (open_risk #2): LIVE-could-crash on the DIRTY multi-block path.
On a CLEAN first block the production path never hits the partial retry (safe — matches the green
single-block regression). BUT in a multi-block session, if block 2 decodes garbage (every CRC
fails -> clean=0 -> PARTIAL), bigblock_block_to_arq's PARTIAL branch
(test_bigblock_arq_unit.cc:250-281) queues K stock CFG16 per-frame retx into the RECEIVER's
retransmit_frames[]. The carve copy itself is bounded (len clamped to MAX_SACK_FRAME_SIZE=256 ==
row width; rci guarded by MAX_RETRANSMIT_HEADROOM). The flagged instability is in the DOWNSTREAM
stock per-frame selective-repeat consuming a big-block-origin partial under a CFG16 block context
(frame==codeword, EOB=K-1 synthetic). The fix author defends against it in the FAIL-BEFORE arm by
stopping after the first block (MERCURY_SIM2_STOP_AFTER_FIRST_BLOCK) precisely to avoid "the
slow/unstable post-partial stock per-frame retry loop." On HW a deep-fade block-2 would exercise
this path. Mitigation before a fading-channel HW multi-block run: audit/bound the post-partial
big-block selective-repeat path (it has no dedicated regression). Per HW results, the big-block is
a CLEAN/slow-fade high-rate mode — keep it off deep-fade, which also keeps block 2 clean.

### action_needed: none-proceed-to-hw (clean path). No production bug in the cross-block arming.
Proviso: before a FADING multi-block HW run, bound the post-partial selective-repeat path.
