#ifndef MERCURY_TEST_MAIN_H
#define MERCURY_TEST_MAIN_H

#include <cstddef>
#include <cstdio>
#include <string>

int test_shutdown_atomic();
int run_pas_selftest();
int run_cfg17_selftest();
int run_tinterp_seed_selftest();
int run_tinterp_receive_path_selftest();
int run_acq_bounds_selftest();
int run_subpeak_rescue_selftest();
int run_chase_ab_selftest();
int run_frame0_vehicle_selftest();
int run_frame0_replay_selftest(const char* path);
int run_crc_escape_selftest();
int run_chase_selftest();
void arm_test_watchdog();
int run_mini0_carried_timing_test();
int run_mini1_carried_timing_test();

int run_main_test_dispatch(int argc, char* argv[], int i,
    void (*install_termination_handlers)(), int (*run_frame0_mf_selftest)(), bool& handled);

class cl_arq_controller;
class cl_telecom_system;

struct MainTestOptions {
    int  test_rsp_bsi_corrupt_at_cli = 0; // --test-rsp-bsi-corrupt-at=N: SACK Design A Step 4 synthetic discard test
    bool test_rsp_sack_rsp_crc_corrupt_cli = false; // --test-rsp-sack-rsp-crc-corrupt: SACK Design A Step 7 CRC8 fault injection (one-shot)
    int  test_rsp_sack_rsp_crc_corrupt_count_cli = 0; // --test-rsp-sack-rsp-crc-corrupt-count=N: SACK Design A Step 11 N-shot CRC8 fault injection
    int test_policy_axis1_fire_cli = 0; // --test-policy-axis1-fire=up|down: SACK Design A Step 9 — synthetic Axis-1 trigger.
                                        // 0=off, 1=up (success=100%), 2=down (success=0%). One-shot at startup, then exit.
                                        // Forces sack_v2_enabled=true so policy_evaluate_axis1() is taken.
    int test_policy_axis2_fire_cli = 0; // --test-policy-axis2-fire=up|down: SACK Design A Step 10 — synthetic Axis-2 trigger.
                                        // 0=off, 1=up (clean ring), 2=down (lossy ring). One-shot at startup, then exit.
                                        // Forces sack_v2_enabled=true so policy_evaluate_axis2() is taken.
    const char* test_partial_bsi_advance_cli = NULL; // --test-partial-bsi-advance=mfsk|ofdm: SACK partial-path BSI non-advance reproducer.
                                        // 'mfsk' should FAIL on HEAD (used_mfsk_path=true bypasses send_sack_v2_frame's Step 8a bump).
                                        // 'ofdm' should PASS on HEAD (regression guard for the existing OFDM SACK_RSP path).
                                        // One-shot at startup, then exit. See fact-documents/sack_partial_bsi_advance.md §5.
    bool test_bigblock_arq_unit_cli = false; // --test-bigblock-arq-unit: P2 big-block ARQ-granularization regression.
                                        // 3 cases (clean K=8 / one-bad-cw / lost-EOB). MUST FAIL before P2 wiring (the
                                        // bigblock_block_to_arq stub returns BIGBLOCK_ARQ_NOT_WIRED), PASS after.
                                        // One-shot at startup, then exit rc. See fact-documents/data-flow-bigblock-arq-unit.md §6.
    bool test_sim_inproc_bigblock_cli = false; // --test-sim-inproc-bigblock: STEP 3 single-block end-to-end in the
                                        // in-process 2-instance sim (CMD->RSP->ACK->CMD byte-faithful + one-bad-cw
                                        // partial -> selective-repeat completes). One-shot at startup, then exit rc.
    bool test_bigblock_fullpath_cli = false; // --test-bigblock-fullpath: LIVE 2-instance CFG16 big-block transfer
                                        // through the REAL receive_bigblock+carve+whiten+FIFO deliver path with
                                        // fail-before/pass-after on the same binary. One-shot at startup, exit rc.
    bool test_sim_sustain_cli = false;  // --test-sim-sustain: SIM_INPROC sustain BATTERY (runs BOTH, OR of rc):
                                        // (1) PINNED-CFG15 sustained-delivery stepper-wedge regression (payloads
                                        // 600/2000/4000/8000; SIMFTR_ROOTCAUSE.md §7/§8 fix #1), then (2) OUTER-stepper
                                        // OFDM big-block no-wedge + clean-carve + C0-a byte-correct sustain (Phase b).
                                        // fail-before/pass-after on the same binary. One-shot at startup, exit rc.
    bool test_cfg103_gate_cli = false;   // --test-cfg103-gate: ROBUST_3 (cfg103) wiring guard unit
                                        // (ladder fixed point + fail-closed config_negotiable_to);
                                        // fail-before base arithmetic vs pass-after live, one-shot exit rc.
    bool test_bigblock_multicw_cli = false; // --test-bigblock-multicw: FULL K=8 block (all 8 codewords) through the
                                        // LIVE receive_bigblock+de-whiten+per-cw-CRC carve; 3 arms prove the root
                                        // cause is the RX capture WINDOW (cw1..cw7 corruption), NOT whiten/offset.
    bool test_inband_down_resync_cli = false; // --test-inband-down-resync: in-band down-ladder ROBUST resync —
                                        // CMD demote-to-ROBUST_0 with announce SUPPRESSED forces the RSP's
                                        // production down-ladder; fail-before (primary-sized snapshot truncates
                                        // ROBUST_0 -> 0 bytes) -> pass-after (Rank-1 snapshot+ring sizing fix).
    bool test_bigblock_chanest_cli = false; // --test-bigblock-chanest: GENUINE (ref==NULL) 2-instance CFG16 big-block
                                        // decode under a CFO/SFO-impaired channel; reproduces the HW [RXACQ] meanH
                                        // collapse off-bench (clean passes, CFO/SFO collapses the block estimate).
    bool test_nb_cfo_correction_cli = false; // --test-nb-cfo-correction: narrowband CONFIG_0 residual
                                        // carrier-offset correction regression (control 0 Hz + an in-window
                                        // offset through the in-process sim channel; fail-before/pass-after).
    bool test_bigblock_acqwindow_cli = false; // --test-bigblock-acqwindow: §19 acquisition-window POSITION guard —
                                        // one genuine K=8 block at several in-window preamble offsets in a FIXED
                                        // production-sized window; near-end (tail past window) DEFERS (guard ON) /
                                        // carves truncated bytes_ok=0 (DEFEAT_ACQGUARD). Reproduces the HW ~5.6% bug.
    bool test_topgear_clean_election_cli = false; // --test-topgear-clean-election: prove the CONFIG_17 64-QAM top-gear channel-clean election (verdict/hysteresis/ceiling/demote/rate)
    bool test_bigblock_climb_election_cli = false; // --test-bigblock-climb-election: prove the big-block rung is
                                        // ELECTED by the GEARSHIFT CFG16 transition (load_configuration tail), not
                                        // only at connect. fail-before/pass-after via MERCURY_BIGBLOCK_DEFEAT_ELECTION.
                                        // One-shot at startup, then exit rc. See data-flow-bigblock-arq-unit.md §16.
    bool test_bigblock_carve_suspend_unit_cli = false; // --test-bigblock-carve-suspend-unit: WALL-B FIX-3 RSP
                                        // carve-suspend watchdog unit test (streak state machine + the three consumers
                                        // + a real receive_byte cw0-reject loopback). fail-before via
                                        // MERCURY_BIGBLOCK_DEFEAT_CARVESUSPEND. One-shot at startup, then exit rc.
    bool test_bigblock_txlevel_cli = false; // --test-bigblock-txlevel: measure CFG16 big-block vs stock-OFDM TX
                                        // peak+RMS (HW over-level diag). One-shot at startup, then exit rc.
    bool test_sack_oow_reject_cli = false; // --test-sack-oow-reject: R039 — OFDM SACK_RSP out-of-window
                                        // reject. Drives real decode_sack_v2_frame + real sack_v2_bsi_in_window guard;
                                        // asserts CRC8-valid OOW SACK decoded-but-rejected, in-window accepted. One-shot, exits rc.
    bool test_eob_poison_prev_retx_cli = false; // --test-eob-poison-prev-retx: R038 — prev-retransmit EOB
                                        // poison. Drives the real EOB staging/promotion members; asserts pre-fix early
                                        // ACK-GATE PASS reproduced AND post-fix prevents it. One-shot, exits rc.
    bool test_batch_shrink_strands_prev_cli = false; // --test-batch-shrink-strands-prev: R035 — data_batch_size
                                        // shrink strands the active prev. Drives the REAL set_data_batch_size chokepoint;
                                        // asserts prev counters re-derived (gate reachable) + streaming defense on orphan. One-shot, exits rc.
    bool test_batch_shrink_orphan_defer_cli = false; // --test-batch-shrink-orphan-defer: Fix A (baseline-double-
                                        // delivery.md) — a mid-flight data_batch_size SHRINK must not orphan RECEIVED prev
                                        // frames. Drives the REAL set_data_batch_size shrink; reconstructs the
                                        // copy_data_to_buffer prev delivery set; asserts ZERO bytes lost. DEFEAT env = fail-before.
    bool test_backup_confirm_flush_cli = false; // --test-backup-confirm-flush: Fix C/H#1 (data-flow-fifo-backup.md
                                        // §6.1) — the delivered batch's raw must not survive in fifo_buffer_backup when
                                        // finalize is pre-empted (Root B). Drives register_ack ACK-confirm; asserts the
                                        // re-stage restores NOTHING (no re-delivery) + INV3 un-confirmed never flushed. DEFEAT=fail-before.
    bool test_rxfifo_backpressure_hold_cli = false; // --test-rxfifo-backpressure-hold: Fix H#3 (delivery-integrity-
                                        // audit-monitor.md §3) — the clean ACK precedes delivery; on RX-FIFO back-pressure
                                        // the un-stored tail is lost with a committed ACK. Gate holds the batch until the
                                        // app FIFO has room; asserts ZERO post-ACK loss + full re-delivery. DEFEAT=fail-before.
    bool test_retx_clear_on_recovery_cli = false; // --test-retx-clear-on-recovery: R029 — stale retx queue
                                        // cleared on recovery. Drives the REAL clear_retx_queue(); asserts the queue empties
                                        // of pre-recovery bsi, is idempotent, and repeatable. One-shot, exits rc.
    bool test_restage_requeue_orphan_cli = false; // --test-restage-requeue-orphan: §12 re-stage re-queue orphan/reorder
    bool test_restore_backup_bounds_cli = false; // --test-restore-backup-bounds: large streaming rollback backup restore
    bool test_stream_offset_cli = false; // --test-stream-offset: Option W FOUNDATION — absolute-byte-stream cursor ground-truth
    bool test_l1_stage2_ownership_cli = false;
    bool test_rx_drain_backpressure_cli = false; // --test-rx-drain-backpressure: FIX-6 — RX-delivery drain
                                        // must NOT drop popped bytes when the non-blocking app socket back-pressures.
                                        // FAILS at 62cb3dc (the 61,621-byte stall), PASSES after. One-shot, exits rc.
    bool test_kx_data_tamper_cli = false; // --test-kx-data-tamper: mid-stream AEAD ciphertext tamper fail-secure fire proof
    bool test_decompress_false_accept_cli = false; // --test-decompress-false-accept: streaming decompress-failure
                                        // must NOT push the undecodable raw compressed blob to the app (silent byte-
                                        // corruption). Drives a REAL two-compressor streaming desync + copy_data_to_buffer;
                                        // MERCURY_DECOMPRESS_RAWPUSH_DEFEAT=1 = fail-before. One-shot, exits rc.
    bool test_compact_confirm_rx_cli = false; // --test-compact-confirm-rx: Option B compact confirm LIVE RX path.
                                        // The self-validating compact confirm must accept via its own CRC-gated decode,
                                        // decoupled from the bare 7/16 gate that under-peaks the 26-sym frame. One-shot, exits rc.
    bool test_gap_abort_cli = false;    // --test-gap-abort: FIX-8 — silent lost-batch GAP on post-reset re-adopt.
                                        // Reproduces bench-4 (deliver 0-4, BREAK reset with 5-7 undelivered, present 8):
                                        // fail-before via MERCURY_GAP_ABORT_DEFEAT=1 (silent concat), pass-after aborts
                                        // loudly + delivers EXACTLY batches 0-4. One-shot, exits rc.
    bool test_gap_recover_cli = false;  // --test-gap-recover: RECOVERABLE delivery-time GAP-ABORT
    bool test_retain_overflow_cli = false; // --test-retain-shadow-overflow: deterministic 19>8 hole burst
    bool test_recover_fire_cli = false; // --test-gap-recover-fire: W1 CMD refill fire proof
                                        // (R2a hold + R2c CMD retention). fail-before via
                                        // MERCURY_GAP_RECOVER_DEFEAT=1 (terminal abort, high-water frozen);
                                        // pass-after = HOLD + refill + in-order 6->7->8. One-shot, exits rc.
    bool test_held_cur_fire_cli = false; // --test-held-cur-deliver-fire: R2b RSP deliver-held-cur
                                        // (bytes reach the app FIFO via copy_data_to_buffer) + CMD
                                        // recovery-turnaround credit. fail-before via
                                        // MERCURY_HELD_CUR_DELIVER_DEFEAT / MERCURY_GAP_RECOVER_
                                        // TURNAROUND_DEFEAT. One-shot, exits rc.
    bool test_measured_timers_cli = false; // --test-measured-timers: R6 SRTT/RTTVAR turnaround estimator +
                                        // ack_timeout_data >= receiving_timeout invariant regression (one-shot, exit rc).
    bool test_t1_cli = false;           // --test-t1: T1 zero-dead-time turnaround suites (learned-timing
                                        // calibration + scheduled pre-key re-aim;
                                        // one-shot, exit rc) -- fast proof without the full-PHY --test sweep.
    bool test_arqsmalls_cli = false;    // --test-arqsmalls: Karn discriminator decoupling
                                        // (MERCURY_KARN_RETX_ONLY) + DUTY-R pin-respect seed
                                        // (MERCURY_DUTY_R_PIN_DEFEAT), fast one-shot, exit rc.
    bool test_recovery_ack_capture_exercise_cli = false; // --test-recovery-ack-capture-exercise:
                                        // live production poll, late retained ACK -> 1/1 accept.
    bool test_turnaround_guard_cli = false; // --test-turnaround-guard: R1 turnaround-clearance guard —
                                        // stamp-then-key drives assert the guard waits out the peer's TX->RX
                                        // mute/flush window (default-on) and adds ~0 when already clear / never armed.
                                        // One-shot, exits rc.
    bool test_axis2_quiesce_gate_cli = false; // --test-axis2-quiesce-gate: R4 LINK-PARAMS quiesce gate —
                                        // an Axis-2 move DEFERS while the batch boundary is unclean (retx pending /
                                        // last batch partial) and FIRES when clean; FORCED after AXIS2_MAX_MOVE_DEFER;
                                        // fail-before via MERCURY_AXIS2_QUIESCE_DEFEAT=1 (mixed old-bsi batch). One-shot, exits rc.
    bool test_gap_abort_blind_cli = false; // --test-gap-abort-blind: GAP-ABORT ruler-blinding — the REAL
                                        // rsp_gap_abort_teardown() resets the contiguity ruler to -1, blinding its own
                                        // re-adopt gate, and the CMD (never told) re-drives the same stream -> silent
                                        // concatenation across a dropped batch. Fails on the current tree. One-shot, exits rc.
    bool test_gap_abort_stream_blind_cli = false; // --test-gap-abort-stream-blind: GAP-ABORT stream-BACKSTOP
                                        // companion — the REAL teardown must preserve the Option W byte cursor +
                                        // stamp validity (so w_stream_shift_detected still fires post-abort) and set the
                                        // sticky rsp_stream_aborted latch. MERCURY_GAP_STREAM_BLIND=1 = fail-before arm.
    bool test_config_tag_passband_cli = false; // --test-config-tag-passband: in-band rate-adapt Stage 3a —
                                        // PASSBAND ROUND-TRIP. TX keys the combined RM+gf16ra suffix to real passband
                                        // audio, passes it through CLEAN and AWGN, RX detects it on the passband (real
                                        // base-correlator presence detector) + decodes the right cfg_index, and proves an
                                        // OFDM data frame still LDPC-decodes with the suffix appended. fail-before:
                                        // rebuild with -DSTAGE3A_FAILBEFORE (RX ignores the passband suffix). One-shot,
                                        // exits rc. unilateral-config-tag-design.md §11 Stage 3.
    bool test_config_tag_follow_cli = false; // --test-config-tag-follow: in-band rate-adapt Stage 2 — emit/detect/FOLLOW.
                                        // Forces a CONFIG_10->CONFIG_8 batch-boundary switch; asserts the RX follows the
                                        // config FROM THE TAG (load_configuration) with the PHY twin switching coherently.
                                        // fail-before: rebuild with -DSTAGE2_FAILBEFORE (RX ignores tag -> stays CFG10).
                                        // One-shot, exits rc. unilateral-config-tag-design.md §11 Stage 2.
                                        // See bigblock_p3_hw/_fix8/FIX8_DESIGN.md + FIX8_AUDIT.md.
    bool test_inband_drop_cli = false;  // --test-inband-drop: in-band rate-adapt Stage 3b — LOOPBACK DROP.
    bool test_inband_fallback_cli = false;  // --test-inband-fallback: in-band Stage 4 — LOST-TAG DOWN-LADDER.
    bool test_inband_seamless_cli = false;  // --test-inband-seamless: in-band Stage 3d — PRE-FRAME SEAMLESS.
    bool test_inband_downladder_cli = false;  // --test-inband-downladder: down-ladder BREAK-orphan + silent-snapshot regression.
    bool test_reseat_span_cli = false;  // --test-reseat-span: prev-batch cross-storage index-skew 332-byte deletion keystone.
    bool test_cmd_idskew_cli = false;  // --test-cmd-idskew: CMD-side id-skew root — rebase a fresh new-data batch to slots [0,ND).
    bool test_inband_deliver_cli = false;  // --test-inband-deliver: forward-healthy reverse-ACK miss -> NO-BREAK deliver regression.
    bool test_linkphase_shadow_cli = false; // --test-linkphase-shadow: increment-1 shadow-agreement directed unit.
    bool test_linkphase_mc2_cli = false;    // --test-linkphase-mc2: increment-2 MC-2 slot-floor directed unit.
    bool test_linkphase_optclock_cli = false; // --test-linkphase-optclock: END-stamp freshness directed unit.
    bool test_linkphase_ackwin_cli = false; // --test-linkphase-ackwin: increment-3 reverse-ACK detection-window directed unit.
    bool test_inband_ring_floor_cli = false;  // --test-inband-ring-floor: capture-ring ROBUST-floor over-seat at a climbed OFDM rung.
    bool test_inband_liveness_cli = false;  // --test-inband-liveness: connect-liveness guard (control-plane livelock backstop).
    bool test_inband_no_break_cli = false;  // --test-inband-no-break: in-band Stage 4c — D5 BREAK-OBSOLETE.
    bool test_inband_retag_cli = false;  // --test-inband-retag: in-band Stage 4d — D1 repeat + D4 climb/auto-demote.
    bool test_inband_nack_cli = false;  // --test-inband-nack: in-band Stage 4e — D2 NACK first-class.
    bool test_inband_reannounce_cli = false;  // --test-inband-reannounce: same-config tag owner-law regression.
                                        // Gearshift-driven unilateral drop (W3), tag on the real passband (W1),
                                        // RX follows from the passband tag (W2 + HINGE), SACK confirms (bsi),
                                        // ZERO SET_CONFIG on the wire, both ends config-track, PHY-twin coherent,
                                        // + the R7 mixed-config gap-gate case. fail-before: MERCURY_INBAND_RATE
                                        // unset OR -DINBAND_STAGE3B_FAILBEFORE. One-shot, exits rc.
                                        // data-flow-perbatch-config.md §12 / unilateral-config-tag-design.md §11 Stage 3.
    bool test_recovery_ack_capture_cli = false; // --test-recovery-ack-capture: LEVER 1 focused
                                        // capture-window test (recovery-ACK late/truncated + unwritten-head
                                        // ring). See arq_responder.cc test_recovery_ack_capture +
                                        // fact-documents/data-flow-recovery-ack-capture.md §7.
    bool test_data_ack_multiwindow_cli = false; // --test-data-ack-multiwindow: Track A — multi-window
                                        // DATA-ACK/SACK correlator. Synthesizes a real ACK+SACK burst at an
                                        // OLDER ring phase with a silent newest tail: fail-before (newest-tail
                                        // decode MISSES), pass-after (mw_find_ack_sack_phase recovers it with
                                        // CRC12 pass), no-false-accept on pure silence. One-shot, exits rc.
                                        // See fact-documents/data-flow-data-ack-sack-correlator.md §7.
    bool test_inorder_demote_cli = false; // --test-inorder-demote: D3.1 — UNIFIED in-order delivery
    bool test_climb_bsi_rollback_cli = false; // --test-climb-bsi-rollback: CLIMB-CHURN commander-side bsi rollback on a climb-UP promote
                                        // across EVERY demote case (BREAK + the 4 SET_CONFIG-only demotes +
                                        // PREV-BUMP strand). fail-before via MERCURY_GAP_ABORT_DEFEAT=1
                                        // (silent concat on the SET_CONFIG cases), pass-after aborts loudly
                                        // OR delivers contiguous. One-shot, exits rc.
                                        // See bigblock_p3_hw/_d31_fade/D31_INORDER_DESIGN.md.
    bool test_guard_reanchor_cli = false; // --test-guard-reanchor: live reverse-SACK guard reference
    bool test_spec_sack_cli = false;    // --test-spec-sack: LEVER #2 — speculative/prompt SACK. Frame-k still-decoding
                                        // at the window-fraction deadline -> in-window SACK bit_k=0 -> CMD retx ->
                                        // byte-faithful re-receive -> single in-order delivery, no silent loss.
                                        // fail-before via env-off (stall reproduced), pass-after via MERCURY_SPEC_SACK=1.
                                        // One-shot, exits rc. See fact-documents/turnaround-eff.md §8/§9.
    bool test_batchsize_desync_cli = false; // --test-batchsize-desync: res_c3100 CMD>RSP batch-size desync
                                        // silent-corruption regression (silent-corruption-residual.md §8).
                                        // fail-before via MERCURY_BATCHSIZE_DESYNC_DEFEAT=1.
    bool test_batch_shrink_orphan_current_cli = false; // --test-batch-shrink-orphan-current: rx_btf=-1
                                        // current-batch shrink tail-drop (marginal-SNR silent corruption).
                                        // fail-before via MERCURY_BATCHSHRINK_ORPHAN_DEFEAT=1, pass-after defers.
    bool test_linkphase_pending_confirm_cli = false; // --test-linkphase-pending-confirm: SEAM-2
                                        // deferred prev-confirm ARM/FLUSH fire proof.
                                        // fail-before via MERCURY_PENDING_CONFIRM_DEFEAT=1.
    bool test_eob_loss_batch_truncation_cli = false; // --test-eob-loss-batch-truncation: D5 — EOB-inference
    bool test_dedup_rebase_cli = false; // --test-dedup-rebase: in-band demote-rebase double-delivery
                                        // (byte-stream corruption) regression. fail-before via
                                        // MERCURY_STREAM_DEDUP_DEFEAT=1. One-shot exits rc; also runs in --test.
                                        // batch truncation. A 30-frame batch loses its EOB-marked tail; the
                                        // wired batch_total_frames recovers the true length. fail-before via
                                        // MERCURY_D5_INFER_DEFEAT=1 (silent 29-frame skip), pass-after holds
                                        // then delivers all 30 faithfully after the retx. One-shot, exits rc.
                                        // See TRACK_C_D2D3D5_DESIGN.md §5.3 / data-flow-prev-bump.md §8.
    bool test_v2_pendingack_flip_alias_cli = false; // --test-v2-pendingack-flip-alias: R030 — v2 PENDING_ACK
                                        // flip aliasing. Diverged index/wire space; drives the REAL v2_flip_resolve_slot();
                                        // asserts retx skipped + new-data -> correct slot + no FREE/foreign PENDING_ACK. One-shot, exits rc.
    bool test_retx_slot_order_cli = false; // --test-retx-slot-order: timing redesign — retx never rides wire slot 0.
                                        // Drives the REAL v2_rotate_retx_behind_lead()+v2_flip_resolve_slot(); asserts slot0=new-data,
                                        // EOB wire-final, retx bytes verbatim, flip -1 for retx only, ND==1 byte-identical. One-shot, exits rc.
    bool test_bigblock_livepath_cli = false; // --test-bigblock-livepath: GAP-2 — real CONNECT + real SET_CONFIG
                                        // handshake robust->CFG16 (NO pin), then a 1374B K=8 transfer through the
                                        // REAL send_batch->bigblock_send_one_block emit + receive_byte cw0-CRC gate
                                        // + carve + FIFO. Answers: does sim reproduce the HW cw0-CRC reject? exit rc.
    bool test_data_anchored_promote_cli = false; // --test-data-anchored-promote: Option B (data-anchored gearshift
                                        // promotion) regression. Drives break_target_with_anchor() + policy_evaluate_axis1()
                                        // with last_data_viable_config primed; asserts BREAK floors at the anchor and the
                                        // up-shifter promotes only one rung past it. One-shot, exits rc. See
                                        // fact-documents/gearshift-start-and-recovery.md §6.4.
    bool test_shutdown_atomic_cli = false; // --test-shutdown-atomic: R006 — shutdown_ atomicity
                                        // regression (race_audit R006). Compile-time static_assert (fail-before on
                                        // pre-fix plain-bool tree) + cross-thread runtime smoke. One-shot, exits rc.
    bool test_probe_backoff_cli = false; // --test-probe-backoff: FIX-B floor-probe back-off regression
                                        // (gearshift-floor-probe-backoff.md §7). Drives the REAL arm/gate/reset/predicate
                                        // machinery + the v2 policy_evaluate_axis1 UP gate over the SIM virtual clock.
                                        // PB1 FAIL-BEFORE/PASS-AFTER (armed CONFIG_0 suppressed + UP gate blocks; revert
                                        // -> promotes), PB2 virtual-clock elapse lifts, PB3 exponential+cap, PB4 reset,
                                        // PB5 INV-2 deep-SNR escape unchanged. One-shot, exits rc.
    bool test_phantom_ack_gate_cli = false; // --test-phantom-ack-gate: phantom-ACK content-gate regression.
                                        // Drives data_ack_bare_pattern_acceptable() across WB/NB x CRC-valid/CRC-absent
                                        // (the WB-no-CRC phantom cell must be REJECTED) + asserts a rejected phantom leaves
                                        // data_ack_received NO, does not raise last_data_viable_config / reset the BREAK
                                        // panic counter, and BREAK still reaches ROBUST_0. One-shot, exits rc. See
                                        // fact-documents/gearshift-start-and-recovery.md §8.
    bool test_clean_batch_viability_cli = false; // --test-clean-batch-viability: CLEAN-BATCH VIABILITY regression (§9).
    bool test_idle_switch_role_race_cli = false; // --test-idle-switch-role-race: idle SWITCH_ROLE race regression.
                                        // Drives the REAL process_buffer_data_commander() idle branch with a freshly-
                                        // CONNECTED empty-tx Commander; asserts SWITCH_ROLE is queued before any data
                                        // (the connected-but-0-deliver root cause). FAILS-BEFORE on monitor. One-shot.
    bool test_break_noprogress_cli = false; // --test-break-noprogress-teardown: BREAK no-progress teardown regression
                                        // (idle-switchrole-race.md §3/§4 Part C). Replays the shared break_noprogress_step
                                        // kernel: teardown fires at exactly K dead cycles; progress on a live BREAK resets
                                        // the streak (negative control). FAILS-BEFORE with -DBREAK_NOPROGRESS_FAILBEFORE.
    bool test_rx_ctrl_drop_cli = false; // --test-rx-ctrl-drop: RECEIVED-stuck control-slot regression
                                        // (data-flow-control-slot-lifecycle.md §6). Drives the shared RECEIVED-state
                                        // watchdog predicate + the produce-gate replication: a stranded RECEIVED slot is
                                        // freed and a later control frame lands. FAILS-BEFORE with -DRX_CTRL_DROP_FAILBEFORE.
    bool test_robust0_compress_deadlock_cli = false; // --test-robust0-compress-deadlock: ROBUST_0+streaming-compression
    bool test_streaming_compress_overshoot_cli = false; // --test-streaming-compress-overshoot: stale-EMA no-fit must retry compressed
    bool test_mixbatch_fill_overpop_cli = false; // --test-mixbatch-fill-overpop: mixbatch fill over-pop reorder regression
    bool test_mixbatch_fill_overpop_compressed_cli = false; // --test-mixbatch-fill-overpop-compressed: comp-leg over-pop + force-FREE data-loss regression
                                        // deadlock regression. Drives the REAL process_buffer_data_commander() data-fill
                                        // at ROBUST_0 (max_frame==7==COMPRESS_HEADER_SIZE) with streaming compression +
                                        // a real compressible payload; asserts >0 application bytes are staged. FAILS on
                                        // fef293f (every batch stages 0 payload → 0 throughput). See
                                        // fact-documents/data-flow-compress-frame-fill.md §5.
    bool test_cumulative_ack_cli = false; // --test-cumulative-ack: Tier-2 cumulative-n_r self-heal/gap-invariant/cap-gate regression (data-flow-forgiving-ack.md §T2.6).
    bool test_enc_failclosed_cli = false; // --test-enc-failclosed: encryption negotiation fail-closed regression (opt-in + peer-no-cap -> REFUSE; default-off -> plaintext). Fails-before under -DENC_FAILOPEN_FAILBEFORE.
    bool test_a3_decouple_safety_cli = false; // --test-a3-decouple-safety: the §2 CHECKPOINT — single-miss non-load-bearing (anti-0-bytes, byte-faithful) + genuine-death net intact, demote IN PLACE (data-flow-forgiving-ack.md §T2.2/§6).
    bool test_pas_cli = false;          // --test-pas: PAS/PCS distribution-matcher bijection + histogram self-test (feat/pcs).
    bool test_cfg17_cli = false;        // --test-cfg17: CFG17 shaped-64-QAM composition (PAS+TINTERP-seed+ratio-nvfix) failing-first (feat/cfg17).
    bool test_tinterp_seed_cli = false; // --test-tinterp-seed: TINTERP-SEED production it=0 estimator seed-swap failing-first (staging/tinterp-seed).
    bool test_tinterp_receive_cli = false; // --test-tinterp-receive: actual receive_byte producer/selector transitions.
    bool test_decode_marathon_cli = false; // --test-decode-marathon: LEVER C parallel==serial big-block decode integrity (decode-marathon-C.md §8).
    bool test_fir_lever_cli = false;    // --test-fir-lever: exact-AVX2 FIR lever law (unset=default-on, 0=scalar authority) + byte-exactness at the production entry points.
    bool test_climb_engine_cli = false; // --test-climb-engine: integrated 3-bug climb regression (gearshift-climb-engine.md §7).
    bool test_break_weld_cli = false;   // --test-break-weld: BREAK recovery target pin (diagnostic knob).
    bool test_rung_floor_cli = false;   // --test-rung-floor: per-rung MEASURED election floor gate + failure memory (DATAFLOW_AUDIT_rung_floor.md): wb13 fail-before/pass-after, never-raise, arm/survive/decay/reset, defeat knob.
    bool test_acksuffix_cli = false;    // --test-acksuffix: ACK-suffix pollution guard fail-before/pass-after (forward-DATA SNR vs control-frame decode).
                                        // Asserts a PARTIAL SACK does NOT raise last_data_viable_config, reset the BREAK
                                        // panic counter / break_drop_step, advance the FRAME-UP counter, or clear the 85%
                                        // up-promotion gate; a CLEAN all-ones batch does all of those; and that BREAK can
                                        // still reach ROBUST_0 after a partial-only run. One-shot, exits rc. See §9.
    int test_policy_axis1_then_axis2_cli = 0; // --test-policy-axis1-then-axis2=up|down: SACK Design A Step 10 — fire Axis-1
                                        // (engages axis2_cooldown_batches=3) then attempt Axis-2 fire (should be SUPPRESSED).
                                        // 1=axis1=up then axis2=up; 2=axis1=down then axis2=down. One-shot at startup, then exit.
    int test_policy_axis3_fire_cli = 0; // --test-policy-axis3-fire={ok,miss,walk}: SACK Design A Step 11 — synthetic Axis-3 trigger.
                                        // 0=off, 1=single ok event, 2=single miss event, 3=composite walk demo
                                        // (3 misses → ON→PROBE, 2 more → PROBE→OFF, 20 ticks → OFF→PROBE, ok → PROBE→ON).
                                        // One-shot at startup, then exit. Forces sack_v2_enabled=true.
    int test_policy_axis1_then_axis3_cli = 0; // --test-policy-axis1-then-axis3=miss: SACK Design A Step 11 — fire Axis-1
                                        // supremacy then attempt Axis-3 fires (should be SUPPRESSED during the 3-batch cooldown).
                                        // 1=axis1 supremacy + 3 attempted Axis-3 misses. One-shot at startup, then exit.
    int test_policy_axis3_miss_burst_cli = 0; // --test-policy-axis3-miss-burst=N: SACK Design A Step 11 — fire N miss events
                                        // directly into policy_evaluate_axis3 at startup, then exit. Used by Gate 3
                                        // (drives ON→PROBE at N=3, PROBE→OFF at N=5) and Gate 6 (large N then tick to 20
                                        // batches to drive OFF→PROBE).
    int test_policy_axis3_recover_cli = 0; // --test-policy-axis3-recover=N: SACK Design A Step 11 — fire N misses then a single
                                        // ok (Gate 5: PROBE→ON recovery demo).
    int test_policy_axis3_offperiodic_cli = 0; // --test-policy-axis3-offperiodic=N: drive 5+ misses to OFF, then tick N batches
                                        // (Gate 6: OFF→PROBE 20-batch periodic re-probe).
    int test_policy_axis2_ceiling_fire_cli = 0; // --test-policy-axis2-ceiling-fire=1: SACK Design A Step 12 — drive a synthetic
                                        // Axis-2 down-move (sets proven_ceiling) then attempt an up-move (must be VETOED
                                        // by the ceiling). One-shot at startup, then exit.
    int test_policy_break_supremacy_cli = 0; // --test-policy-break-supremacy=1: SACK Design A Step 12 — invoke the supremacy
                                        // hook with a synthetic BREAK reason; assert Axis-2 + Axis-3 reset + 3 evaluations
                                        // suppressed. One-shot at startup, then exit.
};

bool parse_main_test_option(int& argc, char* argv[], int& i, MainTestOptions& test_opts);
void run_main_post_tests(cl_arq_controller& ARQ, cl_telecom_system& telecom_system,
    const MainTestOptions& test_opts);

int run_main_test_tx_level_cli_validation(bool (*parse_tx_level_cli)(const char* value, double& level, std::string& error));
int run_main_test_bandpass_cli_validation(bool (*parse_bandpass_cli)(const char* value, double& low_hz, double& high_hz, std::string& error));
int run_main_test_audio_device_path_bounds(void (*copy_audio_device_path)(char* destination, const char* source));
int run_main_test_agw_server_start_fail_closed(int (*validate_modem_server_startup)(int result, FILE* diagnostic));
int run_main_test_encryption_cli_validation(bool (*parse_encryption_mode_cli)(const char* value, int& mode, std::string& error));

void apply_main_test_faults(cl_arq_controller& ARQ, const MainTestOptions& test_opts);
void apply_main_test_fault_count(cl_arq_controller& ARQ, const MainTestOptions& test_opts);

#endif
