/*
 * Mercury: A configurable open-source software-defined modem.
 * Copyright (C) 2022-2024 Fadi Jerji
 * Author: Fadi Jerji
 * Email: fadi.jerji@  <gmail.com, caisresearch.com, ieee.org>
 * ORCID: 0000-0002-2076-5831
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#ifndef INC_MFSK_CTRL_CODEC_TESTS_H_
#define INC_MFSK_CTRL_CODEC_TESTS_H_

// Run the MFSK control-suffix codec test suite (Phase B Wave 1).
// Returns the number of failed tests (0 = all pass). See
// fact-documents/phase-b-mfsk-connect-research.md §11.7 for the list.
int run_mfsk_ctrl_codec_tests();

// Run ONLY the §22 OFDM fine-timing phase-invariant magnitude regression
// suite (fix/ofdm-fine-timing-magnitude). Fast + deterministic — excludes the
// long stochastic MFSK detector sweeps in the full suite above. Returns the
// number of failed tests (0 = all pass). See
// fact-documents/ofdm-fine-timing-magnitude.md §4.
int run_ofdm_fine_timing_tests();

// LEVER P: run ONLY the preamble-amortization schedule + effective-length unit
// tests (pure functions, no PHY bring-up). Fast + deterministic. Returns the
// number of failed tests (0 = all pass). See
// fact-documents/data-flow-preamble-amortization.md §1.
int run_preamble_sched_tests();

// fix/break-fh-gate: run ONLY the §23 BREAK forward-health gate suite (FH-latch
// suppression of the held-CFG16 marginal-OFDM alias + K-of-N corroboration +
// genuine-BREAK survival, in both gate states). Fast + deterministic. Returns the
// number of failed tests (0 = all pass). Wired via main.cc --test-break-fh.
int run_break_fh_gate_tests();

// BREAK OFDM-alias false-positive sweep: drive real non-BREAK OFDM data frames
// (config x Es/N0 grid) through acquisition+decode+the BREAK correlator and report
// whether the detonation predicate (coarse<0.30 && metric>=det_thr && matched>=thr)
// ever fires on data that is NOT a transmitted BREAK. Investigation harness (always
// returns 0). Wired via main.cc --test-break-alias.
int run_break_alias_sweep();

// recovery-ack-robustness.md: run ONLY the recovery-ACK robustness suite (marginal-ACK
// combining, listen-window ms-mirror, DELTA-1 reps-agnostic BREAK, DELTA-2 CFO-refine
// decision gate). Fast iteration. Returns failed count. Wired via main.cc
// --test-recovery-ack.
int run_recovery_ack_tests();

// recovery-ack-fine (STAGE 1): run ONLY the recovery control-ACK fine-pass
// timing-straddle sweep (fail-before no-fine collapse / pass-after fine recovery
// over the hardened bar). Fast + deterministic. Returns failed count. Wired via
// main.cc --test-recovery-ack-fine; also included in the full --test suite.
int run_recovery_ack_fine_tests();
// Run only the Moose CFO half-correction dead-zone regression at real CFG15
// geometry. Fast and deterministic; also included in the full --test suite.
// Define MOOSE_CFO_FAILBEFORE to exercise the former half-clamp behavior.
int run_moose_deadzone_tests();

// Thin-lattice cross-pilot noise-variance regression. Runs the production
// sparse-grid estimator from 10 through 30 dB Es/N0 and a dense cfg16 control.
int run_pilot_thin_nv_tests();

#endif // INC_MFSK_CTRL_CODEC_TESTS_H_
