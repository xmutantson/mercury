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

// feat/revack-geometry: run ONLY the §24 coordinated recovery-ACK geometry suite
// (INV-G both peers compute the SAME deterministic config-derived key offset + the
// CMD snapshot is CENTERED on the deterministic ACK block; fail-before/pass-after on
// REVACK_GEOMETRY_FAILBEFORE). Fast + deterministic, pure geometry (no audio).
// Returns the number of failed tests (0 = all pass). Wired via main.cc
// --test-revack-geometry. Also included in the full --test suite.
int run_revack_geometry_tests();

#endif // INC_MFSK_CTRL_CODEC_TESTS_H_
