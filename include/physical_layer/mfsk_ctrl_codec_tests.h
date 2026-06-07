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

#endif // INC_MFSK_CTRL_CODEC_TESTS_H_
