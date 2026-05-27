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

// Unit tests for the BP+OSD decoder. Invoked by `mercury.exe --test`
// at startup; exits with status 0 on pass, non-zero on fail.

#ifndef LDPC_BP_OSD_TESTS_H_
#define LDPC_BP_OSD_TESTS_H_

// Returns the number of failed tests (0 on full pass). Prints PASS/FAIL
// lines to stdout per test, plus a one-line summary.
int run_ldpc_bp_osd_tests();

#endif
