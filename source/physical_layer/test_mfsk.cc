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


#include "physical_layer/mfsk.h"

// Test-only injection point: stuff de-hopped payload tones directly into
// last_ack_sack_suffix_tones[] and flag the capture as valid, bypassing
// the RF path. The symbol-domain round-trip test uses this so it can
// validate the decode primitive without setting up FFT plumbing.
void cl_mfsk::test_inject_ack_sack_capture(const int* tones, int count)
{
	int n = ack_sack_suffix_len();
	if (n == 0 || tones == nullptr) {
		last_ack_sack_capture_valid = false;
		return;
	}
	if (count > n) count = n;
	if (count > MAX_ACK_SACK_SUFFIX) count = MAX_ACK_SACK_SUFFIX;
	for (int i = 0; i < count; i++)
		last_ack_sack_suffix_tones[i] = tones[i] & (M - 1);
	for (int i = count; i < MAX_ACK_SACK_SUFFIX; i++)
		last_ack_sack_suffix_tones[i] = -1;
	last_ack_sack_capture_valid = (count == n);
}

// Test-only: stuff CONNECT-suffix payload tones directly into the capture
// buffer (bypasses RF). Mirror of test_inject_ack_sack_capture.
void cl_mfsk::test_inject_connect_capture(const int* tones, int count)
{
	int n = ack_sack_suffix_len();
	if (n == 0 || tones == nullptr) {
		last_connect_capture_valid = false;
		return;
	}
	if (count > n) count = n;
	if (count > MAX_ACK_SACK_SUFFIX) count = MAX_ACK_SACK_SUFFIX;
	for (int i = 0; i < count; i++)
		last_connect_suffix_tones[i] = tones[i] & (M - 1);
	for (int i = count; i < MAX_ACK_SACK_SUFFIX; i++)
		last_connect_suffix_tones[i] = -1;
	last_connect_capture_valid = (count == n);
}
