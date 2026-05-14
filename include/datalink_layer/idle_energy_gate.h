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

#ifndef INC_DATALINK_LAYER_IDLE_ENERGY_GATE_H_
#define INC_DATALINK_LAYER_IDLE_ENERGY_GATE_H_

#include <cmath>

/*
 * Idle-scan cadence energy gate — IDLE_SCAN_CADENCE_RESEARCH.md §5.1 / §6 Step 3.
 *
 * cl_arq_controller::process_main()'s IDLE/DROPPED block calls
 * measure_signal_only() every ~2 ms loop, which runs the FIR_rx_time_sync
 * filter (via passband_to_baseband_decimated). That FIR is dead-weight work
 * on a quiet channel — the IDLE path only updates a GUI signal-strength meter,
 * it does NOT detect calls (call detection is receive_hail_pattern() in the
 * LISTENING state). This gate is the same two-stage pattern already shipping
 * for the ACK detector (Opt 1, arq_common.cc:4099-4124): a cheap O(N) raw-
 * passband RMS probe decides whether the expensive FIR is worth running.
 *
 * IDLE_ENERGY_GATE_RMS — calibrated from a MEASURED RPi1 quiet-channel capture
 * (idle_gate_measure.py, link_status:Idle, 70 s):
 *   measured quiet-channel raw-passband RMS: mean 0.000369, max 0.000383.
 * 0.002 sits 5.2x (~14.4 dB) above that measured quiet floor and 10x (20 dB)
 * below a real on-air MFSK tone (~0.02 RMS, see Opt 1 arq_common.cc:4099) —
 * comfortably above silence/rx-mute noise, comfortably below the weakest real
 * signal. The probe is RAW PASSBAND (the buffer as captured, before any
 * passband_to_baseband* call) precisely so the gate itself costs no FIR.
 */
static const double IDLE_ENERGY_GATE_RMS = 0.002;

/*
 * Raw-passband RMS of buf[0..n) — O(N), one pass, no allocation. A negative
 * return (-1.0) flags a malformed call (null/empty buffer).
 */
static inline double idle_passband_rms(const double* buf, int n)
{
	if (buf == nullptr || n <= 0)
		return -1.0;
	double sumsq = 0.0;
	for (int i = 0; i < n; i++)
		sumsq += buf[i] * buf[i];
	return std::sqrt(sumsq / (double)n);
}

/*
 * Returns true  => gate OPEN   => there is enough energy to be worth the FIR;
 *                                 the caller should run measure_signal_only().
 *         false => gate CLOSED => the buffer is quiet (RMS < gate_rms);
 *                                 the caller should SKIP the FIR this loop.
 *
 * buf : raw passband samples (the just-captured IDLE buffer).
 * n   : number of samples in buf (the whole signal_period buffer).
 * gate_rms : threshold (normally IDLE_ENERGY_GATE_RMS).
 *
 * O(N), one pass, no allocation — comparable in cost to measure_signal_stregth
 * and far below the FIR it gates. Fails OPEN on a malformed call: a gate must
 * never be the reason the modem stops measuring the channel.
 */
static inline bool idle_energy_gate_open(const double* buf, int n, double gate_rms)
{
	double rms = idle_passband_rms(buf, n);
	if (rms < 0.0)
		return true;   // fail-open: never suppress the FIR on a malformed call
	return rms >= gate_rms;
}

#endif  // INC_DATALINK_LAYER_IDLE_ENERGY_GATE_H_
