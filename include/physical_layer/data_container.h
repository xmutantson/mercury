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

#ifndef INC_DATA_CONTAINER_H_
#define INC_DATA_CONTAINER_H_

#include <complex>
#include <atomic>
#define _Atomic(X) std::atomic< X >

#include "physical_defines.h"
#include "misc.h"

class cl_data_container
{
	public:
	cl_data_container();
	~cl_data_container();
	int* data_bit;
	int* data_bit_energy_dispersal;
	int* data_byte;
	int* encoded_data;
	int* bit_interleaved_data;
	std::complex <double>* modulated_data;
	std::complex <double>* ofdm_symbol_modulated_data;
	std::complex <double>* ofdm_symbol_demodulated_data;
	std::complex <double>* ofdm_framed_data;
	std::complex <double>* ofdm_time_freq_interleaved_data;
	std::complex <double>* ofdm_time_freq_deinterleaved_data;
	std::complex <double>* ofdm_deframed_data;
	std::complex <double>* ofdm_deframed_data_without_amplitude_restoration;
	std::complex <double>* equalized_data;
	std::complex <double>* equalized_data_without_amplitude_restoration;
	std::complex <double>* preamble_symbol_modulated_data;
	std::complex <double>* preamble_data;
	double* passband_data;
	double* passband_delayed_data;
	double* ready_to_process_passband_delayed_data;
	std::complex <double>* baseband_data;
	std::complex <double>* baseband_data_interpolated;
	// Plan-B (decimate-before-time_sync): decimated-rate copy of the RX
	// passband, produced by passband_to_baseband_decimated() and consumed by
	// the Schmidl-Cox preamble search. Same element count as baseband_data
	// (Nofdm*buffer_Nsymb) — it holds the whole buffer at the decimated rate.
	std::complex <double>* baseband_data_decimated;
	// Plan-B Step 5: small full-rate scratch slice. The primary preamble
	// search runs its coarse phase on the decimated buffer, then mixes+FIRs
	// only a small full-rate slice around the coarse peak into this buffer
	// for the fine (sub-decimated-grid) refinement. Sized (3*preamble_nSymb+4)
	// *Nofdm*interp — strictly ≥ the largest fine slice (3*pream_len_full +
	// 4*gi_full). Keeps baseband_data_interpolated (still the full-buffer
	// full-rate buffer until Step 6) untouched.
	std::complex <double>* baseband_data_fine_slice;
	int baseband_data_fine_slice_size;
	float* demodulated_data;
	float* deinterleaved_data;
	int* hd_decoded_data_bit;
	int* hd_decoded_data_byte;
	int nData,Nc,M,Nfft,Nofdm,Nsymb,preamble_nSymb,nBits,Ngi,interpolation_rate;
	void set_size(int nData, int Nc,int M,int Nfft, int Nofdm, int Nsymb, int preamble_nSymb, int interpolation_rate);

	// ---- PRECOOK (Stage 1): persistent shared capture ring ----
	// The legacy set_size() FREES (deinit) + new[] + memset the whole ring on every config
	// switch, held under capture_prep_mutex for ~100-200 ms → the audio-capture thread (C1)
	// blocks the whole time → demod goes deaf (the anchor stall, S2). Precook allocates every
	// buffer ONCE at startup at the MAX geometry across all reachable configs and NEVER frees
	// it; a config switch then only publishes the per-config geometry SCALARS + resets the ring
	// cursor + does a bounded memset — a sub-µs leaf critical section. See
	// _research/PRECOOK_IMPLEMENTATION_PLAN.md §2/§7 and
	// fact-documents/data-flow-precook-ring.md.
	//
	// precook_ring_pinned: true once alloc_shared_buffers() has run. When set, set_size() no
	//   longer frees/reallocs (unless a config needs MORE than the pinned capacity, the rare
	//   NB↔WB grow path) and init() publishes geometry via set_active_geometry() instead.
	bool precook_ring_pinned{false};
	// staged_buffer_Nsymb: the per-config target buffer_Nsymb computed by set_active_geometry()
	//   OUTSIDE the capture_prep_mutex (it is NOT the C1-visible atomic — publishing it early,
	//   with a stale ring_write_index, could drive C1 out of bounds). publish_active_ring()
	//   copies it into the atomic buffer_Nsymb UNDER the lock, together with the cursor reset.
	int staged_buffer_Nsymb{0};
	// pinned_capacity_buffer_Nsymb: the largest buffer_Nsymb the pinned physical ring can hold
	//   (== the max_buffer_Nsymb alloc_shared_buffers was sized for). A per-config natural
	//   buffer_Nsymb never exceeds it within one bandwidth; only a rare NB↔WB grow can.
	int pinned_capacity_buffer_Nsymb{0};
	// pinned_capacity_samples: the physical per-mirror sample capacity of the pinned passband ring
	//   (== max_Nofdm·max_buffer_Nsymb·interp alloc_shared_buffers sized). publish_active_ring()
	//   clamps the published sp against THIS (not just the buffer_Nsymb symbol count) so a config
	//   whose Nofdm differs from the pin's ref_Nofdm can never drive C1's 2·sp mirror write / the
	//   demod read out of the allocation. 0 until the ring is pinned. See data-flow-precook-ring.md.
	int pinned_capacity_samples{0};
	// PRECOOK V2 (Step A/C) — per-dimension pinned capacities recorded at pin time (the ABSOLUTE
	//   max over the CLOSED geometry-input domain: {NB,WB} × FULL_CONFIG_LADDER × thin-grid Ngrid ×
	//   the startup config, all under the live gi/geometry). Every bundle build (Step B) asserts its
	//   dimension ≤ the matching capacity; set_active_geometry() (Step C) refuses+aborts loudly if a
	//   live geometry ever exceeds one (a closed-domain violation = a named startup-class abort, not
	//   a silent live 0-connect / heap overrun). 0 until the ring is pinned.
	int pinned_capacity_Nc{0};              // max Nc (WB=50): pre_eq + per-symbol subcarrier count
	int pinned_capacity_nsymb_nc{0};        // max (Nsymb·Nc): the demod/equalizer scratch planes
	int pinned_capacity_nData{0};           // max nData: modulated_data
	int pinned_capacity_total_frame_size{0};// max Nofdm·(Nsymb+preamble)·interp: TX passband buffers
	int pinned_capacity_preamble{0};        // max preamble_nSymb: preamble/fine-slice sizing
	int pinned_capacity_fine_slice{0};      // max (3·preamble+4)·Nofdm·interp: baseband_data_fine_slice

	// Allocate EVERY variable-size buffer ONCE at the given MAX geometry and pin the ring
	// (precook_ring_pinned=true). Called once at startup (main.cc, before the capture thread
	// spawns) via cl_telecom_system::precook_pin_shared_ring(). Frees any buffers a prior
	// (natural-sized) load already allocated, reallocs them at MAX, memsets the ring once.
	// Does NOT touch the per-config active geometry SCALARS (they keep describing the currently
	// loaded config). Nofdm/interpolation_rate are geometry-invariant across configs.
	void alloc_shared_buffers(int max_nData, int max_Nc, int max_M, int max_Nfft,
		int max_Nofdm, int max_Nsymb, int max_preamble_nSymb, int interpolation_rate,
		int max_buffer_Nsymb);
	// Set ONLY the per-config geometry scalars (nData/nBits/Nc/M/Nofdm/Nfft/Ngi/Nsymb/
	// preamble_nSymb/interpolation_rate/total_frame_size/baseband_data_fine_slice_size) and STAGE
	// the target buffer_Nsymb into staged_buffer_Nsymb. NO new[]/deinit, NO publish of the
	// C1-visible atomic buffer_Nsymb. Runs OUTSIDE the lock in the precook load path.
	void set_active_geometry(int nData, int Nc, int M, int Nfft, int Nofdm, int Nsymb,
		int preamble_nSymb, int interpolation_rate);
	// Publish the staged geometry to the C1-visible ring state ATOMICALLY (caller MUST hold
	// capture_prep_mutex): buffer_Nsymb=staged, frames_to_read/data_ready/nUnder/ring_write_index
	// reset, and a bounded memset [0, 2·sp) of the (never-freed) passband ring. Leaf: no call-out.
	void publish_active_ring();

	_Atomic(int) frames_to_read;
	_Atomic(int) data_ready;
	_Atomic(int) nUnder_processing_events;
	_Atomic(int) buffer_Nsymb;
	int buffer_Nsymb_min{0};  // Monitor mode: minimum buffer size (0=auto-calculate)
	_Atomic(int) rx_mute;
	_Atomic(int) rx_mute_samples;  // samples zeroed by rx_mute since last MF search
	_Atomic(int) ring_write_index;  // current write position in double-mapped ring buffer

	int total_frame_size;

	double* passband_data_tx;
	double* passband_data_tx_buffer;
	double* passband_data_tx_filtered_fir_1;
	double* passband_data_tx_filtered_fir_2;
	double* ready_to_transmit_passband_data_tx;

	int *bit_energy_dispersal_sequence;

	void deinit();


};


#endif
