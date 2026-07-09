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

#include "physical_layer/data_container.h"
#include "debug/canary_guard.h"
#include <cstring>


cl_data_container::cl_data_container()
{
	this->nData=0;
	this->nBits=0;
	this->Nc=0;
	this->M=0;
	this->Nofdm=0;
	this->Nfft=0;
	this->Ngi=0;
	this->Nsymb=0;
	this->data_bit=NULL;
	this->data_bit_energy_dispersal=NULL;
	this->data_byte=NULL;
	this->encoded_data=NULL;
	this->bit_interleaved_data=NULL;
	this->ofdm_time_freq_interleaved_data=NULL;
	this->ofdm_time_freq_deinterleaved_data=NULL;
	this->modulated_data=NULL;
	this->ofdm_framed_data=NULL;
	this->ofdm_symbol_modulated_data=NULL;
	this->preamble_symbol_modulated_data=NULL;
	this->preamble_data=NULL;
	this->ofdm_symbol_demodulated_data=NULL;
	this->ofdm_deframed_data=NULL;
	this->ofdm_deframed_data_without_amplitude_restoration=NULL;
	this->equalized_data= NULL;
	this->equalized_data_without_amplitude_restoration= NULL;
	this->demodulated_data=NULL;
	this->deinterleaved_data=NULL;
	this->hd_decoded_data_bit=NULL;
	this->hd_decoded_data_byte=NULL;

	this->buffer_Nsymb=0;
	this->preamble_nSymb=0;

	this->passband_data=NULL;
	this->passband_delayed_data=NULL;
	this->ready_to_process_passband_delayed_data=NULL;
	this->baseband_data=NULL;
	this->baseband_data_interpolated=NULL;
	this->baseband_data_decimated=NULL;
	this->baseband_data_fine_slice=NULL;
	this->baseband_data_fine_slice_size=0;

	this->frames_to_read=0;
	this->data_ready=0;
	this->nUnder_processing_events=0;
	this->rx_mute=0;
	this->ring_write_index=0;
	this->interpolation_rate=0;

	this->total_frame_size=0;

	this->passband_data_tx=NULL;
	this->passband_data_tx_buffer=NULL;
	this->passband_data_tx_filtered_fir_1=NULL;
	this->passband_data_tx_filtered_fir_2=NULL;
	this->ready_to_transmit_passband_data_tx=NULL;

	this->bit_energy_dispersal_sequence=NULL;
}

cl_data_container::~cl_data_container()
{
	this->deinit();
}

// Set ONLY the per-config geometry SCALARS + STAGE the target buffer_Nsymb. No alloc, no
// publish of the C1-visible atomic buffer_Nsymb. Safe to run OUTSIDE capture_prep_mutex:
// Nofdm/interpolation_rate are geometry-invariant across configs (writing the same value),
// the rest (nData/nBits/Nc/M/Nfft/Ngi/Nsymb/preamble_nSymb/total_frame_size/fine_slice_size)
// are read only by the ARQ thread (the same thread that calls this), and buffer_Nsymb is left
// untouched (only STAGED) so the audio thread keeps seeing the OLD consistent ring geometry
// until publish_active_ring() flips it under the lock. See PRECOOK plan §2.1/§3.
void cl_data_container::set_active_geometry(int nData, int Nc, int M, int Nfft, int Nofdm, int Nsymb, int preamble_nSymb, int frequency_interpolation_rate)
{
	this->nData=nData;
	this->nBits=nData*(int)log2(M);
	this->Nc=Nc;
	this->M=M;
	this->Nofdm=Nofdm;
	this->Nfft=Nfft;
	this->Ngi=Nofdm-Nfft;
	this->Nsymb=Nsymb;
	this->preamble_nSymb=preamble_nSymb;
	this->interpolation_rate=frequency_interpolation_rate;
	this->total_frame_size=Nofdm*(Nsymb+preamble_nSymb)*frequency_interpolation_rate;
	// Plan-B Step 5: full-rate fine-slice scratch logical size (physical array ≥ this).
	this->baseband_data_fine_slice_size=(3*preamble_nSymb+4)*Nofdm*frequency_interpolation_rate;

	// PRECOOK V2 (Step C) — LOUD CLOSURE. The pinned scratch/TX buffers (alloc_shared_buffers) were
	// sized at the ABSOLUTE max over the CLOSED domain (Step A: both bandwidths × full ladder × thin
	// grid × startup). Every REACHABLE geometry therefore fits — a value that EXCEEDS a pinned
	// capacity is a config outside the domain the precook was built for. Publishing it would drive
	// the demod/TX scratch reads (Nsymb·Nc planes, nData, total_frame_size) out of the fixed
	// allocation = silent heap corruption on a life-critical PHY. Refuse it LOUDLY and abort rather
	// than corrupt: this converts any future unhandled state into a named startup-class abort, not a
	// live 0-connect. In-domain (the live gate) this NEVER fires. (publish_active_ring separately
	// clamps the C1-visible ring sp/buffer_Nsymb; this guards the ARQ-thread scratch dims.)
	if(this->precook_ring_pinned)
	{
		const char* over = nullptr; int need = 0, cap = 0;
		if(this->pinned_capacity_Nc > 0 && Nc > this->pinned_capacity_Nc)
			{ over="Nc"; need=Nc; cap=this->pinned_capacity_Nc; }
		else if(this->pinned_capacity_nsymb_nc > 0 && Nsymb*Nc > this->pinned_capacity_nsymb_nc)
			{ over="Nsymb*Nc"; need=Nsymb*Nc; cap=this->pinned_capacity_nsymb_nc; }
		else if(this->pinned_capacity_nData > 0 && nData > this->pinned_capacity_nData)
			{ over="nData"; need=nData; cap=this->pinned_capacity_nData; }
		else if(this->pinned_capacity_total_frame_size > 0 && this->total_frame_size > this->pinned_capacity_total_frame_size)
			{ over="total_frame_size"; need=this->total_frame_size; cap=this->pinned_capacity_total_frame_size; }
		else if(this->pinned_capacity_preamble > 0 && preamble_nSymb > this->pinned_capacity_preamble)
			{ over="preamble_nSymb"; need=preamble_nSymb; cap=this->pinned_capacity_preamble; }
		else if(this->pinned_capacity_fine_slice > 0 && this->baseband_data_fine_slice_size > this->pinned_capacity_fine_slice)
			{ over="fine_slice"; need=this->baseband_data_fine_slice_size; cap=this->pinned_capacity_fine_slice; }
		if(over != nullptr)
		{
			fprintf(stderr, "[PRECOOK-REFUSE] geometry exceeds pinned capacity: dim=%s need=%d cap=%d "
				"(Nc=%d Nsymb=%d nData=%d Nofdm=%d) — config OUTSIDE the closed precook domain; "
				"aborting rather than corrupt the pinned PHY scratch. This is a domain-closure bug: "
				"extend precook_pin_shared_ring's walk to cover this geometry.\n",
				over, need, cap, Nc, Nsymb, nData, Nofdm);
			fflush(stderr);
			abort();
		}
	}

	// Buffer: frame + turnaround + frame + tail margin.
	// Tail margin gives headroom for preambles that land close to buffer end.
	double sym_time_ms = 1000.0 * Nofdm * frequency_interpolation_rate / 48000.0;
	int frame_symb = preamble_nSymb + Nsymb;
	// NB OFDM (Nc=10) has longer inter-batch turnaround than WB (Nc=50):
	// NB ACK pattern TX (~1.5s) + Commander processing/compression (~1.5s) = ~3s.
	// 4000ms gives ~1s headroom for radio propagation delay.
	double turnaround_ms = (Nc <= 10) ? 4000.0 : 2000.0;
	int turnaround_symb = (int)ceil(turnaround_ms / sym_time_ms) + 4;
	int margin = frame_symb / 2;
	if(margin < 20) margin = 20;
	if(margin > 50) margin = 50;
	int min_buf = frame_symb + turnaround_symb + frame_symb + margin;
	if(min_buf < 32) min_buf = 32;
	if(this->buffer_Nsymb_min > 0 && min_buf < this->buffer_Nsymb_min)
		min_buf = this->buffer_Nsymb_min;
	this->staged_buffer_Nsymb = min_buf;   // NOT published to the atomic here (see publish_active_ring)
}

// Publish the staged geometry to the C1-visible ring state ATOMICALLY. CALLER MUST HOLD
// capture_prep_mutex (the audio capture thread reads buffer_Nsymb outside the lock to size its
// wait, then re-reads sp INSIDE the lock and uses THAT for the ring write — so flipping
// buffer_Nsymb together with ring_write_index=0 under the lock guarantees the audio thread
// never writes with a new modulus against a stale cursor). Leaf: pure scalar stores + one
// bounded memset of the (never-freed) ring; NO call-out that could re-take the mutex (INV-8).
void cl_data_container::publish_active_ring()
{
	int active_bn = this->staged_buffer_Nsymb;
	// Fail-safe tripwire: the pinned ring is sized (precook_pin_shared_ring) for the MAX
	// buffer_Nsymb across every FULL_CONFIG_LADDER config in BOTH bandwidths, so a per-config
	// natural (or the robust-floor seat) never exceeds it. If some out-of-ladder/experimental
	// config ever staged a larger window, publishing it would drive C1's ring write out of the
	// physical allocation — clamp to the pinned capacity (acquisition degrades, but NO OOB) and
	// log loudly rather than corrupt the heap. Never fires in standard operation.
	if(this->precook_ring_pinned && this->pinned_capacity_buffer_Nsymb > 0
	   && active_bn > this->pinned_capacity_buffer_Nsymb)
	{
		fprintf(stderr, "[PRECOOK] FATAL: staged buffer_Nsymb=%d exceeds pinned capacity=%d "
			"(config out of sized range) — clamping to avoid ring OOB\n",
			active_bn, this->pinned_capacity_buffer_Nsymb);
		fflush(stderr);
		active_bn = this->pinned_capacity_buffer_Nsymb;
	}
	int sp = this->Nofdm * active_bn * this->interpolation_rate;
	// SAMPLE-BASED OOB tripwire (§CAP-STALE root defense): the buffer_Nsymb clamp above only bounds
	// the SYMBOL count. The physical ring is sized in SAMPLES (Nofdm·bn·interp), so a config whose
	// Nofdm exceeds the pin's ref_Nofdm can overrun the allocation even with buffer_Nsymb ≤ capacity
	// (this was the live-acq failure: bundles built at Nofdm=310 vs a ring pinned at 292). The root
	// fix (bundles inherit the live gi) keeps Nofdm invariant so this never fires, but enforce the
	// sp invariant here too: clamp the published window to what physically fits rather than let C1's
	// 2·sp mirror write / the demod read run OOB. Log loudly — a hit means a geometry regression.
	if(this->precook_ring_pinned && this->pinned_capacity_samples > 0
	   && sp > this->pinned_capacity_samples)
	{
		int fit_bn = (this->Nofdm > 0 && this->interpolation_rate > 0)
			? this->pinned_capacity_samples / (this->Nofdm * this->interpolation_rate) : active_bn;
		fprintf(stderr, "[PRECOOK] FATAL: published sp=%d (Nofdm=%d bn=%d interp=%d) exceeds pinned "
			"sample capacity=%d — Nofdm/geometry regression vs pin; clamping bn %d→%d to avoid ring OOB\n",
			sp, this->Nofdm, active_bn, this->interpolation_rate, this->pinned_capacity_samples,
			active_bn, fit_bn);
		fflush(stderr);
		active_bn = (fit_bn > 0) ? fit_bn : 1;
		sp = this->Nofdm * active_bn * this->interpolation_rate;
	}
	this->buffer_Nsymb = active_bn;   // ATOMIC publish — the C1-critical store
	this->frames_to_read = this->preamble_nSymb + this->Nsymb;
	this->data_ready = 0;
	this->nUnder_processing_events = 0;
	this->ring_write_index = 0;
	// Bounded reset (≤ current allocation), NOT a free/malloc: zero only the active window so the
	// energy gate skips empty frames until real audio arrives (INV-7); the modulus change would
	// otherwise leave a mis-indexed accumulation. C2 copies exactly sp; C1 writes within 2·sp.
	if(this->passband_delayed_data != NULL && sp > 0)
		memset(this->passband_delayed_data, 0, (size_t)2*sp*sizeof(double));
}

// Allocate EVERY variable-size buffer ONCE at the given MAX geometry and pin the ring. Called
// once at startup (before the capture thread spawns) via precook_pin_shared_ring(). Frees the
// natural-sized buffers a prior load allocated and reallocs them at MAX, memsets the ring once.
// Leaves the per-config active SCALARS untouched (they keep describing the currently loaded
// config, whose buffer_Nsymb ≤ max_buffer_Nsymb and Nsymb ≤ max_Nsymb → every consumer indexes
// in bounds). PRECOOK plan §2.1.
void cl_data_container::alloc_shared_buffers(int max_nData, int max_Nc, int max_M, int max_Nfft,
	int max_Nofdm, int max_Nsymb, int max_preamble_nSymb, int frequency_interpolation_rate,
	int max_buffer_Nsymb)
{
	const int CTRL_SUFFIX_FEC_MAX_NSYMB = 128;  // 4×16 WB connect base reps + 64 GF16RA_MAX_N (see set_size note)
	int alloc_Nsymb = (max_Nsymb > 48) ? max_Nsymb : 48;
	if (alloc_Nsymb < CTRL_SUFFIX_FEC_MAX_NSYMB) alloc_Nsymb = CTRL_SUFFIX_FEC_MAX_NSYMB;

	// N_MAX-fixed arrays (size-invariant)
	CDELETE(this->data_bit); this->data_bit=CNEW(int, N_MAX, "dc.data_bit");
	CDELETE(this->data_bit_energy_dispersal); this->data_bit_energy_dispersal=CNEW(int, N_MAX, "dc.data_bit_energy_dispersal");
	CDELETE(this->data_byte); this->data_byte=CNEW(int, N_MAX, "dc.data_byte");
	CDELETE(this->encoded_data); this->encoded_data=CNEW(int, N_MAX, "dc.encoded_data");
	CDELETE(this->bit_interleaved_data); this->bit_interleaved_data=CNEW(int, N_MAX, "dc.bit_interleaved_data");
	CDELETE(this->demodulated_data); this->demodulated_data=CNEW(float, N_MAX, "dc.demodulated_data");
	CDELETE(this->deinterleaved_data); this->deinterleaved_data=CNEW(float, N_MAX, "dc.deinterleaved_data");
	CDELETE(this->hd_decoded_data_bit); this->hd_decoded_data_bit=CNEW(int, N_MAX, "dc.hd_decoded_data_bit");
	CDELETE(this->hd_decoded_data_byte); this->hd_decoded_data_byte=CNEW(int, N_MAX, "dc.hd_decoded_data_byte");
	CDELETE(this->bit_energy_dispersal_sequence); this->bit_energy_dispersal_sequence=CNEW(int, N_MAX, "dc.bit_energy_dispersal_seq");

	// Scratch (Nsymb·Nc / Nofdm·Nsymb / preamble·*) at MAX
	CDELETE(this->modulated_data); this->modulated_data=CNEW(std::complex<double>, max_nData, "dc.modulated_data");
	CDELETE(this->ofdm_framed_data); this->ofdm_framed_data=CNEW(std::complex<double>, alloc_Nsymb*max_Nc, "dc.ofdm_framed_data");
	CDELETE(this->ofdm_time_freq_interleaved_data); this->ofdm_time_freq_interleaved_data=CNEW(std::complex<double>, max_Nsymb*max_Nc, "dc.ofdm_time_freq_interleaved_data");
	CDELETE(this->ofdm_time_freq_deinterleaved_data); this->ofdm_time_freq_deinterleaved_data=CNEW(std::complex<double>, max_Nsymb*max_Nc, "dc.ofdm_time_freq_deinterleaved_data");
	CDELETE(this->ofdm_symbol_modulated_data); this->ofdm_symbol_modulated_data=CNEW(std::complex<double>, max_Nofdm*alloc_Nsymb, "dc.ofdm_symbol_modulated_data");
	CDELETE(this->ofdm_symbol_demodulated_data); this->ofdm_symbol_demodulated_data=CNEW(std::complex<double>, max_Nsymb*max_Nc, "dc.ofdm_symbol_demodulated_data");
	CDELETE(this->ofdm_deframed_data); this->ofdm_deframed_data=CNEW(std::complex<double>, max_Nsymb*max_Nc, "dc.ofdm_deframed_data");
	CDELETE(this->ofdm_deframed_data_without_amplitude_restoration); this->ofdm_deframed_data_without_amplitude_restoration=CNEW(std::complex<double>, max_Nsymb*max_Nc, "dc.ofdm_deframed_data_noamp");
	CDELETE(this->equalized_data); this->equalized_data=CNEW(std::complex<double>, max_Nsymb*max_Nc, "dc.equalized_data");
	CDELETE(this->equalized_data_without_amplitude_restoration); this->equalized_data_without_amplitude_restoration=CNEW(std::complex<double>, max_Nsymb*max_Nc, "dc.equalized_data_noamp");
	CDELETE(this->preamble_symbol_modulated_data); this->preamble_symbol_modulated_data=CNEW(std::complex<double>, max_preamble_nSymb*max_Nofdm, "dc.preamble_symbol_mod");
	CDELETE(this->preamble_data); this->preamble_data=CNEW(std::complex<double>, max_preamble_nSymb*max_Nc, "dc.preamble_data");

	// Ring + baseband planes at MAX buffer_Nsymb
	int max_sp = max_Nofdm * max_buffer_Nsymb * frequency_interpolation_rate;
	CDELETE(this->passband_delayed_data); this->passband_delayed_data=CNEW(double, 2*max_sp, "dc.passband_delayed_data");
	memset(this->passband_delayed_data, 0, (size_t)2*max_sp*sizeof(double));
	CDELETE(this->ready_to_process_passband_delayed_data); this->ready_to_process_passband_delayed_data=CNEW(double, max_sp, "dc.ready_to_process_pdd");
	memset(this->ready_to_process_passband_delayed_data, 0, (size_t)max_sp*sizeof(double));
	CDELETE(this->baseband_data); this->baseband_data=CNEW(std::complex<double>, max_Nofdm*max_buffer_Nsymb, "dc.baseband_data");
	CDELETE(this->baseband_data_interpolated); this->baseband_data_interpolated=CNEW(std::complex<double>, max_sp, "dc.baseband_data_interp");
	CDELETE(this->baseband_data_decimated); this->baseband_data_decimated=CNEW(std::complex<double>, max_Nofdm*max_buffer_Nsymb, "dc.baseband_data_decimated");
	int max_fine = (3*max_preamble_nSymb+4)*max_Nofdm*frequency_interpolation_rate;
	CDELETE(this->baseband_data_fine_slice); this->baseband_data_fine_slice=CNEW(std::complex<double>, max_fine, "dc.baseband_data_fine_slice");

	// passband_data: max(frame, ack) at MAX
	int max_passband_frame = (max_Nsymb + max_preamble_nSymb) * max_Nofdm * frequency_interpolation_rate;
	int max_passband_ack = 16 * max_Nofdm * frequency_interpolation_rate;
	CDELETE(this->passband_data); this->passband_data=CNEW(double, (max_passband_frame > max_passband_ack) ? max_passband_frame : max_passband_ack, "dc.passband_data");

	// TX buffers at MAX total_frame_size
	int max_total_frame_size = max_Nofdm*(max_Nsymb+max_preamble_nSymb)*frequency_interpolation_rate;
	CDELETE(this->passband_data_tx); this->passband_data_tx=CNEW(double, max_total_frame_size, "dc.passband_data_tx");
	CDELETE(this->passband_data_tx_buffer); this->passband_data_tx_buffer=CNEW(double, 3*max_total_frame_size, "dc.passband_data_tx_buffer");
	CDELETE(this->passband_data_tx_filtered_fir_1); this->passband_data_tx_filtered_fir_1=CNEW(double, 2*max_total_frame_size, "dc.passband_data_tx_filt1");
	CDELETE(this->passband_data_tx_filtered_fir_2); this->passband_data_tx_filtered_fir_2=CNEW(double, 2*max_total_frame_size, "dc.passband_data_tx_filt2");
	CDELETE(this->ready_to_transmit_passband_data_tx); this->ready_to_transmit_passband_data_tx=CNEW(double, max_total_frame_size, "dc.ready_to_tx_passband");

	(void)max_M; (void)max_Nfft;
	this->pinned_capacity_buffer_Nsymb = max_buffer_Nsymb;
	this->pinned_capacity_samples = max_sp;   // physical per-mirror sample capacity (Nofdm·bn·interp)
	// PRECOOK V2 (Step A): record the remaining per-dimension capacities the scratch/TX buffers were
	// sized to, so Step B's build asserts and Step C's set_active_geometry refuse can compare against
	// the exact allocation. alloc_Nsymb (≥ max_Nsymb, ≥128 ctrl-suffix floor) is what the scratch
	// planes were actually allocated at; use max_Nsymb for the logical demod-plane capacity check.
	this->pinned_capacity_Nc               = max_Nc;
	this->pinned_capacity_nsymb_nc         = alloc_Nsymb * max_Nc;   // scratch planes sized alloc_Nsymb·Nc
	this->pinned_capacity_nData            = max_nData;
	this->pinned_capacity_total_frame_size = max_total_frame_size;
	this->pinned_capacity_preamble         = max_preamble_nSymb;
	this->pinned_capacity_fine_slice       = max_fine;
	this->precook_ring_pinned = true;   // from here on, set_size()/init() take the no-free path
}

void cl_data_container::set_size(int nData, int Nc, int M, int Nfft , int Nofdm, int Nsymb, int preamble_nSymb, int frequency_interpolation_rate)
{
	// PRECOOK split: scalars first (also computes staged_buffer_Nsymb / total_frame_size /
	// fine_slice_size). The physical alloc below runs only when NOT pinned (legacy: the caller
	// deinit()'d first, or this is a direct force_resize/force_set_natural realloc), or on the
	// rare NB↔WB grow past the pinned capacity. The publish at the end sets the C1-visible
	// atomic buffer_Nsymb + ring cursor reset. In ALL callers of set_size the capture thread is
	// either not yet spawned (startup) or the caller holds capture_prep_mutex across the cycle
	// (load_configuration Bug #42 lock, force_resize/force_set_natural own lock) — so the publish
	// here is always under the lock.
	set_active_geometry(nData, Nc, M, Nfft, Nofdm, Nsymb, preamble_nSymb, frequency_interpolation_rate);
	int min_buf = this->staged_buffer_Nsymb;

	// When pinned, the physical buffers are already allocated at MAX and must NEVER be freed
	// (the whole point — C1 keeps a stable ring pointer). set_size then only sets scalars +
	// publishes (force_resize/force_set_natural, which hold the lock). Legacy (unpinned): the
	// caller deinit()'d first (or this is a direct realloc), so allocate.
	bool need_alloc = !this->precook_ring_pinned;
	if(need_alloc)
	{
		this->data_bit=CNEW(int, N_MAX, "dc.data_bit");
		this->data_bit_energy_dispersal=CNEW(int, N_MAX, "dc.data_bit_energy_dispersal");
		this->data_byte=CNEW(int, N_MAX, "dc.data_byte");
		this->encoded_data=CNEW(int, N_MAX, "dc.encoded_data");
		this->bit_interleaved_data=CNEW(int, N_MAX, "dc.bit_interleaved_data");
		this->modulated_data=CNEW(std::complex<double>, nData, "dc.modulated_data");
		// ACK pattern generation reuses ofdm_framed_data and ofdm_symbol_modulated_data.
		// NB Sidelnikov ACK uses up to 48 symbols (M=4), WB Welch-Costas uses 16.
		// For high-order modulations (16QAM+), Nsymb < 48, so allocate for the max.
		// §19 (tier2-suffix-fec-design.md §19.4 C2): the Tier-2 FEC CONNECT pattern
		// is connect_base(16 WB) + coded ctrl-suffix (up to gf16ra GF16RA_MAX_N=64)
		// = up to 80 symbols, which EXCEEDS the old 48 floor on short-frame robust
		// configs (ROBUST_0 Nsymb < 48) → would overflow ofdm_framed_data /
		// ofdm_symbol_modulated_data. (Uncoded path unaffected — 16+13=29<48.)
		// §20 (§20.3 C2): base-pattern combining emits the 16-sym base block up to
		// MAX_CONNECT_PREAMBLE_REPS=4 times → R×16 + coded suffix(≤64) = 4*16+64=128
		// symbols. Floor at 128 so the combined+coded CONNECT pattern always fits
		// regardless of repfact and reps. (Kept as a literal — data_container does not
		// include mfsk.h/mfsk_ctrl_codec.h; the two constants are MAX_CONNECT_PREAMBLE
		// _REPS=4 and GF16RA_MAX_N=64. If either grows, raise this in lockstep.)
		const int CTRL_SUFFIX_FEC_MAX_NSYMB = 128;  // 4×16 WB connect base reps + 64 GF16RA_MAX_N
		int alloc_Nsymb = (Nsymb > 48) ? Nsymb : 48;
		if (alloc_Nsymb < CTRL_SUFFIX_FEC_MAX_NSYMB) alloc_Nsymb = CTRL_SUFFIX_FEC_MAX_NSYMB;
		this->ofdm_framed_data=CNEW(std::complex<double>, alloc_Nsymb*Nc, "dc.ofdm_framed_data");
		this->ofdm_time_freq_interleaved_data=CNEW(std::complex<double>, Nsymb*Nc, "dc.ofdm_time_freq_interleaved_data");
		this->ofdm_time_freq_deinterleaved_data=CNEW(std::complex<double>, Nsymb*Nc, "dc.ofdm_time_freq_deinterleaved_data");
		this->ofdm_symbol_modulated_data=CNEW(std::complex<double>, Nofdm*alloc_Nsymb, "dc.ofdm_symbol_modulated_data");
		this->ofdm_symbol_demodulated_data=CNEW(std::complex<double>, Nsymb*Nc, "dc.ofdm_symbol_demodulated_data");
		this->ofdm_deframed_data=CNEW(std::complex<double>, Nsymb*Nc, "dc.ofdm_deframed_data");
		this->ofdm_deframed_data_without_amplitude_restoration=CNEW(std::complex<double>, Nsymb*Nc, "dc.ofdm_deframed_data_noamp");
		this->equalized_data=CNEW(std::complex<double>, Nsymb*Nc, "dc.equalized_data");
		this->equalized_data_without_amplitude_restoration=CNEW(std::complex<double>, Nsymb*Nc, "dc.equalized_data_noamp");
		this->preamble_symbol_modulated_data=CNEW(std::complex<double>, preamble_nSymb*Nofdm, "dc.preamble_symbol_mod");
		this->preamble_data=CNEW(std::complex<double>, preamble_nSymb*Nc, "dc.preamble_data");
		this->demodulated_data=CNEW(float, N_MAX, "dc.demodulated_data");
		this->deinterleaved_data=CNEW(float, N_MAX, "dc.deinterleaved_data");
		this->hd_decoded_data_bit=CNEW(int, N_MAX, "dc.hd_decoded_data_bit");
		this->hd_decoded_data_byte=CNEW(int, N_MAX, "dc.hd_decoded_data_byte");

		this->bit_energy_dispersal_sequence=CNEW(int, N_MAX, "dc.bit_energy_dispersal_seq");

		// ACK pattern uses 16*Nofdm*freq_interp passband samples, which can exceed
		// the normal frame size at high modulations (16QAM+). Allocate for whichever is larger.
		int passband_frame = (Nsymb + preamble_nSymb) * Nofdm * frequency_interpolation_rate;
		int passband_ack = 16 * Nofdm * frequency_interpolation_rate;
		this->passband_data=CNEW(double, (passband_frame > passband_ack) ? passband_frame : passband_ack, "dc.passband_data");
		this->passband_delayed_data=CNEW(double, 2*Nofdm*min_buf*frequency_interpolation_rate, "dc.passband_delayed_data");
		memset(this->passband_delayed_data, 0, 2*Nofdm*min_buf*frequency_interpolation_rate * sizeof(double));
		this->ready_to_process_passband_delayed_data=CNEW(double, Nofdm*min_buf*frequency_interpolation_rate, "dc.ready_to_process_pdd");
		memset(this->ready_to_process_passband_delayed_data, 0, Nofdm*min_buf*frequency_interpolation_rate * sizeof(double));
		this->baseband_data=CNEW(std::complex<double>, Nofdm*min_buf, "dc.baseband_data");
		this->baseband_data_interpolated=CNEW(std::complex<double>, Nofdm*min_buf*frequency_interpolation_rate, "dc.baseband_data_interp");
		// Plan-B: decimated-rate buffer for the Schmidl-Cox preamble search.
		// passband_to_baseband_decimated writes (Nofdm*buffer_Nsymb*freq_interp)/M
		// = Nofdm*buffer_Nsymb samples here. Allocated now; first consumer wired
		// in a later Plan-B step.
		this->baseband_data_decimated=CNEW(std::complex<double>, Nofdm*min_buf, "dc.baseband_data_decimated");
		// Plan-B Step 5: full-rate fine-slice scratch. The fine slice is
		// 3*pream_len_full plus an FIR guard margin on each side (so the FIR
		// transient never reaches the searched region). FIR_rx_time_sync has
		// <= Ngi*interp taps, so 2*Ngi*interp covers both guard margins with
		// headroom. (3*preamble_nSymb+4)*Nofdm*freq_interp is a safe upper bound.
		this->baseband_data_fine_slice=CNEW(std::complex<double>, this->baseband_data_fine_slice_size, "dc.baseband_data_fine_slice");

		this->passband_data_tx=CNEW(double, this->total_frame_size, "dc.passband_data_tx");
		this->passband_data_tx_buffer=CNEW(double, 3*this->total_frame_size, "dc.passband_data_tx_buffer");
		this->passband_data_tx_filtered_fir_1=CNEW(double, 2*this->total_frame_size, "dc.passband_data_tx_filt1");
		this->passband_data_tx_filtered_fir_2=CNEW(double, 2*this->total_frame_size, "dc.passband_data_tx_filt2");
		this->ready_to_transmit_passband_data_tx=CNEW(double, this->total_frame_size, "dc.ready_to_tx_passband");
	}
	(void)min_buf;

	// Publish the C1-visible ring state (atomic buffer_Nsymb + cursor reset + bounded memset).
	// Buffer is zero-initialized. Do NOT fill with random noise: after a config switch (e.g.
	// MFSK→OFDM gearshift), random data in the buffer passes the energy gate and gets decoded as
	// noise, producing garbage channel estimates (mean_H ≈ 0.12). Zero-fill lets the energy gate
	// skip empty frames until real audio arrives from the capture thread.
	publish_active_ring();
}

void cl_data_container::deinit()
{
	this->nData=0;
	this->nBits=0;
	this->Nc=0;
	this->M=0;
	this->Nofdm=0;
	this->Nfft=0;
	this->Ngi=0;
	this->Nsymb=0;
	this->total_frame_size=0;

	CDELETE(this->data_bit);
	CDELETE(this->data_bit_energy_dispersal);
	CDELETE(this->data_byte);
	CDELETE(this->encoded_data);
	CDELETE(this->bit_interleaved_data);
	CDELETE(this->modulated_data);
	CDELETE(this->ofdm_framed_data);
	CDELETE(this->ofdm_time_freq_interleaved_data);
	CDELETE(this->ofdm_time_freq_deinterleaved_data);
	CDELETE(this->ofdm_symbol_modulated_data);
	CDELETE(this->ofdm_symbol_demodulated_data);
	CDELETE(this->ofdm_deframed_data);
	CDELETE(this->ofdm_deframed_data_without_amplitude_restoration);
	CDELETE(this->equalized_data);
	CDELETE(this->equalized_data_without_amplitude_restoration);
	CDELETE(this->preamble_symbol_modulated_data);
	CDELETE(this->preamble_data);
	CDELETE(this->demodulated_data);
	CDELETE(this->deinterleaved_data);
	CDELETE(this->hd_decoded_data_bit);
	CDELETE(this->hd_decoded_data_byte);
	CDELETE(this->bit_energy_dispersal_sequence);

	this->buffer_Nsymb=0;

	CDELETE(this->passband_data);
	CDELETE(this->passband_delayed_data);
	CDELETE(this->ready_to_process_passband_delayed_data);
	CDELETE(this->baseband_data);
	CDELETE(this->baseband_data_interpolated);
	CDELETE(this->baseband_data_decimated);
	CDELETE(this->baseband_data_fine_slice);
	this->baseband_data_fine_slice_size=0;
	CDELETE(this->passband_data_tx);
	CDELETE(this->passband_data_tx_buffer);
	CDELETE(this->passband_data_tx_filtered_fir_1);
	CDELETE(this->passband_data_tx_filtered_fir_2);
	CDELETE(this->ready_to_transmit_passband_data_tx);

	this->frames_to_read=0;
	this->data_ready=0;
	this->nUnder_processing_events=0;
	this->ring_write_index=0;
	this->interpolation_rate=0;
}
