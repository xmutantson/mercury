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


#include "physical_layer/ofdm.h"
#include "physical_layer/fir_filter.h"
#include "physical_layer/ldpc.h"
#include "physical_layer/psk.h"
#include <cstring>


// ===========================================================================
// PRECOOK gate helpers — byte-compare owning buffers between an init()-built
// object and its copy_from() copy (BUNDLE_FIELD_CHECKLIST PART 6). Each returns
// NULL on byte-identical, else the name of the first differing field so the
// --test gate localizes a missed field to a single buffer.
// ===========================================================================

const char* cl_FIR::precook_deep_equal(const cl_FIR& o) const
{
	if(filter_nTaps != o.filter_nTaps) return "FIR.filter_nTaps";
	if((filter_coefficients == NULL) != (o.filter_coefficients == NULL)) return "FIR.filter_coefficients(null-mismatch)";
	if(filter_coefficients != NULL && filter_nTaps > 0)
		if(memcmp(filter_coefficients, o.filter_coefficients, sizeof(double) * (size_t)filter_nTaps) != 0)
			return "FIR.filter_coefficients";
	return NULL;
}

const char* cl_psk::precook_deep_equal(const cl_psk& o) const
{
	if(nSymbols != o.nSymbols) return "psk.nSymbols";
	if(nBits != o.nBits) return "psk.nBits";
	if((constellation == NULL) != (o.constellation == NULL)) return "psk.constellation(null-mismatch)";
	if(constellation != NULL && nSymbols > 0)
		if(memcmp(constellation, o.constellation, sizeof(std::complex<double>) * (size_t)nSymbols) != 0)
			return "psk.constellation";
	if((D_buf == NULL) != (o.D_buf == NULL)) return "psk.D_buf(null-mismatch)";
	if(D_buf != NULL && nSymbols > 0)
		if(memcmp(D_buf, o.D_buf, sizeof(float) * (size_t)nSymbols) != 0)
			return "psk.D_buf";
	if((LLR_buf == NULL) != (o.LLR_buf == NULL)) return "psk.LLR_buf(null-mismatch)";
	if(LLR_buf != NULL && nBits > 0)
		if(memcmp(LLR_buf, o.LLR_buf, sizeof(float) * (size_t)nBits) != 0)
			return "psk.LLR_buf";
	return NULL;
}

const char* cl_ldpc::precook_deep_equal(const cl_ldpc& o) const
{
	if(N != o.N)             return "ldpc.N";
	if(P != o.P)             return "ldpc.P";
	if(K != o.K)             return "ldpc.K";
	if(configuration != o.configuration) return "ldpc.configuration";
	if(Cwidth != o.Cwidth)   return "ldpc.Cwidth";
	if(Vwidth != o.Vwidth)   return "ldpc.Vwidth";
	if(dwidth != o.dwidth)   return "ldpc.dwidth";
	// QC matrices must be the SAME pointer (shared const static tables).
	if(QCmatrixEnc != o.QCmatrixEnc) return "ldpc.QCmatrixEnc(ptr)";
	if(QCmatrixC   != o.QCmatrixC)   return "ldpc.QCmatrixC(ptr)";
	if(QCmatrixV   != o.QCmatrixV)   return "ldpc.QCmatrixV(ptr)";
	if(QCmatrixd   != o.QCmatrixd)   return "ldpc.QCmatrixd(ptr)";
	if((R == NULL) != (o.R == NULL)) return "ldpc.R(null-mismatch)";
	if(R != NULL && N > 0 && Vwidth > 0)
		if(memcmp(R, o.R, sizeof(double) * (size_t)N * (size_t)Vwidth) != 0) return "ldpc.R";
	if((Q == NULL) != (o.Q == NULL)) return "ldpc.Q(null-mismatch)";
	if(Q != NULL && N > 0 && Vwidth > 0)
		if(memcmp(Q, o.Q, sizeof(double) * (size_t)N * (size_t)Vwidth) != 0) return "ldpc.Q";
	if((V_pos == NULL) != (o.V_pos == NULL)) return "ldpc.V_pos(null-mismatch)";
	if(V_pos != NULL && P > 0 && Cwidth > 0)
		if(memcmp(V_pos, o.V_pos, sizeof(int) * (size_t)P * (size_t)Cwidth) != 0) return "ldpc.V_pos";
	return NULL;
}

const char* cl_pilot_configurator::precook_deep_equal(const cl_pilot_configurator& o) const
{
	if(Nc_max != o.Nc_max)   return "pilot.Nc_max";
	if(nPilots != o.nPilots) return "pilot.nPilots";
	if(sparse_wide_data_carriers != o.sparse_wide_data_carriers) return "pilot.sparse_wide_data_carriers";
	if((virtual_carrier == NULL) != (o.virtual_carrier == NULL)) return "pilot.virtual_carrier(null-mismatch)";
	if(virtual_carrier != NULL && Nc_max > 0)
		if(memcmp(virtual_carrier, o.virtual_carrier, sizeof(struct st_carrier) * (size_t)Nc_max * (size_t)Nc_max) != 0)
			return "pilot.virtual_carrier";
	if((sequence == NULL) != (o.sequence == NULL)) return "pilot.sequence(null-mismatch)";
	if(sequence != NULL && nPilots > 0)
		if(memcmp(sequence, o.sequence, sizeof(std::complex<double>) * (size_t)nPilots) != 0)
			return "pilot.sequence";
	return NULL;
}

const char* cl_preamble_configurator::precook_deep_equal(const cl_preamble_configurator& o) const
{
	if(Nsymb != o.Nsymb) return "preamble.Nsymb";
	if(Nc != o.Nc)       return "preamble.Nc";
	if((sequence == NULL) != (o.sequence == NULL)) return "preamble.sequence(null-mismatch)";
	if(sequence != NULL && Nsymb > 0 && Nc > 0)
		if(memcmp(sequence, o.sequence, sizeof(std::complex<double>) * (size_t)Nsymb * (size_t)Nc) != 0)
			return "preamble.sequence";
	return NULL;
}

const char* cl_ofdm::precook_deep_equal(const cl_ofdm& o) const
{
	if(Nfft != o.Nfft)   return "ofdm.Nfft";
	if(Nc != o.Nc)       return "ofdm.Nc";
	if(Nsymb != o.Nsymb) return "ofdm.Nsymb";

	if((ofdm_frame == NULL) != (o.ofdm_frame == NULL)) return "ofdm.ofdm_frame(null-mismatch)";
	if(ofdm_frame != NULL && Nsymb > 0 && Nc > 0)
		if(memcmp(ofdm_frame, o.ofdm_frame, sizeof(struct st_carrier) * (size_t)Nsymb * (size_t)Nc) != 0)
			return "ofdm.ofdm_frame";

	{
		int pre_nsymb = preamble_configurator.Nsymb;
		if((ofdm_preamble == NULL) != (o.ofdm_preamble == NULL)) return "ofdm.ofdm_preamble(null-mismatch)";
		if(preamble_configurator.Nsymb != o.preamble_configurator.Nsymb)
			return "preamble.Nsymb";
		if(ofdm_preamble != NULL && pre_nsymb > 0 && Nc > 0)
			if(memcmp(ofdm_preamble, o.ofdm_preamble, sizeof(struct st_carrier) * (size_t)pre_nsymb * (size_t)Nc) != 0)
				return "ofdm.ofdm_preamble";
	}

	if((estimated_channel == NULL) != (o.estimated_channel == NULL)) return "ofdm.estimated_channel(null-mismatch)";
	if(estimated_channel != NULL && Nsymb > 0 && Nc > 0)
		if(memcmp(estimated_channel, o.estimated_channel, sizeof(struct st_channel_complex) * (size_t)Nsymb * (size_t)Nc) != 0)
			return "ofdm.estimated_channel";

	if((estimated_channel_without_amplitude_restoration == NULL) != (o.estimated_channel_without_amplitude_restoration == NULL))
		return "ofdm.est_channel_noamp(null-mismatch)";
	if(estimated_channel_without_amplitude_restoration != NULL && Nsymb > 0 && Nc > 0)
		if(memcmp(estimated_channel_without_amplitude_restoration, o.estimated_channel_without_amplitude_restoration,
		          sizeof(struct st_channel_complex) * (size_t)Nsymb * (size_t)Nc) != 0)
			return "ofdm.est_channel_noamp";

	if(mfsk_corr_template_len != o.mfsk_corr_template_len) return "ofdm.mfsk_corr_template_len";
	if((mfsk_corr_template == NULL) != (o.mfsk_corr_template == NULL)) return "ofdm.mfsk_corr_template(null-mismatch)";
	if(mfsk_corr_template != NULL && mfsk_corr_template_len > 0)
		if(memcmp(mfsk_corr_template, o.mfsk_corr_template, sizeof(std::complex<double>) * (size_t)mfsk_corr_template_len) != 0)
			return "ofdm.mfsk_corr_template";

	if(ofdm_corr_template_len != o.ofdm_corr_template_len) return "ofdm.ofdm_corr_template_len";
	if((ofdm_corr_template == NULL) != (o.ofdm_corr_template == NULL)) return "ofdm.ofdm_corr_template(null-mismatch)";
	if(ofdm_corr_template != NULL && ofdm_corr_template_len > 0)
		if(memcmp(ofdm_corr_template, o.ofdm_corr_template, sizeof(std::complex<double>) * (size_t)ofdm_corr_template_len) != 0)
			return "ofdm.ofdm_corr_template";

	// FFT tables.
	if((fft_twiddle == NULL) != (o.fft_twiddle == NULL)) return "ofdm.fft_twiddle(null-mismatch)";
	if(fft_twiddle != NULL && Nfft > 0)
		if(memcmp(fft_twiddle, o.fft_twiddle, sizeof(std::complex<double>) * (size_t)(Nfft / 2)) != 0)
			return "ofdm.fft_twiddle";
	if((fft_scratch == NULL) != (o.fft_scratch == NULL)) return "ofdm.fft_scratch(null-mismatch)";
	if(fft_scratch != NULL && Nfft > 0)
		if(memcmp(fft_scratch, o.fft_scratch, sizeof(std::complex<double>) * (size_t)Nfft) != 0)
			return "ofdm.fft_scratch";
	if((fft_bit_rev == NULL) != (o.fft_bit_rev == NULL)) return "ofdm.fft_bit_rev(null-mismatch)";
	if(fft_bit_rev != NULL && Nfft > 0)
		if(memcmp(fft_bit_rev, o.fft_bit_rev, sizeof(int) * (size_t)Nfft) != 0)
			return "ofdm.fft_bit_rev";

	// Nested sub-objects.
	const char* sub;
	if((sub = pilot_configurator.precook_deep_equal(o.pilot_configurator)) != NULL)       return sub;
	if((sub = preamble_configurator.precook_deep_equal(o.preamble_configurator)) != NULL) return sub;
	if(FIR_rx_data.precook_deep_equal(o.FIR_rx_data) != NULL)           return "ofdm.FIR_rx_data.filter_coefficients";
	if(FIR_rx_time_sync.precook_deep_equal(o.FIR_rx_time_sync) != NULL) return "ofdm.FIR_rx_time_sync.filter_coefficients";
	if(FIR_tx1.precook_deep_equal(o.FIR_tx1) != NULL)                   return "ofdm.FIR_tx1.filter_coefficients";
	if(FIR_tx2.precook_deep_equal(o.FIR_tx2) != NULL)                   return "ofdm.FIR_tx2.filter_coefficients";

	return NULL;
}

// Exercise independent run settings with initialized geometry at the copy boundary.
int precook_runscoped_selftest()
{
	const char* tag = "[TEST-PRECOOK-RUNSCOPED]";
	int failed = 0;
	cl_ofdm fresh;
	if(fresh.ls_nv_debug_enabled || fresh.ls_use_crosspilot_nv) failed++;
	cl_ofdm bundle;
	// Configure the nested grids before init, as configuration loading does.
	// Wideband preambles use four symbols; the high-density pilot grid uses Dy=3.
	bundle.preamble_configurator.Nsymb = 4;
	bundle.pilot_configurator.Dx = 1;
	bundle.pilot_configurator.Dy = 3;
	bundle.init(256, 50, 8, 36.0f / 256.0f);
	const bool fixture_ok = bundle.preamble_configurator.Nsymb == 4
		&& bundle.preamble_configurator.nPreamble > 0
		&& bundle.preamble_configurator.sequence != NULL
		&& bundle.pilot_configurator.Dx == 1 && bundle.pilot_configurator.Dy == 3
		&& bundle.pilot_configurator.nPilots > 0 && bundle.pilot_configurator.nData > 0
		&& bundle.pilot_configurator.sequence != NULL;
	printf("%s fixture preamble_symbols=%d preamble_bins=%d Dx=%d Dy=%d pilots=%d data=%d %s\n",
		tag, bundle.preamble_configurator.Nsymb, bundle.preamble_configurator.nPreamble,
		bundle.pilot_configurator.Dx, bundle.pilot_configurator.Dy,
		bundle.pilot_configurator.nPilots, bundle.pilot_configurator.nData,
		fixture_ok ? "PASS" : "INVALID");
	if(!fixture_ok) return 1;
	for(int source_flags = 0; source_flags < 4; ++source_flags)
	{
		bundle.ls_nv_debug_enabled = (source_flags & 1) != 0;
		bundle.ls_use_crosspilot_nv = (source_flags & 2) != 0;
		for(int destination_flags = 0; destination_flags < 4; ++destination_flags)
		{
			cl_ofdm live;
			live.ls_nv_debug_enabled = (destination_flags & 1) != 0;
			live.ls_use_crosspilot_nv = (destination_flags & 2) != 0;
			for(int repetition = 0; repetition < 2; ++repetition)
			{
				live.copy_from(bundle);
				const bool flags_ok = live.ls_nv_debug_enabled == ((destination_flags & 1) != 0)
					&& live.ls_use_crosspilot_nv == ((destination_flags & 2) != 0);
				const bool source_ok = bundle.ls_nv_debug_enabled == ((source_flags & 1) != 0)
					&& bundle.ls_use_crosspilot_nv == ((source_flags & 2) != 0);
				const bool geometry_ok = live.Nfft == bundle.Nfft && live.Nc == bundle.Nc
					&& live.Nsymb == bundle.Nsymb && live.gi == bundle.gi
					&& live.precook_deep_equal(bundle) == NULL
					&& live.preamble_configurator.Nsymb == 4
					&& live.preamble_configurator.nPreamble == bundle.preamble_configurator.nPreamble
					&& live.pilot_configurator.Dx == 1 && live.pilot_configurator.Dy == 3
					&& live.pilot_configurator.nPilots == bundle.pilot_configurator.nPilots
					&& live.pilot_configurator.nData == bundle.pilot_configurator.nData
					&& live.ofdm_frame != bundle.ofdm_frame
					&& live.pilot_configurator.carrier == live.ofdm_frame
					&& live.preamble_configurator.carrier == live.ofdm_preamble;
				const bool ok = flags_ok && source_ok && geometry_ok;
				printf("%s source=%d destination=%d repetition=%d flags=%d source_unchanged=%d geometry=%d %s\n",
					tag, source_flags, destination_flags, repetition, (int)flags_ok,
					(int)source_ok, (int)geometry_ok, ok ? "PASS" : "FAIL");
				if(!ok) failed++;
			}
			// Self-copy must leave both the run settings and owned geometry intact.
			live.ls_nv_debug_enabled = (destination_flags & 1) != 0;
			live.ls_use_crosspilot_nv = (destination_flags & 2) != 0;
			const st_carrier* frame = live.ofdm_frame;
			live.copy_from(live);
			const bool self_ok = live.ofdm_frame == frame
				&& live.ls_nv_debug_enabled == ((destination_flags & 1) != 0)
				&& live.ls_use_crosspilot_nv == ((destination_flags & 2) != 0);
			if(!self_ok) failed++;
		}
	}
	printf("%s %s (failed=%d)\n", tag, failed == 0 ? "ALL PASS" : "FAILURES", failed);
	fflush(stdout);
	return failed;
}

