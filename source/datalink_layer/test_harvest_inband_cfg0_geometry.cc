// Fresh-decoder CONFIG_0 geometry regression.
//
// The live primary is startup-patched to a 36/256 OFDM guard interval.  Exercise
// the production scoped down-decoder builder without patching the child decoder,
// then require its live CONFIG_0 geometry to match the primary.  Bare monitor
// creates the child from the constructor's 54/256 default (Nofdm=310) and fails;
// the inheritance fix creates Nofdm=292 and passes.
//
// CLI: --test-harvest-inband-cfg0-geometry

#include "datalink_layer/arq.h"
#include "common/common_defines.h"
#include <cstdio>

int cl_arq_controller::test_harvest_inband_cfg0_geometry()
{
	const char* tag = "[TEST-HARVEST-INBAND-CFG0-GEOMETRY]";
	const int production_ngi = 36;
	const int expected_nofdm = 256 + production_ngi;

	cl_telecom_system primary;
	cl_arq_controller rx;
	primary.operation_mode = ARQ_MODE;
	primary.narrowband_enabled = NO;
	primary.default_configurations_telecom_system.ofdm_gi =
		(float)production_ngi / 256.0f;
	rx.telecom_system = &primary;
	rx.narrowband_enabled = NO;
	rx.role = RESPONDER;
	rx.robust_enabled = YES;
	rx.sack_v2_enabled = true;
	rx.inband_rate_enabled = 1;
	rx.load_configuration(ROBUST_0, FULL, NO);
	rx.link_status = CONNECTED;
	rx.connection_status = RECEIVING;
	rx.passive_monitor = false;

	const int cfg0_idx = config_ladder_index(CONFIG_0);
	const int built = rx.inband_ensure_down_decoders(cfg0_idx, cfg0_idx);
	cl_telecom_system* cfg0 = NULL;
	for(int i = 0; i <= INBAND_DOWN_D_MAX; ++i)
	{
		if(rx.inband_down_decoders[i] != NULL &&
		   rx.inband_down_decoder_cfg[i] == CONFIG_0)
		{
			cfg0 = rx.inband_down_decoders[i];
			break;
		}
	}

	int got_nofdm = cfg0 ? cfg0->data_container.Nofdm : -1;
	int got_ngi = cfg0
		? (int)((double)cfg0->ofdm.gi * (double)cfg0->ofdm.Nfft + 0.5)
		: -1;
	bool pass = built >= 1 && cfg0 != NULL &&
		got_nofdm == expected_nofdm && got_ngi == production_ngi;

	printf("%s %s: built=%d child Nofdm=%d Ngi=%d; required Nofdm=%d Ngi=%d\n",
		tag, pass ? "PASS-AFTER" : "FAIL-BEFORE", built,
		got_nofdm, got_ngi, expected_nofdm, production_ngi);
	fflush(stdout);
	rx.inband_free_down_decoders();
	return pass ? 0 : 1;
}
