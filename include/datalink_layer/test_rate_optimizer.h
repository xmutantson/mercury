#ifndef RATE_OPTIMIZER_TEST_ACCESS_H_
#define RATE_OPTIMIZER_TEST_ACCESS_H_

#include "datalink_layer/rate_optimizer.h"

inline int cl_rate_optimizer::context_generation_for_test() const { return context_generation; }

inline double cl_rate_optimizer::atomic_application_transport_gain_for_test() const { return atomic_application_transport_gain; }

inline bool cl_rate_optimizer::atomic_application_transport_gain_valid_for_test() const { return atomic_application_transport_gain_valid; }

inline void cl_rate_optimizer::set_mode_for_test(e_gearshift_v2_mode m) { mode = m; }

inline void cl_rate_optimizer::set_policy_for_test(const st_rate_policy& p) { policy = p; }

inline int cl_rate_optimizer::probe_application_samples_for_test() const { return probe_application_samples; }

inline int cl_rate_optimizer::probe_outcome_samples_for_test() const { return probe_outcome_samples; }

inline double cl_rate_optimizer::probe_mean_bps_for_test() const {
        return probe_application_samples > 0
            ? probe_application_bps_sum / (double)probe_application_samples : 0.0;
    }

inline double cl_rate_optimizer::probe_channel_ms_for_test() const { return probe_channel_ms; }

inline double cl_rate_optimizer::probe_max_channel_ms_for_test() const { return probe_max_channel_ms; }

inline bool cl_rate_optimizer::switch_inflight_for_test() const { return switch_inflight; }

inline int cl_rate_optimizer::switch_from_cfg_for_test() const { return switch_from_cfg; }

inline int cl_rate_optimizer::switch_to_cfg_for_test() const { return switch_to_cfg; }

inline e_gearshift_v2_action cl_rate_optimizer::switch_action_for_test() const { return switch_action; }

inline bool cl_rate_optimizer::probe_blocked_for_test(int cfg, const st_rate_observation& obs) const {
        return probe_target_blocked(cfg, obs);
    }

inline double cl_rate_optimizer::context_volatility_for_test() const { return context_volatility; }

inline st_rate_prediction cl_rate_optimizer::predict_for_test(
        int cfg, const st_rate_observation& obs) const
{
    return predict_config(cfg, obs);
}

#endif // RATE_OPTIMIZER_TEST_ACCESS_H_
