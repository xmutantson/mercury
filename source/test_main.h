#ifndef MERCURY_TEST_MAIN_H
#define MERCURY_TEST_MAIN_H

int test_shutdown_atomic();
int run_pas_selftest();
int run_cfg17_selftest();
int run_tinterp_seed_selftest();
int run_tinterp_receive_path_selftest();
int run_acq_bounds_selftest();
int run_subpeak_rescue_selftest();
int run_chase_ab_selftest();
int run_frame0_vehicle_selftest();
int run_frame0_mf_selftest();
int run_frame0_replay_selftest(const char* path);
int run_crc_escape_selftest();
int run_chase_selftest();
void arm_test_watchdog();
int run_mini0_carried_timing_test();
int run_mini1_carried_timing_test();

#endif
