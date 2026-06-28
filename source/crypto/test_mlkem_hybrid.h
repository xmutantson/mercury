/*
 * ML-KEM-768 hybrid KEX regression suite — entry point.
 * Wired via mercury.exe --test. See test_mlkem_hybrid.cc for the cases.
 */
#ifndef MERCURY_TEST_MLKEM_HYBRID_H
#define MERCURY_TEST_MLKEM_HYBRID_H

// Runs the hybrid ML-KEM + X25519 KEX regression suite. Returns the number of
// failed cases (0 = all pass). Self-contained: no IONOS / RF / telecom_system.
int run_mlkem_hybrid_tests();

#endif
