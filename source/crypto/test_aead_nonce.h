/*
 * AEAD bsi-bound nonce regression suite — entry point.
 * Wired via mercury.exe --test. See test_aead_nonce.cc for the cases.
 */
#ifndef MERCURY_TEST_AEAD_NONCE_H
#define MERCURY_TEST_AEAD_NONCE_H

// Runs the AEAD nonce regression suite. Returns the number of failed cases
// (0 = all pass). Self-contained: no IONOS / RF / telecom_system.
int run_aead_nonce_tests();

#endif
