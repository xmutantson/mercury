/*
 * Mercury: A configurable open-source software-defined modem.
 *
 * sim_clock_tests.h — unit tests for the -x sim virtual time source.
 * Returns the number of failed tests (0 = all pass). Wired via
 * `mercury.exe --test` (see source/main.cc). See
 * fact-documents/sim-arq-channel.md.
 */
#ifndef INC_SIM_CLOCK_TESTS_H_
#define INC_SIM_CLOCK_TESTS_H_

int run_sim_clock_tests();

#endif // INC_SIM_CLOCK_TESTS_H_
