/*
 * Winlink dictionary priming + version-lock regression suite entry point.
 * Wired into main.cc --test and the standalone --test-winlink-dict.
 */
#ifndef TEST_WINLINK_DICT_H
#define TEST_WINLINK_DICT_H

// Runs the dict priming / version-lock / no-regression battery against the
// production cl_compressor. Returns 0 on all-pass, 1 on any failure.
int run_winlink_dict_tests();

#endif
