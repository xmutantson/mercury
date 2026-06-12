#ifndef INC_TEST_SE_RECLAIM_H_
#define INC_TEST_SE_RECLAIM_H_

// SE-reclaim grid-selector regression suite (Stages 1-2; Stage 5 transition test
// lives on cl_arq_controller). Paired with fact-documents/data-flow-se-reclaim.md.
// Wired via mercury.exe --test-se-reclaim. Fast + deterministic, no IONOS/RF.
// Returns the number of failed tests (0 = all pass).
int run_se_reclaim_tests();

#endif // INC_TEST_SE_RECLAIM_H_
