#ifndef INC_LP_TRANSITION_HARNESS_TEST_ENTRY_H_
#define INC_LP_TRANSITION_HARNESS_TEST_ENTRY_H_

#include <string>

namespace mercury {
int run_lp_transition_harness_tests();
int run_lp_transition_demo(std::string* trace_out);
}  // namespace mercury

#endif
