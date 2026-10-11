/* Deterministic, in-process two-peer transition harness (test infrastructure). */
#ifndef INC_LP_TRANSITION_HARNESS_H_
#define INC_LP_TRANSITION_HARNESS_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "datalink_layer/protocol_contract_kernel.h"

namespace mercury {

enum class LpDeliveryAction : std::uint8_t {
    Deliver = 0,
    Drop,
    Delay,
    ReorderHold,
    ReorderRelease
};

enum class LpHarnessEvent : std::uint8_t {
    Connect = 0,
    CmdKeydownStart,
    DataFrameTx,
    DataFrameDelivered,
    CmdAudioEgressEnd,
    ForwardBurstEndProven,
    AckSlotOpen,
    ResponderKeyed,
    AckFrameTx,
    AckFrameDelivered,
    AckDecoded,
    AckSlotClose,
    NextBatchReady
};

struct LpScriptStep {
    std::uint64_t at_sample;
    LpHarnessEvent event;
    LpDeliveryAction action;
    std::uint64_t action_samples;
    std::uint32_t audio_samples;
    protocol::Generation generation;

    LpScriptStep(std::uint64_t at, LpHarnessEvent ev,
                 LpDeliveryAction act = LpDeliveryAction::Deliver,
                 std::uint64_t action_delay = 0,
                 std::uint32_t audio_count = 0,
                 protocol::Generation gen = protocol::Generation::from_wire(0))
        : at_sample(at), event(ev), action(act), action_samples(action_delay),
          audio_samples(audio_count), generation(gen) {}
};

struct LpHarnessResult {
    bool ok;
    std::string trace;
    std::size_t delivered;
    std::size_t dropped;
    std::size_t delayed;
    std::size_t reordered;

    LpHarnessResult()
        : ok(false), delivered(0), dropped(0), delayed(0), reordered(0) {}
};

class LpTransitionHarness {
public:
    explicit LpTransitionHarness(std::uint64_t seed);
    ~LpTransitionHarness();

    LpHarnessResult run(const std::vector<LpScriptStep>& script);
    static std::vector<LpScriptStep> clean_exchange_script();
    static std::vector<LpScriptStep> action_coverage_script();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

/* Standalone entry points used by --test and the focused CLI gate. */
int run_lp_transition_harness_tests();
int run_lp_transition_demo(std::string* trace_out);

}  // namespace mercury

#endif  // INC_LP_TRANSITION_HARNESS_H_
