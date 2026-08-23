/*
 * Deterministic two-peer transition harness.
 *
 * The fixture owns two production controllers and invokes their existing
 * link-phase producer entry points. It is test infrastructure only: no live
 * controller path calls this file.
 */
#include "datalink_layer/lp_transition_harness.h"

#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <map>
#include <sstream>
#include <utility>

#include "common/sim_clock.h"
#include "datalink_layer/arq.h"
#include "datalink_layer/datalink_defines.h"

namespace mercury {

class LpTransitionHarness::Impl {
public:
    explicit Impl(std::uint64_t seed) : seed_(seed) {}
    std::uint64_t seed_;
};

namespace {

struct QueuedStep {
    LpScriptStep step;
    std::uint64_t serial;
};

struct DeadlineSet {
    std::map<std::string, std::uint64_t> values;
};

static const char* event_name(LpHarnessEvent event)
{
    switch(event) {
    case LpHarnessEvent::Connect: return "CONNECT";
    case LpHarnessEvent::CmdKeydownStart: return "CMD_KEYDOWN_START";
    case LpHarnessEvent::DataFrameTx: return "DATA_FRAME_TX";
    case LpHarnessEvent::DataFrameDelivered: return "DATA_FRAME_DELIVERED";
    case LpHarnessEvent::CmdAudioEgressEnd: return "CMD_AUDIO_EGRESS_END";
    case LpHarnessEvent::ForwardBurstEndProven: return "FORWARD_BURST_END_PROVEN";
    case LpHarnessEvent::AckSlotOpen: return "ACK_SLOT_OPEN";
    case LpHarnessEvent::ResponderKeyed: return "RSP_KEYDOWN_START";
    case LpHarnessEvent::AckFrameTx: return "ACK_FRAME_TX";
    case LpHarnessEvent::AckFrameDelivered: return "ACK_FRAME_DELIVERED";
    case LpHarnessEvent::AckDecoded: return "ACK_DECODED";
    case LpHarnessEvent::AckSlotClose: return "ACK_SLOT_CLOSE";
    case LpHarnessEvent::NextBatchReady: return "NEXT_BATCH_READY";
    }
    return "UNKNOWN";
}

static const char* action_name(LpDeliveryAction action)
{
    switch(action) {
    case LpDeliveryAction::Deliver: return "deliver";
    case LpDeliveryAction::Drop: return "drop";
    case LpDeliveryAction::Delay: return "delay";
    case LpDeliveryAction::ReorderHold: return "reorder_hold";
    case LpDeliveryAction::ReorderRelease: return "reorder_release";
    }
    return "unknown";
}

static protocol::LinkPhaseEvent contract_event(LpHarnessEvent event)
{
    switch(event) {
    case LpHarnessEvent::Connect: return protocol::LinkPhaseEvent::ConnectAccepted;
    case LpHarnessEvent::CmdKeydownStart: return protocol::LinkPhaseEvent::CmdKeydownStart;
    case LpHarnessEvent::DataFrameTx: return protocol::LinkPhaseEvent::DataFrameEmitted;
    case LpHarnessEvent::DataFrameDelivered: return protocol::LinkPhaseEvent::DataFrameDelivered;
    case LpHarnessEvent::CmdAudioEgressEnd: return protocol::LinkPhaseEvent::CmdAudioEgressEnd;
    case LpHarnessEvent::ForwardBurstEndProven: return protocol::LinkPhaseEvent::ForwardBurstEndProven;
    case LpHarnessEvent::AckSlotOpen: return protocol::LinkPhaseEvent::AckSlotOpen;
    case LpHarnessEvent::ResponderKeyed: return protocol::LinkPhaseEvent::ResponderKeyed;
    case LpHarnessEvent::AckFrameTx: return protocol::LinkPhaseEvent::AckFrameEmitted;
    case LpHarnessEvent::AckFrameDelivered: return protocol::LinkPhaseEvent::AckFrameDelivered;
    case LpHarnessEvent::AckDecoded: return protocol::LinkPhaseEvent::AckDecoded;
    case LpHarnessEvent::AckSlotClose: return protocol::LinkPhaseEvent::AckSlotClose;
    case LpHarnessEvent::NextBatchReady: return protocol::LinkPhaseEvent::NextBatchReady;
    }
    return protocol::LinkPhaseEvent::Invalid;
}

static const char* owner_name(int owner)
{
    switch(owner) {
    case 0: return "NONE";
    case 1: return "CMD_KEYED";
    case 2: return "TURNAROUND";
    case 3: return "RSP_KEYED";
    }
    return "UNKNOWN";
}

static const char* connection_name(int state)
{
    switch(state) {
    case IDLE: return "IDLE";
    case TRANSMITTING_DATA: return "TRANSMITTING_DATA";
    case RECEIVING: return "RECEIVING";
    case RECEIVING_ACKS_DATA: return "RECEIVING_ACKS_DATA";
    case ACKNOWLEDGING_DATA: return "ACKNOWLEDGING_DATA";
    default: return "OTHER";
    }
}

static std::string deadline_text(const DeadlineSet& deadlines)
{
    if(deadlines.values.empty()) return "-";
    std::ostringstream out;
    bool first = true;
    for(const auto& entry : deadlines.values) {
        if(!first) out << ',';
        first = false;
        out << entry.first << '@' << entry.second;
    }
    return out.str();
}

static bool queue_less(const QueuedStep& lhs, const QueuedStep& rhs)
{
    if(lhs.step.at_sample != rhs.step.at_sample)
        return lhs.step.at_sample < rhs.step.at_sample;
    return lhs.serial < rhs.serial;
}

static bool is_frame_delivery(LpHarnessEvent event)
{
    return event == LpHarnessEvent::DataFrameDelivered
        || event == LpHarnessEvent::AckFrameDelivered;
}

}  // namespace

LpTransitionHarness::LpTransitionHarness(std::uint64_t seed)
    : impl_(new Impl(seed)) {}

LpTransitionHarness::~LpTransitionHarness() = default;

std::vector<LpScriptStep> LpTransitionHarness::clean_exchange_script()
{
    const protocol::Generation g = protocol::Generation::from_wire(7);
    return {
        LpScriptStep(0,    LpHarnessEvent::Connect,              LpDeliveryAction::Deliver, 0,   0, g),
        LpScriptStep(480,  LpHarnessEvent::CmdKeydownStart,      LpDeliveryAction::Deliver, 0,   0, g),
        LpScriptStep(600,  LpHarnessEvent::DataFrameTx,          LpDeliveryAction::Deliver, 0, 480, g),
        LpScriptStep(720,  LpHarnessEvent::DataFrameDelivered,   LpDeliveryAction::Deliver, 0, 480, g),
        LpScriptStep(1440, LpHarnessEvent::CmdAudioEgressEnd,    LpDeliveryAction::Deliver, 0, 960, g),
        LpScriptStep(1680, LpHarnessEvent::ForwardBurstEndProven,LpDeliveryAction::Deliver, 0,   0, g),
        LpScriptStep(2880, LpHarnessEvent::AckSlotOpen,          LpDeliveryAction::Deliver, 0,   0, g),
        LpScriptStep(3000, LpHarnessEvent::ResponderKeyed,       LpDeliveryAction::Deliver, 0,   0, g),
        LpScriptStep(3120, LpHarnessEvent::AckFrameTx,           LpDeliveryAction::Deliver, 0, 240, g),
        LpScriptStep(3360, LpHarnessEvent::AckFrameDelivered,    LpDeliveryAction::Deliver, 0, 240, g),
        LpScriptStep(3360, LpHarnessEvent::AckDecoded,           LpDeliveryAction::Deliver, 0,   0, g),
        LpScriptStep(4800, LpHarnessEvent::AckSlotClose,         LpDeliveryAction::Deliver, 0,   0, g),
        LpScriptStep(5040, LpHarnessEvent::NextBatchReady,       LpDeliveryAction::Deliver, 0,   0, g)
    };
}

std::vector<LpScriptStep> LpTransitionHarness::action_coverage_script()
{
    std::vector<LpScriptStep> script = clean_exchange_script();
    const protocol::Generation g = protocol::Generation::from_wire(7);
    script.emplace_back(800, LpHarnessEvent::DataFrameTx,
                        LpDeliveryAction::Drop, 0, 120, g);
    script.emplace_back(820, LpHarnessEvent::DataFrameTx,
                        LpDeliveryAction::Delay, 100, 120, g);
    script.emplace_back(840, LpHarnessEvent::DataFrameTx,
                        LpDeliveryAction::ReorderHold, 0, 120, g);
    script.emplace_back(860, LpHarnessEvent::DataFrameTx,
                        LpDeliveryAction::ReorderRelease, 0, 120, g);
    return script;
}

LpHarnessResult LpTransitionHarness::run(const std::vector<LpScriptStep>& script)
{
    LpHarnessResult result;
    const int prior_sim_enabled = sim_clock_enabled();
    sim_clock_set_enabled(1);

    /* Construct only after virtual time is enabled so every cl_timer starts in
     * the sample domain. These are the same controller type production owns. */
    std::unique_ptr<cl_arq_controller> commander(new cl_arq_controller());
    std::unique_ptr<cl_arq_controller> responder(new cl_arq_controller());
    const std::uint64_t origin = sim_clock_now_samples();

    commander->telecom_system = NULL;
    responder->telecom_system = NULL;
    commander->message_transmission_time_ms = 10;
    responder->message_transmission_time_ms = 10;
    commander->data_batch_size = 2;
    responder->data_batch_size = 2;
    commander->receiving_timeout = 50;
    responder->receiving_timeout = 50;
    commander->sack_v2_enabled = true;
    responder->sack_v2_enabled = true;
    commander->role = COMMANDER;
    responder->role = RESPONDER;
    commander->lp_config_gen = 1;
    responder->lp_config_gen = 1;
    commander->lp_reset();
    responder->lp_reset();

    const protocol::SessionEpoch session(1);
    const protocol::ConfigEpoch config(1);
    std::uint64_t ordinal = 0;
    DeadlineSet cmd_deadlines;
    DeadlineSet rsp_deadlines;
    std::vector<QueuedStep> queue;
    std::vector<QueuedStep> held;
    std::uint64_t serial = 0;
    for(const auto& step : script) queue.push_back({step, serial++});

    bool saw_connect = false;
    bool saw_slot_open = false;
    bool saw_ack = false;
    bool saw_slot_close = false;
    bool invariants_ok = true;
    std::ostringstream trace;
    trace << "LP_TRANSITION_TRACE v=1 seed=" << impl_->seed_
          << " sample_rate=" << SIM_CLOCK_SAMPLE_RATE_HZ << '\n';

    auto snapshot = [&](const char* peer, const cl_arq_controller& ctl,
                        const DeadlineSet& deadlines, const char* name,
                        const protocol::LinkPhaseEventToken& token) {
        trace << "sample=" << std::setw(6) << std::setfill('0')
              << (sim_clock_now_samples() - origin) << std::setfill(' ')
              << " event=" << name
              << " peer=" << peer
              << " state=" << connection_name(ctl.connection_status)
              << " session=" << token.session.value()
              << " config=" << token.config.value()
              << " generation=" << static_cast<unsigned>(token.generation.wire_value())
              << " epoch=" << ctl.lp_state.epoch
              << " owner=" << owner_name(static_cast<int>(ctl.lp_state.owner))
              << " token=" << ctl.lp_state.keydown_end_token
              << " deadlines=" << deadline_text(deadlines) << '\n';
    };

    while(!queue.empty()) {
        std::stable_sort(queue.begin(), queue.end(), queue_less);
        QueuedStep item = queue.front();
        queue.erase(queue.begin());

        if(item.step.action == LpDeliveryAction::Delay) {
            trace << "action=" << action_name(item.step.action)
                  << " event=" << event_name(item.step.event)
                  << " from=" << item.step.at_sample
                  << " to=" << (item.step.at_sample + item.step.action_samples) << '\n';
            item.step.at_sample += item.step.action_samples;
            item.step.action = LpDeliveryAction::Deliver;
            item.serial = serial++;
            queue.push_back(item);
            result.delayed++;
            continue;
        }
        if(item.step.action == LpDeliveryAction::ReorderHold) {
            trace << "action=" << action_name(item.step.action)
                  << " event=" << event_name(item.step.event)
                  << " at=" << item.step.at_sample << '\n';
            item.step.action = LpDeliveryAction::Deliver;
            held.push_back(item);
            continue;
        }
        if(item.step.action == LpDeliveryAction::ReorderRelease) {
            trace << "action=" << action_name(item.step.action)
                  << " event=" << event_name(item.step.event)
                  << " at=" << item.step.at_sample
                  << " held=" << held.size() << '\n';
            for(auto& pending : held) {
                pending.step.at_sample = item.step.at_sample + 1;
                pending.serial = serial++;
                queue.push_back(pending);
                result.reordered++;
            }
            held.clear();
            item.step.action = LpDeliveryAction::Deliver;
        }

        const std::uint64_t now_rel = sim_clock_now_samples() - origin;
        if(item.step.at_sample < now_rel) {
            invariants_ok = false;
        } else {
            sim_clock_add_samples(item.step.at_sample - now_rel);
        }

        const char* name = event_name(item.step.event);
        if(item.step.action == LpDeliveryAction::Drop) {
            trace << "action=" << action_name(item.step.action)
                  << " event=" << name << " at=" << item.step.at_sample
                  << " audio_samples=" << item.step.audio_samples << '\n';
            result.dropped++;
        } else {
            if(is_frame_delivery(item.step.event)) result.delivered++;
            switch(item.step.event) {
            case LpHarnessEvent::Connect:
                commander->link_status = CONNECTED;
                responder->link_status = CONNECTED;
                commander->connection_status = TRANSMITTING_DATA;
                responder->connection_status = RECEIVING;
                saw_connect = true;
                break;
            case LpHarnessEvent::CmdKeydownStart:
                commander->linkphase_last_kd_frames = 2;
                commander->linkphase_last_kd_force_full = false;
                commander->lp_note_keydown_start(item.step.generation.wire_value());
                cmd_deadlines.values["FORWARD_BURST_END"] = item.step.at_sample + 960;
                break;
            case LpHarnessEvent::DataFrameTx:
                break;
            case LpHarnessEvent::DataFrameDelivered:
                responder->lp_note_rx_frame0(item.step.generation.wire_value(), 2);
                responder->connection_status = RECEIVING;
                rsp_deadlines.values["FORWARD_BURST_END_BOUND"] =
                    static_cast<std::uint64_t>(responder->lp_state.owner_keydown_end_ms) * 48ULL;
                break;
            case LpHarnessEvent::CmdAudioEgressEnd:
                commander->lp_note_keydown_end(item.step.generation.wire_value(),
                    item.step.audio_samples ? static_cast<int>(item.step.audio_samples) : 960,
                    2, false);
                commander->connection_status = RECEIVING_ACKS_DATA;
                cmd_deadlines.values.erase("FORWARD_BURST_END");
                cmd_deadlines.values["ACK_SLOT_OPEN"] =
                    static_cast<std::uint64_t>(commander->lp_state.next_listen_open_ms) * 48ULL;
                cmd_deadlines.values["ACK_SLOT_CLOSE"] = 4800;
                break;
            case LpHarnessEvent::ForwardBurstEndProven:
                rsp_deadlines.values.erase("FORWARD_BURST_END_BOUND");
                break;
            case LpHarnessEvent::AckSlotOpen:
                if(!commander->lp_ack_window_active(NULL)
                   || item.step.at_sample < static_cast<std::uint64_t>(
                        commander->lp_state.next_listen_open_ms) * 48ULL)
                    invariants_ok = false;
                cmd_deadlines.values.erase("ACK_SLOT_OPEN");
                saw_slot_open = true;
                break;
            case LpHarnessEvent::ResponderKeyed:
                if(!saw_slot_open) invariants_ok = false;
                responder->lp_note_rsp_key();
                responder->connection_status = ACKNOWLEDGING_DATA;
                break;
            case LpHarnessEvent::AckFrameTx:
            case LpHarnessEvent::AckFrameDelivered:
                break;
            case LpHarnessEvent::AckDecoded:
                if(!saw_slot_open) invariants_ok = false;
                commander->lp_note_ack_decoded();
                commander->connection_status = TRANSMITTING_DATA;
                responder->connection_status = RECEIVING;
                cmd_deadlines.values.erase("ACK_SLOT_CLOSE");
                saw_ack = true;
                break;
            case LpHarnessEvent::AckSlotClose:
                if(!saw_slot_open || !saw_ack) invariants_ok = false;
                cmd_deadlines.values.erase("ACK_SLOT_CLOSE");
                saw_slot_close = true;
                break;
            case LpHarnessEvent::NextBatchReady:
                if(!saw_slot_close) invariants_ok = false;
                commander->connection_status = TRANSMITTING_DATA;
                responder->connection_status = RECEIVING;
                break;
            }
        }

        ++ordinal;
        protocol::LinkPhaseEventToken token;
        token.session = session;
        token.config = config;
        token.generation = item.step.generation;
        token.ordinal = ordinal;
        token.event = contract_event(item.step.event);
        if(!token.valid()) invariants_ok = false;

        snapshot("CMD", *commander, cmd_deadlines, name, token);
        snapshot("RSP", *responder, rsp_deadlines, name, token);
    }

    if(!held.empty()) invariants_ok = false;
    result.ok = invariants_ok && saw_connect && saw_slot_open && saw_ack && saw_slot_close;
    trace << "RESULT ok=" << (result.ok ? 1 : 0)
          << " delivered=" << result.delivered
          << " dropped=" << result.dropped
          << " delayed=" << result.delayed
          << " reordered=" << result.reordered << '\n';
    result.trace = trace.str();
    sim_clock_set_enabled(prior_sim_enabled);
    return result;
}

int run_lp_transition_demo(std::string* trace_out)
{
    LpTransitionHarness harness(0xC10CULL);
    const LpHarnessResult result = harness.run(LpTransitionHarness::clean_exchange_script());
    if(trace_out) *trace_out = result.trace;
    return result.ok && result.delivered == 2 ? 0 : 1;
}

int run_lp_transition_harness_tests()
{
    int failures = 0;
    {
        using namespace protocol;
        const Generation wrapped = Generation::from_wire(255).next();
        AckReportIdentity report;
        report.session = SessionEpoch(3);
        report.config = ConfigEpoch(4);
        report.target_generation = Generation::from_wire(0);
        report.bitmap_generation = Generation::from_wire(0);
        report.first_slot = 0;
        report.slot_count = 2;
        report.kind = AckReportKind::PerBatch;
        report.owner = AckReportOwner::LiveSlots;
        const bool kernel_ok = !SessionEpoch().valid() && !ConfigEpoch().valid()
            && wrapped.valid() && wrapped.wire_value() == 0 && report.valid();
        report.bitmap_generation = Generation::from_wire(1);
        const bool mismatch_rejected = !report.valid();
        if(!kernel_ok || !mismatch_rejected) failures++;
        std::printf("[TEST-LP-KERNEL] %s typed-invalid guards, generation 255->0 wrap, "
                    "bitmap-target identity\n",
            (kernel_ok && mismatch_rejected) ? "PASS" : "FAIL");
    }
    const std::uint64_t seeds[] = {1, 0x5eedULL, 0xc0ffeeULL};
    std::size_t comparisons = 0;
    for(std::uint64_t seed : seeds) {
        std::string reference;
        for(int repetition = 0; repetition < 3; ++repetition) {
            LpTransitionHarness harness(seed);
            const LpHarnessResult result = harness.run(
                LpTransitionHarness::action_coverage_script());
            if(!result.ok || result.delivered != 2 || result.dropped != 1
               || result.delayed != 1 || result.reordered != 1) {
                std::printf("[TEST-LP-HARNESS] FAIL seed=%llu repetition=%d "
                            "ok=%d delivered=%zu dropped=%zu delayed=%zu reordered=%zu\n",
                    static_cast<unsigned long long>(seed), repetition + 1,
                    result.ok ? 1 : 0, result.delivered, result.dropped,
                    result.delayed, result.reordered);
                failures++;
            }
            if(repetition == 0) reference = result.trace;
            else {
                comparisons++;
                if(result.trace != reference) {
                    std::printf("[TEST-LP-HARNESS] FAIL seed=%llu repetition=%d "
                                "trace differs byte-for-byte\n",
                        static_cast<unsigned long long>(seed), repetition + 1);
                    failures++;
                }
            }
            std::printf("[TEST-LP-HARNESS] %s seed=%llu repetition=%d/3 "
                        "trace_bytes=%zu\n",
                (result.ok && (repetition == 0 || result.trace == reference)) ? "PASS" : "FAIL",
                static_cast<unsigned long long>(seed), repetition + 1,
                result.trace.size());
        }
    }

    std::string demo_trace;
    const int demo_rc = run_lp_transition_demo(&demo_trace);
    const bool has_open = demo_trace.find("event=ACK_SLOT_OPEN peer=CMD") != std::string::npos;
    const bool has_close = demo_trace.find("event=ACK_SLOT_CLOSE peer=CMD") != std::string::npos;
    if(demo_rc != 0 || !has_open || !has_close) failures++;
    std::printf("[TEST-LP-HARNESS] %s demo clean exchange; reverse-ACK open=%d close=%d "
                "trace_bytes=%zu\n", failures ? "FAIL" : "PASS",
                has_open ? 1 : 0, has_close ? 1 : 0, demo_trace.size());
    std::printf("[TEST-LP-HARNESS] %s determinism_runs=9/9 byte_comparisons=%zu "
                "failures=%d\n", failures ? "FAIL" : "ALL PASS", comparisons, failures);
    return failures;
}

}  // namespace mercury
