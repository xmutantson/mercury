/*
 * Mercury: additive protocol identity and event vocabulary.
 *
 * Most vocabulary remains additive. The config-transition observer has a
 * default-off live integration; no wire field or timer behavior changes while
 * that migration flag is disabled.
 */
#ifndef INC_PROTOCOL_CONTRACT_KERNEL_H_
#define INC_PROTOCOL_CONTRACT_KERNEL_H_

#include <cstdint>

namespace mercury {
namespace protocol {

/*
 * SessionEpoch
 * Owner: the connection/session lifecycle.
 * Validity: zero is invalid; a fresh accepted session allocates a nonzero,
 * monotonically increasing local value. It is never compared across peers
 * until a transport explicitly binds both sides to the same session identity.
 */
class SessionEpoch {
public:
    constexpr SessionEpoch() : value_(0) {}
    explicit constexpr SessionEpoch(std::uint64_t value) : value_(value) {}
    constexpr bool valid() const { return value_ != 0; }
    constexpr std::uint64_t value() const { return value_; }
private:
    std::uint64_t value_;
};

/*
 * ConfigEpoch
 * Owner: the configuration publication/activation transition.
 * Validity: zero is invalid in this vocabulary; an epoch changes before an
 * old-geometry deadline or report may affect a newly activated geometry.
 */
class ConfigEpoch {
public:
    constexpr ConfigEpoch() : value_(0) {}
    explicit constexpr ConfigEpoch(std::uint32_t value) : value_(value) {}
    constexpr bool valid() const { return value_ != 0; }
    constexpr std::uint32_t value() const { return value_; }
private:
    std::uint32_t value_;
};

/*
 * Default-off live config-transition observer.  The link-phase primitive
 * stores its config generation in the high 24 bits of a uint32_t epoch, so a
 * generation outside that range cannot be represented without aliasing an
 * older geometry.  A rejected observation permanently invalidates this
 * contract instance; its caller must stop the transition and clear any
 * timeline evidence derived from it.
 */
class LinkPhaseConfigContract {
public:
    LinkPhaseConfigContract()
        : valid_(true), last_observed_ns_(0), has_observation_(false) {}

    bool observe_transition(ConfigEpoch next, std::uint64_t observed_ns) {
        static constexpr std::uint32_t kMaxPackedConfigEpoch = 0x00ffffffu;
        const bool accepted = valid_ && next.valid()
            && next.value() <= kMaxPackedConfigEpoch
            && (!last_config_.valid()
                || next.value() == last_config_.value() + 1u)
            && (!has_observation_ || observed_ns >= last_observed_ns_);
        if(!accepted) {
            valid_ = false;
            return false;
        }
        last_config_ = next;
        last_observed_ns_ = observed_ns;
        has_observation_ = true;
        return true;
    }

    bool valid() const { return valid_; }

private:
    ConfigEpoch last_config_;
    bool valid_;
    std::uint64_t last_observed_ns_;
    bool has_observation_;
};

/*
 * Generation
 * Owner: the sender while constructing a batch; ownership then moves with the
 * batch into a live slot, retained shadow, journal, or responder receive slot.
 * Validity: wire values 0..255 are all valid, including wrap through zero.
 * The explicit validity bit prevents -1 sentinels from leaking into the type.
 */
class Generation {
public:
    constexpr Generation() : wire_value_(0), valid_(false) {}
    static constexpr Generation from_wire(std::uint8_t value) {
        return Generation(value, true);
    }
    constexpr bool valid() const { return valid_; }
    constexpr std::uint8_t wire_value() const { return wire_value_; }
    constexpr Generation next() const {
        return valid_ ? Generation(static_cast<std::uint8_t>(wire_value_ + 1), true)
                      : Generation();
    }
private:
    constexpr Generation(std::uint8_t value, bool valid)
        : wire_value_(value), valid_(valid) {}
    std::uint8_t wire_value_;
    bool valid_;
};

/*
 * BlockSerial
 * Owner: BlockAckRuntime for one negotiated session/config epoch.
 * Validity: zero is invalid; reset or renegotiation invalidates every prior
 * serial before a new aggregate report can be accepted.
 */
class BlockSerial {
public:
    constexpr BlockSerial() : value_(0) {}
    explicit constexpr BlockSerial(std::uint32_t value) : value_(value) {}
    constexpr bool valid() const { return value_ != 0; }
    constexpr std::uint32_t value() const { return value_; }
private:
    std::uint32_t value_;
};

enum class AckReportKind : std::uint8_t {
    Invalid = 0,
    PerBatch,
    Cumulative,
    Aggregate
};

enum class AckReportOwner : std::uint8_t {
    None = 0,
    LiveSlots,
    RetainedShadow,
    Journal
};

static constexpr std::uint16_t kAckReportSlotCapacity = 8u * 96u;

/*
 * AckReportIdentity
 * Owner: the decoded report until resolution; after validation, exactly one of
 * live slots, retained shadow, or journal owns the target span.
 * Validity: session/config/generation are valid, the span is nonzero, the
 * report kind is explicit, and the owner is resolved before any credit.
 * bitmap_generation must equal target_generation for accepted bitmap reports.
 */
struct AckReportIdentity {
    SessionEpoch session;
    ConfigEpoch config;
    Generation target_generation;
    Generation bitmap_generation;
    BlockSerial block_serial;
    std::uint16_t first_slot;
    std::uint16_t slot_count;
    AckReportKind kind;
    AckReportOwner owner;

    constexpr AckReportIdentity()
        : first_slot(0), slot_count(0), kind(AckReportKind::Invalid),
          owner(AckReportOwner::None) {}

    constexpr bool valid() const {
        return session.valid() && config.valid() && target_generation.valid()
            && bitmap_generation.valid()
            && target_generation.wire_value() == bitmap_generation.wire_value()
            && slot_count != 0
            && static_cast<std::uint32_t>(first_slot) + slot_count
                <= kAckReportSlotCapacity
            && kind != AckReportKind::Invalid
            && owner != AckReportOwner::None
            && (kind != AckReportKind::Aggregate || block_serial.valid());
    }
};

/* Named events are the stable seams against which future timer migrations run. */
enum class LinkPhaseEvent : std::uint8_t {
    Invalid = 0,
    ConnectAccepted,
    HailTxEnd,
    HailListenOpen,
    StartAudioEgressEnd,
    StartCaptureEligible,
    ControlKeydownEnd,
    ControlAckSlotOpen,
    ControlAckSlotClose,
    CmdKeydownStart,
    DataFrameEmitted,
    DataFrameDelivered,
    CmdAudioEgressEnd,
    PeerKeydownEndEstimate,
    ForwardBurstEndProven,
    ForwardBurstEndBound,
    AckSlotOpen,
    ResponderKeyed,
    AckFrameEmitted,
    AckFrameDelivered,
    ResponderAudioEgressEnd,
    AckDecoded,
    AckSlotClose,
    RetransmitTurnStart,
    BlockCommit,
    BreakObserved,
    ConfigActivated,
    RoleActivated,
    ListenerReady,
    NextBatchReady,
    WatchdogDeadline,
    GearshiftDeadline
};

/*
 * LinkPhaseEventToken
 * Owner: the timeline that emits the event.
 * Validity: all identity components and a nonzero local event ordinal are
 * valid. Tokens are local evidence: peers compare event order/duration, not
 * absolute clock values. An old session/config token must mutate no state.
 */
struct LinkPhaseEventToken {
    SessionEpoch session;
    ConfigEpoch config;
    Generation generation;
    std::uint64_t ordinal;
    LinkPhaseEvent event;

    constexpr LinkPhaseEventToken()
        : ordinal(0), event(LinkPhaseEvent::Invalid) {}

    constexpr bool valid() const {
        return session.valid() && config.valid() && generation.valid()
            && ordinal != 0 && event != LinkPhaseEvent::Invalid;
    }
};

}  // namespace protocol
}  // namespace mercury

#endif  // INC_PROTOCOL_CONTRACT_KERNEL_H_
