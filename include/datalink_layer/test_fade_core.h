/*
 * Shared delivery-correctness core.
 *
 * The core deliberately owns no modem framing, crypto primitive, or ARQ policy.
 * A transport authenticates the canonical identity bytes once (as AEAD AAD when
 * AEAD is already present, otherwise under its existing frame authenticator),
 * mints a WireAttestation at that one verification boundary, and hands the
 * verified record to Receiver together with that attestation.  The core then
 * preserves that identity, storage ownership, delivery order, and the
 * receipt/commit split.
 */
#ifndef INC_DATALINK_LAYER_FADE_CORE_H_
#define INC_DATALINK_LAYER_FADE_CORE_H_

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace mercury {
namespace fade {

static const std::size_t ORIGIN_BYTES = 32;
static const std::size_t DIGEST_BYTES = 32;

using Digest = std::array<std::uint8_t, DIGEST_BYTES>;

enum class Direction : std::uint8_t { A_TO_B = 0, B_TO_A = 1 };

struct Address {
  std::uint64_t sender = 0;
  std::uint64_t receiver = 0;
  std::uint32_t connection = 0;
  std::uint32_t stream = 0;
  Direction direction = Direction::A_TO_B;

  bool valid() const;
  bool operator==(const Address& rhs) const;
  bool operator!=(const Address& rhs) const { return !(*this == rhs); }
};

class Origin {
 public:
  Origin();
  static bool from_wire(const std::uint8_t* bytes, std::size_t size,
                        Origin* out);
  bool valid() const;
  const std::array<std::uint8_t, ORIGIN_BYTES>& bytes() const { return bytes_; }
  bool operator==(const Origin& rhs) const { return bytes_ == rhs.bytes_; }
  bool operator!=(const Origin& rhs) const { return !(*this == rhs); }

 private:
  friend class OriginFactory;
  explicit Origin(const std::array<std::uint8_t, ORIGIN_BYTES>& bytes);
  std::array<std::uint8_t, ORIGIN_BYTES> bytes_;
};

/* Produces process-unique CSPRNG origins and refuses an RNG repeat. */
class OriginFactory {
 public:
  bool fresh(Origin* out);
};

class StreamDescriptor {
 public:
  StreamDescriptor();

  static bool create_local(const Address& address, std::uint64_t session,
                           std::uint64_t first_generation,
                           OriginFactory* factory, StreamDescriptor* out);
  static bool from_wire(const Address& address, std::uint64_t session,
                        const Origin& origin, std::uint64_t instance,
                        std::uint64_t first_generation,
                        StreamDescriptor* out);

  bool valid() const;
  const Address& address() const { return address_; }
  std::uint64_t session() const { return session_; }
  const Origin& origin() const { return origin_; }
  std::uint64_t instance() const { return instance_; }
  std::uint64_t first_generation() const { return first_generation_; }
  bool operator==(const StreamDescriptor& rhs) const;
  bool operator!=(const StreamDescriptor& rhs) const { return !(*this == rhs); }

 private:
  Address address_;
  std::uint64_t session_;
  Origin origin_;
  std::uint64_t instance_;
  std::uint64_t first_generation_;
};

struct RecordIdentity {
  StreamDescriptor stream;
  std::uint64_t generation = 0;
  std::uint64_t application_bytes = 0;
  Digest application_digest{};

  bool valid() const { return stream.valid() && application_bytes != 0; }
  bool operator==(const RecordIdentity& rhs) const;
  bool operator!=(const RecordIdentity& rhs) const { return !(*this == rhs); }
};

/*
 * Verification names the mode of the one wire-authentication boundary.
 * AEAD_COVERED means canonical_identity_bytes() was already AEAD-covered; the
 * core intentionally adds no second MAC.  AUTHENTICATED_FRAME is for a
 * non-AEAD carrier whose existing protected frame covers the same bytes.
 */
enum class Protection : std::uint8_t {
  INVALID = 0,
  AEAD_COVERED,
  AUTHENTICATED_FRAME
};

enum class Verification : std::uint8_t {
  FAILED = 0,
  AEAD_VERIFIED,
  FRAME_VERIFIED
};

struct DataRecord {
  RecordIdentity identity;
  Protection protection = Protection::INVALID;
  Digest transported_digest{};
  std::vector<std::uint8_t> transported;
};

enum class FeedbackKind : std::uint8_t { RECEIPT = 1, COMMIT = 2 };

struct Feedback {
  FeedbackKind kind = FeedbackKind::RECEIPT;
  RecordIdentity identity;
  Digest transported_digest{};
  std::uint64_t committed_application_bytes = 0;
};

std::vector<std::uint8_t> canonical_identity_bytes(
    const RecordIdentity& identity);
std::vector<std::uint8_t> canonical_feedback_bytes(const Feedback& feedback);
Digest digest_bytes(const std::uint8_t* bytes, std::size_t size);

/*
 * WireAttestation is the object-bound form of the wire-authentication result.
 * The transport mints one at its single verification boundary, immediately
 * after the existing AEAD or protected-frame check accepts, over exactly the
 * record or feedback the check accepted.  The attestation binds the canonical
 * encoding, the protection mode, and the transported digest, so no field can
 * change between authentication and admission, and a bare success value can
 * no longer stand in for the boundary.  Attestations are move-only and are
 * spent by the call that consumes them; a spent or default-constructed
 * attestation admits nothing.
 */
class WireAttestation {
 public:
  WireAttestation();
  WireAttestation(WireAttestation&& rhs) noexcept;
  WireAttestation& operator=(WireAttestation&& rhs) noexcept;
  WireAttestation(const WireAttestation&) = delete;
  WireAttestation& operator=(const WireAttestation&) = delete;

  static WireAttestation attest_data(const DataRecord& record,
                                     Verification verification);
  static WireAttestation attest_feedback(const Feedback& feedback,
                                         Verification verification);

  bool valid() const;
  Verification verification() const { return verification_; }

 private:
  friend class Receiver;
  friend class SourceRetainer;
  enum class Kind : std::uint8_t { ABSENT = 0, DATA_RECORD, FEEDBACK_RECORD };
  bool covers_data(const DataRecord& record) const;
  bool covers_feedback(const Feedback& feedback) const;
  void consume();
  Kind kind_;
  Verification verification_;
  Digest binding_;
};

enum class SourceState : std::uint8_t {
  SOURCE_STAGED = 0,
  SOURCE_SENT,
  RECEIPT_OBSERVED
};

/*
 * SourceRetainer owns the original source bytes until a matching COMMIT.
 * RECEIPT only changes retransmission state; it never frees source authority.
 * Feedback is applied only with the attestation minted over exactly the
 * feedback bytes the wire authenticator accepted; feedback relabelled after
 * that boundary no longer matches its attestation and is refused.
 */
class SourceRetainer {
 public:
  SourceRetainer(std::uint64_t local_endpoint,
                 const StreamDescriptor& descriptor, Protection protection);

  bool valid() const;
  bool stage(const std::uint8_t* source, std::size_t source_size,
             const std::uint8_t* transported, std::size_t transported_size,
             DataRecord* out);
  bool mark_sent(std::uint64_t generation);
  bool apply_feedback(const Feedback& feedback, WireAttestation&& attestation);
  bool rebind(const StreamDescriptor& replacement);

  std::size_t retained_count() const;
  std::size_t retained_source_bytes() const;
  bool retained(std::uint64_t generation, SourceState* state = nullptr) const;
  bool record(std::uint64_t generation, DataRecord* out) const;

 private:
  struct Entry {
    DataRecord record;
    std::vector<std::uint8_t> source;
    SourceState state = SourceState::SOURCE_STAGED;
  };
  struct Released {
    RecordIdentity identity;
    Digest digest{};
    std::uint64_t application_bytes = 0;
  };

  bool verification_matches(Verification verification) const;

  mutable std::mutex mutex_;
  std::uint64_t local_endpoint_;
  StreamDescriptor descriptor_;
  Protection protection_;
  std::uint64_t next_generation_;
  bool exhausted_;
  std::map<std::uint64_t, Entry> entries_;
  std::map<std::uint64_t, Released> released_;
};

struct CommitView {
  RecordIdentity identity;
  Digest transported_digest{};
  const std::uint8_t* application_bytes = nullptr;
  std::size_t application_size = 0;
};

class Receiver;
class RevocationState;

/* Move-only proof tied to one stored generation and one receiver lifetime. */
class CommitTicket {
 public:
  CommitTicket();
  CommitTicket(CommitTicket&& rhs) noexcept;
  CommitTicket& operator=(CommitTicket&& rhs) noexcept;
  ~CommitTicket();
  CommitTicket(const CommitTicket&) = delete;
  CommitTicket& operator=(const CommitTicket&) = delete;
  bool valid() const;

 private:
  friend class Receiver;
  Receiver* receiver_;
  std::uint64_t lifetime_;
  RecordIdentity identity_;
  Digest digest_;
  bool unused_;
  std::weak_ptr<RevocationState> revocation_state_;
};

enum class PublishResult : std::uint8_t {
  PUBLISHED = 0,
  EXACT_DUPLICATE,
  CONFLICT_REFUSED,
  INVALID
};

enum class AdmitResult : std::uint8_t {
  ACCEPTED = 0,
  EXACT_DUPLICATE,
  REFUSED,
  CONFLICT_REVOKED
};

/*
 * Receiver has exactly one application sink for its lifetime.  Admission,
 * ticket minting, append, ordered-cursor advance, and revoke serialize on the
 * same mutex.  Therefore revoke() returning is a complete revocation barrier.
 *
 * Admission consumes the wire attestation minted for exactly this record and
 * requires a receipt output.  A null receipt output, a missing or spent
 * attestation, or any record field changed since attestation refuses the
 * record before storage; a record can therefore never reach COMMIT without a
 * RECEIPT having been produced first.
 */
class Receiver {
 public:
  using OrderedSink = std::function<bool(const CommitView&)>;
  using Clock = std::chrono::steady_clock;

  Receiver(const Address& expected_address, std::uint64_t expected_session,
           OrderedSink sink, std::size_t max_pending = 64);
  ~Receiver();
  Receiver(const Receiver&) = delete;
  Receiver& operator=(const Receiver&) = delete;

  PublishResult publish_once(const StreamDescriptor& descriptor,
                             Clock::time_point absolute_deadline);
  AdmitResult admit(const DataRecord& record, WireAttestation&& attestation,
                    Feedback* receipt);
  bool prepare_commit(const RecordIdentity& identity, CommitTicket* out);
  bool commit(CommitTicket&& ticket, const std::uint8_t* application_bytes,
              std::size_t application_size, Feedback* committed);
  void revoke();

  bool pinned() const;
  bool revoked() const;
  bool expired() const;
  std::uint64_t next_generation() const;
  std::size_t stored_count() const;
  Clock::time_point deadline() const;

 private:
  struct CommittedRecord {
    RecordIdentity identity;
    Protection protection = Protection::INVALID;
    Digest digest{};
    std::uint64_t application_bytes = 0;
  };

  bool verification_matches(Protection protection,
                            Verification verification) const;
  bool same_stream(const RecordIdentity& identity) const;
  void revoke_locked(bool expired);
  void deadline_worker();

  const Address expected_address_;
  const std::uint64_t expected_session_;
  const OrderedSink sink_;
  const std::size_t max_pending_;
  mutable std::mutex mutex_;
  std::condition_variable deadline_cv_;
  StreamDescriptor descriptor_;
  std::map<std::uint64_t, DataRecord> stored_;
  std::map<std::uint64_t, CommittedRecord> committed_;
  std::uint64_t next_generation_;
  std::uint64_t lifetime_;
  Clock::time_point deadline_;
  bool published_;
  bool revoked_;
  bool expired_;
  bool stop_worker_;
  bool exhausted_;
  std::shared_ptr<RevocationState> revocation_state_;
  std::thread deadline_thread_;
};

int run_fade_core_tests();

}  // namespace fade
}  // namespace mercury

#endif  // INC_DATALINK_LAYER_FADE_CORE_H_
