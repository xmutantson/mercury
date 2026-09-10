#include "datalink_layer/fade_core.h"

#include "../crypto/monocypher.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <limits>
#include <set>
#include <string>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <bcrypt.h>
#else
#include <sys/random.h>
#endif

namespace mercury {
namespace fade {

class RevocationState {
 public:
  RevocationState() : active(false) {}
  std::atomic<bool> active;
};

namespace {

std::mutex g_origin_mutex;
std::set<std::array<std::uint8_t, ORIGIN_BYTES> > g_process_origins;

bool os_random(std::uint8_t* out, std::size_t size) {
  if(!out || size == 0) return false;
#ifdef _WIN32
  return BCryptGenRandom(NULL, out, static_cast<ULONG>(size),
                         BCRYPT_USE_SYSTEM_PREFERRED_RNG) == 0;
#else
  std::size_t offset = 0;
  while(offset < size) {
    const ssize_t got = getrandom(out + offset, size - offset, 0);
    if(got > 0) {
      offset += static_cast<std::size_t>(got);
      continue;
    }
    if(got < 0 && errno == EINTR) continue;
    return false;
  }
  return true;
#endif
}

void put_u32(std::vector<std::uint8_t>* out, std::uint32_t value) {
  for(int shift = 24; shift >= 0; shift -= 8)
    out->push_back(static_cast<std::uint8_t>(value >> shift));
}

void put_u64(std::vector<std::uint8_t>* out, std::uint64_t value) {
  for(int shift = 56; shift >= 0; shift -= 8)
    out->push_back(static_cast<std::uint8_t>(value >> shift));
}

bool all_zero(const std::array<std::uint8_t, ORIGIN_BYTES>& bytes) {
  std::uint8_t folded = 0;
  for(std::uint8_t byte : bytes) folded |= byte;
  return folded == 0;
}

std::uint64_t instance_from_origin(const Origin& origin) {
  std::uint64_t value = 0;
  for(std::size_t i = 0; i < sizeof(value); ++i)
    value = (value << 8) | origin.bytes()[i];
  if(value == 0) {
    for(std::size_t i = sizeof(value); i < 2 * sizeof(value); ++i)
      value = (value << 8) | origin.bytes()[i];
  }
  return value == 0 ? 1 : value;
}

bool digest_equal(const Digest& lhs, const Digest& rhs) {
  std::uint8_t difference = 0;
  for(std::size_t i = 0; i < DIGEST_BYTES; ++i)
    difference |= static_cast<std::uint8_t>(lhs[i] ^ rhs[i]);
  return difference == 0;
}

}  // namespace

bool Address::valid() const {
  return sender != 0 && receiver != 0 && sender != receiver
      && (direction == Direction::A_TO_B || direction == Direction::B_TO_A);
}

bool Address::operator==(const Address& rhs) const {
  return sender == rhs.sender && receiver == rhs.receiver
      && connection == rhs.connection && stream == rhs.stream
      && direction == rhs.direction;
}

Origin::Origin() : bytes_{} {}

Origin::Origin(const std::array<std::uint8_t, ORIGIN_BYTES>& bytes)
    : bytes_(bytes) {}

bool Origin::from_wire(const std::uint8_t* bytes, std::size_t size,
                       Origin* out) {
  if(!bytes || size != ORIGIN_BYTES || !out) return false;
  std::array<std::uint8_t, ORIGIN_BYTES> candidate{};
  std::copy(bytes, bytes + size, candidate.begin());
  if(all_zero(candidate)) return false;
  *out = Origin(candidate);
  return true;
}

bool Origin::valid() const { return !all_zero(bytes_); }

bool OriginFactory::fresh(Origin* out) {
  if(!out) return false;
  for(unsigned attempt = 0; attempt < 16; ++attempt) {
    std::array<std::uint8_t, ORIGIN_BYTES> bytes{};
    if(!os_random(bytes.data(), bytes.size())) return false;
    if(all_zero(bytes)) continue;
    std::lock_guard<std::mutex> lock(g_origin_mutex);
    if(g_process_origins.insert(bytes).second) {
      *out = Origin(bytes);
      return true;
    }
  }
  return false;
}

StreamDescriptor::StreamDescriptor()
    : session_(0), instance_(0), first_generation_(0) {}

bool StreamDescriptor::create_local(const Address& address,
                                    std::uint64_t session,
                                    std::uint64_t first_generation,
                                    OriginFactory* factory,
                                    StreamDescriptor* out) {
  if(!factory || !out || !address.valid() || session == 0) return false;
  Origin origin;
  if(!factory->fresh(&origin)) return false;
  return from_wire(address, session, origin, instance_from_origin(origin),
                   first_generation, out);
}

bool StreamDescriptor::from_wire(const Address& address, std::uint64_t session,
                                 const Origin& origin, std::uint64_t instance,
                                 std::uint64_t first_generation,
                                 StreamDescriptor* out) {
  if(!out || !address.valid() || session == 0 || !origin.valid()
     || instance == 0) return false;
  StreamDescriptor candidate;
  candidate.address_ = address;
  candidate.session_ = session;
  candidate.origin_ = origin;
  candidate.instance_ = instance;
  candidate.first_generation_ = first_generation;
  *out = candidate;
  return true;
}

bool StreamDescriptor::valid() const {
  return address_.valid() && session_ != 0 && origin_.valid()
      && instance_ != 0;
}

bool StreamDescriptor::operator==(const StreamDescriptor& rhs) const {
  return address_ == rhs.address_ && session_ == rhs.session_
      && origin_ == rhs.origin_ && instance_ == rhs.instance_
      && first_generation_ == rhs.first_generation_;
}

bool RecordIdentity::operator==(const RecordIdentity& rhs) const {
  return generation == rhs.generation
      && application_bytes == rhs.application_bytes
      && digest_equal(application_digest, rhs.application_digest)
      && stream == rhs.stream;
}

std::vector<std::uint8_t> canonical_identity_bytes(
    const RecordIdentity& identity) {
  std::vector<std::uint8_t> out;
  if(!identity.valid()) return out;
  static const std::uint8_t domain[] = {
      'M','E','R','C','U','R','Y','-','F','A','D','E','-','C','1',2};
  out.insert(out.end(), domain, domain + sizeof(domain));
  const Address& address = identity.stream.address();
  put_u64(&out, address.sender);
  put_u64(&out, address.receiver);
  put_u32(&out, address.connection);
  put_u32(&out, address.stream);
  out.push_back(static_cast<std::uint8_t>(address.direction));
  put_u64(&out, identity.stream.session());
  out.insert(out.end(), identity.stream.origin().bytes().begin(),
             identity.stream.origin().bytes().end());
  put_u64(&out, identity.stream.instance());
  put_u64(&out, identity.stream.first_generation());
  put_u64(&out, identity.generation);
  put_u64(&out, identity.application_bytes);
  out.insert(out.end(), identity.application_digest.begin(),
             identity.application_digest.end());
  return out;
}

std::vector<std::uint8_t> canonical_feedback_bytes(const Feedback& feedback) {
  std::vector<std::uint8_t> out = canonical_identity_bytes(feedback.identity);
  if(out.empty()) return out;
  out.push_back(static_cast<std::uint8_t>(feedback.kind));
  out.insert(out.end(), feedback.transported_digest.begin(),
             feedback.transported_digest.end());
  put_u64(&out, feedback.committed_application_bytes);
  return out;
}

Digest digest_bytes(const std::uint8_t* bytes, std::size_t size) {
  Digest digest{};
  if(size != 0 && !bytes) return digest;
  crypto_blake2b(digest.data(), digest.size(), bytes, size);
  return digest;
}

SourceRetainer::SourceRetainer(std::uint64_t local_endpoint,
                               const StreamDescriptor& descriptor,
                               Protection protection)
    : local_endpoint_(local_endpoint), descriptor_(descriptor),
      protection_(protection), next_generation_(descriptor.first_generation()),
      exhausted_(false) {}

bool SourceRetainer::valid() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return descriptor_.valid()
      && descriptor_.address().sender == local_endpoint_
      && protection_ != Protection::INVALID;
}

bool SourceRetainer::stage(const std::uint8_t* source,
                           std::size_t source_size,
                           const std::uint8_t* transported,
                           std::size_t transported_size, DataRecord* out) {
  if(!source || source_size == 0 || !transported || transported_size == 0
     || !out) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if(!descriptor_.valid() || descriptor_.address().sender != local_endpoint_
     || protection_ == Protection::INVALID || exhausted_) return false;

  Entry entry;
  entry.record.identity.stream = descriptor_;
  entry.record.identity.generation = next_generation_;
  entry.record.identity.application_bytes = source_size;
  entry.record.identity.application_digest = digest_bytes(source, source_size);
  entry.record.protection = protection_;
  entry.record.transported.assign(transported, transported + transported_size);
  entry.record.transported_digest =
      digest_bytes(entry.record.transported.data(), entry.record.transported.size());
  entry.source.assign(source, source + source_size);
  entry.state = SourceState::SOURCE_STAGED;
  try {
    entries_.emplace(next_generation_, entry);
  } catch (...) {
    return false;
  }
  *out = entry.record;
  if(next_generation_ == std::numeric_limits<std::uint64_t>::max())
    exhausted_ = true;
  else
    ++next_generation_;
  return true;
}

bool SourceRetainer::mark_sent(std::uint64_t generation) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = entries_.find(generation);
  if(it == entries_.end()) return false;
  if(it->second.state == SourceState::SOURCE_STAGED)
    it->second.state = SourceState::SOURCE_SENT;
  return true;
}

bool SourceRetainer::verification_matches(Verification verification) const {
  return (protection_ == Protection::AEAD_COVERED
          && verification == Verification::AEAD_VERIFIED)
      || (protection_ == Protection::AUTHENTICATED_FRAME
          && verification == Verification::FRAME_VERIFIED);
}

bool SourceRetainer::apply_feedback(const Feedback& feedback,
                                    Verification verification) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!verification_matches(verification)
     || feedback.identity.stream != descriptor_) return false;
  auto it = entries_.find(feedback.identity.generation);
  if(it == entries_.end()) {
    const auto released = released_.find(feedback.identity.generation);
    return feedback.kind == FeedbackKind::COMMIT
        && released != released_.end()
        && feedback.identity == released->second.identity
        && digest_equal(feedback.transported_digest, released->second.digest)
        && feedback.committed_application_bytes
            == released->second.application_bytes;
  }
  if(feedback.identity != it->second.record.identity
     || !digest_equal(feedback.transported_digest,
                   it->second.record.transported_digest)) return false;
  if(feedback.kind == FeedbackKind::RECEIPT) {
    if(feedback.committed_application_bytes != 0) return false;
    it->second.state = SourceState::RECEIPT_OBSERVED;
    return true;  // The source remains retained.
  }
  if(feedback.kind != FeedbackKind::COMMIT) return false;
  if(feedback.committed_application_bytes != it->second.source.size())
    return false;
  Released released;
  released.identity = it->second.record.identity;
  released.digest = it->second.record.transported_digest;
  released.application_bytes = it->second.source.size();
  try {
    if(!released_.emplace(feedback.identity.generation, released).second)
      return false;
  } catch (...) {
    return false;
  }
  entries_.erase(it);  // COMMIT is the sole ordinary release operation.
  return true;
}

bool SourceRetainer::rebind(const StreamDescriptor& replacement) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!replacement.valid()
     || replacement.address().sender != local_endpoint_
     || replacement.address().sender != descriptor_.address().sender
     || replacement.address().receiver != descriptor_.address().receiver
     || replacement.address().stream != descriptor_.address().stream
     || replacement.address().direction != descriptor_.address().direction
     || replacement.origin() == descriptor_.origin()
     || replacement.instance() == descriptor_.instance()) return false;

  std::map<std::uint64_t, Entry> rebound;
  std::uint64_t generation = replacement.first_generation();
  bool exhausted = false;
  for(const auto& old : entries_) {
    if(exhausted) return false;
    Entry next = old.second;
    next.record.identity.stream = replacement;
    next.record.identity.generation = generation;
    next.state = SourceState::SOURCE_STAGED;
    rebound.emplace(generation, std::move(next));
    if(generation == std::numeric_limits<std::uint64_t>::max())
      exhausted = true;
    else
      ++generation;
  }
  descriptor_ = replacement;
  entries_.swap(rebound);
  released_.clear();
  next_generation_ = generation;
  exhausted_ = exhausted;
  return true;
}

std::size_t SourceRetainer::retained_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return entries_.size();
}

std::size_t SourceRetainer::retained_source_bytes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::size_t total = 0;
  for(const auto& entry : entries_) total += entry.second.source.size();
  return total;
}

bool SourceRetainer::retained(std::uint64_t generation,
                              SourceState* state) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = entries_.find(generation);
  if(it == entries_.end()) return false;
  if(state) *state = it->second.state;
  return true;
}

bool SourceRetainer::record(std::uint64_t generation, DataRecord* out) const {
  if(!out) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = entries_.find(generation);
  if(it == entries_.end()) return false;
  *out = it->second.record;
  return true;
}

CommitTicket::CommitTicket()
    : receiver_(nullptr), lifetime_(0), unused_(false) {}

CommitTicket::CommitTicket(CommitTicket&& rhs) noexcept
    : receiver_(rhs.receiver_), lifetime_(rhs.lifetime_),
      identity_(rhs.identity_), digest_(rhs.digest_), unused_(rhs.unused_),
      revocation_state_(std::move(rhs.revocation_state_)) {
  rhs.receiver_ = nullptr;
  rhs.unused_ = false;
}

CommitTicket& CommitTicket::operator=(CommitTicket&& rhs) noexcept {
  if(this == &rhs) return *this;
  receiver_ = rhs.receiver_;
  lifetime_ = rhs.lifetime_;
  identity_ = rhs.identity_;
  digest_ = rhs.digest_;
  unused_ = rhs.unused_;
  revocation_state_ = std::move(rhs.revocation_state_);
  rhs.receiver_ = nullptr;
  rhs.unused_ = false;
  return *this;
}

CommitTicket::~CommitTicket() {
  receiver_ = nullptr;
  unused_ = false;
}

bool CommitTicket::valid() const {
  const std::shared_ptr<RevocationState> state = revocation_state_.lock();
  return receiver_ != nullptr && unused_ && state
      && state->active.load(std::memory_order_acquire);
}

Receiver::Receiver(const Address& expected_address,
                   std::uint64_t expected_session, OrderedSink sink,
                   std::size_t max_pending)
    : expected_address_(expected_address), expected_session_(expected_session),
      sink_(std::move(sink)), max_pending_(max_pending), next_generation_(0),
      lifetime_(1), published_(false), revoked_(false), expired_(false),
      stop_worker_(false), exhausted_(false),
      revocation_state_(std::make_shared<RevocationState>()),
      deadline_thread_(&Receiver::deadline_worker, this) {}

Receiver::~Receiver() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    revoke_locked(false);
    stop_worker_ = true;
    deadline_cv_.notify_all();
  }
  if(deadline_thread_.joinable()) deadline_thread_.join();
}

PublishResult Receiver::publish_once(
    const StreamDescriptor& descriptor, Clock::time_point absolute_deadline) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!descriptor.valid() || !expected_address_.valid()
     || descriptor.address() != expected_address_
     || descriptor.session() != expected_session_
     || expected_session_ == 0 || !sink_ || max_pending_ == 0
     || absolute_deadline <= Clock::now()) return PublishResult::INVALID;
  if(published_) {
    if(!revoked_ && descriptor == descriptor_
       && absolute_deadline == deadline_) return PublishResult::EXACT_DUPLICATE;
    return PublishResult::CONFLICT_REFUSED;
  }
  descriptor_ = descriptor;
  next_generation_ = descriptor.first_generation();
  deadline_ = absolute_deadline;
  published_ = true;
  revoked_ = false;
  expired_ = false;
  revocation_state_->active.store(true, std::memory_order_release);
  deadline_cv_.notify_all();
  return PublishResult::PUBLISHED;
}

bool Receiver::verification_matches(Protection protection,
                                    Verification verification) const {
  return (protection == Protection::AEAD_COVERED
          && verification == Verification::AEAD_VERIFIED)
      || (protection == Protection::AUTHENTICATED_FRAME
          && verification == Verification::FRAME_VERIFIED);
}

bool Receiver::same_stream(const RecordIdentity& identity) const {
  return identity.stream == descriptor_;
}

AdmitResult Receiver::admit(const DataRecord& record,
                            Verification verification, Feedback* feedback) {
  if(feedback) *feedback = Feedback();
  std::lock_guard<std::mutex> lock(mutex_);
  if(!published_ || revoked_) return AdmitResult::REFUSED;
  if(Clock::now() >= deadline_) {
    revoke_locked(true);
    return AdmitResult::REFUSED;
  }
  // This check is unconditional and precedes every parser/storage mutation.
  if(!record.identity.valid() || !same_stream(record.identity)
     || !verification_matches(record.protection, verification)
     || record.transported.empty()) return AdmitResult::REFUSED;
  const Digest actual = digest_bytes(record.transported.data(),
                                     record.transported.size());
  if(!digest_equal(actual, record.transported_digest))
    return AdmitResult::REFUSED;

  // A retransmission after lost COMMIT feedback is acknowledged as COMMIT
  // again, without re-entering the sink. Keep this check behind every normal
  // identity/authentication/digest gate so old-generation is never a bypass.
  auto prior = committed_.find(record.identity.generation);
  if(prior != committed_.end()) {
    if(prior->second.identity != record.identity
       || prior->second.protection != record.protection
       || !digest_equal(prior->second.digest, record.transported_digest)) {
      revoke_locked(false);
      return AdmitResult::CONFLICT_REVOKED;
    }
    if(feedback) {
      feedback->kind = FeedbackKind::COMMIT;
      feedback->identity = record.identity;
      feedback->transported_digest = record.transported_digest;
      feedback->committed_application_bytes = prior->second.application_bytes;
    }
    return AdmitResult::EXACT_DUPLICATE;
  }
  if(exhausted_ || record.identity.generation < next_generation_)
    return AdmitResult::REFUSED;
  const std::uint64_t distance = record.identity.generation - next_generation_;
  if(distance >= max_pending_) return AdmitResult::REFUSED;

  auto present = stored_.find(record.identity.generation);
  if(present != stored_.end()) {
    if(present->second.identity == record.identity
       && present->second.protection == record.protection
       && digest_equal(present->second.transported_digest,
                       record.transported_digest)
       && present->second.transported == record.transported) {
      if(feedback) {
        feedback->kind = FeedbackKind::RECEIPT;
        feedback->identity = record.identity;
        feedback->transported_digest = record.transported_digest;
      }
      return AdmitResult::EXACT_DUPLICATE;
    }
    revoke_locked(false);
    return AdmitResult::CONFLICT_REVOKED;
  }
  try {
    stored_.emplace(record.identity.generation, record);
  } catch (...) {
    return AdmitResult::REFUSED;
  }
  if(feedback) {
    feedback->kind = FeedbackKind::RECEIPT;
    feedback->identity = record.identity;
    feedback->transported_digest = record.transported_digest;
  }
  return AdmitResult::ACCEPTED;
}

bool Receiver::prepare_commit(const RecordIdentity& identity,
                              CommitTicket* out) {
  if(!out) return false;
  *out = CommitTicket();
  std::lock_guard<std::mutex> lock(mutex_);
  if(!published_ || revoked_ || exhausted_ || Clock::now() >= deadline_) {
    if(published_ && !revoked_ && Clock::now() >= deadline_)
      revoke_locked(true);
    return false;
  }
  if(!same_stream(identity) || identity.generation != next_generation_)
    return false;
  auto stored = stored_.find(next_generation_);
  if(stored == stored_.end() || stored->second.identity != identity)
    return false;
  out->receiver_ = this;
  out->lifetime_ = lifetime_;
  out->identity_ = identity;
  out->digest_ = stored->second.transported_digest;
  out->unused_ = true;
  out->revocation_state_ = revocation_state_;
  return true;
}

bool Receiver::commit(CommitTicket&& ticket,
                      const std::uint8_t* application_bytes,
                      std::size_t application_size, Feedback* committed) {
  if(committed) *committed = Feedback();
  std::lock_guard<std::mutex> lock(mutex_);
  const std::shared_ptr<RevocationState> ticket_state =
      ticket.revocation_state_.lock();
  const bool ticket_valid = ticket.receiver_ == this && ticket.unused_
      && ticket_state == revocation_state_
      && ticket_state->active.load(std::memory_order_acquire);
  ticket.unused_ = false;
  ticket.receiver_ = nullptr;
  if(!ticket_valid || !application_bytes || application_size == 0
     || !published_ || revoked_ || exhausted_ || ticket.lifetime_ != lifetime_)
    return false;
  if(Clock::now() >= deadline_) {
    revoke_locked(true);
    return false;
  }
  if(!same_stream(ticket.identity_)
     || ticket.identity_.generation != next_generation_) return false;
  if(application_size != ticket.identity_.application_bytes
     || !digest_equal(digest_bytes(application_bytes, application_size),
                      ticket.identity_.application_digest)) return false;
  auto stored = stored_.find(next_generation_);
  if(stored == stored_.end() || stored->second.identity != ticket.identity_
     || !digest_equal(stored->second.transported_digest, ticket.digest_))
    return false;

  CommittedRecord commitment;
  commitment.identity = ticket.identity_;
  commitment.protection = stored->second.protection;
  commitment.digest = ticket.digest_;
  commitment.application_bytes = application_size;
  std::map<std::uint64_t, CommittedRecord>::iterator commitment_it;
  try {
    const auto inserted = committed_.emplace(
        ticket.identity_.generation, commitment);
    if(!inserted.second) return false;
    commitment_it = inserted.first;
  } catch (...) {
    return false;
  }

  CommitView view;
  view.identity = ticket.identity_;
  view.transported_digest = ticket.digest_;
  view.application_bytes = application_bytes;
  view.application_size = application_size;
  bool accepted = false;
  try {
    accepted = sink_(view);
  } catch (...) {
    accepted = false;
  }
  if(!accepted) {
    committed_.erase(commitment_it);
    return false;
  }

  if(committed) {
    committed->kind = FeedbackKind::COMMIT;
    committed->identity = ticket.identity_;
    committed->transported_digest = ticket.digest_;
    committed->committed_application_bytes = application_size;
  }
  stored_.erase(stored);
  if(next_generation_ == std::numeric_limits<std::uint64_t>::max())
    exhausted_ = true;
  else
    ++next_generation_;
  return true;
}

void Receiver::revoke_locked(bool expired) {
  if(revoked_) {
    if(expired) expired_ = true;
    return;
  }
  revoked_ = true;
  expired_ = expired;
  revocation_state_->active.store(false, std::memory_order_release);
  stored_.clear();
  committed_.clear();
  if(lifetime_ != std::numeric_limits<std::uint64_t>::max()) ++lifetime_;
  else lifetime_ = 0;  // No ticket can equal an exhausted prior lifetime.
  deadline_cv_.notify_all();
}

void Receiver::revoke() {
  std::lock_guard<std::mutex> lock(mutex_);
  revoke_locked(false);
}

void Receiver::deadline_worker() {
  std::unique_lock<std::mutex> lock(mutex_);
  deadline_cv_.wait(lock, [this] { return stop_worker_ || published_; });
  if(stop_worker_) return;
  while(!stop_worker_ && published_ && !revoked_) {
    if(deadline_cv_.wait_until(lock, deadline_,
        [this] { return stop_worker_ || revoked_; })) break;
    if(!stop_worker_ && !revoked_ && Clock::now() >= deadline_) {
      revoke_locked(true);
      break;
    }
  }
}

bool Receiver::pinned() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return published_ && !revoked_;
}

bool Receiver::revoked() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return revoked_;
}

bool Receiver::expired() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return expired_;
}

std::uint64_t Receiver::next_generation() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return next_generation_;
}

std::size_t Receiver::stored_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stored_.size();
}

Receiver::Clock::time_point Receiver::deadline() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return deadline_;
}

}  // namespace fade
}  // namespace mercury
