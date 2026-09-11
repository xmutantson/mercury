#include "datalink_layer/fade_core.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <limits>
#include <thread>

namespace mercury {
namespace fade {
namespace {

struct TestLog {
  int failures = 0;
  void check(bool condition, const char* suite, const char* name) {
    std::printf("[TEST-FADE-CORE/%s] %s %s\n", suite,
                condition ? "PASS" : "FAIL", name);
    if(!condition) ++failures;
  }
};

Address address(std::uint64_t sender, std::uint64_t receiver,
                std::uint32_t connection, std::uint32_t stream,
                Direction direction) {
  Address value;
  value.sender = sender;
  value.receiver = receiver;
  value.connection = connection;
  value.stream = stream;
  value.direction = direction;
  return value;
}

bool make_descriptor(OriginFactory* factory, const Address& route,
                     std::uint64_t session, std::uint64_t first,
                     StreamDescriptor* out) {
  return StreamDescriptor::create_local(route, session, first, factory, out);
}

std::vector<std::uint8_t> bytes(std::uint8_t seed, std::size_t count) {
  std::vector<std::uint8_t> out(count);
  for(std::size_t i = 0; i < count; ++i)
    out[i] = static_cast<std::uint8_t>(seed + i * 37u + i * i * 11u);
  return out;
}

std::vector<std::uint8_t> block(std::uint8_t seed) {
  std::vector<std::uint8_t> out(24);
  for(std::size_t i = 0; i < out.size(); ++i)
    out[i] = static_cast<std::uint8_t>(seed + 13 * i);
  return out;
}

Receiver::Clock::time_point future_ms(int milliseconds) {
  return Receiver::Clock::now() + std::chrono::milliseconds(milliseconds);
}

Receiver::Clock::time_point after_ms(int ms) {
  return Receiver::Clock::now() + std::chrono::milliseconds(ms);
}

DataRecord fabricate(const StreamDescriptor& descriptor,
                     std::uint64_t generation,
                     const std::vector<std::uint8_t>& value,
                     Protection protection) {
  DataRecord record;
  record.identity.stream = descriptor;
  record.identity.generation = generation;
  record.identity.application_bytes = value.size();
  record.identity.application_digest = digest_bytes(value.data(), value.size());
  record.protection = protection;
  record.transported = value;
  record.transported_digest = digest_bytes(value.data(), value.size());
  return record;
}

/*
 * The tests model the transport's single wire-authentication boundary: each
 * admission or feedback application first mints the attestation over exactly
 * the bytes the scenario's authenticator saw, then hands it to the core.
 */
AdmitResult admit_verified(Receiver* receiver, const DataRecord& record,
                           Verification verification, Feedback* receipt) {
  return receiver->admit(
      record, WireAttestation::attest_data(record, verification), receipt);
}

bool apply_verified(SourceRetainer* source, const Feedback& feedback,
                    Verification verification) {
  return source->apply_feedback(
      feedback, WireAttestation::attest_feedback(feedback, verification));
}

}  // namespace

int run_fade_core_tests() {
  TestLog log;
  OriginFactory factory;
  const Address forward = address(0x414c504841ULL, 0x425241564fULL,
                                  71, 9, Direction::A_TO_B);
  const Address reverse = address(0x425241564fULL, 0x414c504841ULL,
                                  71, 9, Direction::B_TO_A);
  StreamDescriptor fwd;
  StreamDescriptor rev;
  log.check(make_descriptor(&factory, forward, 7001, 0, &fwd),
            "ORIGIN", "fresh forward directional descriptor");
  log.check(make_descriptor(&factory, reverse, 7001, 0, &rev),
            "ORIGIN", "fresh reverse directional descriptor");
  log.check(fwd.origin() != rev.origin() && fwd.instance() != rev.instance(),
            "ORIGIN", "session directions have distinct immutable origins");

  bool origin_set_unique = true;
  std::vector<Origin> minted;
  for(unsigned i = 0; i < 256; ++i) {
    Origin origin;
    if(!factory.fresh(&origin)) { origin_set_unique = false; break; }
    for(const Origin& prior : minted)
      if(prior == origin) origin_set_unique = false;
    minted.push_back(origin);
  }
  log.check(origin_set_unique, "ORIGIN",
            "256 additional origins contain no process-local repeat");

  // Exact S1b descriptor-forgery reproduction: authenticate generation 1 under
  // baseline 0, then relabel only the stream's first generation to 1.  The
  // relabel must change both canonical encodings, so it cannot reuse either
  // the DATA authenticator or a feedback authenticator.
  const Address forged_address = address(0x1111111111111111ULL,
                                         0x2222222222222222ULL,
                                         77, 9, Direction::A_TO_B);
  std::uint8_t origin_bytes[ORIGIN_BYTES];
  for(std::size_t i = 0; i < ORIGIN_BYTES; ++i)
    origin_bytes[i] = static_cast<std::uint8_t>(0x41 + i);
  Origin origin;
  const bool origin_created =
      Origin::from_wire(origin_bytes, sizeof(origin_bytes), &origin);
  StreamDescriptor baseline0;
  StreamDescriptor baseline1;
  const bool baselines_created = origin_created
      && StreamDescriptor::from_wire(
          forged_address, 9001, origin, 0x4142434445464748ULL, 0, &baseline0)
      && StreamDescriptor::from_wire(
          forged_address, 9001, origin, 0x4142434445464748ULL, 1, &baseline1);

  const std::vector<std::uint8_t> b1 = block(0x83);
  DataRecord genuine1 = fabricate(baseline0, 1, b1,
                                  Protection::AEAD_COVERED);
  DataRecord relabelled1 = genuine1;
  relabelled1.identity.stream = baseline1;
  const bool unequal_descriptors = baseline0 != baseline1;
  const bool canonical_collision =
      canonical_identity_bytes(genuine1.identity)
          == canonical_identity_bytes(relabelled1.identity);
  Feedback genuine_feedback;
  genuine_feedback.kind = FeedbackKind::COMMIT;
  genuine_feedback.identity = genuine1.identity;
  genuine_feedback.transported_digest = genuine1.transported_digest;
  genuine_feedback.committed_application_bytes = b1.size();
  Feedback relabelled_feedback = genuine_feedback;
  relabelled_feedback.identity = relabelled1.identity;
  const bool feedback_collision =
      canonical_feedback_bytes(genuine_feedback)
          == canonical_feedback_bytes(relabelled_feedback);
  std::vector<std::uint8_t> forged_sink;
  Receiver forged_receiver(forged_address, 9001, [&](const CommitView& view) {
    forged_sink.insert(forged_sink.end(), view.application_bytes,
                       view.application_bytes + view.application_size);
    return true;
  });
  Feedback forged_receipt;
  CommitTicket forged_ticket;
  Feedback forged_commit;
  const bool forged_published = baselines_created
      && forged_receiver.publish_once(baseline1, after_ms(2000))
          == PublishResult::PUBLISHED;
  const bool forged_admitted = forged_published
      && admit_verified(&forged_receiver, relabelled1,
                        Verification::AEAD_VERIFIED,
                        &forged_receipt) == AdmitResult::ACCEPTED;
  const bool forged_committed = forged_admitted
      && forged_receiver.prepare_commit(relabelled1.identity, &forged_ticket)
      && forged_receiver.commit(std::move(forged_ticket), b1.data(), b1.size(),
                                &forged_commit);
  const bool descriptor_secure = !(unequal_descriptors && canonical_collision
      && feedback_collision
      && forged_published && forged_admitted && forged_committed
      && forged_sink == b1);
  log.check(baselines_created && unequal_descriptors
            && !canonical_collision && !feedback_collision
            && descriptor_secure,
            "DESCRIPTOR-FORGERY",
            "first_generation relabel cannot reuse canonical DATA or feedback "
            "bytes to commit generation 1 as the first 24-byte block");

  // Cover the whole identity class, not one example descriptor. Every field
  // consulted by Address, StreamDescriptor, or RecordIdentity equality must
  // independently perturb DATA identity bytes and inherited feedback bytes.
  std::uint8_t alternate_origin_bytes[ORIGIN_BYTES];
  std::memcpy(alternate_origin_bytes, origin_bytes,
              sizeof(alternate_origin_bytes));
  alternate_origin_bytes[0] ^= 0x80;
  Origin alternate_origin;
  const bool alternate_origin_created = Origin::from_wire(
      alternate_origin_bytes, sizeof(alternate_origin_bytes),
      &alternate_origin);
  struct IdentityVariant {
    const char* field;
    RecordIdentity identity;
  };
  std::vector<IdentityVariant> identity_variants;
  const auto add_stream_variant = [&](const char* field,
                                      const Address& variant_address,
                                      std::uint64_t session,
                                      const Origin& variant_origin,
                                      std::uint64_t instance,
                                      std::uint64_t first_generation) {
    StreamDescriptor descriptor;
    if(!StreamDescriptor::from_wire(variant_address, session, variant_origin,
                                    instance, first_generation, &descriptor))
      return false;
    RecordIdentity identity = genuine1.identity;
    identity.stream = descriptor;
    identity_variants.push_back(IdentityVariant{field, identity});
    return true;
  };
  Address changed_address = forged_address;
  changed_address.sender += 1;
  bool coverage_setup = alternate_origin_created
      && add_stream_variant("sender is canonically bound", changed_address,
                            9001, origin, 0x4142434445464748ULL, 0);
  changed_address = forged_address;
  changed_address.receiver += 1;
  coverage_setup = add_stream_variant(
      "receiver is canonically bound", changed_address, 9001, origin,
      0x4142434445464748ULL, 0) && coverage_setup;
  changed_address = forged_address;
  changed_address.connection += 1;
  coverage_setup = add_stream_variant(
      "connection is canonically bound", changed_address, 9001, origin,
      0x4142434445464748ULL, 0) && coverage_setup;
  changed_address = forged_address;
  changed_address.stream += 1;
  coverage_setup = add_stream_variant(
      "stream is canonically bound", changed_address, 9001, origin,
      0x4142434445464748ULL, 0) && coverage_setup;
  changed_address = forged_address;
  changed_address.direction = Direction::B_TO_A;
  coverage_setup = add_stream_variant(
      "direction is canonically bound", changed_address, 9001, origin,
      0x4142434445464748ULL, 0) && coverage_setup;
  coverage_setup = add_stream_variant(
      "session is canonically bound", forged_address, 9002, origin,
      0x4142434445464748ULL, 0) && coverage_setup;
  coverage_setup = add_stream_variant(
      "origin is canonically bound", forged_address, 9001, alternate_origin,
      0x4142434445464748ULL, 0) && coverage_setup;
  coverage_setup = add_stream_variant(
      "instance is canonically bound", forged_address, 9001, origin,
      0x4142434445464749ULL, 0) && coverage_setup;
  coverage_setup = add_stream_variant(
      "first_generation is canonically bound", forged_address, 9001, origin,
      0x4142434445464748ULL, 1) && coverage_setup;

  RecordIdentity changed_identity = genuine1.identity;
  changed_identity.generation += 1;
  identity_variants.push_back(
      IdentityVariant{"generation is canonically bound", changed_identity});
  changed_identity = genuine1.identity;
  changed_identity.application_bytes += 1;
  identity_variants.push_back(IdentityVariant{
      "application_bytes is canonically bound", changed_identity});
  changed_identity = genuine1.identity;
  changed_identity.application_digest[0] ^= 0x01;
  identity_variants.push_back(IdentityVariant{
      "application_digest is canonically bound", changed_identity});

  log.check(coverage_setup && identity_variants.size() == 12,
            "IDENTITY-COVERAGE", "all identity-field variants constructed");
  for(const IdentityVariant& variant : identity_variants) {
    Feedback variant_feedback = genuine_feedback;
    variant_feedback.identity = variant.identity;
    log.check(variant.identity != genuine1.identity
              && canonical_identity_bytes(variant.identity)
                  != canonical_identity_bytes(genuine1.identity)
              && canonical_feedback_bytes(variant_feedback)
                  != canonical_feedback_bytes(genuine_feedback),
              "IDENTITY-COVERAGE", variant.field);
  }

  Receiver newer_session(forward, 7002, [](const CommitView&) {
    return true;
  });
  log.check(newer_session.publish_once(fwd, future_ms(5000))
                == PublishResult::INVALID
            && !newer_session.pinned(),
            "REFUTE3", "prior-session offer cannot establish a new receiver");

  std::vector<std::uint8_t> app_sink;
  int sink_calls = 0;
  Receiver receiver(forward, 7001, [&](const CommitView& view) {
    ++sink_calls;
    app_sink.insert(app_sink.end(), view.application_bytes,
                    view.application_bytes + view.application_size);
    return true;
  });

  SourceRetainer source(forward.sender, fwd,
                        Protection::AUTHENTICATED_FRAME);
  const std::vector<std::uint8_t> block0 = bytes(7, 24);
  const std::vector<std::uint8_t> block1 = bytes(0x91, 24);
  DataRecord data0;
  DataRecord data1;
  log.check(source.valid()
            && source.stage(block0.data(), block0.size(), block0.data(),
                            block0.size(), &data0)
            && source.stage(block1.data(), block1.size(), block1.data(),
                            block1.size(), &data1),
            "RETENTION", "two source batches staged with full ownership");

  Feedback receipt;
  Feedback discard;
  log.check(admit_verified(&receiver, data0, Verification::FRAME_VERIFIED,
                           &receipt)
                == AdmitResult::REFUSED
            && receiver.stored_count() == 0,
            "REFUTE3", "pre-publication DATA refused before storage");

  const Receiver::Clock::time_point main_deadline = future_ms(5000);
  log.check(receiver.publish_once(fwd, main_deadline)
                == PublishResult::PUBLISHED,
            "LIFECYCLE", "one live descriptor published");
  log.check(receiver.publish_once(fwd, main_deadline)
                == PublishResult::EXACT_DUPLICATE,
            "LIFECYCLE", "exact control retry is idempotent");
  log.check(receiver.publish_once(rev, main_deadline)
                == PublishResult::INVALID
            && receiver.pinned(),
            "REFUTE3", "conflicting stale offer cannot repin live receiver");

  DataRecord wrong = data0;
  wrong.identity.stream = rev;
  const std::size_t before_bad = receiver.stored_count();
  log.check(admit_verified(&receiver, wrong, Verification::FRAME_VERIFIED,
                           &discard)
                == AdmitResult::REFUSED
            && receiver.stored_count() == before_bad,
            "ADMISSION", "wrong origin/address rejected before storage");
  log.check(admit_verified(&receiver, data0, Verification::AEAD_VERIFIED,
                           &discard)
                == AdmitResult::REFUSED
            && receiver.stored_count() == before_bad,
            "ADMISSION", "verification mode cannot be relabelled");
  wrong = data0;
  wrong.transported[0] ^= 0x80;
  log.check(admit_verified(&receiver, wrong, Verification::FRAME_VERIFIED,
                           &discard)
                == AdmitResult::REFUSED
            && receiver.stored_count() == before_bad,
            "ADMISSION", "sender digest mismatch rejected before storage");
  wrong = data0;
  wrong.identity.generation = 1000;
  log.check(admit_verified(&receiver, wrong, Verification::FRAME_VERIFIED,
                           &discard)
                == AdmitResult::REFUSED
            && receiver.stored_count() == before_bad,
            "REFUTE5", "fast/out-of-window path cannot bypass generation gate");

  Feedback receipt1;
  log.check(admit_verified(&receiver, data1, Verification::FRAME_VERIFIED,
                           &receipt1)
                == AdmitResult::ACCEPTED
            && receipt1.kind == FeedbackKind::RECEIPT
            && receipt1.identity == data1.identity,
            "INTEGRITY24", "successor may be received but keeps sender identity");
  log.check(apply_verified(&source, receipt1, Verification::FRAME_VERIFIED)
            && source.retained_count() == 2
            && source.retained_source_bytes() == 48,
            "RETENTION", "RECEIPT does not release either source batch");
  CommitTicket early;
  log.check(!receiver.prepare_commit(data1.identity, &early)
            && app_sink.empty() && sink_calls == 0,
            "INTEGRITY24", "missing 24-byte generation blocks successor commit");

  Feedback receipt0;
  log.check(admit_verified(&receiver, data0, Verification::FRAME_VERIFIED,
                           &receipt0)
                == AdmitResult::ACCEPTED
            && apply_verified(&source, receipt0, Verification::FRAME_VERIFIED)
            && source.retained_count() == 2,
            "REFUTE1", "late predecessor received without releasing source");
  CommitTicket ticket0;
  CommitTicket duplicate_ticket0;
  Feedback commit0;
  log.check(receiver.prepare_commit(data0.identity, &ticket0)
            && receiver.prepare_commit(data0.identity, &duplicate_ticket0),
            "SINK", "tickets bind one ordered generation and lifetime");
  log.check(receiver.commit(std::move(ticket0), block0.data(), block0.size(),
                            &commit0)
            && commit0.kind == FeedbackKind::COMMIT
            && commit0.identity == data0.identity
            && commit0.committed_application_bytes == 24,
            "COMMIT", "sink acceptance creates distinct COMMIT feedback");
  Feedback replayed_commit0;
  log.check(admit_verified(&receiver, data0, Verification::FRAME_VERIFIED,
                           &replayed_commit0)
                == AdmitResult::EXACT_DUPLICATE
            && replayed_commit0.kind == FeedbackKind::COMMIT
            && replayed_commit0.identity == data0.identity
            && replayed_commit0.committed_application_bytes == block0.size()
            && sink_calls == 1,
            "REFUTE5", "lost COMMIT is replayed without a second sink append");
  Feedback unused;
  log.check(!receiver.commit(std::move(duplicate_ticket0), block0.data(),
                             block0.size(), &unused)
            && sink_calls == 1,
            "SINK", "second consumer cannot append the committed generation");
  log.check(apply_verified(&source, commit0, Verification::FRAME_VERIFIED)
            && apply_verified(&source, replayed_commit0,
                              Verification::FRAME_VERIFIED)
            && source.retained_count() == 1
            && !source.retained(data0.identity.generation),
            "RETENTION", "matching and duplicate COMMIT are idempotent release");

  CommitTicket ticket1;
  Feedback commit1;
  Feedback wrong_length_commit;
  wrong_length_commit.kind = FeedbackKind::COMMIT;
  wrong_length_commit.identity = data1.identity;
  wrong_length_commit.transported_digest = data1.transported_digest;
  wrong_length_commit.committed_application_bytes = block1.size() - 1;
  log.check(!apply_verified(&source, wrong_length_commit,
                            Verification::FRAME_VERIFIED)
            && source.retained_count() == 1,
            "RETENTION", "COMMIT length must match retained source authority");
  std::vector<std::uint8_t> corrupt_application = block1;
  corrupt_application[0] ^= 0x40;
  CommitTicket corrupt_ticket;
  log.check(receiver.prepare_commit(data1.identity, &corrupt_ticket)
            && !receiver.commit(std::move(corrupt_ticket),
                                corrupt_application.data(),
                                corrupt_application.size(), nullptr)
            && receiver.stored_count() == 1
            && receiver.next_generation() == data1.identity.generation
            && sink_calls == 1,
            "COMMIT", "post-decompression bytes must match sender commitment");
  log.check(receiver.prepare_commit(data1.identity, &ticket1)
            && receiver.commit(std::move(ticket1), block1.data(), block1.size(),
                               &commit1)
            && apply_verified(&source, commit1, Verification::FRAME_VERIFIED)
            && source.retained_count() == 0,
            "INTEGRITY24", "predecessor then successor commit byte-exactly");
  std::vector<std::uint8_t> expected = block0;
  expected.insert(expected.end(), block1.begin(), block1.end());
  log.check(app_sink == expected && sink_calls == 2,
            "INTEGRITY24", "ordered sink contains all 48 bytes with no deletion");
  log.check(canonical_feedback_bytes(receipt0)
                != canonical_feedback_bytes(commit0),
            "FEEDBACK", "RECEIPT and COMMIT have distinct authenticated encodings");

  // A commit acknowledgement with any relabelled address is powerless.
  SourceRetainer address_source(forward.sender, fwd,
                                Protection::AUTHENTICATED_FRAME);
  DataRecord address_record;
  address_source.stage(block0.data(), block0.size(), block0.data(),
                       block0.size(), &address_record);
  Feedback relabelled;
  relabelled.kind = FeedbackKind::COMMIT;
  relabelled.identity = address_record.identity;
  relabelled.identity.stream = rev;
  relabelled.transported_digest = address_record.transported_digest;
  relabelled.committed_application_bytes = block0.size();
  log.check(!apply_verified(&address_source, relabelled,
                            Verification::FRAME_VERIFIED)
            && address_source.retained_count() == 1,
            "ADDRESS", "feedback cannot erase source after address relabelling");
  log.check(canonical_identity_bytes(address_record.identity)
                != canonical_identity_bytes(relabelled.identity),
            "ADDRESS", "canonical binding preserves endpoints and direction");

  // Reconnect is an explicit new origin/instance; old feedback cannot cross it.
  StreamDescriptor reconnect;
  log.check(make_descriptor(&factory, forward, 7002, 7, &reconnect)
            && address_source.rebind(reconnect),
            "REFUTE2", "retained source explicitly rebound to a fresh reconnect origin");
  DataRecord rebound;
  log.check(address_source.record(7, &rebound)
            && rebound.identity.stream == reconnect
            && rebound.identity.stream.origin() != fwd.origin(),
            "REFUTE2", "rebound DATA carries new sender-owned instance");
  Feedback stale_feedback;
  stale_feedback.kind = FeedbackKind::COMMIT;
  stale_feedback.identity = address_record.identity;
  stale_feedback.transported_digest = address_record.transported_digest;
  log.check(!apply_verified(&address_source, stale_feedback,
                            Verification::FRAME_VERIFIED)
            && address_source.retained_count() == 1,
            "REFUTE2", "prior-session COMMIT cannot release rebound source");

  // A ticket minted before reset is revoked under the same serialization lock.
  std::vector<std::uint8_t> revoked_sink;
  Receiver revoked_receiver(forward, 7002, [&](const CommitView& view) {
    revoked_sink.insert(revoked_sink.end(), view.application_bytes,
                        view.application_bytes + view.application_size);
    return true;
  });
  revoked_receiver.publish_once(reconnect, future_ms(5000));
  Feedback rebound_receipt;
  admit_verified(&revoked_receiver, rebound, Verification::FRAME_VERIFIED,
                 &rebound_receipt);
  CommitTicket revoked_ticket;
  const bool ticket_minted =
      revoked_receiver.prepare_commit(rebound.identity, &revoked_ticket);
  revoked_receiver.revoke();
  Feedback should_not_commit;
  log.check(ticket_minted
            && !revoked_ticket.valid()
            && !revoked_receiver.commit(std::move(revoked_ticket),
                                        block0.data(), block0.size(),
                                        &should_not_commit)
            && revoked_sink.empty() && revoked_receiver.stored_count() == 0,
            "REVOCATION", "revoke invalidates ticket, storage, and sink consumer");

  // Revoke serializes with an already-running append. Once revoke returns,
  // the sole consumer has finished and cannot be orphaned in the background.
  std::atomic<bool> sink_entered(false);
  std::atomic<bool> release_sink(false);
  std::atomic<bool> revoke_returned(false);
  bool concurrent_commit = false;
  Receiver serial_receiver(forward, 7002, [&](const CommitView&) {
    sink_entered.store(true);
    while(!release_sink.load()) std::this_thread::yield();
    return true;
  });
  serial_receiver.publish_once(reconnect, future_ms(5000));
  admit_verified(&serial_receiver, rebound, Verification::FRAME_VERIFIED,
                 &discard);
  CommitTicket serial_ticket;
  serial_receiver.prepare_commit(rebound.identity, &serial_ticket);
  std::thread commit_thread([&] {
    concurrent_commit = serial_receiver.commit(
        std::move(serial_ticket), block0.data(), block0.size(), nullptr);
  });
  for(int i = 0; i < 1000 && !sink_entered.load(); ++i)
    std::this_thread::yield();
  std::thread revoke_thread([&] {
    serial_receiver.revoke();
    revoke_returned.store(true);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  const bool revoke_waited = sink_entered.load() && !revoke_returned.load();
  release_sink.store(true);
  commit_thread.join();
  revoke_thread.join();
  log.check(revoke_waited && concurrent_commit && revoke_returned.load()
            && serial_receiver.revoked(),
            "REVOCATION", "revoke return is a complete consumer barrier");

  // Sink failure is not commit and leaves the receiver-owned record retryable.
  int failed_sink_calls = 0;
  Receiver backpressure(forward, 7001, [&](const CommitView&) {
    ++failed_sink_calls;
    return false;
  });
  backpressure.publish_once(fwd, future_ms(5000));
  admit_verified(&backpressure, data0, Verification::FRAME_VERIFIED, &discard);
  CommitTicket backpressure_ticket;
  backpressure.prepare_commit(data0.identity, &backpressure_ticket);
  log.check(!backpressure.commit(std::move(backpressure_ticket), block0.data(),
                                 block0.size(), nullptr)
            && failed_sink_calls == 1 && backpressure.stored_count() == 1
            && backpressure.next_generation() == 0,
            "COMMIT", "sink refusal creates no commit and advances no cursor");

  // Deadline is active: duplicate progress cannot re-arm it, and no later API
  // call is required to perform the expiry/revocation.
  std::atomic<int> deadline_sink_calls(0);
  Receiver deadline_receiver(forward, 7001, [&](const CommitView&) {
    ++deadline_sink_calls;
    return true;
  });
  const Receiver::Clock::time_point absolute = future_ms(90);
  deadline_receiver.publish_once(fwd, absolute);
  Feedback deadline_receipt;
  const AdmitResult first_deadline_admit = admit_verified(
      &deadline_receiver, data0, Verification::FRAME_VERIFIED,
      &deadline_receipt);
  bool duplicate_progress = first_deadline_admit == AdmitResult::ACCEPTED;
  for(int i = 0; i < 5; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const AdmitResult result = admit_verified(
        &deadline_receiver, data0, Verification::FRAME_VERIFIED, &discard);
    duplicate_progress = duplicate_progress
        && (result == AdmitResult::EXACT_DUPLICATE
            || result == AdmitResult::REFUSED);
  }
  const bool unchanged_deadline = deadline_receiver.deadline() == absolute;
  for(int i = 0; i < 100 && !deadline_receiver.expired(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  log.check(duplicate_progress && unchanged_deadline
            && deadline_receiver.expired() && deadline_receiver.revoked()
            && deadline_receiver.stored_count() == 0
            && deadline_sink_calls.load() == 0,
            "REFUTE4", "active absolute deadline expires despite progress drip");

  // No modulo alias exists at the end of the full generation space.
  StreamDescriptor near_wrap;
  const std::uint64_t penultimate =
      std::numeric_limits<std::uint64_t>::max() - 1u;
  log.check(make_descriptor(&factory, forward, 7003, penultimate, &near_wrap),
            "GENERATION", "full-width near-wrap descriptor created");
  SourceRetainer wrap_source(forward.sender, near_wrap,
                             Protection::AEAD_COVERED);
  DataRecord wrap0;
  DataRecord wrap1;
  DataRecord wrap2;
  const bool two_staged =
      wrap_source.stage(block0.data(), block0.size(), block0.data(),
                        block0.size(), &wrap0)
      && wrap_source.stage(block1.data(), block1.size(), block1.data(),
                           block1.size(), &wrap1);
  const bool third_refused =
      !wrap_source.stage(block0.data(), block0.size(), block0.data(),
                         block0.size(), &wrap2);
  log.check(two_staged && third_refused
            && wrap0.identity.generation == penultimate
            && wrap1.identity.generation
                == std::numeric_limits<std::uint64_t>::max(),
            "GENERATION", "generation exhaustion refuses rather than wrapping");

  // Exact duplicate DATA yields another receipt only; it never appends.
  std::vector<std::uint8_t> duplicate_sink;
  Receiver duplicate_receiver(forward, 7003, [&](const CommitView& view) {
    duplicate_sink.insert(duplicate_sink.end(), view.application_bytes,
                          view.application_bytes + view.application_size);
    return true;
  });
  duplicate_receiver.publish_once(near_wrap, future_ms(5000));
  Feedback wrap_receipt_a;
  Feedback wrap_receipt_b;
  log.check(admit_verified(&duplicate_receiver, wrap0,
                           Verification::AEAD_VERIFIED, &wrap_receipt_a)
                == AdmitResult::ACCEPTED
            && admit_verified(&duplicate_receiver, wrap0,
                              Verification::AEAD_VERIFIED, &wrap_receipt_b)
                == AdmitResult::EXACT_DUPLICATE
            && duplicate_sink.empty()
            && wrap_receipt_a.kind == FeedbackKind::RECEIPT
            && wrap_receipt_b.kind == FeedbackKind::RECEIPT,
            "FEEDBACK", "duplicate receipt is idempotent and not a commit");

  // The admission boundary is object-bound.  The attestation minted at the
  // wire-authentication boundary is the only admissible witness, it binds
  // every authenticated byte, it is spent by first use, and a RECEIPT output
  // is mandatory.  Each refusal below reproduces one previously accepted
  // misuse of the old bare-enum boundary.
  StreamDescriptor bound_stream;
  log.check(make_descriptor(&factory, forward, 7004, 0, &bound_stream),
            "APIBOUND", "bound stream descriptor created");
  std::vector<std::uint8_t> bound_sink;
  Receiver bound_receiver(forward, 7004, [&](const CommitView& view) {
    bound_sink.insert(bound_sink.end(), view.application_bytes,
                      view.application_bytes + view.application_size);
    return true;
  });
  log.check(bound_receiver.publish_once(bound_stream, future_ms(5000))
                == PublishResult::PUBLISHED,
            "APIBOUND", "bound receiver published");
  SourceRetainer bound_source(forward.sender, bound_stream,
                              Protection::AUTHENTICATED_FRAME);
  DataRecord bound0;
  log.check(bound_source.stage(block0.data(), block0.size(), block0.data(),
                               block0.size(), &bound0),
            "APIBOUND", "bound record staged");

  log.check(bound_receiver.admit(
                bound0,
                WireAttestation::attest_data(bound0,
                                             Verification::FRAME_VERIFIED),
                nullptr) == AdmitResult::REFUSED
            && bound_receiver.stored_count() == 0,
            "APIBOUND", "null receipt output refused before storage");

  WireAttestation attested_before_mutation =
      WireAttestation::attest_data(bound0, Verification::FRAME_VERIFIED);
  DataRecord moved_generation = bound0;
  moved_generation.identity.generation += 1;
  Feedback bound_scratch;
  log.check(bound_receiver.admit(moved_generation,
                                 std::move(attested_before_mutation),
                                 &bound_scratch) == AdmitResult::REFUSED
            && bound_receiver.stored_count() == 0,
            "APIBOUND", "record mutated after attestation refused before storage");

  WireAttestation attested_frame =
      WireAttestation::attest_data(bound0, Verification::FRAME_VERIFIED);
  DataRecord relabelled_protection = bound0;
  relabelled_protection.protection = Protection::AEAD_COVERED;
  log.check(bound_receiver.admit(relabelled_protection,
                                 std::move(attested_frame),
                                 &bound_scratch) == AdmitResult::REFUSED
            && bound_receiver.stored_count() == 0,
            "APIBOUND", "protection relabel after attestation refused");

  WireAttestation absent;
  log.check(bound_receiver.admit(bound0, std::move(absent), &bound_scratch)
                == AdmitResult::REFUSED
            && bound_receiver.stored_count() == 0,
            "APIBOUND", "an absent attestation admits nothing");

  Feedback bound_receipt;
  log.check(admit_verified(&bound_receiver, bound0,
                           Verification::FRAME_VERIFIED, &bound_receipt)
                == AdmitResult::ACCEPTED
            && bound_receipt.kind == FeedbackKind::RECEIPT,
            "APIBOUND", "attested admission still yields a receipt");
  log.check(!WireAttestation::attest_data(bound0,
                                          Verification::FAILED).valid()
            && !WireAttestation::attest_feedback(
                    bound_receipt, Verification::FAILED).valid(),
            "APIBOUND", "a failed verification mints no attestation");
  log.check(apply_verified(&bound_source, bound_receipt,
                           Verification::FRAME_VERIFIED)
            && bound_source.retained_count() == 1,
            "APIBOUND", "attested receipt is applied and retains source");

  // The S1b receipt-promotion reproduction: the attestation was minted over
  // the RECEIPT bytes, then the feedback is relabelled to COMMIT afterwards.
  // The relabelled bytes no longer match the attested bytes, so the retained
  // source survives.
  WireAttestation receipt_attestation = WireAttestation::attest_feedback(
      bound_receipt, Verification::FRAME_VERIFIED);
  Feedback promoted = bound_receipt;
  promoted.kind = FeedbackKind::COMMIT;
  promoted.committed_application_bytes = block0.size();
  log.check(!bound_source.apply_feedback(promoted,
                                         std::move(receipt_attestation))
            && bound_source.retained_count() == 1,
            "APIBOUND", "receipt relabelled to commit cannot release source");

  WireAttestation single_use =
      WireAttestation::attest_data(bound0, Verification::FRAME_VERIFIED);
  log.check(bound_receiver.admit(bound0, std::move(single_use), &bound_scratch)
                == AdmitResult::EXACT_DUPLICATE
            && bound_scratch.kind == FeedbackKind::RECEIPT,
            "APIBOUND", "fresh attestation covers an exact duplicate");
  // Deliberately present the spent witness a second time.
  log.check(bound_receiver.admit(bound0, std::move(single_use), &bound_scratch)
                == AdmitResult::REFUSED,
            "APIBOUND", "an attestation is spent by its first use");

  CommitTicket bound_ticket;
  Feedback bound_commit;
  log.check(bound_receiver.prepare_commit(bound0.identity, &bound_ticket)
            && bound_receiver.commit(std::move(bound_ticket), block0.data(),
                                     block0.size(), &bound_commit)
            && bound_commit.kind == FeedbackKind::COMMIT
            && apply_verified(&bound_source, bound_commit,
                              Verification::FRAME_VERIFIED)
            && bound_source.retained_count() == 0
            && bound_sink == block0,
            "APIBOUND", "attested commit round trip releases source");

  // CI replay of the frozen six-run clean-WGN class.  The observed cohort used
  // 18 timed RF batches per 262144-byte transfer and had mean CONNECT-to-settle
  // occupancy 349.56232966449915 s, of which 319.033 s was forward emission.
  // The core operates at batch ownership boundaries; it must preserve all six
  // byte-exact completions without splitting or otherwise changing RF batches.
  int wgn_completions = 0;
  bool wgn_batch_geometry = true;
  const std::size_t wgn_payload_bytes = 262144;
  const int wgn_batches = 18;
  for(int sample = 0; sample < 6; ++sample) {
    const std::vector<std::uint8_t> payload = bytes(
        static_cast<std::uint8_t>(31 + sample * 19), wgn_payload_bytes);
    std::vector<std::uint8_t> delivered;
    int delivered_batches = 0;
    Receiver wgn_receiver(forward, 8000 + sample,
                          [&](const CommitView& view) {
                            ++delivered_batches;
                            delivered.insert(
                                delivered.end(), view.application_bytes,
                                view.application_bytes + view.application_size);
                            return true;
                          }, wgn_batches);
    StreamDescriptor wgn_stream;
    bool complete = make_descriptor(&factory, forward, 8000 + sample, 0,
                                    &wgn_stream)
        && wgn_receiver.publish_once(wgn_stream, future_ms(5000))
               == PublishResult::PUBLISHED;
    SourceRetainer wgn_source(forward.sender, wgn_stream,
                              Protection::AUTHENTICATED_FRAME);
    std::size_t offset = 0;
    for(int batch = 0; complete && batch < wgn_batches; ++batch) {
      const std::size_t remaining = payload.size() - offset;
      const std::size_t chunks_left =
          static_cast<std::size_t>(wgn_batches - batch);
      const std::size_t chunk = (remaining + chunks_left - 1) / chunks_left;
      DataRecord record;
      Feedback got;
      Feedback put;
      CommitTicket ticket;
      complete = wgn_source.stage(payload.data() + offset, chunk,
                                  payload.data() + offset, chunk, &record)
          && admit_verified(&wgn_receiver, record,
                            Verification::FRAME_VERIFIED, &got)
                 == AdmitResult::ACCEPTED
          && got.kind == FeedbackKind::RECEIPT
          && apply_verified(&wgn_source, got, Verification::FRAME_VERIFIED)
          && wgn_receiver.prepare_commit(record.identity, &ticket)
          && wgn_receiver.commit(std::move(ticket), payload.data() + offset,
                                 chunk, &put)
          && put.kind == FeedbackKind::COMMIT
          && apply_verified(&wgn_source, put, Verification::FRAME_VERIFIED);
      offset += chunk;
    }
    complete = complete && offset == payload.size() && delivered == payload
        && delivered_batches == wgn_batches
        && wgn_source.retained_count() == 0;
    if(complete) ++wgn_completions;
    wgn_batch_geometry = wgn_batch_geometry
        && delivered_batches == wgn_batches;
  }
  const double frozen_forward_occupancy = 319.033 / 349.56232966449915;
  log.check(wgn_completions == 6 && wgn_batch_geometry
            && frozen_forward_occupancy > 0.91
            && frozen_forward_occupancy < 0.915,
            "WGN-GATE", "six clean transfers preserve timed RF batch boundaries");
  std::printf("[TEST-FADE-CORE/WGN-GATE] completions=%d/6 "
              "forward_occupancy=%.3f%% batches_per_transfer=%d\n",
              wgn_completions, 100.0 * frozen_forward_occupancy, wgn_batches);

  std::printf("[TEST-FADE-CORE] %s failures=%d suites=6 integrity24=%s\n",
              log.failures ? "FAIL" : "PASS", log.failures,
              log.failures ? "FAIL" : "PASS");
  return log.failures ? 1 : 0;
}

}  // namespace fade
}  // namespace mercury

#ifdef MERCURY_FADE_CORE_STANDALONE
int main() { return mercury::fade::run_fade_core_tests(); }
#endif
