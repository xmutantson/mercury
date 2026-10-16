#include "datalink_layer/l1_tx_journal.h"

#include <cstdio>
#include <cstdlib>

#ifdef _WIN32
#include <direct.h>
// MSVCRT never declares the POSIX env setters (same shim as arq_common.cc).
static int setenv(const char* name, const char* value, int overwrite) {
	if(!overwrite && getenv(name) != NULL) return 0;
	return _putenv_s(name, value);
}
static int unsetenv(const char* name) { return _putenv_s(name, ""); }
#endif
#include <string>
#include <unistd.h>
#include <sys/stat.h>

using mercury::L1JournalState;
using mercury::L1ResetEvent;
using mercury::L1TerminalQueue;
using mercury::L1TerminalReport;
using mercury::L1TxJournal;

static int failures = 0;

static void check(bool value, const char* row) {
  std::printf("[L1-ROW] %s %s\n", value ? "PASS" : "FAIL", row);
  if (!value) ++failures;
}

static void set_env(const char* name, const char* value) {
  setenv(name, value, 1);
}

// Run in a caller-owned isolated directory. The runner supplies unset/empty
// state settings and keyed sentinels before tracing this process, so fixture
// filesystem operations cannot be mistaken for journal operations.
static int implicit_marker_case(bool disabled) {
  L1TerminalQueue first_terminal, second_terminal;
  L1TxJournal first(&first_terminal);
  first.recover_restart_marker();
  first.begin_session(61, 0x6151);
  const char first_bytes[] = {'a', '\0', 'k', '1'};
  const char second_bytes[] = {'b', '\0', 'k', '2'};
  if (disabled) {
    check(!first.enabled() &&
              !first.stage(31, 3, 0, 1, first_bytes, sizeof(first_bytes), 14) &&
              first.size() == 0 && first_terminal.size() == 0,
          "implicit-disabled-control");
    return failures ? 1 : 0;
  }
  check(first.enabled() && first_terminal.size() == 0,
        "implicit-no-prior-loss-first");
  check(first.stage(31, 3, 0, 1, first_bytes, sizeof(first_bytes), 14),
        "implicit-first-stage");
  L1TxJournal second(&second_terminal);
  second.recover_restart_marker();
  second.begin_session(62, 0x6252);
  check(second_terminal.size() == 0, "implicit-no-prior-loss-second");
  check(second.stage(41, 4, 0, 1, second_bytes, sizeof(second_bytes), 13),
        "implicit-second-stage");
  const mercury::L1JournalKey first_key = {first.session_id(), first.epoch(), 31, 3};
  const mercury::L1JournalKey second_key = {second.session_id(), second.epoch(), 41, 4};
  check(first.size() == 1 && second.size() == 1 &&
            first.entries()[0].key == first_key &&
            second.entries()[0].key == second_key &&
            first.entries()[0].plaintext == std::vector<char>(first_bytes, first_bytes + sizeof(first_bytes)) &&
            second.entries()[0].plaintext == std::vector<char>(second_bytes, second_bytes + sizeof(second_bytes)),
        "implicit-independent-keyed-bytes");
  check(first.mark_sent_many({{31, 3}}) && second.mark_sent_many({{41, 4}}),
        "implicit-mark-sent");
  const L1ResetEvent retained[] = {
      L1ResetEvent::ACK_TIMEOUT, L1ResetEvent::LOCAL_BREAK,
      L1ResetEvent::PEER_BREAK, L1ResetEvent::COLLISION,
      L1ResetEvent::BREAK_RECOVERY_TIMEOUT,
      L1ResetEvent::AUTHENTICATED_RECONNECT, L1ResetEvent::SOFT_RESET,
      L1ResetEvent::RESPONDER_RESET, L1ResetEvent::CONFIG_CHANGE,
      L1ResetEvent::CRYPTO_REKEY, L1ResetEvent::BSI_WRAP,
      L1ResetEvent::QUEUE_FLUSH};
  for (L1ResetEvent event : retained) {
    first.apply(event);
    check(first.size() == 1 && first.entries()[0].key == first_key &&
              first.entries()[0].plaintext == std::vector<char>(first_bytes, first_bytes + sizeof(first_bytes)) &&
              first_terminal.size() == 0,
          mercury::l1_reset_event_name(event));
  }
  check(first.migrate_authenticated(63, 0x6151) && first.size() == 1 &&
            first.entries()[0].key.session_id == first.session_id() &&
            first.entries()[0].key.epoch == 1 && first.entries()[0].key.bsi == 0 &&
            first.entries()[0].key.slot == 0 &&
            first.entries()[0].plaintext == std::vector<char>(first_bytes, first_bytes + sizeof(first_bytes)),
        "implicit-authenticated-migration");
  const mercury::L1JournalKey migrated_key = first.entries()[0].key;
  const std::vector<char> queued = {'q', '\0', 't'};
  L1TerminalReport report;
  check(first.terminalize("implicit-terminal", queued) && first.size() == 0 &&
            first_terminal.take(&report) && first_terminal.size() == 0 &&
            !report.delivery_claimed && !report.prior_process_loss &&
            report.reason == "implicit-terminal" && report.entries.size() == 1 &&
            report.entries[0].key == migrated_key &&
            report.entries[0].plaintext == std::vector<char>(first_bytes, first_bytes + sizeof(first_bytes)) &&
            report.queued_plaintext == queued,
        "implicit-terminal-keyed-receipt");
  check(second.acknowledge(41, 4) && second.size() == 0 && second_terminal.size() == 0,
        "implicit-second-ack-independent");
  std::printf("[L1-IMPLICIT-TEST] %s failures=%d\n", failures ? "FAIL" : "PASS", failures);
  return failures ? 1 : 0;
}

static void explicit_marker_cases(const std::string& marker) {
  L1TerminalQueue left_terminal, right_terminal;
  const std::string other = marker + ".other";
  const char bytes[] = {'x', '\0', 'e', '1'};
  set_env("MERCURY_L1_JOURNAL_STATE", marker.c_str());
  uint64_t lost_session;
  uint32_t lost_epoch;
  {
    L1TxJournal left(&left_terminal);
    left.begin_session(71);
    lost_session = left.session_id();
    lost_epoch = left.epoch();
    check(left.stage(9, 5, 0, 1, bytes, sizeof(bytes), 14), "explicit-left-stage");
    set_env("MERCURY_L1_JOURNAL_STATE", other.c_str());
    L1TxJournal right(&right_terminal);
    right.begin_session(72);
    check(right_terminal.size() == 0 && left_terminal.size() == 0 &&
              right.stage(19, 6, 0, 1, bytes, sizeof(bytes), 13) &&
              right.mark_sent_many({{19, 6}}) && right.acknowledge(19, 6) && left.size() == 1 &&
              left.entries()[0].key.bsi == 9 && left.entries()[0].key.slot == 5 &&
              left.entries()[0].plaintext == std::vector<char>(bytes, bytes + sizeof(bytes)),
          "explicit-distinct-path-isolation");
  }
  set_env("MERCURY_L1_JOURNAL_STATE", marker.c_str());
  L1TerminalReport report;
  L1TxJournal restart(&left_terminal);
  check(left_terminal.take(&report) && report.session_id == lost_session &&
            report.epoch == lost_epoch && report.prior_process_loss &&
            !report.delivery_claimed && report.entries.empty() && report.queued_plaintext.empty(),
        "explicit-keyed-restart-loss");
  restart.recover_restart_marker();
  L1TxJournal restart_again(&left_terminal);
  check(left_terminal.size() == 0, "explicit-restart-exactly-once");

  // A directory at the temporary-file path forces the explicit write to fail.
  // Keep a retained entry first to check failure preserves its full authority.
  L1TxJournal failure(&left_terminal);
  failure.begin_session(73);
  check(failure.stage(29, 7, 0, 1, bytes, sizeof(bytes), 12) &&
            failure.mark_sent_many({{29, 7}}), "explicit-rollback-precondition");
  const mercury::L1JournalEntry before = failure.entries()[0];
  const std::string blocked = marker + ".tmp";
  std::remove(blocked.c_str());
  #ifdef _WIN32
  check(::_mkdir(blocked.c_str()) == 0, "explicit-block-write-fixture");
#else
  check(::mkdir(blocked.c_str(), 0700) == 0, "explicit-block-write-fixture");
#endif
  check(!failure.stage(30, 8, 0, 1, bytes, sizeof(bytes), 11) && failure.size() == 1 &&
            failure.entries()[0].key == before.key &&
            failure.entries()[0].plaintext == before.plaintext &&
            failure.entries()[0].block_serial == before.block_serial &&
            failure.entries()[0].ordered_tx_seq == before.ordered_tx_seq,
        "explicit-write-failure-preserves-owner");
  check(::rmdir(blocked.c_str()) == 0 && failure.acknowledge(29, 7),
        "explicit-write-failure-recovery");
  std::remove(other.c_str());
  set_env("MERCURY_L1_JOURNAL_STATE", marker.c_str());
}

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--implicit-marker")
    return implicit_marker_case(false);
  if (argc == 2 && std::string(argv[1]) == "--disabled-marker")
    return implicit_marker_case(true);
  char marker[160];
  std::snprintf(marker, sizeof(marker), "/tmp/mercury_l1_stage2_%ld.pending",
                (long)getpid());
  std::remove(marker);
  std::remove((std::string(marker) + ".tmp").c_str());
  set_env("MERCURY_L1_JOURNAL", "1");
  set_env("MERCURY_L1_JOURNAL_STATE", marker);

  L1TerminalQueue terminal;
  L1TerminalReport report;
  L1TxJournal journal(&terminal);
  journal.begin_session(37);
  const char payload[] = "owner";
  check(journal.stage(219, 3, 0, 1, payload, sizeof(payload), 16),
        "insert-before-tx");
  journal.mark_sent(219, 3);
  check(!journal.acknowledge(220, 3) && journal.size() == 1,
        "transmitted-bsi-ack-identity");

  const L1ResetEvent retained[] = {
      L1ResetEvent::ACK_TIMEOUT, L1ResetEvent::LOCAL_BREAK,
      L1ResetEvent::PEER_BREAK, L1ResetEvent::COLLISION,
      L1ResetEvent::BREAK_RECOVERY_TIMEOUT,
      L1ResetEvent::AUTHENTICATED_RECONNECT, L1ResetEvent::SOFT_RESET,
      L1ResetEvent::RESPONDER_RESET, L1ResetEvent::CONFIG_CHANGE,
      L1ResetEvent::CRYPTO_REKEY, L1ResetEvent::BSI_WRAP,
      L1ResetEvent::QUEUE_FLUSH};
  for (L1ResetEvent event : retained) {
    const std::size_t before = journal.size();
    journal.apply(event);
    check(journal.size() == before, mercury::l1_reset_event_name(event));
  }

  // A recovery changes the real journal authority epoch; re-stage binds the
  // retained owner to the actual next wire identity without duplicating it.
  const uint32_t recovery_epoch = journal.epoch();
  check(recovery_epoch > 1, "real-epoch-authority");
  check(journal.stage(220, 3, 0, 1, payload, sizeof(payload), 15) &&
            journal.size() == 1,
        "retained-restage-single-owner");
  journal.mark_sent(220, 3);
  check(journal.acknowledge(220, 3) && journal.size() == 0,
        "positive-ack-chokepoint");
  check(journal.acknowledge(220, 3) && journal.size() == 0,
        "positive-reack-idempotent");

  journal.begin_session(40, 0xfeedbeefULL);
  mercury::L1StageItem a, b;
  a.slot = 0; a.batch_index = 0; a.span = 2; a.plaintext = {'a'}; a.configuration = 12;
  b.slot = 1; b.batch_index = 1; b.span = 2; b.plaintext = {'b'}; b.configuration = 12;
  check(journal.stage_batch(17, {a, b}) && journal.size() == 2 &&
            journal.entries()[0].block_serial == journal.entries()[1].block_serial,
        "atomic-batch-stage-block-serial");
  check(journal.mark_sent_many({{17, 0}, {17, 1}}), "atomic-batch-mark-sent");
  check(!journal.acknowledge_many({{17, 0}, {17, 99}}) && journal.size() == 2,
        "atomic-unknown-ack-refusal");
  check(!journal.migrate_authenticated(41, 0xbadULL) && journal.size() == 2,
        "reconnect-transfer-id-refused");
  check(journal.migrate_authenticated(41, 0xfeedbeefULL) &&
            journal.size() == 2 && journal.epoch() == 1 &&
            journal.entries()[0].key.bsi == 0,
        "reconnect-transfer-id-validated-n1");
  check(journal.terminalize("migration-test-complete", {}) && terminal.take(&report),
        "migration-test-terminal-cleanup");

  journal.begin_session(42);
  std::vector<mercury::L1StageItem> too_many(97, a);
  for(std::size_t i=0;i<too_many.size();i++) too_many[i].slot = (uint16_t)i;
  check(!journal.stage_batch(2, too_many) && journal.size() == 0,
        "capacity-refuses-without-eviction");

  check(journal.stage(221, 4, 0, 1, payload, sizeof(payload), 15),
        "teardown-precondition");
  journal.mark_sent(221, 4);
  std::vector<char> queued = {'q', 'u', 'e', 'u', 'e', 'd'};
  check(journal.terminalize("teardown-cancel", queued) &&
            journal.size() == 0 && terminal.size() == 1,
        "teardown-cancel");
  check(terminal.take(&report) && !report.delivery_claimed &&
            report.entries.size() == 1 && report.queued_plaintext == queued,
        "explicit-terminal-owner-receipt");

  // Volatile crash fork: only a dirty marker is durable. Restart emits a
  // terminal loss report and never claims delivery.
  {
    L1TxJournal crashed(&terminal);
    crashed.begin_session(38);
    check(crashed.stage(1, 0, 0, 1, payload, sizeof(payload), 14),
          "process-crash-marker");
  }
  {
    L1TxJournal restarted(&terminal);
    check(terminal.take(&report) && report.prior_process_loss &&
              !report.delivery_claimed && report.entries.empty(),
          "process-restart");
  }

  explicit_marker_cases(marker);
  std::remove(marker);
  std::printf("[L1-JOURNAL-TEST] %s failures=%d\n",
              failures == 0 ? "PASS" : "FAIL", failures);
  return failures == 0 ? 0 : 1;
}
