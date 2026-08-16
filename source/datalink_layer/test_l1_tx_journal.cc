#include "datalink_layer/l1_tx_journal.h"

#include <cstdio>
#include <cstdlib>

#ifdef _WIN32
// MSVCRT never declares the POSIX env setters (same shim as arq_common.cc).
static int setenv(const char* name, const char* value, int overwrite) {
	if(!overwrite && getenv(name) != NULL) return 0;
	return _putenv_s(name, value);
}
static int unsetenv(const char* name) { return _putenv_s(name, ""); }
#endif
#include <string>
#include <unistd.h>

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

int main() {
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

  std::remove(marker);
  std::printf("[L1-JOURNAL-TEST] %s failures=%d\n",
              failures == 0 ? "PASS" : "FAIL", failures);
  return failures == 0 ? 0 : 1;
}
