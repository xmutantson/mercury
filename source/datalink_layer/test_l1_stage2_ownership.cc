#include "datalink_layer/arq.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>
#include <vector>

int cl_arq_controller::test_l1_tx_journal_disabled_fail_closed() {
  struct EnvRestore {
    const char* key;
    bool had;
    std::string value;
    ~EnvRestore() {
      if(had) setenv(key, value.c_str(), 1);
      else unsetenv(key);
    }
  };
  const char* previous_journal = std::getenv("MERCURY_L1_JOURNAL");
  const char* previous_blockack = std::getenv("MERCURY_L1_BLOCKACK");
  EnvRestore restore_journal = {"MERCURY_L1_JOURNAL",
                                previous_journal != nullptr,
                                previous_journal ? std::string(previous_journal) : std::string()};
  EnvRestore restore_blockack = {"MERCURY_L1_BLOCKACK",
                                 previous_blockack != nullptr,
                                 previous_blockack ? std::string(previous_blockack) : std::string()};
  setenv("MERCURY_L1_JOURNAL", "0", 1);
  setenv("MERCURY_L1_BLOCKACK", "0", 1);

  mercury::L1TxJournal journal;
  mercury::L1StageItem item;
  item.slot = 0;
  item.batch_index = 0;
  item.span = 1;
  item.plaintext = {'x'};

  const bool passed = !journal.enabled() && !journal.stage_batch(1, {item}) &&
                      journal.size() == 0;
  std::printf("[TEST-L1-JOURNAL-DISABLED] %s\n", passed ? "PASS" : "FAIL");
  return passed ? 0 : 1;
}

int cl_arq_controller::test_l1_journal_disabled_mark_sent_many() {
  struct EnvRestore {
    const char* key;
    bool had;
    std::string value;
    ~EnvRestore() {
      if(had) setenv(key, value.c_str(), 1);
      else unsetenv(key);
    }
  };
  const char* previous_journal = std::getenv("MERCURY_L1_JOURNAL");
  const char* previous_blockack = std::getenv("MERCURY_L1_BLOCKACK");
  EnvRestore restore_journal = {
      "MERCURY_L1_JOURNAL", previous_journal != nullptr,
      previous_journal ? std::string(previous_journal) : std::string()};
  EnvRestore restore_blockack = {
      "MERCURY_L1_BLOCKACK", previous_blockack != nullptr,
      previous_blockack ? std::string(previous_blockack) : std::string()};

  setenv("MERCURY_L1_JOURNAL", "0", 1);
  setenv("MERCURY_L1_BLOCKACK", "0", 1);
  mercury::L1TxJournal journal;
  const bool passed = !journal.enabled() &&
                      !journal.mark_sent_many({{17, 3}}) &&
                      journal.size() == 0;
  std::printf("[TEST-L1-JOURNAL-DISABLED] %s mark_sent_many fails closed\n",
              passed ? "PASS" : "FAIL");
  return passed ? 0 : 1;
}

int cl_arq_controller::l1_test_timeout_case(const char* marker,
                                             int* awaiting_after,
                                             int* queued_after,
                                             int* reset_completed) {
  *awaiting_after = *queued_after = *reset_completed = 0;
  cl_telecom_system* ts = new cl_telecom_system();
  telecom_system = ts;
  auto cleanup = [&]() {
    deinit_messages_buffers();
    telecom_system = NULL;
    delete ts;
    std::remove(marker);
  };
  role = COMMANDER;
  original_role = COMMANDER;
  robust_enabled = NO;
  narrowband_enabled = NO;
  ts->narrowband_enabled = NO;
  init_configuration = CONFIG_0;
  data_configuration = CONFIG_0;
  ack_configuration = CONFIG_0;
  current_configuration = CONFIG_NONE;
  load_configuration(CONFIG_0, FULL, YES);
  fifo_buffer_tx.set_size(default_configuration_ARQ.fifo_buffer_tx_size);
  fifo_buffer_backup.set_size(default_configuration_ARQ.fifo_buffer_backup_size);
  fifo_buffer_rx.set_size(default_configuration_ARQ.fifo_buffer_rx_size);
  fifo_buffer_tx.flush();
  fifo_buffer_backup.flush();
  fifo_buffer_rx.flush();
  compression_enabled = false;
  encryption_enabled = false;
  sack_enabled = true;
  sack_v2_enabled = true;
  header_carries_d5 = true;
  set_data_batch_size(1);
  link_status = CONNECTED;
  connection_status = RECEIVING_ACKS_DATA;

  const std::vector<char> awaiting = {'A'};
  std::vector<char> queued(61);
  for(std::size_t i=0;i<queued.size();i++)
    queued[i] = (char)(0x91 ^ ((i * 53 + i * i * 5) & 0x7f));

  add_message_tx_data(DATA_SHORT, 1, const_cast<char*>(awaiting.data()));
  int slot = -1;
  for(int i=0;i<nMessages;i++)
    if(messages_tx[i].status != FREE && messages_tx[i].length == 1 &&
       messages_tx[i].data[0] == awaiting[0]) { slot = i; break; }
  if(slot < 0) {
    cleanup();
    return 1;
  }
  messages_tx[slot].batch_seq_id = 219;
  messages_tx[slot].id = (char)slot;
  messages_tx[slot].status = PENDING_ACK;
  messages_tx[slot].ack_timeout = 1000000;
  messages_tx[slot].ack_timer.stop();
  messages_tx[slot].ack_timer.reset();
  if(l1_tx_journal.enabled()) {
    l1_tx_journal.begin_session(37);
    if(!l1_tx_journal.stage(219, (uint16_t)slot, 0, 1,
                            awaiting.data(), awaiting.size(), CONFIG_0)) {
      cleanup();
      return 1;
    }
    l1_tx_journal.mark_sent(219, (uint16_t)slot);
  }
  fifo_buffer_tx.push(const_cast<char*>(queued.data()), (int)queued.size());

  link_timer.stop();
  link_timer.reset();
  link_timer.seconds = link_timeout / 1000 + 2;
  link_timer.miliSeconds = 0;
  link_timer.counting = NO;
  watchdog_timer.stop();
  receiving_timer.stop();
  gear_shift_timer.stop();
  update_status();

  *reset_completed =
      link_status == CONNECTING && current_configuration == init_configuration;
  if(l1_tx_journal.enabled()) {
    mercury::L1TerminalReport report;
    if(l1_terminal_queue.take(&report)) {
      for(const auto& entry : report.entries)
        if(entry.plaintext == awaiting && !report.delivery_claimed) (*awaiting_after)++;
      if(report.queued_plaintext == queued && !report.delivery_claimed) (*queued_after)++;
    }
  } else {
    for(int i=0;i<nMessages;i++)
      if(messages_tx[i].status != FREE && messages_tx[i].length == 1 &&
         messages_tx[i].data[0] == awaiting[0]) (*awaiting_after)++;
    const int occupied = fifo_buffer_tx.get_size() - fifo_buffer_tx.get_free_size();
    if(occupied > 0) (*queued_after)++;
  }

  cleanup();
  return 0;
}

int cl_arq_controller::test_l1_stage2_ownership() {
  struct EnvRestore {
    const char* key;
    bool had;
    std::string value;
    ~EnvRestore() {
      if(had) setenv(key, value.c_str(), 1);
      else unsetenv(key);
    }
  };
  const char* previous_state = std::getenv("MERCURY_L1_JOURNAL_STATE");
  const char* previous_journal = std::getenv("MERCURY_L1_JOURNAL");
  EnvRestore restore_state = {"MERCURY_L1_JOURNAL_STATE",
                              previous_state != nullptr,
                              previous_state ? std::string(previous_state) : std::string()};
  EnvRestore restore_journal = {"MERCURY_L1_JOURNAL",
                                previous_journal != nullptr,
                                previous_journal ? std::string(previous_journal) : std::string()};
  char marker[160];
  std::snprintf(marker, sizeof(marker), "/tmp/mercury_l1_prod_%ld.pending",
                (long)getpid());
  int ba=0, bq=0, br=0, aa=0, aq=0, ar=0;
  setenv("MERCURY_L1_JOURNAL_STATE", marker, 1);
  setenv("MERCURY_L1_JOURNAL", "0", 1);
  { cl_arq_controller before; before.l1_test_timeout_case(marker, &ba, &bq, &br); }
  setenv("MERCURY_L1_JOURNAL", "1", 1);
  { cl_arq_controller after; after.l1_test_timeout_case(marker, &aa, &aq, &ar); }

  int failures = 0;
  auto check = [&](bool ok, const char* name) {
    std::printf("[TEST-L1-STAGE2] %s %s\n", ok ? "PASS" : "FAIL", name);
    if(!ok) failures++;
  };
  check(ba == 0 && bq == 0 && br == 1,
        "fail-before production discard reproduced");
  check(aa == 1 && aq == 1,
        "pass-after accepted terminal owner retains both ranges");
  check(ar == 1, "destructive reset follows terminal acceptance receipt");
  std::printf("[TEST-L1-STAGE2] %s failures=%d before={%d,%d,%d} after={%d,%d,%d}\n",
      failures ? "FAIL" : "PASS", failures, ba, bq, br, aa, aq, ar);
  return failures ? 1 : 0;
}
