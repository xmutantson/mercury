#ifndef MERCURY_L1_TX_JOURNAL_H
#define MERCURY_L1_TX_JOURNAL_H

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <utility>
#include <string>
#include <vector>

namespace mercury {

enum class L1JournalState { STAGED, SENT };

enum class L1ResetEvent {
  ACK_TIMEOUT,
  LOCAL_BREAK,
  PEER_BREAK,
  COLLISION,
  BREAK_RECOVERY_TIMEOUT,
  AUTHENTICATED_RECONNECT,
  TEARDOWN_CANCEL,
  SOFT_RESET,
  RESPONDER_RESET,
  CONFIG_CHANGE,
  CRYPTO_REKEY,
  BSI_WRAP,
  QUEUE_FLUSH,
  PROCESS_RESTART
};

struct L1JournalKey {
  uint64_t session_id = 0;
  uint32_t epoch = 0;
  uint8_t bsi = 0;
  uint16_t slot = 0;
  bool operator==(const L1JournalKey& o) const {
    return session_id == o.session_id && epoch == o.epoch &&
           bsi == o.bsi && slot == o.slot;
  }
};

struct L1JournalEntry {
  L1JournalKey key;
  uint64_t block_serial = 0;
  uint16_t batch_index = 0;
  uint16_t span = 0;
  uint64_t ordered_tx_seq = 0;
  std::vector<char> plaintext;
  int configuration = 0;
  unsigned attempts = 0;
  L1JournalState state = L1JournalState::STAGED;
};

struct L1StageItem {
  uint16_t slot = 0;
  uint16_t batch_index = 0;
  uint16_t span = 0;
  std::vector<char> plaintext;
  int configuration = 0;
};

struct L1TerminalReport {
  uint64_t receipt_id = 0;
  std::string reason;
  bool delivery_claimed = false;
  bool prior_process_loss = false;
  uint64_t session_id = 0;
  uint32_t epoch = 0;
  std::vector<L1JournalEntry> entries;
  std::vector<char> queued_plaintext;
};

class L1TerminalQueue {
 public:
  bool accept(L1TerminalReport report);
  bool take(L1TerminalReport* out);
  std::size_t size() const;

 private:
  mutable std::mutex mutex_;
  std::deque<L1TerminalReport> reports_;
  uint64_t next_receipt_ = 1;
};

class L1TxJournal {
 public:
  explicit L1TxJournal(L1TerminalQueue* terminal_owner = nullptr);

  bool enabled() const { return enabled_; }
  void set_terminal_owner(L1TerminalQueue* owner) { terminal_owner_ = owner; }
  void begin_session(uint8_t wire_connection_id,
                     uint64_t authenticated_transfer_id = 0);
  bool migrate_authenticated(uint8_t wire_connection_id,
                             uint64_t authenticated_transfer_id);
  bool stage_batch(uint8_t transmitted_bsi,
                   const std::vector<L1StageItem>& items);
  bool stage(uint8_t transmitted_bsi, uint16_t slot, uint16_t batch_index,
             uint16_t span, const char* plaintext, std::size_t length,
             int configuration);
  void mark_sent(uint8_t transmitted_bsi, uint16_t slot);
  bool mark_sent_many(const std::vector<std::pair<uint8_t, uint16_t>>& keys);
  bool acknowledge(uint8_t transmitted_bsi, uint16_t slot);
  bool acknowledge_many(const std::vector<std::pair<uint8_t, uint16_t>>& keys);
  void apply(L1ResetEvent event);
  bool terminalize(const char* reason, const std::vector<char>& queued_plaintext);
  void recover_restart_marker();

  std::size_t size() const { return entries_.size(); }
  uint64_t session_id() const { return session_id_; }
  uint32_t epoch() const { return epoch_; }
  bool recovery_open() const { return recovery_open_; }
  const std::vector<L1JournalEntry>& entries() const { return entries_; }

 private:
  L1JournalEntry* find_current(uint8_t bsi, uint16_t slot);
  void bump_epoch();
  bool write_marker();
  void clear_marker();

  bool enabled_ = false;
  bool recovery_open_ = false;
  bool have_last_bsi_ = false;
  uint8_t last_bsi_ = 0;
  uint64_t session_id_ = 0;
  uint32_t epoch_ = 0;
  uint64_t block_serial_ = 0;
  uint64_t ordered_tx_seq_ = 0;
  uint64_t authenticated_transfer_id_ = 0;
  static const std::size_t kSlotCap = 96;
  std::vector<L1JournalEntry> entries_;
  std::vector<L1JournalKey> acknowledged_;
  L1TerminalQueue* terminal_owner_ = nullptr;
  std::string marker_path_;
};

const char* l1_reset_event_name(L1ResetEvent event);

}  // namespace mercury

#endif
