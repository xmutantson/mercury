#include "datalink_layer/l1_tx_journal.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <utility>

namespace mercury {
namespace {
std::atomic<uint64_t> g_session_counter(1);

bool env_on(const char* name) {
  const char* value = std::getenv(name);
  return value && value[0] && std::atoi(value) != 0;
}

bool env_exact_one(const char* name) {
  const char* value = std::getenv(name);
  return value && value[0] == '1' && value[1] == '\0';
}
}  // namespace

const char* l1_reset_event_name(L1ResetEvent event) {
  switch (event) {
    case L1ResetEvent::ACK_TIMEOUT: return "ack-timeout";
    case L1ResetEvent::LOCAL_BREAK: return "local-break";
    case L1ResetEvent::PEER_BREAK: return "peer-break";
    case L1ResetEvent::COLLISION: return "collision";
    case L1ResetEvent::BREAK_RECOVERY_TIMEOUT: return "break-recovery-timeout";
    case L1ResetEvent::AUTHENTICATED_RECONNECT: return "authenticated-reconnect";
    case L1ResetEvent::TEARDOWN_CANCEL: return "teardown-cancel";
    case L1ResetEvent::SOFT_RESET: return "soft-reset";
    case L1ResetEvent::RESPONDER_RESET: return "responder-reset";
    case L1ResetEvent::CONFIG_CHANGE: return "config-change";
    case L1ResetEvent::CRYPTO_REKEY: return "crypto-rekey";
    case L1ResetEvent::BSI_WRAP: return "bsi-wrap";
    case L1ResetEvent::QUEUE_FLUSH: return "queue-flush";
    case L1ResetEvent::PROCESS_RESTART: return "process-restart";
  }
  return "unknown";
}

bool L1TerminalQueue::accept(L1TerminalReport report) {
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    report.receipt_id = next_receipt_++;
    reports_.push_back(std::move(report));
    std::fprintf(stderr, "[L1-TERMINAL] receipt=%llu reason=%s delivery_claimed=0\n",
                 (unsigned long long)reports_.back().receipt_id,
                 reports_.back().reason.c_str());
    return true;
  } catch (...) {
    return false;
  }
}

bool L1TerminalQueue::take(L1TerminalReport* out) {
  if (!out) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (reports_.empty()) return false;
  *out = std::move(reports_.front());
  reports_.pop_front();
  return true;
}

std::size_t L1TerminalQueue::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return reports_.size();
}

L1TxJournal::L1TxJournal(L1TerminalQueue* terminal_owner)
    : enabled_(env_on("MERCURY_L1_JOURNAL") ||
               env_exact_one("MERCURY_L1_BLOCKACK")),
      block_ack_mode_(env_exact_one("MERCURY_L1_BLOCKACK")),
      terminal_owner_(terminal_owner) {
  const char* marker = std::getenv("MERCURY_L1_JOURNAL_STATE");
  marker_path_ = marker && marker[0] ? marker : ".mercury_l1_journal.pending";
  if (enabled_ && terminal_owner_) recover_restart_marker();
}

void L1TxJournal::recover_restart_marker() {
  if (!enabled_ || !terminal_owner_) return;
  std::ifstream in(marker_path_.c_str());
  if (!in.good()) return;
  L1TerminalReport report;
  report.reason = "process-restart-volatile-terminal";
  report.prior_process_loss = true;
  in >> report.session_id >> report.epoch;
  if (terminal_owner_->accept(std::move(report))) clear_marker();
}

void L1TxJournal::begin_session(uint8_t wire_connection_id,
                               uint64_t authenticated_transfer_id) {
  if (!enabled_) return;
  if (!entries_.empty() && !terminalize("unvalidated-reconnect", {})) std::abort();
  uint64_t serial = g_session_counter.fetch_add(1);
  session_id_ = (serial << 8) | wire_connection_id;
  epoch_ = 1;
  block_serial_ = 0;
  ordered_tx_seq_ = 0;
  authenticated_transfer_id_ = authenticated_transfer_id;
  acknowledged_.clear();
  recovery_open_ = false;
  have_last_bsi_ = false;
}

bool L1TxJournal::migrate_authenticated(uint8_t wire_connection_id,
                                        uint64_t authenticated_transfer_id) {
  if (!enabled_) return true;
  if (!authenticated_transfer_id_ ||
      authenticated_transfer_id != authenticated_transfer_id_) return false;
  const uint64_t serial = g_session_counter.fetch_add(1);
  session_id_ = (serial << 8) | wire_connection_id;
  epoch_ = 1;
  uint16_t slot = 0;
  for (auto& entry : entries_) {
    entry.key.session_id = session_id_;
    entry.key.epoch = epoch_;
    entry.key.bsi = 0;
    entry.key.slot = slot++;
    entry.batch_index = entry.key.slot;
    entry.span = (uint16_t)entries_.size();
    entry.state = L1JournalState::STAGED;
  }
  acknowledged_.clear();
  recovery_open_ = true;
  have_last_bsi_ = false;
  return write_marker();
}

L1JournalEntry* L1TxJournal::find_current(uint8_t bsi, uint16_t slot) {
  for (auto& entry : entries_)
    if (entry.key.session_id == session_id_ && entry.key.epoch == epoch_ &&
        entry.key.bsi == bsi && entry.key.slot == slot) return &entry;
  return nullptr;
}

bool L1TxJournal::stage(uint8_t transmitted_bsi, uint16_t slot,
                        uint16_t batch_index, uint16_t span,
                        const char* plaintext, std::size_t length,
                        int configuration) {
  L1StageItem item;
  item.slot = slot;
  item.batch_index = batch_index;
  item.span = span;
  item.plaintext.assign(plaintext, plaintext + length);
  item.configuration = configuration;
  return stage_batch(transmitted_bsi, {item});
}

bool L1TxJournal::stage_batch(uint8_t transmitted_bsi,
                             const std::vector<L1StageItem>& items) {
  if (!enabled_) return true;
  if (items.empty() || items.size() > kBatchSlotCap) return false;
  for (std::size_t i = 0; i < items.size(); ++i) {
    if (items[i].plaintext.empty()) return false;
    for (std::size_t j = i + 1; j < items.size(); ++j)
      if (items[i].slot == items[j].slot) return false;
  }
  if (!session_id_) begin_session(0);
  if (have_last_bsi_ && transmitted_bsi < last_bsi_ &&
      !(block_ack_mode_ && last_bsi_ == 255 && transmitted_bsi == 0))
    apply(L1ResetEvent::BSI_WRAP);
  have_last_bsi_ = true;
  last_bsi_ = transmitted_bsi;
  std::vector<L1JournalEntry> before = entries_;
  const uint64_t serial_before = block_serial_;
  const uint64_t sequence_before = ordered_tx_seq_;
  try {
    if (block_serial_ == UINT64_MAX) return false;
    const uint64_t batch_serial = ++block_serial_;
    for (const auto& item : items) {
      L1JournalEntry* target = find_current(transmitted_bsi, item.slot);
      if (!target) {
        for (auto& old : entries_)
          if (old.key.session_id == session_id_ && old.key.epoch != epoch_ &&
              old.key.slot == item.slot && old.state == L1JournalState::STAGED) {
            target = &old; break;
          }
      }
      if (target) {
        target->key = {session_id_, epoch_, transmitted_bsi, item.slot};
        target->block_serial = batch_serial;
        target->batch_index = item.batch_index;
        target->span = item.span;
        // The first-stage copy is the retransmittable plaintext authority.
        // A retransmit buffer may contain an encoded/encrypted derivative;
        // never replace retained plaintext with that later representation.
        target->configuration = item.configuration;
        target->attempts++;
        target->state = L1JournalState::STAGED;
      } else {
        L1JournalEntry entry;
        entry.key = {session_id_, epoch_, transmitted_bsi, item.slot};
        entry.block_serial = batch_serial;
        entry.batch_index = item.batch_index;
        entry.span = item.span;
        entry.ordered_tx_seq = ++ordered_tx_seq_;
        entry.plaintext = item.plaintext;
        entry.configuration = item.configuration;
        entry.attempts = 1;
        entries_.push_back(std::move(entry));
      }
    }
    if (entries_.size() > kRetainedSlotCap) {
      entries_.swap(before);
      block_serial_ = serial_before;
      ordered_tx_seq_ = sequence_before;
      return false;
    }
  } catch (...) {
    entries_.swap(before);
    block_serial_ = serial_before;
    ordered_tx_seq_ = sequence_before;
    return false;
  }
  if (!write_marker()) {
    entries_.swap(before);
    block_serial_ = serial_before;
    ordered_tx_seq_ = sequence_before;
    return false;
  }
  return true;
}

void L1TxJournal::mark_sent(uint8_t bsi, uint16_t slot) {
  (void)mark_sent_many({{bsi, slot}});
}

bool L1TxJournal::mark_sent_many(
    const std::vector<std::pair<uint8_t, uint16_t>>& keys) {
  if (!enabled_) return true;
  for (const auto& key : keys) if (!find_current(key.first, key.second)) return false;
  for (const auto& key : keys) find_current(key.first, key.second)->state = L1JournalState::SENT;
  return true;
}

bool L1TxJournal::acknowledge(uint8_t bsi, uint16_t slot) {
  return acknowledge_many({{bsi, slot}});
}

bool L1TxJournal::acknowledge_many(
    const std::vector<std::pair<uint8_t, uint16_t>>& keys) {
  if (!enabled_) return true;
  const std::vector<L1JournalEntry> entries_before = entries_;
  const std::vector<L1JournalKey> acknowledged_before = acknowledged_;
  std::vector<L1JournalKey> resolved;
  for (const auto& key : keys) {
    L1JournalEntry* entry = find_current(key.first, key.second);
    if (entry && entry->state == L1JournalState::SENT) {
      resolved.push_back(entry->key);
      continue;
    }
    bool tombstone = false;
    for (const auto& acked : acknowledged_)
      if (acked.session_id == session_id_ && acked.epoch == epoch_ &&
          acked.bsi == key.first && acked.slot == key.second) tombstone = true;
    if (!tombstone) return false;
  }
  for (const auto& key : resolved) {
    for (auto it = entries_.begin(); it != entries_.end(); ++it)
      if (it->key == key) { acknowledged_.push_back(key); entries_.erase(it); break; }
  }
  if (acknowledged_.size() > kRetainedSlotCap)
    acknowledged_.erase(acknowledged_.begin(),
                        acknowledged_.begin() +
                            (acknowledged_.size() - kRetainedSlotCap));
  recovery_open_ = false;
  if (entries_.empty()) clear_marker();
  else if (!write_marker()) {
    entries_ = entries_before;
    acknowledged_ = acknowledged_before;
    return false;
  }
  return true;
}

void L1TxJournal::bump_epoch() {
  if (++epoch_ == 0) ++epoch_;
  for (auto& entry : entries_) entry.state = L1JournalState::STAGED;
  recovery_open_ = true;
}

void L1TxJournal::apply(L1ResetEvent event) {
  if (!enabled_) return;
  switch (event) {
    case L1ResetEvent::LOCAL_BREAK:
    case L1ResetEvent::PEER_BREAK:
    case L1ResetEvent::COLLISION:
    case L1ResetEvent::SOFT_RESET:
    case L1ResetEvent::RESPONDER_RESET:
    case L1ResetEvent::CONFIG_CHANGE:
    case L1ResetEvent::CRYPTO_REKEY:
      if (!recovery_open_) bump_epoch();
      break;
    case L1ResetEvent::AUTHENTICATED_RECONNECT:
      // Retain under the old authority until migrate_authenticated validates
      // the transfer identifier installed by the authenticated session layer.
      break;
    case L1ResetEvent::TEARDOWN_CANCEL:
    case L1ResetEvent::PROCESS_RESTART:
      break;
    case L1ResetEvent::ACK_TIMEOUT:
    case L1ResetEvent::BREAK_RECOVERY_TIMEOUT:
    case L1ResetEvent::BSI_WRAP:
    case L1ResetEvent::QUEUE_FLUSH:
      break;
  }
  write_marker();
}

bool L1TxJournal::terminalize(const char* reason,
                              const std::vector<char>& queued_plaintext) {
  if (!enabled_) return true;
  if (entries_.empty() && queued_plaintext.empty()) return true;
  if (!terminal_owner_) return false;
  L1TerminalReport report;
  report.reason = reason ? reason : "terminal-failure";
  report.session_id = session_id_;
  report.epoch = epoch_;
  report.entries = entries_;
  report.queued_plaintext = queued_plaintext;
  if (!terminal_owner_->accept(std::move(report))) return false;
  entries_.clear();
  clear_marker();
  recovery_open_ = false;
  return true;
}

bool L1TxJournal::write_marker() {
  if (!enabled_ || entries_.empty()) return true;
  const std::string temporary = marker_path_ + ".tmp";
  {
    std::ofstream out(temporary.c_str(), std::ios::trunc);
    if (!out.good()) return false;
    out << session_id_ << ' ' << epoch_ << ' ' << entries_.size() << '\n';
    out.flush();
    if (!out.good()) return false;
  }
  return std::rename(temporary.c_str(), marker_path_.c_str()) == 0;
}

void L1TxJournal::clear_marker() {
  std::remove(marker_path_.c_str());
  std::remove((marker_path_ + ".tmp").c_str());
}

}  // namespace mercury
