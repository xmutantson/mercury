// Directed public-filter regression. Compile separately with
// -DB2F_TRANSFORM_FAILURE_TEST and b2f_handler.cc; link-time codec stubs
// exercise failure contracts without adding production fault-injection knobs.
#ifdef B2F_TRANSFORM_FAILURE_TEST
#include "datalink_layer/b2f_handler.h"
#include "compression/lzhuf_buffer.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {
int mode, calls, fail_at, bad_input, failures;
unsigned char key;
std::vector<char> plain() { return {'p', 0, (char)key, 9, 8}; }
std::vector<char> compressed() { return {'c', 0, (char)key, 1, 2, 3, 4, 5}; }
int codec(bool decode, const uint8_t* in, size_t len,
          uint8_t* out, size_t cap, size_t* out_len) {
  ++calls;
  const auto expected = decode ? compressed() : plain();
  const auto produced = decode ? plain() : compressed();
  if (len != expected.size() || std::memcmp(in, expected.data(), len) != 0)
    ++bad_input;
  if (calls == fail_at && mode) {
    if (mode == 1) return -1;  // leave out_len untouched
    if (mode == 4) { *out_len = 0; return 0; }
    *out_len = mode == 2 ? 3 : produced.size();
    std::memset(out, 0xa5, *out_len);
    return mode == 5 ? 1 : -1;
  }
  if (produced.size() > cap) return -1;
  std::memcpy(out, produced.data(), produced.size());
  *out_len = produced.size();
  return 0;
}
int filter(cl_b2f_handler& h, bool rx, const char* in, int len, char* out) {
  return rx ? h.filter_rx(in, len, out, 1024) : h.filter_tx(in, len, out, 1024);
}
bool setup(cl_b2f_handler& h, bool rx, bool prefix) {
  char out[1024];
  const std::string local = "[PAT-12.18.0-B2FHQEHX$]\r";
  const std::string remote = "[CMS-5.0.0-B2FHQEHX$]\r";
  if (h.filter_tx(local.data(), local.size(), out, sizeof(out)) != (int)local.size() ||
      h.filter_rx(remote.data(), remote.size(), out, sizeof(out)) != (int)remote.size()) return false;
  const std::string proposals = (prefix ? "FC EM PREFIX 5 8\r" : "") +
      std::string("FC EM BODY 5 8\rF>\r");
  const std::string fs = prefix ? "FS ++\r" : "FS +\r";
  return filter(h, rx, proposals.data(), proposals.size(), out) == (int)proposals.size() &&
      filter(h, !rx, fs.data(), fs.size(), out) == (int)fs.size() && h.is_b2f_session();
}
bool passthrough(cl_b2f_handler& h) {
  h.reset();
  const char bytes[] = {'n', 0, 'o', 'n', '-', 'b', '2', 'f'};
  char out[1024];
  return h.filter_tx(bytes, sizeof(bytes), out, sizeof(out)) == sizeof(bytes) &&
      std::memcmp(bytes, out, sizeof(bytes)) == 0 &&
      h.filter_rx(bytes, sizeof(bytes), out, sizeof(out)) == sizeof(bytes) &&
      std::memcmp(bytes, out, sizeof(bytes)) == 0 && !h.has_fatal_error();
}
bool successful_round(cl_b2f_handler& h, bool rx, unsigned char next_key) {
  key = next_key; mode = calls = bad_input = 0; fail_at = 0;
  if (!setup(h, rx, false)) return false;
  auto input = rx ? plain() : compressed();
  auto expected = rx ? compressed() : plain();
  for (char c : std::string("FF\r")) { input.push_back(c); expected.push_back(c); }
  char out[1024];
  int n = filter(h, rx, input.data(), input.size(), out);
  return n == (int)expected.size() && std::memcmp(out, expected.data(), expected.size()) == 0 &&
      calls == 1 && bad_input == 0 && !h.has_fatal_error() &&
      !h.has_pending_tx_work() && !h.has_pending_rx_work();
}
void scenario(bool rx, int fault, bool prefix, bool split, int id) {
  cl_b2f_handler h; h.init();
  key = (unsigned char)(id + 19); mode = fault; calls = bad_input = 0; fail_at = prefix ? 2 : 1;
  bool active = setup(h, rx, prefix);
  auto body = rx ? plain() : compressed();
  std::vector<char> input = body;
  if (prefix) input.insert(input.end(), body.begin(), body.end());
  const std::string suffix = "FF\rFQ\r";
  input.insert(input.end(), suffix.begin(), suffix.end());
  char out[1024];
  bool partial = !split || filter(h, rx, input.data(), 1, out) == 0;
  int n = filter(h, rx, input.data() + (split ? 1 : 0), input.size() - (split ? 1 : 0), out);
  bool closed = n == -1 && h.has_fatal_error() && calls == fail_at && bad_input == 0;
  // A negative count releases no current-call prefix/body/suffix to callers.
  bool refused = h.filter_tx("x", 1, out, sizeof(out)) == -1 &&
      h.filter_rx("y", 1, out, sizeof(out)) == -1 &&
      h.filter_tx(nullptr, 0, out, sizeof(out)) == -1 &&
      h.filter_rx(nullptr, 0, out, sizeof(out)) == -1;
  h.reset();
  bool reset = successful_round(h, rx, (unsigned char)(id + 79));
  h.deinit(); h.init();
  bool reinit = successful_round(h, !rx, (unsigned char)(id + 139));
  bool pass = passthrough(h);
  bool ok = active && partial && closed && refused && reset && reinit && pass;
  std::printf("B2F_FAILURE id=%d rx=%d fault=%d prefix=%d split=%d active=%d partial=%d closed=%d refused=%d reset=%d reinit=%d pass=%d ok=%d\n",
      id, rx, fault, prefix, split, active, partial, closed, refused, reset, reinit, pass, ok);
  if (!ok) ++failures;
}
}  // namespace
int lzhuf_decode_buffer(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t* n) {
  return codec(true, in, len, out, cap, n);
}
int lzhuf_encode_buffer(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t* n) {
  return codec(false, in, len, out, cap, n);
}
int main() {
  // Successful transforms and ordinary binary passthrough must pass before
  // forcing any fault, in each arm of the experiment.
  cl_b2f_handler control; control.init();
  bool tx = successful_round(control, false, 11);
  control.reset();
  bool rx = successful_round(control, true, 17);
  bool pass = passthrough(control);
  std::printf("B2F_FAILURE_CONTROL tx=%d rx=%d pass=%d\n", tx, rx, pass);
  if (!tx || !rx || !pass) return 2;
  int id = 0;
  for (bool direction : {false, true})
    for (int fault = 1; fault <= 5; ++fault)
      for (bool prefix : {false, true})
        for (bool split : {false, true}) scenario(direction, fault, prefix, split, id++);
  std::printf("B2F_FAILURE_RESULT cases=%d failures=%d\n", id, failures);
  return failures ? 1 : 0;
}
#endif
