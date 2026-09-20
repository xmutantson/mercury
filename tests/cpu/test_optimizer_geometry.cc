#include <cstdio>
#include "datalink_layer/optimizer_geometry.h"

static int fails = 0;
static void eqi(const char* n, int g, int w) {
    if (g != w) { std::fprintf(stderr, "FAIL %s got=%d want=%d\n", n, g, w); ++fails; }
}
static void ok(const char* n, bool v) {
    if (!v) { std::fprintf(stderr, "FAIL %s\n", n); ++fails; }
}

int main() {
    // 6x10 grid. Continuous pilots occupy columns 0/9; scatter dx=3,dy=2.
    // Exact union has 20 pilots => 40 data cells. QPSK => 80 bits; N=20 => K=4.
    st_bigblock_candidate_geometry g = optimizer_predict_bigblock_geometry(
        10, 64, 16, 2, 1, 4, 20, 320, 8000.0, 6, 2, 3, 2, 0);
    ok("bigblock geometry valid", g.valid);
    eqi("bigblock codeword count", g.codewords, 4);
    eqi("bigblock keydown", g.keydown_ms, 80);
    // sub_len=40; header=10; caps 29 + 39 + 39 + 35 = 142.
    eqi("bigblock app capacity", g.full_batch_payload_bytes, 142);

    st_bigblock_candidate_geometry capped = optimizer_predict_bigblock_geometry(
        10, 64, 16, 2, 1, 4, 20, 320, 8000.0, 6, 2, 3, 2, 3);
    ok("capped geometry valid", capped.valid);
    eqi("bigblock K cap", capped.codewords, 3);
    // sub_len=40; header=8; caps 31 + 39 + 35 = 105.
    eqi("capped app capacity", capped.full_batch_payload_bytes, 105);

    st_bigblock_candidate_geometry bad = optimizer_predict_bigblock_geometry(
        0, 64, 16, 2, 1, 4, 20, 320, 8000.0, 6, 2, 3, 2, 0);
    ok("invalid geometry rejected", !bad.valid);

    if (fails) return 1;
    std::puts("PASS optimizer candidate big-block geometry");
    return 0;
}
