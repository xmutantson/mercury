#!/usr/bin/env python3
"""gen_incompressible_payload.py — regenerate the fixed incompressible payload.

The FTRT I7 delivered-bps A/B (sim_arq_channel.py) streams a FIXED, high-entropy
file so PPMd/zstd are bypassed and the wire carries RAW PHY rate (NOT the old
bytes(range(256))*8 repeating pattern, which compresses to almost nothing and
inflates the measured rate). The file is committed so runs are reproducible; this
script regenerates it byte-for-byte from a fixed seed.

    python tools/sim/gen_incompressible_payload.py
    # -> payload_incompressible_64k.bin  (65536 bytes, md5 d991876bd9d343271a704d7214997300)

The bytes come from chained SHA-256 over (SEED || counter) so they are
deterministic across machines and ~8 bits/byte entropy (compression-resistant).
"""
import hashlib
import os
import sys

SEED = b"mercury-sim-i7-incompressible-payload-v1"
SIZE = 64 * 1024
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "payload_incompressible_64k.bin")
EXPECT_MD5 = "d991876bd9d343271a704d7214997300"


def build():
    buf = bytearray()
    ctr = 0
    while len(buf) < SIZE:
        buf.extend(hashlib.sha256(SEED + ctr.to_bytes(8, "little")).digest())
        ctr += 1
    return bytes(buf[:SIZE])


def main():
    data = build()
    md5 = hashlib.md5(data).hexdigest()
    with open(OUT, "wb") as f:
        f.write(data)
    print(f"wrote {OUT}  ({len(data)} bytes, md5={md5})")
    if md5 != EXPECT_MD5:
        print(f"WARNING: md5 {md5} != expected {EXPECT_MD5}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
