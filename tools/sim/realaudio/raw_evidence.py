"""raw_evidence.py -- per-attempt raw byte retention for real-audio cells.

Retains, under the cell's log directory and while the attempt runs (so failed and
timed-out attempts keep what they had):
  rx_payload_<tag>.bin   the exact delivered application stream, in arrival order
  tx_sent_<tag>.bin      the exact bytes handed to the commander's data socket
and at finalize() records sha256 + length of both, regenerates the transmitted
reference from its generator (capstone_arms.traffic_slice, a pure function of the
offset) and verifies the sent file against it, and computes an independent byte
comparison of the delivered stream against the reference (first mismatch offset,
matching prefix). Everything needed for an independent byte comparison stays on disk.
"""

import hashlib
import os
import threading

SCHEMA = "raw-evidence/1"


def _sha256(path):
    h = hashlib.sha256()
    n = 0
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
            n += len(block)
    return h.hexdigest(), n


class RawEvidence:
    def __init__(self, logdir, tag, traffic, generator):
        """generator(traffic, offset, length) -> bytes (the TX ruler)."""
        self.logdir = logdir
        self.tag = tag
        self.traffic = traffic
        self.generator = generator
        self.rx_path = os.path.join(logdir, "rx_payload_%s.bin" % tag)
        self.tx_path = os.path.join(logdir, "tx_sent_%s.bin" % tag)
        self._lock = threading.Lock()
        self._rx = open(self.rx_path, "wb")
        self._tx = open(self.tx_path, "wb")
        self.rx_bytes = 0
        self.tx_bytes = 0
        self.errors = []
        self.closed = False

    def _write(self, fh, data, attr):
        with self._lock:
            if self.closed:
                self.errors.append("%s write after finalize (%d B)" % (attr, len(data)))
                return
            try:
                fh.write(data)
                fh.flush()
                setattr(self, attr, getattr(self, attr) + len(data))
            except OSError as exc:
                self.errors.append("%s write failed: %s" % (attr, exc))

    def rx(self, data):
        self._write(self._rx, data, "rx_bytes")

    def tx(self, data):
        self._write(self._tx, data, "tx_bytes")

    def finalize(self, harness_rx_bytes=None, harness_tx_bytes=None):
        with self._lock:
            if not self.closed:
                self.closed = True
                for fh in (self._rx, self._tx):
                    try:
                        fh.close()
                    except OSError as exc:
                        self.errors.append("close failed: %s" % exc)
        out = {"schema": SCHEMA, "rx_payload_path": self.rx_path, "tx_sent_path": self.tx_path,
               "traffic": self.traffic, "errors": list(self.errors)}
        try:
            out["rx_payload_sha256"], out["rx_payload_bytes"] = _sha256(self.rx_path)
            out["tx_sent_sha256"], out["tx_sent_bytes"] = _sha256(self.tx_path)
        except OSError as exc:
            out["errors"].append("hash failed: %s" % exc)
            out["independent_compare"] = "UNAVAILABLE"
            return out
        # reference regenerated from the generator and compared to the bytes actually sent
        ref = self.generator(self.traffic, 0, out["tx_sent_bytes"])
        out["reference_sha256"] = hashlib.sha256(ref).hexdigest()
        with open(self.tx_path, "rb") as f:
            sent = f.read()
        out["tx_matches_reference"] = (sent == ref)
        with open(self.rx_path, "rb") as f:
            got = f.read()
        exp = self.generator(self.traffic, 0, len(got))
        first_bad = None
        if got != exp:
            for i in range(len(got)):
                if got[i] != exp[i]:
                    first_bad = i
                    break
        out["independent_compare"] = "MATCH" if first_bad is None else "MISMATCH"
        out["independent_first_mismatch_offset"] = first_bad
        out["independent_good_prefix_bytes"] = len(got) if first_bad is None else first_bad
        if harness_rx_bytes is not None and harness_rx_bytes != out["rx_payload_bytes"]:
            out["errors"].append("harness rx count %d != retained %d" % (harness_rx_bytes, out["rx_payload_bytes"]))
        if harness_tx_bytes is not None and harness_tx_bytes != out["tx_sent_bytes"]:
            out["errors"].append("harness tx count %d != retained %d" % (harness_tx_bytes, out["tx_sent_bytes"]))
        return out
