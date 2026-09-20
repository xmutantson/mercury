#!/usr/bin/env python3
"""Inventory runtime MERCURY_* gates that look default-off/experimental.

This is a discovery tool, not an automatic safety classifier. It scans runtime
C/C++ sources and prints gates whose nearby comments contain words commonly used
for feature debt. Human review assigns A/B/C/D/E in DEFAULT_OFF_FEATURE_AUDIT.md.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCAN_ROOTS = (ROOT / "source", ROOT / "include")
EXTS = {".c", ".cc", ".cpp", ".h", ".hpp"}
KEYWORDS = re.compile(
    r"default[- ]?off|opt[- ]?in|experimental|held off|deferred|quarantined|"
    r"do[- ]?not[- ]?enable|fail[- ]?before|defeat|byte[- ]?identical",
    re.IGNORECASE,
)
ENV = re.compile(r"\bMERCURY_[A-Z0-9_]+\b")
GETENV = re.compile(r"(?:std::)?getenv\s*\(\s*\"(MERCURY_[A-Z0-9_]+)\"\s*\)")
PP = re.compile(r"^\s*#\s*(?:if|ifdef|ifndef|elif).*\b(MERCURY_[A-Z0-9_]+)\b")

rows: list[tuple[str, str, int, str]] = []
seen: set[tuple[str, str, int]] = set()

for base in SCAN_ROOTS:
    if not base.exists():
        continue
    for path in sorted(p for p in base.rglob("*") if p.suffix in EXTS):
        text = path.read_text(errors="replace")
        lines = text.splitlines()
        for i, line in enumerate(lines):
            names = set(GETENV.findall(line))
            m = PP.search(line)
            if m:
                names.add(m.group(1))
            if not names:
                continue
            lo = max(0, i - 18)
            hi = min(len(lines), i + 5)
            context = "\n".join(lines[lo:hi])
            if not KEYWORDS.search(context):
                continue
            # Include MERCURY names mentioned in the same evidence window so an
            # alias/master defeat gate is not silently missed.
            names.update(ENV.findall(context))
            evidence_lines = [
                x.strip().lstrip("/").strip()
                for x in lines[lo:hi]
                if KEYWORDS.search(x) or "getenv" in x or re.match(r"\s*#\s*(?:if|ifdef|ifndef|elif)", x)
            ]
            evidence = " ".join(evidence_lines)
            evidence = re.sub(r"\s+", " ", evidence)[:360]
            rel = path.relative_to(ROOT).as_posix()
            for name in sorted(names):
                key = (name, rel, i + 1)
                if key in seen:
                    continue
                seen.add(key)
                rows.append((name, rel, i + 1, evidence))

rows.sort(key=lambda r: (r[0], r[1], r[2]))
print("| Gate | Location | Nearby evidence |")
print("| --- | --- | --- |")
for name, rel, line, evidence in rows:
    evidence = evidence.replace("|", "\\|")
    print(f"| `{name}` | `{rel}:{line}` | {evidence} |")
