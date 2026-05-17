#!/usr/bin/env python3
"""Static regression test for the BREAK-handler messages_control reset.

Captures Bug D (SACK_DESIGN_A_PLAN §7.13.14, POST_BREAK_STUCK_INVESTIGATION):
the RSP BREAK handler at arq_responder.cc was missing
`messages_control.status = FREE;`, causing every post-BREAK SET_CONFIG to
be silently dropped at the FREE-gate at arq_responder.cc:281.

Three asserts (one RSP-side fix + two pre-existing CMD-side parallels that
prove the pattern is intentional, not coincidental):

  1. arq_responder.cc BREAK handler resets messages_control.status to FREE
     (Bug D fix at line ~250).
  2. arq_commander.cc post-BREAK ACK path #1 force-FREEs (line ~113).
  3. arq_commander.cc post-BREAK ACK path #2 force-FREEs (line ~185).

Each block is located by anchor regex rather than absolute line number so
the test survives unrelated edits in the same file. If any assert FAILS,
print the missing pattern + cite the fact doc so a future contributor
understands the load-bearing nature of the line before deleting it.

Run:
    python mercury/tools/test_break_handler_reset.py

Exit code 0 on PASS, 1 on FAIL. No build required.
"""
from __future__ import annotations
import os
import re
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MERCURY_ROOT = os.path.dirname(SCRIPT_DIR)
SRC = os.path.join(MERCURY_ROOT, 'source', 'datalink_layer')

# Pattern that matches the assignment regardless of whitespace / formatting.
FREE_RESET_RE = re.compile(r'messages_control\.status\s*=\s*FREE\s*;')


def find_anchor_line(text: str, anchor_re: re.Pattern, label: str) -> int:
    """Return 1-based line number of the first anchor match, or raise."""
    for i, line in enumerate(text.splitlines(), start=1):
        if anchor_re.search(line):
            return i
    raise AssertionError(f'anchor not found for {label}: {anchor_re.pattern!r}')


def slice_lines(text: str, start_1based: int, n_lines: int) -> str:
    """Return text from a 1-based line range (inclusive of start, length n)."""
    lines = text.splitlines()
    return '\n'.join(lines[start_1based - 1: start_1based - 1 + n_lines])


def assert_reset_within(text: str, anchor_re: re.Pattern, window: int,
                        label: str, fact_doc_ref: str) -> tuple[bool, str]:
    """Find anchor, slice `window` lines after, assert FREE_RESET_RE matches.

    Returns (passed, message).
    """
    try:
        anchor_line = find_anchor_line(text, anchor_re, label)
    except AssertionError as e:
        return False, f'{label}: {e}'
    block = slice_lines(text, anchor_line, window)
    if FREE_RESET_RE.search(block):
        m = FREE_RESET_RE.search(block)
        # find the line offset within block for nicer reporting
        for off, line in enumerate(block.splitlines()):
            if FREE_RESET_RE.search(line):
                return True, (f'{label}: PASS — `messages_control.status = FREE;` '
                              f'found at line {anchor_line + off} '
                              f'(within {window}-line window after anchor at '
                              f'line {anchor_line})')
        return True, f'{label}: PASS'
    return False, (
        f'{label}: FAIL — `messages_control.status = FREE;` NOT found in '
        f'{window}-line window after anchor at line {anchor_line}. '
        f'This is Bug D (see {fact_doc_ref}). Without this reset, the '
        f'FREE-gate downstream silently drops every incoming control frame.\n'
        f'Anchor line: {slice_lines(text, anchor_line, 1)!r}\n'
        f'Window contents:\n{block}'
    )


def main() -> int:
    rsp_path = os.path.join(SRC, 'arq_responder.cc')
    cmd_path = os.path.join(SRC, 'arq_commander.cc')
    for p in (rsp_path, cmd_path):
        if not os.path.exists(p):
            print(f'FAIL: source file missing: {p}')
            return 1

    rsp = open(rsp_path, encoding='utf-8').read()
    cmd = open(cmd_path, encoding='utf-8').read()

    results: list[tuple[bool, str]] = []

    # 1. Bug D fix: RSP BREAK handler resets messages_control.status to FREE.
    #    Anchor: the `if (break_detected == YES && link_status == CONNECTED)`
    #    line that opens the handler block. Window: 40 lines should easily
    #    span the printf + force-FREE + send_ack_pattern + load_configuration.
    results.append(assert_reset_within(
        rsp,
        re.compile(r'if\s*\(\s*break_detected\s*==\s*YES\s*&&\s*'
                   r'link_status\s*==\s*CONNECTED\s*\)'),
        window=40,
        label='RSP BREAK handler (Bug D)',
        fact_doc_ref='SACK_DESIGN_A_PLAN.md §7.13.14, POST_BREAK_STUCK_INVESTIGATION.md'))

    # 2 & 3. CMD-side post-BREAK ACK handlers — pre-existing force-FREEs that
    #    establish the pattern (RSP-side was the missing parallel). Anchor:
    #    the comment "cleanup() skips PENDING_ACK status" that documents both
    #    sites. Window: 5 lines should put the force-FREE right after the
    #    comment.
    cmd_anchor_re = re.compile(r'cleanup\(\)\s*skips\s*PENDING_ACK\s*status')
    cmd_anchors = [(i + 1) for i, ln in enumerate(cmd.splitlines())
                   if cmd_anchor_re.search(ln)]
    if len(cmd_anchors) < 2:
        results.append((False,
            f'CMD-side force-FREE parallels: FAIL — expected 2 occurrences of '
            f'"cleanup() skips PENDING_ACK status" comment in arq_commander.cc; '
            f'found {len(cmd_anchors)}. These mark the BREAK-ACK handlers '
            f'whose pattern Bug D replicates on the RSP side.'))
    else:
        for idx, anchor_line in enumerate(cmd_anchors[:2], start=1):
            block = slice_lines(cmd, anchor_line, 5)
            if FREE_RESET_RE.search(block):
                results.append((True,
                    f'CMD BREAK-ACK handler #{idx}: PASS — force-FREE present '
                    f'after anchor at line {anchor_line}'))
            else:
                results.append((False,
                    f'CMD BREAK-ACK handler #{idx}: FAIL — `messages_control.'
                    f'status = FREE;` not in 5 lines after anchor at line '
                    f'{anchor_line}. This is the CMD-side parallel to Bug D; '
                    f'removing it would re-introduce the post-BREAK stuck '
                    f'state on the commander side.'))

    print('=== BREAK-handler reset regression test ===')
    for ok, msg in results:
        print(('PASS' if ok else 'FAIL') + ': ' + msg)
    n_pass = sum(1 for ok, _ in results if ok)
    n_fail = sum(1 for ok, _ in results if not ok)
    print('---')
    print(f'Results: {n_pass} passed, {n_fail} failed')
    return 0 if n_fail == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
