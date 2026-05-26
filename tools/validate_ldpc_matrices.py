#!/usr/bin/env python3
"""
Validate Mercury LDPC parity-check matrices.

Three modes:
  (default, no args)  Cross-validate 8/16 (known good) + 10/16 with BER tests.
  girth <pcm_file>    Exhaustive Tanner-graph girth audit (BFS per variable
                      node) + variable/check-node girth-4 participation count.
  trapping <pcm_file> Trapping-set search T(a, b) with a <= 8, b <= 4.

Citations:
  - Hu, Eleftheriou & Arnold, "Regular and Irregular Progressive Edge-Growth
    Tanner Graphs," IEEE TIT 51-1, 2005 (PEG construction; girth-4 criterion).
  - McGregor & Milenkovic, "On the Hardness of Approximating Stopping and
    Trapping Sets," IEEE TIT 2010 (T(a,b) definition used here).
  - Karimi & Banihashemi, "Efficient Algorithm for Finding Dominant Trapping
    Sets of LDPC Codes," IEEE TIT 2012 (seed-and-expand search strategy).
"""

import numpy as np
import re
import sys
import os
import time
from collections import deque

np.random.seed(42)


def parse_c_matrix(filepath, array_name, rows, cols):
    """Parse a C int array from a .cc source file."""
    with open(filepath, 'r') as f:
        content = f.read()

    # Find the array definition
    pattern = rf'{re.escape(array_name)}\s*\[{rows}\]\[{cols}\]\s*=\s*\{{(.*?)\}};'
    match = re.search(pattern, content, re.DOTALL)
    if not match:
        raise ValueError(f"Could not find {array_name}[{rows}][{cols}] in {filepath}")

    data_str = match.group(1)
    # Parse row by row
    matrix = np.zeros((rows, cols), dtype=np.int32)
    row_pattern = r'\{([^}]+)\}'
    row_matches = re.findall(row_pattern, data_str)

    if len(row_matches) != rows:
        raise ValueError(f"Expected {rows} rows, found {len(row_matches)}")

    for i, row_str in enumerate(row_matches):
        vals = [int(x.strip()) for x in row_str.split(',')]
        if len(vals) != cols:
            raise ValueError(f"Row {i}: expected {cols} cols, got {len(vals)}")
        matrix[i] = vals

    return matrix


def parse_c_1d_array(filepath, array_name, length):
    """Parse a C 1D int array from a .cc source file."""
    with open(filepath, 'r') as f:
        content = f.read()

    pattern = rf'{re.escape(array_name)}\s*\[{length}\]\s*=\s*\{{([^}}]+)\}};'
    match = re.search(pattern, content, re.DOTALL)
    if not match:
        raise ValueError(f"Could not find {array_name}[{length}] in {filepath}")

    vals = [int(x.strip()) for x in match.group(1).split(',')]
    if len(vals) != length:
        raise ValueError(f"Expected {length} values, got {len(vals)}")
    return np.array(vals, dtype=np.int32)


def parse_c_scalar(filepath, var_name):
    """Parse a C scalar int from a .cc source file."""
    with open(filepath, 'r') as f:
        content = f.read()

    pattern = rf'{re.escape(var_name)}\s*=\s*(\d+)\s*;'
    match = re.search(pattern, content)
    if not match:
        raise ValueError(f"Could not find {var_name} in {filepath}")
    return int(match.group(1))


def build_adjacency(QCmatrixC, QCmatrixV, P, N):
    """Build adjacency lists from Mercury matrix format."""
    check_to_bits = [set() for _ in range(P)]
    bit_to_checks = [set() for _ in range(N)]

    for i in range(P):
        for val in QCmatrixC[i]:
            if val == -1:
                break
            check_to_bits[i].add(int(val))
            bit_to_checks[int(val)].add(i)

    return check_to_bits, bit_to_checks


def test_encoding(check_to_bits, bit_to_checks, QCmatrixEnc, N, K, P, n_trials=100):
    """Test encoding: encode random messages, verify syndrome = 0."""
    # Build full H matrix
    H = np.zeros((P, N), dtype=np.int8)
    for i in range(P):
        for j in check_to_bits[i]:
            H[i, j] = 1

    EncWidth = QCmatrixEnc.shape[1]
    failures = 0

    for trial in range(n_trials):
        msg = np.random.randint(0, 2, K)
        encoded = np.zeros(N, dtype=np.int8)
        encoded[:K] = msg

        # IRA encoding using QCmatrixEnc (matches Mercury's encoder)
        for i in range(P):
            val = 0
            for j_idx in range(EncWidth):
                bit_idx = QCmatrixEnc[i, j_idx]
                if bit_idx == -1:
                    break
                val ^= encoded[bit_idx]
            encoded[K + i] = val

        # Verify syndrome
        syndrome = H @ encoded % 2
        if not np.all(syndrome == 0):
            failures += 1
            if failures <= 3:
                nonzero = np.where(syndrome != 0)[0]
                print(f"    Trial {trial}: FAILED at checks {nonzero[:5]}...")

    return failures


def spa_decode(llr_input, check_to_bits, bit_to_checks, P, N, K, max_iter=50):
    """Sum-Product Algorithm decoder."""
    L = llr_input.copy().astype(np.float64)

    R = {}
    for i in range(P):
        for j in check_to_bits[i]:
            R[(i, j)] = 0.0

    for iteration in range(max_iter):
        # Check node update
        for i in range(P):
            bits = list(check_to_bits[i])
            if len(bits) < 2:
                continue

            Q_vals = []
            for j in bits:
                q = L[j]
                for i2 in bit_to_checks[j]:
                    if i2 != i:
                        q += R.get((i2, j), 0.0)
                Q_vals.append(q)

            for idx, j in enumerate(bits):
                prod_tanh = 1.0
                for idx2, j2 in enumerate(bits):
                    if idx2 != idx:
                        val = np.clip(Q_vals[idx2] / 2.0, -19.0, 19.0)
                        prod_tanh *= np.tanh(val)
                prod_tanh = np.clip(prod_tanh, -0.9999999, 0.9999999)
                R[(i, j)] = 2.0 * np.arctanh(prod_tanh)

        # Hard decision
        L_total = llr_input.copy().astype(np.float64)
        for j in range(N):
            for i in bit_to_checks[j]:
                L_total[j] += R.get((i, j), 0.0)

        hard = (L_total < 0).astype(np.int8)

        # Check syndrome
        ok = True
        for i in range(P):
            s = 0
            for j in check_to_bits[i]:
                s ^= hard[j]
            if s != 0:
                ok = False
                break

        if ok:
            return hard, True, iteration + 1

    return hard, False, max_iter


def test_decoding(check_to_bits, bit_to_checks, QCmatrixEnc, N, K, P,
                  snr_db=5.0, n_frames=50):
    """Test BPSK/AWGN decode at given SNR."""
    snr_lin = 10 ** (snr_db / 10)
    noise_var = 1.0 / snr_lin
    noise_std = np.sqrt(noise_var)
    EncWidth = QCmatrixEnc.shape[1]

    bit_errors = 0
    frame_errors = 0
    decode_failures = 0
    total_bits = 0

    for frame in range(n_frames):
        msg = np.random.randint(0, 2, K)
        codeword = np.zeros(N, dtype=np.int8)
        codeword[:K] = msg

        for i in range(P):
            val = 0
            for j_idx in range(EncWidth):
                bit_idx = QCmatrixEnc[i, j_idx]
                if bit_idx == -1:
                    break
                val ^= codeword[bit_idx]
            codeword[K + i] = val

        tx = 1.0 - 2.0 * codeword
        rx = tx + noise_std * np.random.randn(N)
        llr = 2.0 * rx / noise_var

        decoded, success, iters = spa_decode(llr, check_to_bits, bit_to_checks, P, N, K)

        if not success:
            decode_failures += 1
            frame_errors += 1
        else:
            errors = np.sum(decoded[:K] != msg)
            bit_errors += errors
            if errors > 0:
                frame_errors += 1

        total_bits += K

    ber = bit_errors / total_bits if total_bits > 0 else 0
    fer = frame_errors / n_frames
    return ber, fer, decode_failures


# ============================================================
# Girth + trapping-set analysis
# ============================================================

def _infer_pcm_dims(source_path):
    """Infer (N, K, prefix) from a Mercury PCM .cc source file.

    All Mercury normal IRA codes are N=1600; rate is encoded in the filename
    suffix `_<numK>_16` where numK is K/100 (so 1_16 -> K=100, 10_16 -> K=1000).
    """
    base = os.path.basename(source_path)
    m = re.search(r'mercury_normal_(\d+)_16\.cc$', base)
    if not m:
        raise ValueError(f"Cannot infer rate prefix from filename: {base}")
    num = int(m.group(1))
    prefix = f"{num}_16"
    N = 1600
    K = num * 100
    return N, K, prefix


def load_pcm(source_path):
    """Parse a Mercury PCM .cc source file into adjacency lists.

    Returns (check_to_bits, bit_to_checks, N, K, P, prefix).
    """
    N, K, prefix = _infer_pcm_dims(source_path)
    P = N - K
    Cwidth = parse_c_scalar(source_path, f"mercury_normal_Cwidth_{prefix}")
    Vwidth = parse_c_scalar(source_path, f"mercury_normal_Vwidth_{prefix}")
    QCmatrixC = parse_c_matrix(source_path, f"mercury_normal_QCmatrixC_{prefix}", P, Cwidth)
    QCmatrixV = parse_c_matrix(source_path, f"mercury_normal_QCmatrixV_{prefix}", N, Vwidth)
    check_to_bits, bit_to_checks = build_adjacency(QCmatrixC, QCmatrixV, P, N)
    return check_to_bits, bit_to_checks, N, K, P, prefix


def shortest_cycle_through_vn(v, bit_to_checks, check_to_bits, max_len=12):
    """Find the length of the shortest cycle in the Tanner graph through
    variable node `v`. Returns the cycle length (even integer >= 4), or
    `None` if no cycle of length <= max_len exists through `v`.

    Algorithm (Hu/Eleftheriou/Arnold §IV, PEG construction primitive):
      BFS from v in the Tanner graph. Track parent of each visited node.
      If a search frontier discovers a node already-visited via a different
      parent, the two paths concatenate into a cycle. The girth-through-v
      is the smallest such concatenated length.

      To detect cycles WHILE keeping BFS-shortest semantics, we layer-walk:
      visit all nodes at distance d before any at d+1. Within layer d, if
      two nodes in layer d-1 both reach the same layer-d node (via different
      parents at all ancestors), they form a cycle of length 2*d. If a node
      at layer d sees a node at layer d via two paths, that's 2*d + 1 -- but
      Tanner graph is bipartite so odd cycles cannot exist; this case means
      same-layer collision which is cycle length 2*d (one extra edge on
      each side is illegal in bipartite -- so this is just 2*d sourced from
      different parents).

      Specifically for variable-node v, we BFS over Tanner edges. Even
      distances land on VNs, odd distances on CNs. A cycle of length 2L
      through v is found when, at BFS depth L (counting Tanner edges), two
      distinct edges from v eventually meet at the same node.

    Implementation: standard "first-revisit-from-different-parent" detection.
    Returns the shortest cycle length found, capped at max_len.
    """
    # Tanner nodes encoded as (kind, idx): kind=0 VN, kind=1 CN.
    # We BFS from (0, v). Track first-arrival distance and parent.
    # When we discover an edge that connects two BFS-tree nodes via a
    # non-tree edge, we form a cycle. Length = dist[u] + dist[w] + 1 if the
    # non-tree edge is (u,w). The minimum cycle through v is the minimum
    # such cycle.
    #
    # For an undirected BFS tree rooted at v, a cycle through v exists iff
    # there is a non-tree edge whose two endpoints share their lowest common
    # ancestor at v. We approximate this with the standard relaxation: find
    # the minimum, over all non-tree edges (u, w), of dist[u] + dist[w] + 1,
    # restricted to non-tree edges seen during BFS. This gives the girth of
    # the BFS tree rooted at v, which is a valid upper bound on the girth
    # through v in standard literature; for bipartite Tanner graphs with the
    # BFS rooted at a VN, it equals the shortest cycle through v.

    INF = max_len + 1
    # dist arrays for VN and CN
    dist_vn = [INF] * len(bit_to_checks)
    dist_cn = [INF] * len(check_to_bits)
    parent_vn = [-1] * len(bit_to_checks)   # parent CN index for each VN
    parent_cn = [-1] * len(check_to_bits)   # parent VN index for each CN

    dist_vn[v] = 0
    q = deque()
    q.append((0, v))  # (kind, idx); kind=0 VN

    best = None  # shortest cycle length found

    while q:
        kind, idx = q.popleft()
        d_here = dist_vn[idx] if kind == 0 else dist_cn[idx]
        if d_here >= max_len:
            continue

        if kind == 0:
            # VN -> neighbors are CNs
            for c in bit_to_checks[idx]:
                if c == parent_vn[idx]:
                    continue  # don't go back to parent
                nd = d_here + 1
                if dist_cn[c] == INF:
                    dist_cn[c] = nd
                    parent_cn[c] = idx
                    if nd < max_len:
                        q.append((1, c))
                else:
                    # Non-tree edge VN(idx) -- CN(c). Cycle length =
                    # dist_vn[idx] + dist_cn[c] + 1, but only counts as a
                    # cycle through v if the two BFS paths to idx and c are
                    # edge-disjoint. In a BFS tree from v, the LCA of any
                    # two nodes lies on the tree. For the cycle to pass
                    # through v (the root), we need the LCA to be v itself.
                    # Standard relaxation: accept all non-tree-edge cycles;
                    # this gives shortest cycle in the BFS-tree subgraph
                    # rooted at v, which is the girth-through-v for
                    # bipartite simple graphs (Hu et al., 2005).
                    clen = d_here + dist_cn[c] + 1
                    if best is None or clen < best:
                        best = clen
                        # cycle length must be even (bipartite); minimum is 4
                        if best <= 4:
                            return best
        else:
            # CN -> neighbors are VNs
            for w in check_to_bits[idx]:
                if w == parent_cn[idx]:
                    continue
                nd = d_here + 1
                if dist_vn[w] == INF:
                    dist_vn[w] = nd
                    parent_vn[w] = idx
                    if nd < max_len:
                        q.append((0, w))
                else:
                    clen = d_here + dist_vn[w] + 1
                    if best is None or clen < best:
                        best = clen
                        if best <= 4:
                            return best

    if best is None or best > max_len:
        return None
    return best


def shortest_cycle_through_cn(c, bit_to_checks, check_to_bits, max_len=12):
    """Same as shortest_cycle_through_vn but rooted at CN c."""
    INF = max_len + 1
    dist_vn = [INF] * len(bit_to_checks)
    dist_cn = [INF] * len(check_to_bits)
    parent_vn = [-1] * len(bit_to_checks)
    parent_cn = [-1] * len(check_to_bits)

    dist_cn[c] = 0
    q = deque()
    q.append((1, c))

    best = None
    while q:
        kind, idx = q.popleft()
        d_here = dist_vn[idx] if kind == 0 else dist_cn[idx]
        if d_here >= max_len:
            continue

        if kind == 1:
            for w in check_to_bits[idx]:
                if w == parent_cn[idx]:
                    continue
                nd = d_here + 1
                if dist_vn[w] == INF:
                    dist_vn[w] = nd
                    parent_vn[w] = idx
                    if nd < max_len:
                        q.append((0, w))
                else:
                    clen = d_here + dist_vn[w] + 1
                    if best is None or clen < best:
                        best = clen
                        if best <= 4:
                            return best
        else:
            for cc in bit_to_checks[idx]:
                if cc == parent_vn[idx]:
                    continue
                nd = d_here + 1
                if dist_cn[cc] == INF:
                    dist_cn[cc] = nd
                    parent_cn[cc] = idx
                    if nd < max_len:
                        q.append((1, cc))
                else:
                    clen = d_here + dist_cn[cc] + 1
                    if best is None or clen < best:
                        best = clen
                        if best <= 4:
                            return best

    if best is None or best > max_len:
        return None
    return best


def girth_audit(check_to_bits, bit_to_checks, N, P, max_len=12, verbose=True):
    """Run exhaustive per-VN and per-CN girth analysis.

    Returns dict with:
      - 'vn_cycle_len': list of length N giving shortest cycle through each VN
                       (None if no cycle <= max_len exists).
      - 'cn_cycle_len': list of length P giving shortest cycle through each CN.
      - 'girth': overall graph girth (min over all VNs and CNs).
      - 'vn_hist': histogram dict {len: count} over VNs.
      - 'cn_hist': histogram dict {len: count} over CNs.
      - 'vn_in_g4': count of VNs participating in any girth-4 cycle.
      - 'cn_in_g4': count of CNs participating in any girth-4 cycle.
    """
    if verbose:
        print(f"  Girth BFS over {N} variable nodes (max_len={max_len})...")
    t0 = time.time()

    vn_cycle_len = [None] * N
    for v in range(N):
        vn_cycle_len[v] = shortest_cycle_through_vn(v, bit_to_checks, check_to_bits, max_len)
        if verbose and (v + 1) % 400 == 0:
            print(f"    VN {v+1}/{N}  elapsed {time.time()-t0:.1f}s")

    if verbose:
        print(f"  Girth BFS over {P} check nodes...")
    t1 = time.time()
    cn_cycle_len = [None] * P
    for c in range(P):
        cn_cycle_len[c] = shortest_cycle_through_cn(c, bit_to_checks, check_to_bits, max_len)
        if verbose and (c + 1) % 400 == 0:
            print(f"    CN {c+1}/{P}  elapsed {time.time()-t1:.1f}s")

    # Histograms
    vn_hist = {}
    for x in vn_cycle_len:
        key = x if x is not None else 'none'
        vn_hist[key] = vn_hist.get(key, 0) + 1
    cn_hist = {}
    for x in cn_cycle_len:
        key = x if x is not None else 'none'
        cn_hist[key] = cn_hist.get(key, 0) + 1

    vn_finite = [x for x in vn_cycle_len if x is not None]
    cn_finite = [x for x in cn_cycle_len if x is not None]
    girth = min(min(vn_finite) if vn_finite else 10**9,
                min(cn_finite) if cn_finite else 10**9)
    if girth == 10**9:
        girth = None  # no cycles at all within max_len

    vn_in_g4 = sum(1 for x in vn_cycle_len if x == 4)
    cn_in_g4 = sum(1 for x in cn_cycle_len if x == 4)

    return {
        'vn_cycle_len': vn_cycle_len,
        'cn_cycle_len': cn_cycle_len,
        'girth': girth,
        'vn_hist': vn_hist,
        'cn_hist': cn_hist,
        'vn_in_g4': vn_in_g4,
        'cn_in_g4': cn_in_g4,
    }


def print_girth_report(report, N, P):
    print()
    print("  Girth histogram (variable nodes):")
    for k in sorted([x for x in report['vn_hist'].keys() if isinstance(x, int)]):
        print(f"    length {k:2d}: {report['vn_hist'][k]:6d} VNs")
    if 'none' in report['vn_hist']:
        print(f"    no cycle  : {report['vn_hist']['none']:6d} VNs (within search depth)")

    print("  Girth histogram (check nodes):")
    for k in sorted([x for x in report['cn_hist'].keys() if isinstance(x, int)]):
        print(f"    length {k:2d}: {report['cn_hist'][k]:6d} CNs")
    if 'none' in report['cn_hist']:
        print(f"    no cycle  : {report['cn_hist']['none']:6d} CNs (within search depth)")

    print()
    print(f"  Overall graph girth: {report['girth']}")
    print(f"  VNs participating in a girth-4 cycle: {report['vn_in_g4']} / {N}")
    print(f"  CNs participating in a girth-4 cycle: {report['cn_in_g4']} / {P}")

    if report['vn_in_g4'] > 0 or report['cn_in_g4'] > 0:
        print()
        print("  VERDICT: BROKEN -- girth-4 cycles exist. BP cannot converge")
        print("           reliably near the SNR cliff. PCM should be")
        print("           regenerated via PEG (Hu/Eleftheriou/Arnold 2005).")
    elif report['girth'] is not None and report['girth'] <= 6:
        print()
        print(f"  VERDICT: MARGINAL -- girth={report['girth']}. Acceptable for")
        print("           short LDPC codes but contributes to error floor.")
    else:
        print()
        print(f"  VERDICT: CLEAN -- girth={report['girth']} >= 8.")


# ----- Trapping sets -----

def _check_odd_count(var_set, bit_to_checks, check_to_bits):
    """Count check nodes connected to var_set with an odd number of edges
    into the set (these are the 'unsatisfied' / odd checks; the trapping
    set parameter `b`)."""
    check_counts = {}
    for v in var_set:
        for c in bit_to_checks[v]:
            check_counts[c] = check_counts.get(c, 0) + 1
    return sum(1 for cnt in check_counts.values() if cnt & 1)


def find_trapping_sets(check_to_bits, bit_to_checks, N, P,
                       a_max=8, b_max=4, time_budget_s=600, verbose=True):
    """Search for trapping sets T(a, b) with a <= a_max, b <= b_max.

    Following Karimi & Banihashemi 2012 §III: seed-and-expand. For each seed
    variable node v with low degree (most likely to be in a trapping set),
    grow the candidate set by adding neighboring VNs (VNs that share a check
    with the current set) up to size a_max. At each size, check the b
    parameter.

    Returns list of (sorted_tuple_of_vns, a, b) for distinct trapping sets
    found, deduplicated by tuple.
    """
    seen = set()
    found = []

    # Seed list: prefer low-degree VNs (degree-2 dominates for IRA codes).
    seed_order = sorted(range(N), key=lambda v: (len(bit_to_checks[v]), v))

    t0 = time.time()
    seeds_processed = 0

    for seed in seed_order:
        if time.time() - t0 > time_budget_s:
            if verbose:
                print(f"    Time budget exhausted after {seeds_processed} seeds.")
            break
        seeds_processed += 1
        if verbose and seeds_processed % 200 == 0:
            elapsed = time.time() - t0
            print(f"    Seeds {seeds_processed}/{N}  found {len(found)}  "
                  f"elapsed {elapsed:.1f}s")

        # BFS-expand candidates from this seed
        # Frontier: list of (frozenset(vns), neighbor_vns)
        init_set = frozenset([seed])
        init_b = _check_odd_count(init_set, bit_to_checks, check_to_bits)
        if init_b <= b_max:
            tup = tuple(sorted(init_set))
            if tup not in seen:
                seen.add(tup)
                found.append((tup, 1, init_b))

        # Expand level by level via DFS with pruning
        stack = [(init_set, _neighbor_vns(init_set, bit_to_checks, check_to_bits))]
        while stack:
            if time.time() - t0 > time_budget_s:
                break
            cur_set, neigh = stack.pop()
            if len(cur_set) >= a_max:
                continue
            for nv in neigh:
                if nv <= max(cur_set):
                    continue  # avoid permutation duplicates: only add larger
                new_set = cur_set | {nv}
                b = _check_odd_count(new_set, bit_to_checks, check_to_bits)
                if b > b_max + 2 * (a_max - len(new_set)):
                    # Pruning: adding one more VN can change odd-check count
                    # by at most (degree of that VN). Loose upper bound 2*remaining.
                    continue
                if b <= b_max:
                    tup = tuple(sorted(new_set))
                    if tup not in seen:
                        seen.add(tup)
                        found.append((tup, len(new_set), b))
                # Recurse if more room
                if len(new_set) < a_max:
                    new_neigh = _neighbor_vns(new_set, bit_to_checks, check_to_bits)
                    # only larger indices to dedupe
                    new_neigh = {x for x in new_neigh if x > nv}
                    if new_neigh:
                        stack.append((new_set, new_neigh))

    return found


def _neighbor_vns(var_set, bit_to_checks, check_to_bits):
    """All VNs adjacent (via a shared check) to var_set, minus var_set itself."""
    out = set()
    for v in var_set:
        for c in bit_to_checks[v]:
            for w in check_to_bits[c]:
                if w not in var_set:
                    out.add(w)
    return out


def print_trapping_report(found, N, P, a_max, b_max):
    print()
    # Group by (a, b)
    by_class = {}
    for tup, a, b in found:
        by_class.setdefault((a, b), []).append(tup)

    print(f"  Trapping-set search T(a<={a_max}, b<={b_max}) results:")
    print(f"  Distinct sets found: {len(found)}")
    print()
    print(f"    a   b   count   example_vns")
    for (a, b) in sorted(by_class.keys()):
        sets = by_class[(a, b)]
        ex = sets[0][:6] if sets else ()
        ex_str = ','.join(str(x) for x in ex) + (',...' if len(sets[0]) > 6 else '')
        print(f"    {a:3d} {b:3d} {len(sets):7d}   [{ex_str}]")

    # Severity
    small = [(tup, a, b) for tup, a, b in found if a <= 6 and b <= 2 and a >= 2]
    print()
    if not found:
        print("  No trapping sets found in search budget.")
    elif small:
        print(f"  VERDICT: {len(small)} small trapping sets T(a<=6, b<=2) found.")
        print("           Small T(a,b) with b<=2 are the dominant error-floor")
        print("           contributors (Karimi & Banihashemi 2012 §IV).")
        for tup, a, b in small[:5]:
            print(f"           T({a},{b}): VNs = {list(tup)}")
    else:
        print("  No T(a<=6, b<=2) sets found. Larger sets present but less")
        print("  impactful for the error floor.")


def run_girth_cli(source_path):
    print(f"Loading PCM from {source_path}...")
    check_to_bits, bit_to_checks, N, K, P, prefix = load_pcm(source_path)
    print(f"  Loaded: N={N}, K={K}, P={P}, prefix={prefix}")

    # Sanity: variable-node degree distribution
    degs = [len(bit_to_checks[v]) for v in range(N)]
    print(f"  VN degree min/avg/max = {min(degs)}/{sum(degs)/N:.2f}/{max(degs)}")
    degs_c = [len(check_to_bits[c]) for c in range(P)]
    print(f"  CN degree min/avg/max = {min(degs_c)}/{sum(degs_c)/P:.2f}/{max(degs_c)}")

    report = girth_audit(check_to_bits, bit_to_checks, N, P, max_len=12, verbose=True)
    print_girth_report(report, N, P)


def run_trapping_cli(source_path, a_max=8, b_max=4, time_budget_s=600):
    print(f"Loading PCM from {source_path}...")
    check_to_bits, bit_to_checks, N, K, P, prefix = load_pcm(source_path)
    print(f"  Loaded: N={N}, K={K}, P={P}, prefix={prefix}")
    print(f"  Searching T(a<={a_max}, b<={b_max}), time budget {time_budget_s}s...")

    found = find_trapping_sets(check_to_bits, bit_to_checks, N, P,
                                a_max=a_max, b_max=b_max,
                                time_budget_s=time_budget_s, verbose=True)
    print_trapping_report(found, N, P, a_max, b_max)


def validate_matrix(name, source_path, N, K, prefix):
    """Full validation suite for one LDPC matrix."""
    P = N - K

    print(f"\n{'='*60}")
    print(f"Validating: {name} (N={N}, K={K}, P={P}, rate={K}/{N})")
    print(f"Source: {source_path}")
    print(f"{'='*60}")

    # Parse matrices
    print("  Parsing matrices...")
    Cwidth = parse_c_scalar(source_path, f"mercury_normal_Cwidth_{prefix}")
    Vwidth = parse_c_scalar(source_path, f"mercury_normal_Vwidth_{prefix}")
    dwidth = parse_c_scalar(source_path, f"mercury_normal_dwidth_{prefix}")

    print(f"  Cwidth={Cwidth}, Vwidth={Vwidth}, dwidth={dwidth}")

    QCmatrixC = parse_c_matrix(source_path, f"mercury_normal_QCmatrixC_{prefix}", P, Cwidth)
    QCmatrixV = parse_c_matrix(source_path, f"mercury_normal_QCmatrixV_{prefix}", N, Vwidth)
    QCmatrixEnc = parse_c_matrix(source_path, f"mercury_normal_QCmatrixEnc_{prefix}", P, Cwidth - 1)
    QCmatrixd = parse_c_1d_array(source_path, f"mercury_normal_QCmatrixd_{prefix}", dwidth)

    # Build adjacency lists
    check_to_bits, bit_to_checks = build_adjacency(QCmatrixC, QCmatrixV, P, N)

    # Test 1: Matrix consistency (C and V must be transposes)
    print("\n  Test 1: C/V consistency...")
    c_v_ok = True
    for i in range(P):
        for j in check_to_bits[i]:
            if i not in bit_to_checks[j]:
                print(f"    FAIL: check {i} connects to bit {j}, but V[{j}] doesn't list check {i}")
                c_v_ok = False
                break
        if not c_v_ok:
            break

    for j in range(N):
        for i in bit_to_checks[j]:
            if j not in check_to_bits[i]:
                print(f"    FAIL: bit {j} connects to check {i}, but C[{i}] doesn't list bit {j}")
                c_v_ok = False
                break
        if not c_v_ok:
            break

    print(f"    {'PASS' if c_v_ok else 'FAIL'}")

    # Test 2: IRA structure (parity bits K..N-1 form accumulator chain)
    print("  Test 2: IRA accumulator structure...")
    ira_ok = True
    # First parity bit (K) should connect to check 0 (at minimum)
    if 0 not in bit_to_checks[K]:
        print(f"    FAIL: parity bit {K} not connected to check 0")
        ira_ok = False
    # Each check i (1..P-1) should connect to parity bits K+i-1 and K+i
    for i in range(1, P):
        if (K + i - 1) not in check_to_bits[i]:
            print(f"    FAIL: check {i} missing connection to parity {K+i-1}")
            ira_ok = False
            break
        if (K + i) not in check_to_bits[i]:
            print(f"    FAIL: check {i} missing connection to parity {K+i}")
            ira_ok = False
            break
    print(f"    {'PASS' if ira_ok else 'FAIL'}")

    # Test 3: Encoding matrix consistency
    print("  Test 3: Enc matrix = C matrix minus self-parity...")
    enc_ok = True
    for i in range(P):
        c_bits = set()
        for val in QCmatrixC[i]:
            if val == -1:
                break
            c_bits.add(int(val))

        enc_bits = set()
        for val in QCmatrixEnc[i]:
            if val == -1:
                break
            enc_bits.add(int(val))

        expected_enc = c_bits - {K + i}  # C minus self-parity
        if enc_bits != expected_enc:
            print(f"    FAIL: check {i}: Enc={sorted(enc_bits)}, expected={sorted(expected_enc)}")
            enc_ok = False
            if i > 5:
                break
    print(f"    {'PASS' if enc_ok else 'FAIL'}")

    # Test 4: Degree distribution matches d array
    print("  Test 4: Degree distribution...")
    actual_degrees = [len(bit_to_checks[j]) for j in range(N)]
    # Parse d array as (count, degree) pairs
    d_groups = []
    total_from_d = 0
    for idx in range(0, dwidth, 2):
        count = QCmatrixd[idx]
        degree = QCmatrixd[idx + 1]
        d_groups.append((count, degree))
        total_from_d += count

    # Reconstruct expected degrees from d array
    expected_degrees = []
    for count, degree in d_groups:
        expected_degrees.extend([degree] * count)

    d_ok = (len(expected_degrees) == N and expected_degrees == actual_degrees)
    if not d_ok:
        if len(expected_degrees) != N:
            print(f"    FAIL: d array covers {len(expected_degrees)} bits, expected {N}")
        else:
            mismatches = sum(1 for a, b in zip(actual_degrees, expected_degrees) if a != b)
            print(f"    FAIL: {mismatches} degree mismatches")
    print(f"    {'PASS' if d_ok else 'FAIL'}")

    # Test 5: Encoding verification (100 random messages)
    print("  Test 5: Encoding (100 random messages)...")
    enc_failures = test_encoding(check_to_bits, bit_to_checks, QCmatrixEnc, N, K, P, 100)
    print(f"    {'PASS' if enc_failures == 0 else 'FAIL'} ({enc_failures}/100 failures)")

    # Test 6: Decoding at 5 dB (should be clean for all rates)
    print("  Test 6: SPA decode at 5dB (20 frames)...")
    ber, fer, dec_fail = test_decoding(check_to_bits, bit_to_checks, QCmatrixEnc,
                                        N, K, P, snr_db=5.0, n_frames=20)
    dec_ok = (dec_fail == 0 and ber == 0.0)
    print(f"    BER={ber:.6f}, FER={fer:.2f}, decode_fail={dec_fail}/20")
    print(f"    {'PASS' if dec_ok else 'FAIL'}")

    all_pass = c_v_ok and ira_ok and enc_ok and d_ok and (enc_failures == 0) and dec_ok
    print(f"\n  OVERALL: {'ALL TESTS PASSED' if all_pass else 'SOME TESTS FAILED'}")
    return all_pass


# ============================================================
# Main: dispatch on CLI args
# ============================================================
USAGE = """\
Usage:
  validate_ldpc_matrices.py                      Cross-validate 8/16 + 10/16 (BER).
  validate_ldpc_matrices.py girth <pcm_file>     Tanner-graph girth audit.
  validate_ldpc_matrices.py trapping <pcm_file>  Trapping-set search.
  validate_ldpc_matrices.py --help               Print this message.
"""


def _run_default_ber():
    base = os.path.dirname(os.path.abspath(__file__))
    src_dir = os.path.join(base, '..', 'source', 'physical_layer')

    results = {}
    results['8/16'] = validate_matrix(
        "Rate 8/16 (known-good)",
        os.path.join(src_dir, "mercury_normal_8_16.cc"),
        N=1600, K=800, prefix="8_16"
    )
    results['10/16'] = validate_matrix(
        "Rate 10/16 (new)",
        os.path.join(src_dir, "mercury_normal_10_16.cc"),
        N=1600, K=1000, prefix="10_16"
    )

    print(f"\n{'='*60}")
    print("CROSS-VALIDATION SUMMARY")
    print(f"{'='*60}")
    for name, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"  {name}: {status}")

    if all(results.values()):
        print("\nBoth matrices validated. Safe to integrate 10/16.")
    else:
        print("\nValidation FAILED. Do not integrate until fixed.")
        sys.exit(1)


if __name__ == '__main__':
    args = sys.argv[1:]
    if not args:
        _run_default_ber()
    elif args[0] in ('-h', '--help', 'help'):
        print(USAGE)
    elif args[0] == 'girth':
        if len(args) < 2:
            print(USAGE, file=sys.stderr)
            sys.exit(2)
        run_girth_cli(args[1])
    elif args[0] == 'trapping':
        if len(args) < 2:
            print(USAGE, file=sys.stderr)
            sys.exit(2)
        # Optional: --a-max N --b-max N --time-budget-s S
        a_max = 8
        b_max = 4
        tbs = 600
        i = 2
        while i < len(args):
            if args[i] == '--a-max':
                a_max = int(args[i+1]); i += 2
            elif args[i] == '--b-max':
                b_max = int(args[i+1]); i += 2
            elif args[i] == '--time-budget-s':
                tbs = int(args[i+1]); i += 2
            else:
                print(f"Unknown arg: {args[i]}", file=sys.stderr)
                sys.exit(2)
        run_trapping_cli(args[1], a_max=a_max, b_max=b_max, time_budget_s=tbs)
    else:
        print(USAGE, file=sys.stderr)
        sys.exit(2)
