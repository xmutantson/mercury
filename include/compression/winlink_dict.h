/*
 * Winlink-boilerplate priming dictionary — v1 UNIVERSAL static dict.
 *
 * A STATIC, firmware-baked dictionary of the common, deterministic scaffolding
 * that prefixes/structures essentially every Winlink B2F message: the RFC822-ish
 * header block, the //WL2K form markers, the most-common standard form templates
 * (ICS-213, ARRL radiogram, Winlink check-in, ICS-309, field situation report),
 * and standard amateur sign-offs. These bytes are IDENTICAL on TX and RX (baked
 * into both ends, zero wire cost), so the streaming compressor seeded with them
 * can reference them as LZ back-matches / PPMd model context even on the FIRST
 * small message — exactly the warm-up a cold stream lacks.
 *
 * Two artifacts are baked in (see winlink_dict.cc):
 *   WINLINK_DICT_RAW          — the raw dictionary plaintext.
 *   WINLINK_DICT_COMPRESSED   — the canonical streaming-compressed form of
 *                               WINLINK_DICT_RAW produced by the production
 *                               cl_compressor on a fresh stream. Decompressing
 *                               this on EVERY peer warms the shared PPMd model
 *                               with a single deterministic forward pass that is
 *                               byte-identical on TX and RX (role-independent),
 *                               then streaming_commit(RAW) loads the zstd prefix.
 *
 * VERSION-LOCK: WINLINK_DICT_VERSION identifies the EXACT dict bytes. It is
 * stamped into the streaming-header dict-tag (algo_flags bits 5-7) on every TX
 * frame; an RX whose active dict version differs falls back to the cold path
 * (no priming, no corruption). Version 0 is RESERVED = "no dict / cold". v1 is
 * the universal dict below. Bumping the dict bytes MUST bump this version, and
 * WINLINK_DICT_COMPRESSED MUST be regenerated for the new bytes
 * (tools/gen_winlink_dict.cc).
 *
 * Ordering note: the MOST common / most-recently-useful boilerplate is placed
 * LAST, because both LZ77 back-references and PPMd context favor recent history
 * (the end of the prefix window is "closest" to the message being compressed).
 */
#ifndef WINLINK_DICT_H
#define WINLINK_DICT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Universal dictionary version. 0 = reserved (no dict / cold). 1..7 valid
 * (3-bit field in the streaming header). Bump on ANY change to WINLINK_DICT_RAW.
 *   v1: hand-curated 3703B boilerplate (RFC822/MIME + ICS form scaffolding).
 *   v2: ZDICT-trained generalizable Winlink boilerplate (METAR/TAF, NWS forecast
 *       tables, Saildocs, email-quote structure, MIME envelope) mined from a real
 *       transmitted-traffic corpus + PII-scrubbed, combined with the v1 form
 *       scaffolding. 15191B raw (operator-specific signature filler stripped).
 *       ~1.39x more wire reduction on held-out small messages (held-out
 *       small-regime x-VARA 1.64->2.27). See
 *       tools/winlink_dict_v2.txt + fact-documents/data-flow-winlink-dict-priming.md. */
#define WINLINK_DICT_VERSION       2
#define WINLINK_DICT_VERSION_NONE  0   /* cold / no-dict tag in a streaming frame */
#define WINLINK_DICT_VERSION_MAX   7   /* 3-bit header field cap */

/* Raw dictionary plaintext (NUL-terminated; length excludes the NUL). */
extern const char         WINLINK_DICT_RAW[];
extern const unsigned int WINLINK_DICT_RAW_LEN;

/* Canonical streaming-compressed form of WINLINK_DICT_RAW (produced by the
 * production cl_compressor on a fresh stream — see tools/gen_winlink_dict.cc).
 * Decompressing this warms the PPMd model identically on every peer. */
extern const unsigned char WINLINK_DICT_COMPRESSED[];
extern const unsigned int  WINLINK_DICT_COMPRESSED_LEN;

#ifdef __cplusplus
}
#endif

#endif
