/*
 * Mercury: A configurable open-source software-defined modem.
 * Copyright (C) 2022-2024 Fadi Jerji
 * Author: Fadi Jerji
 * Email: fadi.jerji@  <gmail.com, caisresearch.com, ieee.org>
 * ORCID: 0000-0002-2076-5831
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "datalink_layer/arq.h"
#include "audioio/audioio.h"
#include "debug/canary_guard.h"
#include <time.h>
#ifdef __GLIBC__
#include <malloc.h>
#endif
#include "common/timing_log.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <chrono>
#include <cstdlib>

extern "C" {
    extern double noise_snr_db;
    extern double noise_signal_dbfs;
}

#ifdef MERCURY_GUI_ENABLED
#include "gui/gui_state.h"
#endif

// SACK_RX_TRACE: env-gated diagnostic for the SACK_RSP receive path.
// Mirror of the macro in arq_commander.cc. Enable via MERCURY_SACK_RX_TRACE=1.
static inline bool sack_rx_trace_enabled_common()
{
	static int cached = -1;
	if(cached < 0)
	{
		const char* e = std::getenv("MERCURY_SACK_RX_TRACE");
		cached = (e && *e && *e != '0') ? 1 : 0;
	}
	return cached != 0;
}
#define SACK_TRACE(fmt, ...) do { \
	if(sack_rx_trace_enabled_common()) { \
		printf("[SACK-RX-TRACE] " fmt "\n", ##__VA_ARGS__); \
		fflush(stdout); \
	} \
} while(0)

extern cbuf_handle_t capture_buffer;
extern cbuf_handle_t playback_buffer;

// ---------------------------------------------------------------------------
// SIM virtual-clock spin helpers (sim-arq-channel.md §Inc2).
//
// Two loop shapes recur ~24x across the TX path:
//   (1) PTT pre/post-key delay:  while(t.get_elapsed_time_ms() < ms) msleep(1);
//   (2) playback drain:          while(size_buffer(playback_buffer) > 0) msleep(1);
//
// Under -x sim, cl_timer already measures VIRTUAL channel time and the drain
// is serviced by the TX bridge thread, so both loops auto-virtualize: the PTT
// spin exits as soon as VIRTUAL time passes delay_ms (which, with virtual time
// advancing ~5-10x faster than wall-clock, takes a fraction of the real
// delay), and the drain exits as soon as the bridge has shipped the frame.
//
// The poll granularity matters. A first cut used std::this_thread::yield()
// (no sleep). That ran the ARQ thread, the RX-prep thread, the TX bridge and
// the RX bridge ALL hot-spinning at max frequency, which made the CMD<->RSP
// PTT turnaround timing race: the two peers are SEPARATE processes with
// SEPARATE virtual clocks coupled only through the real-time relay, and
// adversarial hot-spinning let one peer's virtual clock sprint past the
// other's, corrupting the half-duplex turnaround (RSP's HAIL response was
// dropped by a CMD that had already raced through its ptt_off_delay). It also
// pegged a core to no purpose. So in SIM mode we sleep a SHORT real interval
// (SIM_SPIN_SLEEP_US) instead of yielding: it hands the core to the sibling
// threads, keeps the two processes loosely paced together (so their virtual
// clocks stay coupled via the relay), and is far shorter than the virtual
// durations being waited — so the faster-than-real-time speed-up is preserved
// (the loop still exits on the VIRTUAL condition, the sleep only bounds the
// poll overshoot to <1 ms). In PRODUCTION (sim disabled) both helpers do
// EXACTLY the stock `msleep(1)` — byte-identical behavior. Factored to ONE
// definition each so the policy lives in a single place.
static const int SIM_SPIN_SLEEP_US = 200;
static inline void sim_spin_sleep()
{
	std::this_thread::sleep_for(std::chrono::microseconds(SIM_SPIN_SLEEP_US));
}

// ---------------------------------------------------------------------------
// SIM_INPROC step-pump hook (single-process-sim-refactor.md §2).
//
// In the two-process paced sim the spin-loops below exit because CONCURRENT
// sibling threads make their exit condition true (the TX bridge drains
// playback_buffer; the RX bridge + capture-prep thread advance the virtual
// clock via rx_transfer). A SINGLE-THREAD stepper has no sibling threads, so
// without help the waits would block forever inside process_main → DEADLOCK.
//
// g_sim_inproc_pump is a step-pump callback the SIM_INPROC stepper installs.
// When non-null (ONLY under -m SIM_INPROC), the spin-loops call it instead of
// sim_spin_sleep(): from INSIDE the wait it re-enters the loopback-drain +
// clock-advance the stepper would otherwise do after process_main returns.
//
// CRITICAL FIDELITY GUARD: the pump only makes the EXISTING exit predicate
// eventually true — it NEVER changes the threshold and NEVER lets the loop
// exit early. ptt_busy_wait still exits at get_elapsed_time_ms() >= delay_ms;
// drain_playback_wait still exits at size_buffer(playback_buffer) == 0. The
// pump advances the clock through the SAME rx_transfer → sim_clock_add_samples
// accounting as the capture-prep thread, so the exit instant is identical to
// the two-process path.
//
// Default null. Production (-x wasapi/alsa) and the two-process paced sim
// never install it, so their spin-loop bodies are BYTE-IDENTICAL to before
// (sim_spin_sleep() under sim, msleep(1) otherwise).
typedef void (*sim_inproc_pump_fn)(void* ctx);
static sim_inproc_pump_fn g_sim_inproc_pump = nullptr;
static void*              g_sim_inproc_pump_ctx = nullptr;

void arq_set_sim_inproc_pump(sim_inproc_pump_fn fn, void* ctx)
{
	g_sim_inproc_pump = fn;
	g_sim_inproc_pump_ctx = ctx;
}

// SIM_INPROC post-delivery spin-abort (simftr stepper-wedge fix, fact-documents/
// SIMFTR_ROOTCAUSE.md §7 fix #1). The 2-instance stepper's outer-loop delivery check
// (rx_have >= payload_len, arq_commander.cc test_sim_inproc_2) lives at the loop
// BOTTOM, after both peers' process_main() calls. A complete one-batch transfer whose
// post-ACK turnaround leaves the COMMANDER in TRANSMITTING_DATA can fall into a
// self-sustaining idle-keepalive ↔ pumped-wait cycle INSIDE one process_main() call,
// so process_main never returns and the outer delivery check is never re-reached —
// even though the RESPONDER already holds every payload byte in fifo_buffer_rx (the
// transfer SUCCEEDED). The step-pump runs DURING that spin; when it observes the
// responder holding the full payload it sets this flag, and the spin helpers below
// return early so process_main unwinds back to the outer loop, which then drains the
// FIFO and breaks on the (now-true) delivery predicate.
//
// FIDELITY: this is a HARNESS-only early-out that fires ONLY post-delivery — by the
// time it is armed the responder already has all bytes, so there is NO in-flight PHY
// frame whose single-symbol pacing (9336c23) could be perturbed; the commander is only
// emitting idle keepalives. It is double-gated: the early-out is reachable ONLY when a
// pump is installed (g_sim_inproc_pump != nullptr, i.e. ONLY under -m SIM_INPROC) AND
// this flag is set. Production (-x wasapi/alsa) and the two-process paced sim install
// no pump, so the flag is never consulted and the spin-loop bodies are byte-identical
// to before. The flag is reset at the start of every stepper run.
static bool g_sim_inproc_deliver_done = false;
void arq_set_sim_inproc_deliver_done(bool on) { g_sim_inproc_deliver_done = on; }
bool arq_sim_inproc_deliver_done()            { return g_sim_inproc_deliver_done; }

// STEPPER-CORE REWRITE Phase b: the DRAIN-TO-WIRE-ONLY pump (arq_commander.cc
// sim_inproc_drain_to_wire_only). Installed alongside the main pump by the OUTER stepper. Under
// the OFDM DATA path, drain_playback_wait spins THIS (not the full pump) to EGRESS the big-block
// out of the TX play ring into the in-flight wire WITHOUT bursting it to the RX — the outer loop
// then feeds the wire -> RX one symbol per iter (decode pacing preserved). Shares the same ctx.
static sim_inproc_pump_fn g_sim_inproc_drain_only_pump = nullptr;
void arq_set_sim_inproc_drain_only_pump(sim_inproc_pump_fn fn)
{
	g_sim_inproc_drain_only_pump = fn;
}

// SIM_INPROC TCP-poll gate (single-process-sim-refactor.md §10.5/§10.7-item-4).
// process_main()'s top blocks poll tcp_socket_control / tcp_socket_data (accept,
// recv, transmit). The 2-instance stepper binds NO socket and injects user
// commands + data DIRECTLY (process_user_command + fifo_buffer_*), so those polls
// would syscall on uninit'd sockets (accept(fd=0) → spam + churn). When this flag
// is set (ONLY while the 2-instance stepper runs) process_main skips the three TCP
// blocks. Default false → production + the single-instance Stage-2 prototype +
// the paced sim are byte-identical (the single-instance prototype drives
// send_batch directly, never process_main, so it does not need this either).
static bool g_sim_inproc_skip_tcp = false;
void arq_set_sim_inproc_skip_tcp(bool on) { g_sim_inproc_skip_tcp = on; }
bool arq_sim_inproc_skip_tcp()            { return g_sim_inproc_skip_tcp; }

// SIM_INPROC OUTER-STEPPER gate (sim2-stepper-rewrite Phase b). Non-null/true ONLY
// while the 2-instance OUTER-loop stepper (MERCURY_SIM2_STEPPER=outer) is driving.
// When set, drain_playback_wait() QUEUE-and-returns (no spin-drain): the outer loop
// is the SOLE DAC-drain driver, so every TX site becomes queue-and-return uniformly
// with this single seam (tx_transfer already queues, audioio.c:1691). Gated SEPARATELY
// from arq_sim_inproc_active() so the LEGACY pump stepper (still runnable until Phase d)
// keeps its pump-driven blocking drain. Production + paced sim never set it (false →
// verbatim blocking body → byte-identical). See data-flow-sim2-ofdm-delivery-cadence.md §10.
static bool g_sim_inproc_outer_stepper = false;
void arq_set_sim_inproc_outer_stepper(bool on) { g_sim_inproc_outer_stepper = on; }
bool arq_sim_inproc_outer_stepper_active()     { return g_sim_inproc_outer_stepper; }

// SIM_INPROC OUTER-STEPPER active-instance modulation tag (sim2-stepper-rewrite Phase b).
// sim2_activate(m) records whether the now-active instance is on an OFDM config. The
// outer-stepper TX-wait seams use it to decide pacing vs co-routine delivery:
//   - OFDM active  -> the DATA path: drain_playback_wait / ptt_busy_wait QUEUE-and-return
//     (clock-only); the outer loop paces the big-block ONE SYMBOL PER ITER (the §8 cadence
//     the per-symbol stepper exists to provide; pumping here would BURST-deliver and break
//     OFDM decode).
//   - ROBUST/MFSK active -> the HANDSHAKE / control-ACK path: keep the LEGACY co-routine
//     pump (the peer's HAIL / control-ACK reply must be delivered INTRA-process_main so the
//     same call's receive_* poll sees it — the outer loop cannot inject delivery mid-call,
//     and send_*_pattern's post-TX RX-ring flush would wipe an out-of-band reply; the burst
//     is harmless for MFSK control frames, G2 proves it). This is the per-modulation seam
//     that lets ONE uniform process_main serve both the MFSK handshake and the OFDM data
//     phase. Default false (no active OFDM instance). See §10.4.
static bool g_sim2_active_is_ofdm = false;
void arq_set_sim2_active_is_ofdm(bool on) { g_sim2_active_is_ofdm = on; }

// SIM_INPROC OUTER-STEPPER data-batch-TX scope (sim2-stepper-rewrite Phase b). True ONLY
// while send_batch() is emitting a DATA batch (set at its top, cleared at every exit). This
// is the precise discriminator the per-modulation tag alone could NOT give: at CFG16 the DATA
// (big-block, OFDM) needs per-symbol pacing, but the RSP's batch ACK is an MFSK ACK PATTERN
// sent on the SAME current_configuration=16 — so a config-only gate would wrongly pace the MFSK
// ACK per-symbol and the CMD's correlator would never see it (nAcked_data stuck at 0 -> the CMD
// re-sends bsi=0 forever, delivering duplicate blocks -> bytes_ok=0). Requiring this flag
// confines the per-symbol no-op to the OFDM big-block DATA TX; ACK patterns / control / HAIL keep
// the legacy co-routine pump (intra-call delivery). See §10.4.
static bool g_sim2_in_data_batch_tx = false;
void arq_set_sim2_in_data_batch_tx(bool on) { g_sim2_in_data_batch_tx = on; }

// STEPPER-CORE REWRITE Phase b: count of [SIM2-DEADLOCK-BREAK] floor fires this session. The
// durable --test-sim-sustain asserts this is 0 (a passing arm NEVER wedges; any fire = a
// stranded TX tail = a regression). Reset by the test before each run.
static long g_sim2_deadlock_break_count = 0;
void arq_reset_sim2_deadlock_break_count() { g_sim2_deadlock_break_count = 0; }
long arq_get_sim2_deadlock_break_count()   { return g_sim2_deadlock_break_count; }

// True iff the outer stepper is driving AND we are emitting an OFDM DATA batch (send_batch on an
// OFDM config) — i.e. this TX-wait must EGRESS the big-block out of play (to the wire, via the
// drain-only pump) so the SAME process_main's post-TX path does not strand it, and leave the
// per-symbol delivery to the outer loop. OFDM-gated so the ROBUST/MFSK 19-B data batch (the G2
// arm) stays on the legacy co-routine pump and remains BYTE-IDENTICAL to the legacy stepper
// (dropping the OFDM conjunct re-routes the small MFSK data batch through egress and shifts
// G2's iters 15->16 — a determinism regression). FALSE for the MFSK HANDSHAKE / ACK patterns /
// control frames (legacy co-routine pump delivers them intra-call).
static inline bool sim_outer_stepper_paces_active()
{
	return g_sim_inproc_outer_stepper && g_sim2_active_is_ofdm && g_sim2_in_data_batch_tx;
}

// True iff the outer stepper is driving AND the active instance is OFDM — i.e. we are in the
// OFDM DATA PHASE and the outer loop owns clock+delivery. Used by the end-of-process_main
// pacing-floor pump (which must NOT fire in the OFDM phase even AFTER send_batch returns —
// the queued block is still in play and a pump would BURST-drain it before the outer loop
// paces it). The ACK patterns deliver via their OWN drain (in_data_batch_tx is false there,
// so their drain spins the legacy pump). FALSE for the MFSK handshake (legacy pump runs).
static inline bool sim_outer_stepper_ofdm_phase()
{
	return g_sim_inproc_outer_stepper && g_sim2_active_is_ofdm;
}

// Single place the step-pump is invoked from inside the spin loops. When no
// pump is installed this is exactly the prior body: sim_spin_sleep() under the
// virtual clock, msleep(1) on the production wall-clock path.
static inline void sim_spin_or_pump(bool sim_clock_on)
{
	if (g_sim_inproc_pump != nullptr)
		g_sim_inproc_pump(g_sim_inproc_pump_ctx);  // step-pumpable (SIM_INPROC only)
	else if (sim_clock_on)
		sim_spin_sleep();                          // two-process paced sim
	else
		msleep(1);                                 // production
}

// NOTE: external linkage (not static) so the SIM_INPROC prototype in
// arq_commander.cc can drive the EXACT same spin-loop functions the
// production TX path uses (proving the real helpers are step-pumpable, not a
// re-implementation). Bodies are unchanged from the prior static versions, so
// the production path is behaviorally identical.
void ptt_busy_wait(cl_timer& t, int delay_ms)
{
	// STEPPER-CORE REWRITE Phase b (sim2-stepper-rewrite): under the OUTER-loop stepper,
	// on the OFDM DATA path the PTT on/off turnaround must NOT spin the pump — spinning it
	// would re-enter the pump's BURST deliver+decode-drive at depth 0 DURING process_main
	// (right after a batch was queued, sim2_drain_to_wire drains the WHOLE play buffer in
	// one shot), defeating the outer loop's per-symbol pacing and re-introducing the (iii)
	// nested-drain coupling this rewrite eliminates. Instead ADVANCE THE SHARED VIRTUAL
	// CLOCK by delay_ms directly (clock-only tick): the exit predicate is satisfied so any
	// in-call wait terminates, while the TX drain is left to the OUTER loop's per-symbol
	// feed. On the ROBUST/MFSK HANDSHAKE path the gate is FALSE -> the verbatim spin body
	// runs (legacy co-routine pump), so the peer's HAIL/control-ACK reply is delivered
	// intra-call. Production + paced sim keep the verbatim spin body (byte-identical).
	if (sim_outer_stepper_paces_active())
	{
		if (delay_ms > 0)
			sim_clock_add_samples((uint64_t)delay_ms * (SIM_CLOCK_SAMPLE_RATE_HZ / 1000));
		return;
	}
	const bool sim_clock_on = sim_clock_enabled();
	// EXIT CONDITION UNCHANGED: virtual (or wall) time must pass delay_ms.
	// SIM_INPROC-only post-delivery abort (see arq_set_sim_inproc_deliver_done):
	// double-gated on (pump installed) AND (deliver_done) — both false on every
	// production / paced-sim path, so the loop body is byte-identical there.
	while (t.get_elapsed_time_ms() < delay_ms)
	{
		if (g_sim_inproc_pump != nullptr && g_sim_inproc_deliver_done)
			return;
		sim_spin_or_pump(sim_clock_on);
	}
}

void drain_playback_wait()
{
	// STEPPER-CORE REWRITE Phase b (sim2-stepper-rewrite): under the OUTER-loop stepper, on the
	// OFDM DATA path drain_playback_wait EGRESSES the big-block out of the TX play ring into the
	// in-flight WIRE (via the drain-ONLY pump — NO RX deliver, NO decode burst), then returns. The
	// OUTER loop feeds the wire -> RX ONE SYMBOL PER ITER, so OFDM decode still sees the §8
	// per-symbol cadence and the nested-drain (i)/(ii)/(iii) call-stack coupling (§9.3) that
	// defeated all 8 localized fixes is gone (the drain-only pump never delivers, never
	// re-enters a peer process_main). EGRESS (not no-op) is REQUIRED because the SAME process_main's
	// post-TX ACK path clear_buffer(playback_buffer)s any residual — a block left in play would be
	// DISCARDED the instant the CMD enters the ACK wait (rx never gets it). The wire is 12.288 MB
	// (~19 blocks) so a single in-flight block never saturates it -> the spin always terminates (no
	// deadlock-break floor needed). INV-D (§10.2): the post-TX bookkeeping touches ONLY the RX ring
	// + STATE fields; with play now EMPTY the ACK-path flush is harmless. On the ROBUST/MFSK
	// HANDSHAKE path the gate is FALSE -> the verbatim spin body runs (legacy co-routine pump
	// delivers the beacon/control intra-call). Production + paced sim leave the gate false.
	if (sim_outer_stepper_paces_active())
	{
		if (g_sim_inproc_drain_only_pump != nullptr)
			while (size_buffer(playback_buffer) > 0)
				g_sim_inproc_drain_only_pump(g_sim_inproc_pump_ctx);
		return;
	}
	const bool sim_clock_on = sim_clock_enabled();
	// EXIT CONDITION UNCHANGED on production + the two-process paced sim: the
	// playback ring must be fully drained (the real audio DAC always drains it
	// at the sample rate, so the loop always terminates).
	//
	// SIM_INPROC NO-PROGRESS DEADLOCK BREAK (fix/bigblock-d3-carve): the
	// single-thread cooperative pump (sim_inproc_pump_2) can only drain TX
	// playback -> the in-flight WIRE while the wire has free space, and it can
	// only deliver the wire -> RX (freeing the wire) at pump call-DEPTH 0. When
	// this drain_playback_wait runs NESTED (depth>0 — e.g. driven from the peer's
	// own wait while the RX is stalled), and the RX is NOT consuming the wire
	// (the D3 RETXCOPY false-lock arm never carves a deliverable block, so it
	// never re-arms / drains), the wire SATURATES and the TX playback can never
	// move -> this loop spins forever (HW has no analogue: a real half-duplex DAC
	// drains unconditionally; the finite sim "wire" models samples in flight and
	// a jammed channel must eventually DROP the un-deliverable TX tail). Bound the
	// SIM_INPROC spin by NO-PROGRESS: if the playback occupancy has not shrunk for
	// a large number of consecutive pump calls, the channel is jammed -> abandon
	// the undrainable tail (exit the wait) so the outer stepper regains control
	// and its own stall/iter cutoff terminates the run. This is a harness-only
	// flow-control floor: passing arms ALWAYS make progress (the RX consumes the
	// wire every depth-0 pump), so the guard NEVER fires for them (verified). The
	// production + paced-sim paths (pump==null) keep the verbatim unbounded body.
	const bool inproc = arq_sim_inproc_active();
	// A HEALTHY drain makes progress on (essentially) every pump call: at pump
	// call-depth 0 each pump moves one TX symbol playback->wire and delivers a
	// symbol wire->RX, so the playback occupancy strictly shrinks and the
	// no-progress counter resets. The DEADLOCK case makes ZERO progress on EVERY
	// pump call (depth>0 + saturated wire). 2000 consecutive no-progress pump
	// calls is therefore orders of magnitude above any healthy transient yet
	// bounds the jammed-channel wall time to a few ms; the passing arms never
	// reach even a handful (verified — they deliver and exit normally).
	const long NO_PROGRESS_LIMIT = 2000;
	size_t prev_occ = size_buffer(playback_buffer);
	long   no_progress = 0;
	while (size_buffer(playback_buffer) > 0)
	{
		// SIM_INPROC-only post-delivery abort (see arq_set_sim_inproc_deliver_done):
		// double-gated on (pump installed) AND (deliver_done) — both false on every
		// production / paced-sim path, so the loop body is byte-identical there. Once
		// the responder holds the full payload there is nothing left to drain that the
		// outer loop needs, so unwind back to it (it breaks on the delivery predicate).
		if (g_sim_inproc_pump != nullptr && g_sim_inproc_deliver_done)
			return;
		sim_spin_or_pump(sim_clock_on);
		if (inproc)
		{
			size_t occ = size_buffer(playback_buffer);
			if (occ < prev_occ) { prev_occ = occ; no_progress = 0; }
			else if (++no_progress >= NO_PROGRESS_LIMIT)
			{
				g_sim2_deadlock_break_count++;   // Phase b: assertable by --test-sim-sustain
				printf("[SIM2-DEADLOCK-BREAK] drain_playback_wait: playback "
				       "un-drainable for %ld pump calls (wire jammed, RX not "
				       "consuming) -> abandoning %zu-byte TX tail so the stepper "
				       "can terminate (harness-only; production DAC always drains)\n",
				       no_progress, occ);
				fflush(stdout);
				break;
			}
		}
	}
}

// ---------------------------------------------------------------------------
// SIM_INPROC settle-wait helpers (single-process-sim-refactor.md §5.7 / §7).
//
// arq_sim_inproc_active(): the step-pump pointer is the authoritative signal.
// It is non-null ONLY while the -m SIM_INPROC single-thread stepper is driving
// (arq_commander.cc test_sim_inproc installs it, clears it on exit). Every
// production path and the two-process paced sim leave it null.
bool arq_sim_inproc_active()
{
	return g_sim_inproc_pump != nullptr;
}

// pumped_settle_wait(): virtual-clock-ify a wall settle-wait WITHOUT changing
// its exit semantics. On EVERY non-SIM_INPROC path (pump null) this is the
// verbatim wall-clock body — msleep(wait_ms) — so production + the paced sim
// are byte-identical. Under SIM_INPROC the wall msleep would freeze the single
// shared sample-counter virtual clock (a peer instance's view of time stops),
// so instead we run a cl_timer + step-pump loop: the SAME EXIT PREDICATE
// (elapsed >= wait_ms, identical to ptt_busy_wait) with the pump advancing the
// shared clock through the SAME rx_transfer -> sim_clock_add_samples accounting
// as the two-process RX bridge. No early exit, no threshold change — only the
// clock-advance mechanism differs.
void pumped_settle_wait(int wait_ms)
{
	if (wait_ms <= 0)
		return;
	// STEPPER-CORE REWRITE Phase b: under the OUTER-loop stepper, pumped_settle_wait STILL
	// pumps (clock-pumping via the step-pump), exactly like the legacy stepper. Its callers
	// are the HANDSHAKE / ACK-turnaround settle guards (the HAIL listen loop
	// arq_commander.cc:423, the control-ACK arrival rescan arq_common.cc:7437, the
	// send_ack_pattern* / send_mfsk_ack_sack ROBUST guards). These need the peer reply
	// delivered INTRA-process_main (the peer's HAIL/control-ACK reply must land in the RX
	// ring BEFORE the same call's receive_* poll — the outer loop cannot inject delivery
	// mid-process_main). The pump's burst delivery is HARMLESS here: these are all MFSK /
	// ROBUST control frames (G2 proves MFSK decodes under burst), and the OFDM DATA batch is
	// never in the play buffer during a pumped_settle_wait call (it is drained only by the
	// now-no-op drain_playback_wait + clock-only ptt_busy_wait inside send_batch, then
	// send_batch RETURNS — so no pumped_settle_wait fires with a data batch queued). The
	// OFDM data path's per-symbol pacing is owned entirely by the outer loop. Production +
	// paced sim keep the verbatim body (byte-identical).
	if (g_sim_inproc_pump == nullptr)
	{
		msleep(wait_ms);            // production + two-process paced sim: verbatim
		return;
	}
	// SIM_INPROC: step-pumped wait, same exit predicate as ptt_busy_wait.
	cl_timer t;
	t.start();
	while (t.get_elapsed_time_ms() < wait_ms)
	{
		// SIM_INPROC-only post-delivery abort (see arq_set_sim_inproc_deliver_done):
		// unwind the post-transfer keepalive spin once the responder holds the full
		// payload. Reachable ONLY under -m SIM_INPROC (pump installed) AND post-
		// delivery (flag set); the msleep production/paced-sim path above already
		// returned, so this is never consulted off the SIM_INPROC stepper.
		if (g_sim_inproc_deliver_done)
			return;
		sim_spin_or_pump(true);     // pump installed -> advances the shared clock
	}
}

// sim_inproc_rx_mute_settle(): the RX_MUTE drain guard (B5 / ADD-ON1). The wait
// exists to let in-flight ASYNC AUDIO CALLBACKS finish writing the capture ring
// before circular_buf_reset() zeroes it. Under SIM_INPROC the stepper OWNS RX —
// there is NO async audio-callback thread, nothing is in flight — so the drain
// purpose is MOOT and the wait becomes a no-op. The caller's circular_buf_reset
// (instantaneous state, no clock semantics) is UNCHANGED and still fires. On
// production/paced-sim (pump null) it is the verbatim msleep(wait_ms).
void sim_inproc_rx_mute_settle(int wait_ms)
{
	if (g_sim_inproc_pump != nullptr)
		return;                     // no async drainer in-process -> moot, no-op
	msleep(wait_ms);                // production + paced sim: verbatim
}

static const int RX_MUTE_GUARD_MS = 50;

cl_arq_controller::cl_arq_controller()
{
	connection_status=IDLE;
	link_status=IDLE;
	nMessages=0;
	max_message_length=0;
	max_data_length=0;
	max_header_length=0;

	messages_last_ack_bu.data=NULL;
	messages_control.data=NULL;
	messages_rx_buffer.data=NULL;
	messages_tx=NULL;
	messages_rx=NULL;
	messages_rx_prev=NULL;   // SACK Design A Step 8a — RSP prev-batch storage
	messages_batch_tx=NULL;
	messages_batch_ack=NULL;
	message_TxRx_byte_buffer=NULL;

	message_batch_counter_tx=0;
	v2_retx_prefix_count=0;  // R030: reset every batch build; set to R on v2 mixed
	ack_timeout_data=1000;
	ack_timeout_control=1000;
	link_timeout=10000;
	watchdog_timeout=1000;
	receiving_timeout=10000;
	switch_role_timeout=1000;
	switch_role_test_timeout=1000;
	gearshift_timeout=1000;
	connection_timeout=30000;
	nResends=3;
	stats.nSent_data=0;
	stats.nAcked_data=0;
	stats.nReceived_data=0;
	stats.nLost_data=0;
	stats.nReSent_data=0;
	stats.nAcks_sent_data=0;
	stats.nBatches_sent=0;
	stats.nBatches_acked=0;
	stats.nBatches_fully_acked=0;  // CLEAN-BATCH VIABILITY (§9)
	stats.nNAcked_data=0;

	stats.nSent_control=0;
	stats.nAcked_control=0;
	stats.nReceived_control=0;
	stats.nLost_control=0;
	stats.nReSent_control=0;
	stats.nAcks_sent_control=0;
	stats.nNAcked_control=0;
	stats.success_rate_data=0;


	last_transmission_block_stats.nSent_data=0;
	last_transmission_block_stats.nAcked_data=0;
	last_transmission_block_stats.nReceived_data=0;
	last_transmission_block_stats.nLost_data=0;
	last_transmission_block_stats.nReSent_data=0;
	last_transmission_block_stats.nAcks_sent_data=0;
	last_transmission_block_stats.nBatches_sent=0;
	last_transmission_block_stats.nBatches_acked=0;
	last_transmission_block_stats.nBatches_fully_acked=0;  // CLEAN-BATCH VIABILITY (§9)
	last_transmission_block_stats.nNAcked_data=0;

	last_transmission_block_stats.nSent_control=0;
	last_transmission_block_stats.nAcked_control=0;
	last_transmission_block_stats.nReceived_control=0;
	last_transmission_block_stats.nLost_control=0;
	last_transmission_block_stats.nReSent_control=0;
	last_transmission_block_stats.nAcks_sent_control=0;
	last_transmission_block_stats.nNAcked_control=0;
	last_transmission_block_stats.success_rate_data=0;

	measurements.SNR_uplink=-99.9;
	measurements.SNR_downlink=-99.9;
	measurements.signal_stregth_dbm=-99.9;
	measurements.frequency_offset=-99.9;

	data_batch_size=1;
	nominal_batch_size=1;
	batch_consec_acks=0;
	control_batch_size=1;
	ack_batch_size=1;
	batch_rx_frame_count=0;
	batch_data_delivered=false;

	// SACK / SACK_V2 are unconditional after CAP_SACK + CAP_SACK_V2 removal.
	// --no-sack still lets the operator disable SACK locally via disable_sack;
	// negotiation is computed once at TEST_CONNECTION time in arq_responder.cc
	// / arq_commander.cc using only the local flag. enable_sack_v2 stays true
	// for harness compat (--enable-sack-v2 / --disable-sack-v2 are no-ops now).
	sack_enabled=false;       // Set at TEST_CONNECTION negotiation.
	sack_v2_enabled=false;    // Set at TEST_CONNECTION negotiation.
	enable_sack_v2=true;
	radio_batch_size=25;
	crypto_batch_size=20;
	retransmit_headroom=5;
	crypto_batch_counter_tx=0;
	crypto_batch_counter_rx=0;
	retransmit_count=0;
	sack_retransmit_active=false;
	// WALL-B FIX-9 D2 REFINE (_fix9/d2refine/D2_REFINE_DESIGN.md §4 Q3): the retx-turnaround gates
	// for the D2 robust reverse-ACK geometry. INIT FALSE so a pure-clean session never fires the
	// settle/widen (byte-identical to D2-off). Both are re-derived per-turnaround, never on the wire.
	data_ack_retx_turnaround=false;
	ack_tx_retx_turnaround=false;
	retransmit_batch_id=-1;
	for(int i=0;i<MAX_RETRANSMIT_HEADROOM;i++) {
		retransmit_frame_lengths[i]=0;
		retransmit_frame_positions[i]=0;
		retransmit_frame_types[i]=0;
		retransmit_frame_batch_seq_ids[i]=-1;  // Step 3: unset until SACK populates
		retransmit_frame_seq_with_eob[i]=0;    // §7.13.39 Fix 3: original seq+EOB byte
	}
	// SACK Design A Step 3 — batch_seq_id state (CMD counter, RSP store).
	// cmd_batch_seq_id starts at 0; first new-data batch goes out under value 0.
	// captured_batch_seq_id_for_retransmit is -1 (no SACK retransmit pending).
	// last_received_batch_seq_id is -1 (no DATA frame parsed yet).
	cmd_batch_seq_id=0;
	captured_batch_seq_id_for_retransmit=-1;
	last_received_batch_seq_id=-1;
	// SACK Design A Step 4 — RSP cross-batch routing state. All gated on
	// sack_v2_enabled; v1 path leaves these at their sentinels. Per §4.2.3,
	// `current_expected` is adopted from the first v2 DATA frame received;
	// `prev` stays -1 until the first ACK-GATE-PASS. `drop_count` tracks
	// [RSP-V2-DROP] events for test assertions.
	rsp_current_expected_batch_seq_id=-1;
	rsp_prev_batch_seq_id=-1;
	// FIX-8 (data-integrity): reset-surviving delivery high-water mark. Init -1
	// here (ctor) and at reset_session_state() ONLY — it must SURVIVE the BREAK
	// reset + FULL load_configuration so the post-reset adopt gate can detect a
	// dropped-batch hole. See bigblock_p3_hw/_fix8/FIX8_DESIGN.md §4.2-§4.3.
	rsp_last_delivered_batch_seq_id=-1;
	rsp_v2_drop_count=0;
	test_rsp_bsi_corrupt_at=0;
	test_rsp_bsi_v2_frame_counter=0;
	// SACK Design A Step 7 — SACK_RSP OFDM control-frame counters.
	// All start at 0 / -1 / false sentinels; only written when sack_v2_enabled.
	rsp_sack_v2_tx_count=0;
	cmd_sack_v2_rx_count=0;
	cmd_sack_v2_crc_fail_count=0;
	for(int i=0; i<(int)sizeof(cmd_sack_v2_last_rx_bitmap); i++)
		cmd_sack_v2_last_rx_bitmap[i]=0;
	cmd_sack_v2_last_rx_nbits=-1;
	cmd_sack_v2_last_rx_batch_seq_id=-1;
	test_rsp_sack_rsp_crc_corrupt=false;
	test_rsp_sack_rsp_crc_corrupt_armed=false;
	test_rsp_sack_rsp_crc_corrupt_count=0;
	// SACK Design A Step 8a — RSP prev-batch parallel-storage bookkeeping.
	// All gated on sack_v2_enabled; v1 path leaves these at their sentinels.
	// `messages_rx_prev` itself is allocated in init_messages_buffers().
	rsp_prev_batch_active=false;
	rsp_prev_batch_received_count=0;
	rsp_prev_batch_expected_count=0;
	rsp_prev_batch_delivered_count=0;
	rsp_prev_batch_stale_count=0;
	// SACK Design A Step 10 — Axis 2 controller state (adaptive batch size).
	// All CMD-side; gated on sack_v2_enabled at the call sites. Ring + counters
	// + cooldown all start at zero. v1 sessions leave these untouched.
	for(int i=0;i<AXIS2_RING_DEPTH;i++) axis2_partial_rate_ring[i]=0.0f;
	axis2_partial_rate_count=0;
	axis2_partial_rate_pos=0;
	axis2_consecutive_good_batches=0;
	axis2_consecutive_bad_batches=0;
	axis2_cooldown_batches=0;
	// SACK Design A Step 12 — Axis-2 proven-ceiling state (§4.3.4 invariant #7).
	// Default ceiling = -1 (no cap). Set on Axis-2 down-move; cleared on
	// recovery (20 batches) or on any Axis-1 supremacy event.
	batch_size_proven_ceiling=-1;
	batch_size_ceiling_recovery_batches=0;
	axis2_ceiling_blocks_count=0;
	axis2_evaluations=0;
	axis2_move_up_count=0;
	axis2_move_down_count=0;
	axis2_skipped_in_cooldown=0;
	test_policy_axis2_fire_armed=0;
	test_cmd_axis2_suppressed_after_axis1=0;
	rsp_set_link_params_rx_count=0;
	rsp_set_link_params_crc_fail_count=0;
	pending_link_params_batch_size=-1;
	pending_link_params_sack_mode=-1;
	// FIX-A — ROBUST dwell-batch transport state (data-flow-robust-tier-arq-batch.md §5.2/§5.3).
	pending_robust_dwell_batch=-1;
	robust_dwell_batch_active=false;
	// FIX-B — FLOOR-PROBE BACK-OFF state (gearshift-floor-probe-backoff.md §3/§5).
	// Seed the window to INIT and zero the per-rung deadline array (no probe
	// suppressed at session start). reset_session_state() mirrors this.
	probe_backoff_ms=PROBE_BACKOFF_MS_INIT;
	for(int i=0;i<FULL_CONFIG_LADDER_SIZE;i++) probe_backoff_until_ms[i]=0ULL;
	// SACK Design A Step 11 — Axis 3 controller state (SACK mode ON↔PROBE↔OFF).
	// Initial state = ON (per §4.3.2 spec). The mode is materially ON only on
	// sack_v2_enabled sessions; on v1 sessions the field stays at its sentinel
	// but no Axis-3 code path is reached (all sites gated on sack_v2_enabled).
	axis3_sack_mode = SACK_MODE_ON;
	for(int i=0;i<AXIS3_RING_DEPTH;i++) axis3_recent_sack_ok[i]=false;
	axis3_recent_sack_ok_count=0;
	axis3_recent_sack_ok_pos=0;
	axis3_consecutive_sack_misses=0;
	axis3_batches_since_off=0;
	axis3_cooldown_batches=0;
	axis3_evaluations=0;
	axis3_ok_events=0;
	axis3_miss_events=0;
	axis3_move_on_to_probe_count=0;
	axis3_move_probe_to_on_count=0;
	axis3_move_probe_to_off_count=0;
	axis3_move_off_to_probe_count=0;
	axis3_skipped_in_cooldown=0;
	test_policy_axis3_fire_armed=0;
	test_policy_axis3_walk_armed=0;
	crypto_buf[0].clear();
	crypto_buf[1].clear();
	message_transmission_time_ms=500;
	ctrl_transmission_time_ms=500;
	ack_pattern_time_ms=0;
	role=RESPONDER;
	original_role=RESPONDER;
	connection_id=0;
	assigned_connection_id=0;
	block_ready=0;
	block_under_tx=NO;
	ack_batch_size=1;
	my_call_sign="";
	destination_call_sign="";
	user_command_buffer="";
	telecom_system=NULL;

	print_stats_frequency_hz=2;

	init_configuration=CONFIG_0;
	current_configuration=CONFIG_0;
	negotiated_configuration=CONFIG_0;
	last_data_configuration=CONFIG_0;
	ack_configuration=CONFIG_0;
	data_configuration=CONFIG_0;
	forward_configuration=CONFIG_NONE;
	reverse_configuration=CONFIG_NONE;

	gear_shift_on=NO;
	robust_enabled=NO;
	narrowband_enabled=NO;
	commander_configured_nb=-1;
	nb_probe_max=2;
	session_narrowband=false;
	bandwidth_mode=BW_AUTO;
	// SACK v1 + v2 ON by default. The historical B2 fix (2026-05-12) that
	// turned SACK off was obsoleted by SACK Design A (5 commits e73968a..1acdb3c
	// on monitor branch, May 16-17 2026), which solved the regressions that
	// motivated the disable. SACK is now load-bearing for everything above
	// CFG6 (the optimizer's calibrated floor) — turning it off forfeits
	// partial-batch recovery, piggyback retx, and all of Design A's gains.
	// Opt-out remains available via --no-sack / --disable-sack-v2.
	disable_sack=false;
	local_capability=0;  // CAP_WB_CAPABLE / CAP_ENCRYPTION set per-mode at LISTEN ON / CONNECT.
	peer_capability=0;
	handshake_confirmed=false;
	handshake_retries_left=MAX_HANDSHAKE_RETRIES;
	wb_upgrade_pending=false;
	psk_mismatch_pending=false;
	compression_enabled=false;
	force_compress=false;
	b2f_compression_pending=false;
	compress_ratio_estimate=2.0f;
	batch_uncompressed_size=0;
	encryption_mode=ENCRYPT_OFF;
	encryption_enabled=false;
	tx_batch_counter=0;
	rx_batch_counter=0;
	consecutive_auth_failures=0;
	kx_data_buf=NULL;
	kx_data_len=0;
	memset(psk_hex, 0, sizeof(psk_hex));
	passive_monitor=false;
	monitor_stdout=false;
	monitor_consec_ofdm_fail=0;
	for(int i=0; i<NUMBER_OF_CONFIGS; i++) monitor_decoders[i]=NULL;
	monitor_decoders_ready=false;
	monitor_primary_buffer_nsymb=0;
	gear_shift_algorithm=SUCCESS_BASED_LADDER;

	gear_shift_up_success_rate_precentage=85;
	gear_shift_down_success_rate_precentage=55;
	gear_shift_block_for_nBlocks_total=5;
	gear_shift_blocked_for_nBlocks=0;
	gear_shift_down_consecutive_fails=0;
	success_rate_data_clean=100.0;  // CLEAN-BATCH VIABILITY (§9) — neutral until first block
	consecutive_data_acks=0;
	frame_shift_threshold=3;
	frame_gearshift_just_applied=false;
	frame_gearshift_retry_count=0;

	turboshift_phase=TURBO_FORWARD;
	turboshift_active=true;
	turboshift_last_good=-1;
	turboshift_initiator=false;
	turboshift_retries=1;
	turbo_settle_pending=false;
	turbo_supershift_announce_pending=false;
	supershift_proven_ceiling=-1;
	// Option B (data-anchored promotion): see arq.h. Ctor PRE-INIT placeholder —
	// robust_enabled is still NO (set just above) and init_configuration is still
	// CONFIG_0 here, so session_floor_anchor() returns CONFIG_0; this is overwritten
	// by the AUTHORITATIVE seat in init() (arq_common.cc, after the robust/non-robust
	// branch resolves init_configuration). Routed through the helper for consistency
	// with the other two seats. See gearshift-climb-engine.md §17 (the anchor was
	// poisoned to CONFIG_0 here because CONNECT skips reset_session_state, so the
	// ctor value was the value the first session climbed from).
	last_data_viable_config=session_floor_anchor(robust_enabled, init_configuration);
	// DEEP-SNR DOWN-HYSTERESIS (gearshift-climb-engine.md §10/§11): no BREAK has
	// fired and no clean streak exists yet. anchor_consec_break_fails counts
	// consecutive BREAKs AT the anchor rung (→ demote at K); the sustained-anchor
	// counters gate the anchor-RAISE on N consecutive clean batches per rung.
	anchor_consec_break_fails=0;
	clean_batches_at_current_config=0;
	clean_batches_config=CONFIG_NONE;
	// CLEAN-BATCH VIABILITY (§9): no batch delivered yet — promotion-gating flag
	// starts FALSE. Re-set per batch at TX start (arq_commander.cc:1244/1739).
	last_batch_fully_acked=false;
	skip_turbo_reverse=false;
	max_config_override=-1;
	optimizer_disabled=false;
	turbo_snr_ack_enabled=false;
	turbo_received_snr=-99.0f;
	turbo_best_snr=-99.0f;
	turbo_switch_role_retries=0;
	ack_diag_peak_matched=0;
	ack_diag_peak_metric=0.0;
	ack_diag_poll_count=0;
	ack_diag_peak_mask=0;
	v2_ackpat_defer_count_this_window=0;  // Bug A fix (§7.13.1)
	// §7.13.29 init
	cmd_last_applied_sack_bsi = -1;
	// climb-engine Bug 1 (gearshift-climb-engine.md §4): clean-confirm dedupe
	// tracker, split from the partial tracker above. Same lifecycle (ctor init).
	cmd_last_applied_clean_bsi = -1;
	sack_arrival_history_count = 0;
	sack_arrival_history_next_idx = 0;
	for(int i=0; i<SACK_ARRIVAL_HISTORY; i++) sack_arrival_history_ms[i] = 0;
	v2_dispatch_last_rwi = -1;
	v2_dispatch_min_advance_syms = 1;

	phy_reinit_settle_us=300000;  // Phase-2 flag default = HEAD (b806b76 Bug #60)
	ack_metric_threshold=0.5;     // Phase-2 flag default = HEAD (7076a4b 3.0→0.5)
	sack_timeout_extra_ms=0;      // SACK_FIX_PLAN §7 step 3: default 0 post-(a).
	                              // Pre-(a) default was 3000 (commit 7076a4b)
	                              // — a band-aid for the now-closed Plan-A
	                              // turnaround race. CLI override
	                              // --sack-timeout-extra-ms=N is preserved.
	// Note: disable_sack initialized at top of constructor (~line 175) before
	// the local_capability assignment so the mask there sees the correct value.
	emergency_nack_count=0;
	emergency_nack_threshold=3;
	emergency_break_active=0;
	emergency_break_retries=3;
	emergency_previous_config=CONFIG_0;
	break_drop_step=2;  // 2026-05-24: start aggressive (was 1) — first BREAK
	                    // drops 2 configs, then doubles 4,8,16,32... uncapped.
	breaks_since_last_data_success=0;
	// WALL-B FIX-5 (audit R3): a fresh session must NEVER inherit a stale carve cooldown
	// (it would needlessly cap the new link at CFG15). Mirror supershift_proven_ceiling's
	// init exactly. Member-initializers in arq.h cover construction; this is belt-and-
	// suspenders for the init() path that re-runs this block.
	bigblock_carve_cooldown_batches=0;
	bigblock_carve_cooldown_span=0;
	cfg16_revack_starve_fails=0;   // WALL-B FIX-9 D3: fresh session never inherits a stale starve count
	break_recovery_phase=0;
	break_recovery_retries=0;
	ceiling_success_count=0;
	break_detected=NO;
	break_probe_consec_match=0;   // fix/break-fh-gate: fresh K-of-N streak (no-op read when env off)
	hail_detected=NO;
	hail_sent=NO;

	ptt_on_delay_ms=0;
	ptt_off_delay_ms=0;
	time_left_to_send_last_frame=0;

	last_message_sent_type=NONE;
	last_message_sent_code=NONE;

	last_message_received_type=NONE;
	last_message_received_code=NONE;

	last_received_message_sequence=255;
	last_received_end_of_batch_seq=-1;
	rx_buffer_eob_seq=-1;  // R038: per-frame v2 EOB staging (set in receive())
	data_ack_received=NO;
	repeating_last_ack=NO;
	disconnect_requested=NO;
	connection_attempts=0;
	max_connection_attempts=15;
	exit_on_disconnect=NO;
	had_control_connection=NO;

	this->messages_control_bu.status=FREE;
	this->messages_control_bu.data=NULL;
	this->messages_control_bu.data=new char[N_MAX / 8];
	if(this->messages_control_bu.data==NULL)
	{
		exit(MEMORY_ERROR);
	}

	// Phase 3a (Effective-Rate Optimizer) — zero-init the rolling window.
	// Same effect as opt_reset_window(), but called explicitly here so the
	// invariant "ring arrays start zero" is enforced before any session.
	for (int i = 0; i < OPTIMIZER_WINDOW_SIZE; i++) {
		opt_batch_bytes_delivered[i] = 0;
		opt_batch_wire_ms[i] = 0;
		opt_batch_sack_count[i] = 0;
		opt_batch_failed[i] = 0;
		opt_batch_config[i] = 0;
	}
	opt_window_head = 0;
	opt_window_count = 0;
	opt_batch_tx_start_ms = 0;
	opt_diag_emit_counter = 0;
	opt_pending_switch_cfg = -1;  // Phase 3c — no pending optimizer switch

}



cl_arq_controller::~cl_arq_controller()
{
	if(messages_control_bu.data!=NULL)
	{
		delete[] messages_control_bu.data;
	}
	this->deinit_messages_buffers();
}


void cl_arq_controller::set_nResends(int nResends)
{
	if(nResends>0)
	{
		this->nResends=nResends;
	}
}


void cl_arq_controller::set_ack_timeout_control(int ack_timeout_control)
{
	if(ack_timeout_control>0)
	{
		this->ack_timeout_control=ack_timeout_control;
	}

	if(messages_control.status==PENDING_ACK)
	{
		messages_control.ack_timeout=this->ack_timeout_control;
	}
	if(messages_control_bu.status==PENDING_ACK)
	{
		messages_control_bu.ack_timeout=this->ack_timeout_control;
	}
}

void cl_arq_controller::set_ack_timeout_data(int ack_timeout_data)
{
	if(ack_timeout_data>0)
	{
		this->ack_timeout_data=ack_timeout_data;
	}

	if(messages_tx!=NULL)
	{
		for(int i=0;i<nMessages;i++)
		{
			if(messages_tx[i].status==PENDING_ACK)
			{
				messages_tx[i].ack_timeout=this->ack_timeout_data;
			}
		}
	}
}

void cl_arq_controller::set_receiving_timeout(int receiving_timeout)
{
	if(receiving_timeout>0)
	{
		this->receiving_timeout=receiving_timeout;
	}
}

void cl_arq_controller::set_link_timeout(int link_timeout)
{
	if(link_timeout>0)
	{
		this->link_timeout=link_timeout;
	}
}


void cl_arq_controller::set_nMessages(int nMessages)
{
	if(nMessages>0 && nMessages<256)
	{
		this->nMessages=nMessages;
	}
	else
	{
		this->nMessages=255;
	}
}

void cl_arq_controller::set_max_buffer_length(int max_data_length, int max_message_length, int max_header_length)
{
	if(max_data_length>0 && max_data_length<256 && max_data_length<max_message_length)
	{
		this->max_data_length=max_data_length;
	}

	if(max_message_length>0 && max_message_length<256)
	{
		this->max_message_length=max_message_length;
	}

	if(max_header_length>0)
	{
		this->max_header_length=max_header_length;
	}
}

void cl_arq_controller::set_ack_batch_size(int ack_batch_size)
{
	if (ack_batch_size>0)
	{
		this->ack_batch_size=ack_batch_size;
	}
}

// R035 (race audit 2026-06-06): re-derive the active RSP prev-batch counters
// when the data batch SHRINKS, and fire the streaming desync defense if any
// already-RECEIVED prev slot is orphaned beyond the new (smaller) batch.
//
// THE BUG: rsp_prev_batch_expected_count is FROZEN at arm-time from the OLD
// data_batch_size (bump_bsi_and_transfer_prev, arq_common.cc:4437). An Axis-2
// down-move (e.g. 15->10) or a robust-dwell revert (8->1) shrinks data_batch_size
// while prev is active, but no path re-derives expected_count. The LIVE prev-write
// bound (arq_responder.cc:686, loc >= data_batch_size reject) then rejects any
// prev frame whose slot is in [new, old), so rsp_prev_batch_received_count can
// never reach the frozen expected_count -> the prev never delivers via the
// completion gate (arq_responder.cc:728) -> the eventual re-bump hits the
// stale-discard (arq_common.cc:4366-4379) which FREEs messages_rx_prev[] WITHOUT
// streaming_reset() -> PPMd streaming desync (the ff829d5 class).
//
// THE FIX (mirrors the delivery leg's streaming defense, arq_responder.cc:731):
//   - received_count := count of RECEIVED prev slots in [0, new)
//   - expected_count := min(old expected, new)  (gate becomes reachable)
//   - if any RECEIVED prev slot is orphaned in [new, old): the prev cannot be
//     delivered intact, so fire streaming_reset() (guarded is_streaming() &&
//     batch_data_delivered) NOW, before the data is lost, instead of silently
//     desyncing at the later stale-discard.
//
// Called from the set_data_batch_size() chokepoint (both the robust and OFDM
// clamp branches) with the post-clamp value, BEFORE data_batch_size is updated.
void cl_arq_controller::rescan_prev_on_batch_shrink(int new_batch)
{
	if(!sack_v2_enabled) return;
	if(!rsp_prev_batch_active) return;
	int old_batch = this->data_batch_size;
	if(new_batch >= old_batch) return;   // only SHRINK strands the prev path

	// Recompute received_count within the new (smaller) bound, and detect
	// already-RECEIVED slots that the new bound orphans in [new_batch, old_batch).
	int new_received = 0;
	for(int i = 0; i < new_batch && i < this->nMessages; i++)
		if(messages_rx_prev[i].status == RECEIVED) new_received++;
	int orphaned_received = 0;
	for(int i = new_batch; i < old_batch && i < this->nMessages; i++)
		if(messages_rx_prev[i].status == RECEIVED) orphaned_received++;

	int old_expected = rsp_prev_batch_expected_count;
	int new_expected = old_expected;
	if(new_expected > new_batch) new_expected = new_batch;
	if(new_expected < 1)         new_expected = 1;

	// If RECEIVED prev data is orphaned beyond the new bound, the prev batch can
	// no longer be delivered intact. Fire the streaming defense (same guard +
	// handshake as the in-order delivery leg) BEFORE the data is discarded, so
	// the next TX batch detects RX-cold and resets — no silent PPMd desync.
	if(orphaned_received > 0 && compressor.is_streaming() && batch_data_delivered)
	{
		compressor.streaming_reset();
		printf("[STREAMING] Reset: prev-batch orphaned by data_batch_size shrink "
			"%d->%d (orphaned_received=%d) — R035 desync defense\n",
			old_batch, new_batch, orphaned_received);
		fflush(stdout);
	}

	printf("[RSP-V2-PREV-RESHRINK] data_batch_size %d->%d prev_batch_seq_id=%d "
		"received %d->%d expected %d->%d orphaned_received=%d\n",
		old_batch, new_batch, rsp_prev_batch_seq_id,
		rsp_prev_batch_received_count, new_received,
		old_expected, new_expected, orphaned_received);
	fflush(stdout);

	rsp_prev_batch_received_count = new_received;
	rsp_prev_batch_expected_count = new_expected;
}

// R029 (race audit 2026-06-06): the single owner of zeroing the TX retransmit
// queue. See the arq.h declaration for the full rationale. Called from every
// messages_tx[]-freeing recovery site; idempotent (count already 0 -> no-op log
// suppressed). retransmit_count is the only live cursor — the parallel arrays
// (retransmit_frames/_lengths/_positions/_types/_batch_seq_ids/_seq_with_eob) are
// only ever read over [0, retransmit_count), so zeroing the count discards them.
void cl_arq_controller::clear_retx_queue()
{
	if(retransmit_count != 0)
	{
		printf("[RETX-CLEAR] dropping %d stale retransmit frame(s) on recovery "
			"(old-epoch/old-config bytes; plaintext re-queued for re-send under "
			"the new epoch)\n", retransmit_count);
		fflush(stdout);
	}
	retransmit_count = 0;
}

// R030 (race audit 2026-06-06): resolve which messages_tx[] slot the post-TX
// PENDING_ACK flip must mark for messages_batch_tx[batch_idx]. Returns the
// messages_tx[] array index, or -1 to SKIP (retx-prefix frame with no live
// slot, or no owning slot found). The flip (send_batch) and the regression test
// (--test-v2-pendingack-flip-alias) BOTH call this so the test exercises the
// exact production predicate. See the flip comment in send_batch() and
// data-flow-arq-recovery-cluster.md §4.2 / §5.5.
//
//   - v2 MIXED batch (v2_retx_prefix_count > 0):
//       * batch_idx < v2_retx_prefix_count  -> retx prefix: payload lives in
//         retx_scratch[], original messages_tx[] slot was freed to ACKED at SACK
//         capture (arq_commander.cc:2983). NO live slot -> return -1 (skip).
//       * else (new-data): messages_batch_tx[batch_idx].id is the OVERWRITTEN
//         wire id (pos_in_new_batch), NOT the array index. Find the owning slot
//         by (batch_seq_id, low7-seq) -- the duplicate-(bsi,seq) validation at
//         arq_commander.cc:1797 guarantees that tuple is UNIQUE among populated
//         slots, so the match is unambiguous.
//   - v2 NON-mixed batch (v2_retx_prefix_count == 0) and v1: wire .id ==
//     messages_tx[] array index -> return it directly (legacy behaviour).
int cl_arq_controller::v2_flip_resolve_slot(int batch_idx)
{
	if(v2_retx_prefix_count > 0)
	{
		if(batch_idx < v2_retx_prefix_count)
			return -1;  // retx prefix: no messages_tx[] slot to flip
		int want_bsi = (int)(unsigned char)messages_batch_tx[batch_idx].batch_seq_id;
		int want_seq = (int)((unsigned char)messages_batch_tx[batch_idx].sequence_number & 0x7F);
		for(int s = 0; s < this->nMessages; s++)
		{
			if(messages_tx[s].status == ADDED_TO_BATCH_BUFFER
			   && (int)(unsigned char)messages_tx[s].batch_seq_id == want_bsi
			   && ((int)((unsigned char)messages_tx[s].sequence_number & 0x7F)) == want_seq)
				return s;
		}
		// No owning slot — should not happen (the new-data fill wrote the tuple).
		// Skip rather than alias a wrong slot.
		printf("[CMD-V2-FLIP-NOSLOT] no messages_tx slot for bsi=%d seq=%d "
			"(batch_idx=%d) — skipping PENDING_ACK flip (R030 guard)\n",
			want_bsi, want_seq, batch_idx);
		fflush(stdout);
		return -1;
	}
	return (int)(unsigned char)messages_batch_tx[batch_idx].id;
}

void cl_arq_controller::set_data_batch_size(int data_batch_size)
{
	// CHOKEPOINT: robust => batch 1 (the single enforcement point).
	// At any robust/MFSK config the batch MUST be 1 (all-or-nothing pattern ACK;
	// a clean all-ones batch == one delivered MFSK frame). The CMD and RSP
	// compute their clean-ACK target as (1<<data_batch_size)-1 independently
	// (arq_commander.cc:131/2518, arq_responder.cc:801/1711); if the two sides
	// ever hold different robust batch sizes the targets never match, no clean
	// credit fires, and the gearshift climb cannot start or advance. Four
	// successive wire failures of the climb-fix family were all this same class
	// (CMD/RSP robust-batch mismatch via a different producer each time:
	// SACK-recompute predicate skew, Axis-2 growth, SET_LINK_PARAMS clamp). Per
	// the data-flow-batch-size.md audit (CLAUDE.md §5) we stop guarding each
	// producer in isolation and enforce the invariant HERE, at the sole setter,
	// so no current OR future producer can bypass it.
	//
	// current_configuration is the authoritative live-PHY config: set in
	// load_configuration() (arq_common.cc:1168) BEFORE its own batch sizing runs,
	// and it is the SAME variable the Axis-2 robust guard (policy_evaluate_axis2)
	// and the RSP SACK recompute already key off. NOTE: the SACK-test direct
	// assigns (this->data_batch_size = 25/30, arq_responder.cc:2895/3000)
	// deliberately bypass this setter and are unaffected (OFDM-batch SACK tests).
	// FIX-A (data-flow-robust-tier-arq-batch.md §5.2): the robust clamp is now a
	// RANGE clamp [1..ROBUST_DWELL_BATCH_MAX], not force-to-1. The batch=1 invariant
	// is still the DEFAULT (load_configuration() seeds 1 on every robust load, the
	// connect-path SACK recompute leaves 1, and any out-of-range request is clamped
	// in here), but a PROVEN+PARKED robust dwell may request a multi-frame batch via
	// the dedicated ROBUST_DWELL_BATCH_OP transport. The CMD/RSP-agreement invariant
	// is preserved because the same range clamp runs on BOTH sides' setter (the op
	// applies the SAME value through here on each peer) — there is no asymmetric
	// [10,32] floor like SET_LINK_PARAMS (OR-2 / L4). recalculate_ack_timeout_for_batch()
	// keeps the data-ACK timeout tracking the wider batch (L3): without it CMD times
	// out mid-batch and full-retransmits, the OPPOSITE of FIX-A's goal.
	if(is_robust_config(current_configuration))
	{
		int lo = 1, hi = ROBUST_DWELL_BATCH_MAX;
		int clamped = data_batch_size;
		if(clamped < lo) clamped = lo;
		if(clamped > hi) clamped = hi;
		int prev = this->data_batch_size;
		if(prev != clamped)
		{
			printf("[BATCH-CHOKEPOINT] robust config %d: batch %d -> %d "
				"(robust dwell range [%d,%d]; CMD/RSP must agree)\n",
				current_configuration, data_batch_size, clamped, lo, hi);
			fflush(stdout);
		}
		// R035: re-derive an active prev-batch's counters against the new (smaller)
		// batch BEFORE updating data_batch_size (robust-dwell revert 8->1 is a shrink).
		rescan_prev_on_batch_shrink(clamped);
		this->data_batch_size = clamped;
		// L3: keep the data-ACK timeout tracking the batch ONLY on an actual change
		// to a multi-frame batch (the dwell raise) or back down (the revert). We do
		// NOT recompute on the load_configuration() seed-to-1 (prev already 1): at
		// that point message_transmission_time_ms is not yet recomputed for the new
		// config, and set_ack_timeout_data() would read a stale value. The dwell
		// raise/revert always fire AFTER load_configuration has finished (steady
		// state), so message_transmission_time_ms is current there.
		if(prev != clamped && (prev > 1 || clamped > 1))
			recalculate_ack_timeout_for_batch();
		return;
	}

	if (data_batch_size>0)
	{
		// Resolve the post-clamp target FIRST so R035's prev-shrink rescan sees
		// the value that will actually be stored.
		int target;
		if(data_batch_size<(max_data_length+max_header_length-ACK_MULTI_ACK_RANGE_HEADER_LENGTH-1))
			target = data_batch_size;
		else
			target = (max_data_length+max_header_length-ACK_MULTI_ACK_RANGE_HEADER_LENGTH-1);
		// R035: re-derive an active prev-batch's counters against the new (smaller)
		// batch BEFORE updating data_batch_size (Axis-2 down-move 15->10 is a shrink).
		rescan_prev_on_batch_shrink(target);
		this->data_batch_size = target;
	}
}

void cl_arq_controller::set_control_batch_size(int control_batch_size)
{
	if (control_batch_size>0)
	{
		this->control_batch_size=control_batch_size;
	}
}

void cl_arq_controller::set_role(int role)
{
	if(role==COMMANDER)
	{
		this->role=COMMANDER;
	}
	else
	{
		this->role=RESPONDER;
	}
	calculate_receiving_timeout();
}

void cl_arq_controller::calculate_receiving_timeout()
{
	if(this->role==COMMANDER)
	{
		if(ack_pattern_time_ms > 0)
		{
			// SACK_FIX_PLAN §7 step 3 (Candidate (a)) — geometry-derived
			// post-TX timeout. Replaces the pre-fix formula whose two
			// hardcoded 3000-ms adders were band-aids for the pre-Plan-A
			// turnaround race (now structurally closed by f2dbf34).
			//
			// timeout = frame_drain + sack_arrival + margin
			//   frame_drain  = 2 * message_transmission_time_ms
			//                  (CMD's last frame still in channel)
			//   sack_arrival = ptt_off_delay + RSP_DECODE_MARGIN_MS
			//                  + pattern_time + ptt_on_delay
			//                  (geometry of: CMD->silence; RSP decodes;
			//                   RSP keys SACK; SACK LE at CMD)
			//   margin       = SACK_ARRIVAL_MARGIN_MS (calibrated:
			//                  SACK_FIX_PLAN.md §11.1, worst observed
			//                  arrival 2102 ms vs geometric 1768 ms)
			//
			// sack_timeout_extra_ms is preserved as a runtime override
			// (default now 0); set with --sack-timeout-extra-ms=N if a
			// field deployment surfaces a regression.
			// Step 15: legacy MFSK SACK pattern is gone. OFDM SACK_RSP rides on a
			// normal control-frame TX whose timing is already covered by
			// frame_drain (CMD TX drain) + ack_pattern_time_ms (the ACK-pattern
			// window the RSP keys up after deciding clean batch vs SACK_RSP). No
			// separate pattern_time inflation is needed for v2 SACK.
			int pattern_time = ack_pattern_time_ms;
			int frame_drain  = 2 * message_transmission_time_ms;
			int sack_arrival = ptt_off_delay_ms + RSP_DECODE_MARGIN_MS
			                 + pattern_time + ptt_on_delay_ms;
			int margin       = SACK_ARRIVAL_MARGIN_MS;
			int timeout = frame_drain + sack_arrival + margin;
			// WALL-B FIX-9 D2 (FIX9_ROOTCAUSE.md §4 D2, FIX9_D2_DESIGN.md §3.3): when the data-ACK
			// the CMD is waiting for will be keyed on the robust turnaround geometry (the forward
			// data config is OFDM — reverse_ack_uses_robust_geometry), WIDEN the listen window IN
			// LOCKSTEP with the RSP's matching pre-TX settle (send_mfsk_ack_sack:6581). The two
			// (ptt_off+ptt_on) terms mirror the RSP's new pre-key settle EXACTLY; the drift margin
			// covers the accumulated per-batch clock-slip the stock per-frame frame_drain did not
			// absorb. Without this the CMD window stays narrow and the slipped robust-geometry ACK
			// STILL arrives late -> the FIX9_ROOTCAUSE §5 window-mismatch. Robust forward config:
			// the predicate is FALSE -> adder=0 -> byte-identical.
			//
			// WALL-B FIX-9 D2 REFINE (_fix9/d2refine/D2_REFINE_DESIGN.md §2): gate the widen ALSO on
			// data_ack_retx_turnaround so it fires ONLY on a RETRANSMIT turnaround (the batch carried
			// retx frames OR a recent CFG16 reverse-ACK was lost), NOT on a clean first-pass batch —
			// the clean first-pass ACK decodes WITHOUT the robust geometry (the control proves 0
			// ACK-timeouts), so the +(ptt+drift) widen there is pure cost (~15% clean, recovered
			// here). The CMD-widen condition is a SUPERSET of the RSP-settle condition (a CMD retx
			// batch only exists after an RSP PARTIAL SACK, which set the RSP settle), so the
			// FIX9_ROOTCAUSE §5 lockstep INV-1 (CMD window ⊇ RSP-keyed ACK window) is PRESERVED — see
			// §2.2. FAIL-BEFORE (-DFIX9_D2REFINE_FAILBEFORE): drop the gate -> D2's unconditional
			// widen -> the clean window widens (Part W2a fails).
			bool d2_geometry_fires =
				reverse_ack_uses_robust_geometry(current_configuration)
#ifndef FIX9_D2REFINE_FAILBEFORE
				&& data_ack_retx_turnaround
#endif
				;
			if(d2_geometry_fires)
				timeout += ptt_off_delay_ms + ptt_on_delay_ms + ROBUST_ACK_DRIFT_MARGIN_MS;
			// During turboshift, RSP calls load_configuration() on every probe,
			// adding ~200-500ms overhead. Extend receive window to prevent
			// premature timeout before ACK arrives.
			if(gear_shift_on && turboshift_phase != TURBO_DONE)
				timeout += 2000;
			// Runtime override safety net (--sack-timeout-extra-ms=N).
			// Default 0 post-fix; re-inflate at runtime if needed.
			if(sack_enabled)
				timeout += sack_timeout_extra_ms;
			printf("[CMD-POST-TX-CALIB] timeout=%dms = frame_drain=%d + sack_arrival=%d (ptt_off=%d + rsp_decode=%d + pattern=%d + ptt_on=%d) + margin=%d + extra=%d + d2_robust_ack=%d (retx_turn=%d) batch=%d sack=%d\n",
				timeout, frame_drain, sack_arrival,
				ptt_off_delay_ms, RSP_DECODE_MARGIN_MS, pattern_time, ptt_on_delay_ms,
				margin, sack_enabled ? sack_timeout_extra_ms : 0,
				d2_geometry_fires
					? (ptt_off_delay_ms + ptt_on_delay_ms + ROBUST_ACK_DRIFT_MARGIN_MS) : 0,
				(int)data_ack_retx_turnaround,
				data_batch_size, sack_enabled ? 1 : 0);
			fflush(stdout);
			set_receiving_timeout(timeout);
		}
		else
		{
			set_receiving_timeout((ack_batch_size+1)*ctrl_transmission_time_ms+time_left_to_send_last_frame+ptt_on_delay_ms);
		}
	}
	else
	{
		int rsp_timeout = (data_batch_size)*message_transmission_time_ms+time_left_to_send_last_frame+ptt_on_delay_ms;
		// RSP timeout: base timeout covers batch reception
		printf("[RSP-TIMEOUT] batch=%d msg_time=%d time_left=%d ptt=%d sack=%d -> timeout=%d\n",
			data_batch_size, message_transmission_time_ms, time_left_to_send_last_frame, ptt_on_delay_ms, sack_enabled ? 1 : 0, rsp_timeout);
		fflush(stdout);
		set_receiving_timeout(rsp_timeout);
	}
}

void cl_arq_controller::recalculate_ack_timeout_for_batch()
{
	if(ack_pattern_time_ms > 0)
	{
		// Step 15: legacy MFSK SACK pattern is gone. OFDM SACK_RSP rides on
		// the normal control-frame TX timing — pattern_time stays equal to
		// ack_pattern_time_ms.
		int pattern_time = ack_pattern_time_ms;
		set_ack_timeout_data((data_batch_size+2)*message_transmission_time_ms + pattern_time
			+ 4*ptt_on_delay_ms + 4*ptt_off_delay_ms + 3000);
	}
	else
		set_ack_timeout_data((data_batch_size+1)*message_transmission_time_ms+control_batch_size*message_transmission_time_ms+2*ack_batch_size*ctrl_transmission_time_ms+time_left_to_send_last_frame+4*ptt_on_delay_ms+4*ptt_off_delay_ms);
}

// SACK-negotiation batch recompute — the single shared body for the CMD
// (TEST_CONNECTION_ACK, arq_commander.cc) and RSP (TEST_CONNECTION,
// arq_responder.cc) handlers. Extracting it makes the two sides run IDENTICAL
// code so they CANNOT diverge on data_batch_size — the root failure mode of the
// climb-fix family (4 wire failures, all CMD/RSP robust-batch mismatch). See
// data-flow-batch-size.md §4/§5.
//
// Gates on current_configuration (the live-PHY config). On a robust connect this
// is ROBUST_0 (set by the startup load_configuration(data_configuration=ROBUST_0)
// and every robust control-frame TX); on an OFDM connect it is the OFDM config.
// negotiated_configuration is NOT usable here: on a fresh unpinned `-g -R`
// connect it is still its ctor-default CONFIG_0 (the connect path never writes
// it), which made the old CMD gate (is_robust_config(negotiated_configuration))
// falsely recompute at ROBUST_0 -> CMD batch=5 vs RSP batch=1 -> climb never
// started. The set_data_batch_size() chokepoint backstops robust => batch 1.
void cl_arq_controller::sack_negotiated_recompute_batch(const char* who)
{
	// ROBUST/MFSK configs are EXCLUDED from the >=5 floor: load_configuration()
	// (arq_common.cc:1229-1234) pins data_batch_size=1 for robust and its OFDM
	// batch-scaling (:1269) is already !is_robust_config-gated. Re-applying the
	// SACK floor at robust would clobber the pin and force batch>=5, where a
	// clean all-ones MFSK ACK requires every frame to survive first-pass at the
	// floor SNR (it never does). At batch=1 every delivered MFSK frame is itself
	// an all-ones batch -> clean ACKs accumulate and the climb advances.
	// SACK-GATE P1 (R-B, bug #9) — BIG-BLOCK RUNG PIN: data_batch_size == K.
	// The big-block framing rung is a CFG16 (OFDM, NON-robust) config, so without
	// this pin the non-robust branch below runs the 30s-target formula and elects
	// data_batch_size ~= 25. But a big-block decode emits ONE K-bit SACK bitmap
	// (cw_ok, K = nBits/ldpc.N at the thin grid, K=8 at CFG16). The CMD clean-ACK
	// accept gate (cmd_clean_data_ack_crc_valid, arq_commander.cc:136-139) derives
	// all_ones = (1<<data_batch_size)-1; with batch=25 that is 0x1FFFFFF, which can
	// NEVER equal the RSP's all-clean K=8 bitmap 0xFF -> clean ACK never matches ->
	// zero clean credit -> the #9 "4 wire failures". Pinning data_batch_size = K
	// makes CMD all_ones == 0xFF == the RSP big-block bitmap.
	//
	// This runs in the SHARED election body (CMD via TEST_CONNECTION_ACK +
	// arq_commander.cc, RSP via TEST_CONNECTION + arq_responder.cc), so BOTH peers
	// elect K from the SAME PHY geometry source (telecom_system->
	// bigblock_codeword_count(), the identical nBits/ldpc.N the TX/RX workers use)
	// and CANNOT diverge — the divergence-proof property the climb-fix family lacked.
	// It is a BATCH-SIZE election at the CFG16 rung, NOT an authority change: the
	// optimizer/gearshift authority (optimizer_is_in_control, last_data_viable_config,
	// anchor_consec_break_fails, probe_backoff) is UNTOUCHED. The rung is detected by
	// the framing-mode flag (telecom_system->bigblock_framing_enabled), the CFG16-rung
	// framing bit the gearshift elects. set_data_batch_size()'s non-robust branch
	// accepts K (>0, below the cap) cleanly; recalculate_ack_timeout_for_batch()
	// (below) re-sizes the data-ACK timeout for the pinned batch.
	// P3 HW FIX: pin batch=K ONLY at the CONFIG_16 big-block rung (was: any non-robust
	// config). At CONFIG_0..15 bigblock_codeword_count() returns that config's K (e.g. 1)
	// and pinning batch=1 there, combined with the carve engaging, stalled the climb. The
	// block framing only runs at CONFIG_16 (see bigblock_send_one_block / the RX carve), so
	// the K-pin belongs there too; CONFIG_0..15 use the stock 30s-target batch below.
	bool bigblock_rung = (telecom_system != NULL)
	                   && telecom_system->bigblock_framing_enabled
	                   && current_configuration == CONFIG_16;
	if(bigblock_rung)
	{
		int K = telecom_system->bigblock_codeword_count();
		if(K > 0)
		{
			if(K > nMessages) K = nMessages;
			set_data_batch_size(K);
			nominal_batch_size = K;
		}
	}
	else if(!is_robust_config(current_configuration))
	{
		int max_batch = (message_transmission_time_ms > 0)
			? (int)(30000.0 / message_transmission_time_ms + 0.5) : 31;
		if(max_batch < 5) max_batch = 5;
		if(max_batch > nMessages) max_batch = nMessages;
		int new_batch = radio_batch_size;
		if(new_batch > max_batch) new_batch = max_batch;
		set_data_batch_size(new_batch);
		nominal_batch_size = new_batch;
	}
	recalculate_ack_timeout_for_batch();
	printf("[SACK] %s Enabled (radio_batch=%d crypto_batch=%d headroom=%d batch=%d robust=%d bigblock=%d)\n",
		who ? who : "?", radio_batch_size, crypto_batch_size, retransmit_headroom, data_batch_size,
		is_robust_config(current_configuration) ? 1 : 0, bigblock_rung ? 1 : 0);
}

void cl_arq_controller::set_call_sign(std::string call_sign)
{
	if(call_sign!= "")
	{
		this->my_call_sign=call_sign;
	}
}

int cl_arq_controller::get_nOccupied_messages()
{
	int nOccupied_messages=0;
	for(int i=0;i<this->nMessages;i++)
	{
		if(this->messages_tx[i].status!=FREE)
		{
			nOccupied_messages++;
		}
	}
	return nOccupied_messages;
}

int cl_arq_controller::get_nFree_messages()
{
	int nFree_messages=0;
	for(int i=0;i<this->nMessages;i++)
	{
		if(this->messages_tx[i].status==FREE)
		{
			nFree_messages++;
		}
	}
	return nFree_messages;
}

int cl_arq_controller::get_nTotal_messages()
{
	return this->nMessages;
}

int cl_arq_controller::get_nToSend_messages()
{
	int nMessages_to_send=0;
	for(int i=0;i<this->nMessages;i++)
	{
		if(messages_tx[i].status==ADDED_TO_LIST)
		{
			nMessages_to_send++;
		}
		else if (messages_tx[i].status==ACK_TIMED_OUT && messages_tx[i].nResends>0)
		{
			nMessages_to_send++;
		}
	}
	return nMessages_to_send;
}

int cl_arq_controller::get_nPending_Ack_messages()
{
	int nPending_Ack_messages=0;
	for(int i=0;i<this->nMessages;i++)
	{
		if(this->messages_tx[i].status==PENDING_ACK)
		{
			nPending_Ack_messages++;
		}
	}
	return nPending_Ack_messages;
}

int cl_arq_controller::get_nReceived_messages()
{
	int nReceived_messages=0;
	for(int i=0;i<this->nMessages;i++)
	{
		if(this->messages_rx[i].status==RECEIVED)
		{
			nReceived_messages++;
		}
	}
	return nReceived_messages;
}

int cl_arq_controller::get_nAcked_messages()
{
	int nAcked_messages=0;
	for(int i=0;i<this->nMessages;i++)
	{
		if(this->messages_rx[i].status==ACKED)
		{
			nAcked_messages++;
		}
	}
	return nAcked_messages;
}


void cl_arq_controller::messages_control_backup()
{
	messages_control_bu.ack_timeout=messages_control.ack_timeout;
	messages_control_bu.id=messages_control.id;
	messages_control_bu.length=messages_control.length;
	messages_control_bu.nResends=messages_control.nResends;
	messages_control_bu.status=messages_control.status;
	messages_control_bu.type=messages_control.type;
	int copy_len=max_data_length+max_header_length-CONTROL_ACK_CONTROL_HEADER_LENGTH;
	if(copy_len > N_MAX/8) copy_len = N_MAX/8;
	for(int i=0;i<copy_len;i++)
	{
		messages_control_bu.data[i]=messages_control.data[i];
	}
}
void cl_arq_controller::messages_control_restore()
{
	int copy_len=max_data_length+max_header_length-CONTROL_ACK_CONTROL_HEADER_LENGTH;
	if(copy_len > N_MAX/8) copy_len = N_MAX/8;
	for(int i=0;i<copy_len;i++)
	{
		messages_control.data[i]=messages_control_bu.data[i];
	}
	messages_control.ack_timeout=messages_control_bu.ack_timeout;
	messages_control.id=messages_control_bu.id;
	messages_control.length=messages_control_bu.length;
	messages_control.nResends=messages_control_bu.nResends;
	messages_control.status=messages_control_bu.status;
	messages_control.type=messages_control_bu.type;
	messages_control.ack_timer.start();
}


int cl_arq_controller::init(int tcp_base_port, int gear_shift_on, int initial_mode)
{
	int success=SUCCESSFUL;

	fifo_buffer_tx.set_size(default_configuration_ARQ.fifo_buffer_tx_size);
	fifo_buffer_rx.set_size(default_configuration_ARQ.fifo_buffer_rx_size);
	fifo_buffer_backup.set_size(default_configuration_ARQ.fifo_buffer_backup_size);

	set_link_timeout(default_configuration_ARQ.link_timeout);

	if (tcp_base_port)
	{
		tcp_socket_control.port = tcp_base_port;
		tcp_socket_data.port = tcp_base_port + 1;
	}
	else
	{
		tcp_socket_control.port = default_configuration_ARQ.tcp_socket_control_port;
		tcp_socket_data.port = default_configuration_ARQ.tcp_socket_data_port;
	}

	tcp_socket_control.timeout_ms=default_configuration_ARQ.tcp_socket_control_timeout_ms;
	tcp_socket_data.timeout_ms=default_configuration_ARQ.tcp_socket_data_timeout_ms;

	this->gear_shift_on = gear_shift_on;
	gear_shift_algorithm=default_configuration_ARQ.gear_shift_algorithm;
	current_configuration=CONFIG_NONE;

	if(robust_enabled)
	{
		// ROBUST mode: use requested ROBUST config for all roles
		// Gearshift between ROBUST levels handled separately
		init_configuration = initial_mode;
		data_configuration = initial_mode;
		ack_configuration = initial_mode;
	}
	else
	{
		init_configuration = initial_mode;
		data_configuration = initial_mode;
		ack_configuration=default_configuration_ARQ.ack_configuration;
	}

	// AUTHORITATIVE data-viability anchor seat (gearshift-climb-engine.md §17).
	// The ctor seated CONFIG_0 (robust_enabled / init_configuration were not yet
	// set there), and the CONNECT command handler (process_user_command) does NOT
	// call reset_session_state — so on the FIRST connect the anchor would otherwise
	// hold the ctor's CONFIG_0 while load_configuration() drives current_configuration
	// to the real start (ROBUST_0 on a -R session). That divergence
	// (anchor=CONFIG_0=OFDM, live=ROBUST_0) opened the §15 SUPERSHIFT re-trigger gate
	// is_ofdm_config(anchor) at t=0 and rocketed the climb to CONFIG_9 at WGN:-10.
	// Seat it HERE, after the robust/non-robust branch has resolved init_configuration
	// and AFTER robust_enabled is known (main.cc sets ARQ.robust_enabled before
	// init()): session_floor_anchor() returns ROBUST_0 on a -R session (gate CLOSED at
	// t=0) and init_configuration (the start/pinned config) otherwise.
	last_data_viable_config = session_floor_anchor(robust_enabled, init_configuration);

	if(tcp_socket_data.init()!=SUCCESS || tcp_socket_control.init()!=SUCCESS )
	{
		printf("Error initializing the TCP sockets. Exiting..\n");
		exit(-1);
	}

	load_configuration(ack_configuration,FULL,NO);
	load_configuration(data_configuration,PHYSICAL_LAYER_ONLY,YES);

	// Phase 3c (Effective-Rate Optimizer) — load calibration table once at
	// init. Inert if missing; only enables on parse success. Path override
	// via MERCURY_RATE_TABLE env var; defaults to relative path.
	opt_load_rate_table();

	print_stats_timer.start();

//	TEST TX data
//		process_user_command("MYCALL rx001");
//		process_user_command("LISTEN ON");
//
//		process_user_command("MYCALL tx001");
//		process_user_command("CONNECT tx001 rx001");


//		std::string str="sent_quest1234";
//		char data;
//
//		for(int i=0;i<str.length();i++)
//		{
//			data=(char)str[i];
//			fifo_buffer_tx.push(&data,1);
//		}

	return success;
}

void cl_arq_controller::init_monitor_decoders()
{
	if(!passive_monitor || monitor_decoders_ready) return;

	printf("[MONITOR] Initializing %d parallel OFDM decoders...\n", NUMBER_OF_CONFIGS);
	fflush(stdout);

	// First, determine the largest buffer_Nsymb we need.
	// Init a temporary decoder as CONFIG_0 (BPSK 1/16 = largest OFDM frame)
	// to read its auto-calculated buffer_Nsymb.
	{
		cl_telecom_system tmp;
		tmp.narrowband_enabled = telecom_system->narrowband_enabled;
		tmp.load_configuration(CONFIG_0);
		monitor_primary_buffer_nsymb = tmp.data_container.buffer_Nsymb.load();
		printf("[MONITOR] CONFIG_0 buffer_Nsymb = %d (used as minimum for all decoders)\n",
			monitor_primary_buffer_nsymb);
		// tmp destructs here, freeing its buffers
	}

	for(int cfg = 0; cfg < NUMBER_OF_CONFIGS; cfg++)
	{
		monitor_decoders[cfg] = new cl_telecom_system();
		monitor_decoders[cfg]->narrowband_enabled = telecom_system->narrowband_enabled;
		// Force all decoders to use the largest buffer size so they can all
		// process the same audio snapshot (CONFIG_0 has the most symbols).
		monitor_decoders[cfg]->data_container.buffer_Nsymb_min = monitor_primary_buffer_nsymb;
		monitor_decoders[cfg]->load_configuration(cfg);
		printf("[MONITOR] Decoder CONFIG_%d ready (Nsymb=%d buffer_Nsymb=%d)\n",
			cfg, monitor_decoders[cfg]->data_container.Nsymb,
			monitor_decoders[cfg]->data_container.buffer_Nsymb.load());
	}

	// Set buffer_Nsymb_min on the primary telecom_system so the capture
	// buffer is large enough for the largest OFDM frame. Takes effect on
	// next load_configuration() call (first OFDM config switch via SET_CONFIG).
	// During MFSK phase, parallel OFDM decode is not used (MFSK uses the
	// regular single-decoder path), so the MFSK buffer size is fine.
	telecom_system->data_container.buffer_Nsymb_min = monitor_primary_buffer_nsymb;

	monitor_decoders_ready = true;
	printf("[MONITOR] All %d parallel decoders initialized.\n", NUMBER_OF_CONFIGS);
	fflush(stdout);
}

void cl_arq_controller::reinit_monitor_decoders()
{
	if(!passive_monitor) return;

	printf("[MONITOR] Reinitializing parallel decoders for %s mode...\n",
		telecom_system->narrowband_enabled ? "NB" : "WB");
	fflush(stdout);

	// Destroy existing decoders
	for(int cfg = 0; cfg < NUMBER_OF_CONFIGS; cfg++)
	{
		if(monitor_decoders[cfg])
		{
			delete monitor_decoders[cfg];
			monitor_decoders[cfg] = NULL;
		}
	}
	monitor_decoders_ready = false;

	// Recalculate buffer_Nsymb for the new bandwidth
	{
		cl_telecom_system tmp;
		tmp.narrowband_enabled = telecom_system->narrowband_enabled;
		tmp.load_configuration(CONFIG_0);
		monitor_primary_buffer_nsymb = tmp.data_container.buffer_Nsymb.load();
		printf("[MONITOR] CONFIG_0 buffer_Nsymb = %d (new bandwidth)\n",
			monitor_primary_buffer_nsymb);
	}

	// Recreate all decoders with correct bandwidth
	for(int cfg = 0; cfg < NUMBER_OF_CONFIGS; cfg++)
	{
		monitor_decoders[cfg] = new cl_telecom_system();
		monitor_decoders[cfg]->narrowband_enabled = telecom_system->narrowband_enabled;
		monitor_decoders[cfg]->data_container.buffer_Nsymb_min = monitor_primary_buffer_nsymb;
		monitor_decoders[cfg]->load_configuration(cfg);
		printf("[MONITOR] Decoder CONFIG_%d ready (Nsymb=%d buffer_Nsymb=%d)\n",
			cfg, monitor_decoders[cfg]->data_container.Nsymb,
			monitor_decoders[cfg]->data_container.buffer_Nsymb.load());
	}

	telecom_system->data_container.buffer_Nsymb_min = monitor_primary_buffer_nsymb;
	monitor_decoders_ready = true;
	printf("[MONITOR] Parallel decoders reinitialized (%d decoders, %s).\n",
		NUMBER_OF_CONFIGS, telecom_system->narrowband_enabled ? "NB" : "WB");
	fflush(stdout);
}

int cl_arq_controller::parallel_monitor_decode(double* audio, int audio_len,
                                                st_receive_stats& out_stats)
{
	if(!monitor_decoders_ready) return -1;

	// === GATE 1: Energy check ===
	// Quick scan for signal presence. If all samples below threshold,
	// no OFDM frame exists — skip everything. Eliminates decode attempts
	// during silence (~50% of iterations).
	{
		double peak = 0;
		int step = 64;
		for(int i = 0; i < audio_len; i += step)
		{
			double v = fabs(audio[i]);
			if(v > peak) peak = v;
		}
		if(peak < 0.05)
		{
			out_stats.message_decoded = NO;
			out_stats.delay = -1;
			return -1;
		}
	}

	// === Sequential decode with config memory + protocol knowledge ===
	// Try order priority:
	//   1. forward_configuration (from SET_CONFIG — the config commander is sending)
	//   2. reverse_configuration (responder→commander config after SWITCH_ROLE)
	//   3. Last 2 successfully decoded configs (handles alternation patterns)
	//   4. Remaining configs (rare — only on first encounter or after long gap)
	// The monitor doesn't need real-time decode — the ring buffer holds
	// ~5s (WB) to ~54s (NB) of audio. Worst case: 17 sequential attempts
	// × ~30ms = ~500ms. Still far faster than real-time and uses 1 CPU core.
	static int last_config_a = 0;  // Most recent successful config
	static int last_config_b = -1; // Second most recent (different from a)

	// Build try order: protocol-known configs first, then history, then rest
	int try_order[NUMBER_OF_CONFIGS];
	int idx = 0;
	bool used[NUMBER_OF_CONFIGS] = {};

	// Protocol-level knowledge: SET_CONFIG tells us exactly which configs are active
	int fwd = (int)this->forward_configuration;
	int rev = (int)this->reverse_configuration;
	if(fwd >= 0 && fwd < NUMBER_OF_CONFIGS && !used[fwd])
	{
		try_order[idx++] = fwd;
		used[fwd] = true;
	}
	if(rev >= 0 && rev < NUMBER_OF_CONFIGS && !used[rev])
	{
		try_order[idx++] = rev;
		used[rev] = true;
	}
	// History-based: last 2 successful configs
	if(last_config_a >= 0 && last_config_a < NUMBER_OF_CONFIGS && !used[last_config_a])
	{
		try_order[idx++] = last_config_a;
		used[last_config_a] = true;
	}
	if(last_config_b >= 0 && last_config_b < NUMBER_OF_CONFIGS && !used[last_config_b])
	{
		try_order[idx++] = last_config_b;
		used[last_config_b] = true;
	}
	// Fill remaining
	for(int cfg = 0; cfg < NUMBER_OF_CONFIGS; cfg++)
	{
		if(!used[cfg])
			try_order[idx++] = cfg;
	}

	bool preamble_seen = false;

	for(int t = 0; t < NUMBER_OF_CONFIGS; t++)
	{
		int cfg = try_order[t];
		cl_telecom_system* dec = monitor_decoders[cfg];

		int dec_buf_len = dec->data_container.Nofdm
			* dec->data_container.buffer_Nsymb.load()
			* dec->data_container.interpolation_rate;
		int copy_len = (audio_len < dec_buf_len) ? audio_len : dec_buf_len;
		memcpy(dec->data_container.ready_to_process_passband_delayed_data,
			audio, copy_len * sizeof(double));
		if(copy_len < dec_buf_len)
			memset(&dec->data_container.ready_to_process_passband_delayed_data[copy_len],
				0, (dec_buf_len - copy_len) * sizeof(double));

		st_receive_stats stats = dec->receive_byte(
			dec->data_container.ready_to_process_passband_delayed_data,
			dec->data_container.data_byte);

		if(stats.message_decoded == YES)
		{
			if(cfg != last_config_a)
			{
				last_config_b = last_config_a;
				last_config_a = cfg;
			}
			// Save decoded data to staging buffer — NOT to primary's data_byte.
			// load_configuration() may deinit/reinit the primary on cross-modulation
			// switches (BPSK→QPSK etc), destroying data_byte. The caller copies
			// from monitor_decoded_data to primary AFTER load_configuration.
			monitor_decoded_len = dec->get_frame_size_bytes();
			if(monitor_decoded_len > N_MAX / 8) monitor_decoded_len = N_MAX / 8;
			memcpy(monitor_decoded_data,
				dec->data_container.data_byte,
				monitor_decoded_len * sizeof(int));
			out_stats = stats;
			printf("[MONITOR] CONFIG_%d DECODED (SNR=%.1f iter=%d, tried %d/%d)\n",
				cfg, stats.SNR, stats.iterations_done, t + 1, NUMBER_OF_CONFIGS);
			fflush(stdout);
			return cfg;
		}

		if(stats.delay >= 0)
			preamble_seen = true;

		// Preamble gate: if the first decoder found no preamble,
		// no other config will either (Schmidl-Cox is config-independent).
		if(t == 0 && !preamble_seen)
		{
			out_stats.message_decoded = NO;
			out_stats.delay = -1;
			return -1;
		}
	}

	// All 17 configs tried, none decoded (interference / noise)
	out_stats.message_decoded = NO;
	out_stats.delay = -1;
	return -1;
}

int cl_arq_controller::get_configuration(double SNR)
{
	int configuration;
	configuration =telecom_system->get_configuration(SNR);
	return configuration;
}

void cl_arq_controller::load_configuration(int configuration, int level, int backup_configuration)
{
	printf("[CFG] load_configuration(%d) current=%d level=%s backup=%s\n",
		configuration, this->current_configuration,
		level == FULL ? "FULL" : "PHYS_ONLY",
		backup_configuration == YES ? "YES" : "NO");
	if(configuration==this->current_configuration)
	{
		printf("[CFG] Already on config %d, skipping\n", configuration);
		return;
	}
	if(current_configuration!=CONFIG_NONE)
	{
		if(level==FULL)
		{
			printf("[CANARY] Pre-FULL-deinit canary check (config %d -> %d)\n",
				current_configuration, configuration);
			fflush(stdout);
			check_buffer_canaries("pre_FULL_deinit");
			this->restore_backup_buffer_data();
			this->deinit_messages_buffers();
		}
		if(backup_configuration==YES)
		{
			this->last_data_configuration= this->current_configuration;
		}
	}
	else
	{
		if(level==FULL)
		{
			printf("[CFG] CONFIG_NONE path: deinit_messages_buffers\n");
			fflush(stdout);
			this->deinit_messages_buffers();
			printf("[CFG] CONFIG_NONE path: deinit_messages_buffers done\n");
			fflush(stdout);
		}
		if(backup_configuration==YES)
		{
			this->last_data_configuration=configuration;
		}
	}

	this->current_configuration=configuration;

	// WALL-B FIX-3 (C1 reset): a REAL arq-layer config change (we passed the no-op early-return
	// at the top, so configuration != the previous current_configuration) starts the new config
	// with a CLEAN carve-fail streak. This (a) re-arms the carve on the FIRST block of a fresh
	// CFG16 visit after a session reset + re-climb (RISK-D: a stale streak must not suspend the
	// first block), and (b) lets a CFG16 carve that suspended -> demoted the link to CFG15 leave
	// the suspended state cleanly. NO-OP off any big-block path (the streak is 0 there anyway).
	this->bigblock_rx_carve_fail_streak = 0;

	// Canary check during PHYS_ONLY transitions — track when corruption first appears
	if(level != FULL)
	{
		check_buffer_canaries("PHYS_ONLY_transition");
	}

	// Pause audio capture during PHY reconfig to prevent race condition:
	// telecom_system->load_configuration may deinit/reinit buffers that
	// the audio callback accesses (passband_delayed_data, etc.)
	telecom_system->data_container.frames_to_read = 0;
	printf("[CFG] Calling telecom_system->load_configuration(%d) nb=%d\n",
		configuration, telecom_system->narrowband_enabled);
	fflush(stdout);
	telecom_system->load_configuration(configuration);
	printf("[CFG] telecom_system->load_configuration done\n");
	fflush(stdout);

	// Note: after config switch (e.g. MFSK→OFDM), the zeroed buffer may still
	// receive stale audio from VB-Cable's internal buffer (~7 symbols). This can
	// cause Schmidl-Cox false triggers on MFSK remnants. However, extended buffer
	// flushing causes gearshift timeouts (NB CONFIG_0 buffer_Nsymb=581 → 13.2s).
	// The OFDM decoder's energy gate + mean_H threshold handle stale data adequately.
	// WB OFDM fails on VB-Cable (cause under investigation — likely TX clipping
	// or Moose freq sync issue, not VB-Cable itself). On real radio, the buffer
	// starts zeroed (memset in set_size) and VB-Cable latency is not an issue.
	int nBytes_header=0;
	if (ACK_MULTI_ACK_RANGE_HEADER_LENGTH>nBytes_header) nBytes_header=ACK_MULTI_ACK_RANGE_HEADER_LENGTH;
	if (CONTROL_ACK_CONTROL_HEADER_LENGTH>nBytes_header) nBytes_header=CONTROL_ACK_CONTROL_HEADER_LENGTH;
	// SACK Design A Steps 1+2 — DATA_LONG header is 4 bytes (v1) or 5 bytes (v2);
	// DATA_SHORT header is 5 bytes (v1) or 6 bytes (v2).
	// §7.13.34 fix: size for the LARGER (v2) value unconditionally, not gated
	// on sack_v2_enabled. The previous gating broke when SACK v2 capability was
	// negotiated AFTER load_configuration() ran (no re-init triggered, so
	// max_header_length stayed at v1 size, then the first v2 DATA_LONG frame
	// blew past the buffer and exit(0) at line ~3286). Cost is one extra byte
	// of header buffer per frame — negligible (≤0.7% of payload at CFG15).
	{
		int eff_long = effective_data_long_header_length(true);
		if (eff_long>nBytes_header) nBytes_header=eff_long;
		int eff_short = effective_data_short_header_length(true);
		if (eff_short>nBytes_header) nBytes_header=eff_short;
	}

	int nBytes_data=(telecom_system->data_container.nBits-telecom_system->ldpc.P-telecom_system->outer_code_reserved_bits)/8 - nBytes_header;
	int nBytes_message=(telecom_system->data_container.nBits)/8 ;


	set_max_buffer_length(nBytes_data, nBytes_message, nBytes_header);
	set_nMessages(default_configuration_ARQ.nMessages);
	set_nResends(default_configuration_ARQ.nResends);

	if(level==FULL)
	{
		set_data_batch_size(default_configuration_ARQ.batch_size);
	}
	set_ack_batch_size(default_configuration_ARQ.ack_batch_size);
	set_control_batch_size(default_configuration_ARQ.control_batch_size);

	// MFSK modes: single ACK/control frame per batch (no redundant copies).
	// LDPC provides cliff-effect protection; if a frame decodes, it's correct.
	// Saves 1 frame per ACK cycle + 1 per control cycle = major time savings
	// at 4.6-7.3s per frame.
	if(is_robust_config(configuration))
	{
		set_ack_batch_size(1);
		set_control_batch_size(1);
		// FIX-A P2 (adversarial-review fix, 2026-06-03): the robust DATA-batch reset
		// (set_data_batch_size(1) + robust_dwell_batch_active=false) is DEFERRED to
		// after the message_transmission_time_ms recompute below — see the relocated
		// block. The ack/control batch resets above are timing-independent and stay
		// here. ack/control single-frame: LDPC provides cliff-effect protection; if a
		// frame decodes it's correct. Saves 1 frame per ACK/control cycle (major at
		// 4.6-7.3s per frame).
	}

	gear_shift_up_success_rate_precentage=default_configuration_ARQ.gear_shift_up_success_rate_limit_precentage;
	gear_shift_down_success_rate_precentage=default_configuration_ARQ.gear_shift_down_success_rate_limit_precentage;

	gear_shift_block_for_nBlocks_total=default_configuration_ARQ.gear_shift_block_for_nBlocks_total;
	gear_shift_blocked_for_nBlocks=0;
	consecutive_data_acks=0;
	// NOTE: turboshift state is NOT reset here — it persists across config changes.
	// Only reset at connection init (see init code above).

	message_transmission_time_ms=ceil((1000.0*(telecom_system->data_container.Nsymb+telecom_system->data_container.preamble_nSymb)*telecom_system->data_container.Nofdm*telecom_system->frequency_interpolation_rate)/(float)(telecom_system->frequency_interpolation_rate*(telecom_system->bandwidth/telecom_system->ofdm.Nc)*telecom_system->ofdm.Nfft));
	if(telecom_system->ctrl_nsymb > 0)
	{
		ctrl_transmission_time_ms=ceil((1000.0*(telecom_system->ctrl_nsymb+telecom_system->data_container.preamble_nSymb)*telecom_system->data_container.Nofdm*telecom_system->frequency_interpolation_rate)/(float)(telecom_system->frequency_interpolation_rate*(telecom_system->bandwidth/telecom_system->ofdm.Nc)*telecom_system->ofdm.Nfft));
	}
	else
	{
		ctrl_transmission_time_ms=message_transmission_time_ms;
	}
    // TODO: After audio I/O rewrite we don't use this anymore. Was:
	// time_left_to_send_last_frame=(float)telecom_system->speaker.frames_to_leave_transmit_fct/(float)(telecom_system->frequency_interpolation_rate*(telecom_system->bandwidth/telecom_system->ofdm.Nc)*telecom_system->ofdm.Nfft);
    time_left_to_send_last_frame=0;

	// FIX-A P2 (adversarial-review fix, 2026-06-03): RELOCATED robust DATA-batch reset.
	// data-flow-robust-tier-arq-batch.md §5.3 — every robust config load (connect,
	// BREAK→ROBUST_0, turbo-reverse, ROBUST_0→ROBUST_1 climb step) resets the dwell
	// batch to its initial pin of 1 AND clears the raised flag, so a later proven+parked
	// dwell at the NEW rung must re-earn the raise. This is the config-change revert leg
	// of the symmetric revert — it runs on BOTH the CMD (which then re-evaluates) and the
	// RSP (which adopts via the op).
	//
	// WHY HERE (not back at the is_robust_config(configuration) reset block above): on a
	// robust→robust reload while the dwell was RAISED (e.g. prev batch=4 → 1 on a
	// ROBUST_0→ROBUST_1 step) the chokepoint's recalculate_ack_timeout_for_batch()
	// (set_data_batch_size, arq_common.cc:~700) fires because prev>1. If the reset ran
	// before message_transmission_time_ms is recomputed (the OLD site), that recompute
	// read the STALE old-config frame time. Placing the reset AFTER the
	// message_transmission_time_ms / ctrl_transmission_time_ms recompute above guarantees
	// the chokepoint recompute uses the NEW config's frame time (L3). nominal_batch_size
	// is set immediately below from the now-correct data_batch_size=1.
	if(is_robust_config(configuration))
	{
		set_data_batch_size(1);
		robust_dwell_batch_active = false;
	}

	// Scale data_batch_size based on block duration (OFDM modes only).
	// MFSK modes keep batch_size=1 for pattern ACK optimization.
	// Batch sizing: with all-or-nothing MFSK ACK, P(batch success) = p_ofdm^N × p_ack.
	// Start conservative (5 frames), adaptive growth finds optimal batch for link quality.
	// nominal_batch_size = ceiling from target_time_ms — adaptive mechanism grows toward it.
	// 30 s target (was 12 s): longer batches amortize the ACK turnaround over
	// more frames, raising wire efficiency. At target=30000 WB CFG6 reaches
	// batch=25 (vs 10 previously); NB CFG7+ unblocks from the 5-frame floor.
	// SACK bitmap cap is 32 frames (MAX_SACK_BATCH_SIZE), and radio_batch_size
	// stays at 25 by default — fits comfortably. Synchronized CMD/RSP via the
	// same formula in arq_commander.cc:3133 and arq_responder.cc:1708.
	if(!is_robust_config(configuration) && message_transmission_time_ms > 0)
	{
		int target_time_ms = 30000;
		int max_batch = (int)((float)target_time_ms / message_transmission_time_ms + 0.5);
		if(max_batch < 5) max_batch = 5;
		if(max_batch > nMessages) max_batch = nMessages;
		// Batch sizing: with SACK, partial batches are retransmitted selectively,
		// so larger batches improve duty cycle without risking full retransmission.
		// Without SACK: batch=10 balances duty cycle vs batch failure probability.
		int fixed_batch = sack_enabled ? radio_batch_size : 10;
		if(fixed_batch > max_batch) fixed_batch = max_batch;
		set_data_batch_size(fixed_batch);
		nominal_batch_size = fixed_batch;
		printf("[CFG] Batch scaling: msg_time=%dms initial=%d max=%d nMessages=%d\n",
			message_transmission_time_ms, data_batch_size, max_batch, nMessages);
		fflush(stdout);
	}
	else
	{
		nominal_batch_size = data_batch_size;
	}

	// ACK pattern transmission time (universal: all modes)
	if(telecom_system->ack_pattern_passband_samples > 0)
	{
		ack_pattern_time_ms = (int)ceil(1000.0 * telecom_system->ack_pattern_passband_samples / telecom_system->sampling_frequency);
	}
	else
	{
		ack_pattern_time_ms = 0;
	}

	// With pattern-based ACK, control_batch_size=1: LDPC cliff effect makes
	// redundant copies wasteful, and halving TX means faster ACK round-trip.
	// ack_batch_size=1: irrelevant for pattern ACK but keeps consistency.
	if(ack_pattern_time_ms > 0)
	{
		set_control_batch_size(1);
		set_ack_batch_size(1);
	}

	if(ack_pattern_time_ms > 0)
	{
		// ACK is a short tone pattern, not a full LDPC frame.
		// Bug #44: responder turnaround includes CMD frame TX time, so
		// ack_timeout must cover: TX + responder ftr (frame_TX + 4000ms) + ACK.
		set_ack_timeout_data((data_batch_size+2)*message_transmission_time_ms + ack_pattern_time_ms + 4*ptt_on_delay_ms + 4*ptt_off_delay_ms + 3000);
		set_ack_timeout_control((control_batch_size+1)*message_transmission_time_ms + ack_pattern_time_ms + 2*ptt_on_delay_ms + 2*ptt_off_delay_ms + 3000);
	}
	else
	{
		set_ack_timeout_data((data_batch_size+1)*message_transmission_time_ms+control_batch_size*message_transmission_time_ms+2*ack_batch_size*ctrl_transmission_time_ms+time_left_to_send_last_frame+4*ptt_on_delay_ms+4*ptt_off_delay_ms);
		set_ack_timeout_control(control_batch_size*message_transmission_time_ms+ack_batch_size*ctrl_transmission_time_ms+time_left_to_send_last_frame+2*ptt_on_delay_ms+2*ptt_off_delay_ms);
	}

	// During turboshift, extend ack_timeout_control (the overall NAck deadline).
	// receiving_timeout margin is handled inside calculate_receiving_timeout().
	if(gear_shift_on && turboshift_phase != TURBO_DONE)
	{
		set_ack_timeout_control(ack_timeout_control + 2000);
	}

	ptt_on_delay_ms=default_configuration_ARQ.ptt_on_delay_ms;
	ptt_off_delay_ms=default_configuration_ARQ.ptt_off_delay_ms;
	pilot_tone_ms=default_configuration_ARQ.pilot_tone_ms;
	pilot_tone_hz=default_configuration_ARQ.pilot_tone_hz;
	switch_role_timeout=default_configuration_ARQ.switch_role_timeout_ms;
	// MFSK modes: frame durations are 4-7s, so both sides have ample prep time
	// during the frame itself. Reduce role-switch wait from 1500ms to 200ms.
	if(is_robust_config(configuration))
		switch_role_timeout = 200;

	switch_role_test_timeout=(nResends/3)*ack_timeout_control;
	watchdog_timeout=(nResends/3)*ack_timeout_data;
	gearshift_timeout=(nResends/3)*ack_timeout_data;

	// Ensure connection_timeout and link_timeout are adequate for MFSK frame durations.
	{
		int ack_time = (ack_pattern_time_ms > 0) ? ack_pattern_time_ms : (ack_batch_size * ctrl_transmission_time_ms);

		// Connection handshake: 2 round-trips of control+ack batches
		int min_ct = 2 * (control_batch_size * message_transmission_time_ms + ack_time)
			+ 4 * ptt_on_delay_ms + 4 * ptt_off_delay_ms + 5000;
		if (connection_timeout < min_ct)
			connection_timeout = min_ct;

		// Link timeout: must survive a full data+ack round-trip
		int min_lt = (data_batch_size + 2) * message_transmission_time_ms + ack_time
			+ 2 * ptt_on_delay_ms + 2 * ptt_off_delay_ms + 5000;
		if (link_timeout < min_lt)
			link_timeout = min_lt;
	}

	calculate_receiving_timeout();

	// Reset OFDM batch prediction state on config change.
	// After turboshift, ofdm_batch_active/ofdm_search_raw hold stale positions
	// from control frames decoded at the old config.  The new config has different
	// Nsymb/frame geometry, so the old ofdm_skip prediction lands in the wrong
	// place → wrong delay → wrong Moose freq offset → LDPC fails on real HF.
	telecom_system->receive_stats.ofdm_batch_active = false;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_drift_per_frame = 0.0;

	// Guard: ack_timeout must cover the full TX + receive window.
	// ack_timer starts at frame send (T=0); receiving_timer starts after TX + PTT.
	// Without this, ack_timeout can expire while CMD is still polling for ACK.
	if(this->role == COMMANDER)
	{
		int min_ack = message_transmission_time_ms + ptt_off_delay_ms + receiving_timeout + 500;
		if(ack_timeout_control < min_ack)
			set_ack_timeout_control(min_ack);
		if(ack_timeout_data < min_ack)
			set_ack_timeout_data(min_ack);
	}

	if(level==FULL)
	{
		printf("[CFG] init_messages_buffers (nMessages=%d)\n", nMessages);
		fflush(stdout);
		this->init_messages_buffers();
		printf("[CFG] init_messages_buffers done\n");
		fflush(stdout);
	}

	// §21 (tier2-suffix-fec-design.md): PRODUCTION enhanced-CONNECT enable.
	// telecom_system->load_configuration() (just above) recomputed
	// ctrl_suffix_pattern_passband_samples at the UNCODED 13-tone / single-base
	// length, so the FEC + combining state must be RE-APPLIED on every config
	// switch (the set_* hooks re-derive that member at the current Nofdm).
	//
	// The enhanced ctrl-suffix (GF(16) RA FEC §19 + base-pattern combining §20) is
	// the UNCONDITIONAL DEFAULT at the robust tier — it is NOT capability-negotiated
	// (Mercury shipped no version → no legacy peers; the GF(16) RA codeword is
	// systematic, backward-compatible by construction). PRODUCTION TRIGGER = LOCAL
	// robust tier (§21.3): when the session is at ROBUST_0/1/2 the CONNECT suffix is
	// enhanced (FEC R¼ + combining R=4) — once/session, airtime-cheap (§4: +2% of a
	// 7.9 s ROBUST frame). The RX runs the try-both decode (uncoded-13-first, then
	// the GF(16) soft decode) gated on its OWN robust tier, §21.5. At OFDM configs
	// (CONFIG_6+) the enhanced state is turned OFF → the CONNECT suffix is
	// byte-identical (and CONNECT establishment normally happens at the robust
	// floor anyway). Re-applied on EVERY config switch so a turboshift up to OFDM
	// disables it and a fall back to robust re-enables it.
	//
	// TEST OVERRIDE: MERCURY_SUFFIX_FEC=1 / MERCURY_CONNECT_REPS=N force the state
	// regardless of tier (for pinned-config sim/HW A/B). The env knob, when set,
	// WINS over the tier trigger. Cached on first call (no getenv() in the hot
	// config-switch path on Pi).
	{
		static int  fec_env_cached   = 0;
		static int  fec_env_force     = -1;   // -1 = unset, 0/1 = forced value
		static int  reps_env_cached   = 0;
		static int  reps_env_force     = -1;   // -1 = unset, >=1 = forced reps
		if(!fec_env_cached)
		{
			const char* e = std::getenv("MERCURY_SUFFIX_FEC");
			if(e != nullptr) { fec_env_force = (e[0] == '1') ? 1 : 0;
				printf("[CFG] MERCURY_SUFFIX_FEC override = %d (test)\n", fec_env_force); fflush(stdout); }
			fec_env_cached = 1;
		}
		if(!reps_env_cached)
		{
			const char* e = std::getenv("MERCURY_CONNECT_REPS");
			if(e != nullptr) { reps_env_force = atoi(e); if(reps_env_force < 1) reps_env_force = 1;
				printf("[CFG] MERCURY_CONNECT_REPS override = %d (test)\n", reps_env_force); fflush(stdout); }
			reps_env_cached = 1;
		}
		// Production trigger: enhanced CONNECT at the robust tier.
		bool robust_tier = is_robust_config(configuration);
		bool fec_on  = (fec_env_force >= 0) ? (fec_env_force == 1) : robust_tier;
		int  reps    = (reps_env_force >= 0) ? reps_env_force
		                                     : (robust_tier ? CONNECT_PREAMBLE_REPS_PROD : 1);
		// Apply (idempotent; WB-only — NB has connect_pattern_nsymb=0 so the set_*
		// hooks no-op the passband member). DISABLE at OFDM so the CONNECT suffix
		// is byte-identical there. set_connect_preamble_reps must be called AFTER
		// set_suffix_fec (both re-derive ctrl_suffix_pattern_passband_samples from
		// the CURRENT coded-suffix length × base reps; reps last = correct member).
		telecom_system->set_suffix_fec(fec_on, 3);
		telecom_system->set_connect_preamble_reps(reps);
	}

	// BIG-BLOCK RUNG ELECTION ON THE GEARSHIFT TRANSITION (P4 — the "not wired into
	// the gearshift" gap, fact-doc data-flow-bigblock-arq-unit.md §16). The big-block
	// TX switch (bigblock_send_one_block, arq_common.cc:3648) and RX carve
	// (arq_common.cc:6888) already SELF-GATE on bigblock_framing_enabled && M!=MFSK &&
	// current_configuration==CONFIG_16, so they engage automatically once the live
	// config is CFG16 — no TX-switch wiring is needed here. The ONLY missing piece on
	// the climb path was the BATCH-SIZE ELECTION: sack_negotiated_recompute_batch()
	// pins data_batch_size=K (the R-B/#9 pin) so the clean-ACK all_ones target
	// (1<<data_batch_size)-1 equals the K-bit big-block cw_ok bitmap (0xFF at K=8).
	// That election fired ONLY at CONNECT negotiation; on a robust/`-R` connect that
	// ran at ROBUST_0 (rung guard false), and nothing re-elected when the gearshift
	// later climbed onto CFG16 -> data_batch_size stayed at the stock 30s value (~25)
	// -> all_ones=0x1FFFFFF != 0xFF -> ZERO clean credit -> every block PARTIAL (the
	// HW-observed bigblock_rung=0 / clean=0 failure).
	//
	// load_configuration() is the SINGLE chokepoint every config switch flows through,
	// on BOTH the CMD and RSP side (each peer runs its own gearshift -> its own
	// load_configuration). Re-invoking the SHARED election body here makes both peers
	// run IDENTICAL code from the SAME PHY geometry source (bigblock_codeword_count())
	// at their CFG16 transition, so they elect the SAME K and CANNOT diverge (the
	// R-B/#9 symmetry contract, §16.7). Gated STRICTLY on the bigblock rung; OFF the
	// rung the shared body's own outer guard early-returns to the stock 30s/robust
	// branch already run above, so the stock per-frame path is BYTE-IDENTICAL.
	//
	// TEST ESCAPE (fail-before/pass-after on the SAME binary, NOT a production knob):
	// MERCURY_BIGBLOCK_DEFEAT_ELECTION=1 skips this tail call so the transition leaves
	// data_batch_size at the stock value — the --test-bigblock-climb-election
	// fail-before arm. getenv() is read HERE (inside the rare CFG16+framing guard, NOT
	// every config switch) so the in-process A/B test can flip the flag between arms
	// without the static-cache trap; production (env unset) takes the fast NULL return.
	if(telecom_system != NULL
		&& telecom_system->bigblock_framing_enabled
		&& current_configuration == CONFIG_16)
	{
		bool defeat_election = false;
		{
			const char* e = std::getenv("MERCURY_BIGBLOCK_DEFEAT_ELECTION");
			if(e && *e && atoi(e) != 0) defeat_election = true;
		}
		if(!defeat_election)
		{
			printf("[BIGBLOCK-ELECT] gearshift CFG16 transition -> electing big-block "
				"rung (role=%s)\n", (role==COMMANDER) ? "CMD" : "RSP");
			fflush(stdout);
			sack_negotiated_recompute_batch((role==COMMANDER) ? "CMD" : "RSP");
		}
		else
		{
			printf("[BIGBLOCK-ELECT] DEFEAT_ELECTION=1 — skipping CFG16 rung election "
				"(test fail-before arm; data_batch_size stays %d)\n", data_batch_size);
			fflush(stdout);
		}
	}
}

void cl_arq_controller::return_to_last_configuration()
{
	if(last_data_configuration==this->current_configuration)
	{
		return;
	}
	int tmp;
	this->load_configuration(last_data_configuration,FULL,YES);
	tmp= last_data_configuration;
	last_data_configuration=current_configuration;
	current_configuration=tmp;
}

// Canary: 16 bytes of 0xCC appended to each buffer to detect overflow.
// check_canaries() validates them; any corruption pinpoints the overflow target.
#define CANARY_SIZE 16
#define CANARY_BYTE 0xCC

static void set_canary(char* buf, int data_size)
{
	memset(buf + data_size, CANARY_BYTE, CANARY_SIZE);
}

// Diagnostic globals for crash handler — set before each canary read
// so we know which buffer was being checked when the crash occurs.
volatile const char* g_canary_check_name = NULL;
volatile int g_canary_check_idx = -1;
volatile const char* g_canary_check_ptr = NULL;

static int check_canary(const char* buf, int data_size, const char* name, int idx)
{
	g_canary_check_name = name;
	g_canary_check_idx = idx;
	g_canary_check_ptr = buf;
	if(buf == NULL) return 0;
	for(int j=0; j<CANARY_SIZE; j++)
	{
		if((unsigned char)buf[data_size + j] != CANARY_BYTE)
		{
			printf("[CANARY] OVERFLOW %s[%d] at offset %d (byte=0x%02x, expected 0xCC)\n",
				name, idx, data_size + j, (unsigned char)buf[data_size + j]);
			fflush(stdout);
			return 1;
		}
	}
	return 0;
}

void cl_arq_controller::check_buffer_canaries(const char* caller)
{
	const int alloc_size = N_MAX / 8;
	int corrupted = 0;

	if(messages_tx != NULL)
	{
		for(int i=0; i<nMessages; i++)
			corrupted += check_canary(messages_tx[i].data, alloc_size, "messages_tx", i);
	}
	if(messages_rx != NULL)
	{
		for(int i=0; i<nMessages; i++)
			corrupted += check_canary(messages_rx[i].data, alloc_size, "messages_rx", i);
	}
	// SACK Design A Step 8a — parallel prev-batch buffer canaries.
	if(messages_rx_prev != NULL)
	{
		for(int i=0; i<nMessages; i++)
			corrupted += check_canary(messages_rx_prev[i].data, alloc_size, "messages_rx_prev", i);
	}
	if(messages_batch_ack != NULL)
	{
		for(int i=0; i<255; i++)
			corrupted += check_canary(messages_batch_ack[i].data, alloc_size, "messages_batch_ack", i);
	}
	corrupted += check_canary(messages_last_ack_bu.data, alloc_size, "messages_last_ack_bu", 0);
	corrupted += check_canary(messages_control.data, alloc_size, "messages_control", 0);
	corrupted += check_canary(messages_rx_buffer.data, alloc_size, "messages_rx_buffer", 0);
	corrupted += check_canary(message_TxRx_byte_buffer, alloc_size, "message_TxRx_byte_buffer", 0);

	if(corrupted > 0)
	{
		printf("[CANARY] %d canary violations detected! caller=%s\n", corrupted, caller);
		fflush(stdout);
	}
}

int cl_arq_controller::init_messages_buffers()
{
	int success=SUCCESSFUL;

	// Allocate all message buffers with N_MAX/8 (= 200 bytes) instead of
	// the current config's max_message_length. This prevents heap overflow
	// when PHYS_ONLY config transitions (e.g., turboshift or gearshift)
	// increase max_message_length without reallocating these buffers.
	// N_MAX = 1600 bits is the absolute maximum LDPC codeword size.
	const int alloc_size = N_MAX / 8;

	this->messages_tx=new st_message[nMessages];

	if(this->messages_tx==NULL)
	{
		success=MEMORY_ERROR;
	}
	else
	{
		for(int i=0;i<this->nMessages;i++)
		{
			this->messages_tx[i].ack_timeout=0;
			this->messages_tx[i].id=0;
			this->messages_tx[i].length=0;
			this->messages_tx[i].nResends=0;
			this->messages_tx[i].status=FREE;
			this->messages_tx[i].type=NONE;
			this->messages_tx[i].data=NULL;
			this->messages_tx[i].batch_seq_id=-1;  // Step 3: unset until TX path assigns

			this->messages_tx[i].data=new char[alloc_size + CANARY_SIZE];
			set_canary(this->messages_tx[i].data, alloc_size);

			if(this->messages_tx[i].data==NULL)
			{
				success=MEMORY_ERROR;
			}
		}
	}

	this->messages_rx=new st_message[nMessages];

	if(this->messages_rx==NULL)
	{
		success=MEMORY_ERROR;
	}
	else
	{
		for(int i=0;i<this->nMessages;i++)
		{
			this->messages_rx[i].ack_timeout=0;
			this->messages_rx[i].id=0;
			this->messages_rx[i].length=0;
			this->messages_rx[i].nResends=0;
			this->messages_rx[i].status=FREE;
			this->messages_rx[i].type=NONE;
			this->messages_rx[i].data=NULL;
			this->messages_rx[i].batch_seq_id=-1;  // Step 3: unset until RX parses (v2)

			this->messages_rx[i].data=new char[alloc_size + CANARY_SIZE];
			set_canary(this->messages_rx[i].data, alloc_size);

			if(this->messages_rx[i].data==NULL)
			{
				success=MEMORY_ERROR;
			}
		}
	}

	// SACK Design A Step 8a — allocate parallel prev-batch storage.
	// Sized identically to messages_rx (nMessages slots, each data buffer
	// alloc_size + CANARY_SIZE). Only ever populated when sack_v2_enabled.
	// On v1 sessions, the buffer sits idle — no read/write — so its presence
	// is byte-invisible to the v1 wire path (the v1↔v1 SHA-256 stability
	// gate proves this).
	this->messages_rx_prev=new st_message[nMessages];

	if(this->messages_rx_prev==NULL)
	{
		success=MEMORY_ERROR;
	}
	else
	{
		for(int i=0;i<this->nMessages;i++)
		{
			this->messages_rx_prev[i].ack_timeout=0;
			this->messages_rx_prev[i].id=0;
			this->messages_rx_prev[i].length=0;
			this->messages_rx_prev[i].nResends=0;
			this->messages_rx_prev[i].status=FREE;
			this->messages_rx_prev[i].type=NONE;
			this->messages_rx_prev[i].data=NULL;
			this->messages_rx_prev[i].batch_seq_id=-1;

			this->messages_rx_prev[i].data=new char[alloc_size + CANARY_SIZE];
			set_canary(this->messages_rx_prev[i].data, alloc_size);

			if(this->messages_rx_prev[i].data==NULL)
			{
				success=MEMORY_ERROR;
			}
		}
	}

	// Allocate 255 elements (absolute max nMessages) rather than current
	// data_batch_size, because PHYS_ONLY config transitions can increase
	// both nMessages and data_batch_size without reallocating (Bug #15).
	const int max_batch_alloc = 255;
	this->messages_batch_tx=new st_message[max_batch_alloc];

	if(this->messages_batch_tx==NULL)
	{
		success=MEMORY_ERROR;
	}
	else
	{
		for(int i=0;i<max_batch_alloc;i++)
		{
			this->messages_batch_tx[i].ack_timeout=0;
			this->messages_batch_tx[i].id=0;
			this->messages_batch_tx[i].length=0;
			this->messages_batch_tx[i].nResends=0;
			this->messages_batch_tx[i].status=FREE;
			this->messages_batch_tx[i].type=NONE;
			this->messages_batch_tx[i].data=NULL;
			this->messages_batch_tx[i].batch_seq_id=-1;  // Step 3: unset until build_batch assigns
		}
	}

	// Same fix for ack batch array (Bug #15).
	this->messages_batch_ack=new st_message[max_batch_alloc];

	if(this->messages_batch_ack==NULL)
	{
		success=MEMORY_ERROR;
	}
	else
	{
		for(int i=0;i<max_batch_alloc;i++)
		{
			this->messages_batch_ack[i].ack_timeout=0;
			this->messages_batch_ack[i].id=0;
			this->messages_batch_ack[i].length=0;
			this->messages_batch_ack[i].nResends=0;
			this->messages_batch_ack[i].status=FREE;
			this->messages_batch_ack[i].type=NONE;
			this->messages_batch_ack[i].data=NULL;
			this->messages_batch_ack[i].batch_seq_id=-1;  // Step 3: ACK frames never carry batch_seq_id

			this->messages_batch_ack[i].data=new char[alloc_size + CANARY_SIZE];
			set_canary(this->messages_batch_ack[i].data, alloc_size);

			if(this->messages_batch_ack[i].data==NULL)
			{
				success=MEMORY_ERROR;
			}
		}
	}

	this->messages_last_ack_bu.status=FREE;
	this->messages_last_ack_bu.data=NULL;
	this->messages_last_ack_bu.batch_seq_id=-1;  // Step 3
	this->messages_last_ack_bu.data=new char[alloc_size + CANARY_SIZE];
	set_canary(this->messages_last_ack_bu.data, alloc_size);

	if(this->messages_last_ack_bu.data==NULL)
	{
		success=MEMORY_ERROR;
	}

	this->messages_control.status=FREE;
	this->messages_control.data=NULL;
	this->messages_control.batch_seq_id=-1;  // Step 3: control frames never carry batch_seq_id
	this->messages_control.data=new char[alloc_size + CANARY_SIZE];
	set_canary(this->messages_control.data, alloc_size);

	if(this->messages_control.data==NULL)
	{
		success=MEMORY_ERROR;
	}

	this->messages_rx_buffer.status=FREE;
	this->messages_rx_buffer.data=NULL;
	this->messages_rx_buffer.batch_seq_id=-1;  // Step 3: populated on v2 DATA RX
	this->messages_rx_buffer.data=new char[alloc_size + CANARY_SIZE];
	set_canary(this->messages_rx_buffer.data, alloc_size);

	if(this->messages_rx_buffer.data==NULL)
	{
		success=MEMORY_ERROR;
	}

	this->messages_control.status=FREE;
	this->message_TxRx_byte_buffer=new char[alloc_size + CANARY_SIZE];
	set_canary(this->message_TxRx_byte_buffer, alloc_size);

	if(this->message_TxRx_byte_buffer==NULL)
	{
		success=MEMORY_ERROR;
	}
	return success;
}
int cl_arq_controller::deinit_messages_buffers()
{
	int success=SUCCESSFUL;

	// Check all canaries before freeing — any corruption reveals the overflow target
	check_buffer_canaries("deinit_messages_buffers");

	if(messages_tx!=NULL)
	{
		for(int i=0;i<nMessages;i++)
		{
			if(messages_tx[i].data!=NULL)
			{
				delete[] messages_tx[i].data;
				messages_tx[i].data=NULL;
			}
		}
		delete[] messages_tx;
		messages_tx=NULL;
	}

	if(messages_rx!=NULL)
	{
		for(int i=0;i<nMessages;i++)
		{
			if(messages_rx[i].data!=NULL)
			{
				delete[] messages_rx[i].data;
				messages_rx[i].data=NULL;
			}
		}
		delete[] messages_rx;
		messages_rx=NULL;
	}

	// SACK Design A Step 8a — free parallel prev-batch storage.
	if(messages_rx_prev!=NULL)
	{
		for(int i=0;i<nMessages;i++)
		{
			if(messages_rx_prev[i].data!=NULL)
			{
				delete[] messages_rx_prev[i].data;
				messages_rx_prev[i].data=NULL;
			}
		}
		delete[] messages_rx_prev;
		messages_rx_prev=NULL;
	}
	// Reset prev-batch state (parallels what the constructor does so a
	// post-deinit re-init starts with no false prev_active claim).
	rsp_prev_batch_active=false;
	rsp_prev_batch_received_count=0;
	rsp_prev_batch_expected_count=0;

	if(messages_batch_ack!=NULL)
	{
		for(int i=0;i<255;i++)
		{
			if(messages_batch_ack[i].data!=NULL)
			{
				delete[] messages_batch_ack[i].data;
				messages_batch_ack[i].data=NULL;
			}
		}
		delete[] messages_batch_ack;
		messages_batch_ack=NULL;
	}

	if(messages_last_ack_bu.data!=NULL)
	{
		delete[] messages_last_ack_bu.data;
		messages_last_ack_bu.data=NULL;
	}
	if(messages_control.data!=NULL)
	{
		delete[] messages_control.data;
		messages_control.data=NULL;
	}
	if(messages_rx_buffer.data!=NULL)
	{
		delete[] messages_rx_buffer.data;
		messages_rx_buffer.data=NULL;
	}
	if(messages_batch_tx!=NULL)
	{
		delete[] messages_batch_tx;
		messages_batch_tx=NULL;
	}
	if(message_TxRx_byte_buffer!=NULL)
	{
		delete[] message_TxRx_byte_buffer;
		message_TxRx_byte_buffer=NULL;
	}

	return success;
}

void cl_arq_controller::update_status()
{
	for(int i=0;i<nMessages;i++)
	{
		if(messages_tx[i].status==PENDING_ACK && messages_tx[i].ack_timer.get_elapsed_time_ms()>=messages_tx[i].ack_timeout)
		{
			messages_tx[i].status=ACK_TIMED_OUT;
			stats.nNAcked_data++;
		}
	}

	if(messages_control.status==PENDING_ACK && messages_control.ack_timer.get_elapsed_time_ms()>=messages_control.ack_timeout)
	{
		messages_control.status=ACK_TIMED_OUT;
		stats.nNAcked_control++;
	}

	// Check connection attempt timeout - separate from link_timer which gets restarted on every message
	if((link_status==CONNECTING || link_status==NEGOTIATING || link_status==CONNECTION_ACCEPTED) &&
	   connection_attempt_timer.counting==1 &&
	   connection_attempt_timer.get_elapsed_time_ms()>=connection_timeout)
	{
		std::cout<<"Connection attempt timeout after "<<connection_timeout<<" ms"<<std::endl;

		// NB/WB auto-negotiation: if commander is still in NB probe mode at
		// timeout, switch to WB and retry. Covers two cases:
		//   1. connection_attempts < nb_probe_max (original Phase 1→2)
		//   2. connection_attempts >= nb_probe_max but still NB (HAIL retry
		//      loop consumed all probe attempts without triggering switch-back)
		if(role == COMMANDER && link_status == CONNECTING &&
		   commander_configured_nb >= 0 &&
		   narrowband_enabled == YES && commander_configured_nb != YES)
		{
			connection_attempts = nb_probe_max;  // Skip remaining probes
			printf("[NB-NEG] Commander: timeout — restoring WB (attempts=%d)\n",
				connection_attempts);
			fflush(stdout);
			switch_narrowband_mode(NO);  // Switch to WB
			hail_detected = NO;  // Reset so WB HAIL is attempted
			// Reset timer and control message for fresh WB attempt
			connection_attempt_timer.reset();
			connection_attempt_timer.start();
			messages_control.status = FREE;
			// Stay in CONNECTING — process_messages_commander will send new HAIL/START_CONNECTION
			return;
		}

		// Send CANCELPENDING and DISCONNECTED to Winlink (connection attempt timed out)
		if(role==COMMANDER && tcp_socket_control.get_status()==TCP_STATUS_ACCEPTED)
		{
			std::string str="CANCELPENDING\r";
			tcp_socket_control.message->length=str.length();
			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();

			str="DISCONNECTED\r";
			tcp_socket_control.message->length=str.length();
			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();
		}

		this->link_status=DROPPED;
		reset_session_state();
		reset_all_timers();
		connection_attempt_timer.stop();
		connection_attempt_timer.reset();

		fifo_buffer_tx.flush();
		fifo_buffer_backup.flush();
		fifo_buffer_rx.flush();

		// Reset messages_control so new CONNECT commands can work
		messages_control.status=FREE;

		// After failed connection attempt, always switch to RESPONDER/LISTENING
		// so we can receive incoming connections from the other side
		set_role(RESPONDER);
		link_status=LISTENING;
		connection_status=RECEIVING;
		load_configuration(init_configuration, FULL, YES);
		printf("Switching to RESPONDER mode after connection timeout\n");
	}

	// Check for max connection attempts
	if((link_status==CONNECTING || link_status==NEGOTIATING || link_status==CONNECTION_ACCEPTED) &&
	   connection_attempts >= max_connection_attempts)
	{
		std::cout<<"Maximum connection attempts ("<<max_connection_attempts<<") reached"<<std::endl;

		// Send CANCELPENDING and DISCONNECTED to Winlink (max connection attempts reached)
		if(role==COMMANDER && tcp_socket_control.get_status()==TCP_STATUS_ACCEPTED)
		{
			std::string str="CANCELPENDING\r";
			tcp_socket_control.message->length=str.length();
			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();

			str="DISCONNECTED\r";
			tcp_socket_control.message->length=str.length();
			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();
		}

		this->link_status=DROPPED;
		reset_session_state();
		reset_all_timers();
		connection_attempt_timer.stop();
		connection_attempt_timer.reset();

		fifo_buffer_tx.flush();
		fifo_buffer_backup.flush();
		fifo_buffer_rx.flush();

		// Reset messages_control so new CONNECT commands can work
		messages_control.status=FREE;

		// After failed connection attempts, always switch to RESPONDER/LISTENING
		// so we can receive incoming connections from the other side
		set_role(RESPONDER);
		link_status=LISTENING;
		connection_status=RECEIVING;
		load_configuration(init_configuration, FULL, YES);
		printf("Switching to RESPONDER mode after max connection attempts\n");
	}

	if(link_timer.get_elapsed_time_ms()>=link_timeout)
	{
		this->link_status=DROPPED;
		reset_session_state();
		reset_all_timers();

		fifo_buffer_tx.flush();
		fifo_buffer_backup.flush();
		fifo_buffer_rx.flush();

		// Notify Winlink of disconnect
		if(tcp_socket_control.get_status()==TCP_STATUS_ACCEPTED)
		{
			std::string str="DISCONNECTED\r";
			tcp_socket_control.message->length=str.length();
			for(int i=0;i<(int)tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();
		}

		if(this->role==COMMANDER)
		{
			// Stay as commander, retry connection at init config.
			// The responder side already dropped to LISTENING.
			// connection_attempt_timeout handler will give up after max retries.
			printf("[LINK-TIMEOUT] Commander retrying connection at init config\n");
			fflush(stdout);
			// Re-save NB preference (reset_session_state cleared it)
			commander_configured_nb = narrowband_enabled;
			load_configuration(init_configuration, FULL, YES);
			link_status=CONNECTING;
			connection_status=TRANSMITTING_CONTROL;

			// Reset turboshift for fresh probe on reconnect
			turboshift_active = true;
			turboshift_phase = TURBO_DONE;
			turboshift_last_good = -1;
			turbo_settle_pending = false;
			turbo_supershift_announce_pending = false;
			supershift_proven_ceiling = -1;
			turbo_snr_ack_enabled = false;
			turbo_received_snr = -99.0f;

			messages_control.status = FREE;
			connection_attempts = 0;
			connection_attempt_timer.reset();
			connection_attempt_timer.start();
			// Don't add_message_control here — CONNECTING flow in
			// process_messages_commander handles NB probe + START_CONNECTION
		}
		else if(this->role==RESPONDER)
		{
			link_status=LISTENING;
			connection_status=RECEIVING;
			load_configuration(init_configuration, FULL, YES);
		}
	}

	if(watchdog_timer.get_elapsed_time_ms()>= watchdog_timeout)
	{
		if(original_role==COMMANDER)
		{
			set_role(COMMANDER);
			link_status=CONNECTED;
			connection_status=TRANSMITTING_DATA;
			for(int i=0;i<nMessages;i++)
			{
				messages_tx[i].status=FREE;
			}
			clear_retx_queue();  // R029: watchdog recovery re-queues plaintext; drop stale retx

			char restore_buf[N_MAX/8 * 20];
			int total_restore = 0;
			int data_read_size;
			for(int i=0;i<get_nTotal_messages();i++)
			{
				data_read_size=fifo_buffer_backup.pop(restore_buf + total_restore,max_data_length+max_header_length);
				if(data_read_size!=0)
				{
					total_restore += data_read_size;
				}
				else
				{
					break;
				}
			}
			if(total_restore > 0)
				fifo_buffer_tx.push_front(restore_buf, total_restore);
			fifo_buffer_backup.flush();

		}
		else if(original_role==RESPONDER)
		{
			set_role(RESPONDER);
			link_status=CONNECTED;
			connection_status=RECEIVING;

			for(int i=0;i<nMessages;i++)
			{
				messages_rx[i].status=FREE;
			}
		}

		last_data_configuration=data_configuration;
		load_configuration(data_configuration,PHYSICAL_LAYER_ONLY,YES);

		gear_shift_blocked_for_nBlocks=0;  // Force cooldown: wait N blocks before shifting up again

		watchdog_timer.stop();
		watchdog_timer.reset();
		watchdog_timer.start();
		gear_shift_timer.stop();
		gear_shift_timer.reset();
		receiving_timer.stop();
		receiving_timer.reset();

	}

	// Fallback: if we're in COMMANDER mode and haven't received anything for 60+ seconds
	// while supposedly connected, force switch to RESPONDER mode to break infinite loops
	const int FORCED_ROLE_SWITCH_TIMEOUT = 180000;  // 180 seconds (6 BREAK recovery cycles at ~25s each)
	if(role==COMMANDER && link_status==CONNECTED &&
	   receiving_timer.get_elapsed_time_ms() >= FORCED_ROLE_SWITCH_TIMEOUT)
	{
		printf("Forced role switch: no RX for %d seconds, switching to RESPONDER\n", FORCED_ROLE_SWITCH_TIMEOUT/1000);

		reset_session_state();
		set_role(RESPONDER);
		link_status=LISTENING;
		connection_status=RECEIVING;
		load_configuration(init_configuration, FULL, YES);
		reset_all_timers();

		// Flush buffers
		fifo_buffer_tx.flush();
		fifo_buffer_backup.flush();
		fifo_buffer_rx.flush();
		messages_control.status=FREE;
	}

	if(gear_shift_on==YES && gear_shift_timer.get_elapsed_time_ms()>=gearshift_timeout)
	{
		gear_shift_timer.stop();
		gear_shift_timer.reset();

		if(gear_shift_algorithm==SNR_BASED)
		{
			messages_control_backup();
			load_configuration(init_configuration,PHYSICAL_LAYER_ONLY,YES);
			messages_control_restore();

			if(this->role==COMMANDER)
			{
				for(int i=0;i<nMessages;i++)
				{
					messages_tx[i].status=FREE;
				}
				clear_retx_queue();  // R029: gearshift-down (SNR_BASED) recovery re-queues plaintext

				char restore_buf[N_MAX/8 * 20];
				int total_restore = 0;
				int data_read_size;
				for(int i=0;i<get_nTotal_messages();i++)
				{
					data_read_size=fifo_buffer_backup.pop(restore_buf + total_restore,max_data_length+max_header_length);
					if(data_read_size!=0)
					{
						total_restore += data_read_size;
					}
					else
					{
						break;
					}
				}
				if(total_restore > 0)
					fifo_buffer_tx.push_front(restore_buf, total_restore);
				fifo_buffer_backup.flush();

				if (current_configuration!= last_data_configuration)
				{
					add_message_control(TEST_CONNECTION);
					gear_shift_timer.start();
					connection_status=TRANSMITTING_CONTROL;
				}
				else
				{
					connection_status=TRANSMITTING_DATA;
				}
			}
			else if(this->role==RESPONDER)
			{
				for(int i=0;i<nMessages;i++)
				{
					messages_rx[i].status=FREE;
				}

				connection_status=RECEIVING;
			}

		}
		else if(gear_shift_algorithm==SUCCESS_BASED_LADDER)
		{
			// During turboshift, retry/ceiling is handled in the control NAck
			// handler (arq_commander.cc). Skip the normal gearshift-down here.
			// During BREAK recovery, the SET_CONFIG for recovery is in-flight.
			// gearshift_timeout must not overwrite data_configuration or connection_status.
			if((turboshift_active || break_recovery_phase != 0 || emergency_break_active
				|| turboshift_phase != TURBO_DONE) && this->role==COMMANDER)
			{
				// SWITCH_ROLE timeout during turboshift: the other side likely
				// received SWITCH_ROLE and already became commander. Assume it
				// was received and become responder so we hear their frames.
				if(!turboshift_active && break_recovery_phase == 0
					&& !emergency_break_active && turboshift_phase != TURBO_DONE)
				{
					printf("[TURBO] SWITCH_ROLE timeout — assuming received, becoming responder\n");
					fflush(stdout);
					set_role(RESPONDER);
					link_status = CONNECTED;
					connection_status = RECEIVING;
					watchdog_timer.start();
					link_timer.start();
					telecom_system->data_container.frames_to_read =
						telecom_system->data_container.preamble_nSymb
						+ telecom_system->data_container.Nsymb;
					telecom_system->data_container.nUnder_processing_events = 0;
					telecom_system->receive_stats.mfsk_search_raw = 0;
					telecom_system->receive_stats.ofdm_search_raw = 0;
					telecom_system->receive_stats.ofdm_batch_active = false;
				}
				else
				{
					printf("[GEARSHIFT] Timeout during turboshift/break-recovery — skipping\n");
					fflush(stdout);
				}
				return;
			}

			messages_control_backup();
			data_configuration=config_ladder_down(current_configuration, robust_enabled);
			load_configuration(data_configuration,PHYSICAL_LAYER_ONLY,YES);
			messages_control_restore();

			if(this->role==COMMANDER)
			{
				gear_shift_blocked_for_nBlocks=0;

				for(int i=0;i<nMessages;i++)
				{
					messages_tx[i].status=FREE;
				}
				clear_retx_queue();  // R029: gearshift-down (SUCCESS_BASED_LADDER) recovery re-queues plaintext

				char restore_buf[N_MAX/8 * 20];
				int total_restore = 0;
				int data_read_size;
				for(int i=0;i<get_nTotal_messages();i++)
				{
					data_read_size=fifo_buffer_backup.pop(restore_buf + total_restore,max_data_length+max_header_length);
					if(data_read_size!=0)
					{
						total_restore += data_read_size;
					}
					else
					{
						break;
					}
				}
				if(total_restore > 0)
					fifo_buffer_tx.push_front(restore_buf, total_restore);
				fifo_buffer_backup.flush();

				connection_status=TRANSMITTING_DATA;
			}
			else if(this->role==RESPONDER)
			{
				for(int i=0;i<nMessages;i++)
				{
					messages_rx[i].status=FREE;
				}

				connection_status=RECEIVING;
			}
		}
	}

	if(!turboshift_active && switch_role_test_timer.get_elapsed_time_ms()>switch_role_test_timeout)
	{
		switch_role_test_timer.stop();
		switch_role_test_timer.reset();

		set_role(RESPONDER);
		this->link_status=CONNECTED;
		this->connection_status=RECEIVING;

		this->messages_control.ack_timeout=0;
		this->messages_control.id=0;
		this->messages_control.length=0;
		this->messages_control.nResends=0;
		this->messages_control.status=FREE;
		this->messages_control.type=NONE;

	}


	if(print_stats_timer.get_elapsed_time_ms()>(int)(1000.0/print_stats_frequency_hz))
	{
		print_stats_timer.start();
		print_stats();
	}

}

void cl_arq_controller::cleanup()
{

	if(messages_control.status==ACKED)
	{
		// SEND ACK TO USER
		this->messages_control.ack_timeout=0;
		this->messages_control.id=0;
		this->messages_control.length=0;
		this->messages_control.nResends=0;
		this->messages_control.status=FREE;
		this->messages_control.type=NONE;
	}
	else if(messages_control.status==FAILED_)
	{
		// SEND FAILED TO USER
		this->messages_control.ack_timeout=0;
		this->messages_control.id=0;
		this->messages_control.length=0;
		this->messages_control.nResends=0;
		this->messages_control.status=FREE;
		this->messages_control.type=NONE;
	}


	for(int i=0;i<this->nMessages;i++)
	{
		if(messages_tx[i].status==ACKED)
		{
			// SEND ACK TO USER
			this->messages_tx[i].ack_timeout=0;
			this->messages_tx[i].id=0;
			this->messages_tx[i].length=0;
			this->messages_tx[i].nResends=0;
			this->messages_tx[i].status=FREE;
			this->messages_tx[i].type=NONE;
		}
		else if(messages_tx[i].status==FAILED_)
		{
			// SEND FAILED TO USER
			this->messages_tx[i].ack_timeout=0;
			this->messages_tx[i].id=0;
			this->messages_tx[i].length=0;
			this->messages_tx[i].nResends=0;
			this->messages_tx[i].status=FREE;
			this->messages_tx[i].type=NONE;
		}
	}


}


void cl_arq_controller::pad_messages_batch_tx(int size)
{
	// Start from last unique message so it gets a duplicate first,
	// then wrap to 0. With 12 unique + 11 dups: IDs 0-9,11 get 2 copies,
	// only ID 10 has 1 copy (less critical than ID 0 header or last frame).
	int counter = (message_batch_counter_tx > 0) ? message_batch_counter_tx - 1 : 0;
	if(message_batch_counter_tx!=0 && message_batch_counter_tx<size)
	{
		for(int i=0;i<size-message_batch_counter_tx;i++)
		{
			int slot = i + message_batch_counter_tx;
			messages_batch_tx[slot]=messages_batch_tx[counter];
			// Assign unique ID so RSP stores each frame in a distinct slot
			// (duplicate data is fine, but colliding IDs break SACK bitmap).
			messages_batch_tx[slot].id = slot;
			counter++;
			if(counter>=message_batch_counter_tx)
			{
				counter=0;
			}
		}
		message_batch_counter_tx=size;
	}
}

void cl_arq_controller::process_main()
{
	std::string command="";

	// §10.5: the 2-instance SIM_INPROC stepper binds NO socket and injects user
	// commands + data directly (process_user_command + fifo_buffer_*). Skip the
	// TCP control + data poll blocks in that mode. Default false (gate cleared) →
	// production + paced sim run the verbatim blocks → byte-identical.
	if (!arq_sim_inproc_skip_tcp())
	{
	if (tcp_socket_control.get_status()==TCP_STATUS_ACCEPTED)
	{
		// Mark that we had a control connection
		had_control_connection=YES;

		if(tcp_socket_control.timer.counting==0)
		{
			tcp_socket_control.timer.start();
		}
		int nBytes_received=tcp_socket_control.receive();
		if(nBytes_received>0)
		{
			tcp_socket_control.timer.start();

			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				user_command_buffer+=tcp_socket_control.message->buffer[i];
			}
		}
		else if(nBytes_received==0 || (tcp_socket_control.timer.get_elapsed_time_ms()>=tcp_socket_control.timeout_ms && tcp_socket_control.timeout_ms!=INFINITE_))
		{
			// Check if client disconnected cleanly (nBytes_received==0)
			if(nBytes_received==0 && exit_on_disconnect==YES && had_control_connection==YES)
			{
				std::cout<<std::endl;
				std::cout<<"Control connection closed by client - exiting as requested"<<std::endl;
				exit(0);
			}

			fifo_buffer_tx.flush();
			fifo_buffer_backup.flush();
			fifo_buffer_rx.flush();

			tcp_socket_control.check_incomming_connection();
			if (tcp_socket_control.get_status()==TCP_STATUS_ACCEPTED)
			{
				tcp_socket_control.timer.start();
			}

		}
		size_t pos=std::string::npos;
		do
		{
			// Strip any leading \n characters (from Windows \r\n line endings)
			while(!user_command_buffer.empty() && user_command_buffer[0]=='\n')
			{
				user_command_buffer=user_command_buffer.substr(1);
			}

			size_t pos=user_command_buffer.find('\r');
			if(pos!=std::string::npos)
			{
				command=user_command_buffer.substr(0, pos);
				process_user_command(command);
				user_command_buffer=user_command_buffer.substr(pos+1,std::string::npos);
			}
		}while(pos!=std::string::npos);

	}
	else
	{
		tcp_socket_control.check_incomming_connection();
		if (tcp_socket_control.get_status()==TCP_STATUS_ACCEPTED)
		{
			tcp_socket_control.timer.start();
		}
	}


	if (tcp_socket_data.get_status()==TCP_STATUS_ACCEPTED)
	{
		if(tcp_socket_data.timer.counting==0)
		{
			tcp_socket_data.timer.start();
		}
		// Only receive from TCP if FIFO has room for the max recv size.
		// Otherwise data pulled from the socket would be silently dropped
		// by push() (returns 0 when full). Leaving data in the TCP socket
		// buffer creates natural backpressure to the sender.
		if(fifo_buffer_tx.get_free_size() >= MAX_BUFFER_SIZE)
		{
			int nBytes_received=tcp_socket_data.receive();
			if(nBytes_received>0)
			{
				tcp_socket_data.timer.start();

				// B2F filter: parse outgoing stream, unroll LZHUF payloads
				if(b2f_handler.is_initialized())
				{
					char b2f_buf[MAX_BUFFER_SIZE * 4]; // plaintext can be larger than LZHUF
					int b2f_len = b2f_handler.filter_tx(
						tcp_socket_data.message->buffer,
						tcp_socket_data.message->length,
						b2f_buf, sizeof(b2f_buf));

					// Auto-arm compression when B2F SID detected (Winlink traffic).
					// CAP_COMPRESSION removed — always unconditional.
					if(!compression_enabled && !force_compress &&
					   !b2f_compression_pending && b2f_handler.is_b2f_session())
					{
						b2f_compression_pending = true;
						printf("[COMPRESS] B2F detected — will arm on next ACK\n");
						fflush(stdout);
					}

					if(b2f_len > 0)
						fifo_buffer_tx.push(b2f_buf, b2f_len);
					else if(!b2f_handler.is_b2f_session())
						fifo_buffer_tx.push(tcp_socket_data.message->buffer, tcp_socket_data.message->length);
					// else: B2F active, parser accumulating partial line -- skip raw push
				}
				else
				{
					fifo_buffer_tx.push(tcp_socket_data.message->buffer, tcp_socket_data.message->length);
				}

				// Suppress unsolicited BUFFER advertisements while the link
				// is not yet CONNECTED (or actively tearing down). During
				// CONNECTING / NEGOTIATING / LISTENING / DROPPED the upstream
				// is still mid-handshake (e.g. waiting on `CONNECTED\r`); if
				// the client pre-pushes data (test harnesses do this), the
				// resulting flood of BUFFER lines on the control socket can
				// shadow or precede the CONNECT reply that the client is
				// blocked on. The data is still accepted into fifo_buffer_tx
				// — it will flow as soon as the link comes up. Once the
				// session is CONNECTED or DISCONNECTING, flow-control
				// advertisements resume normally.
				if(link_status==CONNECTED || link_status==DISCONNECTING)
				{
					std::string str="BUFFER ";
					str+=std::to_string(fifo_buffer_tx.get_size()-fifo_buffer_tx.get_free_size());
					str+='\r';
					for(long unsigned int i=0;i<str.length();i++)
					{
						tcp_socket_control.message->buffer[i]=str[i];
					}
					tcp_socket_control.message->length=str.length();
					tcp_socket_control.transmit();
				}
			}
			else if(nBytes_received==0)
			{
				// TCP connection closed (FIN received) — flush and re-accept
				fifo_buffer_tx.flush();
				fifo_buffer_backup.flush();
				fifo_buffer_rx.flush();

				tcp_socket_data.check_incomming_connection();

				if (tcp_socket_data.get_status()==TCP_STATUS_ACCEPTED)
				{
					tcp_socket_data.timer.start();
				}
			}
			else if(role == COMMANDER &&
				tcp_socket_data.timer.get_elapsed_time_ms()>=tcp_socket_data.timeout_ms &&
				tcp_socket_data.timeout_ms!=INFINITE_)
			{
				// Commander only: timeout waiting for data from TCP client.
				// Responder skips this — its data client is read-only (never sends),
				// so recv() always returns EWOULDBLOCK. The old code treated this as
				// a disconnect, flushing buffers and dropping the connection every
				// 1 second. (Bug #61: responder data delivery)
				fifo_buffer_tx.flush();
				fifo_buffer_backup.flush();
				fifo_buffer_rx.flush();

				tcp_socket_data.check_incomming_connection();

				if (tcp_socket_data.get_status()==TCP_STATUS_ACCEPTED)
				{
					tcp_socket_data.timer.start();
				}
			}
		}

	}
	else
	{
		tcp_socket_data.check_incomming_connection();
		if (tcp_socket_data.get_status()==TCP_STATUS_ACCEPTED)
		{
			tcp_socket_data.timer.start();
		}
	}
	}  // §10.5: end of the (skippable) TCP control + data poll blocks

	// Signal measurement when idle: measure_signal_only() uses FIR_rx_time_sync,
	// the same filter that receive_byte() uses for preamble detection. Running both
	// on the same iteration corrupts the FIR delay line state, making Schmidl-Cox
	// GI correlation fail (Bug #28). Only run when receive() is NOT called.
	// LISTENING has active receive() calls → signal strength comes from receive_byte().
	if(link_status == IDLE || link_status == DROPPED)
	{
		MUTEX_LOCK(&capture_prep_mutex);
		if(telecom_system->data_container.frames_to_read == 0)
		{
			int signal_period = telecom_system->data_container.Nofdm *
				telecom_system->data_container.buffer_Nsymb *
				telecom_system->data_container.interpolation_rate;

			int rwi = telecom_system->data_container.ring_write_index;
			memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
				&telecom_system->data_container.passband_delayed_data[rwi],
				signal_period * sizeof(double));

			MUTEX_UNLOCK(&capture_prep_mutex);

#ifdef IDLE_GATE_TRACE
			// Step-0 instrumentation (IDLE_SCAN_CADENCE_RESEARCH.md §6 Step 0):
			// measure the raw-passband RMS of the buffer the Step-3 gate would
			// probe, and count how often this IDLE-loop FIR path runs. File-
			// static counters only (no class members — ODR hazard, see Plan B
			// §10). Whole #ifdef block is compiled out of the default build, so
			// the binary without -DIDLE_GATE_TRACE is byte-identical to baseline.
			{
				const double* idle_buf =
					telecom_system->data_container.ready_to_process_passband_delayed_data;
				double idle_sumsq = 0.0;
				for(int i = 0; i < signal_period; i++)
					idle_sumsq += idle_buf[i] * idle_buf[i];
				double idle_rms = std::sqrt(idle_sumsq / (double)signal_period);

				static long long idle_gate_fir_runs = 0;   // FIR (measure_signal_only) invocations
				static double    idle_gate_rms_sum  = 0.0;  // running RMS sum for mean
				static double    idle_gate_rms_min  = 1e30;
				static double    idle_gate_rms_max  = 0.0;
				static long long idle_gate_rms_n    = 0;    // RMS samples (== FIR runs here)
				static long long idle_gate_last_ms  = -1;

				idle_gate_fir_runs++;
				idle_gate_rms_n++;
				idle_gate_rms_sum += idle_rms;
				if(idle_rms < idle_gate_rms_min) idle_gate_rms_min = idle_rms;
				if(idle_rms > idle_gate_rms_max) idle_gate_rms_max = idle_rms;

				long long now_ms = mtl::now_ms();
				if(idle_gate_last_ms < 0) idle_gate_last_ms = now_ms;
				if(now_ms - idle_gate_last_ms >= 5000)
				{
					mtl::log_event_kv("idle_gate_trace",
						"fir_runs=%lld n=%lld rms_mean=%.6f rms_min=%.6f rms_max=%.6f rms_last=%.6f",
						idle_gate_fir_runs, idle_gate_rms_n,
						idle_gate_rms_n ? idle_gate_rms_sum / (double)idle_gate_rms_n : 0.0,
						idle_gate_rms_min, idle_gate_rms_max, idle_rms);
					idle_gate_last_ms = now_ms;
				}
			}
#endif

			measurements.signal_stregth_dbm = telecom_system->measure_signal_only(
				telecom_system->data_container.ready_to_process_passband_delayed_data);
		}
		else
		{
			MUTEX_UNLOCK(&capture_prep_mutex);
		}
	}

	process_messages();
	// ARQ main-loop pacing floor. In production this 2 ms sleep caps the poll
	// rate at ~500 Hz (plenty for a real radio's frame cadence). Under -x sim
	// it would throttle the whole control loop to wall-clock 500 Hz and erase
	// the faster-than-real-time speed-up, so use the short SIM spin sleep: it
	// hands the core to the RX-prep / TX-bridge threads (which drive virtual
	// time) and keeps the two peer processes loosely paced together (a raw
	// yield here hot-spun the loop and helped desync the CMD<->RSP turnaround).
	// Gated on the flag -> production unchanged.
	//
	// §5.7 pacing-floor: route the sim branch through sim_spin_or_pump so the
	// SIM_INPROC inline stepper ADVANCES the shared virtual clock here too
	// (a bare sim_spin_sleep() is a 200us WALL sleep that would freeze a peer
	// instance's clock view). When no pump is installed (the two-process paced
	// sim) sim_spin_or_pump(true) IS sim_spin_sleep() — byte-identical. The
	// production (sim disabled) branch is the verbatim usleep(2000).
	//
	// STEPPER-CORE REWRITE Phase b: under the OUTER-loop stepper, in the OFDM DATA PHASE the
	// outer loop is the SOLE clock-drain + RX-feed driver, so this end-of-process_main pump
	// call must NOT fire (one pump = one out-of-order depth-0 deliver+decode-drive that would
	// BURST-drain the big-block still queued in play AFTER send_batch returned — the exact
	// polls=0 / nAcked_data=0 / re-send-bsi=0 failure). Gated on the OFDM-PHASE predicate (NOT
	// the narrower in-data-batch-tx scope, which the send_batch RAII already cleared by here),
	// so the whole OFDM data phase routes clock+delivery through the outer loop. The MFSK ACK
	// patterns deliver via their OWN drain (legacy pump). On the ROBUST/MFSK HANDSHAKE the gate
	// is FALSE -> the legacy pump runs. Paced sim + production are byte-identical.
	if (sim_outer_stepper_ofdm_phase())
		;  // OFDM data phase: outer loop owns clock + delivery; no in-process_main pump
	else if (sim_clock_enabled())
		sim_spin_or_pump(true);
	else
		usleep(2000);
}

void cl_arq_controller::process_user_command(std::string command)
{

	if(command.substr(0,7)=="MYCALL ")
	{
		this->my_call_sign=command.substr(7);

		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command.substr(0,8)=="CONNECT ")
	{
		command=command.substr(8,std::string::npos);
		this->my_call_sign=command.substr(0,command.find(" "));
		this->destination_call_sign=command.substr(my_call_sign.length()+1);
		commander_configured_nb=narrowband_enabled;
		local_capability = ((bandwidth_mode == BW_AUTO) ? CAP_WB_CAPABLE : 0) | ((encryption_mode != ENCRYPT_OFF) ? CAP_ENCRYPTION : 0);
		peer_capability = 0;
		wb_upgrade_pending = false;
		compression_enabled = false;
		original_role=COMMANDER;
		set_role(COMMANDER);
		link_status=CONNECTING;
		reset_all_timers();

		// Reset messages_control so new connection can add START_CONNECTION
		messages_control.status=FREE;

		// Start connection attempt timer and reset counter
		connection_attempts=0;
		connection_attempt_timer.reset();
		connection_attempt_timer.start();

		// Send OK acknowledgement
		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
		tcp_socket_control.transmit();

		// Send PENDING status to indicate connection attempt is starting
		std::string str="PENDING\r";
		tcp_socket_control.message->length=str.length();
		for(int i=0;i<tcp_socket_control.message->length;i++)
		{
			tcp_socket_control.message->buffer[i]=str[i];
		}
		// Note: transmit() will be called in process_main after all commands are processed
	}
	else if(command=="DISCONNECT")
	{
		disconnect_requested=YES;

		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command=="ABORT")
	{
		// Abort connection attempt or active session - immediate teardown
		if(link_status==CONNECTING || link_status==NEGOTIATING || link_status==CONNECTION_ACCEPTED
			|| link_status==CONNECTED || link_status==DISCONNECTING)
		{
			printf("[ABORT] Aborting session (link_status=%d)\n", link_status);
			fflush(stdout);

			// Immediate teardown — no CLOSE_CONNECTION negotiation
			reset_session_state();

			set_role(RESPONDER);
			link_status=LISTENING;
			connection_status=RECEIVING;
			load_configuration(init_configuration, FULL, YES);

			// Clear buffers and reset timers
			fifo_buffer_tx.flush();
			fifo_buffer_backup.flush();
			fifo_buffer_rx.flush();
			reset_all_timers();

			// Reset messages_control so new CONNECT commands can work
			messages_control.status=FREE;

			// Send CANCELPENDING to cancel the connection attempt
			std::string str="CANCELPENDING\r";
			tcp_socket_control.message->length=str.length();
			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();

			// Send DISCONNECTED to fully clear Winlink's state and show we're free
			str="DISCONNECTED\r";
			tcp_socket_control.message->length=str.length();
			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();
		}

		// Send OK acknowledgement
		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command=="LISTEN ON")
	{
		original_role=RESPONDER;
		set_role(RESPONDER);
		local_capability = ((bandwidth_mode == BW_AUTO) ? CAP_WB_CAPABLE : 0) | ((encryption_mode != ENCRYPT_OFF) ? CAP_ENCRYPTION : 0);
		peer_capability = 0;
		wb_upgrade_pending = false;
		compression_enabled = false;
		link_status=LISTENING;
		connection_status=RECEIVING;
		reset_session_state();
		reset_all_timers();

		// Load init_configuration so we can hear incoming START_CONNECTION messages
		load_configuration(init_configuration, FULL, YES);

		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command=="LISTEN OFF")
	{
		original_role=RESPONDER;
		set_role(RESPONDER);
		link_status=IDLE;
		connection_status=IDLE;
		reset_all_timers();

		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command=="BW500")
	{
		// Narrowband only mode (500 Hz, Nc=10)
		printf("[BW] Setting NB only (500 Hz)\n");
		fflush(stdout);
		bandwidth_mode = BW_NB_ONLY;
		local_capability = ((encryption_mode != ENCRYPT_OFF) ? CAP_ENCRYPTION : 0);
#ifdef MERCURY_GUI_ENABLED
		g_gui_state.bandwidth_mode.store(BW_NB_ONLY);
#endif
		if(narrowband_enabled != YES)
			switch_narrowband_mode(YES);

		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command=="BW2300" || command=="BW2750")
	{
		// Auto mode (start NB, upgrade to WB if peer supports)
		printf("[BW] Setting auto mode (%s)\n", command.c_str());
		fflush(stdout);
		bandwidth_mode = BW_AUTO;
		local_capability = CAP_WB_CAPABLE | ((encryption_mode != ENCRYPT_OFF) ? CAP_ENCRYPTION : 0);
#ifdef MERCURY_GUI_ENABLED
		g_gui_state.bandwidth_mode.store(BW_AUTO);
#endif
		// Start in NB (auto-negotiation will upgrade if peer supports WB)
		if(narrowband_enabled != YES)
			switch_narrowband_mode(YES);

		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command=="BW2500")
	{
		// Legacy command — treat same as BW2300
		printf("[BW] Setting auto mode (BW2500, legacy)\n");
		fflush(stdout);
		bandwidth_mode = BW_AUTO;
		local_capability = CAP_WB_CAPABLE | ((encryption_mode != ENCRYPT_OFF) ? CAP_ENCRYPTION : 0);
#ifdef MERCURY_GUI_ENABLED
		g_gui_state.bandwidth_mode.store(BW_AUTO);
#endif
		if(narrowband_enabled != YES)
			switch_narrowband_mode(YES);

		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command=="VERSION")
	{
		std::string reply="VERSION Mercury " VERSION__ "\r";
		for(long unsigned int i=0;i<reply.length();i++)
		{
			tcp_socket_control.message->buffer[i]=reply[i];
		}
		tcp_socket_control.message->length=reply.length();
	}
	else if(command=="BUFFER TX")
	{
		std::string reply="BUFFER ";
		reply+=std::to_string(fifo_buffer_tx.get_size()-fifo_buffer_tx.get_free_size());
		reply+='\r';
		for(long unsigned int i=0;i<reply.length();i++)
		{
			tcp_socket_control.message->buffer[i]=reply[i];
		}
		tcp_socket_control.message->length=reply.length();
	}
	else if(command.substr(0,9)=="NOISESNR ")
	{
		// Dynamic noise injection control: "NOISESNR <db>" or "NOISESNR OFF"
		std::string arg=command.substr(9);
		if(arg=="OFF" || arg=="off") {
			noise_snr_db = 999.0;
			printf("[NOISE-Z] Noise OFF\n");
		} else {
			noise_snr_db = std::stod(arg);
			printf("[NOISE-Z] SNR set to %.1f dB\n", noise_snr_db);
		}
		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command.substr(0,12)=="NOISESIGNAL ")
	{
		// Set expected wire signal level for noise calibration: "NOISESIGNAL <dBFS>"
		std::string arg=command.substr(12);
		noise_signal_dbfs = std::stod(arg);
		printf("[NOISE-Z] Signal level set to %.1f dBFS\n", noise_signal_dbfs);
		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else
	{
		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}

	if (tcp_socket_control.get_status()==TCP_STATUS_ACCEPTED)
	{
		tcp_socket_control.transmit();
	}
}

void cl_arq_controller::ptt_on()
{
	if(passive_monitor) return;  // Never transmit in monitor mode
	std::string str="PTT ON\r";
	tcp_socket_control.message->length=str.length();

	for(int i=0;i<tcp_socket_control.message->length;i++)
	{
		tcp_socket_control.message->buffer[i]=str[i];
	}
	tcp_socket_control.transmit();
}
void cl_arq_controller::ptt_off()
{
	std::string str="PTT OFF\r";
	tcp_socket_control.message->length=str.length();

	for(int i=0;i<tcp_socket_control.message->length;i++)
	{
		tcp_socket_control.message->buffer[i]=str[i];
	}
	tcp_socket_control.transmit();
}



void cl_arq_controller::process_messages()
{
	this->update_status();
	if(this->role==COMMANDER)
	{
		process_messages_commander();
		process_buffer_data_commander();
	}
	else if(this->role==RESPONDER)
	{
		process_messages_responder();
		process_buffer_data_responder();
	}
}

void cl_arq_controller::reset_all_timers()
{
	link_timer.stop();
	link_timer.reset();
	watchdog_timer.stop();
	watchdog_timer.reset();
	gear_shift_timer.stop();
	gear_shift_timer.reset();
	receiving_timer.stop();
	receiving_timer.reset();
	switch_role_timer.stop();
	switch_role_timer.reset();
}

void cl_arq_controller::reset_session_state()
{
	// FIX-6: drop any RX-delivery tail buffered behind a back-pressured app socket.
	// A fresh session must not re-emit bytes from the previous connection's stream.
	rx_deliver_pending_len = 0;

	// Config state — must match init() defaults
	negotiated_configuration = init_configuration;
	data_configuration = init_configuration;
	forward_configuration = CONFIG_NONE;
	reverse_configuration = CONFIG_NONE;
	ack_configuration = init_configuration;

	// Turboshift — fresh state for next connection
	turboshift_phase = TURBO_FORWARD;
	turboshift_active = true;
	turboshift_last_good = -1;
	turbo_settle_pending = false;
	turboshift_initiator = false;
	turboshift_retries = 1;
	supershift_proven_ceiling = -1;
	// Option B (data-anchored promotion): reset to the session FLOOR (NOT the raw
	// init_configuration, which is CONFIG_0 on a GUI build whose initial_config isn't
	// robust — see gearshift-climb-engine.md §17). Nothing has carried data yet, so
	// BREAK floors at and probes climb one rung above the floor (ROBUST_0 for a -R
	// gearshift session, init_configuration/pinned config otherwise). See arq.h / §6.
	last_data_viable_config = session_floor_anchor(robust_enabled, init_configuration);
	// DEEP-SNR DOWN-HYSTERESIS (gearshift-climb-engine.md §10/§11): fresh session —
	// no anchor-rung BREAK streak and no clean streak yet.
	anchor_consec_break_fails = 0;
	clean_batches_at_current_config = 0;
	clean_batches_config = CONFIG_NONE;
	// FIX-A — fresh session: the robust dwell batch is not raised; the pin is 1
	// (data-flow-robust-tier-arq-batch.md §5.3). Mirrors the ctor init.
	pending_robust_dwell_batch = -1;
	robust_dwell_batch_active = false;
	// FIX-B — fresh session: no rung is under a floor-probe back-off; window at
	// INIT (gearshift-floor-probe-backoff.md §5). Mirrors the ctor init.
	probe_backoff_ms = PROBE_BACKOFF_MS_INIT;
	for(int i=0;i<FULL_CONFIG_LADDER_SIZE;i++) probe_backoff_until_ms[i]=0ULL;
	turbo_snr_ack_enabled = false;
	turbo_received_snr = -99.0f;
	turbo_switch_role_retries = 0;

	// BREAK / recovery
	emergency_nack_count = 0;
	emergency_break_active = 0;
	emergency_break_retries = 3;
	emergency_previous_config = init_configuration;
	break_drop_step = 2;  // initial aggression — see ctor comment
	breaks_since_last_data_success = 0;
	// WALL-B FIX-5 (audit R3): zero the carve cooldown on session reset / new CONNECT so a
	// fresh session is never capped at CFG15 by a prior session's carve-dead memory. Mirrors
	// the supershift_proven_ceiling = -1 reset at :3767.
	bigblock_carve_cooldown_batches = 0;
	bigblock_carve_cooldown_span = 0;
	cfg16_revack_starve_fails = 0;   // WALL-B FIX-9 D3 (R3 parity): clear the CFG16 reverse-ACK
	                                 // starvation streak on session reset / new CONNECT.
	break_recovery_phase = 0;
	break_recovery_retries = 0;
	ceiling_success_count = 0;
	break_detected = NO;
	break_probe_consec_match = 0;   // fix/break-fh-gate: fresh K-of-N streak (no-op read when env off)
	// fix/break-fh-gate: age the forward-health latch out on session reset so a stale
	// forward-OFDM decode from a prior session never suppresses an early BREAK. The
	// receive-frame index is monotonic (never reset); subtracting the window guarantees
	// break_fh_suppress() reads "not recent" until a fresh forward OFDM frame decodes.
	last_forward_ofdm_decode_frame = rx_receive_frame_index - (BREAK_FH_LATCH_FRAMES + 1);
	hail_detected = NO;
	hail_sent = NO;

	// Compression — always deinit (safe if not initialized; handles deferred pre-init)
	compressor.deinit();
	compression_enabled = false;
	b2f_compression_pending = false;

	// B2F handler — reset state for next connection
	b2f_handler.reset();

	// Encryption — wipe all key material (volatile memset, compiler can't elide)
	cipher_suite.wipe();
	encryption_enabled = false;
#ifdef MERCURY_GUI_ENABLED
	g_gui_state.encryption_active.store(false);
	// Don't clear psk_mismatch here — let it persist so the GUI shows the error.
	// It gets cleared on next successful encryption activation.
#endif
	tx_batch_counter = 0;
	rx_batch_counter = 0;
	consecutive_auth_failures = 0;
	if (kx_data_buf) { free(kx_data_buf); kx_data_buf = NULL; }
	kx_data_len = 0;

	// Data exchange
	block_under_tx = NO;
	consecutive_data_acks = 0;
	success_rate_data_clean = 100.0;  // CLEAN-BATCH VIABILITY (§9) — neutral per session
	frame_gearshift_just_applied = false;
	data_ack_received = NO;
	last_batch_fully_acked = false;  // CLEAN-BATCH VIABILITY (§9) — clear per session
	repeating_last_ack = NO;

	// Message tracking
	last_message_sent_type = NONE;
	last_message_sent_code = NONE;
	last_message_received_type = NONE;
	last_message_received_code = NONE;
	last_received_message_sequence = 255;

	// Return to NB after session ends — NB is the discovery/HAIL mode.
	// Skip when nb_probe_max==0: commander skips NB probing and hails in WB directly,
	// so responder must also stay in WB to detect WB HAIL patterns.
	if(nb_probe_max > 0 || bandwidth_mode == BW_NB_ONLY)
	{
		if(narrowband_enabled != YES)
		{
			printf("[NB-SWITCH] Restoring narrowband after session end\n");
			fflush(stdout);
		}
		narrowband_enabled = YES;
		telecom_system->narrowband_enabled = YES;
	}
	current_configuration = CONFIG_NONE;
	telecom_system->current_configuration = CONFIG_NONE;
	// v7 §2.3 (moved here from telecom_system::load_configuration): clear the
	// Moose-measured carrier offset on session boundary so a new correspondent
	// or post-drift session starts with a clean detector mixer. Earlier v7
	// placement at load_configuration() wiped this every data<->ack PHY swap
	// per batch, regressing cfg=6 throughput ~26%.
	telecom_system->last_coarse_freq_offset = 0.0;
	commander_configured_nb = -1;
	session_narrowband = false;
	peer_capability = 0;
	wb_upgrade_pending = false;
	handshake_confirmed = false;  // v9
	handshake_retries_left = MAX_HANDSHAKE_RETRIES;  // v9

	// Connection
	connection_id = 0;
	assigned_connection_id = 0;
	connection_attempts = 0;
	disconnect_requested = NO;

	// Phase 3a (Effective-Rate Optimizer) — wipe the rolling window. Prior
	// session's stats describe a different channel and would mislead the
	// Phase-3c decision layer. See EFFECTIVE_RATE_OPTIMIZER_DESIGN.md §4.1.
	opt_reset_window();
	// Phase 3c — also wipe cooldown / last-switch state so the optimizer
	// starts each session fresh (channel may have changed since last
	// session ended).
	rate_opt.reset_session_state();
	opt_pending_switch_cfg = -1;

	// SACK Design A — wipe axis-1/2/3 controller state. Prior session may
	// have degraded into SACK_MODE_OFF or built up partial-rate-window
	// history that would skew the new session's policy decisions. Mirror
	// the constructor initialization at the top of this file (see ~line
	// 215-260). Without this, a session that landed in SACK_MODE_OFF and
	// then reconnected (potentially to a different peer / improved channel)
	// would suppress SACK_RSP TX indefinitely. Previously masked when SACK
	// was opt-in; now load-bearing since SACK is default-on.
	axis2_consecutive_good_batches = 0;
	axis2_consecutive_bad_batches  = 0;
	axis2_partial_rate_count       = 0;
	axis2_partial_rate_pos         = 0;
	axis2_cooldown_batches         = 0;
	axis3_sack_mode                = SACK_MODE_ON;
	axis3_consecutive_sack_misses  = 0;
	axis3_recent_sack_ok_count     = 0;
	axis3_recent_sack_ok_pos       = 0;
	axis3_batches_since_off        = 0;

	// FIX-8 (data-integrity): a true session boundary (CONNECT / disconnect /
	// role-switch — R4/R029) starts a fresh transfer, so the delivery high-water
	// mark must be cleared. This is the ONLY production clear besides the ctor;
	// it is deliberately NOT cleared on the BREAK reset (arq_responder.cc:474)
	// nor the FULL load_configuration, so it SURVIVES the mid-transfer reset that
	// produces the dropped-batch gap. See bigblock_p3_hw/_fix8/FIX8_DESIGN.md §4.2.
	rsp_last_delivered_batch_seq_id = -1;

	// R029: a session reset (FORCED_ROLE_SWITCH, disconnect, role-switch) abandons
	// the entire in-flight TX state. The retransmit queue's frames belong to the
	// dead session's crypto epoch + bsi window — discard them so they cannot be
	// prepended to the first batch of the next session.
	clear_retx_queue();
}

void cl_arq_controller::opt_load_rate_table()
{
	// --no-optimizer: skip table load entirely. opt_evaluate_batch_end()
	// also early-exits, so the optimizer is fully inert. Used by
	// tools/effective_rate_calibrate.py to keep the optimizer from
	// switching configs mid-calibration.
	if (optimizer_disabled) {
		printf("[OPT] disabled via --no-optimizer, table not loaded\n");
		fflush(stdout);
		return;
	}
	// Path resolution chain (each tried until one yields a table with
	// valid cells; load() prints its own diagnostic per attempt):
	//   1. $MERCURY_RATE_TABLE  (env override)
	//   2. mercury/effective_rate_table.json       (real calibration output)
	//   3. effective_rate_table.json               (same, run from mercury/)
	//   4. mercury/effective_rate_table.synthetic.json   (dev synthetic)
	//   5. effective_rate_table.synthetic.json
	const char* env = std::getenv("MERCURY_RATE_TABLE");
	if (env && *env) {
		if (rate_opt.load(env)) return;
	}
	if (rate_opt.load("mercury/effective_rate_table.json")) return;
	if (rate_opt.load("effective_rate_table.json")) return;
	if (rate_opt.load("mercury/effective_rate_table.synthetic.json")) return;
	rate_opt.load("effective_rate_table.synthetic.json");
}

bool cl_arq_controller::opt_evaluate_batch_end(int* out_recommended_cfg)
{
	if (out_recommended_cfg) *out_recommended_cfg = current_configuration;
	// --no-optimizer: hard short-circuit before any state mutation
	// (including the cooldown tick) so the optimizer is fully inert under
	// calibration. opt_load_rate_table() also no-ops when disabled.
	if (optimizer_disabled)                return false;
	// Always drain the cooldown counter regardless of gate outcome so the
	// counter reflects elapsed batches, not "batches the optimizer actually
	// looked at".
	rate_opt.notify_cooldown_tick();

	// Hard gates — caller may add more.
	if (!rate_opt.is_enabled())            return false;
	if (!sack_v2_enabled)                  return false;
	if (turboshift_active)                 return false;
	if (emergency_break_active != 0)       return false;
	if (link_status != CONNECTED)          return false;
	if (role != COMMANDER)                 return false;
	// ROBUST_X is owned by the gearshift / break path. NB sessions are
	// supported when the loaded table has a "table_nb" section; otherwise
	// rate_opt.evaluate() short-circuits cleanly on NB.
	if (!is_ofdm_config(current_configuration)) return false;

	const bool is_nb = (narrowband_enabled == YES);

	// Below-table-range gate: when gearshift/BREAK has dropped us below
	// where the calibration table has data, OR when the observed channel
	// is worse than anything we calibrated for, the optimizer goes silent
	// and lets the dumber-but-safer gearshift/turboshift/BREAK system own
	// the link entirely. Re-engages automatically when both conditions
	// return to the calibrated region. min_cfg / max_sack are populated
	// at load() time from the actual table contents. NB and WB have
	// separate calibration ranges.
	int    min_cfg  = rate_opt.min_calibrated_cfg(is_nb);
	double max_sack = rate_opt.max_calibrated_sack_rate(is_nb);
	if (min_cfg >= 0 && current_configuration < min_cfg) return false;
	if (max_sack >= 0.0 && get_current_sack_rate() > max_sack) return false;

	int target = rate_opt.evaluate(current_configuration,
	                               get_current_effective_rate_bps(),
	                               get_current_sack_rate(),
	                               get_current_window_count(),
	                               is_nb ? NB_CONFIG_MAX : WB_CONFIG_MAX,
	                               is_nb);

	if (target == current_configuration) return false;
	if (out_recommended_cfg) *out_recommended_cfg = target;
	return true;
}

void cl_arq_controller::switch_narrowband_mode(int nb_enabled)
{
	if(narrowband_enabled == nb_enabled)
		return;
	printf("[NB-SWITCH] Switching to %s mode (init_config=%d, cur_config=%d)\n",
		nb_enabled ? "narrowband" : "wideband", init_configuration, current_configuration);
	fflush(stdout);

	// Pause audio processing before the switch: set data_ready=0 so the
	// processing thread won't start a new receive() cycle, and stop the
	// capture_prep thread from accumulating nUnder during the transition.
	telecom_system->data_container.data_ready = 0;
	// Reset nUnder and anti-re-decode markers from the previous bandwidth
	// mode. Stale nUnder from large NB frames (Nsymb=80) would corrupt the
	// first frames_to_read calculation after switching to WB.
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.mfsk_search_raw = 0;

	narrowband_enabled = nb_enabled;
	telecom_system->narrowband_enabled = nb_enabled;
	// Force reload by clearing current config on BOTH ARQ and PHY
	// (telecom_system skips load_configuration if config number matches,
	// even though narrowband_enabled changed the physical parameters)
	current_configuration = CONFIG_NONE;
	telecom_system->current_configuration = CONFIG_NONE;

	printf("[NB-SWITCH] Calling load_configuration(%d, FULL, YES)\n", init_configuration);
	fflush(stdout);
	load_configuration(init_configuration, FULL, YES);
	printf("[NB-SWITCH] load_configuration complete, cur_config=%d Nc=%d Nsymb=%d Nofdm=%d\n",
		current_configuration, telecom_system->ofdm.Nc,
		telecom_system->ofdm.Nsymb, telecom_system->data_container.Nofdm);
	fflush(stdout);

	opt_reset_window();
	rate_opt.reset_session_state();
	opt_pending_switch_cfg = -1;
}


// ============================================================================
// STEP 2 — live send-path wiring for the big-block (P3 prereq).
//
// bigblock_send_one_block(): emit the current new-data batch as ONE big-block
// instead of the per-frame preamble loop. Gated by send_batch() on
// telecom_system->bigblock_framing_enabled (the CFG16-rung framing flag, default
// OFF — FORCED true for this validation; the gearshift AUTO-election is DEFERRED
// to P4). Returns true when it HANDLED the batch (the caller skips the per-frame
// loop); false when it declined (caller falls through to the stock per-frame path
// — e.g. MFSK, retx batch, control frame in the batch, or no DATA frames).
//
// §5 audit (data-flow-bigblock-arq-unit.md): this is the TX producer of the block.
// It feeds the K codewords' REAL ARQ bytes via transmit_byte -> transmit_bigblock
// (P2.1 payload arg). It does NOT touch the optimizer/gearshift authority
// (optimizer_is_in_control arq.h:2041-2058, last_data_viable_config :2086,
// anchor_consec_break_fails :2106, probe_backoff :2031). Retx stays STOCK CFG16
// per-frame framing (P2.5) — this helper declines a retx batch.
//
// The per-codeword payload byte capacity (sub_len) = ldpc.K/8 (systematic info
// bytes per LDPC codeword). The block carries K = bigblock_codeword_count()
// codewords; the new-data batch packs min(K, message_batch_counter_tx) DATA
// frames, each frame's application payload occupying one sub-codeword slot
// (zero-padded to sub_len). The RX carve (bigblock_block_to_arq) is the exact
// inverse (INV-6). The block's batch_seq_id = the batch's bsi (one block = one
// batch, INV-1/P2.6).
//
// bigblock_pack_block(): build the K*sub_len on-wire block payload (header in cw0
// prefix + per-codeword app bytes + the whole-block CRC-32 in cw(K-1)'s trailer +
// the per-codeword CRC-8 tails) from the current new-data batch. Extracted from
// bigblock_send_one_block so the CAP-RESERVATION rule (V2 FIX-1, fact-doc §9) lives
// in ONE place exercised by BOTH production AND the MAX-PAYLOAD test arm — the V1
// LIVELOCK existed precisely because the test builder reserved the block-CRC field
// while production (cwc_cap=174 with no K-1 reservation) did NOT. On success returns
// true and fills out_payload (the packed byte image, == bigblock_tx_block_payload),
// out_K, out_sub_len, out_ndata, out_lengths; also stashes bigblock_tx_block_*.
bool cl_arq_controller::bigblock_pack_block(int n_data,
                                            std::vector<unsigned char>& out_payload,
                                            int& out_K, int& out_sub_len,
                                            int& out_ndata,
                                            std::vector<int>& out_lengths)
{
	if(telecom_system == NULL) return false;
	int K = telecom_system->bigblock_codeword_count();
	if(K <= 0) return false;
	if(n_data <= 0) return false;
	if(n_data > K) return false;   // batch larger than the block can carry -> stock path

	// sub_len = systematic info bytes per codeword (ldpc.K/8). The block payload is
	// K*sub_len bytes; we fill the first n_data sub-codewords from the batch frames'
	// application payloads and zero-pad the rest.
	int sub_len = telecom_system->ldpc.K / 8;
	if(sub_len <= 0) return false;
	const int alloc_size = N_MAX / 8;
	if(sub_len > alloc_size) sub_len = alloc_size;

	const int hdr_total = BIGBLOCK_HDR_TOTAL_BYTES(K);   // 2 + 2*K bytes
	// FAILURE-2 fix: each sub-codeword reserves its LAST byte for an on-wire CRC-8
	// (BIGBLOCK_CW_CRC_BYTES). The per-codeword app capacity shrinks by that 1 byte:
	//   cw0    app cap = sub_len - hdr_total - CRC          (header prefix + CRC both reserved)
	//   cwc    app cap = sub_len            - CRC           (1 <= c <= K-2)
	//   cwK-1  app cap = sub_len            - CRC - BLOCK   (V2 FIX-1: ALSO reserve the 4-byte
	//                                                         whole-block CRC-32 trailer field)
	const int cw0_cap = sub_len - hdr_total - BIGBLOCK_CW_CRC_BYTES;  // cw0 app capacity
	if(cw0_cap < 0) return false;                        // sub_len too small for header+CRC
	const int cwc_cap = sub_len - BIGBLOCK_CW_CRC_BYTES;             // cwc (1<=c<=K-2) app capacity
	if(cwc_cap < 0) return false;
	// V2 FIX-1 (LIVELOCK, fact-doc §9): the block-CRC field lives in cw(K-1)'s trailer at
	// BIGBLOCK_BLOCK_CRC_OFFSET = (K-1)*sub_len + (sub_len-1-4). Reserve it out of cw(K-1)'s
	// app capacity so a genuinely-clean full block never writes an app byte into the field
	// (TX would then overwrite it with the CRC AND its CRC image would diverge from the RX
	// recompute, which zeroes the field -> deterministic false-reject livelock). A defeat hook
	// MERCURY_BIGBLOCK_DEFEAT_CAPFIX=1 restores the PRE-FIX unreserved cap on the SAME binary so
	// the MAX-PAYLOAD fail-before is reproducible without a revert build. Production never sets it.
	bool defeat_capfix = false;
	{ const char* e = std::getenv("MERCURY_BIGBLOCK_DEFEAT_CAPFIX"); if(e && *e && atoi(e)!=0) defeat_capfix = true; }
	const int cwlast_cap = defeat_capfix ? cwc_cap : (cwc_cap - BIGBLOCK_BLOCK_CRC_BYTES);  // cw(K-1) app cap
	if(cwlast_cap < 0) return false;
	// INV-9 guard: frame 0 must fit in the reduced cw0 capacity. The big-block lattice
	// gives sub_len >> max_frame + hdr_total, so this never fires at the CFG16 rung.
	{
		int len0 = messages_batch_tx[0].length;
		if(len0 > cw0_cap) return false;                 // -> stock per-frame path
	}

	long block_payload_len = (long)K * (long)sub_len;
	std::vector<int>           block_payload((size_t)block_payload_len, 0);
	std::vector<unsigned char> block_payload_bytes((size_t)block_payload_len, 0);
	// the bsi the block advertises (one block = one batch). Use the first DATA
	// frame's batch_seq_id (the new-data builder stamps every frame the same bsi);
	// defensive 0 if unset.
	int block_bsi = messages_batch_tx[0].batch_seq_id;
	if(block_bsi < 0) block_bsi = 0;

	// Clamp each frame to its sub-codeword app capacity and record the per-codeword
	// length table (the RX reads it back to deliver the exact byte count per slot).
	std::vector<int> tx_lengths((size_t)K, 0);
	for(int i=0;i<n_data && i<K;i++)
	{
		// V2 FIX-1: the LAST codeword reserves the block-CRC field too (cwlast_cap).
		int cap = (i == 0) ? cw0_cap : (i == K - 1 ? cwlast_cap : cwc_cap);
		int len = messages_batch_tx[i].length;
		if(len < 0) len = 0;
		if(len > cap) len = cap;          // a frame longer than the sub-codeword is clamped
		tx_lengths[i] = len;
	}

	// --- Wire header in cw0 prefix: [bsi][n_data][length[0..K-1] uint16 LE] ----------
	block_payload[0]       = (int)(block_bsi & 0xFF);
	block_payload_bytes[0] = (unsigned char)(block_bsi & 0xFF);
	block_payload[1]       = (int)(n_data & 0xFF);
	block_payload_bytes[1] = (unsigned char)(n_data & 0xFF);
	for(int c=0;c<K;c++)
	{
		int lo = BIGBLOCK_HDR_FIXED_BYTES + 2*c;
		unsigned char b_lo = (unsigned char)(tx_lengths[c] & 0xFF);
		unsigned char b_hi = (unsigned char)((tx_lengths[c] >> 8) & 0xFF);
		block_payload[(size_t)lo + 0]       = (int)b_lo;
		block_payload_bytes[(size_t)lo + 0] = b_lo;
		block_payload[(size_t)lo + 1]       = (int)b_hi;
		block_payload_bytes[(size_t)lo + 1] = b_hi;
	}

	// --- App payloads: frame c -> sub-codeword c (cw0 after the header prefix) --------
	for(int i=0;i<n_data && i<K;i++)
	{
		int base = (i == 0) ? hdr_total : (i * sub_len);
		int len  = tx_lengths[i];
		for(int j=0;j<len;j++)
		{
			unsigned char b = (unsigned char)messages_batch_tx[i].data[j];
			block_payload[(size_t)base + j]       = (int)b;
			block_payload_bytes[(size_t)base + j] = b;
		}
		// (remaining bytes of this sub-codeword already 0 = pad)
	}

	// --- D2_BLOCKCRC: WHOLE-BLOCK CRC-32 ON THE WIRE (fix/bigblock-d3-carve) ----------
	// Stack a block-level CRC-32 over the ENTIRE assembled K*sub_len payload ON TOP of the
	// per-codeword CRC-8s. It lives in cw (K-1)'s TRAILER at BIGBLOCK_BLOCK_CRC_OFFSET(K,sub_len)
	// — the 4 bytes just before that codeword's per-cw CRC-8 tail — so cw0's header/app capacity
	// is UNCHANGED (a cw0 placement shrank app below the ~155B frames and stalled the block).
	// Stamped BEFORE the per-cw CRC-8 loop. To keep TX and RX computing the CRC-32 over an
	// IDENTICAL byte image with NO circular dependency, the CRC-32 covers the payload with TWO
	// sets of bytes treated as ZERO: (a) its own 4 block-CRC bytes, and (b) all K per-codeword
	// CRC-8 tail bytes (BIGBLOCK_CW_CRC_OFFSET(c)). At this point in TX BOTH are still 0 (the
	// per-cw loop has not run, the field is unwritten) so we compute directly; RX explicitly
	// zeroes the SAME bytes before recomputing. V2 FIX-1 reserves the field out of cw(K-1)'s app
	// cap above, so no app byte ever occupies the field (TX/RX CRC images match).
	{
		long bcrc_off = BIGBLOCK_BLOCK_CRC_OFFSET(K, sub_len);   // cw(K-1) trailer, before its CRC-8
		if(bcrc_off >= 0 && bcrc_off + BIGBLOCK_BLOCK_CRC_BYTES <= block_payload_len)
		{
			uint32_t bcrc = CRC32_calc((char*)block_payload_bytes.data(), (int)block_payload_len);
			for(int b=0;b<BIGBLOCK_BLOCK_CRC_BYTES;b++)
			{
				unsigned char by = (unsigned char)((bcrc >> (8*b)) & 0xFF);
				block_payload[(size_t)bcrc_off + b]       = (int)by;
				block_payload_bytes[(size_t)bcrc_off + b] = by;
			}
		}
	}

	// --- FAILURE-2 fix: per-codeword CRC-8 ON THE WIRE -------------------------------
	// Stamp a CRC-8 over each codeword's first BIGBLOCK_CW_CRC_SPAN(sub_len) bytes into its
	// tail byte. cw0's CRC covers [header | block-CRC | app | pad]; cw(K-1)'s covers
	// [app | block-CRC field | pad]. Stamped AFTER the block-CRC-32.
	for(int c=0;c<K;c++)
	{
		int crc_off  = BIGBLOCK_CW_CRC_OFFSET(c, sub_len);
		int crc_span = BIGBLOCK_CW_CRC_SPAN(sub_len);
		if(crc_off < 0 || crc_off >= block_payload_len || crc_span < 0) continue;
		unsigned char crc = CRC8_calc((char*)&block_payload_bytes[(size_t)c*sub_len], crc_span);
		block_payload[(size_t)crc_off]       = (int)crc;
		block_payload_bytes[(size_t)crc_off] = crc;
	}

	// Stash the TX block payload + geometry + per-codeword lengths so the in-process
	// single-block harness (and any RX in the same process) can carve it back
	// byte-faithfully.
	bigblock_tx_block_payload = block_payload_bytes;
	bigblock_tx_block_K       = K;
	bigblock_tx_block_sub_len = sub_len;
	bigblock_tx_block_bsi     = (unsigned char)(block_bsi & 0xFF);
	bigblock_tx_block_ndata   = n_data;
	bigblock_tx_block_lengths = tx_lengths;

	out_payload = block_payload_bytes;
	out_K       = K;
	out_sub_len = sub_len;
	out_ndata   = n_data;
	out_lengths = tx_lengths;
	return true;
}

bool cl_arq_controller::bigblock_send_one_block()
{
	if(telecom_system == NULL) return false;
	if(!telecom_system->bigblock_framing_enabled) return false;
	if(telecom_system->M == MOD_MFSK) return false;
	// P3 HW FIX: the big-block framing is a CFG16-RUNG mode (validated geometry K=8,
	// sub_len=ldpc.K/8). It must engage ONLY at CONFIG_16. The flag alone is not enough:
	// the gearshift starts at ROBUST_0 and climbs through OTHER OFDM configs (CONFIG_0..15,
	// all is_ofdm_config()==true, all !is_robust_config()). At those rungs the block has a
	// DIFFERENT, unvalidated geometry (HW-observed CONFIG_0: K=1 sub_len=12) that corrupts
	// the data path and STALLS the climb (every batch carves clean=0 -> no clean ACK ->
	// gearshift BREAKs, never reaches CFG16). Gate on the live config == CONFIG_16 so the
	// stock per-frame path carries CONFIG_0..15 and the block engages only at the validated
	// rung. (The gearshift AUTO-election of this rung is still P4; here the flag is FORCED.)
	if(current_configuration != CONFIG_16) return false;
	// Retx batches stay STOCK CFG16 per-frame framing (P2.5).
	if(sack_retransmit_active) return false;
	if(message_batch_counter_tx <= 0) return false;

	// Only emit a block for an all-DATA new-data batch. A CONTROL/ACK frame mixed
	// into the batch keeps the stock per-frame path (the block carries data only).
	int n_data = 0;
	for(int i=0;i<message_batch_counter_tx;i++)
	{
		if(messages_batch_tx[i].type==DATA_LONG || messages_batch_tx[i].type==DATA_SHORT)
			n_data++;
		else
			return false;   // non-data frame present -> decline, stock path handles it
	}
	if(n_data <= 0) return false;

	// Build the K*sub_len on-wire block payload (header + app + block-CRC-32 + per-cw CRC-8s)
	// via the SHARED packer so the V2 FIX-1 cap reservation (fact-doc §9) is identical to the
	// MAX-PAYLOAD test arm. On decline (geometry too small / frame 0 doesn't fit cw0) -> stock
	// per-frame path. The packer also stashes bigblock_tx_block_* (the in-process carve ground
	// truth, INV-6).
	int K = 0, sub_len = 0, packed_ndata = 0;
	std::vector<unsigned char> block_payload_bytes;
	std::vector<int> tx_lengths;
	if(!bigblock_pack_block(n_data, block_payload_bytes, K, sub_len, packed_ndata, tx_lengths))
		return false;
	long block_payload_len = (long)block_payload_bytes.size();
	// transmit_byte takes an int* payload; mirror the packed byte image.
	std::vector<int> block_payload((size_t)block_payload_len, 0);
	for(long i=0;i<block_payload_len;i++) block_payload[(size_t)i] = (int)block_payload_bytes[(size_t)i];

	// Emit ONE big-block through the production transmit_byte (branches to
	// transmit_bigblock when bigblock_framing_enabled). data = the K*sub_len real
	// ARQ payload bytes; nBytes = block_payload_len (>0 so transmit_bigblock packs
	// the real bytes, P2.1, NOT the PRBS fallback).
	int active_nsymb = telecom_system->get_active_nsymb();
	int frame_output_size = telecom_system->data_container.Nofdm
		* telecom_system->data_container.interpolation_rate
		* (active_nsymb + telecom_system->data_container.preamble_nSymb);
	// the block is ~ (4 preamble + Ngrid) symbols; size the TX buffer generously.
	int block_buf_samples = telecom_system->bigblock_tx_total_samples();
	if(block_buf_samples <= 0) block_buf_samples = frame_output_size;
	int block_pb_capacity = block_buf_samples + frame_output_size;
	std::vector<double> block_pb((size_t)block_pb_capacity, 0.0);

	// HEAP-OVERRUN ROOT-CAUSE FIX (fact-doc §13): this is the ONLY ARQ producer that hands
	// transmit_byte a BLOCK-sized `out`. Arm the per-call block-emit intent (+ the real
	// capacity) for the duration of THIS transmit_byte so its CFG16 branch may emit the
	// K-codeword waveform here; every other CFG16 transmit_byte (stock per-frame / single-
	// frame / control) leaves the flag false and keeps the per-frame OFDM geometry. The
	// guard auto-clears on scope exit (exception-safe).
	{
		cl_telecom_system::bigblock_emit_scope emit_guard(telecom_system, block_pb_capacity);
		telecom_system->transmit_byte(block_payload.data(), (int)block_payload_len,
			block_pb.data(), NO_FILTER_MESSAGE);
	}
	int K_tx = telecom_system->bigblock_last_tx_K;
	int n_tx = telecom_system->bigblock_last_tx_samples;
	printf("[BIGBLOCK-TX] one-block emit: K=%d (n_data=%d) sub_len=%d bsi=%u "
		"block_samples=%d\n", K_tx, n_data, sub_len, (unsigned)bigblock_tx_block_bsi, n_tx);
	fflush(stdout);
	if(K_tx <= 0 || n_tx <= 0) return false;

	// === TX-LEVEL PARITY (P3 HW, fact-doc §18) — band-limit the block through the
	// SAME FIR_tx1->FIR_tx2 cascade a regular CFG16 OFDM batch applies (send_batch,
	// arq_common.cc:4591-4592) before tx_transfer. The stock per-frame passband is
	// emitted RAW (NO_FILTER) by transmit_byte; the batch assembler is what runs the
	// WHOLE packed buffer through the two band-pass FIRs, and that band-limit is the
	// stage that disciplines a regular frame's final wire level (it flattens the
	// per-subcarrier pre_equalization_channel boost: STOCK +FIR peak 0.583->0.492,
	// rms 0.184->0.139, --test-bigblock-txlevel). The big-block had pre-eq + the
	// TX_SIG_OFDM level-cal applied at the modulator (telecom_system.cc:7072-7110)
	// but went to the wire WITHOUT this FIR, so it transmitted ~+1.4 dB peak / +2.3 dB
	// RMS HOTTER than a stock frame (HW scope: ~1400-1450 mVp-p vs the 1000 mVp-p
	// calibrated sweet spot). Apply the IDENTICAL conditioning here, in the same
	// order, so the block transmits at the SAME level and spectrum as a regular frame
	// — just longer under one preamble.
	//
	// Mirror send_batch's FIR scheme EXACTLY: edge-replicate a frame_output_size lead
	// pad and trail pad around the real block so the ~96-tap (2x97-1)/2 group-delay
	// transient bites only padding, then extract the n_tx real samples back from the
	// filtered buffer. block_pb already carries frame_output_size of slack capacity
	// (block_pb_capacity = block_buf_samples + frame_output_size, :3863), but build a
	// dedicated padded buffer so the lead/trail replication is unambiguous. The RX is
	// unchanged: FIR is an in-band band-pass, the OFDM signal sits inside the band, and
	// the pilot-based channel estimate captures+divides out any residual shaping — the
	// exact mechanism the stock OFDM RX already relies on for its own FIR'd frames.
	//
	// BLOCKED / DEFAULT-OFF (fact-doc §18.3): applying the FIR achieves the level goal
	// (RMS ratio bb/stock+FIR -> ~1.0, vs +2.3 dB without it) BUT costs ~0.5-1 dB of
	// decode margin that the big-block's BESPOKE RX (bigblock_rx_passband, sparse-pilot
	// estimate) cannot absorb on a 32-QAM CFG16 block: exactly ONE middle codeword (cw2)
	// miscorrects on the PERFECT channel, the --test-bigblock-multicw/-fullpath byte gate
	// drops 8/8 -> 7/8 and the partial-block path then never delivers (gate hang). The
	// pre-eq-only block "passes" 8/8 only because its UN-cut pre-eq edge boost (~+4.8x on
	// the band edges) over-powers those subcarriers — i.e. it passes BY running +2.3 dB
	// hot, not by having real margin. At the calibrated level the RX is genuinely ~1 cw
	// short via the FIR — the FIR's STEEP per-subcarrier band-edge attenuation is what the
	// sparse (2-cont-col + scat_dx=3/dy=4) thin grid cannot track, NOT the absolute level.
	//
	// === APPROACH A — FLAT GLOBAL GAIN CUT (default, P3 HW) =====================
	// The FIR breaks decode because it RESHAPES the spectrum (steep band-edge rolloff the
	// thin grid can't equalize). But on a clean channel the big-block decode is LEVEL-
	// INVARIANT for a FIXED SHAPE: the sparse pilot estimate already divides out the pre-eq
	// shape and decodes the pre-eq'd block 8/8 at the hot level. So instead of the FIR,
	// apply a SINGLE UNIFORM scalar to the whole block to bring its RMS down to a stock
	// CFG16 +FIR frame's RMS. A uniform scale preserves the SHAPE exactly -> the thin-grid
	// estimate is unchanged -> decode stays 8/8, while the wire level matches a regular
	// frame (the +2.3 dB HW overdrive closes), with ZERO pilot/geometry/throughput change.
	//
	// The cancel factor is MEASURED, not hardcoded (CLAUDE.md §1): it is the SAME level
	// reduction the FIR cascade would impose on THIS block. Apply FIR_tx1->FIR_tx2 to a
	// scratch copy (the band-limit reference), measure the FIR'd-vs-raw data-region RMS
	// ratio, and scale the raw (un-FIR'd, shape-preserving) block by that ratio. This makes
	// the flat-gain path land at the SAME disciplined level the FIR achieves
	// (--test-bigblock-txlevel STOCK +FIR data rms 0.139) — self-calibrating, config-
	// independent, derived from the modem's own band-pass response, no magic constant.
	//
	// A/B escape hatches: MERCURY_BIGBLOCK_FIR=1 -> old (decode-breaking) FIR path;
	// MERCURY_BIGBLOCK_NOGAINCUT=1 -> raw hot block (the previous default, +2.3 dB).
	bool bb_apply_fir = false;
	{ const char* e=std::getenv("MERCURY_BIGBLOCK_FIR"); if(e && atoi(e)!=0) bb_apply_fir=true; }
	bool bb_no_gaincut = false;
	{ const char* e=std::getenv("MERCURY_BIGBLOCK_NOGAINCUT"); if(e && atoi(e)!=0) bb_no_gaincut=true; }
	if(!bb_apply_fir)
	{
		if(bb_no_gaincut)
		{
			// legacy raw path: pre-eq + level-cal block, no level discipline (+2.3 dB hot).
			tx_transfer(block_pb.data(), n_tx);
			return true;
		}
		// FLAT-GAIN cut. Data region = samples AFTER the shared preamble (preamble_nSymb
		// OFDM symbols), matching --test-bigblock-txlevel's data-region RMS basis (the HW
		// concern is per-symbol transmit power, set by the data region).
		int pre_samp = telecom_system->data_container.preamble_nSymb
			* telecom_system->data_container.Nofdm
			* telecom_system->data_container.interpolation_rate;
		int dlo = (pre_samp < n_tx) ? pre_samp : 0;
		auto rms_region = [](const double* s, int lo, int hi)->double{
			double acc=0.0; int n=(hi>lo)?(hi-lo):0;
			for(int i=lo;i<hi;i++) acc += s[i]*s[i];
			return (n>0) ? std::sqrt(acc/(double)n) : 0.0;
		};
		double raw_rms = rms_region(block_pb.data(), dlo, n_tx);

		// FIR'd reference (scratch only — NOT transmitted). Same pad scheme as send_batch /
		// the FIR path below so the group-delay transient bites padding, giving a faithful
		// band-limited level reference.
		int pad = frame_output_size;
		int total_fir = pad + n_tx + pad;
		double fir_rms = raw_rms;   // fallback: identity (no cut) if FIR scratch fails
		if(total_fir > 0 && raw_rms > 1e-12)
		{
			std::vector<double> fir_in((size_t)total_fir, 0.0);
			std::vector<double> fir_t1((size_t)total_fir, 0.0);
			std::vector<double> fir_t2((size_t)total_fir, 0.0);
			for(int i=0;i<n_tx;i++) fir_in[(size_t)pad+i] = block_pb[(size_t)i];
			int rep = (pad < n_tx) ? pad : n_tx;
			for(int i=0;i<rep;i++)
			{
				fir_in[(size_t)i]            = block_pb[(size_t)i];
				fir_in[(size_t)(pad+n_tx)+i] = block_pb[(size_t)(n_tx-rep)+i];
			}
			telecom_system->ofdm.FIR_tx1.apply(fir_in.data(), fir_t1.data(), total_fir);
			telecom_system->ofdm.FIR_tx2.apply(fir_t1.data(), fir_t2.data(), total_fir);
			fir_rms = rms_region(&fir_t2[(size_t)pad], dlo, n_tx);
		}
		double g_flat = (raw_rms > 1e-12) ? (fir_rms / raw_rms) : 1.0;
		if(!(g_flat > 0.0) || g_flat > 1.0) g_flat = (g_flat>1.0)?1.0:1.0;  // clamp: never boost
		printf("[BIGBLOCK-TX] flat-gain cut: raw_rms=%.6f fir_rms=%.6f g=%.4f (%.2f dB) "
			"[shape-preserving level discipline; decode unchanged]\n",
			raw_rms, fir_rms, g_flat, 20.0*std::log10((g_flat>0)?g_flat:1e-12));
		fflush(stdout);
		for(int i=0;i<n_tx;i++) block_pb[(size_t)i] *= g_flat;
		tx_transfer(block_pb.data(), n_tx);
		return true;
	}
	{
		int pad = frame_output_size;                  // same pad width send_batch uses
		int total_fir = pad + n_tx + pad;             // lead pad + block + trail pad
		std::vector<double> fir_in((size_t)total_fir, 0.0);
		std::vector<double> fir_t1((size_t)total_fir, 0.0);
		std::vector<double> fir_t2((size_t)total_fir, 0.0);
		// real block in the middle
		for(int i=0;i<n_tx;i++) fir_in[(size_t)pad+i] = block_pb[(size_t)i];
		// lead pad = replicate the block's leading `pad` samples; trail pad = replicate
		// the block's trailing `pad` samples (send_batch:4578-4585 edge replication).
		int rep = (pad < n_tx) ? pad : n_tx;
		for(int i=0;i<rep;i++)
		{
			fir_in[(size_t)i]                 = block_pb[(size_t)i];                 // lead
			fir_in[(size_t)(pad+n_tx)+i]      = block_pb[(size_t)(n_tx-rep)+i];      // trail
		}
		telecom_system->ofdm.FIR_tx1.apply(fir_in.data(), fir_t1.data(), total_fir);
		telecom_system->ofdm.FIR_tx2.apply(fir_t1.data(), fir_t2.data(), total_fir);
		// the real block is the [pad, pad+n_tx) region of the filtered buffer.
		tx_transfer(&fir_t2[(size_t)pad], n_tx);
	}

	// GAP-2 LIVE-PATH tally (diag/livepath-sim): a real big-block was emitted onto the
	// wire (the TX switch engaged + tx_transfer succeeded). SIM_INPROC-only test-static.
	sim2_tx_block_emits++;

	return true;
}

// bigblock_block_ftr_or(): MULTI-CW WINDOW FIX (fact-doc §17). The big-block decode
// snapshots buffer_Nsymb samples (receive_bigblock, telecom_system.cc:8219), but the
// snapshot only fires when frames_to_read counts down to 0 — so frames_to_read sets HOW
// MANY FRESH symbols are accumulated into the ring before the block is handed to the
// decoder. The block spans bigblock_rx_block_nsymb() (= preamble_nSymb + Ngrid, ~64) OFDM
// symbols; a stock CFG16 frame is get_active_nsymb()+preamble_nSymb (~13). Codewords map
// SEQUENTIALLY across the block's symbols (cw0 = earliest, cw7 = latest), so a stock-frame
// window snapshots only the block HEAD: cw0's symbols are fresh -> cw0 byte-correct; cw1..7
// fall outside the fresh region -> decode from silence/stale ring -> DETERMINISTIC garbage
// (the §15.2 partial-block signature, recurring on the live multi-block path because the
// §15.2 fix only guarded the messages_control turnaround + the post-carve re-arm, NOT the
// per-block ACK-turnaround / FAIL re-arms — see §17.3).
//
// This helper raises a stock frames_to_read value to the FULL block span (+10 turnaround
// margin, matching the §15.2 :6994 re-arm) WHEN the bigblock rung is live, and returns
// stock_ftr UNCHANGED otherwise. Reuses the EXACT geometry source the §15.2 sites use
// (bigblock_rx_block_nsymb()), so there is no parallel mechanism and the two peers cannot
// diverge. On every non-CFG16 / framing-off / MFSK path the guard is false -> the binary is
// byte-identical to baseline. MERCURY_BIGBLOCK_DEFEAT_FIX=1 bypasses the clamp (returns the
// stock value) so the SAME binary reproduces the pre-fix truncated-window corruption for the
// fail-before/pass-after A/B (production never sets it).
int cl_arq_controller::bigblock_block_ftr_or(int stock_ftr)
{
	if(telecom_system == NULL) return stock_ftr;
	if(!(telecom_system->bigblock_framing_enabled
	     && telecom_system->M != MOD_MFSK
	     && current_configuration == CONFIG_16))
		return stock_ftr;
	// WALL-B FIX-3 (C2b): the SINGLE chokepoint reverting ALL SIX block-span re-arm sites
	// (arq_commander.cc:4802, arq_common.cc:5911/6023/6495/9117/9189) to the stock per-frame
	// cadence once the carve is SUSPENDED. While suspended the carve never runs, so cw1..7
	// stale-ring garbage cannot occur (INV-3) — the RSP is decoding STOCK per-frame CFG16
	// frames (the CFG16-PHY demote SET_CONFIG / per-frame data), which WANT the stock window.
	// On recovery (a config change to CFG15, or a carve accept resetting the streak) the
	// block-span re-arm resumes BEFORE the next carve.
	if(bigblock_carve_suspended()) return stock_ftr;
	// reproducer hook: restore the pre-fix stock-frame arming for the fail-before A/B.
	{ const char* e = std::getenv("MERCURY_BIGBLOCK_DEFEAT_FIX");
	  if(e && *e && atoi(e)!=0) return stock_ftr; }
	int block_nsymb = telecom_system->bigblock_rx_block_nsymb();
	if(block_nsymb <= 0) return stock_ftr;          // geometry unavailable -> leave stock
	int block_ftr = block_nsymb + 10;               // block span + turnaround margin
	return (block_ftr > stock_ftr) ? block_ftr : stock_ftr;
}

// bigblock_carve_suspended(): WALL-B FIX-3 — the carve-suspend predicate. Returns true iff
// the RSP has accumulated K (BIGBLOCK_CARVE_SUSPEND_K=3) consecutive cw0-CRC carve REJECTS
// with 0 accepts while parked at CFG16, so the CFG16 carve route + block-span re-arm should
// be SUSPENDED (the RSP then decodes the CFG16-PHY demote SET_CONFIG / BREAK on the stock
// per-frame path it already proved during the climb). The streak only ever increments on the
// CFG16 big-block reject branch (arq_common.cc:8186) and resets on accept / config change, so
// off the CFG16 big-block rung the streak is 0 and this is always false (byte-identical).
// MERCURY_BIGBLOCK_DEFEAT_CARVESUSPEND=1 forces FALSE (restores the pre-fix deaf RSP) for the
// fail-before A/B arm; production never sets it.
bool cl_arq_controller::bigblock_carve_suspended()
{
	if(bigblock_rx_carve_fail_streak < BIGBLOCK_CARVE_SUSPEND_K) return false;
	{ const char* e = std::getenv("MERCURY_BIGBLOCK_DEFEAT_CARVESUSPEND");
	  if(e && *e && atoi(e)!=0) return false; }   // reproducer: pre-fix deaf RSP
	return true;
}

// BREAK forward-health gate (fix/break-fh-gate). Default-OFF env MERCURY_BREAK_FH_GATE.
// Cached once: getenv is a syscall, and this is polled on every failed-decode receive().
// break_fh_gate_test_override: UNIT-TEST seam only (-1 = honor env, 0/1 = force). It lets
// run_break_fh_gate_tests() exercise BOTH gate states in one process despite the cached env
// read; production NEVER sets it, so the env path is unchanged -> default-off byte-identical.
int cl_arq_controller::break_fh_gate_test_override = -1;
bool cl_arq_controller::break_fh_gate_enabled()
{
	if(break_fh_gate_test_override >= 0) return break_fh_gate_test_override != 0;
	static const bool en = (std::getenv("MERCURY_BREAK_FH_GATE") != nullptr);
	return en;
}

// FIX-A forward-health LATCH. Returns true (probe should be SUPPRESSED) iff the gate
// is enabled AND a forward OFDM frame decoded within the last BREAK_FH_LATCH_FRAMES
// receive() iterations. When the env is unset this is unconditionally false, so the
// receive() BREAK gate is bit-identical to 48103fa.
bool cl_arq_controller::break_fh_suppress() const
{
	if(!break_fh_gate_enabled()) return false;
	// last_forward_ofdm_decode_frame inits far in the past => not recent at session start.
	return (rx_receive_frame_index - last_forward_ofdm_decode_frame) <= BREAK_FH_LATCH_FRAMES;
}

// FIX-B K-of-N corroboration. The caller passes the per-frame match decision
// (metric>=threshold && matched>=break_match_threshold, already evaluated). When the
// env is unset this is a pure pass-through (single-shot, returns probe_matched) so the
// receive() gate is bit-identical. When set, break_detected is returned true only after
// BREAK_KOFN_K consecutive matches; any non-match (or a suppressed/aged frame) resets the
// streak — a real BREAK is retried/sustained and survives K-of-N, the per-batch alias does not.
bool cl_arq_controller::break_kofn_corroborate(bool probe_matched)
{
	if(!break_fh_gate_enabled())
		return probe_matched;             // byte-identical: one match detonates
	if(!probe_matched)
	{
		break_probe_consec_match = 0;     // streak broken
		return false;
	}
	break_probe_consec_match++;
	if(break_probe_consec_match >= BREAK_KOFN_K)
	{
		break_probe_consec_match = 0;     // consumed: re-arm for the next independent BREAK
		return true;
	}
	return false;                         // need more corroboration
}

// FIX-D: the WALL-B FIX-3 carve-suspend gate-lift, gated on MERCURY_BREAK_FH_GATE.
// When the FH gate is enabled, return false so the only-remaining OFDM-alias guard
// (coarse_metric<0.30) is NOT lifted in carve-suspend state (the marginal-CFG16 alias
// domain). When unset, return bigblock_carve_suspended() exactly -> bit-identical.
bool cl_arq_controller::break_fh_carve_lift()
{
	if(break_fh_gate_enabled()) return false;
	return bigblock_carve_suspended();
}

// WALL-B FIX-3 — streak state machine, the ONE source of truth shared by the receive()
// carve-gate branches (arq_common.cc:8186/8229) and the unit test
// (test_bigblock_arq_unit.cc). bigblock_note_carve_reject(): a real CFG16 cw0-CRC carve
// REJECT — accumulate the consecutive-fail streak; at K (=3) the carve route + block-span
// re-arm are SUSPENDED. Returns true iff this reject just crossed K (so the caller can fire
// the [BB-CARVE-SUSPEND] log / SIM tally exactly once). Reset on a real carve accept and on
// any config change (load_configuration). bigblock_note_carve_accept(): a real block carved
// -> the carve is viable again, reset the streak to 0 (the recovery event; on the
// carve-SUCCESS path this fires first so the streak never reaches K -> carve-success
// byte-identical, INV-6).
bool cl_arq_controller::bigblock_note_carve_reject()
{
	bigblock_rx_carve_fail_streak++;
	if(bigblock_rx_carve_fail_streak == BIGBLOCK_CARVE_SUSPEND_K
	   && bigblock_carve_suspended())
	{
		printf("[BB-CARVE-SUSPEND] CFG16 big-block carve SUSPENDED after %d consecutive "
			"cw0-CRC rejects (0 carves) -> RSP decodes CFG16-PHY control/BREAK on the "
			"stock per-frame path (stays at CFG16 PHY; carve re-arms on a real block or "
			"a config change)\n", bigblock_rx_carve_fail_streak);
		fflush(stdout);
		return true;
	}
	return false;
}

void cl_arq_controller::bigblock_note_carve_accept()
{
	bigblock_rx_carve_fail_streak = 0;
}

// bigblock_acq_window_fits(): ACQUISITION-WINDOW POSITION GUARD (fact-doc §19). After
// receive_bigblock located the head preamble (bigblock_last_rx_head_delay_samples) and
// recorded the captured-window length (bigblock_last_rx_capture_nsamples), test whether the
// FULL block — head + preamble_nSymb + Ngrid OFDM symbols (= bigblock_rx_block_nsymb()
// symbols counting from head_delay) — fits inside the captured samples. When it does NOT,
// bb_at zero-padded the tail (the block landed too late in the window / its tail had not yet
// arrived in the ring at snapshot time), so the carve would see a truncated block and the
// estimate collapses -> cw0 wire-CRC fails. The receive() guard then DEFERS instead of
// carving garbage. head_delay<0 (acq fail) or capture<=0 (no decode) => treated as "not a
// locatable block" -> returns true (let the existing cw0-CRC gate handle it as today; do not
// defer on a non-block). Geometry from the SAME bigblock_rx_block_nsymb() the §17 sites use.
bool cl_arq_controller::bigblock_acq_window_fits()
{
	if(telecom_system == NULL) return true;
	long head = telecom_system->bigblock_last_rx_head_delay_samples;
	int  cap  = telecom_system->bigblock_last_rx_capture_nsamples;
	if(head < 0 || cap <= 0) return true;            // no locatable block -> do not defer
	int block_nsymb = telecom_system->bigblock_rx_block_nsymb();
	if(block_nsymb <= 0) return true;                // geometry unavailable -> do not defer
	int sym_samples = telecom_system->data_container.Nofdm
	                  * telecom_system->data_container.interpolation_rate;
	if(sym_samples <= 0) return true;
	// block_end = head + (preamble_nSymb + Ngrid)*sym_samples = head + block_nsymb*sym_samples.
	// bigblock_rx_passband reads forward from data_start = head + preamble_nSymb*sym_samples for
	// Ngrid*sym_samples; the END coincides with head + block_nsymb*sym_samples.
	long block_end = head + (long)block_nsymb * (long)sym_samples;
	return block_end <= (long)cap;
}

// bigblock_receive_carve(): the RX side of STEP 2. After receive_byte() branched
// to receive_bigblock() and decoded ONE block (stashing the per-codeword clean
// vector telecom_system->bigblock_last_rx_cw_ok + the K decoded info-bit
// sub-units in data_byte_out), translate the block into the ARQ data unit via the
// already-built+gated bigblock_block_to_arq() carve (P2.4/2.5/2.6): carve cw_ok ->
// messages_rx[], set the synthetic EOB=K-1, fire one ACK / partial SACK / bsi-once.
//
// info_bits = the decoded systematic info bits receive_bigblock wrote to `out`
// (K*ldpc.K of them); we re-pack them into K*sub_len bytes (sub_len = ldpc.K/8,
// the same packing the TX used) so the carve is byte-faithful (INV-6).
//
// PHASE 1 (fact-doc §11): the block carries a SELF-DESCRIBING header in cw0's prefix
// [bsi, n_data, length[0..K-1]]. When use_wire_header is true (the live path), the
// carve PARSES that header off the decoded payload and uses the WIRE bsi (authoritative,
// drift-proof) + the per-codeword length table (compression transparency). fallback_bsi
// is used only when use_wire_header is false (legacy) or the header is unusable.
//
// §5 audit: this is the RX consumer wiring. It calls ONLY bigblock_block_to_arq
// GAP-3 CARVE-GATE HARDENING (cfg16-controlack-hold): is the just-decoded CFG16
// acquisition a REAL big-block? A real block's cw0 carries the FEC+CRC-protected
// [bsi, n_data, length-table] header followed (like every codeword) by a CRC-8 tail
// byte over its first BIGBLOCK_CW_CRC_SPAN(sub_len) de-whitened bytes. A single OFDM
// control frame, stale audio, or noise that the unconditional CFG16->receive_bigblock
// gate mis-routed into the carver does NOT produce a cw0 whose recomputed CRC-8 matches
// — so this is the structural discriminator the pinpoint identified (the cw0
// wire-header CRC). We recompute ONLY cw0's CRC here (cheap, header-bearing); the full
// per-codeword demote still runs inside bigblock_receive_carve for the accepted block.
// Reuses the EXACT de-whiten + CRC8_calc + BIGBLOCK_CW_CRC_* the TX/carve use, so it
// cannot drift from the on-wire format. Returns false (reject) on any inconsistency.
bool cl_arq_controller::bigblock_rx_cw0_header_valid()
{
	if(telecom_system == NULL) return false;
	int K = telecom_system->bigblock_last_rx_K;
	if(K <= 0) return false;
	// WALL-B FIX-3 carve-suspend TEST injection (SIM_INPROC only): force the cw0-CRC carve to
	// REJECT — the exact HW symptom (the CFG16 carve mis-decodes the block-span window and fails
	// cw0-CRC) — WITHOUT mutating the underlying passband audio (unlike the D3 RINGPHASE/STALERING
	// levers, which corrupt the snapshot and would also break the stock decode of the short
	// CFG16-PHY control frame). This isolates the carve-fail event from the audio so the
	// carve-suspend watchdog test can verify the RSP decodes the intact control/BREAK on the
	// stock per-frame path once the carve is suspended. Read each call (rare CFG16 path); zero
	// production effect (production never sets it). bigblock_block_to_arq still gets a real
	// decoded block, so the suspend path is exercised against a genuine acquisition.
	//   MERCURY_BIGBLOCK_SIM_CARVEFAIL < 0 (or "all"): reject EVERY cw0 check (persistent fail,
	//     Test A/B — the carve never recovers, so the streak reaches K and stays suspended).
	//   MERCURY_BIGBLOCK_SIM_CARVEFAIL = N > 0: reject the FIRST N cw0 checks of this run then
	//     pass — a TRANSIENT carve fail (Test C / RISK-A: streak must reset on the first accept
	//     and the block must still carve byte-faithful, never starved).
	{ const char* e = std::getenv("MERCURY_BIGBLOCK_SIM_CARVEFAIL");
	  if(e && *e){
	    int n = (std::string(e)=="all") ? -1 : atoi(e);
	    if(n < 0) return false;                              // persistent reject
	    static long carvefail_remaining = -2;                // -2 = "not yet primed this run"
	    static std::string carvefail_seen = "\x01";
	    if(carvefail_seen != std::string(e)){ carvefail_seen = e; carvefail_remaining = n; }
	    if(carvefail_remaining > 0){ carvefail_remaining--; return false; }   // transient reject
	  }
	}
	int sub_len = telecom_system->ldpc.K / 8;
	if(sub_len <= 0) return false;
	const int alloc_size = N_MAX / 8;
	if(sub_len > alloc_size) sub_len = alloc_size;
	int crc_span = BIGBLOCK_CW_CRC_SPAN(sub_len);
	int crc_off  = BIGBLOCK_CW_CRC_OFFSET(0, sub_len);   // cw0's CRC tail byte index
	if(crc_span <= 0 || crc_off < 0 || crc_off >= sub_len) return false;
	// Need cw0's full sub_len*8 decoded info bits.
	const std::vector<int>& bits = telecom_system->bigblock_rx_infobits;
	int need_bits = sub_len * 8;
	if((int)bits.size() < need_bits) return false;
	// De-whiten ONLY cw0's bits (self-inverse XOR over the SAME PRBS the TX whitened
	// with). bigblock_whiten_bits whitens the WHOLE block from bit 0; cw0 occupies the
	// first sub_len*8 bits, so de-whitening a length-need_bits prefix recovers cw0.
	std::vector<int> dw(bits.begin(), bits.begin() + need_bits);
	telecom_system->bigblock_whiten_bits(dw.data(), need_bits);
	std::vector<unsigned char> cw0((size_t)sub_len, 0);
	for(int b=0;b<sub_len;b++)
	{
		unsigned char byte = 0;
		for(int bit=0;bit<8;bit++)
			if(dw[b*8 + bit] & 1) byte |= (unsigned char)(1u << bit);
		cw0[b] = byte;
	}
	unsigned char calc = CRC8_calc((char*)cw0.data(), crc_span);
	unsigned char wire = cw0[(size_t)crc_off];
	return (calc == wire);
}

// (already §5-audited) — it does NOT touch the optimizer/gearshift authority.
// Returns SUCCESSFUL when the block was carved into the ARQ layer.
//
// === §1.5 CROSS-LAYER DATA-FLOW AUDIT — block-integrity state (D2_BLOCKCRC) =========
// New shared state added by this fix: a WHOLE-BLOCK CRC-32 carried in cw (K-1)'s TRAILER
// (datalink_defines.h BIGBLOCK_BLOCK_CRC_OFFSET(K,sub_len) = the 4 bytes just before that
// codeword's per-cw CRC-8), 4 bytes uint32 LE, over the assembled K*sub_len de-whitened
// payload with its own 4 bytes AND all K per-cw CRC-8 tail bytes zeroed.
//   1. PRODUCERS (writers of the wire CRC-32 field):
//      - TX: bigblock_send_one_block (arq_common.cc, "D2_BLOCKCRC: WHOLE-BLOCK CRC-32"
//        block) writes it once per emitted block, BEFORE the per-cw CRC-8 loop (so both
//        zeroed sets are still 0 at compute) and AFTER all app payloads (input is final).
//      - Tests: every harness that hand-builds a wire block (test_bigblock_arq_unit.cc
//        tx_truth builders) must stamp it the SAME way (added in this fix).
//   2. CONSUMERS (readers/checkers):
//      - RX: bigblock_receive_carve (THIS function, "D2_BLOCKCRC: WHOLE-BLOCK CRC-32
//        VERIFY") recomputes over the de-whitened payload (zeroing the block-CRC field +
//        all per-cw CRC tails) and clears ALL cw_ok on mismatch — ONLY when the per-cw
//        layer reports the block fully clean (n_clean==K), i.e. the would-be DELIVER path.
//      - bigblock_rx_cw0_header_valid (the receive() carve GATE) recomputes cw0's per-cw
//        CRC-8; UNAFFECTED — the block-CRC lives in cw K-1, not cw0, so cw0's header/CRC-8
//        span is byte-identical to before this fix (no false gate reject of real blocks).
//   3. VALID STATES: before any producer writes, the 4 field bytes are 0 (the zero
//      placeholder). On a total acquisition miss (info_bits==NULL) there is no payload ->
//      the RX check is SKIPPED. A degenerate sub_len that cannot hold the trailer ->
//      bounds guard skips the check (no false reject of a tiny non-CFG16 geometry).
//   4. INVARIANTS the consumers assume: (a) TX and RX zero the SAME bytes (block-CRC field
//      + per-cw CRC tails) before computing -> identical input, no self/circular reference;
//      (b) the field lives in cw (K-1) so it does NOT shrink cw0's app capacity (a cw0
//      placement dropped cw0 below the ~155B first frame -> bigblock declined -> block never
//      emitted; this placement keeps cw0 unchanged and steals 4B from cw K-1's ~174B app);
//      (c) on mismatch ALL cw_ok are cleared -> the EXISTING PARTIAL/SACK branch of
//      bigblock_block_to_arq runs (no new delivery path) -> the block is re-sent, never
//      delivered. The per-cw CRC-8 demote is UNCHANGED and still picks the SACK gaps when
//      the block-CRC passes; the block-CRC only fires when the per-cw layer said all-clean.
//   5. WHAT THE FIX CHANGES: it adds a reject decision BEFORE bigblock_block_to_arq. The
//      only consumer of cw_ok downstream is bigblock_block_to_arq; clearing all bits is a
//      valid input it already handles (n_clean=0 -> PARTIAL). No optimizer/gearshift state
//      is touched (the carve never did). The wire grows 4 bytes/block in cw K-1's app
//      region. No legacy peers exist -> the trailer is unconditional.
int cl_arq_controller::bigblock_receive_carve(const int* info_bits,
		unsigned char fallback_bsi, bool use_wire_header)
{
	if(telecom_system == NULL) return ERROR_;
	int K = telecom_system->bigblock_last_rx_K;
	if(K <= 0) return ERROR_;
	int sub_len = telecom_system->ldpc.K / 8;
	if(sub_len <= 0) return ERROR_;
	const int alloc_size = N_MAX / 8;
	if(sub_len > alloc_size) sub_len = alloc_size;

	// the K-bit per-codeword clean vector (the SACK granularity).
	const std::vector<int>& cw_ok_vec = telecom_system->bigblock_last_rx_cw_ok;
	std::vector<int> cw_ok((size_t)K, 0);
	for(int c=0;c<K;c++)
		cw_ok[c] = (c < (int)cw_ok_vec.size()) ? (cw_ok_vec[c] ? 1 : 0) : 0;

	// re-pack the decoded info bits -> K*sub_len bytes (LSB-first, the byte_to_bit
	// convention transmit_bigblock used). info_bits may be null on a total acq miss.
	// HEAP-OVERRUN ROOT-CAUSE FIX (fact-doc §13): on the live path info_bits is
	// telecom_system->bigblock_rx_infobits (sized K*ldpc.K by receive_bigblock), NOT
	// data_container.data_byte[N_MAX] — this carve reads nbits = K*sub_len*8 = K*ldpc.K ints,
	// which would over-read the 1600-int data_byte. (The in-process test passes its own
	// correctly-sized vector.) nbits == K*ldpc.K by construction (sub_len == ldpc.K/8).
	// PHASE 1 (fact-doc §11.6): the real-bytes TX energy-disperses (whitens) the payload
	// bits before LDPC encode (so a zero-padded short compressed frame still decodes).
	// De-whiten the decoded bits HERE (self-inverse XOR, same PRBS) before re-packing to
	// bytes — recovers the exact payload. Copy into a local buffer first (don't mutate the
	// source in place).
	std::vector<unsigned char> payload((size_t)K * sub_len, 0);
	if(info_bits != NULL)
	{
		int nbits = K * sub_len * 8;
		std::vector<int> dw((size_t)nbits, 0);
		for(int i=0;i<nbits;i++) dw[i] = info_bits[i] & 1;
		telecom_system->bigblock_whiten_bits(dw.data(), nbits);   // de-whiten (self-inverse)
		for(int c=0;c<K;c++)
		{
			for(int b=0;b<sub_len;b++)
			{
				unsigned char byte = 0;
				for(int bit=0;bit<8;bit++)
				{
					int idx = (c*sub_len + b)*8 + bit;
					if(dw[idx] & 1) byte |= (unsigned char)(1u << bit);
				}
				payload[(size_t)c*sub_len + b] = byte;
			}
		}
	}

	// --- D2_BLOCKCRC FALSEPASS REPRODUCER HOOK (fix/bigblock-d3-carve) ----------------
	// Model the HW NO-GO (WINRUN_FINAL_VERDICT.json): a corrupt K-block whose 8 per-cw
	// CRC-8 all FALSE-PASS (the LDPC miscorrected each codeword to a valid-but-wrong word
	// whose recomputed CRC-8 still matched), so the block was DELIVERED with wrong bytes.
	// MERCURY_BIGBLOCK_FALSEPASS_CW=k corrupts codeword k's DE-WHITENED PAYLOAD bytes and
	// then RE-STAMPS its per-cw CRC-8 over the corrupt bytes, so the per-cw demote below
	// PASSES on the wrong data (a self-consistent false-locked window) — exactly the HW
	// case where ONLY a whole-block CRC can catch it. We mutate the local `payload` AFTER
	// de-whiten (the byte image the demote + carve consume), corrupting app bytes only (not
	// the cw's own CRC byte, which we then recompute). Production never sets it.
	{
		const char* e = std::getenv("MERCURY_BIGBLOCK_FALSEPASS_CW");
		int fp_cw = (e && *e) ? atoi(e) : -1;
		// ONE-SHOT: corrupt only the FIRST block of the run (bigblock_first_clean is still <0
		// until bigblock_block_to_arq records this first carve). The re-sent copy (after the
		// block-CRC reject routes the first block to PARTIAL/SACK) is NOT injected -> it delivers
		// clean, proving the recover-via-re-send pass-after.
		if(info_bits != NULL && fp_cw >= 0 && fp_cw < K && bigblock_first_clean < 0)
		{
			int crc_off  = BIGBLOCK_CW_CRC_OFFSET(fp_cw, sub_len);
			int crc_span = BIGBLOCK_CW_CRC_SPAN(sub_len);
			int base     = fp_cw * sub_len;
			// flip the first app byte AFTER any cw0 header prefix so a delivered block is
			// guaranteed wrong (cw0 header bytes are not app data). Use codeword-interior
			// bytes well clear of the CRC tail.
			int app_start = (fp_cw == 0) ? BIGBLOCK_HDR_TOTAL_BYTES(K) : 0;
			int flip_at   = base + app_start;
			if(crc_span > 0 && flip_at >= 0 && flip_at < base + crc_span)
			{
				payload[(size_t)flip_at] ^= 0xFF;   // corrupt one app byte (wrong on delivery)
				// re-stamp THIS codeword's per-cw CRC-8 over the now-corrupt bytes so the
				// per-cw demote PASSES (the false-pass). Block-CRC over the assembled payload
				// will NOT match the TX's clean-payload CRC-32 -> the block must be rejected.
				if(crc_off >= 0 && crc_off < (int)((long)K*sub_len))
					payload[(size_t)crc_off] =
						(unsigned char)CRC8_calc((char*)&payload[(size_t)base], crc_span);
				printf("[BIGBLOCK-RX] FALSEPASS-INJECT cw=%d: corrupted app byte @%d + re-stamped "
					"per-cw CRC-8 (per-cw gate will PASS; only the block CRC-32 can catch this)\n",
					fp_cw, flip_at);
				fflush(stdout);
			}
		}
	}

	// --- FAILURE-2 fix: per-codeword WIRE CRC-8 verify -> cw_ok demote --------------
	// The PHY-layer cw_ok producer (bigblock_rx_passband) is an ORACLE compare on the
	// single-instance loopback (cw_info_ref) and is FORCED CLEAN (all 1s) on the live
	// 2-instance path (ref==NULL) — so a MISCORRECTED codeword arrived "clean" and was
	// never retransmitted (silent corruption). Recompute the on-wire CRC-8 over each
	// codeword's first BIGBLOCK_CW_CRC_SPAN(sub_len) de-whitened bytes and compare to its
	// tail CRC byte. This is the REAL per-codeword detector; it can only DEMOTE cw_ok[c]
	// (a CRC failure clears it), never PROMOTE a genuine bit-mismatch the oracle caught.
	// It runs BEFORE the cw0 header-trust gate below, so a CRC-failed cw0 (cw_ok[0]==0)
	// forces the fallback-bsi path and is NOT length-table-parsed (§5 cross-layer). Skip
	// only when there are no decoded bytes (total acquisition miss: payload all-zero).
	//
	// REPRODUCER HOOK (fact-doc §13.R, mirrors the heap-fix MERCURY_BIGBLOCK_OLDGATE): set
	// MERCURY_BIGBLOCK_NOCRC=1 to DISABLE the wire-CRC demote on the SAME binary — this
	// restores the PRE-FIX silent-corruption behavior (the PHY producer's forced-clean
	// cw_ok stands) so the bit-flip-cw_ok test (CASE D) can show its fail-before without a
	// revert build. Production never sets it; it exists purely for the local fail-before
	// / pass-after proof.
	bool nocrc_demote = false;
	{ const char* e = std::getenv("MERCURY_BIGBLOCK_NOCRC"); if(e && *e && atoi(e)!=0) nocrc_demote = true; }
	if(info_bits != NULL && !nocrc_demote)
	{
		int crc_span = BIGBLOCK_CW_CRC_SPAN(sub_len);
		for(int c=0;c<K;c++)
		{
			if(crc_span < 0) break;
			int crc_off = BIGBLOCK_CW_CRC_OFFSET(c, sub_len);
			if(crc_off < 0 || crc_off >= (int)((long)K*sub_len)) { cw_ok[c] = 0; continue; }
			unsigned char calc = CRC8_calc(
				(char*)&payload[(size_t)c*sub_len], crc_span);
			unsigned char wire = payload[(size_t)crc_off];
			if(calc != wire) cw_ok[c] = 0;   // demote-only: CRC mismatch => failed codeword
		}
	}

	// --- D2_BLOCKCRC: WHOLE-BLOCK CRC-32 VERIFY -> reject (route to PARTIAL/SACK) ------
	// The block-level integrity anchor (datalink_defines.h BIGBLOCK_BLOCK_CRC_*) stacked ON
	// TOP of the per-codeword CRC-8. The HW NO-GO (WINRUN_FINAL_VERDICT.json) showed all 8
	// per-cw CRC-8 false-passing a corrupt K=8 block -> 1374 wrong bytes delivered. This gate
	// is the SAFETY NET for exactly that case: it fires ONLY when the per-cw layer believes the
	// block is FULLY CLEAN (n_clean == K) — i.e. it would otherwise DELIVER. When the per-cw
	// CRC-8 already demoted >=1 codeword the block is going PARTIAL regardless, the gap codeword
	// is re-sent + re-validated, and re-checking the (expected-mismatching) block-CRC here would
	// only DEFEAT the per-codeword selective-repeat granularity (CASE D / SACK) — so we skip it.
	// When n_clean == K, recompute the CRC-32 over the assembled DE-WHITENED payload (with the 4
	// block-CRC bytes zeroed, the SAME placeholder the TX used so there is no self-reference) and
	// compare to cw0's header field. On MISMATCH the "all-clean" verdict is FALSE: the block is
	// corrupt despite all per-cw gates passing, so it MUST NOT be delivered. Clear ALL cw_ok ->
	// n_clean=0 < K -> bigblock_block_to_arq routes the WHOLE block to the EXISTING PARTIAL/SACK
	// gap path (re-send), never copy_data_to_buffer. We clear all codewords because a 32-bit block
	// check localizes nothing — a block-wide false-clean means the whole assembled payload is
	// suspect, and re-sending the entire block is the byte-faithful action for a life-critical
	// modem. Skip on a total acquisition miss (info_bits==NULL) and when the geometry cannot hold
	// the field (degenerate sub_len). §1.5 producer/consumer audit: see the function-header block.
	// REPRODUCER HOOK (mirrors §17 MERCURY_BIGBLOCK_DEFEAT_FIX / D2 DEFEAT_D2): set
	// MERCURY_BIGBLOCK_DEFEAT_BLOCKCRC=1 to DISABLE the block-CRC reject on the SAME binary,
	// restoring the PRE-FIX silent wrong-byte delivery (the per-cw false-pass stands) so the
	// FALSEPASS fail-before is provable without a revert build. Production never sets it.
	bool defeat_blockcrc = false;
	{ const char* e = std::getenv("MERCURY_BIGBLOCK_DEFEAT_BLOCKCRC"); if(e && *e && atoi(e)!=0) defeat_blockcrc = true; }
	int n_clean_precheck = 0;
	for(int c=0;c<K;c++) if(cw_ok[c]) n_clean_precheck++;
	if(info_bits != NULL && !defeat_blockcrc && n_clean_precheck == K)
	{
		long total   = (long)K * sub_len;
		long bcrc_off = BIGBLOCK_BLOCK_CRC_OFFSET(K, sub_len);   // cw(K-1) trailer, before its CRC-8
		if(bcrc_off >= 0 && bcrc_off + BIGBLOCK_BLOCK_CRC_BYTES <= total)
		{
			// read the wire CRC-32 (uint32 LE) then build the SAME zeroed image the TX computed
			// over: zero the 4 block-CRC field bytes AND all K per-codeword CRC-8 tail bytes
			// (the TX computed the block-CRC before either was written = both 0). This decouples
			// the two CRC layers with no circular dependency.
			uint32_t wire_bcrc = 0;
			for(int b=0;b<BIGBLOCK_BLOCK_CRC_BYTES;b++)
				wire_bcrc |= ((uint32_t)payload[(size_t)bcrc_off + b]) << (8*b);
			std::vector<unsigned char> chk(payload.begin(), payload.begin() + total);
			for(int b=0;b<BIGBLOCK_BLOCK_CRC_BYTES;b++) chk[(size_t)bcrc_off + b] = 0;
			for(int c=0;c<K;c++){
				int cwc_off = BIGBLOCK_CW_CRC_OFFSET(c, sub_len);
				if(cwc_off >= 0 && cwc_off < total) chk[(size_t)cwc_off] = 0;
			}
			uint32_t calc_bcrc = CRC32_calc((char*)chk.data(), (int)total);
			if(calc_bcrc != wire_bcrc)
			{
				printf("[BIGBLOCK-RX] BLOCK-CRC MISMATCH (calc=%08x wire=%08x) — all %d per-cw "
					"CRC-8 FALSE-PASSED a corrupt K=%d block; REJECTING (clear all cw_ok -> "
					"PARTIAL/SACK re-send, NOT delivered)\n",
					(unsigned)calc_bcrc, (unsigned)wire_bcrc, K, K);
				fflush(stdout);
				for(int c=0;c<K;c++) cw_ok[c] = 0;   // force PARTIAL: never deliver a block-CRC-failed block
			}
		}
	}

	// --- PHASE 1: parse the cw0 wire header [bsi, n_data, length[0..K-1] uint16 LE] ---
	unsigned char block_bsi = fallback_bsi;
	int cw0_offset = 0;
	// V3 FIX (fact-doc §10/§14): the REAL decoded n_data (cw0 header byte payload[1], the count of
	// filled codewords the TX emitted). Parsed here when cw0 is clean and STASHED at the FIX-2 arm
	// site so bigblock_partial_block_crc_ok() rebuilds the header byte with the actual value
	// (NOT a hard-coded K) — an UNDER-FILLED clean block (n_data<K) would otherwise false-reject
	// forever (LIVELOCK). -1 == unset (no usable header) -> consumer falls back to K.
	int wire_n_data = -1;
	const int* sub_lengths_ptr = nullptr;
	std::vector<int> wire_lengths;
	const int hdr_total = BIGBLOCK_HDR_TOTAL_BYTES(K);
	bool header_usable = use_wire_header && info_bits != NULL
		&& hdr_total <= sub_len;            // header must fit in cw0
	if(header_usable && cw_ok[0])
	{
		// cw0 (which carries the header) must have decoded clean for the header to be
		// trusted. The wire bsi is authoritative (drift-proof); the length table sizes
		// each delivered slot. cw0's app bytes start AFTER the header prefix.
		block_bsi = payload[0];
		// payload[1] = n_data (filled-codeword count); clamp to [1,K] defensively (the TX
		// emits n_data in [1,K], bigblock_pack_block() :3921-3922). hdr_total>=2 always (it
		// is BIGBLOCK_HDR_FIXED_BYTES=2 + 2K) and hdr_total<=sub_len is checked above, so
		// payload[1] is in-bounds whenever the header is usable.
		wire_n_data = (int)(unsigned char)payload[1];
		if(wire_n_data < 1) wire_n_data = 1;
		if(wire_n_data > K) wire_n_data = K;
		wire_lengths.assign((size_t)K, 0);
		for(int c=0;c<K;c++)
		{
			int lo = BIGBLOCK_HDR_FIXED_BYTES + 2*c;
			int len = (int)payload[lo] | ((int)payload[lo+1] << 8);
			// FAILURE-2 fix: the codeword's tail byte is the CRC, NOT app data — reserve it
			// so the delivered length can never include the CRC byte (matches the TX caps
			// cw0_cap/cwc_cap above). cw0 also reserves the header prefix.
			// V2 FIX-1 (LIVELOCK): the LAST codeword (c==K-1) ALSO carries the 4-byte whole-block
			// CRC-32 field in its trailer (BIGBLOCK_BLOCK_CRC_OFFSET = cw(K-1)-local [sub_len-1-4 ..
			// sub_len-1-1]), so its app capacity reserves BIGBLOCK_BLOCK_CRC_BYTES too. Mirrors the
			// TX cap in bigblock_pack_block(); without it a genuinely-clean full block delivers a
			// wire length that runs into the block-CRC field and the block-CRC zeroing diverges TX
			// vs RX -> deterministic false-reject livelock (fact-doc §9).
			int cap = (c == 0) ? (sub_len - hdr_total - BIGBLOCK_CW_CRC_BYTES)
			          : (c == K - 1) ? (sub_len - BIGBLOCK_CW_CRC_BYTES - BIGBLOCK_BLOCK_CRC_BYTES)
			                         : (sub_len - BIGBLOCK_CW_CRC_BYTES);
			if(cap < 0)   cap = 0;
			if(len < 0)   len = 0;
			if(len > cap) len = cap;       // never deliver past a codeword's app capacity
			wire_lengths[c] = len;
		}
		cw0_offset      = hdr_total;
		sub_lengths_ptr = wire_lengths.data();
	}
	else
	{
		// FALLBACK (legacy / CRC-failed-cw0 / no-header): we cannot trust the length
		// table, but the codeword tail byte is STILL the CRC and must not be delivered as
		// app data. Pass an explicit uniform table = the per-codeword capacity minus the
		// CRC byte (cw0 also minus the header prefix) so the CRC byte stays out of the
		// delivered payload. cw0_offset stays 0 (legacy: cw0 had no header prefix on the
		// no-header path; on the CRC-failed-cw0 path cw0 is a SACK gap anyway, so its
		// length is unused). This only fires off the production live path when cw0's CRC
		// fails — the block is mostly re-requested, so exactness here is not load-bearing.
		wire_lengths.assign((size_t)K, 0);
		for(int c=0;c<K;c++)
		{
			int cap = sub_len - BIGBLOCK_CW_CRC_BYTES;
			if(cap < 0) cap = 0;
			wire_lengths[c] = cap;
		}
		sub_lengths_ptr = wire_lengths.data();
	}

	// --- V2 FIX-2 (fact-doc §10): stash the assembled-block integrity context for the
	// PARTIAL / SACK-completed delivery gate. The full-clean block-CRC gate above only
	// fires when n_clean==K; a KEPT codeword that the per-cw CRC-8 FALSE-PASSES inside a
	// PARTIAL block is otherwise delivered at the prev-batch completion (arq_responder.cc)
	// with NO block-CRC check (the §5 residual). We stash the TX block-CRC-32 value + the
	// geometry HERE so the completion can reassemble the K-codeword image from the prev
	// slots and re-verify the SAME CRC-32 before delivering. ARM only when:
	//   (a) this is a real header-bearing big-block carve (header_usable + cw0 clean),
	//   (b) the block is PARTIAL (n_clean < K) — the only path that reaches the prev
	//       completion; a clean block (n_clean==K) is already gated above and delivers now,
	//   (c) cw(K-1) decoded CRC-8-clean — it carries the block-CRC field (its CRC-8 span
	//       covers the field), so the stashed value is trustworthy; if cw(K-1) is itself the
	//       gap the field is untrustworthy AND never recovered (the app-only retx omits it),
	//       so we do NOT arm — that one codeword stays at the unchanged per-cw CRC-8 floor,
	//       and NO false reject of a genuinely-clean completion is ever introduced.
	// Reset to disarmed first (a clean carve / non-armable PARTIAL clears any stale stash).
	bigblock_partial_armed  = false;
	bigblock_partial_n_data = -1;   // V3: clear stale n_data with the rest of the stash
	// C6 MEASURE-ONLY: clear the cw(K-1)-gap residual-exposure one-shot on EVERY carve entry
	// (sibling lifecycle of the FIX-2 stash). Set below only when the residual pattern holds.
	bigblock_residual_armed = false;
	{
		int n_clean_stash = 0;
		for(int c=0;c<K;c++) if(cw_ok[c]) n_clean_stash++;
		long total_sl = (long)K * sub_len;
		long bcrc_off = BIGBLOCK_BLOCK_CRC_OFFSET(K, sub_len);
		bool cwlast_clean = (K-1 >= 0 && K-1 < (int)cw_ok.size()) ? (cw_ok[K-1] != 0) : false;
		if(info_bits != NULL && header_usable && cw_ok[0]
		   && n_clean_stash < K && cwlast_clean
		   && bcrc_off >= 0 && bcrc_off + BIGBLOCK_BLOCK_CRC_BYTES <= total_sl)
		{
			unsigned int wire_bcrc = 0;
			for(int b=0;b<BIGBLOCK_BLOCK_CRC_BYTES;b++)
				wire_bcrc |= ((unsigned int)payload[(size_t)bcrc_off + b]) << (8*b);
			bigblock_partial_block_bsi      = (int)(unsigned char)block_bsi;
			bigblock_partial_K              = K;
			bigblock_partial_sub_len        = sub_len;
			bigblock_partial_hdr_total      = hdr_total;
			bigblock_partial_cw0_offset     = cw0_offset;
			bigblock_partial_expected_crc32 = wire_bcrc;
			// V3 FIX (fact-doc §10/§14): stash the REAL decoded n_data (parsed above when cw0 is
			// clean — a FIX-2 arm precondition, so wire_n_data is always set here). The completion
			// gate reconstructs cw0's header byte img[1] with THIS value, matching the TX exactly,
			// so a genuinely-clean UNDER-FILLED (n_data<K) block reassembles byte-identical -> CRC
			// MATCH -> delivers, instead of false-rejecting forever (the §10 livelock).
			bigblock_partial_n_data         = (wire_n_data >= 1 && wire_n_data <= K) ? wire_n_data : K;
			bigblock_partial_lengths.assign((size_t)K, 0);
			for(int c=0;c<K;c++)
				bigblock_partial_lengths[(size_t)c] = sub_lengths_ptr ? sub_lengths_ptr[c] : 0;
			bigblock_partial_armed = true;
			printf("[BIGBLOCK-RX] FIX-2: armed PARTIAL block-CRC gate bsi=%d K=%d n_data=%d sub_len=%d "
				"expected_crc32=%08x (verified at prev-batch completion before delivery)\n",
				bigblock_partial_block_bsi, K, bigblock_partial_n_data, sub_len, wire_bcrc);
			fflush(stdout);
		}
	}

	// --- C6 MEASURE-ONLY (block-crc-upgrade-design.md §7 [?], bigblock-integrity.md §5/§12):
	// detect the cw(K-1)-gap PARTIAL block-CRC RESIDUAL-EXPOSURE pattern and arm a one-shot
	// signal for the prev-batch completion to COUNT (it never refuses or alters delivery).
	// The FIX-2 PARTIAL gate above arms ONLY when cw(K-1) decoded clean (it carries the
	// block-CRC-32 field); when cw(K-1) is ITSELF the gap the block is delivered with NO
	// whole-block CRC-32 re-verify, so any OTHER kept codeword that FALSE-PASSED its per-cw
	// CRC-8 rides the unchanged 2^-8 floor. We arm the residual one-shot when, on a real
	// header-bearing PARTIAL carve (header_usable + cw0 clean), cw(K-1) is the gap (cw_ok[K-1]==0)
	// AND there is >=1 OTHER kept codeword (cw_ok[c]==1, c != K-1) that this carve will deliver.
	// The prev-batch completion increments bigblock_partial_crc_residual_count + logs
	// [PARTIAL-CRC-RESIDUAL] when such a block is actually DELIVERED (counting here would
	// over-count blocks that never complete). NO delivery behaviour change; per-cw CRC-8
	// UNWEAKENED. Mutually exclusive with the FIX-2 arm above (that requires cwlast_clean).
	{
		// REPRODUCER HOOK (mirrors DEFEAT_BLOCKCRC / DEFEAT_NDATA): set
		// MERCURY_BIGBLOCK_DEFEAT_C6RESIDUAL=1 to SUPPRESS the residual-exposure arming on the
		// SAME binary, so the prev-batch completion never increments the counter — the test's
		// FAIL-BEFORE (counter stays 0). Production never sets it. MEASURE-ONLY hook: disabling
		// it changes nothing about delivery (the carve/delivery behaviour is identical either way;
		// only the diagnostic counter is suppressed).
		bool defeat_c6 = false;
		{ const char* e = std::getenv("MERCURY_BIGBLOCK_DEFEAT_C6RESIDUAL");
		  if(e && *e && atoi(e)!=0) defeat_c6 = true; }
		int n_clean_resid = 0;
		for(int c=0;c<K;c++) if(cw_ok[c]) n_clean_resid++;
		bool cwlast_gap = (K-1 >= 0 && K-1 < (int)cw_ok.size()) ? (cw_ok[K-1] == 0) : false;
		int other_kept = n_clean_resid - (cwlast_gap ? 0 : 1);   // kept slots besides cw(K-1)
		if(!defeat_c6 && info_bits != NULL && header_usable && cw_ok[0]
		   && n_clean_resid < K && cwlast_gap && other_kept >= 1)
		{
			bigblock_residual_armed     = true;
			bigblock_residual_block_bsi = (int)(unsigned char)block_bsi;
			bigblock_residual_kept_slots = other_kept;
			printf("[BIGBLOCK-RX] C6: armed cw(K-1)-gap residual-exposure measure bsi=%d K=%d "
				"n_clean=%d other_kept=%d (cw(K-1) is the gap -> block-CRC-32 NOT armable; "
				"%d kept slot(s) ride the per-cw CRC-8 floor; counted at delivery)\n",
				bigblock_residual_block_bsi, K, n_clean_resid, other_kept, other_kept);
			fflush(stdout);
		}
	}

	int rc = bigblock_block_to_arq(cw_ok.data(), K, block_bsi, payload.data(), sub_len,
		sub_lengths_ptr, cw0_offset);
	// HONEST DIAGNOSTIC (D1 fix, fact-doc bigblock-delivery-handoff §2): report the REAL
	// per-codeword clean count (n_clean after the wire-CRC-8 demote above), NOT the PHY
	// forced ORACLE bigblock_last_rx_cw_ok_count. On the live 2-instance RX (cw_info_ref==NULL)
	// the oracle is FORCED all-1s (telecom_system.cc:8067-8070), so it printed cw_ok_count=8
	// even when the carve routed PARTIAL clean<K — a misleading log that hid D2 on HW. n_clean
	// is the count of demoted-clean codewords actually delivered/SACKed by the carve.
	int n_clean_real = 0;
	for(int c=0;c<K;c++) if(cw_ok[c]) n_clean_real++;
	printf("[BIGBLOCK-RX] carve: K=%d n_clean=%d (oracle_cw_ok_count=%d) wire_bsi=%u "
		"(fallback=%u used_hdr=%d) sub_len=%d rc=%d\n",
		K, n_clean_real, telecom_system->bigblock_last_rx_cw_ok_count, (unsigned)block_bsi,
		(unsigned)fallback_bsi, (int)(sub_lengths_ptr != nullptr), sub_len, rc);
	fflush(stdout);
	return rc;
}

// V2 FIX-2 (fact-doc §10): re-verify the stashed whole-block CRC-32 over the K-codeword
// payload REASSEMBLED from messages_rx_prev[0..K-1] at the prev-batch completion, BEFORE
// the completion delivers via copy_data_to_buffer (arq_responder.cc). The codeword-aligned
// wire image (per-cw CRC tails + the block-CRC field) was discarded by
// bump_bsi_and_transfer_prev (it transfers only per-frame app bytes), so we reconstruct the
// SAME image the TX computed the CRC-32 over: cw0 = [reconstructed header | app | pad];
// cwc (c>=1) = [app | pad]; ALL per-cw CRC-8 tail bytes AND the 4 block-CRC field bytes
// ZEROED (the TX computed the block-CRC with both sets still 0, the RX recompute zeroes them).
// A KEPT codeword that the per-cw CRC-8 FALSE-PASSED carries WRONG app bytes -> the reassembled
// image differs from the TX image -> CRC-32 differs from the stashed expected -> return false
// (do NOT deliver -> re-request). Returns true (deliver) when the block is byte-consistent.
// Caller guards on bigblock_partial_armed + bsi match.
bool cl_arq_controller::bigblock_partial_block_crc_ok()
{
	int K       = bigblock_partial_K;
	int sub_len = bigblock_partial_sub_len;
	int hdr_total   = bigblock_partial_hdr_total;
	int cw0_offset  = bigblock_partial_cw0_offset;
	if(K <= 0 || sub_len <= 0) return true;            // nothing to check -> do not block delivery
	long total = (long)K * (long)sub_len;
	long bcrc_off = BIGBLOCK_BLOCK_CRC_OFFSET(K, sub_len);
	if(bcrc_off < 0 || bcrc_off + BIGBLOCK_BLOCK_CRC_BYTES > total) return true;  // degenerate -> skip

	std::vector<unsigned char> img((size_t)total, 0);   // codeword-aligned, zero pad

	// Reconstruct cw0's wire header [bsi, n_data, length[0..K-1] uint16 LE]. The header bytes
	// are part of the CRC-32 image, so they must match the TX exactly; the TX wrote
	// [bsi][n_data][lengths] where n_data = the batch's filled-codeword count.
	// V3 FIX (fact-doc §10/§14): use the REAL decoded n_data stashed at the carve
	// (bigblock_partial_n_data, = cw0 header byte payload[1]). The PRE-V3 code hard-coded
	// img[1]=K, which is correct ONLY for a full K-frame batch; an UNDER-FILLED (n_data<K)
	// block — a FIFO-drained / end-of-document tick, near-certain on a finite document's final
	// CFG16 tick — then reassembled with img[1]=K != TX n_data, CRC-MISMATCHED, REJECTED, and
	// re-emitted byte-identical -> re-rejected FOREVER (a deterministic false-reject LIVELOCK on a
	// genuinely-clean block, the §10 blocker). Falling back to K when n_data is unset (-1) is
	// safe: arming requires a clean cw0 header, so the value is always parsed when armed.
	int img_ndata = (bigblock_partial_n_data >= 1 && bigblock_partial_n_data <= K)
	                ? bigblock_partial_n_data : K;
	// REPRODUCER HOOK (fact-doc §14, mirrors DEFEAT_CAPFIX / DEFEAT_PARTIALCRC): set
	// MERCURY_BIGBLOCK_DEFEAT_NDATA=1 to restore the PRE-V3 hard-coded img[1]=K on the SAME
	// binary, so the n_data<K false-reject LIVELOCK fail-before is reproducible without a
	// revert build. With it set, a genuinely-clean UNDER-FILLED block reassembles with
	// img[1]=K != TX n_data -> CRC MISMATCH -> REJECT (the livelock). Production never sets it.
	{ const char* e = std::getenv("MERCURY_BIGBLOCK_DEFEAT_NDATA");
	  if(e && *e && atoi(e)!=0) img_ndata = K; }
	if(hdr_total >= 2 && hdr_total <= sub_len)
	{
		img[0] = (unsigned char)(bigblock_partial_block_bsi & 0xFF);
		img[1] = (unsigned char)(img_ndata & 0xFF);
		for(int c=0;c<K;c++)
		{
			int lo = BIGBLOCK_HDR_FIXED_BYTES + 2*c;
			if(lo + 1 >= sub_len) break;                // header must fit cw0
			int L = (c < (int)bigblock_partial_lengths.size()) ? bigblock_partial_lengths[(size_t)c] : 0;
			img[(size_t)lo + 0] = (unsigned char)(L & 0xFF);
			img[(size_t)lo + 1] = (unsigned char)((L >> 8) & 0xFF);
		}
	}

	// Place each codeword's delivered app bytes from messages_rx_prev[] at its base offset.
	for(int c=0;c<K && c<this->nMessages; c++)
	{
		int base = (c == 0) ? cw0_offset : (c * sub_len);
		int L    = (c < (int)bigblock_partial_lengths.size()) ? bigblock_partial_lengths[(size_t)c] : 0;
		if(L < 0) L = 0;
		int avail = messages_rx_prev[c].length;
		if(avail < 0) avail = 0;
		int n = (L < avail) ? L : avail;               // deliver-exact: clamp to the stashed wire length
		for(int j=0;j<n && (base + j) < total; j++)
			img[(size_t)base + j] = (unsigned char)messages_rx_prev[c].data[j];
		// bytes [base+n .. base+L) stay 0 (pad) — matches the TX's zero-pad image.
	}

	// Zero the per-cw CRC-8 tails + the block-CRC field (the SAME placeholders the TX used).
	for(int c=0;c<K;c++)
	{
		long off = BIGBLOCK_CW_CRC_OFFSET(c, sub_len);
		if(off >= 0 && off < total) img[(size_t)off] = 0;
	}
	for(int b=0;b<BIGBLOCK_BLOCK_CRC_BYTES;b++) img[(size_t)bcrc_off + b] = 0;

	unsigned int calc = (unsigned int)CRC32_calc((char*)img.data(), (int)total);
	bool ok = (calc == bigblock_partial_expected_crc32);
	printf("[BIGBLOCK-RX] FIX-2: prev-batch completion block-CRC %s (calc=%08x expected=%08x "
		"bsi=%d K=%d n_data=%d) %s\n",
		ok ? "MATCH" : "MISMATCH", calc, bigblock_partial_expected_crc32,
		bigblock_partial_block_bsi, K, img_ndata,
		ok ? "-> deliver" : "-> REJECT (a kept codeword FALSE-PASSED per-cw CRC-8; not delivered)");
	fflush(stdout);
	return ok;
}

// C6 MEASURE-ONLY (block-crc-upgrade-design.md §7 [?], bigblock-integrity.md §5/§12): count a
// DELIVERED PARTIAL big-block whose cw(K-1) was in the gap — the sub-case where the whole-block
// CRC-32 (FIX-2) could NOT arm (cw(K-1) carries the CRC-32 field at BIGBLOCK_BLOCK_CRC_OFFSET, so
// when it is itself the gap the field is never recovered and the carve's `cwlast_clean` arm
// precondition is FALSE). Such a block is delivered WITHOUT the block-CRC-32 net, so any OTHER kept
// codeword that FALSE-PASSED its per-cw CRC-8 rides the unchanged 2^-8 floor — the residual the
// CRC-32 exists to eliminate, which §7 asks to QUANTIFY before deciding the CLOSE (refuse-deliver /
// second-CRC). The carve arms `bigblock_residual_armed` (one-shot, bsi-tagged) when the pattern
// holds; this helper, called from the prev-batch completion AFTER delivery, increments the counter
// and logs [PARTIAL-CRC-RESIDUAL] when the delivered bsi matches. MEASURE-ONLY: it changes NO
// delivery behaviour, never weakens the per-cw CRC-8, and refuses nothing — the block was already
// delivered by the caller. Big-block-scoped + bsi-gated, so a non-big-block / non-residual prev
// completion is byte-identical to before (the armed flag is false or the bsi mismatches). One-shot:
// the armed flag is cleared here regardless, so a stale arm cannot re-count a later block.
void cl_arq_controller::note_bigblock_partial_crc_residual(int delivered_bsi)
{
	if(bigblock_residual_armed && bigblock_residual_block_bsi == delivered_bsi)
	{
		bigblock_partial_crc_residual_count++;
		printf("[PARTIAL-CRC-RESIDUAL] prev_batch_seq_id=%d DELIVERED with cw(K-1) in the gap -> "
			"whole-block CRC-32 NOT armable (cw(K-1) carries the CRC-32 field); %d other kept "
			"codeword(s) delivered at the per-cw CRC-8 2^-8 floor (residual_count=%lld) -- "
			"MEASURE-ONLY, delivery unchanged\n",
			delivered_bsi, bigblock_residual_kept_slots, bigblock_partial_crc_residual_count);
		fflush(stdout);
	}
	bigblock_residual_armed = false;   // one-shot: consumed at this completion (armed or not)
}


void cl_arq_controller::send(st_message* message, int message_location)
{
	printf("send()\n");

	int header_length=0;
	if(message->type==DATA_LONG)
	{
		// SACK Design A Steps 1+3 — DATA_LONG header growth + batch_seq_id plumbing.
		// v1 (default): 4 bytes [type, conn_id, seq(EOB bit7), id].
		// v2: 5 bytes [type, conn_id, seq(EOB bit7), batch_seq_id, id].
		// batch_seq_id source (v2 only): message->batch_seq_id field, masked mod 256.
		// Sentinel -1 (unset, defensive — should not happen for v2 DATA TX) → 0.
		message_TxRx_byte_buffer[0]=message->type;
		message_TxRx_byte_buffer[1]=connection_id;
		message_TxRx_byte_buffer[2]=message->sequence_number;
		if(sack_v2_enabled)
		{
			int bsi = message->batch_seq_id;
			if(bsi < 0) bsi = 0;  // defensive: caller should have assigned
			message_TxRx_byte_buffer[3]=(char)(bsi & 0xFF);
			message_TxRx_byte_buffer[4]=message->id;
		}
		else
		{
			message_TxRx_byte_buffer[3]=message->id;
		}
		header_length=effective_data_long_header_length(sack_v2_enabled);
	}
	else if (message->type==DATA_SHORT)
	{
		// SACK Design A Steps 2+3 — DATA_SHORT header growth + batch_seq_id plumbing.
		// v1 (default): 5 bytes [type, conn_id, seq(EOB bit7), id, length].
		// v2: 6 bytes [type, conn_id, seq(EOB bit7), batch_seq_id, id, length].
		message_TxRx_byte_buffer[0]=message->type;
		message_TxRx_byte_buffer[1]=connection_id;
		message_TxRx_byte_buffer[2]=message->sequence_number;
		if(sack_v2_enabled)
		{
			int bsi = message->batch_seq_id;
			if(bsi < 0) bsi = 0;
			message_TxRx_byte_buffer[3]=(char)(bsi & 0xFF);
			message_TxRx_byte_buffer[4]=message->id;
			message_TxRx_byte_buffer[5]=message->length;
		}
		else
		{
			message_TxRx_byte_buffer[3]=message->id;
			message_TxRx_byte_buffer[4]=message->length;
		}
		header_length=effective_data_short_header_length(sack_v2_enabled);
	}
	else if (message->type==ACK_RANGE || message->type==ACK_MULTI)
	{
		message_TxRx_byte_buffer[0]=message->type;
		message_TxRx_byte_buffer[1]=connection_id;
		message_TxRx_byte_buffer[2]=message->sequence_number;
		header_length=ACK_MULTI_ACK_RANGE_HEADER_LENGTH;
	}
	else if (message->type==SACK_RSP)
	{
		// SACK Design A Step 7 — OFDM SACK_RSP control frame. Same 3-byte
		// header as ACK_RANGE / ACK_MULTI / CONTROL: [type, conn_id, seq_num].
		// The payload (set by the caller in message->data, length tells how
		// many bytes total) is [batch_seq_id, bitmap..., CRC8]. We do NOT
		// touch the payload here — only the header bytes.
		message_TxRx_byte_buffer[0]=message->type;
		message_TxRx_byte_buffer[1]=connection_id;
		message_TxRx_byte_buffer[2]=message->sequence_number;
		header_length=ACK_MULTI_ACK_RANGE_HEADER_LENGTH;
	}
	else if (message->type==CONTROL || message->type==ACK_CONTROL)
	{
		message_TxRx_byte_buffer[0]=message->type;
		message_TxRx_byte_buffer[1]=connection_id;
		message_TxRx_byte_buffer[2]=message->sequence_number;
		header_length=CONTROL_ACK_CONTROL_HEADER_LENGTH;
	}

	for(int i=0;i<message->length;i++)
	{
		message_TxRx_byte_buffer[i+header_length]=message->data[i];
	}

	if(header_length>max_header_length)
	{
		std::cout<<"header size is too big, adjust the configuration parameters"<<std::endl;
		exit(0);
	}

	for(int i=0;i<(header_length+message->length);i++)
	{
		telecom_system->data_container.data_byte[i]=(int)(unsigned char)message_TxRx_byte_buffer[i];
	}

	// Bug #34 diagnostic: print TX frame bytes for CONTROL messages
	if(message->type == CONTROL || message->type == ACK_CONTROL)
	{
		int total = header_length + message->length;
		printf("[TX-CTRL] type=%d hdr=%d len=%d total=%d bytes:",
			message->type, header_length, message->length, total);
		for(int i = 0; i < total && i < 10; i++)
			printf(" %02x", telecom_system->data_container.data_byte[i] & 0xFF);
		printf("\n");
		fflush(stdout);
	}

	telecom_system->transmit_byte(telecom_system->data_container.data_byte,header_length+message->length,telecom_system->data_container.ready_to_transmit_passband_data_tx,message_location);

	{
		int active_nsymb = telecom_system->get_active_nsymb();
		tx_transfer(telecom_system->data_container.ready_to_transmit_passband_data_tx,
					telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate *
					(active_nsymb + telecom_system->data_container.preamble_nSymb));
	}

	drain_playback_wait();

	last_message_sent_type=message->type;
	if(message->type==CONTROL || message->type==ACK_CONTROL)
	{
		last_message_sent_code=message->data[0];
	}
	last_received_message_sequence=-1;

}

void cl_arq_controller::send_batch()
{
	if(passive_monitor) return;  // Never transmit in monitor mode
	// STEPPER-CORE REWRITE Phase b: mark the whole send_batch body as DATA-batch TX so the
	// outer-stepper TX-wait seams (drain_playback_wait / ptt_busy_wait) QUEUE-and-return
	// (per-symbol pacing) for the OFDM DATA path ONLY — NOT for the MFSK ACK patterns / control
	// the RSP sends on the same CFG16 (§10.4). RAII clears it at EVERY exit (the big-block
	// return + the normal end). No-op outside the outer stepper (the flag is only read there).
	struct DataBatchTxScope {
		DataBatchTxScope()  { arq_set_sim2_in_data_batch_tx(true);  }
		~DataBatchTxScope() { arq_set_sim2_in_data_batch_tx(false); }
	} _data_batch_tx_scope;
	// === DIAG: always print TX activity (remove after debug) ===
	printf("[CMD-TX] CONFIG_%d batch=%d type=%d pream=%d Nsymb=%d\n",
		current_configuration, message_batch_counter_tx,
		message_batch_counter_tx > 0 ? messages_batch_tx[0].type : -1,
		telecom_system->data_container.preamble_nSymb,
		telecom_system->data_container.Nsymb);
	fflush(stdout);
	if(g_verbose) {
		printf("[TX] send_batch() on CONFIG_%d, %d messages, first type=%d\n",
			current_configuration, message_batch_counter_tx,
			message_batch_counter_tx > 0 ? messages_batch_tx[0].type : -1);
		fflush(stdout);
	}

	// Flush capture buffer at the START of send_batch(), before TX begins.
	// On VB-Cable (and real radios), the responder decodes the frame and sends
	// its ACK pattern while the commander may still be draining playback or in
	// PTT-off delay. By flushing here, the buffer is clean BEFORE self-echo
	// starts, and the ACK pattern that arrives after the frame is preserved.
	// The order-aware ACK detector distinguishes ACK tones from OFDM self-echo.
	circular_buf_reset(capture_buffer);
	{
		int buf_samples = telecom_system->data_container.Nofdm * telecom_system->data_container.buffer_Nsymb * telecom_system->data_container.interpolation_rate;
		MUTEX_LOCK(&capture_prep_mutex);
		memset(telecom_system->data_container.passband_delayed_data, 0, 2 * buf_samples * sizeof(double));
		telecom_system->data_container.ring_write_index = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
	}
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.delay_of_last_decoded_message = -1;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;

	// rx_mute during TX: capture_prep_thread writes zeros to ring buffer,
	// so self-echo never enters. After playback drain, we unmute and flush
	// so the ACK arrives into a clean buffer. (Replaces Bug #38 post-TX
	// ring zero which destroyed early-arriving ACK audio.)
	telecom_system->data_container.rx_mute = 1;

	ptt_on();

	cl_timer ptt_on_delay, ptt_off_delay;
	ptt_on_delay.start();

	// STEP 2 — big-block live send path. When the CFG16-rung framing flag is set
	// (FORCED true for this validation; gearshift AUTO-election DEFERRED to P4),
	// emit the new-data batch as ONE big-block (one acquisition, K codewords) via
	// transmit_byte -> transmit_bigblock instead of the per-frame preamble loop.
	// Declines (returns false) for MFSK / retx / mixed-control batches, which fall
	// through to the stock per-frame path below (byte-identical when the flag is
	// off — the default). On a handled block, do the SAME post-TX bookkeeping the
	// per-frame path does (ptt-off-delay, capture flush + unmute, ack timers,
	// frames_to_read reset) and return.
	if(bigblock_send_one_block())
	{
		ptt_busy_wait(ptt_on_delay, ptt_on_delay_ms);

		if(g_verbose) { printf("[TX] big-block: waiting for playback buffer to drain...\n"); fflush(stdout); }
		drain_playback_wait();
		mtl::log_event("cmd_batch_last_sym_out");

		// Unmute + flush right after playback drain (mirrors the per-frame tail).
		circular_buf_reset(capture_buffer);
		{
			int buf_samples = telecom_system->data_container.Nofdm
				* telecom_system->data_container.buffer_Nsymb
				* telecom_system->data_container.interpolation_rate;
			MUTEX_LOCK(&capture_prep_mutex);
			memset(telecom_system->data_container.passband_delayed_data, 0,
				2 * buf_samples * sizeof(double));
			telecom_system->data_container.ring_write_index = 0;
			MUTEX_UNLOCK(&capture_prep_mutex);
		}
		mtl::log_event("cmd_ring_reset_done");
		telecom_system->data_container.rx_mute = 0;
		telecom_system->data_container.rx_mute_samples = 0;
		mtl::log_event("cmd_post_tx_unmute");

		ptt_off_delay.start();
		ptt_busy_wait(ptt_off_delay, ptt_off_delay_ms);
		ptt_off();
		mtl::log_event("cmd_ptt_off");

		// Post-TX bookkeeping: arm ack timers + PENDING_ACK on each DATA frame's
		// owning messages_tx slot (SAME as the per-frame path), then clear the
		// batch slots. The block is ONE batch -> the K frames PENDING_ACK on one ACK.
		for(int i=0;i<message_batch_counter_tx;i++)
		{
			if(messages_batch_tx[i].type==DATA_LONG || messages_batch_tx[i].type==DATA_SHORT)
			{
				int id = (int)(unsigned char)messages_batch_tx[i].id;
				messages_tx[id].ack_timer.start();
				messages_tx[id].status=PENDING_ACK;
				if(messages_tx[id].ack_timeout == 0)
					messages_tx[id].ack_timeout = ack_timeout_data;
				if(messages_tx[id].nResends == 0)
					messages_tx[id].nResends = nResends;
			}
			this->messages_batch_tx[i].ack_timeout=0;
			this->messages_batch_tx[i].id=0;
			this->messages_batch_tx[i].length=0;
			this->messages_batch_tx[i].nResends=0;
			this->messages_batch_tx[i].status=FREE;
			this->messages_batch_tx[i].type=NONE;
		}
		message_batch_counter_tx=0;

		telecom_system->data_container.frames_to_read =
			telecom_system->data_container.preamble_nSymb;
		printf("[TX-END-BIGBLOCK] frames_to_read=%d\n",
			telecom_system->data_container.frames_to_read.load());
		fflush(stdout);
		return;
	}

	int active_nsymb = telecom_system->get_active_nsymb();
	int frame_output_size = telecom_system->data_container.Nofdm*telecom_system->data_container.interpolation_rate*(active_nsymb+telecom_system->data_container.preamble_nSymb);

	double *batch_frames_output_data=NULL;
	double *batch_frames_output_data_filtered1=NULL;
	double *batch_frames_output_data_filtered2=NULL;

	int batch_alloc_count = (message_batch_counter_tx+2)*frame_output_size;
	batch_frames_output_data=new double[batch_alloc_count];
	batch_frames_output_data_filtered1=new double[batch_alloc_count];
	batch_frames_output_data_filtered2=new double[batch_alloc_count];

	if (batch_frames_output_data==NULL)
	{
		exit(-31);
	}
	if (batch_frames_output_data_filtered1==NULL)
	{
		exit(-32);
	}
	if (batch_frames_output_data_filtered2==NULL)
	{
		exit(-33);
	}

	int header_length=0;
	// LEVER P: preamble amortization. Frames are concatenated into ONE gapless
	// PTT waveform; with variable-length frames (anchor FULL + tail MINI) we pack
	// them CONTIGUOUSLY rather than at a fixed frame_output_size stride. Slot 0 is
	// the leading FIR pad (frame_output_size samples). Frame i is written at
	// frame_pack_off[i] (running offset starting at frame_output_size). frame_len[i]
	// is its actual emitted length (from tx_last_emitted_frame_samples). MFSK and
	// retx batches force every frame FULL so frame_len == frame_output_size and the
	// packing degenerates to the legacy fixed-stride layout (byte-identical).
	std::vector<int> frame_pack_off(message_batch_counter_tx, 0);
	std::vector<int> frame_len(message_batch_counter_tx, frame_output_size);
	int pack_cursor = frame_output_size;  // first frame begins after the lead pad
	const bool batch_force_full = sack_retransmit_active;  // retx => re-anchor every frame
	for(int i=0;i<message_batch_counter_tx;i++)
	{
		// SACK retransmit: sequence_number already set to original position
		// with end-of-batch flag. Don't override.
		if(!sack_retransmit_active)
		{
			messages_batch_tx[i].sequence_number=i;
			// Mark last DATA frame in batch with bit 7 so responder knows actual batch size.
			// Only for data frames — control frames must not set this flag.
			if(i == message_batch_counter_tx - 1
				&& (messages_batch_tx[i].type == DATA_LONG || messages_batch_tx[i].type == DATA_SHORT))
				messages_batch_tx[i].sequence_number |= 0x80;
		}

		header_length=0;

		if(messages_batch_tx[i].type==DATA_LONG)
		{
			// SACK Design A Steps 1+3 — DATA_LONG header growth + batch_seq_id plumbing.
			// See cl_arq_controller::send() for full format documentation.
			message_TxRx_byte_buffer[0]=messages_batch_tx[i].type;
			message_TxRx_byte_buffer[1]=connection_id;
			message_TxRx_byte_buffer[2]=messages_batch_tx[i].sequence_number;
			if(sack_v2_enabled)
			{
				int bsi = messages_batch_tx[i].batch_seq_id;
				if(bsi < 0) bsi = 0;  // defensive
				message_TxRx_byte_buffer[3]=(char)(bsi & 0xFF);
				message_TxRx_byte_buffer[4]=messages_batch_tx[i].id;
			}
			else
			{
				message_TxRx_byte_buffer[3]=messages_batch_tx[i].id;
			}
			header_length=effective_data_long_header_length(sack_v2_enabled);
		}
		else if (messages_batch_tx[i].type==DATA_SHORT)
		{
			// SACK Design A Steps 2+3 — DATA_SHORT header growth + batch_seq_id plumbing.
			// See cl_arq_controller::send() for full format documentation.
			message_TxRx_byte_buffer[0]=messages_batch_tx[i].type;
			message_TxRx_byte_buffer[1]=connection_id;
			message_TxRx_byte_buffer[2]=messages_batch_tx[i].sequence_number;
			if(sack_v2_enabled)
			{
				int bsi = messages_batch_tx[i].batch_seq_id;
				if(bsi < 0) bsi = 0;
				message_TxRx_byte_buffer[3]=(char)(bsi & 0xFF);
				message_TxRx_byte_buffer[4]=messages_batch_tx[i].id;
				message_TxRx_byte_buffer[5]=messages_batch_tx[i].length;
			}
			else
			{
				message_TxRx_byte_buffer[3]=messages_batch_tx[i].id;
				message_TxRx_byte_buffer[4]=messages_batch_tx[i].length;
			}
			header_length=effective_data_short_header_length(sack_v2_enabled);
		}
		else if (messages_batch_tx[i].type==ACK_RANGE || messages_batch_tx[i].type==ACK_MULTI)
		{
			message_TxRx_byte_buffer[0]=messages_batch_tx[i].type;
			message_TxRx_byte_buffer[1]=connection_id;
			message_TxRx_byte_buffer[2]=messages_batch_tx[i].sequence_number;
			header_length=ACK_MULTI_ACK_RANGE_HEADER_LENGTH;
		}
		else if (messages_batch_tx[i].type==SACK_RSP)
		{
			// SACK Design A Step 7 — OFDM SACK_RSP. 3-byte standard header;
			// payload [batch_seq_id, bitmap..., CRC8] is carried in
			// messages_batch_tx[i].data[..] verbatim. Symmetric with the send()
			// branch above.
			message_TxRx_byte_buffer[0]=messages_batch_tx[i].type;
			message_TxRx_byte_buffer[1]=connection_id;
			message_TxRx_byte_buffer[2]=messages_batch_tx[i].sequence_number;
			header_length=ACK_MULTI_ACK_RANGE_HEADER_LENGTH;
		}
		else if (messages_batch_tx[i].type==CONTROL || messages_batch_tx[i].type==ACK_CONTROL)
		{
			message_TxRx_byte_buffer[0]=messages_batch_tx[i].type;
			message_TxRx_byte_buffer[1]=connection_id;
			message_TxRx_byte_buffer[2]=messages_batch_tx[i].sequence_number;
			header_length=CONTROL_ACK_CONTROL_HEADER_LENGTH;
		}

		for(int j=0;j<messages_batch_tx[i].length;j++)
		{
			message_TxRx_byte_buffer[j+header_length]=messages_batch_tx[i].data[j];
		}

		if(header_length>max_header_length)
		{
			// §7.13.34 — was exit(0). Don't kill the whole modem on an
			// unexpected header size; log loudly and skip this frame so
			// the link can survive whatever miscalibrated the buffers.
			printf("[ERR-HDR-OVERFLOW] header_length=%d > max_header_length=%d (type=%d, sack_v2=%d) — skipping frame\n",
				header_length, max_header_length,
				(int)messages_batch_tx[i].type, sack_v2_enabled?1:0);
			fflush(stdout);
			continue;
		}

		for(int j=0;j<(header_length+messages_batch_tx[i].length);j++)
		{
			telecom_system->data_container.data_byte[j]=(int)(unsigned char)message_TxRx_byte_buffer[j];
		}

		if(g_verbose) {
			int total = header_length + messages_batch_tx[i].length;
			printf("[TX-BYTES] frame=%d type=%d connid=%d hdr=%d len=%d bytes:",
				i, messages_batch_tx[i].type, (int)(unsigned char)connection_id,
				header_length, messages_batch_tx[i].length);
			for(int j=0; j<total && j<12; j++)
				printf(" %02x", (unsigned char)message_TxRx_byte_buffer[j]);
			printf("\n");
			fflush(stdout);
		}

		// LEVER P: per-frame preamble schedule. Only DATA frames amortize; any
		// CONTROL/ACK frame mixed into the batch keeps the FULL preamble (force
		// full). The schedule is keyed on the in-batch frame index i and the
		// batch-wide force-full flag (retx). transmit_bit reads the override and
		// reports the actual emitted length via tx_last_emitted_frame_samples.
		bool frame_is_data = (messages_batch_tx[i].type==DATA_LONG || messages_batch_tx[i].type==DATA_SHORT);
		bool frame_force_full = batch_force_full || !frame_is_data;
		telecom_system->tx_preamble_nsymb_override =
			cl_telecom_system::preamble_sched_nsymb(i, frame_force_full,
				telecom_system->data_container.preamble_nSymb);

		frame_pack_off[i] = pack_cursor;
		telecom_system->transmit_byte(telecom_system->data_container.data_byte,header_length+messages_batch_tx[i].length,&batch_frames_output_data[pack_cursor],NO_FILTER_MESSAGE);
		telecom_system->tx_preamble_nsymb_override = -1;  // reset (defensive)

		frame_len[i] = telecom_system->tx_last_emitted_frame_samples;
		if(frame_len[i] <= 0 || frame_len[i] > frame_output_size)
			frame_len[i] = frame_output_size;  // defensive clamp
		pack_cursor += frame_len[i];


		last_message_sent_type=messages_batch_tx[i].type;
		if(messages_batch_tx[i].type==CONTROL || messages_batch_tx[i].type==ACK_CONTROL)
		{
			last_message_sent_code=messages_batch_tx[i].data[0];
		}
		last_received_message_sequence=-1;

	}

	// LEVER P: contiguous packed layout. The frames region is
	// [frame_output_size, pack_cursor). Lead pad (slot 0) = copy of the first
	// frame_output_size samples of frame 0; trail pad = copy of the last
	// frame_output_size samples of the final frame. This mirrors the legacy
	// edge-replication so the FIR transient never bites a real sample.
	int frames_region_len = pack_cursor - frame_output_size;  // sum of frame_len[i]
	for(int i=0;i<frame_output_size;i++) //padding start and end to prepare for filtering
	{
		// lead pad: replicate the first frame's leading samples
		batch_frames_output_data[i]=batch_frames_output_data[frame_output_size+i];
		// trail pad: replicate the final frame's trailing samples (the last
		// frame_output_size samples of the packed frames region)
		batch_frames_output_data[pack_cursor+i]=batch_frames_output_data[pack_cursor-frame_output_size+i];
	}

	{
		int total_fir_size = pack_cursor + frame_output_size;  // lead pad + frames + trail pad
		memset(batch_frames_output_data_filtered1, 0, total_fir_size * sizeof(double));
		memset(batch_frames_output_data_filtered2, 0, total_fir_size * sizeof(double));
		telecom_system->ofdm.FIR_tx1.apply(batch_frames_output_data,batch_frames_output_data_filtered1,total_fir_size);
		telecom_system->ofdm.FIR_tx2.apply(batch_frames_output_data_filtered1,batch_frames_output_data_filtered2,total_fir_size);

		// DIAG: TX peak amplitude after FIR filtering
		{
			double pk_pre = 0, pk_post = 0;
			for(int j = 0; j < total_fir_size; j++) {
				if(fabs(batch_frames_output_data[j]) > pk_pre) pk_pre = fabs(batch_frames_output_data[j]);
				if(fabs(batch_frames_output_data_filtered2[j]) > pk_post) pk_post = fabs(batch_frames_output_data_filtered2[j]);
			}
			printf("[TX-PEAK] pre_fir=%.4f post_fir=%.4f frames=%d size=%d cfg=%d\n",
				pk_pre, pk_post, message_batch_counter_tx, total_fir_size, current_configuration);
			fflush(stdout);
		}
	}

	// === TX SELF-TEST: verify matched filter template vs actual batch TX output ===
	// The first frame in the batch starts at offset frame_output_size in the filtered data
	// (position 0 is the padding copy). Preamble is at the start of the first frame.
	{
		static int batch_selftest_count = 0;
		static int batch_selftest_last_config = -1;
		if(batch_selftest_last_config != current_configuration) {
			batch_selftest_count = 0;
			batch_selftest_last_config = current_configuration;
		}
		if(batch_selftest_count < 1 && telecom_system->ofdm.ofdm_corr_template != NULL
			&& telecom_system->M != MOD_MFSK)
		{
			batch_selftest_count++;
			int interp = telecom_system->frequency_interpolation_rate;
			int Nofdm_l = telecom_system->data_container.Nofdm;
			int preamble_nsymb = telecom_system->data_container.preamble_nSymb;
			// First frame in batch is at offset frame_output_size (slot 1; slot 0 is padding)
			double* frame_pb = &batch_frames_output_data_filtered2[frame_output_size];
			int frame_len = frame_output_size;

			std::complex<double>* tx_bb = new std::complex<double>[frame_len];
			telecom_system->ofdm.passband_to_baseband(frame_pb, frame_len, tx_bb,
				telecom_system->sampling_frequency, telecom_system->carrier_frequency,
				telecom_system->carrier_amplitude, 1, &telecom_system->ofdm.FIR_rx_time_sync);

			int sym_interp = Nofdm_l * interp;
			printf("[TX-SELFTEST-BATCH] CONFIG_%d frames=%d frame_size=%d\n",
				current_configuration, message_batch_counter_tx, frame_output_size);
			double total_metric = 0;
			for(int k = 0; k < preamble_nsymb && k < telecom_system->ofdm.ofdm_corr_template_nsymb; k++)
			{
				int tmpl_off = k * Nofdm_l;
				int rx_off = k * sym_interp;
				double cr = 0, ci = 0, e_t = 0, e_r = 0;
				for(int n = 0; n < Nofdm_l; n++)
				{
					int rx_idx = rx_off + n * interp;
					if(rx_idx >= frame_len) break;
					std::complex<double> rx = tx_bb[rx_idx];
					double t_re = telecom_system->ofdm.ofdm_corr_template[tmpl_off + n].real();
					double t_im = telecom_system->ofdm.ofdm_corr_template[tmpl_off + n].imag();
					e_t += t_re*t_re + t_im*t_im;
					e_r += rx.real()*rx.real() + rx.imag()*rx.imag();
					cr += t_re*rx.real() + t_im*rx.imag();
					ci += t_im*rx.real() - t_re*rx.imag();
				}
				double cs = (e_t*e_r > 1e-30) ? (cr*cr + ci*ci) / (e_t*e_r) : 0;
				printf("  sym%d: cs=%.4f e_t=%.3f e_r=%.3f |corr|2=%.3f\n",
					k, cs, e_t, e_r, cr*cr+ci*ci);
				total_metric += cs;
			}
			printf("  total=%.4f (expect ~%.1f if template matches TX)\n",
				total_metric, (double)preamble_nsymb);
			printf("  tmpl[0..3]:");
			for(int n = 0; n < 4 && n < Nofdm_l; n++)
				printf(" (%.4f,%.4f)",
					telecom_system->ofdm.ofdm_corr_template[n].real(),
					telecom_system->ofdm.ofdm_corr_template[n].imag());
			printf("\n  tx_bb[0..3]:");
			for(int n = 0; n < 4; n++) {
				int idx = n * interp;
				if(idx < frame_len)
					printf(" (%.4f,%.4f)", tx_bb[idx].real(), tx_bb[idx].imag());
			}
			printf("\n");
			fflush(stdout);

			delete[] tx_bb;
		}
	}

	ptt_busy_wait(ptt_on_delay, ptt_on_delay_ms);

	// Generate pilot tone if enabled (configurable frequency to warm up TX/amp)
	if(pilot_tone_ms > 0 && pilot_tone_hz > 0)
	{
		const double SAMPLE_RATE = 48000.0;
		const double PILOT_FREQ = (double)pilot_tone_hz;
		const double PI = 3.14159265358979323846;
		int pilot_samples = (int)(pilot_tone_ms * SAMPLE_RATE / 1000.0);
		double* pilot_buffer = new double[pilot_samples];

		for(int i = 0; i < pilot_samples; i++)
		{
			// Generate sine wave with soft ramp up/down to avoid clicks
			double t = (double)i / SAMPLE_RATE;
			double envelope = 1.0;
			int ramp_samples = (int)(SAMPLE_RATE * 0.005); // 5ms ramp
			if(i < ramp_samples)
				envelope = (double)i / ramp_samples;
			else if(i > pilot_samples - ramp_samples)
				envelope = (double)(pilot_samples - i) / ramp_samples;

			pilot_buffer[i] = envelope * 0.5 * sin(2.0 * PI * PILOT_FREQ * t);
		}

		tx_transfer(pilot_buffer, pilot_samples);
		delete[] pilot_buffer;
	}

	// LEVER P: transmit each frame at its actual packed offset and actual length
	// (variable: anchor FULL, tail MINI). The frames are contiguous, so the wire
	// sees one gapless waveform; per-frame tx_transfer granularity is preserved
	// for the sim pacing / capture-prep symbol cadence.
	for(int i=0;i<message_batch_counter_tx;i++)
	{
		if(g_verbose) { printf("[TX] tx_transfer frame %d/%d, off=%d size=%d\n", i, message_batch_counter_tx, frame_pack_off[i], frame_len[i]); fflush(stdout); }
		tx_transfer(&batch_frames_output_data_filtered2[frame_pack_off[i]], frame_len[i]);
	}

	if(g_verbose) { printf("[TX] Waiting for playback buffer to drain...\n"); fflush(stdout); }
	// wait buffer to be played
	drain_playback_wait();

	// M1 (SACK turnaround trace): the true "last DATA audio sample left the
	// sound card" instant — the playback buffer just drained, and this is
	// BEFORE the capture ring reset. Anchors the CMD↔RSP overlap window.
	mtl::log_event("cmd_batch_last_sym_out");

	// Unmute + flush right after playback drain (before ptt_off_delay).
	// During TX, rx_mute=1 kept self-echo out of the ring.
	// Now unmute so the RSP's ACK can arrive during ptt_off_delay.
	// Safe: on real radio, still keyed during ptt_off_delay (no RX audio).
	// On VB-Cable, no self-echo (separate in/out cables).
	circular_buf_reset(capture_buffer);
	{
		int buf_samples = telecom_system->data_container.Nofdm
			* telecom_system->data_container.buffer_Nsymb
			* telecom_system->data_container.interpolation_rate;
		MUTEX_LOCK(&capture_prep_mutex);
		memset(telecom_system->data_container.passband_delayed_data, 0,
			2 * buf_samples * sizeof(double));
		telecom_system->data_container.ring_write_index = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
	}
	// M2 (SACK turnaround trace): post-TX capture ring reset complete, rx still
	// muted. The M2 -> cmd_post_tx_unmute delta is the CMD-side dead window
	// where freshly-captured RX audio would be clobbered by the ring reset.
	mtl::log_event("cmd_ring_reset_done");
	telecom_system->data_container.rx_mute = 0;
	telecom_system->data_container.rx_mute_samples = 0;
	mtl::log_event("cmd_post_tx_unmute");

	ptt_off_delay.start();
	ptt_busy_wait(ptt_off_delay, ptt_off_delay_ms);

	ptt_off();
	mtl::log_event("cmd_ptt_off");

	if (batch_frames_output_data!=NULL)
	{
		delete[] batch_frames_output_data;
		batch_frames_output_data=NULL;
	}
	if (batch_frames_output_data_filtered1!=NULL)
	{
		delete[] batch_frames_output_data_filtered1;
		batch_frames_output_data_filtered1=NULL;
	}
	if (batch_frames_output_data_filtered2!=NULL)
	{
		delete[] batch_frames_output_data_filtered2;
		batch_frames_output_data_filtered2=NULL;
	}

	for(int i=0;i<message_batch_counter_tx;i++)
	{
		if(messages_batch_tx[i].type==DATA_LONG || messages_batch_tx[i].type==DATA_SHORT)
		{
			// R030 (race audit 2026-06-06): resolve the messages_tx[] slot to flip
			// PENDING_ACK. messages_batch_tx[i].id is the WIRE id, which on a v2
			// MIXED batch is NOT the messages_tx[] array index:
			//   - retx-prefix frames (i < v2_retx_prefix_count): .id = the ORIGINAL
			//     wire slot of a PRIOR batch; their payload is in retx_scratch[] and
			//     their original messages_tx[] slot was freed to ACKED at SACK
			//     capture (arq_commander.cc:2983). They have NO live messages_tx[]
			//     slot in THIS batch — their delivery is tracked via the SACK
			//     bitmap / retx queue, NOT via a messages_tx[] PENDING_ACK. Flipping
			//     messages_tx[orig_wire_slot] aliased a FREE/foreign slot (the R030
			//     bug). SKIP them.
			//   - new-data frames (i >= v2_retx_prefix_count): .id = pos_in_new_batch
			//     (overwritten at arq_commander.cc:1645), NOT the array index. Find
			//     the owning slot by (batch_seq_id, low7-seq) — both were written to
			//     messages_tx[idx] at fill time (arq_commander.cc:1616/1644) and to
			//     messages_batch_tx[i] (send_batch's renumber is suppressed on mixed
			//     batches via sack_retransmit_active), so the tuple matches exactly.
			// On a v2 NON-mixed batch (v2_retx_prefix_count==0) and on v1, the wire
			// .id == messages_tx[] array index, so the legacy direct path is correct.
			int id = v2_flip_resolve_slot(i);
			if(id < 0)
				continue;  // retx-prefix (no slot) or no owning slot — see helper
			messages_tx[id].ack_timer.start();
			messages_tx[id].status=PENDING_ACK;
			// Ensure padded (duplicate) frames have valid timeout/resend
			// so they don't instantly expire (ack_timeout=0, nResends=0).
			if(messages_tx[id].ack_timeout == 0)
				messages_tx[id].ack_timeout = ack_timeout_data;
			if(messages_tx[id].nResends == 0)
				messages_tx[id].nResends = nResends;
		}
		if(messages_batch_tx[i].type==CONTROL)
		{
			messages_control.ack_timer.start();
			messages_control.status=PENDING_ACK;
		}

		this->messages_batch_tx[i].ack_timeout=0;
		this->messages_batch_tx[i].id=0;
		this->messages_batch_tx[i].length=0;
		this->messages_batch_tx[i].nResends=0;
		this->messages_batch_tx[i].status=FREE;
		this->messages_batch_tx[i].type=NONE;

	}
	message_batch_counter_tx=0;



	// Commander: after batch TX, set small ftr for quick ACK polling.
	// receive_ack_pattern() checks frames_to_read==0 before polling, so
	// a large ftr would delay the first ACK check by seconds, causing the
	// ACK pattern to scroll past the detection window.  preamble_nSymb (4)
	// gives a ~90ms initial delay, then receive_ack_pattern sets ftr=2.
	telecom_system->data_container.frames_to_read =
		telecom_system->data_container.preamble_nSymb;
	printf("[TX-END] frames_to_read=%d (ctrl=%d)\n", telecom_system->data_container.frames_to_read.load(), telecom_system->mfsk_ctrl_mode ? 1 : 0);
	fflush(stdout);
}

// Transmit short ACK tone pattern instead of LDPC-encoded ACK frame
void cl_arq_controller::send_ack_pattern()
{
	if(passive_monitor) return;
	cl_timer ack_turnaround_timer;
	ack_turnaround_timer.start();
	printf("[TX-ACK-PAT] Sending ACK pattern on CONFIG_%d at t=%dms\n", current_configuration, (int)ack_turnaround_timer.get_elapsed_time_ms()); fflush(stdout);
	mtl::log_event("rsp_ack_send_start");

	// Wait for the full OFDM frame to finish being received before
	// transmitting. With high-redundancy LDPC (e.g. CONFIG_0 rate 1/16),
	// the decoder converges before the frame is fully captured. Transmitting
	// prematurely on a half-duplex link collides with the incoming frame
	// and destroys subsequent audio via rx_mute + buffer flush.
	// Total commander TX: ptt_on_delay + pilot + OFDM frame + ptt_off_delay.
	// We see the preamble in audio (after ptt_on_delay + pilot), so remaining
	// channel time = remaining OFDM symbols + ptt_off_delay + ptt_on_delay.
	if(is_ofdm_config(current_configuration))
	{
		int interp = telecom_system->data_container.interpolation_rate;
		int sym_samples = telecom_system->data_container.Nofdm * interp;
		int frame_sym = telecom_system->data_container.preamble_nSymb
		              + telecom_system->data_container.Nsymb;
		int buf_sym = telecom_system->data_container.buffer_Nsymb.load();
		int delay_sym = (sym_samples > 0)
		              ? telecom_system->receive_stats.delay / sym_samples : 0;
		int frame_end_sym = delay_sym + frame_sym;
		int remaining_sym = frame_end_sym - buf_sym;
		if(remaining_sym < 0) remaining_sym = 0;

		// Remaining OFDM audio + commander's PTT tail + guard margin
		int wait_ms = (remaining_sym * telecom_system->data_container.Nofdm
		              * 1000 + 47999) / 48000;
		wait_ms += ptt_off_delay_ms + ptt_on_delay_ms;

		if(wait_ms > 0)
		{
			printf("[TX-ACK-PAT] Waiting %dms (remaining=%dsym delay=%dsym buf=%dsym frame=%dsym ptt_off=%d ptt_on=%d)\n",
				wait_ms, remaining_sym, delay_sym, buf_sym, frame_sym, ptt_off_delay_ms, ptt_on_delay_ms);
			fflush(stdout);
			pumped_settle_wait(wait_ms);  // §5.7-B1: virtual-clock-ify (same exit predicate); verbatim msleep on production
		}
	}
	else
	{
		// MFSK: frame is fully captured before decode, but the commander's
		// radio still needs PTT-off + TX→RX hardware switching time.
		// Without this, the ACK fires ~10ms after decode — before the
		// commander has switched to RX.
		int wait_ms = ptt_off_delay_ms + ptt_on_delay_ms;
		printf("[TX-ACK-PAT] MFSK guard %dms (ptt_off=%d, ptt_on=%d)\n",
			wait_ms, ptt_off_delay_ms, ptt_on_delay_ms);
		fflush(stdout);
		pumped_settle_wait(wait_ms);  // §5.7-B1: virtual-clock-ify (same exit predicate); verbatim msleep on production
	}

	printf("[TX-ACK-PAT] Guard done at t=%dms\n", (int)ack_turnaround_timer.get_elapsed_time_ms()); fflush(stdout);
	ptt_on();

	cl_timer ptt_on_delay_timer, ptt_off_delay_timer;
	ptt_on_delay_timer.start();

	int pattern_samples = telecom_system->ack_pattern_passband_samples;
	int symbol_period = telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate;

	// Allocate buffers: pattern + 1 symbol padding at each end for FIR filtering
	int padded_size = pattern_samples + 2 * symbol_period;
	double *raw_output = new double[padded_size];
	double *filtered1 = new double[padded_size];
	double *filtered2 = new double[padded_size];

	if(!raw_output || !filtered1 || !filtered2) exit(-34);

	memset(raw_output, 0, padded_size * sizeof(double));

	// Generate ACK pattern passband into the middle section
	telecom_system->generate_ack_pattern_passband(&raw_output[symbol_period]);

	// Pad start and end with copies of first/last symbol for FIR boundary
	memcpy(&raw_output[0], &raw_output[symbol_period], symbol_period * sizeof(double));
	memcpy(&raw_output[symbol_period + pattern_samples], &raw_output[pattern_samples], symbol_period * sizeof(double));

	// FIR filter chain (same as send_batch)
	memset(filtered1, 0, padded_size * sizeof(double));
	memset(filtered2, 0, padded_size * sizeof(double));
	telecom_system->ofdm.FIR_tx1.apply(raw_output, filtered1, padded_size);
	telecom_system->ofdm.FIR_tx2.apply(filtered1, filtered2, padded_size);

	// Wait PTT on delay
	ptt_busy_wait(ptt_on_delay_timer, ptt_on_delay_ms);

	// Pilot tone (if enabled)
	if(pilot_tone_ms > 0 && pilot_tone_hz > 0)
	{
		const double SAMPLE_RATE = 48000.0;
		const double PILOT_FREQ = (double)pilot_tone_hz;
		const double PI = 3.14159265358979323846;
		int pilot_samples = (int)(pilot_tone_ms * SAMPLE_RATE / 1000.0);
		double* pilot_buffer = new double[pilot_samples];

		for(int i = 0; i < pilot_samples; i++)
		{
			double t = (double)i / SAMPLE_RATE;
			double envelope = 1.0;
			int ramp_samples = (int)(SAMPLE_RATE * 0.005);
			if(i < ramp_samples)
				envelope = (double)i / ramp_samples;
			else if(i > pilot_samples - ramp_samples)
				envelope = (double)(pilot_samples - i) / ramp_samples;
			pilot_buffer[i] = envelope * 0.5 * sin(2.0 * PI * PILOT_FREQ * t);
		}

		tx_transfer(pilot_buffer, pilot_samples);
		delete[] pilot_buffer;
	}

	// Transmit the filtered ACK pattern (skip padding at start)
	printf("[TX-ACK-PAT] Audio start at t=%dms (%d samples)\n", (int)ack_turnaround_timer.get_elapsed_time_ms(), pattern_samples); fflush(stdout);
	tx_transfer(&filtered2[symbol_period], pattern_samples);

	// Wait for playback to drain
	drain_playback_wait();

	printf("[TX-ACK-PAT] Audio done at t=%dms\n", (int)ack_turnaround_timer.get_elapsed_time_ms()); fflush(stdout);
	mtl::log_event("rsp_ack_audio_done");

	delete[] raw_output;
	delete[] filtered1;
	delete[] filtered2;

	// Flush capture buffer immediately after drain — BEFORE ptt_off_delay.
	// On VB-Cable (zero propagation delay), the commander detects the ACK
	// pattern mid-TX and starts its guard timer. If we wait for ptt_off_delay
	// (200ms) before flushing, the commander's data can arrive while the
	// responder is still muted, destroying seq=00's preamble.
	// Flushing now lets the capture thread receive clean audio during
	// ptt_off_delay, giving 200ms+ margin instead of potentially negative.
	telecom_system->data_container.rx_mute = 1;
	sim_inproc_rx_mute_settle(RX_MUTE_GUARD_MS);  // §5.7-B5: gate-off (no async drainer in-process); reset still fires
	circular_buf_reset(capture_buffer);
	{
		int buf_samples = telecom_system->data_container.Nofdm * telecom_system->data_container.buffer_Nsymb * telecom_system->data_container.interpolation_rate;
		MUTEX_LOCK(&capture_prep_mutex);
		memset(telecom_system->data_container.passband_delayed_data, 0, 2 * buf_samples * sizeof(double));
		telecom_system->data_container.ring_write_index = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
	}
	telecom_system->data_container.rx_mute = 0;
	telecom_system->data_container.rx_mute_samples = 0;
	// Bug #41: Reset nUnder after flush. The flush destroyed all pre-flush
	// audio, so nUnder accumulated during ACK TX is stale — those captured
	// symbols no longer exist in the buffer. Without this reset, same-modulation
	// transitions (e.g. CONFIG_0→CONFIG_1) keep stale nUnder (~28 symbols),
	// the ftr calculation subtracts it, shrinking the capture window so the
	// commander's next frame arrives past upper_bound.
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.delay_of_last_decoded_message = -1;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;
	// ftr = exactly one frame. Turnaround waiting is handled by ptt_off_delay
	// below (200ms) plus the capture thread filling the buffer during that time.
	// ftr = one frame + 10 symbol margin. The margin covers PTT turnaround
	// settle time (~240ms) so the receiver doesn't start decoding noise
	// before the next TX batch arrives. Without it, early OFDM-FAILs cascade.
	{
		int rx_frame = telecom_system->data_container.preamble_nSymb
		             + telecom_system->data_container.Nsymb;
		// MULTI-CW WINDOW FIX (fact-doc §17): at the bigblock rung the NEXT thing we
		// receive is ONE K-codeword block (~64 sym), not a stock frame (~13). Snapshot
		// MUST wait for the WHOLE block or cw1..cw7 read a stale ring (cw0-ok garbage).
		telecom_system->data_container.frames_to_read = bigblock_block_ftr_or(rx_frame + 10);
	}

	printf("[TX-ACK-PAT] Done at t=%dms, flushed capture buffer, nUnder reset, ftr=%d\n", (int)ack_turnaround_timer.get_elapsed_time_ms(), telecom_system->data_container.frames_to_read.load());
	mtl::log_event_kv("rsp_post_ack_flush_done", "ftr=%d", telecom_system->data_container.frames_to_read.load());
	fflush(stdout);

	// PTT off delay + release after flush. The capture thread is active
	// during this delay, receiving silence (VB-Cable) or post-TX settling
	// noise (real radio — rejected by preamble energy gate / metric threshold).
	ptt_off_delay_timer.start();
	ptt_busy_wait(ptt_off_delay_timer, ptt_off_delay_ms);

	ptt_off();
}

// Transmit ACK + SNR suffix pattern (turboshift only)
void cl_arq_controller::send_ack_pattern_with_snr(float snr)
{
	if(passive_monitor) return;
	printf("[TX-ACK-SNR] Sending ACK+SNR pattern (SNR=%.1f dB, tone=%d) on CONFIG_%d\n",
		snr, telecom_system->ack_mfsk.snr_to_tone(snr), current_configuration);
	fflush(stdout);

	// Guard delay for MFSK modes (same as send_ack_pattern)
	if(is_robust_config(current_configuration))
	{
		int wait_ms = ptt_off_delay_ms + ptt_on_delay_ms;
		pumped_settle_wait(wait_ms);  // §5.7-B2: virtual-clock-ify (same exit predicate); verbatim msleep on production
	}

	ptt_on();

	cl_timer ptt_on_delay_timer, ptt_off_delay_timer;
	ptt_on_delay_timer.start();

	int pattern_samples = telecom_system->ack_snr_pattern_passband_samples;
	int symbol_period = telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate;

	int padded_size = pattern_samples + 2 * symbol_period;
	double *raw_output = new double[padded_size];
	double *filtered1 = new double[padded_size];
	double *filtered2 = new double[padded_size];

	if(!raw_output || !filtered1 || !filtered2) exit(-36);

	memset(raw_output, 0, padded_size * sizeof(double));

	telecom_system->generate_ack_snr_pattern_passband(&raw_output[symbol_period], snr);

	memcpy(&raw_output[0], &raw_output[symbol_period], symbol_period * sizeof(double));
	memcpy(&raw_output[symbol_period + pattern_samples], &raw_output[pattern_samples], symbol_period * sizeof(double));

	memset(filtered1, 0, padded_size * sizeof(double));
	memset(filtered2, 0, padded_size * sizeof(double));
	telecom_system->ofdm.FIR_tx1.apply(raw_output, filtered1, padded_size);
	telecom_system->ofdm.FIR_tx2.apply(filtered1, filtered2, padded_size);

	ptt_busy_wait(ptt_on_delay_timer, ptt_on_delay_ms);

	if(pilot_tone_ms > 0 && pilot_tone_hz > 0)
	{
		const double SAMPLE_RATE = 48000.0;
		const double PILOT_FREQ = (double)pilot_tone_hz;
		const double PI = 3.14159265358979323846;
		int pilot_samples = (int)(pilot_tone_ms * SAMPLE_RATE / 1000.0);
		double* pilot_buffer = new double[pilot_samples];
		for(int i = 0; i < pilot_samples; i++)
		{
			double t = (double)i / SAMPLE_RATE;
			double envelope = 1.0;
			int ramp_samples = (int)(SAMPLE_RATE * 0.005);
			if(i < ramp_samples)
				envelope = (double)i / ramp_samples;
			else if(i > pilot_samples - ramp_samples)
				envelope = (double)(pilot_samples - i) / ramp_samples;
			pilot_buffer[i] = envelope * 0.5 * sin(2.0 * PI * PILOT_FREQ * t);
		}
		tx_transfer(pilot_buffer, pilot_samples);
		delete[] pilot_buffer;
	}

	tx_transfer(&filtered2[symbol_period], pattern_samples);

	drain_playback_wait();

	delete[] raw_output;
	delete[] filtered1;
	delete[] filtered2;

	// Same flush sequence as send_ack_pattern
	telecom_system->data_container.rx_mute = 1;
	sim_inproc_rx_mute_settle(RX_MUTE_GUARD_MS);  // §5.7-B5: gate-off (no async drainer in-process); reset still fires
	circular_buf_reset(capture_buffer);
	{
		int buf_samples = telecom_system->data_container.Nofdm * telecom_system->data_container.buffer_Nsymb * telecom_system->data_container.interpolation_rate;
		MUTEX_LOCK(&capture_prep_mutex);
		memset(telecom_system->data_container.passband_delayed_data, 0, 2 * buf_samples * sizeof(double));
		telecom_system->data_container.ring_write_index = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
	}
	telecom_system->data_container.rx_mute = 0;
	telecom_system->data_container.rx_mute_samples = 0;
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.delay_of_last_decoded_message = -1;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;
	{
		int rx_frame = telecom_system->data_container.preamble_nSymb
		             + telecom_system->data_container.Nsymb;
		// MULTI-CW WINDOW FIX (fact-doc §17): block-span the window at the bigblock rung.
		telecom_system->data_container.frames_to_read = bigblock_block_ftr_or(rx_frame + 10);
	}

	printf("[TX-ACK-SNR] Done, flushed capture buffer, ftr=%d\n", telecom_system->data_container.frames_to_read.load());
	fflush(stdout);

	ptt_off_delay_timer.start();
	ptt_busy_wait(ptt_off_delay_timer, ptt_off_delay_ms);

	ptt_off();
}

// ============================================================================
// SACK Design A Step 7 — OFDM SACK_RSP control frame TX/RX helpers
// ============================================================================
// A single OFDM LDPC control frame (~390 ms at WB_CFG10) carries the partial-
// batch bitmap. Wire layout AFTER the standard 3-byte msg header
// [type=0x42, conn_id, seq_num=0]:
//
//   payload = [batch_seq_id : u8][bitmap : ceil(N/8) bytes][CRC8 : u8]
//
// where N = data_batch_size at TX time (both peers know N because
// data_batch_size is negotiated at TEST_CONNECTION and never moves within a
// Design A Step 7 session). CRC8 covers (batch_seq_id || bitmap_bytes); it
// does NOT cover the standard msg header (whose integrity is already
// guaranteed by the OFDM LDPC codeword's CRC16). Polynomial: POLY_CRC8
// (=0xF4 in datalink_defines.h), matching the existing CRC8_calc() helper.
//
// Step 15: the legacy MFSK SACK pattern path (send_sack_pattern /
// receive_sack_pattern, sack_ldpc, mercury_sack_*_16) has been deleted. v2
// OFDM SACK_RSP is now the only SACK transport.

// SACK Design A Step 8a — bsi-bump-and-prev-transfer helper.
// Hoisted out of send_sack_v2_frame() per fact-documents/sack_partial_bsi_advance.md
// §6g (2026-05-21). Callers must invoke this BEFORE choosing a SACK transport
// (OFDM SACK_RSP via send_sack_v2_frame, or MFSK suffix via send_mfsk_ack_sack)
// so the invariant fires regardless of which wire path carries the bitmap.
// The original load-bearing bug: the MFSK suffix branch at arq_responder.cc:
// 1198-1204 set used_mfsk_path=true and skipped send_sack_v2_frame(), which
// transitively skipped this bump-and-transfer block — leaving
// rsp_current_expected_batch_seq_id stuck on the partial's bsi and causing
// subsequent mixbatch new-bsi frames to drop as out_of_window
// (rsp:8019 of phase4_v2_benchmark_wgn18_mfsk_logs/sack_lossy_wgn18_WB_CFG15_sackv2_r4_rsp.log).
//
// Internally gated by (sack_v2_enabled && rsp_current_expected_batch_seq_id >= 0)
// so callers can invoke unconditionally — no-op outside v2 sessions or before
// the first frame adopts a bsi.
//
// Invariants honored (unchanged from the inlined version):
//   • §4.3.4 #1: still one outstanding batch (data_ack_received gate
//     unchanged); CMD must finish retransmits for batch N before next.
//   • §4.3.4 #2: monotonic +1 mod 256, never reset.
//   • §4.3.4 #3: no silent corruption — match-prev now routes to its OWN
//     storage; the previous Step 4 path (which routed match-prev hits
//     into `messages_rx[]` and would have overwritten current-batch
//     slots in a future Step 8b mixed batch) is replaced.
void cl_arq_controller::bump_bsi_and_transfer_prev()
{
	if(!(sack_v2_enabled && rsp_current_expected_batch_seq_id >= 0))
		return;

	// If a prev batch is still active when we bump again, it means CMD
	// never finished filling the previous prev batch via retransmits.
	// We must discard the stale prev to make room (it would otherwise
	// corrupt subsequent prev-routing). Log + count for visibility.
	if(rsp_prev_batch_active)
	{
		rsp_prev_batch_stale_count++;
		printf("[RSP-V2-PREV-STALE] discarding incomplete prev batch_seq_id=%d "
			"(received=%d/%d) — replacing with new prev_batch_seq_id=%d "
			"(stale_count=%lld)\n",
			rsp_prev_batch_seq_id, rsp_prev_batch_received_count,
			rsp_prev_batch_expected_count,
			rsp_current_expected_batch_seq_id, rsp_prev_batch_stale_count);
		fflush(stdout);
		// Clear stale prev slots before re-using the buffer.
		for(int i=0; i<this->nMessages; i++)
			messages_rx_prev[i].status = FREE;
	}

	// Determine expected count for the *new* prev (the batch we're
	// about to seal). Mirror the EOB-or-data_batch_size inference used
	// by `process_messages_acknowledging_data`'s rx_received counter.
	int prev_expected = data_batch_size;
	if(last_received_end_of_batch_seq >= 0)
	{
		int eob = last_received_end_of_batch_seq + 1;
		if(eob < prev_expected) prev_expected = eob;
	}
	if(prev_expected < 1) prev_expected = 1;
	if(prev_expected > this->nMessages) prev_expected = this->nMessages;

	// Transfer (not copy) batch-N content from messages_rx → messages_rx_prev:
	// memcpy the payload, copy the metadata, then free the source slot.
	// We pre-count the post-transfer RECEIVED slots so prev-batch
	// completion can detect "already complete on transfer" (which only
	// happens if SACK fires at a moment where all expected slots happen
	// to be RECEIVED but the ACK-GATE-PASS branch was preempted; under
	// the normal SACK partial branch, rx_received < expected by
	// construction, so received_count is strictly < expected_count here).
	int xferred = 0;
	int xferred_received = 0;
	const int alloc_size = N_MAX / 8;
	for(int i=0; i<this->data_batch_size && i<this->nMessages; i++)
	{
		messages_rx_prev[i].type   = messages_rx[i].type;
		messages_rx_prev[i].id     = messages_rx[i].id;
		messages_rx_prev[i].length = messages_rx[i].length;
		messages_rx_prev[i].status = messages_rx[i].status;
		messages_rx_prev[i].batch_seq_id = messages_rx[i].batch_seq_id;
		if(messages_rx[i].length > 0 && messages_rx[i].length <= alloc_size)
		{
			memcpy(messages_rx_prev[i].data, messages_rx[i].data,
				messages_rx[i].length);
		}
		if(messages_rx[i].status == RECEIVED) xferred_received++;
		xferred++;
		// Free the source slot so the current-batch (N+1) frames land
		// into a clean messages_rx[].
		messages_rx[i].status = FREE;
		messages_rx[i].length = 0;
		messages_rx[i].batch_seq_id = -1;
	}
	// Make sure prev slots beyond data_batch_size are FREE (defensive —
	// they should already be).
	for(int i=this->data_batch_size; i<this->nMessages; i++)
	{
		if(messages_rx_prev[i].status != FREE)
			messages_rx_prev[i].status = FREE;
	}

	rsp_prev_batch_seq_id = rsp_current_expected_batch_seq_id;
	rsp_current_expected_batch_seq_id =
		(rsp_current_expected_batch_seq_id + 1) & 0xFF;
	rsp_prev_batch_active            = true;
	rsp_prev_batch_received_count    = xferred_received;
	rsp_prev_batch_expected_count    = prev_expected;
	printf("[RSP-V2-PREV-BUMP] prev_batch_seq_id=%d next_expected=%d "
		"transferred=%d received_on_transfer=%d/%d (cross-storage routing armed)\n",
		rsp_prev_batch_seq_id, rsp_current_expected_batch_seq_id,
		xferred, rsp_prev_batch_received_count, rsp_prev_batch_expected_count);
	fflush(stdout);
}

// FIX-8 (data-integrity): advance the reset-surviving delivery high-water mark
// to `bsi`, MONOTONIC-with-wrap. Called at the two real delivery commits only.
//
// The 8-bit batch_seq_id wraps mod-256, so "forward" is the shorter direction:
// the forward distance from the current mark `m` to `bsi` is ((bsi - m) & 0xFF),
// which is in [1,128] for a genuine forward step and in [129,255] for a backward
// step (a late older prev). We advance ONLY on a forward step (or first-set from
// -1), so a recovered out-of-order PREV batch delivered after a newer CURRENT
// already advanced the mark can never regress it (audit R1). A re-delivery of the
// SAME bsi (distance 0) is a no-op (idempotent). Pure arithmetic; the call sites
// own the sack_v2_enabled gate.
void cl_arq_controller::advance_last_delivered(int bsi)
{
	int b = bsi & 0xFF;
	if(rsp_last_delivered_batch_seq_id < 0)
	{
		rsp_last_delivered_batch_seq_id = b;
		return;
	}
	int fwd = (b - (rsp_last_delivered_batch_seq_id & 0xFF)) & 0xFF;
	// fwd in [1,128] = forward step → advance. fwd==0 (same) or [129,255]
	// (backward / late older prev) → leave the mark where it is.
	if(fwd >= 1 && fwd <= 128)
		rsp_last_delivered_batch_seq_id = b;
}

// D3.1 (data-integrity): the shared LOUD GAP-ABORT teardown. Centralizes the
// block previously inlined at the FIX-8 re-adopt gate (arq_responder.cc:646-688)
// so the delivery-time gate and the SET_CONFIG re-baseline path use the IDENTICAL
// action. Control-port error, DROPPED, clear the bsi family + prev-buffer +
// carve-arm, reset_session_state(). See D31_INORDER_DESIGN.md §2.
void cl_arq_controller::rsp_gap_abort_teardown(const char* reason)
{
	printf("[RSP-V2-GAP-ABORT] %s -> aborting transfer (refusing silent concatenation)\n",
		reason ? reason : "non-contiguous delivery");
	fflush(stdout);

	// Control-port error (mirror the PSK-mismatch emit at arq_common.cc:9502-9507).
	{
		const char* err_msg = "BATCH GAP - non-contiguous delivery, transfer aborted\r";
		int elen = (int)strlen(err_msg);
		for(int e2=0; e2<elen; e2++)
			tcp_socket_control.message->buffer[e2] = err_msg[e2];
		tcp_socket_control.message->length = elen;
		tcp_socket_control.transmit();
	}
#ifdef MERCURY_GUI_ENABLED
	gui_push_monitor_event("[BATCH GAP — non-contiguous delivery, transfer aborted]", false);
#endif
	this->link_status = DROPPED;

	// Clear the bsi family + in-flight prev-buffer + carve-arm so a stale prev /
	// cur cannot misroute the next session's first frame (FIX8_AUDIT §7 R8).
	rsp_current_expected_batch_seq_id = -1;
	rsp_prev_batch_seq_id             = -1;
	rsp_prev_batch_active             = false;
	rsp_prev_batch_received_count     = 0;
	rsp_prev_batch_expected_count     = 0;
	bigblock_partial_armed            = false;
	bigblock_residual_armed           = false;   // C6 measure-only: clear the cw(K-1)-gap residual one-shot
	for(int i=0; i<this->nMessages; i++)
		messages_rx_prev[i].status = FREE;
	reset_session_state();
	// reset_session_state() set rsp_last_delivered = -1 too.
}

long long cl_arq_controller::send_sack_v2_frame(const bool* bitmap, int nframes,
                                                unsigned char batch_seq_id)
{
	if(passive_monitor) return 0;
	if(nframes <= 0 || nframes > MAX_SACK_BATCH_SIZE)
	{
		printf("[TX-SACK-V2] ERROR: nframes=%d out of range [1..%d]\n",
			nframes, MAX_SACK_BATCH_SIZE);
		fflush(stdout);
		return 0;
	}

	// SACK Design A Step 8a — bsi-bump-and-prev-transfer was previously inlined
	// here. As of fact-documents/sack_partial_bsi_advance.md §6g (2026-05-21),
	// it has been hoisted into bump_bsi_and_transfer_prev() and is now invoked
	// by callers BEFORE the transport-choice block at arq_responder.cc:~1181.
	// This makes the invariant fire for the MFSK suffix branch too (which used
	// to bypass send_sack_v2_frame() entirely via used_mfsk_path=true and
	// thereby skip the bump-and-transfer). send_sack_v2_frame() is now a pure
	// wire-transmit primitive — no bsi/prev side effects.

	int bitmap_bytes = (nframes + 7) / 8;
	int payload_len  = 1 /*batch_seq_id*/ + bitmap_bytes + 1 /*CRC8*/;

	// Build the payload directly into a scratch buffer first so we can
	// CRC-cover the (batch_seq_id || bitmap) prefix before writing CRC8.
	unsigned char payload[1 + (MAX_SACK_BATCH_SIZE + 7) / 8 + 1];
	payload[0] = batch_seq_id;
	for(int b = 0; b < bitmap_bytes; b++) payload[1 + b] = 0;
	for(int i = 0; i < nframes; i++)
	{
		if(bitmap[i])
			payload[1 + (i / 8)] |= (unsigned char)(1u << (i % 8));
	}
	unsigned char crc = CRC8_calc((char*)payload, 1 + bitmap_bytes);

	// Optional CRC8 fault injection (CLI --test-rsp-sack-rsp-crc-corrupt).
	// One-shot: clears the armed flag after firing exactly once.
	if(test_rsp_sack_rsp_crc_corrupt_armed)
	{
		unsigned char corrupted = (unsigned char)(crc ^ 0xFFu);
		printf("[TX-SACK-V2-CRC-CORRUPT] frame: CRC8 0x%02x -> 0x%02x (synthetic fault injection)\n",
			(unsigned)crc, (unsigned)corrupted);
		fflush(stdout);
		crc = corrupted;
		test_rsp_sack_rsp_crc_corrupt_armed = false;
	}
	// SACK Design A Step 11 — N-shot CRC8 fault injection. Decrements per
	// SACK_RSP TX. Used by Gate 3 (ON→PROBE→OFF) and Gate 5 (PROBE→ON).
	else if(test_rsp_sack_rsp_crc_corrupt_count > 0)
	{
		unsigned char corrupted = (unsigned char)(crc ^ 0xFFu);
		printf("[TX-SACK-V2-CRC-CORRUPT-N] frame: CRC8 0x%02x -> 0x%02x "
			"(N-shot, %d remaining after this)\n",
			(unsigned)crc, (unsigned)corrupted,
			test_rsp_sack_rsp_crc_corrupt_count - 1);
		fflush(stdout);
		crc = corrupted;
		test_rsp_sack_rsp_crc_corrupt_count--;
	}
	payload[1 + bitmap_bytes] = crc;

	// Log the ground-truth TX bitmap byte-for-byte so the loopback test can
	// assert byte-identity with the CMD-side decoded bitmap.
	{
		char hex[2 * sizeof(payload) + 1];
		for(int b = 0; b < payload_len; b++)
			snprintf(&hex[2*b], 3, "%02x", payload[b]);
		hex[2*payload_len] = '\0';
		printf("[TX-SACK-V2] batch_seq_id=%u nframes=%d bitmap_bytes=%d payload=%s crc8=0x%02x\n",
			(unsigned)batch_seq_id, nframes, bitmap_bytes, hex, (unsigned)crc);
		fflush(stdout);
	}

	// Stage as a single-frame OFDM batch via messages_batch_tx[0]. Use the
	// existing send_batch() path so the OFDM TX is bit-identical to any other
	// control-class frame's wire shape.
	//
	// Bug B fix (iter 2, SACK_DESIGN_A_PLAN §7.13.5): messages_batch_tx[i].data
	// starts NULL (arq_common.cc:1530). The design assumes slots are
	// populated via struct-copy from a pre-allocated source (messages_tx[]
	// has .data at arq_common.cc:1436; messages_control has .data at
	// arq_common.cc:1579; messages_batch_ack[] has .data at
	// arq_common.cc:1555). Writing directly into messages_batch_tx[0].data[..]
	// without a prior struct-copy crashes RSP with a NULL-pointer deref
	// (SIGSEGV, silent on Linux without core-dump enabled). RSP never
	// stages data frames so its messages_batch_tx[0].data was permanently
	// NULL before this fix. Same idiom: arq_responder.cc:720, 757.
	messages_control.type = SACK_RSP;
	messages_control.sequence_number = 0;
	messages_control.id = 0;
	messages_control.length = payload_len;
	for(int b = 0; b < payload_len; b++)
		messages_control.data[b] = (char)payload[b];
	messages_control.status = ADDED_TO_BATCH_BUFFER;
	messages_control.batch_seq_id = batch_seq_id;  // diagnostic mirror

	message_batch_counter_tx = 0;
	messages_batch_tx[0] = messages_control;  // struct-copy inherits valid .data
	message_batch_counter_tx = 1;

	// Full-length OFDM frame on the data configuration (NOT the MFSK ack
	// config — SACK_RSP carries real LDPC-coded payload).
	telecom_system->set_mfsk_ctrl_mode(false);
	// SACK_DESIGN_A_PLAN §7.13.29 — REMOVED the §7.13.25 double-shot
	// (`pad_messages_batch_tx(2)`). The trace data showed that BOTH copies
	// were missed by CMD's cross-check for the same reason: the
	// double-shot wire time (~1070 ms WB_CFG15) EXCEEDS the receive ring
	// length (~834 ms). The preamble of the first shot scrolled off
	// before CMD's MFSK ACK detector built up enough match-count to
	// trigger the cross-check. Sending a second copy didn't help because
	// the same scroll happens to it too. Single-shot SACK_RSP at
	// WB_CFG15 is ~540 ms wire — its preamble stays in the ring for the
	// full duration, giving the cross-check a chance to actually find it.
	// If single-shot proves unreliable on lossy channels we can revisit
	// with a robust-config SACK_RSP TX (CFG10/CFG4) rather than redundancy.

	SACK_TRACE("RSP TX SACK_RSP: bsi=%u nframes=%d payload_len=%d crc8=0x%02x",
		(unsigned)batch_seq_id, nframes, payload_len, (unsigned)crc);
	auto t_start = std::chrono::steady_clock::now();
	send_batch();
	auto t_end = std::chrono::steady_clock::now();
	long long elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		t_end - t_start).count();

	// §7.13.32 — Release the staging slot AFTER the wire TX completes.
	// send_batch() reads messages_control via messages_batch_tx[0] (the
	// struct-copy at the top of this function); once it returns the wire
	// is committed and the staging buffer is no longer needed. Leaving
	// status==ADDED_TO_BATCH_BUFFER here was the latent bug that
	// §7.13.31.1 surfaced: the next inbound CONTROL frame (e.g.,
	// SET_LINK_PARAMS following a POLICY-MOVE) is decoded successfully
	// by RX but then dropped at arq_responder.cc:281 because
	// messages_control is "busy" — RSP never APPLIES the new params and
	// CMD's 10x retransmits all bounce off the same gate, collapsing the
	// link. Symmetric with the post-RX ACK path that already force-FREEs
	// at arq_responder.cc:834.
	messages_control.status = FREE;

	rsp_sack_v2_tx_count++;
	printf("[TX-SACK-V2] send_batch() wire_ms=%lld ctrl_tx_time_ms=%d (legacy MFSK SACK ~1168 ms baseline)\n",
		elapsed_ms, ctrl_transmission_time_ms);
	fflush(stdout);
	SACK_TRACE("RSP TX SACK_RSP done: wire_ms=%lld tx_count=%lld", elapsed_ms, rsp_sack_v2_tx_count);
	return elapsed_ms;
}

// RSP-side TX wrapper: MFSK ACK+SACK pattern (16 base + 13 suffix = 29
// symbols on WB) carrying [bsi:8 | bitmap:32 | crc12:12]. Replaces the
// removed OFDM_ACK_CLEAN clean-batch path AND covers the partial-batch
// case (bitmap = actual per-frame mask). See
// mercury/fact-documents/mfsk-robust-ack.md for the design.
//
// Models send_ack_pattern_with_snr() for shape; uses
// telecom_system->generate_ack_sack_pattern_passband() to produce the audio.
//
// WB-only — returns 0 if MFSK_ACK_SACK_ENABLED=0 at compile time, or if
// the runtime mfsk M<16 (NB). On the NB path the caller falls back to
// the legacy MFSK ACK pattern (no SACK; NB never had the symbol-rate
// budget for SACK and the receiver implicitly treats any pattern hit
// as a clean ACK). On WB, returns wall-clock TX time in ms.
long long cl_arq_controller::send_mfsk_ack_sack(unsigned char batch_seq_id,
                                                uint32_t bitmap)
{
	if(passive_monitor) return 0;

#if !MFSK_ACK_SACK_ENABLED
	(void)batch_seq_id; (void)bitmap;
	return 0;  // Feature compiled out — caller falls back to OFDM path.
#else
	// Runtime guard: NB session (M=8) has ack_sack_suffix_len()==0.
	if(telecom_system->ack_mfsk.ack_sack_suffix_len() <= 0)
		return 0;

	// §21.3 per-batch ACK gate: the enhanced (GF(16) FEC) ACK suffix is eligible
	// ONLY at the robust tier — CONFIG_6+ → uncoded → byte-identical (the hard
	// throughput-neutrality constraint). This is a THROUGHPUT gate, not a
	// capability negotiation (the enhanced ctrl-suffix is the unconditional
	// default at the robust tier; the CAP_SUFFIX_FEC negotiation was removed in
	// cleanup/drop-suffix-fec-cap). The ENABLE is held off (ARQ_ACK_SUFFIX_FEC_
	// ENABLE=0, §21.3) pending the ACK coded-window sizing work, so this stays
	// false in 100% of cases — the predicate is wired + observable so flipping the
	// master enable is a one-line follow-on. We set the per-call flag from the gate
	// and clear it after TX so it can never leak to a later non-eligible ACK (the
	// §21.1-class shared-state discipline).
	bool ack_fec_eligible = ack_suffix_fec_eligible();
	telecom_system->ack_mfsk.ack_suffix_fec_coded =
		(ARQ_ACK_SUFFIX_FEC_ENABLE != 0) && ack_fec_eligible;
	if(g_verbose && ack_fec_eligible)
	{
		printf("[TX-MFSK-ACK-SACK] enhanced-ACK eligible (robust tier); "
			"enable=%d coded=%d\n", (int)(ARQ_ACK_SUFFIX_FEC_ENABLE != 0),
			(int)telecom_system->ack_mfsk.ack_suffix_fec_coded);
		fflush(stdout);
	}

	int nsymb = telecom_system->ack_mfsk.ack_sack_pattern_nsymb();
	if(nsymb <= 0 || telecom_system->ack_sack_pattern_passband_samples <= 0)
	{
		telecom_system->ack_mfsk.ack_suffix_fec_coded = false;
		return 0;
	}

	auto t_start = std::chrono::steady_clock::now();

	// Compute CRC12 over the 40-bit [bsi || bitmap] payload (big-endian).
	// CRC12 protects against false-accept after correlator lock — see
	// mercury/fact-documents/mfsk-robust-ack.md §3.2.
	char crc_input[5];
	crc_input[0] = (char)batch_seq_id;
	crc_input[1] = (char)((bitmap >> 24) & 0xFF);
	crc_input[2] = (char)((bitmap >> 16) & 0xFF);
	crc_input[3] = (char)((bitmap >>  8) & 0xFF);
	crc_input[4] = (char)( bitmap        & 0xFF);
	uint16_t crc12 = CRC12_calc(crc_input, 5);

	printf("[TX-MFSK-ACK-SACK] batch_seq_id=%u bitmap=0x%08x crc12=0x%03x nsymb=%d on CONFIG_%d\n",
		(unsigned)batch_seq_id, (unsigned)bitmap, (unsigned)crc12, nsymb, current_configuration);
	fflush(stdout);

	// WALL-B FIX-9 D2 REFINE (_fix9/d2refine/D2_REFINE_DESIGN.md §2.1): read+clear the per-call
	// retx-turnaround flag the caller set (partial/prev paths -> TRUE, clean first-pass -> FALSE).
	// CLEAR immediately after reading so it can never leak to a later non-eligible ACK (the §21.1
	// shared-state discipline, same as ack_suffix_fec_coded).
	bool this_ack_is_retx_turnaround = ack_tx_retx_turnaround;
	ack_tx_retx_turnaround = false;

	// Guard delay for MFSK modes (same as send_ack_pattern_with_snr).
	// WALL-B FIX-9 D2 (FIX9_ROOTCAUSE.md §4 D2, FIX9_D2_DESIGN.md §3.2): the pre-TX settle ALSO
	// fires at OFDM forward configs (reverse_ack_uses_robust_geometry), not just the robust tier.
	// The reverse data-ACK PHY is config-independent in tone set, but at an OFDM config the tight
	// turnaround gives the CMD's receiver NO time to flush its long-batch capture ring and re-arm
	// the ACK correlator at the post-batch phase — so under inter-Pi clock drift the ppm-slipped
	// ACK lands outside the CMD window (correlator pure-silent, the D3 collapse). Keying the ACK
	// with the robust pre-TX settle (the same fatter turnaround the robust tier already uses) gives
	// the CMD that re-arm margin. The CMD's calculate_receiving_timeout() widens its listen window
	// in LOCKSTEP on the SAME predicate (FIX9_ROOTCAUSE §5 invariant). Robust path UNCHANGED (the
	// first disjunct was already true there — no double-settle).
	//
	// WALL-B FIX-9 D2 REFINE: the OFDM settle now ALSO requires this_ack_is_retx_turnaround so it
	// fires ONLY on a retransmit turnaround (partial/prev ACK), NOT on a clean first-pass ACK — the
	// clean ACK keys on the tight OFDM turnaround it always used (= D3-base), recovering the ~15%
	// clean cost. Robust tier UNCHANGED (the is_robust_config disjunct is untouched). FAIL-BEFORE
	// (-DFIX9_D2REFINE_FAILBEFORE): drop the retx requirement -> D2's unconditional OFDM settle.
	bool ofdm_settle = reverse_ack_uses_robust_geometry(current_configuration)
#ifndef FIX9_D2REFINE_FAILBEFORE
		&& this_ack_is_retx_turnaround
#endif
		;
	if(is_robust_config(current_configuration) || ofdm_settle)
	{
		int wait_ms = ptt_off_delay_ms + ptt_on_delay_ms;
		printf("[TX-MFSK-ACK-SACK] D2 robust-geometry settle=%dms (robust=%d ofdm_retx=%d) on CONFIG_%d\n",
			wait_ms, is_robust_config(current_configuration) ? 1 : 0,
			ofdm_settle ? 1 : 0, current_configuration);
		fflush(stdout);
		pumped_settle_wait(wait_ms);  // §5.7-B3: virtual-clock-ify (same exit predicate); verbatim msleep on production
	}

	ptt_on();

	cl_timer ptt_on_delay_timer, ptt_off_delay_timer;
	ptt_on_delay_timer.start();

	int pattern_samples = telecom_system->ack_sack_pattern_passband_samples;
	int symbol_period = telecom_system->data_container.Nofdm
	                  * telecom_system->data_container.interpolation_rate;

	// Allocate buffers: pattern + 1 symbol padding at each end for FIR filtering
	int padded_size = pattern_samples + 2 * symbol_period;
	double *raw_output = new double[padded_size];
	double *filtered1  = new double[padded_size];
	double *filtered2  = new double[padded_size];

	if(!raw_output || !filtered1 || !filtered2) exit(-37);

	memset(raw_output, 0, padded_size * sizeof(double));

	// Generate ACK+SACK pattern passband into the middle section
	telecom_system->generate_ack_sack_pattern_passband(&raw_output[symbol_period],
		batch_seq_id, bitmap, crc12);

	// Pad start and end with copies of first/last symbol for FIR boundary
	memcpy(&raw_output[0], &raw_output[symbol_period],
		symbol_period * sizeof(double));
	memcpy(&raw_output[symbol_period + pattern_samples], &raw_output[pattern_samples],
		symbol_period * sizeof(double));

	// FIR filter chain (same as send_batch / send_ack_pattern_with_snr)
	memset(filtered1, 0, padded_size * sizeof(double));
	memset(filtered2, 0, padded_size * sizeof(double));
	telecom_system->ofdm.FIR_tx1.apply(raw_output, filtered1, padded_size);
	telecom_system->ofdm.FIR_tx2.apply(filtered1, filtered2, padded_size);

	// Wait PTT on delay
	ptt_busy_wait(ptt_on_delay_timer, ptt_on_delay_ms);

	// Pilot tone (if enabled)
	if(pilot_tone_ms > 0 && pilot_tone_hz > 0)
	{
		const double SAMPLE_RATE = 48000.0;
		const double PILOT_FREQ  = (double)pilot_tone_hz;
		const double PI = 3.14159265358979323846;
		int pilot_samples = (int)(pilot_tone_ms * SAMPLE_RATE / 1000.0);
		double* pilot_buffer = new double[pilot_samples];
		for(int i = 0; i < pilot_samples; i++)
		{
			double t = (double)i / SAMPLE_RATE;
			double envelope = 1.0;
			int ramp_samples = (int)(SAMPLE_RATE * 0.005);
			if(i < ramp_samples)
				envelope = (double)i / ramp_samples;
			else if(i > pilot_samples - ramp_samples)
				envelope = (double)(pilot_samples - i) / ramp_samples;
			pilot_buffer[i] = envelope * 0.5 * sin(2.0 * PI * PILOT_FREQ * t);
		}
		tx_transfer(pilot_buffer, pilot_samples);
		delete[] pilot_buffer;
	}

	// Transmit the filtered ACK+SACK pattern (skip padding at start)
	printf("[TX-MFSK-ACK-SACK] Audio start (%d samples)\n", pattern_samples);
	fflush(stdout);
	tx_transfer(&filtered2[symbol_period], pattern_samples);

	// Wait for playback to drain
	drain_playback_wait();

	printf("[TX-MFSK-ACK-SACK] Audio done\n");
	fflush(stdout);

	delete[] raw_output;
	delete[] filtered1;
	delete[] filtered2;

	// Same flush sequence as send_ack_pattern / send_ack_pattern_with_snr
	telecom_system->data_container.rx_mute = 1;
	sim_inproc_rx_mute_settle(RX_MUTE_GUARD_MS);  // §5.7-B5: gate-off (no async drainer in-process); reset still fires
	circular_buf_reset(capture_buffer);
	{
		int buf_samples = telecom_system->data_container.Nofdm
		                * telecom_system->data_container.buffer_Nsymb
		                * telecom_system->data_container.interpolation_rate;
		MUTEX_LOCK(&capture_prep_mutex);
		memset(telecom_system->data_container.passband_delayed_data, 0,
			2 * buf_samples * sizeof(double));
		telecom_system->data_container.ring_write_index = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
	}
	telecom_system->data_container.rx_mute = 0;
	telecom_system->data_container.rx_mute_samples = 0;
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.delay_of_last_decoded_message = -1;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;
	{
		int rx_frame = telecom_system->data_container.preamble_nSymb
		             + telecom_system->data_container.Nsymb;
		// MULTI-CW WINDOW FIX (fact-doc §17): block-span the window at the bigblock rung.
		telecom_system->data_container.frames_to_read = bigblock_block_ftr_or(rx_frame + 10);
	}

	printf("[TX-MFSK-ACK-SACK] Done, flushed capture buffer, ftr=%d\n",
		telecom_system->data_container.frames_to_read.load());
	fflush(stdout);

	ptt_off_delay_timer.start();
	ptt_busy_wait(ptt_off_delay_timer, ptt_off_delay_ms);

	ptt_off();

	// §21.3: clear the per-call ACK FEC flag so it can NEVER leak to a later,
	// non-eligible ACK (e.g. after a turboshift to an OFDM config). The CONNECT
	// suffix_fec_coded is untouched — this is the ACK-only flag.
	telecom_system->ack_mfsk.ack_suffix_fec_coded = false;

	auto t_end = std::chrono::steady_clock::now();
	long long elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		t_end - t_start).count();
	return elapsed_ms;
#endif  // MFSK_ACK_SACK_ENABLED
}

bool cl_arq_controller::decode_sack_v2_frame(bool* out_bitmap, int nframes,
                                             unsigned char* out_batch_seq_id)
{
	// Caller has verified messages_rx_buffer.type == SACK_RSP and
	// messages_rx_buffer.status == RECEIVED. Payload bytes live in
	// messages_rx_buffer.data[0..]. Expected layout:
	//   [batch_seq_id : u8][bitmap : ceil(N/8) bytes][CRC8 : u8]
	if(nframes <= 0 || nframes > MAX_SACK_BATCH_SIZE)
	{
		printf("[CMD-SACK-V2-DECODE] ERROR: nframes=%d out of range\n", nframes);
		fflush(stdout);
		return false;
	}
	int bitmap_bytes = (nframes + 7) / 8;
	int payload_len  = 1 + bitmap_bytes + 1;

	unsigned char payload[1 + (MAX_SACK_BATCH_SIZE + 7) / 8 + 1];
	for(int b = 0; b < payload_len; b++)
		payload[b] = (unsigned char)messages_rx_buffer.data[b];

	unsigned char rx_crc       = payload[1 + bitmap_bytes];
	unsigned char computed_crc = CRC8_calc((char*)payload, 1 + bitmap_bytes);

	if(rx_crc != computed_crc)
	{
		cmd_sack_v2_crc_fail_count++;
		// Compose a hex dump for forensic analysis (no fabrication —
		// the bitmap is discarded; the caller falls back to existing
		// retransmit-timeout logic, exactly as if the OFDM frame had
		// been lost in the air per §9.4/A2).
		char hex[2 * sizeof(payload) + 1];
		for(int b = 0; b < payload_len; b++)
			snprintf(&hex[2*b], 3, "%02x", payload[b]);
		hex[2*payload_len] = '\0';
		printf("[CMD-SACK-V2-CRC-FAIL] rx_crc=0x%02x computed=0x%02x nframes=%d payload=%s fail_count=%lld (discarding bitmap; no fabrication per §9.4/A2)\n",
			(unsigned)rx_crc, (unsigned)computed_crc, nframes, hex,
			cmd_sack_v2_crc_fail_count);
		fflush(stdout);
		return false;
	}

	// CRC pass — write out the bitmap.
	*out_batch_seq_id = payload[0];
	for(int i = 0; i < nframes; i++)
	{
		out_bitmap[i] = (payload[1 + (i / 8)] & (1u << (i % 8))) != 0;
	}

	// Update CMD-side observability state (for tests).
	cmd_sack_v2_rx_count++;
	cmd_sack_v2_last_rx_batch_seq_id = (int)payload[0];
	cmd_sack_v2_last_rx_nbits = nframes;
	int copy_n = bitmap_bytes;
	if(copy_n > (int)sizeof(cmd_sack_v2_last_rx_bitmap))
		copy_n = (int)sizeof(cmd_sack_v2_last_rx_bitmap);
	for(int b = 0; b < copy_n; b++)
		cmd_sack_v2_last_rx_bitmap[b] = payload[1 + b];

	// Log decoded bitmap byte-for-byte for the v2<->v2 byte-identity test.
	char hex[2 * sizeof(payload) + 1];
	for(int b = 0; b < payload_len; b++)
		snprintf(&hex[2*b], 3, "%02x", payload[b]);
	hex[2*payload_len] = '\0';
	printf("[CMD-SACK-V2] batch_seq_id=%u nframes=%d bitmap_bytes=%d payload=%s crc8_ok=0x%02x rx_count=%lld\n",
		(unsigned)payload[0], nframes, bitmap_bytes, hex,
		(unsigned)rx_crc, cmd_sack_v2_rx_count);
	fflush(stdout);
	return true;
}

// Transmit BREAK tone pattern — emergency "drop to ROBUST_0" signal
void cl_arq_controller::send_break_pattern()
{
	if(passive_monitor) return;
	printf("[TX-BREAK] Sending BREAK pattern on CONFIG_%d\n", current_configuration);
	fflush(stdout);

	// Window-stabilization cooldown after BREAK. The optimizer's 50-batch
	// rolling window has stale "BREAK-era" failed batches in it that would
	// poison the eff_bps / sack_rate means for several batches at the new
	// (post-BREAK) config. 8 batches gives the window time to refill ~16%
	// with fresh post-BREAK measurements before we let the optimizer act
	// on them. At mid-tier configs (batch ~1.5s) this is ~10s; at low
	// configs (batch ~12s, where BREAK often lands us) it's ~100s.
	//
	// Note: anti-thrashing is PRIMARILY handled by the below-table-range
	// gate in opt_evaluate_batch_end() — if the channel is still bad
	// post-BREAK, sack_rate > max_calibrated_sack_rate keeps the optimizer
	// silent indefinitely without needing this cooldown. This cooldown
	// only matters when the channel IMPROVED enough to clear the gate but
	// the window mean is still skewed by old data.
	rate_opt.force_cooldown(8);

	ptt_on();

	cl_timer ptt_on_delay_timer, ptt_off_delay_timer;
	ptt_on_delay_timer.start();

	int pattern_samples = telecom_system->ack_pattern_passband_samples;
	int symbol_period = telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate;

	int padded_size = pattern_samples + 2 * symbol_period;
	double *raw_output = new double[padded_size];
	double *filtered1 = new double[padded_size];
	double *filtered2 = new double[padded_size];

	if(!raw_output || !filtered1 || !filtered2) exit(-35);

	memset(raw_output, 0, padded_size * sizeof(double));

	// Generate BREAK pattern passband (different tones from ACK)
	telecom_system->generate_break_pattern_passband(&raw_output[symbol_period]);

	memcpy(&raw_output[0], &raw_output[symbol_period], symbol_period * sizeof(double));
	memcpy(&raw_output[symbol_period + pattern_samples], &raw_output[pattern_samples], symbol_period * sizeof(double));

	memset(filtered1, 0, padded_size * sizeof(double));
	memset(filtered2, 0, padded_size * sizeof(double));
	telecom_system->ofdm.FIR_tx1.apply(raw_output, filtered1, padded_size);
	telecom_system->ofdm.FIR_tx2.apply(filtered1, filtered2, padded_size);

	ptt_busy_wait(ptt_on_delay_timer, ptt_on_delay_ms);

	if(pilot_tone_ms > 0 && pilot_tone_hz > 0)
	{
		const double SAMPLE_RATE = 48000.0;
		const double PILOT_FREQ = (double)pilot_tone_hz;
		const double PI = 3.14159265358979323846;
		int pilot_samples = (int)(pilot_tone_ms * SAMPLE_RATE / 1000.0);
		double* pilot_buffer = new double[pilot_samples];

		for(int i = 0; i < pilot_samples; i++)
		{
			double t = (double)i / SAMPLE_RATE;
			double envelope = 1.0;
			int ramp_samples = (int)(SAMPLE_RATE * 0.005);
			if(i < ramp_samples)
				envelope = (double)i / ramp_samples;
			else if(i > pilot_samples - ramp_samples)
				envelope = (double)(pilot_samples - i) / ramp_samples;
			pilot_buffer[i] = envelope * 0.5 * sin(2.0 * PI * PILOT_FREQ * t);
		}

		tx_transfer(pilot_buffer, pilot_samples);
		delete[] pilot_buffer;
	}

	tx_transfer(&filtered2[symbol_period], pattern_samples);

	drain_playback_wait();

	delete[] raw_output;
	delete[] filtered1;
	delete[] filtered2;

	// Flush before ptt_off_delay (same rationale as send_ack_pattern).
	telecom_system->data_container.rx_mute = 1;
	sim_inproc_rx_mute_settle(RX_MUTE_GUARD_MS);  // §5.7-B5: gate-off (no async drainer in-process); reset still fires
	circular_buf_reset(capture_buffer);
	{
		int buf_samples = telecom_system->data_container.Nofdm * telecom_system->data_container.buffer_Nsymb * telecom_system->data_container.interpolation_rate;
		MUTEX_LOCK(&capture_prep_mutex);
		memset(telecom_system->data_container.passband_delayed_data, 0, 2 * buf_samples * sizeof(double));
		telecom_system->data_container.ring_write_index = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
	}
	telecom_system->data_container.rx_mute = 0;
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.delay_of_last_decoded_message = -1;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;
	{
		// One frame + 10 symbol margin for PTT turnaround settle.
		int frame_symb = telecom_system->data_container.preamble_nSymb + telecom_system->data_container.Nsymb;
		telecom_system->data_container.frames_to_read = frame_symb + 10;
	}

	printf("[TX-BREAK] Done, flushed capture buffer, ftr=%d\n", telecom_system->data_container.frames_to_read.load());
	fflush(stdout);

	ptt_off_delay_timer.start();
	ptt_busy_wait(ptt_off_delay_timer, ptt_off_delay_ms);

	ptt_off();
}

// =============================================================================
// Phase B Wave 2 v2 — PHY-level helpers for MFSK CONNECT
// =============================================================================
//
// These are deep PHY-only helpers. They emit / decode the CONNECT base + suffix
// bits-on-the-wire. They DO NOT touch messages_control, messages_rx_buffer,
// link_status, connection_status, or any timer state. The callers
// (process_messages_tx_control on TX, process_messages_rx_data_control on RX,
// etc.) handle all state-machine bookkeeping via the unchanged legacy paths.
//
// See fact-documents/phase-b-mfsk-connect-research.md §13 for the
// architectural rationale (the bypass-and-replicate design of Wave 2 v1 was
// abandoned after 4 serial sibling bugs; v2 routes through legacy).

// Pack [type:2 | payload38:38] into 5 bytes MSB-first for CRC12.
// The CRC12_calc helper expects forward-bit MSB-first input — same shape as
// the existing ack_sack CRC12 input at arq_common.cc:4308-4313.
static inline void pack_ctrl_typed40_msb_v2(uint8_t out_bytes[5],
                                            mfsk_ctrl_frame_type type,
                                            uint64_t payload38)
{
	uint64_t typed40 = ((uint64_t)(type & 0x3) << 38) | (payload38 & ((1ULL << 38) - 1ULL));
	for(int b = 0; b < 5; b++)
		out_bytes[b] = (uint8_t)((typed40 >> (8 * (4 - b))) & 0xFF);
}

// §19 (INCREMENT 1): CRC-12 callback wrapping the PRODUCTION
// cl_arq_controller::CRC12_calc (init=0xFFF) for the GF(16) RA suffix decoder's
// CRC accept gate. ctx = cl_arq_controller*. Matches ctrl_crc12_fn. NEVER
// inline the CRC (v1 bug #1 was an init mismatch between TX CRC12_calc and an
// inlined RX copy). Identical convention to the test harness prod_crc12_cb.
static uint16_t arq_ctrl_crc12_cb(void* ctx, const unsigned char* data, int n)
{
	cl_arq_controller* self = static_cast<cl_arq_controller*>(ctx);
	return self->CRC12_calc((const char*)data, n) & 0x0FFF;
}

// Shared TX core: emit CONNECT base + 13-symbol ctrl-suffix for `type` with
// `payload38`. Returns wall-clock TX time in ms, 0 if unsupported.
//
// Modeled closely on send_mfsk_ack_sack() at arq_common.cc:4287. Same PTT /
// FIR / pilot-tone / RX-mute / capture-flush sequence — the only PHY-level
// differences are (a) the CONNECT base pattern uses g=3 tones (vs ACK's g=5),
// and (b) the suffix carries the 2-bit type discriminator.
static long long send_mfsk_ctrl_suffix_phy_core(cl_arq_controller* self,
                                                cl_telecom_system* telecom_system,
                                                mfsk_ctrl_frame_type type,
                                                uint64_t payload38,
                                                const char* tag)
{
	if(self->passive_monitor) return 0;

	// Runtime guard: NB session (M=8) has no CONNECT base pattern.
	if(telecom_system->ack_mfsk.connect_pattern_nsymb <= 0) return 0;
	if(telecom_system->ctrl_suffix_pattern_passband_samples <= 0) return 0;

	auto t_start = std::chrono::steady_clock::now();

	// CRC12 over the 5-byte [type:2|payload:38] big-endian field. Uses the
	// production CRC12_calc helper (init=0xFFF) — never inline this (v1 bug
	// #1 was an init=0 mismatch between sender and receiver inline copy).
	uint8_t typed_bytes[5];
	pack_ctrl_typed40_msb_v2(typed_bytes, type, payload38);
	uint16_t crc12 = self->CRC12_calc((char*)typed_bytes, 5);

	printf("[TX-MFSK-CTRL-%s] type=%d p38=0x%010llx crc12=0x%03x on CONFIG_%d\n",
		tag, (int)type, (unsigned long long)payload38, (unsigned)crc12,
		self->current_configuration);
	fflush(stdout);

	// Guard delay for MFSK modes (same as send_mfsk_ack_sack:4321-4325).
	if(is_robust_config(self->current_configuration))
	{
		int wait_ms = self->ptt_off_delay_ms + self->ptt_on_delay_ms;
		pumped_settle_wait(wait_ms);  // §5.7-B4: virtual-clock-ify (same exit predicate); verbatim msleep on production
	}

	self->ptt_on();

	cl_timer ptt_on_delay_timer, ptt_off_delay_timer;
	ptt_on_delay_timer.start();

	int pattern_samples = telecom_system->ctrl_suffix_pattern_passband_samples;
	int symbol_period = telecom_system->data_container.Nofdm
	                  * telecom_system->data_container.interpolation_rate;

	int padded_size = pattern_samples + 2 * symbol_period;
	double *raw_output = new double[padded_size];
	double *filtered1  = new double[padded_size];
	double *filtered2  = new double[padded_size];
	if(!raw_output || !filtered1 || !filtered2) exit(-37);
	memset(raw_output, 0, padded_size * sizeof(double));

	int written = telecom_system->generate_ctrl_suffix_pattern_passband(
		&raw_output[symbol_period], type, payload38, crc12);
	if(written != pattern_samples)
	{
		printf("[TX-MFSK-CTRL-%s] generate returned %d, expected %d — abort\n",
			tag, written, pattern_samples);
		fflush(stdout);
		delete[] raw_output;
		delete[] filtered1;
		delete[] filtered2;
		self->ptt_off();
		return 0;
	}

	memcpy(&raw_output[0], &raw_output[symbol_period],
		symbol_period * sizeof(double));
	memcpy(&raw_output[symbol_period + pattern_samples], &raw_output[pattern_samples],
		symbol_period * sizeof(double));

	memset(filtered1, 0, padded_size * sizeof(double));
	memset(filtered2, 0, padded_size * sizeof(double));
	telecom_system->ofdm.FIR_tx1.apply(raw_output, filtered1, padded_size);
	telecom_system->ofdm.FIR_tx2.apply(filtered1, filtered2, padded_size);

	ptt_busy_wait(ptt_on_delay_timer, self->ptt_on_delay_ms);

	// Pilot tone (same conditional shape as send_mfsk_ack_sack:4367-4387).
	if(self->pilot_tone_ms > 0 && self->pilot_tone_hz > 0)
	{
		const double SAMPLE_RATE = 48000.0;
		const double PILOT_FREQ  = (double)self->pilot_tone_hz;
		const double PI = 3.14159265358979323846;
		int pilot_samples = (int)(self->pilot_tone_ms * SAMPLE_RATE / 1000.0);
		double* pilot_buffer = new double[pilot_samples];
		for(int i = 0; i < pilot_samples; i++)
		{
			double t = (double)i / SAMPLE_RATE;
			double envelope = 1.0;
			int ramp_samples = (int)(SAMPLE_RATE * 0.005);
			if(i < ramp_samples)
				envelope = (double)i / ramp_samples;
			else if(i > pilot_samples - ramp_samples)
				envelope = (double)(pilot_samples - i) / ramp_samples;
			pilot_buffer[i] = envelope * 0.5 * sin(2.0 * PI * PILOT_FREQ * t);
		}
		tx_transfer(pilot_buffer, pilot_samples);
		delete[] pilot_buffer;
	}

	tx_transfer(&filtered2[symbol_period], pattern_samples);

	drain_playback_wait();

	delete[] raw_output;
	delete[] filtered1;
	delete[] filtered2;

	// Same capture-flush sequence as send_mfsk_ack_sack:4406-4429.
	telecom_system->data_container.rx_mute = 1;
	sim_inproc_rx_mute_settle(RX_MUTE_GUARD_MS);  // §5.7-B5: gate-off (no async drainer in-process); reset still fires
	circular_buf_reset(capture_buffer);
	{
		int buf_samples = telecom_system->data_container.Nofdm
		                * telecom_system->data_container.buffer_Nsymb
		                * telecom_system->data_container.interpolation_rate;
		MUTEX_LOCK(&capture_prep_mutex);
		memset(telecom_system->data_container.passband_delayed_data, 0,
			2 * buf_samples * sizeof(double));
		telecom_system->data_container.ring_write_index = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
	}
	telecom_system->data_container.rx_mute = 0;
	telecom_system->data_container.rx_mute_samples = 0;
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.delay_of_last_decoded_message = -1;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;
	// Short ftr (2 symbols) so the next poll cycle can run the MFSK
	// suffix detector immediately. Same as send_hail_pattern's post-TX
	// state (receive_hail_pattern at :4867 also leaves ftr=2 on no-detect).
	// The legacy LDPC path overrides ftr to preamble+Nsymb later if needed.
	telecom_system->data_container.frames_to_read = 2;

	printf("[TX-MFSK-CTRL-%s] Done, flushed capture buffer, ftr=%d\n",
		tag, telecom_system->data_container.frames_to_read.load());
	fflush(stdout);

	ptt_off_delay_timer.start();
	ptt_busy_wait(ptt_off_delay_timer, self->ptt_off_delay_ms);

	self->ptt_off();

	auto t_end = std::chrono::steady_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(
		t_end - t_start).count();
}

// CMD-side TX (Site A in §13.2): MFSK START_CONN suffix carrying
// [nb_flag:1 | sender_pack:36 | reserved:1]. The sender callsign comes
// from the existing messages_control build at arq_commander.cc:454 (base
// callsign stripped of SSID), but the helper takes it as a parameter so
// callers stay decoupled from messages_control.
long long cl_arq_controller::send_mfsk_start_conn_phy(const std::string& sender_call)
{
	bool nb_flag = (narrowband_enabled == YES || commander_configured_nb == YES);
	uint64_t p38 = 0;
	pack_start_conn_payload(&p38, nb_flag, sender_call.c_str(),
		(int)sender_call.length());
	return send_mfsk_ctrl_suffix_phy_core(this, telecom_system,
		MFSK_CTRL_START_CONN, p38, "CONNECT-START");
}

// RSP-side TX (Site C in §13.2): MFSK TEST_CONNECTION_ACK suffix carrying
// [echoed_cap:2 | own_cap:2 | ssid:8 | reserved:26].
long long cl_arq_controller::send_mfsk_test_ack_phy(uint8_t echoed_cap,
                                                    uint8_t own_cap,
                                                    uint8_t ssid)
{
	uint64_t p38 = 0;
	pack_test_ack_payload(&p38, echoed_cap, own_cap, ssid);
	return send_mfsk_ctrl_suffix_phy_core(this, telecom_system,
		MFSK_CTRL_TEST_ACK, p38, "CONNECT-ACK");
}

// Shared RX core: snapshot the capture-buffer tail, run the CONNECT base
// detector + suffix decode, verify CRC12 via the production CRC12_calc,
// require the type discriminator to match `expected_type`. Returns true on
// a clean type-matched CRC-validated decode; the caller unpacks `out_p38`.
//
// IMPORTANT: this is called from BEFORE the legacy LDPC receive() path runs,
// so the gate at "if frames_to_read != 0" is the only place we sample the
// buffer. The caller (Site B / Site D) overrides frames_to_read to 2 if it
// finds a larger value (mirroring HAIL's override at arq_responder.cc:128-136)
// — v1 bug #3 was the lack of this override.
static bool receive_mfsk_ctrl_suffix_phy_core(cl_arq_controller* self,
                                              cl_telecom_system* telecom_system,
                                              mfsk_ctrl_frame_type expected_type,
                                              uint64_t* out_p38,
                                              const char* tag)
{
	int conn_nsymb = telecom_system->ack_mfsk.connect_pattern_nsymb;
	// §19.4 C6: use the CODED suffix length (52 with Tier-2 FEC, 13 uncoded) so
	// the captured passband tail actually CONTAINS the full coded suffix plus
	// the existing 16-symbol margin. tail_samples is clamped to signal_period
	// below (the ring), which is hundreds of symbols at the robust configs, so
	// the larger coded window fits. §20.3 C6: the base now occupies
	// connect_base_total_nsymb() (R×16 when combining) — the capture tail must
	// hold all R base reps + the suffix + margin (R=1 → conn_nsymb, byte-identical).
	int base_total_nsymb = telecom_system->ack_mfsk.connect_base_total_nsymb();
	int suffix_nsymb = telecom_system->ack_mfsk.ctrl_suffix_len();
	if(conn_nsymb <= 0 || suffix_nsymb <= 0) return false;
	const int tail_nsymb = base_total_nsymb + suffix_nsymb + 16;
	int sym_samples = telecom_system->data_container.Nofdm
	                * telecom_system->data_container.interpolation_rate;
	int signal_period = sym_samples * telecom_system->data_container.buffer_Nsymb;
	int tail_samples = tail_nsymb * sym_samples;
	if(tail_samples > signal_period) tail_samples = signal_period;
	int tail_offset = signal_period - tail_samples;

	MUTEX_LOCK(&capture_prep_mutex);

	if(telecom_system->data_container.frames_to_read != 0)
	{
		MUTEX_UNLOCK(&capture_prep_mutex);
		return false;
	}

	int rwi = telecom_system->data_container.ring_write_index;
	memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
		&telecom_system->data_container.passband_delayed_data[rwi + tail_offset],
		tail_samples * sizeof(double));

	telecom_system->data_container.data_ready = 0;
	MUTEX_UNLOCK(&capture_prep_mutex);

	mfsk_ctrl_frame_type rx_type;
	uint64_t rx_p38 = 0;
	uint16_t rx_crc12 = 0;
	int rx_matched = 0;
	// §19: pass the production CRC12 callback so the GF(16) FEC decode path can
	// run its CRC accept gate (no-op for the uncoded path, which returns the
	// unpacked crc12 for the outer re-check below).
	bool decoded = telecom_system->decode_ctrl_suffix_from_passband(
		telecom_system->data_container.ready_to_process_passband_delayed_data,
		tail_samples, &rx_type, &rx_p38, &rx_crc12, &rx_matched,
		arq_ctrl_crc12_cb, self);

	if(!decoded)
	{
		telecom_system->data_container.frames_to_read = 2;
		telecom_system->data_container.nUnder_processing_events = 0;
		return false;
	}

	// Type-discriminator routing: drop mismatched types (might be a CONNECT
	// suffix landing in the wrong receive window — e.g. RSP heard another
	// CMD's TEST_ACK while it was waiting for a START_CONN).
	if(rx_type != expected_type)
	{
		static int wrongtype_log = 0;
		if((wrongtype_log++ & 0x3F) == 0)
		{
			printf("[RX-MFSK-CTRL-%s] wrong type rx=%d expected=%d matched=%d "
				"(rate-limited log)\n",
				tag, (int)rx_type, (int)expected_type, rx_matched);
			fflush(stdout);
		}
		telecom_system->data_container.frames_to_read = 2;
		telecom_system->data_container.nUnder_processing_events = 0;
		return false;
	}

	// CRC12 validation via the production CRC12_calc helper (NEVER inline
	// — v1 bug #1 was an init-mismatch between sender's CRC12_calc and an
	// inlined RX-side copy that defaulted to init=0).
	uint8_t typed_bytes[5];
	pack_ctrl_typed40_msb_v2(typed_bytes, rx_type, rx_p38);
	uint16_t expected = self->CRC12_calc((char*)typed_bytes, 5);
	if(rx_crc12 != expected)
	{
		printf("[RX-MFSK-CTRL-%s] CRC12 fail type=%d p38=0x%010llx rx=0x%03x "
			"exp=0x%03x matched=%d\n",
			tag, (int)rx_type, (unsigned long long)rx_p38,
			(unsigned)rx_crc12, (unsigned)expected, rx_matched);
		fflush(stdout);
		telecom_system->data_container.frames_to_read = 2;
		telecom_system->data_container.nUnder_processing_events = 0;
		return false;
	}

	*out_p38 = rx_p38;
	// Flush the matched audio region — the next caller iteration must not
	// re-detect this same frame. Mirror receive_hail_pattern at :4856-4863.
	MUTEX_LOCK(&capture_prep_mutex);
	telecom_system->data_container.frames_to_read =
		telecom_system->data_container.preamble_nSymb + telecom_system->data_container.Nsymb;
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;
	MUTEX_UNLOCK(&capture_prep_mutex);
	return true;
}

// RSP-side RX (Site B in §13.2): detect MFSK START_CONN, unpack callsign +
// NB flag. Returns true on a clean decode; caller (the new block in
// process_messages_rx_data_control) synthesizes messages_rx_buffer.
bool cl_arq_controller::receive_mfsk_start_conn_phy(char out_call[7],
                                                    int* out_call_len,
                                                    bool* out_nb_flag)
{
	if(!out_call || !out_call_len || !out_nb_flag) return false;
	uint64_t p38 = 0;
	if(!receive_mfsk_ctrl_suffix_phy_core(this, telecom_system,
			MFSK_CTRL_START_CONN, &p38, "CONNECT-START"))
		return false;
	if(!unpack_start_conn_payload(p38, out_nb_flag, out_call, out_call_len))
		return false;
	printf("[RX-MFSK-CTRL-CONNECT-START] sender='%s' (len=%d) nb=%d\n",
		out_call, *out_call_len, *out_nb_flag ? 1 : 0);
	fflush(stdout);
	return true;
}

// CMD-side RX (Site D in §13.2): detect MFSK TEST_ACK, unpack caps + SSID.
// Returns true on a clean decode; caller (the new block in
// process_messages_rx_acks_control) synthesizes messages_control.data[].
bool cl_arq_controller::receive_mfsk_test_ack_phy(uint8_t* out_echoed_cap,
                                                  uint8_t* out_own_cap,
                                                  uint8_t* out_ssid)
{
	if(!out_echoed_cap || !out_own_cap || !out_ssid) return false;
	uint64_t p38 = 0;
	if(!receive_mfsk_ctrl_suffix_phy_core(this, telecom_system,
			MFSK_CTRL_TEST_ACK, &p38, "CONNECT-ACK"))
		return false;
	if(!unpack_test_ack_payload(p38, out_echoed_cap, out_own_cap, out_ssid))
		return false;
	printf("[RX-MFSK-CTRL-CONNECT-ACK] echoed_cap=0x%02X own_cap=0x%02X ssid=%u\n",
		*out_echoed_cap, *out_own_cap, *out_ssid);
	fflush(stdout);
	return true;
}

// CMD-side TX (Site E in §14): MFSK TEST_CONNECTION suffix carrying
// [snr_q:4 | local_cap:2 | ssid:8 | reserved:24]. The legacy CMD-side
// LDPC build at arq_commander.cc:466-475 populates messages_control.data
// with float SNR + capability + SSID; Site E reads those fields, quantizes
// SNR via cl_mfsk::snr_to_tone (M=16 → 4 bits, range -5..+25 dB step 2 dB),
// packs and emits the suffix.
long long cl_arq_controller::send_mfsk_test_conn_phy(float snr,
                                                     uint8_t local_cap,
                                                     uint8_t ssid)
{
	// snr_to_tone at M=16 is the canonical 4-bit SNR quantizer
	// (mfsk.cc:549-559). Returns 0..15.
	int snr_tone = telecom_system->ack_mfsk.snr_to_tone(snr);
	uint8_t snr_q = (uint8_t)(snr_tone & 0xF);
	uint64_t p38 = 0;
	pack_test_conn_payload(&p38, snr_q, local_cap, ssid);
	return send_mfsk_ctrl_suffix_phy_core(this, telecom_system,
		MFSK_CTRL_TEST_CONN, p38, "CONNECT-TEST");
}

// RSP-side RX (Site F in §14): detect MFSK TEST_CONN, unpack snr_q + caps + SSID.
// Returns true on a clean decode; caller (the new block in
// process_messages_rx_data_control) reconstructs float SNR via
// cl_mfsk::tone_to_snr and synthesizes messages_rx_buffer.data[] in the
// LDPC TEST_CONNECTION layout the legacy consumer at arq_responder.cc:1880-2079
// reads.
bool cl_arq_controller::receive_mfsk_test_conn_phy(uint8_t* out_snr_q,
                                                   uint8_t* out_local_cap,
                                                   uint8_t* out_ssid)
{
	if(!out_snr_q || !out_local_cap || !out_ssid) return false;
	uint64_t p38 = 0;
	if(!receive_mfsk_ctrl_suffix_phy_core(this, telecom_system,
			MFSK_CTRL_TEST_CONN, &p38, "CONNECT-TEST"))
		return false;
	if(!unpack_test_conn_payload(p38, out_snr_q, out_local_cap, out_ssid))
		return false;
	printf("[RX-MFSK-CTRL-CONNECT-TEST] snr_q=%u (=%.1f dB) local_cap=0x%02X ssid=%u\n",
		*out_snr_q, telecom_system->ack_mfsk.tone_to_snr((int)*out_snr_q),
		*out_local_cap, *out_ssid);
	fflush(stdout);
	return true;
}

// TX "I am Mercury" HAIL beacon — prefix + optional CRC suffix for directed hailing.
void cl_arq_controller::send_hail_pattern()
{
	if(passive_monitor) return;
	printf("[TX-HAIL] Sending HAIL beacon (%s, %d symbols)\n",
		telecom_system->ack_mfsk.hail_directed ? "directed" : "undirected",
		telecom_system->ack_mfsk.hail_detect_nsymb);
	fflush(stdout);

	ptt_on();

	cl_timer ptt_on_delay_timer, ptt_off_delay_timer;
	ptt_on_delay_timer.start();

	int hail_nsymb = telecom_system->ack_mfsk.hail_detect_nsymb;
	int sym_samples = telecom_system->data_container.Nofdm
	                * telecom_system->data_container.interpolation_rate;
	int pattern_samples = hail_nsymb * sym_samples;
	int symbol_period = telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate;

	int padded_size = pattern_samples + 2 * symbol_period;
	double *raw_output = new double[padded_size];
	double *filtered1 = new double[padded_size];
	double *filtered2 = new double[padded_size];

	if(!raw_output || !filtered1 || !filtered2) exit(-36);

	memset(raw_output, 0, padded_size * sizeof(double));

	telecom_system->generate_hail_pattern_passband(&raw_output[symbol_period]);

	memcpy(&raw_output[0], &raw_output[symbol_period], symbol_period * sizeof(double));
	memcpy(&raw_output[symbol_period + pattern_samples], &raw_output[pattern_samples], symbol_period * sizeof(double));

	memset(filtered1, 0, padded_size * sizeof(double));
	memset(filtered2, 0, padded_size * sizeof(double));
	telecom_system->ofdm.FIR_tx1.apply(raw_output, filtered1, padded_size);
	telecom_system->ofdm.FIR_tx2.apply(filtered1, filtered2, padded_size);

	ptt_busy_wait(ptt_on_delay_timer, ptt_on_delay_ms);

	if(pilot_tone_ms > 0 && pilot_tone_hz > 0)
	{
		const double SAMPLE_RATE = 48000.0;
		const double PILOT_FREQ = (double)pilot_tone_hz;
		const double PI = 3.14159265358979323846;
		int pilot_samples = (int)(pilot_tone_ms * SAMPLE_RATE / 1000.0);
		double* pilot_buffer = new double[pilot_samples];

		for(int i = 0; i < pilot_samples; i++)
		{
			double t = (double)i / SAMPLE_RATE;
			double envelope = 1.0;
			int ramp_samples = (int)(SAMPLE_RATE * 0.005);
			if(i < ramp_samples)
				envelope = (double)i / ramp_samples;
			else if(i > pilot_samples - ramp_samples)
				envelope = (double)(pilot_samples - i) / ramp_samples;
			pilot_buffer[i] = envelope * 0.5 * sin(2.0 * PI * PILOT_FREQ * t);
		}

		tx_transfer(pilot_buffer, pilot_samples);
		delete[] pilot_buffer;
	}

	tx_transfer(&filtered2[symbol_period], pattern_samples);

	drain_playback_wait();

	delete[] raw_output;
	delete[] filtered1;
	delete[] filtered2;

	// Flush before ptt_off_delay (same rationale as send_ack_pattern).
	telecom_system->data_container.rx_mute = 1;
	sim_inproc_rx_mute_settle(RX_MUTE_GUARD_MS);  // §5.7-B5: gate-off (no async drainer in-process); reset still fires
	circular_buf_reset(capture_buffer);
	{
		int buf_samples = telecom_system->data_container.Nofdm * telecom_system->data_container.buffer_Nsymb * telecom_system->data_container.interpolation_rate;
		MUTEX_LOCK(&capture_prep_mutex);
		memset(telecom_system->data_container.passband_delayed_data, 0, 2 * buf_samples * sizeof(double));
		telecom_system->data_container.ring_write_index = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
	}
	telecom_system->data_container.rx_mute = 0;
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.delay_of_last_decoded_message = -1;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;
	// Short ftr for fast HAIL response scanning (not a full LDPC frame).
	// The listen loop polls receive_hail_pattern() which needs ftr==0.
	telecom_system->data_container.frames_to_read = 2;

	printf("[TX-HAIL] Done, flushed capture buffer, ftr=%d\n", telecom_system->data_container.frames_to_read.load());
	fflush(stdout);

	ptt_off_delay_timer.start();
	ptt_busy_wait(ptt_off_delay_timer, ptt_off_delay_ms);

	ptt_off();
}

// RX: Detect HAIL beacon in capture buffer tail (same mechanism as receive_ack_pattern).
bool cl_arq_controller::receive_hail_pattern()
{
	const int tail_nsymb = telecom_system->ack_mfsk.hail_detect_nsymb + 8 + 16;
	int sym_samples = telecom_system->data_container.Nofdm
	                * telecom_system->data_container.interpolation_rate;
	int signal_period = sym_samples * telecom_system->data_container.buffer_Nsymb;
	int tail_samples = tail_nsymb * sym_samples;
	if(tail_samples > signal_period)
		tail_samples = signal_period;
	int tail_offset = signal_period - tail_samples;

	MUTEX_LOCK(&capture_prep_mutex);

	if(telecom_system->data_container.frames_to_read == 0)
	{
		int rwi = telecom_system->data_container.ring_write_index;
		memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
			&telecom_system->data_container.passband_delayed_data[rwi + tail_offset],
			tail_samples * sizeof(double));

		telecom_system->data_container.data_ready = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);

		int matched_count = 0;
		int suffix_matched = 0;
		int suffix_start = telecom_system->ack_mfsk.hail_directed
		                  ? telecom_system->ack_mfsk.ack_pattern_nsymb : 0;
		double metric = telecom_system->detect_hail_pattern_from_passband(
			telecom_system->data_container.ready_to_process_passband_delayed_data,
			tail_samples, &matched_count, suffix_start, &suffix_matched);

		// Separate base pattern and suffix verification:
		// Base: enough Sidelnikov symbols match (same for all stations)
		// Suffix: callsign-derived tones must match independently (directed HAIL only)
		int base_matched = matched_count - suffix_matched;
		bool base_ok = base_matched >= telecom_system->ack_mfsk.hail_match_threshold;
		bool suffix_ok = !telecom_system->ack_mfsk.hail_directed
		              || suffix_matched >= (telecom_system->ack_mfsk.HAIL_SUFFIX_LEN - 1);
		// Per-match quality (diagnostic only now — see HAIL-detection-floor
		// fact-doc §9/§10): noise gives metric/matched ≈ 2/Nc (0.2 NB, 0.04 WB).
		// This value is NO LONGER a gate: the old quality>=0.3 gate sat ~8 dB
		// above the matched-count floor (at matched=16 it implied metric>=4.8,
		// stricter than the metric gate) and contributed ≈0 FAR — the base_ok
		// count gate (8/16 WB, 24-40 NB) is the load-bearing FAR defense
		// (measured 0/5000, fact-doc §4). Retained only for [HAIL-POLL] /
		// [HAIL] Detected diagnostics below.
		double quality = (matched_count > 0) ? metric / matched_count : 0.0;
		// HAIL-POLL diagnostic: opt-in via MERCURY_HAIL_POLL=1 env var.
		// Logs near-threshold polls only (≥40% base match OR metric ≥2.0 OR
		// quality ≥0.2) to distinguish "no signal" (Pi state drift) from
		// "signal present but degraded" without flooding logs. Cached on
		// first call so we don't pay getenv() cost in the hot poll loop.
		static int hail_poll_enabled = -1;
		if(hail_poll_enabled < 0)
			hail_poll_enabled = (getenv("MERCURY_HAIL_POLL") != nullptr) ? 1 : 0;
		// DRIFT-INSTR: opt-in via MERCURY_DRIFT_INSTR=1. Logs the suspect
		// state variables from fact-documents/pi-audio-drift-audit.md every
		// 60 seconds at the HAIL poll site. Use during long soak tests to
		// catch which value diverges at drift onset.
		static int drift_instr_enabled = -1;
		if(drift_instr_enabled < 0)
			drift_instr_enabled = (getenv("MERCURY_DRIFT_INSTR") != nullptr) ? 1 : 0;
		if(drift_instr_enabled)
		{
			static long long last_drift_log_ms = 0;
			struct timespec ts;
			clock_gettime(CLOCK_MONOTONIC, &ts);
			long long now_ms = (long long)ts.tv_sec * 1000LL + (long long)ts.tv_nsec / 1000000LL;
			if(now_ms - last_drift_log_ms >= 60000LL)
			{
				last_drift_log_ms = now_ms;
				size_t heap_uordblks = 0;
#ifdef __GLIBC__
				struct mallinfo2 mi = mallinfo2();
				heap_uordblks = (size_t)mi.uordblks;
#endif
				int sp_now = telecom_system->data_container.Nofdm
				           * (int)telecom_system->data_container.buffer_Nsymb
				           * telecom_system->data_container.interpolation_rate;
				printf("[DRIFT-INSTR] t=%lld lcfo=%.4f pss=%lu nv=%.4e rwi=%d sp=%d nupe=%d uordblks=%zu\n",
					now_ms,
					telecom_system->last_coarse_freq_offset,
					telecom_system->ofdm.passband_start_sample,
					telecom_system->ofdm.noise_variance_estimate,
					(int)telecom_system->data_container.ring_write_index,
					sp_now,
					(int)telecom_system->data_container.nUnder_processing_events,
					heap_uordblks);
				fflush(stdout);
			}
		}
		bool near_threshold = base_matched >= (telecom_system->ack_mfsk.hail_match_threshold * 4 / 10)
		                   || metric >= 2.0
		                   || quality >= 0.2;
		if(hail_poll_enabled && near_threshold)
		{
			printf("[HAIL-POLL] base=%d/%d suffix=%d/%d metric=%.2f quality=%.2f%s%s%s\n",
				base_matched, telecom_system->ack_mfsk.hail_match_threshold,
				suffix_matched, telecom_system->ack_mfsk.HAIL_SUFFIX_LEN,
				metric, quality,
				base_ok ? " base_ok" : "",
				suffix_ok ? " suffix_ok" : "",
				telecom_system->ack_mfsk.hail_directed ? " (directed)" : "");
			fflush(stdout);
		}
		// Gate = base count (FAR defense) + directed-suffix count + the
		// config-tuned detection metric floor. Aligned with the sibling HAIL
		// receive() site (arq_common.cc:6373) and the ACK/BREAK/CONNECT
		// consumers, which all use ack_pattern_detection_threshold (0.65 at
		// ROBUST_0, telecom_system.cc:5505-5510) — NOT the old hardcoded 3.0.
		// The old metric>=3.0 && quality>=0.3 soft gates cost ~8 dB of
		// establishment reach for ≈0 FAR benefit (HAIL-detection-floor §4/§5/§9).
		if(base_ok && suffix_ok && metric >= telecom_system->ack_pattern_detection_threshold)
		{
			printf("[HAIL] Detected: base=%d/%d suffix=%d/%d metric=%.1f quality=%.2f%s\n",
				base_matched, telecom_system->ack_mfsk.hail_match_threshold,
				suffix_matched, telecom_system->ack_mfsk.HAIL_SUFFIX_LEN,
				metric, quality,
				telecom_system->ack_mfsk.hail_directed ? " (directed)" : "");
			fflush(stdout);
#ifdef MERCURY_GUI_ENABLED
			gui_push_monitor_event("[HAIL detected]", false);
#endif
			MUTEX_LOCK(&capture_prep_mutex);
			telecom_system->data_container.frames_to_read =
				telecom_system->data_container.preamble_nSymb + telecom_system->data_container.Nsymb;
			telecom_system->data_container.nUnder_processing_events = 0;
			telecom_system->receive_stats.mfsk_search_raw = 0;
			telecom_system->receive_stats.ofdm_search_raw = 0;
			telecom_system->receive_stats.ofdm_batch_active = false;
			MUTEX_UNLOCK(&capture_prep_mutex);
			return true;
		}

		telecom_system->data_container.frames_to_read = 2;
		telecom_system->data_container.nUnder_processing_events = 0;
		return false;
	}

	telecom_system->data_container.data_ready = 0;
	MUTEX_UNLOCK(&capture_prep_mutex);
	return false;
}

// Receive and detect ACK tone pattern, returns true if detected.
// Scans the TAIL (newest symbols) of the capture buffer.
// Buffer was zeroed after TX, so the tail contains only fresh audio.
// Called frequently (every ~2 symbols / 45ms) to adapt to any round-trip latency.
// When turbo_snr_ack_enabled, also decodes 8 SNR suffix symbols and stores in turbo_received_snr.
void cl_arq_controller::zero_mfsk_ack_audio_tail()
{
	// §7.13.29 — zero the tail-end of the ring buffer that
	// receive_ack_pattern() just scanned. After a strict (matched>=12)
	// MFSK ACK detection, this prevents the OFDM Schmidl-Cox scan from
	// false-firing on the Welch-Costas tone pattern (which has periodic
	// structure that looks like an OFDM preamble's repeated halves).
	// Mirrors the tail-window math from receive_ack_pattern().
	int ack_nsymb = telecom_system->ack_mfsk.ack_pattern_nsymb;
	int pattern_len = turbo_snr_ack_enabled
		? telecom_system->ack_mfsk.ack_snr_pattern_nsymb()
		: ack_nsymb;
	const int tail_nsymb = ack_nsymb + pattern_len + 16;
	int sym_samples = telecom_system->data_container.Nofdm
	                * telecom_system->data_container.interpolation_rate;
	int signal_period = sym_samples * telecom_system->data_container.buffer_Nsymb;
	int tail_samples = tail_nsymb * sym_samples;
	if(tail_samples > signal_period) tail_samples = signal_period;

	MUTEX_LOCK(&capture_prep_mutex);
	if(telecom_system->data_container.passband_delayed_data != NULL)
	{
		int rwi = telecom_system->data_container.ring_write_index;
		int sp = signal_period;
		int tail_offset = sp - tail_samples;
		// Ring has 2× capacity layout (live + mirror) — zero both halves
		// for each ring position so wrap-around reads see the zeros too.
		for(int k = 0; k < tail_samples; k++)
		{
			int pos = (rwi + tail_offset + k) % sp;
			telecom_system->data_container.passband_delayed_data[pos] = 0.0;
			telecom_system->data_container.passband_delayed_data[pos + sp] = 0.0;
		}
	}
	MUTEX_UNLOCK(&capture_prep_mutex);
}

void cl_arq_controller::commit_ack_pattern_consumed()
{
	// §7.13.29 — apply the audio-advance bookkeeping that
	// receive_ack_pattern(defer_audio_advance=true) deferred. Caller invokes
	// this once it has decided the ACK detection is legitimate (e.g. the v2
	// SACK_RSP cross-check failed to find a SACK_RSP, so the MFSK ACK stands).
	MUTEX_LOCK(&capture_prep_mutex);
	telecom_system->data_container.frames_to_read = 4;
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;
	MUTEX_UNLOCK(&capture_prep_mutex);
}

bool cl_arq_controller::receive_ack_pattern(bool defer_audio_advance,
                                            bool multiwindow_scan)
{
	// Tail must cover the entire fresh audio region (= initial guard).
	// Tail = pattern length + margin + SNR suffix. Ensures ACKs arriving early are captured.
	// Tail must be large enough that even if the ACK is detected late in the buffer,
	// the suffix symbols still fit. Need: ack_pattern_nsymb for search range +
	// full pattern (with suffix) + margin.
	// WB: 16+20+16=52. NB M=8: 32+40+16=88. NB M=4: 48+56+16=120.
	int ack_nsymb = telecom_system->ack_mfsk.ack_pattern_nsymb;
	int pattern_len = turbo_snr_ack_enabled ?
		telecom_system->ack_mfsk.ack_snr_pattern_nsymb() :
		ack_nsymb;
	const int tail_nsymb = ack_nsymb + pattern_len + 16;
	int sym_samples = telecom_system->data_container.Nofdm
	                * telecom_system->data_container.interpolation_rate;
	int signal_period = sym_samples * telecom_system->data_container.buffer_Nsymb;
	int tail_samples = tail_nsymb * sym_samples;
	if(tail_samples > signal_period)
		tail_samples = signal_period;
	int tail_offset = signal_period - tail_samples;

	// ------------------------------------------------------------------
	// SIM_INPROC control-ACK arrival-window re-scan (ADDITIVE; pump-armed
	// poll, single-process-sim-refactor.md §5.7-B8 sibling).
	//
	// In production the ~500 Hz capture-prep thread CONTINUOUSLY refreshes the
	// ring between A's process_main ticks, so the once-per-tick poll below
	// naturally sweeps the entire ACK arrival window: whichever tick the
	// trailing ACK sample reaches the demod, that tick's snapshot sees it.
	//
	// The single-thread stepper has NO prep thread. The peer's control-ACK
	// pattern lands in A's ring exactly ONCE (when the §10.5 deferral releases
	// the rx->tx wire while A is idle), and the pump's idle-silence symbols
	// then push it out of the tail before A's NEXT per-tick scan — so every
	// scan sees a silent tail (tail_rms < gate -> energy gate skips the
	// correlator) and A never advances to NEGOTIATING. Identical class to the
	// already-fixed B8 HAIL RX-poll (arq_commander.cc:428) and the inverted
	// spin-loops: the detector is fine, the poll just never sees the buffer at
	// the instant the reply lands.
	//
	// FIX (B8 mirror): when the step-pump is installed (ONLY under -m
	// SIM_INPROC), pump the SHARED virtual clock forward in short slices and
	// re-probe the tail until the ACK pattern's ENERGY is present in a
	// scannable state (frames_to_read==0 AND tail RMS over the gate), or a
	// bounded arrival window elapses. The pump drains peer->A through the wire
	// + advances the clock through the SAME rx_transfer accounting the prep
	// thread uses (and co-routine-drives the peer so it actually emits the
	// reply), so this is the in-thread equivalent of letting the prep thread
	// refresh the ring across the window. The UNCHANGED detection body below
	// then runs exactly as in production — SAME energy gate, SAME correlator,
	// SAME threshold (ack_match_threshold + ack_metric_threshold), SAME
	// frames_to_read / defer_audio_advance bookkeeping. We only change WHEN the
	// poll sees the buffer, never WHETHER it accepts. On a real miss the window
	// expires and the body's normal miss path (frames_to_read=2; return false)
	// fires verbatim, so A re-polls next tick exactly as in production.
	//
	// Production + the two-process paced sim leave the pump null
	// (arq_sim_inproc_active()==false) -> this whole block is skipped ->
	// byte-identical.
	//
	// STEPPER-CORE REWRITE Phase b: this pump-armed control-ACK re-scan RUNS under the
	// outer stepper too (pumped_settle_wait still pumps for handshake/control-ACK
	// turnarounds — see its Phase-b note). It delivers the peer's control-ACK reply
	// INTRA-process_main so A's same-call poll sees it (the outer loop cannot inject
	// delivery mid-process_main). These are MFSK / control frames; the burst is harmless.
	if(arq_sim_inproc_active())
	{
		// Arrival window: tail span + the responder's PTT/turnaround margin
		// (~1 s, see [TX-ACK-PAT] guard) expressed in virtual ms, capped so a
		// genuinely silent line still returns within one ack-poll cadence.
		double sym_ms = (sym_samples > 0)
			? (double)sym_samples * 1000.0 / (double)SIM_CLOCK_SAMPLE_RATE_HZ
			: 1.0;
		int window_ms = (int)((double)tail_nsymb * sym_ms) + 1200;
		const double SIM_ACK_PROBE_GATE_RMS = 0.001; // == ACK_ENERGY_GATE_RMS
		int elapsed_ms = 0;
		// Slice = 2 symbols (== the body's frames_to_read=2 re-poll cadence),
		// so the pump delivers whole symbols the way the prep thread would.
		int slice_ms = (int)(2.0 * sym_ms) + 1;
		// Probe the WHOLE tail (not just the last 8 symbols the body's CPU-gate
		// uses). The stepper delivers the peer's ACK as ONE batch (drains the
		// whole rx->tx wire at once: ACK pattern + the peer's trailing idle
		// silence), so unlike the ~500 Hz prep thread it does NOT leave the ACK
		// in the last few symbols — the ACK sits MID-tail with ~10-15 symbols of
		// trailing silence. A last-8-symbol probe would therefore always read
		// silence; we break as soon as the ACK ENERGY appears anywhere in the
		// tail (the same span the correlator below searches). Pumping FURTHER
		// only adds trailing silence and drifts the ACK out of the tail, so we
		// must stop the instant energy is present — pump only to WAIT for it.
		while(elapsed_ms < window_ms)
		{
			bool scannable = false;
			MUTEX_LOCK(&capture_prep_mutex);
			if(telecom_system->data_container.frames_to_read == 0)
			{
				int rwi = telecom_system->data_container.ring_write_index;
				double* tp = &telecom_system->data_container
					.passband_delayed_data[rwi + tail_offset];
				double sumsq = 0.0;
				for(int i = 0; i < tail_samples; i++) sumsq += tp[i] * tp[i];
				double rms = std::sqrt(sumsq / tail_samples);
				if(rms >= SIM_ACK_PROBE_GATE_RMS)
					scannable = true;   // ACK energy is in the tail NOW — scan it
			}
			MUTEX_UNLOCK(&capture_prep_mutex);
			if(scannable)
				break;                  // run the UNCHANGED detection body below
			pumped_settle_wait(slice_ms);   // advance shared clock + deliver reply
			elapsed_ms += slice_ms;
		}
	}

	MUTEX_LOCK(&capture_prep_mutex);

	if(telecom_system->data_container.frames_to_read == 0)
	{
		// mw_hit: the multi-window scan validated a real ACK at an older phase
		// (full correlator >= thresholds). When set, the body's 8-symbol energy
		// pre-gate (probe_n) MUST be bypassed: under the paced two-process sim
		// arq_sim_inproc_active()==false leaves probe_n=8*sym_samples, which only
		// samples the LAST 8 symbols of the snapshot — but a MW-repositioned ACK
		// sits MID-tail, so that 8-symbol gate would read silence and skip the
		// (already-validated) correlator, dropping the detection. mw_hit forces
		// the body past the pre-filter; the body's UNCHANGED correlator then
		// re-confirms on the chosen phase and returns true.
		bool mw_hit = false;
		// ------------------------------------------------------------------
		// CMD multi-window control-ACK match (CONNECT round-2 fix #2(a)).
		// CONNECT_FINAL §3/§7, SUBMODE_ROOTCAUSE §7 #2(a), _connect2/CMD_MULTIWINDOW_DESIGN.md.
		//
		// The newest-tail snapshot below correlates ONLY the last `tail_nsymb`
		// (=80 on CONFIG_100 NB) symbols. On the slow 2-phase MFSK handshake the
		// peer's control ACK arrives once, then trailing idle silence scrolls it
		// out of the newest tail before this poll's ftr==0 snapshot fires — so
		// every snapshot reads silence (CMD [CAP-PEAK] pk=0.000000) and the CMD
		// never advances (sub-mode B, all snr900). The ACK is NOT lost: the ring
		// retains ~buffer_Nsymb (1301) symbols of history, far more than the ACK
		// round-trip, so the burst still sits at an OLDER phase.
		//
		// When the caller asks for a multi-window scan (the CONNECT control-ACK
		// wait only, arq_commander.cc:1981), step the search phase back through
		// the retained ring in ACK-stride increments and pick the FIRST older
		// phase whose tail RMS clears the energy gate (signal present). We only
		// MOVE where the snapshot reads; the UNCHANGED detection body below then
		// runs verbatim on that phase — SAME energy gate, SAME correlator, SAME
		// ack_match_threshold + ack_metric_threshold. A genuine miss (silence at
		// every phase) leaves tail_offset at the newest tail and falls through to
		// the normal miss path (ftr=2; return false), re-polled next tick exactly
		// as before. This does NOT restart receiving_timer and does NOT shrink ftr
		// (the §2 happy-path invariant). Production/paced DATA-ACK/BREAK/HAIL pass
		// multiwindow_scan==false -> byte-identical.
		//
		// The double-mapped ring (passband_delayed_data sized 2*signal_period,
		// data_container.cc:170) makes [rwi + off] for off in [0, tail_offset] a
		// contiguous, in-bounds tail via the mirror copy.
		if(multiwindow_scan && tail_offset > 0)
		{
			const int rwi_mw = telecom_system->data_container.ring_write_index;
			const double MW_GATE_RMS = 0.001; // == ACK_ENERGY_GATE_RMS below
			// Stride one ACK-pattern length per phase so consecutive search
			// windows overlap by the full search range (no ACK can fall entirely
			// between two phases). Bound the number of older phases to the ACK
			// round-trip neighbourhood (not the whole 1301-symbol ring) to keep
			// the per-poll correlator cost modest.
			int stride = pattern_len * sym_samples;
			if(stride < sym_samples) stride = sym_samples;
			const int MW_MAX_PHASES = 24; // ~24*pattern_len symbols of look-back
			int chosen_off = -1;
			for(int ph = 1; ph <= MW_MAX_PHASES; ph++)
			{
				int off = tail_offset - ph * stride;
				if(off < 0) break;
				// Energy pre-gate (cheap): only correlate phases that hold signal.
				const double* pp = &telecom_system->data_container
					.passband_delayed_data[rwi_mw + off];
				double sumsq = 0.0;
				for(int i = 0; i < tail_samples; i++) sumsq += pp[i] * pp[i];
				double rms = std::sqrt(sumsq / tail_samples);
				if(rms < MW_GATE_RMS) continue;
				// Run the SAME correlator on this older phase (turbo/non-turbo
				// share the same accept thresholds; we use the plain ACK
				// correlator as the gate — the body below re-runs the exact
				// detector and bookkeeping for the chosen phase).
				memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
					pp, tail_samples * sizeof(double));
				int mw_matched = 0; uint32_t mw_mask = 0;
				double mw_metric = telecom_system->detect_ack_pattern_from_passband(
					telecom_system->data_container.ready_to_process_passband_delayed_data,
					tail_samples, &mw_matched, &mw_mask);
				if(mw_matched >= telecom_system->ack_mfsk.ack_match_threshold
				   && mw_metric >= ack_metric_threshold)
				{
					chosen_off = off;
					printf("[CMD-ACK-MW] control-ACK found at older phase off=%d "
						"(newest_tail_off=%d phase=%d matched=%d metric=%.2f)\n",
						off, tail_offset, ph, mw_matched, mw_metric);
					fflush(stdout);
					break;
				}
			}
			if(chosen_off >= 0)
			{
				tail_offset = chosen_off; // body below snapshots+detects this phase
				mw_hit = true;            // bypass the 8-symbol energy pre-gate
			}
		}
		// Snapshot only the tail (newest audio) — smaller copy, shorter mutex hold
		int rwi = telecom_system->data_container.ring_write_index;
		memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
			&telecom_system->data_container.passband_delayed_data[rwi + tail_offset],
			tail_samples * sizeof(double));

		telecom_system->data_container.data_ready = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);

		int matched_count = 0;

		// Energy gate (Opt 1): the ACK matched filter runs ~528 FFTs per call,
		// dominating CPU during idle polling. Skip the entire detection when
		// the recent audio tail has no signal. Floor measured at -73 dBFS
		// (≈0.0002 RMS); a single MFSK tone is at ≈0.02 RMS. Gate at 0.001
		// (≈14 dB above noise) — well below the smallest real ACK level but
		// well above silence/rx_mute noise.
		// Per-window reset of energy-log tracker (diagnostic).
		static bool energy_logged_this_window = false;
		if(ack_diag_poll_count == 0)
		{
			mtl::log_event("cmd_first_ack_poll");
			energy_logged_this_window = false;
		}
		const double ACK_ENERGY_GATE_RMS = 0.001;
		// Energy-gate window: last 8 symbols in production (CPU pre-filter — the
		// prep thread leaves a freshly-arrived ACK in the newest symbols). Under
		// the SIM_INPROC pump the peer's ACK is delivered as ONE batch (ACK +
		// trailing idle silence), so it sits MID-tail; probing only the last 8
		// symbols would read silence and skip a tail that DOES contain the ACK.
		// Widen the GATE to the full tail under SIM so it does not pre-filter out
		// a present ACK. This changes ONLY the CPU pre-filter span — the
		// correlator below already searches the whole tail and its accept
		// threshold (ack_match_threshold + ack_metric_threshold) is unchanged, so
		// detection semantics are identical. Production/paced-sim keep the 8-symbol
		// window (pump null) -> byte-identical.
		int probe_n = arq_sim_inproc_active() ? tail_samples : (8 * sym_samples);
		if(probe_n > tail_samples) probe_n = tail_samples;
		double* tail_ptr = telecom_system->data_container.ready_to_process_passband_delayed_data
			+ (tail_samples - probe_n);
		double sumsq = 0.0;
		for(int i = 0; i < probe_n; i++) sumsq += tail_ptr[i] * tail_ptr[i];
		double tail_rms = std::sqrt(sumsq / probe_n);
		if(!energy_logged_this_window && tail_rms > 0.005)
		{
			mtl::log_event_kv("cmd_ack_buffer_energy", "rms=%.4f", tail_rms);
			energy_logged_this_window = true;
		}
		if(!mw_hit && tail_rms < ACK_ENERGY_GATE_RMS)
		{
			// Silent buffer — skip FFT-heavy ACK search this poll. (mw_hit
			// bypasses this pre-filter: the MW scan already validated a real
			// ACK at this phase with the full correlator; the body's correlator
			// below re-confirms it. Without the bypass, the 8-symbol pre-gate
			// reads the silent last-8 of a mid-tail ACK and drops the detection.)
			ack_diag_poll_count++;
			MUTEX_LOCK(&capture_prep_mutex);
			telecom_system->data_container.frames_to_read = 2;
			telecom_system->data_container.nUnder_processing_events = 0;
			MUTEX_UNLOCK(&capture_prep_mutex);
			return false;
		}

		if(turbo_snr_ack_enabled)
		{
			// Turboshift mode: detect ACK and decode SNR suffix.
			// The suffix symbols arrive AFTER the ACK pattern. The ACK is
			// detected when 24+ of 32 symbols are in the buffer, but the
			// 8 suffix symbols may still be in flight (~200ms NB, ~100ms WB).
			// When ACK is found but suffix is unreadable, defer and keep
			// polling until the suffix arrives (up to 500ms timeout).
			bool snr_valid = false;
			float decoded_snr = telecom_system->detect_ack_snr_from_passband(
				telecom_system->data_container.ready_to_process_passband_delayed_data,
				tail_samples, &matched_count, &snr_valid);

			if(matched_count >= telecom_system->ack_mfsk.ack_match_threshold)
			{
				// Step 15: legacy MFSK SACK-before-ACK guard removed —
				// OFDM SACK_RSP cannot false-trigger the MFSK ACK correlator
				// (different waveform); the SACK pattern correlator that
				// produced the contention is gone.

				if(snr_valid)
				{
					turbo_received_snr = decoded_snr;
					if(decoded_snr > turbo_best_snr)
						turbo_best_snr = decoded_snr;
					// SUPERSHIFT SNR-sentinel fix (climb follow-up #1, Option A;
					// data-flow-snr-measurements.md §1.5). ALSO populate
					// measurements.SNR_uplink so the SUPERSHIFT re-trigger gate
					// (arq_commander.cc, "measurements.SNR_uplink > -90") becomes
					// eligible mid-climb. The canonical producer (:6051) writes
					// SNR_uplink for ALL roles from a decoded LDPC frame, but the
					// CMD decodes NO LDPC data on the forward pattern-ACK climb, so
					// SNR_uplink would otherwise stay at the ctor sentinel -99.9 and
					// the elevator could never engage (slow one-rung ladder climb).
					// This path is CMD-only (receive_ack_pattern is CMD-only), so we
					// write SNR_uplink only — matching :6051's CMD-role semantics
					// (SNR_downlink is RESPONDER-only there). Same decoded value the
					// suffix carries. Accepted tradeoff: the value goes stale after
					// turbo (Option B's steady-state ACK suffix is NOT done here).
					measurements.SNR_uplink = snr_uplink_from_suffix(decoded_snr);
					turbo_snr_defer_timer.reset();
					printf("[CMD-ACK-SNR] ACK detected with SNR=%.1f dB (matched=%d)\n",
						decoded_snr, matched_count);
					fflush(stdout);

#ifdef MERCURY_GUI_ENABLED
					gui_push_monitor_event("[ACK+SNR]", false);
#endif
					if(defer_audio_advance)
					{
						// §7.13.29 — leave ring untouched for follow-up
						// SACK_RSP cross-check; caller invokes
						// commit_ack_pattern_consumed() if it accepts the ACK.
						MUTEX_LOCK(&capture_prep_mutex);
						telecom_system->data_container.frames_to_read = 0;
						MUTEX_UNLOCK(&capture_prep_mutex);
					}
					else
					{
						MUTEX_LOCK(&capture_prep_mutex);
						telecom_system->data_container.frames_to_read = 4;
						telecom_system->data_container.nUnder_processing_events = 0;
						telecom_system->receive_stats.mfsk_search_raw = 0;
						telecom_system->receive_stats.ofdm_search_raw = 0;
						telecom_system->receive_stats.ofdm_batch_active = false;
						MUTEX_UNLOCK(&capture_prep_mutex);
					}
					return true;
				}
				else
				{
					// ACK detected but suffix not yet arrived. Start/check defer timer.
					if(turbo_snr_defer_timer.counting != COUNTING)
					{
						turbo_snr_defer_timer.start();
						printf("[CMD-ACK-SNR] ACK detected, waiting for suffix (matched=%d)\n",
							matched_count);
						fflush(stdout);
					}
					else if(turbo_snr_defer_timer.get_elapsed_time_ms() > 500)
					{
						// Timeout: accept ACK without SNR
						turbo_received_snr = -99.0f;
						turbo_snr_defer_timer.reset();
						printf("[CMD-ACK-SNR] ACK detected, suffix timeout (matched=%d)\n",
							matched_count);
						fflush(stdout);

#ifdef MERCURY_GUI_ENABLED
						gui_push_monitor_event("[ACK+SNR]", false);
#endif
						if(defer_audio_advance)
						{
							// §7.13.29 — defer the ring advance; caller commits.
							MUTEX_LOCK(&capture_prep_mutex);
							telecom_system->data_container.frames_to_read = 0;
							MUTEX_UNLOCK(&capture_prep_mutex);
						}
						else
						{
							MUTEX_LOCK(&capture_prep_mutex);
							telecom_system->data_container.frames_to_read = 4;
							telecom_system->data_container.nUnder_processing_events = 0;
							telecom_system->receive_stats.mfsk_search_raw = 0;
							telecom_system->receive_stats.ofdm_search_raw = 0;
							telecom_system->receive_stats.ofdm_batch_active = false;
							MUTEX_UNLOCK(&capture_prep_mutex);
						}
						return true;
					}
					// else: keep polling, suffix not yet in buffer
					return false;
				}
			}
			else
			{
				// No ACK detected at all — reset defer timer if it was running
				if(turbo_snr_defer_timer.counting == COUNTING &&
				   turbo_snr_defer_timer.get_elapsed_time_ms() > 500)
					turbo_snr_defer_timer.reset();

				// ACK poll diagnostic removed — printf/fflush in hot polling loop caused ~175-875ms cumulative latency
			}
		}
		else
		{
			// Normal mode: just detect ACK pattern
			uint32_t this_mask = 0;
			// Phase D timing — instrument ACK FFT CPU cost. The
			// detect_ack_pattern_from_passband call runs ~528 FFTs on the M=16
			// MFSK pattern. Compare cumulative FFT CPU to the 774ms ACK dwell
			// observed in wgn22 traces to tell wire-wait vs CPU dominance.
			long long _fft_t0_ms = mtl::now_ms();
			double metric = telecom_system->detect_ack_pattern_from_passband(
				telecom_system->data_container.ready_to_process_passband_delayed_data,
				tail_samples, &matched_count, &this_mask);
			mtl::log_event_kv("cmd_ack_fft", "cpu_ms=%lld matched=%d metric=%.2f",
				mtl::now_ms() - _fft_t0_ms, matched_count, metric);

			// Track peak detection values for timeout diagnostic (no printf in hot loop)
			if(matched_count > ack_diag_peak_matched)
			{
				ack_diag_peak_matched = matched_count;
				ack_diag_peak_mask = this_mask;
			}
			if(metric > ack_diag_peak_metric) ack_diag_peak_metric = metric;
			ack_diag_poll_count++;

			// Metric threshold: For WB M=16, matched>=8/16 has P(false)~5.6e-5/pos.
			// Random noise has metric≈8/Nc=0.16 at 8 matches. metric>=0.5 rejects
			// noise while accepting marginal signals (was 3.0, caused ~50% timeouts).
			// Phase-2: --ack-metric-threshold=F overrides.
			if(matched_count >= telecom_system->ack_mfsk.ack_match_threshold && metric >= ack_metric_threshold)
			{
				// Step 15: legacy MFSK SACK-before-ACK guard removed —
				// OFDM SACK_RSP cannot false-trigger the MFSK ACK correlator,
				// and the SACK pattern correlator that drove the cross-check
				// no longer exists. (Bug C v2.1 cross-check still runs in
				// arq_commander.cc:1789-1840 against decode_sack_v2_frame.)

#ifdef MERCURY_GUI_ENABLED
				gui_push_monitor_event("[ACK]", false);
#endif
				if(defer_audio_advance)
				{
					// §7.13.29 — leave ring untouched for follow-up SACK_RSP
					// cross-check; caller invokes commit_ack_pattern_consumed()
					// if it accepts the ACK.
					MUTEX_LOCK(&capture_prep_mutex);
					telecom_system->data_container.frames_to_read = 0;
					MUTEX_UNLOCK(&capture_prep_mutex);
				}
				else
				{
					MUTEX_LOCK(&capture_prep_mutex);
					telecom_system->data_container.frames_to_read = 4;
					telecom_system->data_container.nUnder_processing_events = 0;
					telecom_system->receive_stats.mfsk_search_raw = 0;
					telecom_system->receive_stats.ofdm_search_raw = 0;
					telecom_system->receive_stats.ofdm_batch_active = false;
					MUTEX_UNLOCK(&capture_prep_mutex);
				}
				return true;
			}
		}

		// Not detected — poll again in 2 symbols (~45ms). Step 15: legacy
		// receive_sack_pattern() chain is gone; this throttle now only gates
		// the ACK-pattern check.
		telecom_system->data_container.frames_to_read = 2;
		telecom_system->data_container.nUnder_processing_events = 0;
		return false;
	}

	telecom_system->data_container.data_ready = 0;
	MUTEX_UNLOCK(&capture_prep_mutex);
	return false;
}

void cl_arq_controller::receive()
{
	// BREAK forward-health gate (fix/break-fh-gate): monotonic receive() iteration
	// counter. ALWAYS maintained but read ONLY by the env-gated FH suppressor, so
	// when MERCURY_BREAK_FH_GATE is unset nothing consumes it -> byte-identical.
	rx_receive_frame_index++;

	int signal_period = telecom_system->data_container.Nofdm * telecom_system->data_container.buffer_Nsymb * telecom_system->data_container.interpolation_rate; // in samples
	int symbol_period = telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate;

#if 0 // TODO:  do we need this?
	if(telecom_system->data_container.data_ready == 0)
	{
		msleep(1);
		return;
	}
#endif
	MUTEX_LOCK(&capture_prep_mutex);
	st_receive_stats received_message_stats;


	if(telecom_system->data_container.frames_to_read==0)
	{


		int rwi = telecom_system->data_container.ring_write_index;
		memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data, &telecom_system->data_container.passband_delayed_data[rwi], signal_period * sizeof(double));

		// ===== D3 ACQUISITION-NONDETERMINISM SIM INJECTION (fix/bigblock-d3-carve) =====
		// The in-process SIM_INPROC big-block path delivers 1200/1200 deterministically because
		// its single-symbol pacing + decode-drive lands the block at a CONSTANT, frame-aligned
		// head every run, the carve ZEROES the whole ring after each block (no stale carry), and
		// the CMD emits each block exactly once (no co-resident retransmit copy). HW has NONE of
		// those properties (independent Pi audio clocks => arbitrary block-arrival phase; a
		// continuously-running ring carrying the previous burst; and multiple co-resident retx
		// copies when no SACK is accepted). So D3 — (1) the rwi-relative carve PHASE LOTTERY
		// (head must be <= cap-block_span to fit, bigblock_acq_window_fits arq_common.cc:4226),
		// (2) the global energy-argmax FALSE-LOCK onto a fresher retransmit copy (ofdm.cc:2685,
		// early_exit=0.0 at telecom_system.cc:7434), and (3) the INVERTED §22 wait-for-tail re-arm
		// (:7460 pushes the head LATER on the rwi-relative snapshot) — is SILENT in sim. These
		// three env-gated impairments reproduce the HW acquisition behavior so the D3 fix can be
		// validated off-bench. Production NEVER sets these vars => zero production effect; the
		// block also gates on CFG16 + big-block framing so the stock per-frame path is untouched.
		// The snapshot is the SINGLE chokepoint: receive_byte->receive_bigblock acquisition
		// (telecom_system.cc:7433 time_sync over Nofdm*buf_syms of THIS buffer) AND the carve both
		// read ready_to_process_passband_delayed_data, so mutating it here moves the head for both.
		if(telecom_system->bigblock_framing_enabled
		   && telecom_system->M != MOD_MFSK
		   && current_configuration == CONFIG_16)
		{
			// Read the env EACH PASS (this CFG16+big-block gate is rare relative to the FFT
			// decode that follows, and test_sim_inproc_bigblock_multicw toggles these vars
			// between arms in ONE process — a cached-once read would lock arm A's values).
			int d3_ringphase = -3;   // -3 = disabled; -1 = random; >=0 = fixed right-shift samples
			const char* e = std::getenv("MERCURY_BIGBLOCK_SIM_RINGPHASE");
			if(e && *e){
				if(std::string(e)=="rand" || atoi(e)<0) d3_ringphase = -1;
				else d3_ringphase = atoi(e);
			}
			const char* es = std::getenv("MERCURY_BIGBLOCK_SIM_STALERING");
			int d3_stalering = (es && *es && atoi(es)!=0) ? 1 : 0;
			const char* er = std::getenv("MERCURY_BIGBLOCK_SIM_RETXCOPY");
			int d3_retxcopy = (er && *er && atoi(er)!=0) ? 1 : 0;
			// xorshift32 RNG state for the random-phase lottery. Re-seeded whenever the seed env
			// changes (per-arm determinism) so each arm's randomness is reproducible.
			static unsigned long d3_rng = 0;
			static std::string   d3_seed_seen = "\x01";   // sentinel "never set"
			const char* ee = std::getenv("MERCURY_BIGBLOCK_SIM_RINGSEED");
			std::string seed_now = (ee && *ee) ? std::string(ee) : std::string("");
			if(seed_now != d3_seed_seen){
				d3_seed_seen = seed_now;
				d3_rng = seed_now.empty() ? 0x9E3779B9UL : (unsigned long)strtoul(seed_now.c_str(),nullptr,10);
				if(d3_rng == 0) d3_rng = 0x9E3779B9UL;
			}
			bool d3_active = (d3_ringphase != -3) || d3_stalering==1 || d3_retxcopy==1;
			if(d3_active && signal_period > 0)
			{
				double* snap = telecom_system->data_container.ready_to_process_passband_delayed_data;
				int sp = signal_period;
				// Pick the per-snapshot right-shift (how much LATER the block lands in the window).
				int shift;
				if(d3_ringphase == -1){
					// xorshift32 LCG: uniformly random phase in [0,sp) — the independent-clock lottery.
					d3_rng ^= d3_rng << 13; d3_rng ^= d3_rng >> 17; d3_rng ^= d3_rng << 5;
					shift = (int)(d3_rng % (unsigned long)sp);
				} else if(d3_ringphase >= 0){
					shift = d3_ringphase % sp;
				} else {
					shift = 0;   // phase disabled, but stale/retx still apply
				}
				// (1)+(2): build the impaired snapshot in a scratch buffer.
				//   - RINGPHASE: the real block content moves to [shift, shift+sp) (right-shift),
				//     so its head_delay grows by `shift`. Content past sp is LOST (the HW overrun:
				//     the tail is FUTURE/unproduced => bb_at zero-pads it, telecom_system.cc:7515).
				//   - STALERING: the vacated front [0,shift) is filled with the PREVIOUS snapshot
				//     (stale resident audio) instead of silence — so the window is never clean.
				//   - RETXCOPY: a FRESHER (slightly higher-energy) copy of the original block head
				//     is laid near the ring END so the global energy-weighted argmax (early_exit=0)
				//     locks onto IT (a future-tailed copy) instead of the earlier real block.
				static std::vector<double> d3_prev;          // last snapshot (stale-ring source)
				std::vector<double> orig(snap, snap + sp);   // the real (constant-phase) block
				std::vector<double> out(sp, 0.0);
				if(d3_stalering==1 && (int)d3_prev.size()==sp)
					for(int i=0;i<sp;i++) out[i] = d3_prev[i];   // start from the stale burst
				// right-shift the real block by `shift`, dropping the overrun tail.
				for(int i=0; i+shift < sp; i++) out[i+shift] = orig[i];
				if(d3_retxcopy==1){
					// FALSE-LOCK model: the CMD re-emits the block when no SACK is accepted, so 2+
					// copies co-reside in the ring (trace_falselock §1, val_cmd_off_A1.log three
					// "one-block emit bsi=5"). The big-block time_sync passes early_exit=0
					// (telecom_system.cc:7434) so time_sync_preamble_halfsym (ofdm.cc:2646-2691)
					// returns the GLOBAL energy-weighted argmax, NOT the earliest preamble — with
					// multiple copies it picks the FRESHEST/LATEST (least channel-decayed => highest
					// energy) whose PREAMBLE is in-window but whose BODY tail is future. Lay such a
					// copy: a LATER head (rpos, in-window so its preamble wins the argmax) at 1.6x
					// amplitude (fresher), its body overrunning the window end (future => bb_at
					// zero-pads the late codewords => demote). The real (earlier) block stays too,
					// but the argmax false-locks the louder later copy -> head jumps FORWARD (the
					// HW 116664->132932 forward drift), exactly the §22-can't-recover false-lock.
					int rpos = (sp/2);   // ~half-way: preamble fully in-window, body overruns the end
					for(int i=0; i+rpos < sp && i < sp; i++) out[i+rpos] += 1.6 * orig[i];
				}
				memcpy(snap, out.data(), (size_t)sp * sizeof(double));
				d3_prev.assign(snap, snap + sp);   // remember for the next snapshot's stale carry
				static int d3_logn = 0;
				if(d3_logn < 64){
					d3_logn++;
					printf("[BBTX-SIM-D3] inject ringphase=%s shift=%d stalering=%d retxcopy=%d sp=%d\n",
						(d3_ringphase==-1?"rand":(d3_ringphase>=0?"fixed":"off")),
						shift, d3_stalering, d3_retxcopy, sp);
					fflush(stdout);
				}
			}
		}

		// DIAG: ring buffer snapshot debug (verbose only — buffer scan is expensive)
		if(g_verbose)
		{
			double peak_head = 0, peak_mid = 0, peak_tail = 0;
			int sp = signal_period;
			for(int i = 0; i < sp/10 && i < sp; i++) {
				double v = fabs(telecom_system->data_container.ready_to_process_passband_delayed_data[i]);
				if(v > peak_head) peak_head = v;
			}
			for(int i = sp/2 - sp/20; i < sp/2 + sp/20 && i < sp; i++) {
				double v = fabs(telecom_system->data_container.ready_to_process_passband_delayed_data[i]);
				if(v > peak_mid) peak_mid = v;
			}
			for(int i = sp - sp/10; i < sp; i++) {
				double v = fabs(telecom_system->data_container.ready_to_process_passband_delayed_data[i]);
				if(v > peak_tail) peak_tail = v;
			}
			static int snap_count = 0;
			snap_count++;
			printf("[RING-SNAP] #%d rwi=%d sp=%d head=%.4f mid=%.4f tail=%.4f nUnder=%d M=%.0f\n",
				snap_count, rwi, sp, peak_head, peak_mid, peak_tail,
				telecom_system->data_container.nUnder_processing_events.load(),
				telecom_system->M);
			fflush(stdout);
		}

		// Previously: zero passband_delayed_data after copy to prevent stale
		// preamble re-detection from self-echo. Now handled by rx_mute (Bug #44):
		// capture thread zeros audio during TX, so self-echo never enters buffer.
		// Removing zeroing is critical for OFDM gearshift: after ACK TX + flush,
		// CMD turnaround takes ~1-2s. With zeroing + ftr=8 anti-spin, only 8
		// symbols accumulate before the next attempt zeros everything — signal
		// never builds up for a full 52-symbol frame. Without zeroing, signal
		// accumulates across attempts until a full frame is available.

		// Clear data_ready while we have the lock, before unlocking
		telecom_system->data_container.data_ready = 0;

		MUTEX_UNLOCK(&capture_prep_mutex);

		if(telecom_system->M != MOD_MFSK && g_verbose)
		{
			int sym_samples = telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate;
			int buf_nsymb = telecom_system->data_container.buffer_Nsymb;
			int chunk_symb = (buf_nsymb + 9) / 10;
			int chunk_samples = chunk_symb * sym_samples;
			printf("[BUF-ENERGY] nUnder=%d |", telecom_system->data_container.nUnder_processing_events.load());
			for(int c = 0; c < signal_period; c += chunk_samples)
			{
				double peak = 0.0;
				int end = (c + chunk_samples < signal_period) ? c + chunk_samples : signal_period;
				for(int s = c; s < end; s++)
				{
					double v = fabs(telecom_system->data_container.ready_to_process_passband_delayed_data[s]);
					if(v > peak) peak = v;
				}
				printf(" %.3f", peak);
			}
			printf("\n");
			fflush(stdout);
		}

#ifdef MERCURY_GUI_ENABLED
		// Apply live LDPC iteration limit from GUI.
		// GOTCHA (Q3 per-config override): this overwrite is unconditional and
		// runs after load_configuration(), so a GUI update can clobber the
		// per-config nIteration_max=200 that ROBUST tier configs (100/101/102)
		// install in their load path. If you ever see ROBUST tier underperform
		// at low SNR while the GUI is showing a smaller iteration cap, suspect
		// this line. Safer fix (deferred — medium risk): guard with
		// `!is_robust_config(current_configuration)` so the override only fires
		// for OFDM configs.
		int gui_ldpc_max = g_gui_state.ldpc_iterations_max.load();
		if (gui_ldpc_max >= 5 && gui_ldpc_max <= 50)
			telecom_system->ldpc.nIteration_max = gui_ldpc_max;
#endif

		auto proc_start = std::chrono::steady_clock::now();

		// Monitor mode with parallel decoders: try all 17 OFDM configs sequentially.
		// Falls through to single receive_byte() for MFSK modes or if decoders not ready.
		if(passive_monitor && monitor_decoders_ready && !is_robust_config(current_configuration))
		{
			int winning_config = parallel_monitor_decode(
				telecom_system->data_container.ready_to_process_passband_delayed_data,
				signal_period, received_message_stats);
			if(winning_config >= 0)
			{
				// Switch primary config FIRST (may deinit/reinit data_byte),
				// then copy decoded data from staging buffer.
				if(winning_config != current_configuration)
				{
					printf("[MONITOR] Parallel decode found CONFIG_%d (was CONFIG_%d)\n",
						winning_config, current_configuration);
					data_configuration = winning_config;
					load_configuration(winning_config, PHYSICAL_LAYER_ONLY, YES);
				}
				// Copy decoded data from staging buffer to primary's data_byte.
				// Safe now: load_configuration has finished reinit.
				memcpy(telecom_system->data_container.data_byte,
					monitor_decoded_data, monitor_decoded_len * sizeof(int));
			}
		}
		else
		{
			// WALL-B FIX-3 (C2a): when the CFG16 carve is SUSPENDED (K consecutive cw0-CRC
			// rejects, 0 accepts), force receive_byte onto the STOCK per-frame decoder for
			// this acquisition — the SAME bigblock_rx_force_stock path the GAP-3 cw0-CRC
			// fallback uses — so the carve-parked RSP decodes the CFG16-PHY demote SET_CONFIG
			// (and per-frame CFG16 traffic) normally instead of routing the audio into the
			// K=8 carve that keeps rejecting it. Gated on bigblock_carve_suspended() (CFG16 &&
			// big-block framing && streak>=K), so off-rung / carve-success it is a NO-OP
			// (byte-identical). The block-span re-arm is reverted in lockstep by the
			// bigblock_block_ftr_or chokepoint (C2b), so the stock decode also snapshots at
			// the per-frame cadence the control frame needs.
			bool carve_suspend_force_stock =
				(telecom_system->bigblock_framing_enabled
				 && telecom_system->M != MOD_MFSK
				 && current_configuration == CONFIG_16
				 && bigblock_carve_suspended());
			if(carve_suspend_force_stock)
				telecom_system->bigblock_rx_force_stock = true;
			received_message_stats = telecom_system->receive_byte(
				telecom_system->data_container.ready_to_process_passband_delayed_data,
				telecom_system->data_container.data_byte);
			if(carve_suspend_force_stock)
				telecom_system->bigblock_rx_force_stock = false;
		}

		// STEP 2 — big-block RX carve. When the CFG16-rung framing flag is set and
		// receive_byte branched to receive_bigblock (gated identically), the decode
		// produced ONE block's K-bit cw_ok + the K decoded info-bit sub-units (in
		// data_byte). Translate the block into the ARQ data unit via
		// bigblock_receive_carve -> bigblock_block_to_arq (carve cw_ok -> messages_rx[],
		// synthetic EOB, one ACK / partial SACK / bsi-once) and SKIP the per-frame
		// messages_rx_buffer dispatch below (one acquisition = one carve, not K
		// per-frame parses). Gated + default-off -> zero stock-path change.
		//
		// PHASE 1 (fact-doc §11): the block's bsi is carried ON THE WIRE (cw0 header,
		// FEC-protected). The carve READS the wire bsi (use_wire_header=true) — drift-
		// proof across a SUSTAINED multi-block session. rsp_current_expected_batch_seq_id
		// is passed only as a FALLBACK (used if cw0 didn't decode / header unusable).
		bool bigblock_rx_handled = false;
		// P3 HW FIX: carve ONLY at the CONFIG_16 rung (matches the TX gate in
		// bigblock_send_one_block). is_ofdm_config() (configs 0..16) wrongly engaged the
		// carve at CONFIG_0..15 during the climb, where the block geometry is unvalidated
		// (K=1 at CONFIG_0) — corrupting RX and stalling the gearshift. Stock per-frame
		// parse handles CONFIG_0..15.
		// GAP-3 CARVE-GATE HARDENING (cfg16-controlack-hold): the route at
		// telecom_system.cc:1024 sent EVERY CFG16 OFDM acquisition into receive_bigblock
		// with NO big-block marker check, so a single OFDM control frame (SET_CONFIG /
		// ACK turnaround), stale audio, or noise was carved into a fake K-codeword block
		// (the red-herring "whitening misalignment" signature: wire_bsi=garbage). Require
		// a REAL big-block here: cw0's de-whitened wire-CRC-8 must validate (a real block
		// always carries the FEC+CRC-protected cw0 header; a mis-routed control/stale/noise
		// frame does not). When it FAILS, this was not a block — re-decode the SAME captured
		// passband on the STOCK per-frame path (bigblock_rx_force_stock suppresses the route
		// for one receive_byte call) so the control frame is parsed normally and ACKed,
		// instead of being eaten by the carver. Mirrors the TX, which already declines
		// control (bigblock_send_one_block). Real big-blocks (the dominant CFG16 traffic
		// once GAP-1 holds the rung) pass the cw0 CRC and carve exactly as before.
		bool bigblock_rx_candidate =
			(telecom_system->bigblock_framing_enabled
			 && telecom_system->M != MOD_MFSK
			 && current_configuration == CONFIG_16
			 && telecom_system->bigblock_last_rx_K > 0
			 // WALL-B FIX-3 (C2a): once SUSPENDED, receive_byte ran the STOCK per-frame
			 // decoder (above), so there is no fresh carve to translate — fall through to
			 // the per-frame parse with the stock received_message_stats. NO-OP off-rung /
			 // carve-success (streak<K), so the carve-translate path is byte-identical there.
			 && !bigblock_carve_suspended());
		// ACQUISITION-WINDOW POSITION GUARD — WAIT-FOR-TAIL (fact-doc §22, supersedes the §19
		// defer-and-re-arm REGRESSION). Run BEFORE the cw0-CRC gate. When the located block's
		// tail ran past the captured window (head + block_span > cap: the block landed too late
		// in the snapshot, so its tail samples are FUTURE — not yet produced into the ring at
		// snapshot time — and bb_at zero-padded them), the block-wide estimate collapses and the
		// cw0 CRC fails even on a perfect timing lock.
		//
		// §22 ROOT CAUSE of the §19 regression: the §19 recovery re-armed frames_to_read to a
		// FULL block-span (bigblock_block_ftr_or(0) = block_nsymb+10 = 74 sym) and walked away,
		// expecting the block to "re-land earlier on the next snapshot." On the LIVE path that
		// premise is false TWICE: (1) the CMD emits each big-block EXACTLY ONCE then waits on a
		// positive SACK — there is NO re-presentation, only the SAME single transmission captured
		// later; (2) waiting a full 74-sym block-span advances ring_write_index by 74 sym, so the
		// head (typ. sym ~70-124) scrolls clean OFF THE BACK of the 133-sym window before the
		// next snapshot — destroying the only copy. Result on HW: every attempt deferred, 0
		// carves, CMD-waits-SACK / RSP-waits-rearrival DEADLOCK (winrun_recovery.json:
		// accept_frac 0.0, WORSE than the pre-fix 0.056).
		//
		// THE FIX: the ring slides forward by symbol_period per produced symbol (audioio.c:1390),
		// and each snapshot is the most-recent `cap` samples. To bring the missing tail in-ring
		// we wait ONLY as many fresh symbols as the overrun needs — NOT a block-span. After
		// wait_syms symbols the window slid forward by wait_syms*symbol_period, so head_new =
		// head - wait_syms*symbol_period <= cap-block_span (fits) AND the tail (overrun samples
		// past the old window end) is now inside. Geometric safety (§22.4): block_span(64 sym) <=
		// ring(133 sym), so when the tail just arrives the head sits at sym ~69 with 64 sym of
		// head-room behind it — the head STAYS in-ring through the wait. The SAME single
		// transmission is recovered; no NAK/retransmit needed, so the SACK the CMD is waiting for
		// IS produced and the deadlock is broken. Leave the ring INTACT (do NOT wipe — unlike the
		// carve-success path; the head must survive) and do NOT touch ring_write_index (producer-
		// owned). SKIP the carve this pass; re-attempt on the next snapshot.
		// MERCURY_BIGBLOCK_DEFEAT_ACQGUARD=1 bypasses the wait (carve the truncated block as
		// pre-fix) for the fail-before/pass-after A/B. Production never sets it.
		bool acqguard_defeat = false;
		{ const char* e = std::getenv("MERCURY_BIGBLOCK_DEFEAT_ACQGUARD");
		  if(e && *e && atoi(e) != 0) acqguard_defeat = true; }
		// Bounded: each wait is SHORT (a few symbols, the overrun), not a block-span, so allow a
		// few consecutive waits before falling through to the cw0-CRC gate on a genuinely absent
		// block (no infinite spin). Reset on every accept (:7475 carve-success).
		const int BIGBLOCK_RX_MAX_DEFERS = 6;
		if(bigblock_rx_candidate && !acqguard_defeat && !bigblock_acq_window_fits()
		   && bigblock_rx_defer_count < BIGBLOCK_RX_MAX_DEFERS)
		{
			bigblock_rx_defer_count++;
			// Compute the wait: exactly enough fresh symbols for the overrun tail to arrive, +1
			// symbol of margin so the tail is comfortably in-ring against a producer race.
			long head        = telecom_system->bigblock_last_rx_head_delay_samples;
			long cap_samples = telecom_system->bigblock_last_rx_capture_nsamples;
			int  block_nsymb = telecom_system->bigblock_rx_block_nsymb();
			int  sym_samples = telecom_system->data_container.Nofdm
			                   * telecom_system->data_container.interpolation_rate;
			long block_span  = (long)block_nsymb * (long)sym_samples;
			long overrun     = (head >= 0 && sym_samples > 0)
			                   ? (head + block_span - cap_samples) : 0;
			int  wait_syms   = 1;   // safe default if geometry is unavailable
			if(overrun > 0 && sym_samples > 0)
				wait_syms = (int)((overrun + sym_samples - 1) / sym_samples) + 1;
			if(wait_syms < 1) wait_syms = 1;
			// D3 §22 UN-INVERSION (fix/bigblock-d3-carve): the snapshot is rwi-relative, so after
			// the producer advances ring_write_index by wait_syms symbols the SAME located block's
			// window offset becomes head1 = head - wait_syms*sym_samples — i.e. it slides EARLIER
			// (toward the front of the window) by exactly the tail it was missing, landing at
			// head1 ~ cap - block_span (FITS) with its now-produced tail in-ring. This direction is
			// correct ONLY because the acquisition now re-locks the SAME EARLIEST copy each pass
			// (the earliest-preamble early-exit, telecom_system.cc bigblock_rx_passband). With the
			// old global energy-argmax the re-snapshot false-locked a still-FRESHER later copy, so
			// head DRIFTED FORWARD (head1 > head, HW 116664->132932) — that forward drift WAS the
			// "inverted §22". HEAD-SURVIVAL CAP: never wait so long that the block's head scrolls
			// off the OLDEST edge before the next snapshot (head1 must stay >= 0). The head sits
			// `head` samples ahead of the window's oldest sample, so the most we can slide is
			// `head` samples; cap wait_syms to floor(head/sym) so head1 = head - wait_syms*sym >= 0.
			// (Geometry guarantees this is never binding for a real overrun — block_span < cap, so
			// when the tail just arrives head ~ cap-block_span, far above 0 — but the cap makes the
			// re-arm provably head-preserving rather than relying on that invariant holding.)
			if(head >= 0 && sym_samples > 0)
			{
				int max_wait = (int)(head / sym_samples);   // keep head1 = head - wait*sym >= 0
				if(max_wait >= 1 && wait_syms > max_wait) wait_syms = max_wait;
			}
			printf("[BBTX-ACQ-WAIT] CFG16 big-block (K=%d) tail past capture window "
				"(head=%ld cap=%ld block_nsymb=%d overrun=%ld wait_syms=%d defer=%d/%d) -> "
				"waiting %d fresh symbols for the tail; same single transmission, ring kept; "
				"head slides EARLIER to ~%ld (FITS, earliest-lock keeps it the SAME block)\n",
				telecom_system->bigblock_last_rx_K,
				head, cap_samples, block_nsymb, overrun, wait_syms,
				bigblock_rx_defer_count, BIGBLOCK_RX_MAX_DEFERS, wait_syms,
				head - (long)wait_syms*(long)sym_samples);
			fflush(stdout);
			// Re-arm ONLY the short wait (NOT a block-span) and leave the ring intact so the
			// already-arrived head + body survive while the producer fills the tail. Do NOT carve;
			// the downstream per-frame parse keys on received_message_stats (forced NO below).
			MUTEX_LOCK(&capture_prep_mutex);
			telecom_system->data_container.frames_to_read = wait_syms;
			telecom_system->data_container.nUnder_processing_events = 0;
			telecom_system->receive_stats.ofdm_search_raw = 0;
			telecom_system->receive_stats.ofdm_batch_active = false;
			MUTEX_UNLOCK(&capture_prep_mutex);
			// clear the RX-K marker so the carve gate below is FALSE; not a delivered block.
			telecom_system->bigblock_last_rx_K = 0;
			received_message_stats.message_decoded = NO;
			bigblock_rx_candidate = false;
			bigblock_rx_handled = true;   // suppress the per-frame parse for this deferred pass
		}
		if(bigblock_rx_candidate && !bigblock_rx_cw0_header_valid())
		{
			// GAP-2 LIVE-PATH tally (diag/livepath-sim): this acquisition was a CFG16
			// big-block candidate that FAILED the cw0 wire-CRC gate. Count it so the
			// live-path regression can answer reproduces_cw0crc_reject without re-parsing
			// stdout. SIM_INPROC-only mutation of a test-static; zero production effect.
			cl_arq_controller::sim2_gate_rejects++;
			// WALL-B FIX-3 (C1): a real cw0-CRC carve REJECT while parked at CFG16 with the
			// big-block rung. Accumulate the consecutive-fail streak (the ONE source of truth
			// bigblock_note_carve_reject); at K (=3) the carve route + block-span re-arm are
			// SUSPENDED (bigblock_carve_suspended()) so the RSP decodes the CFG16-PHY demote
			// SET_CONFIG / BREAK on the stock per-frame path it already proved during the climb,
			// instead of staying structurally deaf until the global LINK watchdog session-resets
			// (the HW wall-B 0-delivery). Reset on a real carve accept (below) and on any config
			// change (load_configuration). This branch only runs while bigblock_rx_candidate
			// (CFG16 && big-block framing && K>0 && NOT-already-suspended), so off-rung the
			// streak stays 0 (byte-identical). bigblock_note_carve_reject() logs the
			// [BB-CARVE-SUSPEND] transition on the K-th reject.
			bigblock_note_carve_reject();
			printf("[BBTX-GATE] CFG16 acquisition (K=%d) failed cw0 wire-CRC -> NOT a "
				"big-block; re-decoding on stock per-frame path (control/stale/noise, "
				"not carved)\n", telecom_system->bigblock_last_rx_K);
			fflush(stdout);
			// One-shot stock re-decode of the SAME captured passband. receive_bigblock
			// ran its normalize/blank on a LOCAL snapshot (telecom_system.cc:8246-8251),
			// so ready_to_process_passband_delayed_data is intact for a second decode.
			telecom_system->bigblock_rx_force_stock = true;
			received_message_stats = telecom_system->receive_byte(
				telecom_system->data_container.ready_to_process_passband_delayed_data,
				telecom_system->data_container.data_byte);
			telecom_system->bigblock_rx_force_stock = false;
			// Not a block: clear the RX-K marker so the carve gate below is FALSE and the
			// downstream per-frame parse keys on the (stock) received_message_stats.
			telecom_system->bigblock_last_rx_K = 0;
			bigblock_rx_candidate = false;
			// §19: this acquisition is abandoned (control/stale/noise, OR a defer-cap
			// give-up). Reset the defer cap so the NEXT real block can defer afresh.
			bigblock_rx_defer_count = 0;
		}
		if(bigblock_rx_candidate)
		{
			// GAP-2 LIVE-PATH tally (diag/livepath-sim): a real big-block whose cw0
			// wire-CRC PASSED the gate and is about to be carved. SIM_INPROC-only.
			cl_arq_controller::sim2_gate_accepts++;
			int fallback_bsi = (rsp_current_expected_batch_seq_id >= 0)
				? (rsp_current_expected_batch_seq_id & 0xFF) : 0;
			// HEAP-OVERRUN ROOT-CAUSE FIX (fact-doc §13): carve from the DEDICATED
			// telecom_system->bigblock_rx_infobits (sized K*ldpc.K = 11200 ints by
			// receive_bigblock), NOT data_container.data_byte[N_MAX=1600]. The carve reads
			// K*sub_len*8 = 11200 ints; reading them out of the 1600-int data_byte was a
			// 9600-int over-read (and the decode copy-in had already over-written it). The
			// member always holds the full block decode; data_byte stays a stock-path buffer.
			bigblock_receive_carve(telecom_system->bigblock_rx_infobits.data(),
				(unsigned char)fallback_bsi, /*use_wire_header=*/true);
			bigblock_rx_handled = true;
			bigblock_rx_defer_count = 0;   // §19: full block carved -> reset the defer cap
			// WALL-B FIX-3 (C1 reset): a real block carved -> the carve is viable again. Reset
			// the carve-fail streak so a later transient 1-2 reject burst re-accumulates from 0
			// (a transiently-failing-then-recovering carve is NOT starved, INV-3/RISK-A). On the
			// carve-SUCCESS path this fires on the FIRST accept, so the streak never reaches K
			// and bigblock_carve_suspended() is always false (carve-success byte-identical).
			bigblock_note_carve_accept();
			// the carve already populated messages_rx[] + advanced ARQ state; the
			// per-frame parse path below keys on received_message_stats.message_decoded.
			received_message_stats.message_decoded = NO;

			// PARTIAL-BLOCK FIX (bigblock-whiten-align): a big-block is ONE acquisition of
			// the WHOLE K-codeword block (preamble + Ngrid data symbols, ~64 sym). The
			// snapshot fires when frames_to_read==0, and message_decoded is forced NO above
			// so the stock per-frame success re-arm (this function, ~line 7041) does NOT run.
			// Re-arm frames_to_read to span a FULL next block so the next snapshot waits for
			// the complete block (otherwise it fires after one stock frame's worth of symbols
			// and the late codewords read silence -> CRC-fail -> 0 delivered). Also zero the
			// just-decoded block region in the ring so its preamble/pilots can't false-trigger
			// the next Schmidl-Cox acquisition (the stock zeroing at ~line 6972 is gated on
			// message_decoded==YES, which we cleared).
			{
				int block_nsymb = telecom_system->bigblock_rx_block_nsymb();
				if(block_nsymb > 0)
				{
					MUTEX_LOCK(&capture_prep_mutex);
					int sp = signal_period;
					// Zero the WHOLE ring after a block decode. The just-decoded block (its
					// preamble + data) sits somewhere in this window; any residual OFDM
					// structure (preamble autocorr, pilots) would false-trigger the NEXT
					// block's Schmidl-Cox acquisition and make it lock the wrong offset
					// (observed: block 2 acquired on block 1's stale preamble -> garbage
					// payload, every codeword CRC-fail). One acquisition == one block, so a
					// full wipe is correct (unlike the stock per-frame path, which keeps
					// trailing batch frames). The next block re-accumulates from silence.
					for(int k = 0; k < sp; k++)
					{
						telecom_system->data_container.passband_delayed_data[k] = 0.0;
						telecom_system->data_container.passband_delayed_data[k + sp] = 0.0;
					}
					// NOTE: do NOT reset ring_write_index — the capture-prep feed keeps
					// advancing it; resetting it mid-stream desyncs the next block's write
					// position and the acquisition lands on a misaligned window.
					telecom_system->data_container.frames_to_read = block_nsymb + 10;
					telecom_system->data_container.nUnder_processing_events = 0;
					telecom_system->receive_stats.ofdm_search_raw = 0;
					telecom_system->receive_stats.ofdm_batch_active = false;
					MUTEX_UNLOCK(&capture_prep_mutex);
				}
			}
		}
		(void)bigblock_rx_handled;

		auto proc_end = std::chrono::steady_clock::now();
		double proc_ms = std::chrono::duration<double, std::milli>(proc_end - proc_start).count();

		// Frame period = (preamble + data symbols) in wall clock time
		double frame_samples = (double)(telecom_system->data_container.Nofdm *
			(telecom_system->data_container.Nsymb + telecom_system->data_container.preamble_nSymb) *
			telecom_system->data_container.interpolation_rate);
		double frame_ms = (frame_samples / 48000.0) * 1000.0;
		float load = (frame_ms > 0) ? (float)(proc_ms / frame_ms) : 0.0f;

#ifdef MERCURY_GUI_ENABLED
		g_gui_state.processing_load.store(load);
		{
			size_t buf_used = size_buffer(capture_buffer);
			size_t buf_cap = circular_buf_capacity(capture_buffer);
			g_gui_state.buffer_fill_pct.store(buf_cap > 0 ? 100.0f * (float)buf_used / (float)buf_cap : 0.0f);
		}
#endif

		measurements.signal_stregth_dbm = received_message_stats.signal_stregth_dbm;

		// Ring buffer: zero the entire decoded frame (preamble + data)
		// after successful decode to prevent false Schmidl-Cox detections.
		// Without this, the data symbols of decoded frames contain OFDM
		// structure (pilots, subcarrier patterns) that produce false
		// autocorrelation peaks at metric 0.08-0.22, causing cascading
		// FAILs that waste 10-20 seconds per turnaround.
		// IMPORTANT: Only on OK decode. Zeroing on FAIL would destroy
		// real preambles during turnaround (data still arriving).
		if(telecom_system->M != MOD_MFSK
			&& received_message_stats.message_decoded == YES
			&& received_message_stats.delay > 0)
		{
			int sp = signal_period;
			// LEVER P: zero only the ACTUAL decoded frame (eff preamble + data).
			// Zeroing the FULL preamble_nSymb on a MINI tail frame would erase
			// 3 symbols into the NEXT frame's MINI preamble in the gapless batch
			// waveform, destroying it. last_eff_preamble_nsymb == preamble_nSymb
			// when amortization is off.
			int zero_eff_pre = telecom_system->receive_stats.last_eff_preamble_nsymb;
			if(zero_eff_pre < 1 || zero_eff_pre > telecom_system->data_container.preamble_nSymb)
				zero_eff_pre = telecom_system->data_container.preamble_nSymb;
			int frame_syms = zero_eff_pre
				+ telecom_system->get_active_nsymb();
			int frame_samples = frame_syms * symbol_period;
			int frame_ring_start = (rwi + received_message_stats.delay) % sp;

			MUTEX_LOCK(&capture_prep_mutex);
			for(int k = 0; k < frame_samples; k++)
			{
				int pos = (frame_ring_start + k) % sp;
				telecom_system->data_container.passband_delayed_data[pos] = 0.0;
				telecom_system->data_container.passband_delayed_data[pos + sp] = 0.0;
			}
			MUTEX_UNLOCK(&capture_prep_mutex);
		}

		// Bug #35 diagnostic: track every decode attempt (verbose only)
		if(g_verbose)
		{
			static int decode_attempt = 0;
			decode_attempt++;
			if(received_message_stats.message_decoded == YES)
			{
				int nrd = telecom_system->data_container.nBits - telecom_system->ldpc.P;
				int fs = (nrd - 16) / 8;  // frame_size with CRC16
				printf("[RX-DECODE#%d] OK: delay=%d iters=%d ofdm_raw=%d ftr=%d nUnder=%d bytes: %02x %02x %02x %02x %02x %02x\n",
					decode_attempt, received_message_stats.delay, received_message_stats.iterations_done,
					telecom_system->receive_stats.ofdm_search_raw,
					telecom_system->data_container.frames_to_read.load(),
					telecom_system->data_container.nUnder_processing_events.load(),
					telecom_system->data_container.data_byte[0] & 0xFF,
					telecom_system->data_container.data_byte[1] & 0xFF,
					telecom_system->data_container.data_byte[2] & 0xFF,
					telecom_system->data_container.data_byte[3] & 0xFF,
					telecom_system->data_container.data_byte[4] & 0xFF,
					telecom_system->data_container.data_byte[5] & 0xFF);
			}
			else
			{
				if(received_message_stats.delay == -1)
					printf("[RX-DECODE#%d] NO-PREAMBLE: mfsk_raw=%d ofdm_raw=%d nUnder=%d link=%d\n",
						decode_attempt,
						telecom_system->receive_stats.mfsk_search_raw,
						telecom_system->receive_stats.ofdm_search_raw,
						telecom_system->data_container.nUnder_processing_events.load(),
						(int)link_status);
				else
					printf("[RX-DECODE#%d] FAIL: delay=%d metric=%.3f ofdm_raw=%d nUnder=%d link=%d conn=%d\n",
						decode_attempt, received_message_stats.delay,
						telecom_system->receive_stats.coarse_metric,
						telecom_system->receive_stats.ofdm_search_raw,
						telecom_system->data_container.nUnder_processing_events.load(),
						(int)link_status, (int)connection_status);
			}
			fflush(stdout);
		}

		if (received_message_stats.message_decoded==YES)
		{
			int rx_nsymb = telecom_system->get_active_nsymb();
			// LEVER P: this frame's actual length is (eff preamble + data). For a
			// MINI tail frame eff=1, so rx_frame is shorter; the next preamble in
			// the gapless batch waveform sits exactly rx_frame symbols ahead. The
			// position chain (ofdm_search_raw / frames_to_read) MUST advance by the
			// ACTUAL length or the batch-predict window misses every tail frame.
			// last_eff_preamble_nsymb == preamble_nSymb when amortization is off.
			int rx_eff_pre = telecom_system->receive_stats.last_eff_preamble_nsymb;
			if(rx_eff_pre < 1 || rx_eff_pre > telecom_system->data_container.preamble_nSymb)
				rx_eff_pre = telecom_system->data_container.preamble_nSymb;  // defensive
			int rx_frame = rx_nsymb + rx_eff_pre;
			int end_of_current_message = received_message_stats.delay / symbol_period  + rx_frame;
			int frames_left_in_buffer = telecom_system->data_container.buffer_Nsymb - end_of_current_message;
			if(frames_left_in_buffer<0)
				frames_left_in_buffer=0;

			int nUnder_snapshot = telecom_system->data_container.nUnder_processing_events.load();
			// Only subtract nUnder when there's buffer margin to absorb it.
			// When frames_left <= nUnder, subtracting causes ftr < rx_frame,
			// which means the next frame drifts 1+ symbols higher per decode.
			// Eventually the preamble lands beyond upper_bound and can't decode.
			int nUnder_adj = (frames_left_in_buffer > nUnder_snapshot) ? nUnder_snapshot : 0;
			telecom_system->data_container.frames_to_read=rx_frame-frames_left_in_buffer-nUnder_adj;

			int ftr_clamped = 0;
			if(telecom_system->data_container.frames_to_read < 0)
			{
				// Buffer already has enough data for the next frame — decode immediately.
				// ofdm_search_raw / mfsk_search_raw will skip past the just-decoded frame.
				telecom_system->data_container.frames_to_read = 0;
				ftr_clamped = 1;
			}

			// Minimum shift to keep next frame within extraction bounds.
			// Must account for the turnaround gap: after this decode, the
			// other station receives the ACK then sends the next frame.
			// The gap is ~2s (~93 symbols at 22.67ms/sym).  Without this,
			// the next preamble lands past the buffer end because the ftr
			// was too small to make room.
			//
			// Safety margin (+4): without it, the next batch frame lands
			// exactly at upper_bound.  Any nUnder event (1-3 symbols of
			// capture-thread latency) pushes the preamble 1 symbol past
			// upper_bound → beyond-bounds FAIL → fast-forward → OK → repeat
			// (alternating 50% FAIL rate on VB-Cable batch runs).
			int upper_bound = telecom_system->data_container.buffer_Nsymb - rx_frame;
			int min_ftr = end_of_current_message - upper_bound + 4;
			if(min_ftr > 0 && telecom_system->data_container.frames_to_read < min_ftr)
			{
				telecom_system->data_container.frames_to_read = min_ftr;
				ftr_clamped = 3;
			}

			// Upper clamp: limit shift to end_of_current_message (flush
			// all decoded audio, keep only fresh buffer for next frame).
			// Old clamp was rx_frame which is too small when the turnaround
			// gap pushes the next preamble far into the buffer.
			if(telecom_system->data_container.frames_to_read > end_of_current_message)
			{
				telecom_system->data_container.frames_to_read = end_of_current_message;
				ftr_clamped = 2;
			}

			// === DIAG: success ftr trace (verbose only) ===
			if(g_verbose)
			{
				printf("[FTR-OK] CONFIG_%d ftr=%d delay_sym=%d end=%d left=%d nUnder=%d clamped=%d\n",
					current_configuration,
					telecom_system->data_container.frames_to_read.load(),
					received_message_stats.delay / symbol_period,
					end_of_current_message, frames_left_in_buffer, nUnder_snapshot, ftr_clamped);
				fflush(stdout);
			}
			if (g_verbose)
				printf("[RX-TIMING] OK: delay=%d delay_symb=%d rx_frame=%d end=%d left=%d nUnder=%d ftr=%d clamped=%d proc=%.0fms\n",
					received_message_stats.delay, received_message_stats.delay / symbol_period,
					rx_frame, end_of_current_message, frames_left_in_buffer, nUnder_snapshot,
					telecom_system->data_container.frames_to_read.load(), ftr_clamped, proc_ms);
			fflush(stdout);

			// MFSK anti-re-decode: after successful decode, record where the old
			// frame ends so the next time_sync_mfsk skips past it entirely.
			// Must skip the full frame (preamble + data), not just the preamble,
			// because MFSK data tones can create false preamble correlations.
			// mfsk_search_raw = frame_end_symb - frames_to_read (base value).
			// telecom_system subtracts nUnder at search time for the effective start.
			if(telecom_system->M == MOD_MFSK)
			{
				int frame_end_symb = received_message_stats.delay / symbol_period + rx_frame;
				telecom_system->receive_stats.mfsk_search_raw =
					frame_end_symb - telecom_system->data_container.frames_to_read;
			}
			else
			{
				// OFDM anti-re-decode: record frame end so next Schmidl-Cox
				// search skips past this decoded preamble.
				// Account for nUnder: during LDPC decode, the buffer shifted
				// nUnder_snapshot times. Without subtracting it here, search_raw
				// overshoots by nUnder symbols after the reset at line 3291,
				// causing the next batch frame to land BEFORE ofdm_skip and
				// become invisible to detection.
				// -1 margin catches frames 1 symbol below expected position.
				int frame_end_symb = received_message_stats.delay / symbol_period + rx_frame;
				telecom_system->receive_stats.ofdm_search_raw =
					frame_end_symb - telecom_system->data_container.frames_to_read - nUnder_snapshot - 1;
				if(telecom_system->receive_stats.ofdm_search_raw < 0)
					telecom_system->receive_stats.ofdm_search_raw = 0;
				// Clamp at upper_bound: search_raw can exceed buffer_Nsymb - rx_frame
				// when ftr is clamped to 0 (frame near buffer end). Without clamping,
				// next search starts past upper_bound → recovery forces unnecessary FAIL.
				int upper_clamp = telecom_system->data_container.buffer_Nsymb.load() - rx_frame;
				if(upper_clamp > 0 && telecom_system->receive_stats.ofdm_search_raw > upper_clamp)
					telecom_system->receive_stats.ofdm_search_raw = upper_clamp;
				telecom_system->receive_stats.ofdm_batch_active = true;

				// BREAK forward-health gate (fix/break-fh-gate) FIX-A: a forward OFDM
				// frame just decoded successfully (this is the M!=MOD_MFSK else-branch).
				// Latch the receive() iteration index; the BREAK probe (decode-FAIL
				// else-branch below) is suppressed while this is recent. Write is
				// unconditional but read ONLY by the env-gated break_fh_suppress() ->
				// byte-identical when MERCURY_BREAK_FH_GATE is unset.
				last_forward_ofdm_decode_frame = rx_receive_frame_index;

				// Opportunistic scan success: reset failure counter
				if(passive_monitor) monitor_consec_ofdm_fail = 0;

				if(g_verbose)
				{
					printf("[OFDM-SKIP] frame_end=%d ftr=%d search_raw=%d clamped=%d\n",
						frame_end_symb, telecom_system->data_container.frames_to_read.load(),
						telecom_system->receive_stats.ofdm_search_raw, ftr_clamped);
					fflush(stdout);
				}
			}

			telecom_system->receive_stats.delay_of_last_decoded_message += (rx_frame - (telecom_system->data_container.frames_to_read + telecom_system->data_container.nUnder_processing_events)) * symbol_period;

			telecom_system->data_container.nUnder_processing_events = 0;

			measurements.frequency_offset = received_message_stats.freq_offset;
			// Always update RX SNR from any decoded LDPC frame (regardless of role).
			// With pattern ACK, the commander never decodes LDPC during ACK detection,
			// so SNR_uplink only refreshes during SWITCH_ROLE when we receive data.
			measurements.SNR_uplink = received_message_stats.SNR;
			if(this->role == RESPONDER)
			{
				measurements.SNR_downlink = received_message_stats.SNR;
			}

			{
				int byte_copy_len = this->max_data_length + this->max_header_length;
				if(byte_copy_len > N_MAX/8) byte_copy_len = N_MAX/8;
				for(int i=0; i < byte_copy_len; i++)
				{
					message_TxRx_byte_buffer[i] = (char)telecom_system->data_container.data_byte[i];
				}
	
			}
			// In passive monitor mode, accept ALL connection_ids and adopt the session
			if(passive_monitor && this->connection_id == 0 && message_TxRx_byte_buffer[1] != BROADCAST_ID)
			{
				this->connection_id = message_TxRx_byte_buffer[1];
				printf("[MONITOR] Adopted connection_id=0x%02x\n",
					(unsigned char)this->connection_id);
				fflush(stdout);
			}
			if(passive_monitor || message_TxRx_byte_buffer[1] == this->connection_id || message_TxRx_byte_buffer[1] == BROADCAST_ID)
			{
				messages_rx_buffer.status=RECEIVED;
				messages_rx_buffer.type=message_TxRx_byte_buffer[0];
				// Bit 7 of sequence_number = end-of-batch flag from commander (data frames only)
				// R038 (race audit 2026-06-06): this capture runs PRE-ROUTING — before
				// the responder classifies the frame as match-current / match-prev /
				// drop. Writing last_received_end_of_batch_seq here for ANY CRC-valid
				// EOB frame let a prev-retransmit / late-duplicate of a SHORTER batch
				// poison the CURRENT batch's effective_batch (early ACK-GATE PASS ->
				// truncated delivery). For v2, STAGE the EOB seq and let the responder
				// promote it ONLY inside the confirmed match-current storage block
				// (arq_responder.cc match-current path). v1 has no bsi routing, so it
				// keeps writing last_received_end_of_batch_seq directly here
				// (byte-for-byte unchanged).
				rx_buffer_eob_seq = -1;
				if((message_TxRx_byte_buffer[2] & 0x80)
					&& (messages_rx_buffer.type == DATA_LONG || messages_rx_buffer.type == DATA_SHORT))
				{
					int eob_seq = message_TxRx_byte_buffer[2] & 0x7F;
					if(sack_v2_enabled)
						rx_buffer_eob_seq = eob_seq;   // v2: stage, promote on match-current
					else
						last_received_end_of_batch_seq = eob_seq;  // v1: unchanged
				}
				messages_rx_buffer.sequence_number=message_TxRx_byte_buffer[2] & 0x7F;
				last_received_message_sequence=messages_rx_buffer.sequence_number;
				// Defensive clamp: never write more than alloc_size (N_MAX/8 = 200) bytes
				// into any .data buffer, regardless of max_data_length + max_header_length.
				const int alloc_size = N_MAX / 8;
				if(messages_rx_buffer.type==ACK_CONTROL  ||  messages_rx_buffer.type==CONTROL)
				{
					int copy_len = max_data_length+max_header_length-CONTROL_ACK_CONTROL_HEADER_LENGTH;
					if(copy_len > alloc_size) copy_len = alloc_size;
					for(int j=0;j<copy_len;j++)
					{
						messages_rx_buffer.data[j]=message_TxRx_byte_buffer[j+CONTROL_ACK_CONTROL_HEADER_LENGTH];
					}
				}
				if( messages_rx_buffer.type==ACK_MULTI || messages_rx_buffer.type==ACK_RANGE)
				{
					int copy_len = max_data_length+max_header_length-ACK_MULTI_ACK_RANGE_HEADER_LENGTH;
					if(copy_len > alloc_size) copy_len = alloc_size;
					for(int j=0;j<copy_len;j++)
					{
						messages_rx_buffer.data[j]=message_TxRx_byte_buffer[j+ACK_MULTI_ACK_RANGE_HEADER_LENGTH];
					}
				}
				else if(messages_rx_buffer.type==SACK_RSP)
				{
					SACK_TRACE("receive() parsed SACK_RSP frame: seq=%d conn_id=0x%02x",
						(int)messages_rx_buffer.sequence_number,
						(unsigned char)message_TxRx_byte_buffer[1]);
					// SACK Design A Step 7 — OFDM SACK_RSP wire shape:
					// 3-byte msg header + payload in messages_rx_buffer.data[0..].
					// Caller (decode_sack_v2_frame) derives length from the
					// payload layout. We DO NOT decode here — only copy bytes.
					int copy_len = max_data_length+max_header_length-ACK_MULTI_ACK_RANGE_HEADER_LENGTH;
					if(copy_len > alloc_size) copy_len = alloc_size;
					for(int j=0;j<copy_len;j++)
					{
						messages_rx_buffer.data[j]=message_TxRx_byte_buffer[j+ACK_MULTI_ACK_RANGE_HEADER_LENGTH];
					}
					// length field on messages_rx_buffer is not set by the wire
					// for ACK-family frames (the LDPC codeword size implicitly
					// bounds it). The caller will validate using the type-
					// specific payload layout.
				}
				else if(messages_rx_buffer.type==DATA_LONG)
				{
					// SACK Design A Steps 1+3 — DATA_LONG header parse + batch_seq_id store.
					// v1 (default): byte[3] = id. v2: byte[3] = batch_seq_id, byte[4] = id.
					// batch_seq_id is stored on messages_rx_buffer and (for diagnostics)
					// on last_received_batch_seq_id. NO routing/RX decision keys off it
					// yet — pure scaffolding per Step 3 (decision is Step 4+).
					int eff_hdr = effective_data_long_header_length(sack_v2_enabled);
					if(sack_v2_enabled)
					{
						int bsi = (unsigned char)message_TxRx_byte_buffer[3];
						messages_rx_buffer.batch_seq_id = bsi;
						last_received_batch_seq_id = bsi;
						messages_rx_buffer.id=message_TxRx_byte_buffer[4];
					}
					else
					{
						messages_rx_buffer.batch_seq_id = -1;  // v1: field not on wire
						messages_rx_buffer.id=message_TxRx_byte_buffer[3];
					}
					int copy_len = max_data_length+max_header_length-eff_hdr;
					if(copy_len > alloc_size) copy_len = alloc_size;
					messages_rx_buffer.length=copy_len;
					for(int j=0;j<copy_len;j++)
					{
						messages_rx_buffer.data[j]=message_TxRx_byte_buffer[j+eff_hdr];
					}
					if(sack_v2_enabled)
					{
						printf("[RX-BATCH-SEQ] type=DATA_LONG id=%d seq=%d batch_seq_id=%d (v2)\n",
							(unsigned char)messages_rx_buffer.id,
							(unsigned char)messages_rx_buffer.sequence_number & 0x7F,
							messages_rx_buffer.batch_seq_id);
						fflush(stdout);
					}

				}
				else if(messages_rx_buffer.type==DATA_SHORT)
				{
					// SACK Design A Steps 2+3 — DATA_SHORT header parse + batch_seq_id store.
					// v1 (default): byte[3] = id, byte[4] = length.
					// v2: byte[3] = batch_seq_id, byte[4] = id, byte[5] = length.
					// batch_seq_id is stored but no decision branches on it (Step 4+).
					int eff_hdr = effective_data_short_header_length(sack_v2_enabled);
					if(sack_v2_enabled)
					{
						int bsi = (unsigned char)message_TxRx_byte_buffer[3];
						messages_rx_buffer.batch_seq_id = bsi;
						last_received_batch_seq_id = bsi;
						messages_rx_buffer.id=message_TxRx_byte_buffer[4];
						messages_rx_buffer.length=(unsigned char)message_TxRx_byte_buffer[5];
					}
					else
					{
						messages_rx_buffer.batch_seq_id = -1;  // v1: field not on wire
						messages_rx_buffer.id=message_TxRx_byte_buffer[3];
						messages_rx_buffer.length=(unsigned char)message_TxRx_byte_buffer[4];
					}
					// Clamp length to buffer size — corrupted frames (e.g., from
					// noise) can have garbage length values that overflow the buffer.
					int max_short_len = max_data_length + max_header_length - eff_hdr;
					if(max_short_len < 0) max_short_len = 0;
					if(max_short_len > alloc_size) max_short_len = alloc_size;
					if(messages_rx_buffer.length > max_short_len)
						messages_rx_buffer.length = max_short_len;
					for(int j=0;j<messages_rx_buffer.length;j++)
					{
						messages_rx_buffer.data[j]=message_TxRx_byte_buffer[j+eff_hdr];
					}
					if(sack_v2_enabled)
					{
						printf("[RX-BATCH-SEQ] type=DATA_SHORT id=%d seq=%d batch_seq_id=%d (v2)\n",
							(unsigned char)messages_rx_buffer.id,
							(unsigned char)messages_rx_buffer.sequence_number & 0x7F,
							messages_rx_buffer.batch_seq_id);
						fflush(stdout);
					}

				}

				last_message_received_type=messages_rx_buffer.type;
				if(messages_rx_buffer.type==CONTROL || messages_rx_buffer.type==ACK_CONTROL)
				{
					last_message_received_code=messages_rx_buffer.data[0];
				}
			}
		}
		else
		{
			// MFSK frame completeness: if the frame extends beyond captured audio,
			// capture the remaining symbols instead of wasting a full recapture cycle.
			// The metric threshold in time_sync_mfsk already filters false preambles,
			// so any detected preamble with overflow is worth recapturing.
			// For NB ROBUST_0, frame (537) = capture window, so ALL real preambles
			// in the new-data half of the buffer have large overflow — must allow it.
			int frame_symb = telecom_system->data_container.preamble_nSymb +
			                 telecom_system->data_container.Nsymb;
			if(received_message_stats.frame_overflow_symbols > 0 &&
			   received_message_stats.frame_overflow_symbols < frame_symb)
			{
				int shift_symbols = received_message_stats.frame_overflow_symbols + 4;

				// Read nUnder BEFORE resetting — these shifts already happened to the
				// live buffer during processing and must be included in the total shift
				// when adjusting search cursor for the recaptured buffer.
				int nUnder_current = telecom_system->data_container.nUnder_processing_events.load();
				int total_shift = shift_symbols + nUnder_current;

				telecom_system->data_container.frames_to_read = shift_symbols;
				telecom_system->data_container.nUnder_processing_events = 0;

				// Adjust anti-re-decode cursor for buffer shift. Separate counters
				// for MFSK and OFDM — update whichever is active.
				int adjusted_search;
				if(telecom_system->M == MOD_MFSK)
				{
					adjusted_search = telecom_system->receive_stats.mfsk_search_raw - total_shift;
					if(adjusted_search < 0) adjusted_search = 0;
					telecom_system->receive_stats.mfsk_search_raw = adjusted_search;
				}
				else
				{
					adjusted_search = telecom_system->receive_stats.ofdm_search_raw - total_shift;
					if(adjusted_search < 0) adjusted_search = 0;
					telecom_system->receive_stats.ofdm_search_raw = adjusted_search;
				}

				if (g_verbose)
					printf("[RX-TIMING] INCOMPLETE: overflow=%d symbols, capturing %d more, nUnder=%d search_raw=%d mod=%d\n",
						received_message_stats.frame_overflow_symbols,
						shift_symbols, nUnder_current, adjusted_search,
						(int)telecom_system->M);
				fflush(stdout);
				return;
			}

			if (g_verbose)
				printf("[RX-TIMING] FAIL: nUnder=%d proc=%.0fms search_raw=%d delay_last=%d mod=%.0f\n",
					telecom_system->data_container.nUnder_processing_events.load(), proc_ms,
					telecom_system->receive_stats.mfsk_search_raw,
					telecom_system->receive_stats.delay_of_last_decoded_message,
					telecom_system->M);
			fflush(stdout);

			// BREAK pattern detection: after failed decode, check for emergency
			// "drop to ROBUST_0" signal from commander during turboshift.
			// Only active when: gearshift enabled + responder role.
			// Commander never needs to detect BREAK (it sends BREAK, not receives).
			// Without gearshift, BREAK has no purpose — disable to avoid false positives
			// (matched=8/16 threshold too easy to hit on random MFSK data).
			//
			// OFDM-alias gate (gearshift_v1 finding): the BREAK Goertzel/FFT
			// detector matches at perfect 16/16 on the OFDM CFG12+ preamble
			// because the OFDM data carriers happen to land in the expected
			// BREAK tone bins. Without this gate, every failed OFDM decode at
			// high config wedged the link by forcing RSP to CFG0 on phantom
			// BREAK. Real BREAK is MFSK only — its Schmidl-Cox correlation
			// is low (~0.1-0.2); OFDM preamble Schmidl-Cox is high (~0.6-1.0).
			// Gate: skip BREAK if Schmidl-Cox just matched — it's OFDM, not
			// a real BREAK pattern.
			// Also gate on link_status==CONNECTED: BREAK is a recovery
			// signal for an in-progress session. Before CONNECT completes,
			// CMD is sending HAIL/CONNECTION patterns (also M=16 MFSK)
			// that can land matched=10/16 on the BREAK detector — false
			// positive that wedges RSP at CFG0 during the handshake
			// (gearshift_v14 finding). RSP doesn't need BREAK recovery
			// while still in LISTENING.
			// WALL-B FIX-3 (C3): when the CFG16 carve is SUSPENDED, LIFT the coarse_metric
			// gate. The carve raises receive_stats.coarse_metric to ~0.5-0.6 on exactly the
			// frames it eats, which otherwise suppresses the BREAK probe (the HW wall-B: the
			// CFG16-PHY BREAK is starved on the same oversized snapshot). Lifted ONLY at
			// streak>=K && CFG16 && big-block framing (bigblock_carve_suspended()) — a state
			// the gearshift_v1 stock-OFDM-data-carrier 16/16 false-BREAK alias does NOT occur
			// in (that is stock OFDM DATA mid-batch, streak<K). break_match_threshold (10/16) +
			// the Schmidl-Cox detect_break_pattern_from_passband still gate the actual match, so
			// no false BREAK fires on a stock OFDM data run (RISK-B, Test E).
			//
			// fix/break-fh-gate (workflow w2ee37gd6):
			//   FIX-D: the WALL-B FIX-3 carve-suspend lift (|| bigblock_carve_suspended()) is
			//          the ONLY remaining OFDM-alias guard once the failed-CFG16-frame coarse
			//          metric drops below 0.30 — but a SUSPENDED carve at CFG16 is EXACTLY the
			//          marginal-decode state in which the 50-subcarrier OFDM argmax aliases the
			//          8 WB break_tones to matched>=10. When the FH gate is enabled we do NOT
			//          honor the lift (break_fh_carve_lift() returns false), so the coarse<0.30
			//          gate stands and the forward-health latch / K-of-N below carry the load.
			//          When the env is unset, break_fh_carve_lift()==bigblock_carve_suspended()
			//          exactly -> byte-identical to 48103fa.
			//   FIX-A: && !break_fh_suppress() — suppress the probe entirely while a forward
			//          OFDM frame decoded within the recent window (a real BREAK comes AFTER the
			//          commander stops forward OFDM, so the latch has aged out). No-op when off.
			if(break_detected == NO && gear_shift_on && role == RESPONDER
			   && link_status == CONNECTED
			   && !break_fh_suppress()
			   && (telecom_system->receive_stats.coarse_metric < 0.30
			       || break_fh_carve_lift()))
			{
				int matched = 0;
				double metric = telecom_system->detect_break_pattern_from_passband(
					telecom_system->data_container.ready_to_process_passband_delayed_data,
					signal_period, &matched);
				// Diagnostic: log every call when the OFDM-alias gate passes.
				// matched < threshold means BREAK wasn't there (or alignment
				// failed even with the always_fine refinement). Useful for
				// surfacing future regressions silently.
				if (g_verbose && matched > 0)
				{
					printf("[BREAK-PROBE] coarse=%.2f matched=%d/%d (thr=%d) metric=%.2f\n",
						telecom_system->receive_stats.coarse_metric, matched,
						telecom_system->ack_mfsk.ack_pattern_nsymb,
						telecom_system->ack_mfsk.break_match_threshold, metric);
					fflush(stdout);
				}
				// Require break_match_threshold (WB:10/16, NB M=8:24/32, NB M=4:40/48).
				// fix/break-fh-gate FIX-B: the per-frame match decision is routed through
				// break_kofn_corroborate(), which when the env is unset is a pure pass-through
				// (single-shot -> byte-identical) and when set requires BREAK_KOFN_K consecutive
				// matches before detonating (the per-batch alias is a one-frame transient; a real
				// BREAK is retried/sustained so survives K-of-N).
				bool probe_matched = (metric >= telecom_system->ack_pattern_detection_threshold
				                      && matched >= telecom_system->ack_mfsk.break_match_threshold);
				if(break_kofn_corroborate(probe_matched))
				{
					printf("[BREAK] Emergency pattern detected! metric=%.2f matched=%d/%d (coarse=%.2f)\n",
						metric, matched, telecom_system->ack_mfsk.ack_pattern_nsymb,
						telecom_system->receive_stats.coarse_metric);
					fflush(stdout);
#ifdef MERCURY_GUI_ENABLED
					gui_push_monitor_event("[BREAK]", false);
#endif
					break_detected = YES;
				}
			}

			// HAIL detection: "I am Mercury" beacon from commander.
			// Active in LISTENING/CONNECTION_RECEIVED state (responder waiting for contact).
			if(hail_detected == NO && role == RESPONDER &&
				(link_status == LISTENING || link_status == CONNECTION_RECEIVED))
			{
				int matched = 0;
				double metric = telecom_system->detect_hail_pattern_from_passband(
					telecom_system->data_container.ready_to_process_passband_delayed_data,
					signal_period, &matched);
				double hail_quality = (matched > 0) ? metric / matched : 0.0;
				if(metric >= telecom_system->ack_pattern_detection_threshold
				   && matched >= telecom_system->ack_mfsk.hail_match_threshold
				   && hail_quality >= 0.3)
				{
					printf("[HAIL] 'I am Mercury' beacon detected! metric=%.2f matched=%d/%d quality=%.2f\n",
						metric, matched, telecom_system->ack_mfsk.ack_pattern_nsymb, hail_quality);
					fflush(stdout);
					hail_detected = YES;
				}
			}

			// MFSK FAIL anti-spin: without this, frames_to_read stays at 0
			// after FAIL (no overflow), causing a tight spin loop where each
			// iteration (~100-200ms) accumulates nUnder_processing_events.
			// Large nUnder skews the MFSK preamble search start position.
			// Small shift lets the buffer accumulate fresh audio.
			if(telecom_system->M == MOD_MFSK && telecom_system->data_container.frames_to_read == 0)
			{
				int mfsk_ftr = telecom_system->data_container.preamble_nSymb * 2;
				if(mfsk_ftr < 16) mfsk_ftr = 16;
				telecom_system->data_container.frames_to_read = mfsk_ftr;
				telecom_system->data_container.nUnder_processing_events = 0;
				telecom_system->receive_stats.mfsk_search_raw = 0;
			}

			// Prevent OFDM FAIL spin loop: pause to let the buffer accumulate
			// fresh audio instead of burning CPU on doomed LDPC decodes.
			// Smart fast-forward (Bug #33): after a config transition, the
			// buffer is half-empty and the preamble lands beyond upper_bound
			// (frame doesn't fit).  Instead of the default 8-symbol shift
			// (~181ms, needing ~17 iterations), calculate the exact overflow
			// and shift by that amount in one go.
			if(telecom_system->M != MOD_MFSK && telecom_system->data_container.frames_to_read == 0)
			{
				// Default anti-spin: NB frames are 5x longer (nc_scale=5),
				// so use 2x larger ftr to scroll past false detections faster.
				int ftr = (telecom_system->ofdm.Nc <= 10) ? 16 : 8;

				if(received_message_stats.delay > 0)
				{
					int sym_period = telecom_system->data_container.Nofdm
						* telecom_system->data_container.interpolation_rate;
					int pream_symb = received_message_stats.delay / sym_period;
					// LEVER P (INC-3): the beyond-bounds fast-forward `upper` must
					// match the MINI-aware extraction/gate bound (telecom_system.cc
					// upper_bound + frame_size_interp). In a MINI batch the tail
					// frames are (Nsymb+1) symbols, so the FULL-frame `upper` over-
					// shifts (skips a decodable tail frame) and misreports beyond-
					// bounds. last_eff_preamble_nsymb carries the active per-frame
					// preamble length (== preamble_nSymb on every non-MINI /
					// amortization-off path, so byte-identical when the feature is off).
					int ff_eff_pre = telecom_system->receive_stats.last_eff_preamble_nsymb;
					if(ff_eff_pre < 1 || ff_eff_pre > telecom_system->data_container.preamble_nSymb)
						ff_eff_pre = telecom_system->data_container.preamble_nSymb;
					int frame_symb = telecom_system->data_container.Nsymb + ff_eff_pre;
					int upper = telecom_system->data_container.buffer_Nsymb - frame_symb;

					if(received_message_stats.frame_data_missing)
					{
						// §7.13.36 — During a v2 SACK polling window, an
						// in-flight SACK_RSP (OFDM partial-batch ACK) may be
						// detected preamble-first because the data symbols
						// haven't fully arrived in the ring yet (sub-frame
						// timing race amplified by compression CPU jitter
						// or SACK_RSP TX-end / RX-window start overlap).
						// The pre-§7.13.36 code zeroed the preamble in the
						// ring to suppress re-detection of stale preambles
						// after a successful decode — but during SACK
						// polling, the still-arriving data needs the
						// preamble intact for the NEXT dispatch to lock
						// and decode the now-complete frame. Without this
						// gate, the preamble gets zeroed before data
						// finishes arriving → SACK_RSP never decoded →
						// CMD wedges (gearshift_v22 finding).
						//
						// §7.13.36.1 — gate preservation on metric >= 0.5:
						// weak preambles (metric 0.2-0.3) are usually
						// autocorrelation false positives from CMD's TX
						// residue or sustained noise, NOT real in-flight
						// frames. Preserving them causes the detector to
						// re-lock onto the same fake preamble for 100+
						// dispatches and never advance (gearshift_v23
						// finding: metric=0.283 stuck for 99 cycles).
						// Real SACK_RSP preambles run metric >= 0.9 (v18
						// working case showed 0.999).
						bool in_v2_sack_dispatch = sack_v2_enabled
							&& role == COMMANDER
							&& data_ack_received == NO
							&& telecom_system->receive_stats.coarse_metric >= 0.5;

						if(!in_v2_sack_dispatch)
						{
							// Preamble detected but data symbols are silence.
							// Zero the stale preamble in the ring to prevent
							// re-detection (shift_left would have slid it off).
							int sp = signal_period;
							int pream_samples = telecom_system->data_container.preamble_nSymb * symbol_period;
							int pream_ring_start = (rwi + received_message_stats.delay) % sp;

							MUTEX_LOCK(&capture_prep_mutex);
							for(int k = 0; k < pream_samples; k++)
							{
								int pos = (pream_ring_start + k) % sp;
								telecom_system->data_container.passband_delayed_data[pos] = 0.0;
								telecom_system->data_container.passband_delayed_data[pos + sp] = 0.0;
							}
							MUTEX_UNLOCK(&capture_prep_mutex);
						}
						// Quick retry — 8 symbols (~49ms WB) is enough to skip past
						// the failed position without waiting a full frame.
						// In the v2 SACK case, the retry instead gives the in-flight
						// data symbols time to arrive in the ring so the next dispatch
						// can lock on the complete frame.
						ftr = 8;
						telecom_system->receive_stats.ofdm_search_raw = 0;
						telecom_system->receive_stats.ofdm_batch_active = false;
						printf("[FTR-INCOMPLETE] pream=%d ftr=%d metric=%.3f v2_sack=%d\n",
							pream_symb, ftr,
							telecom_system->receive_stats.coarse_metric,
							in_v2_sack_dispatch ? 1 : 0);
						fflush(stdout);
						SACK_TRACE("anti-spin INCOMPLETE: pream=%d ftr=%d metric=%.3f delay=%d — ring will shift by 8 syms (preserve preamble=%d)",
							pream_symb, ftr,
							telecom_system->receive_stats.coarse_metric,
							received_message_stats.delay,
							in_v2_sack_dispatch ? 1 : 0);
					}
					else if(pream_symb > upper)
					{
						if(telecom_system->receive_stats.coarse_metric >= 0.5)
						{
							// Beyond-bounds with meaningful metric: fast-forward
							// to bring the preamble within bounds in one shift.
							// +20 margin ensures the preamble lands well within
							// bounds even with timing jitter.
							ftr = pream_symb - upper + 20;
						}
						else
						{
							// Low-metric beyond-bounds: false detection in decoded
							// frame's data region (OFDM symbols create Schmidl-Cox
							// peaks at metric 0.08-0.11). Reset search_raw to 0 to
							// break the cascade — without this, search_raw -= ftr
							// below walks backwards ~8 syms/iter causing 20+ FAILs.
							// Fresh search from prescan gives only 1-2 FAILs.
							ftr = 8;
							telecom_system->receive_stats.ofdm_search_raw = 0;
							telecom_system->receive_stats.ofdm_batch_active = false;
						}
						if(telecom_system->receive_stats.ofdm_search_raw > 0)
						{
							// Track buffer shift: reduce search_raw by ftr so the
							// anti-re-decode skip stays aligned with frame positions.
							telecom_system->receive_stats.ofdm_search_raw -= ftr;
							if(telecom_system->receive_stats.ofdm_search_raw < 0)
							{
								telecom_system->receive_stats.ofdm_search_raw = 0;
								telecom_system->receive_stats.ofdm_batch_active = false;
							}
						}
						printf("[RX-TIMING] OFDM beyond-bounds: pream=%d upper=%d metric=%.3f shift=%d search_raw=%d\n",
							pream_symb, upper,
							telecom_system->receive_stats.coarse_metric,
							ftr,
							telecom_system->receive_stats.ofdm_search_raw);
						fflush(stdout);
					}
					else if(telecom_system->receive_stats.ofdm_search_raw > 0
						&& telecom_system->receive_stats.ofdm_batch_active)
					{
						if(telecom_system->receive_stats.coarse_metric < 0.5)
						{
							// Low-metric FAIL in batch → no more real frames.
							// Real preambles have metric ≥ 0.9; false GI peaks from
							// data symbols or silence give 0.16–0.24. Exit batch
							// but keep search_raw nonzero so next search skips past
							// decoded frames. nUnder decay naturally resets ofdm_skip
							// to 0 as the ring buffer rotates past old audio.
							// Resetting to 0 here causes false preamble detections
							// in the DATA region of already-decoded frames.
							telecom_system->receive_stats.ofdm_batch_active = false;
							ftr = 8;
						}
						else
						{
							// High-metric FAIL in batch (≥0.5) — likely a real
							// preamble that LDPC couldn't decode (rare on clean
							// cable, common at low SNR). Minimal shift to find
							// the next real preamble nearby.
							ftr = 2;
							telecom_system->receive_stats.ofdm_search_raw -= ftr;
							if(telecom_system->receive_stats.ofdm_search_raw < 0)
							{
								telecom_system->receive_stats.ofdm_search_raw = 0;
								telecom_system->receive_stats.ofdm_batch_active = false;
							}
						}
					}
				else if(telecom_system->receive_stats.coarse_metric >= 0.15
					&& telecom_system->receive_stats.coarse_metric < 0.5)
					{
						// Medium-metric within-bounds FAIL (not in batch): likely
						// false preamble from NB OFDM data autocorrelation.
						// NB (Nc=10) has higher Schmidl-Cox metric variance than
						// WB (Nc=50), producing false peaks at 0.15-0.45.
						// These waste LDPC decode time and push real preambles
						// past the buffer boundary, causing NAcks.
						// Zero the false detection region to prevent re-detection,
						// then retry quickly with minimal shift.
						// Sub-threshold detections (metric < detection_threshold)
						// are caught by early return in receive_byte() but now
						// preserve delay at the detected position (not -1), so
						// they CAN reach this path for zeroing when metric >= 0.15.
						{
							int sp = signal_period;
							int pream_samples = telecom_system->data_container.preamble_nSymb * symbol_period;
							int pream_ring_start = (rwi + received_message_stats.delay) % sp;

							MUTEX_LOCK(&capture_prep_mutex);
							for(int k = 0; k < pream_samples; k++)
							{
								int pos = (pream_ring_start + k) % sp;
								telecom_system->data_container.passband_delayed_data[pos] = 0.0;
								telecom_system->data_container.passband_delayed_data[pos + sp] = 0.0;
							}
							MUTEX_UNLOCK(&capture_prep_mutex);
						}
						ftr = 2;
						telecom_system->receive_stats.ofdm_search_raw = 0;
						telecom_system->receive_stats.ofdm_batch_active = false;
						printf("[FTR-FALSE] pream=%d metric=%.3f — zeroed, retry\n",
							pream_symb,
							telecom_system->receive_stats.coarse_metric);
						fflush(stdout);
					}
				// Default ftr (8 WB, 16 NB) applies for other cases
				// (no preamble, high metric without batch mode, etc.)
				}

				if(telecom_system->receive_stats.ofdm_search_raw > 0)
				{
					// Preserve effective search position: ofdm_search_raw - nUnder.
					// Resetting nUnder to 0 without adjusting search_raw would make
					// the effective position jump forward by nUnder symbols, skipping
					// valid preambles.
					int nUnder_snap = telecom_system->data_container.nUnder_processing_events.load();
					telecom_system->receive_stats.ofdm_search_raw -= nUnder_snap;
					if(telecom_system->receive_stats.ofdm_search_raw < 0)
					{
						telecom_system->receive_stats.ofdm_search_raw = 0;
						telecom_system->receive_stats.ofdm_batch_active = false;
					}
				}
				// Monitor mode: when parallel decoders are active, they handle
				// all 17 configs simultaneously — no sequential scan needed.
				// Only use opportunistic scan as fallback if decoders aren't ready.
				if(passive_monitor && !monitor_decoders_ready)
				{
					int frame_symb = telecom_system->data_container.preamble_nSymb
						+ telecom_system->data_container.Nsymb;
					int min_ftr = frame_symb * 2;
					if(ftr < min_ftr) ftr = min_ftr;

					monitor_consec_ofdm_fail++;
					if(monitor_consec_ofdm_fail >= 3 && is_ofdm_config(current_configuration))
					{
						int cur = current_configuration;
						int next = (cur + 1) % NUMBER_OF_CONFIGS;
						int cur_mod = modulation_for_ofdm_config(cur);
						int next_mod = modulation_for_ofdm_config(next);
						bool same_mod = (cur_mod == next_mod);

						printf("[MONITOR] Opportunistic: CONFIG_%d fail #%d → CONFIG_%d (%s)\n",
							cur, monitor_consec_ofdm_fail, next,
							same_mod ? "same-mod, instant" : "cross-mod, wait");
						fflush(stdout);

						data_configuration = next;
						forward_configuration = next;
						load_configuration(next, PHYSICAL_LAYER_ONLY, YES);

						if(same_mod)
						{
							telecom_system->receive_stats.ofdm_search_raw = 0;
							telecom_system->receive_stats.ofdm_batch_active = false;
							ftr = 0;
						}
						else
						{
							frame_symb = telecom_system->data_container.preamble_nSymb
								+ telecom_system->data_container.Nsymb;
							ftr = frame_symb * 2;
						}
					}
				}
				// MULTI-CW WINDOW FIX (fact-doc §17): a big-block decode FAIL that lands in
				// this OFDM anti-spin handler (total acq miss -> no carve re-arm at :7036)
				// would re-arm a small false-preamble shift (8/16), truncating the next
				// block's window. Block-span the re-arm at the rung so the next snapshot
				// waits for a WHOLE block (the carve's full-ring wipe + search_raw reset
				// already handle stale preambles, so a longer wait is safe). NO-OP off-rung.
				// Skip when ftr==0 (the same-mod opportunistic-scan immediate-rescan path,
				// passive_monitor only) so that fast-scan semantics are unchanged.
				if(ftr > 0) ftr = bigblock_block_ftr_or(ftr);
				telecom_system->data_container.frames_to_read = ftr;
				telecom_system->data_container.nUnder_processing_events = 0;
				// === DIAG: OFDM anti-spin ftr trace ===
				// Suppress during rapid same-mod opportunistic scan (ftr==0)
				if(ftr > 0)
				{
					printf("[FTR-FAIL] CONFIG_%d ftr=%d delay=%d metric=%.3f batch=%d search_raw=%d consec_fail=%d\n",
						current_configuration, ftr,
						received_message_stats.delay,
						telecom_system->receive_stats.coarse_metric,
						telecom_system->receive_stats.ofdm_batch_active ? 1 : 0,
						telecom_system->receive_stats.ofdm_search_raw,
						passive_monitor ? monitor_consec_ofdm_fail : -1);
					fflush(stdout);
				}
			}

			// MFSK anti-spin (Bug #37): without this, MFSK NO-PREAMBLE leaves
			// frames_to_read=0 → capture thread never shifts the buffer →
			// receive_byte reprocesses the same content indefinitely.
			//
			// Signal-aware strategy (Bug #43):
			// - NO-PREAMBLE (delay==-1): buffer has noise/silence. Use a small
			//   shift (8 symbols ~181ms) to poll quickly for an incoming frame.
			//   This avoids a 12s penalty when the buffer simply has no signal.
			// - LDPC-FAIL (delay>=0): structured MFSK tones present but decode
			//   failed. Full-frame shift to skip past the stale/bad frame.
			//   Set mfsk_search_raw to search only the fresh region, preventing
			//   partial decode of incomplete frames (Bug #42).
			if(telecom_system->M == MOD_MFSK && telecom_system->data_container.frames_to_read == 0)
			{
				if(received_message_stats.delay >= 0)
				{
					// Structured signal present — full-frame shift
					int ftr = telecom_system->data_container.preamble_nSymb
						+ telecom_system->data_container.Nsymb;
					telecom_system->data_container.frames_to_read = ftr;
					telecom_system->data_container.nUnder_processing_events = 0;
					int buf_nsymb = telecom_system->data_container.buffer_Nsymb.load();
					telecom_system->receive_stats.mfsk_search_raw = buf_nsymb - ftr;
					if(telecom_system->receive_stats.mfsk_search_raw < 0)
						telecom_system->receive_stats.mfsk_search_raw = 0;
				}
				else
				{
					// No signal — small shift, poll again quickly.
					// Preserve effective mfsk_search_raw position.
					int nUnder_snap = telecom_system->data_container.nUnder_processing_events.load();
					telecom_system->receive_stats.mfsk_search_raw -= nUnder_snap;
					if(telecom_system->receive_stats.mfsk_search_raw < 0)
						telecom_system->receive_stats.mfsk_search_raw = 0;
					telecom_system->data_container.frames_to_read = 8;
					telecom_system->data_container.nUnder_processing_events = 0;
				}
			}

			// OFDM anti-spin: on FAIL with structured signal (preamble found but
			// LDPC failed), skip past the detected preamble to avoid re-finding it.
			// Same pattern as MFSK anti-spin above.
			if(telecom_system->M != MOD_MFSK
				&& telecom_system->data_container.frames_to_read == 0
				&& received_message_stats.delay >= 0)
			{
				int rx_frame = telecom_system->get_active_nsymb()
					+ telecom_system->data_container.preamble_nSymb;
				// MULTI-CW WINDOW FIX (fact-doc §17): a FAILED big-block decode (the carve
				// forces message_decoded=NO) lands here when frames_to_read hit 0. Re-arming
				// a stock frame truncates the NEXT block's window (cw1..7 stale-ring garbage)
				// — the chicken-and-egg recurrence (only a CLEAN carve arms the full window
				// at :6994). Block-span the re-arm at the rung so a failed block re-accumulates
				// a WHOLE block before the next snapshot.
				telecom_system->data_container.frames_to_read = bigblock_block_ftr_or(rx_frame);
				telecom_system->data_container.nUnder_processing_events = 0;
				int buf_nsymb = telecom_system->data_container.buffer_Nsymb.load();
				telecom_system->receive_stats.ofdm_search_raw = buf_nsymb - rx_frame;
				if(telecom_system->receive_stats.ofdm_search_raw < 0)
				{
					telecom_system->receive_stats.ofdm_search_raw = 0;
					telecom_system->receive_stats.ofdm_batch_active = false;
				}
				SACK_TRACE("anti-spin OFDM-FAIL full-frame: ftr=%d delay=%d search_raw=%d — ring shifts by full frame",
					rx_frame, received_message_stats.delay,
					telecom_system->receive_stats.ofdm_search_raw);
			}
			else if(telecom_system->M != MOD_MFSK && received_message_stats.delay >= 0)
			{
				// OFDM anti-spin SKIP: ftr != 0, so anti-spin doesn't apply.
				// Frame will be shifted out naturally via ftr countdown.
			}

			if(telecom_system->data_container.frames_to_read==0 && telecom_system->receive_stats.delay_of_last_decoded_message!=-1)
			{
				telecom_system->receive_stats.delay_of_last_decoded_message -= telecom_system->data_container.Nofdm*telecom_system->data_container.interpolation_rate;
				if(telecom_system->receive_stats.delay_of_last_decoded_message < 0)
				{
					telecom_system->receive_stats.delay_of_last_decoded_message = -1;
				}
			}
		}
		// Return here - we already unlocked the mutex at line 1929 and cleared data_ready
		return;
	}

	// frames_to_read != 0: just clear data_ready and unlock
	telecom_system->data_container.data_ready = 0;
	MUTEX_UNLOCK(&capture_prep_mutex);
}


// FIX-6 Mouth B (defense-in-depth, paired with the non-lossy drain in
// process_buffer_data_responder). cl_fifo_buffer::push() (fifo_buffer.cc:85)
// copies NOTHING and returns 0 when length > free_size — pre-fix the four
// producers below ignored that and counted the bytes as delivered, so once
// Mouth A lets the FIFO legitimately back up (app socket slow) this would move
// the silent loss one layer up. This helper closes that: if the chunk does not
// fit, it first DRAINS the FIFO to the app socket (freeing room) and retries;
// only if it STILL cannot fit (genuinely stuck socket AND full 128 KB FIFO) does
// it LOUD-LOG and report the shortfall so the caller does not advance delivery
// accounting for un-stored bytes. Returns the number of bytes actually stored
// (== len on success). The bench3 stall is fixed by Mouth A alone (the FIFO was
// drained empty by the lossy pop); Mouth B exists so the fix cannot reintroduce
// a sibling silent-drop under a faster burst (CLAUDE.md §5).
int cl_arq_controller::fifo_push_rx(const char* buf, int len)
{
	if(len <= 0) return 0;

	int pushed = fifo_buffer_rx.push((char*)buf, len);
	if(pushed == len) return pushed;

	// Didn't fit: drain the app socket to free room, then retry once.
	// (Responder-only path; the CMD has no symmetric RX→app drain — audit §4.5.)
	if(original_role == RESPONDER)
		process_buffer_data_responder();

	pushed = fifo_buffer_rx.push((char*)buf, len);
	if(pushed == len) return pushed;

	// Still cannot store the whole chunk — the app reader is stuck and the FIFO is
	// full. Surface it LOUDLY (never silent) and report the shortfall; the caller
	// must NOT count the un-stored bytes as delivered.
	printf("[RX-FIFO-FULL] app socket back-pressured, FIFO full: stored %d of %d bytes (held)\n",
		pushed, len);
	fflush(stdout);
	return pushed;
}

void cl_arq_controller::copy_data_to_buffer()
{
	int copied = 0;
	int total_bytes = 0;

	// ROBUST_0 compression-deadlock fix (data-flow-compress-frame-fill.md §5/§6).
	// MUST mirror the TX gate in process_buffer_data_commander(): both sides use
	// compression_viable_for_batch() so a robust batch (where batch_capacity ==
	// the streaming header) is a plain headerless DATA frame on both ends. If
	// only one side flipped, the RX would mis-parse a headerless frame as a
	// compression header. Both peers compute this identically from the shared
	// config + the batch=1 robust invariant (data-flow-batch-size.md §1).
	if(compression_viable_for_batch())
	{
		// --- Batch-level decompression ---
		// Reassemble ACKED frames from current batch into one contiguous buffer,
		// then decompress as a single block.
		// IMPORTANT: Only iterate data_batch_size slots (not nMessages) to avoid
		// including stale ACKED data from previous batches in higher-numbered slots.
		char assembled[16384];
		int assembled_size = 0;

		for(int i=0;i<this->data_batch_size;i++)
		{
			if(messages_rx[i].status==ACKED)
			{
				if(assembled_size + messages_rx[i].length <= (int)sizeof(assembled))
				{
					memcpy(assembled + assembled_size,
						messages_rx[i].data, messages_rx[i].length);
					assembled_size += messages_rx[i].length;
				}
				messages_rx[i].status=FREE;
				copied++;
			}
			else
			{
				messages_rx[i].status=FREE;
			}
		}
		// Clear any stale slots beyond batch boundary
		for(int i=this->data_batch_size;i<this->nMessages;i++)
			messages_rx[i].status=FREE;

		if(assembled_size >= compressor.get_header_size())
		{
			// --- Decrypt batch (after reassembly, before decompression) ---
			char* comp_data = assembled;
			int comp_len = assembled_size;
			char decrypt_buf[16384];

			if(cipher_suite.is_active() && assembled_size > 0)
			{
				// Always use full 16-byte auth tag (matches TX side)
				int tag_size = AUTH_TAG_SIZE;
				uint32_t rx_direction = (original_role == COMMANDER)
					? DIRECTION_RSP_TO_CMD : DIRECTION_CMD_TO_RSP;
				printf("[CRYPTO-RX] Decrypting %d bytes, counter=%llu dir=%u tag=%d config=%d\n",
					assembled_size, (unsigned long long)rx_batch_counter,
					rx_direction, tag_size, (int)data_configuration);
				fflush(stdout);
				int plain_size = cipher_suite.decrypt(
					(const uint8_t*)assembled, assembled_size,
					(uint8_t*)decrypt_buf, sizeof(decrypt_buf),
					rx_batch_counter, rx_direction,
					tag_size);
				if(plain_size > 0)
				{
					comp_data = decrypt_buf;
					comp_len = plain_size;
					rx_batch_counter++;
					printf("[CRYPTO-RX] Decrypted: %d -> %d bytes OK\n",
						assembled_size, plain_size);
					fflush(stdout);
				}
				else
				{
					consecutive_auth_failures++;
					printf("[CRYPTO] Decrypt FAILED (batch %llu, fails=%d) — PSK MISMATCH\n",
						(unsigned long long)rx_batch_counter, consecutive_auth_failures);
					fflush(stdout);

					// Report error on control port
					const char* err_msg = "ENCRYPTION FAILURE\r";
					int elen = (int)strlen(err_msg);
					for(int e=0; e<elen; e++)
						tcp_socket_control.message->buffer[e] = err_msg[e];
					tcp_socket_control.message->length = elen;
					tcp_socket_control.transmit();

#ifdef MERCURY_GUI_ENABLED
					g_gui_state.encryption_psk_mismatch.store(true);
					gui_push_monitor_event("[PSK MISMATCH — authentication failed, disconnecting]", false);
#endif
					// Disconnect immediately — mismatched PSK is unrecoverable
					printf("[CRYPTO] Authentication failure — disconnecting\n");
					fflush(stdout);
					this->link_status = DROPPED;
					reset_session_state();
					goto copy_data_done;
				}
			}

			char decomp_buf[COMPRESS_WORKSPACE_SIZE];
			int dec_size = compressor.decompress_block(
				comp_data, comp_len,
				decomp_buf, (int)sizeof(decomp_buf));
			if(dec_size > 0)
			{
#ifdef MERCURY_GUI_ENABLED
				// Monitor tap: plaintext after decompression
				gui_push_monitor_text(decomp_buf, dec_size, false);
#endif
				// Headless monitor: output plaintext to stdout
				if(monitor_stdout)
				{
					fwrite(decomp_buf, 1, dec_size, stdout);
					fflush(stdout);
				}
				int stored = fifo_push_rx(decomp_buf, dec_size);
				total_bytes += stored;
				// Streaming: commit context (raw data = decompressed output).
				// R1 (audit): commit ONLY on a full store so a held batch is not
				// double-committed into the PPMd/zstd carry on re-delivery.
				if(stored == dec_size && compressor.is_streaming())
					compressor.streaming_commit((unsigned char*)decomp_buf, dec_size);
				// Reset auth failure counter on success
				if(cipher_suite.is_active())
					consecutive_auth_failures = 0;
				// Update compression ratio on responder side (EMA)
				int comp_payload = comp_len - compressor.get_header_size();
				if(comp_payload > 0)
				{
					float measured = (float)dec_size / (float)comp_payload;
					compress_ratio_estimate = 0.7f * compress_ratio_estimate + 0.3f * measured;
				}
#ifdef MERCURY_GUI_ENABLED
				// Push algo to GUI from decompressed header (responder side).
				// Mask off the dict-version bits (5-7) so the algo nibble (bits 0-1)
				// is not mislabeled as RAW when a dict-primed frame stamps a version.
				g_gui_state.compression_algo.store((int)((unsigned char)comp_data[0] & COMPRESS_ALGO_MASK));
#endif
			}
			else
			{
				// Decompression error — reset streaming and push raw as fallback
				if(compressor.is_streaming())
					compressor.streaming_reset();
				const unsigned char* ehdr = (const unsigned char*)comp_data;
				printf("[DECOMPRESS] Batch error (assembled %d bytes, hdr: algo=%d comp=%d orig=%d), pushing raw\n",
					comp_len, (int)ehdr[0],
					(int)(ehdr[1] | (ehdr[2] << 8)),
					(int)(ehdr[3] | (ehdr[4] << 8)));
				fflush(stdout);
				total_bytes += fifo_push_rx(comp_data, comp_len);
			}
		}
		else if(assembled_size > 0)
		{
			// Too small for compression header — push raw
#ifdef MERCURY_GUI_ENABLED
			gui_push_monitor_text(assembled, assembled_size, false);
#endif
			if(monitor_stdout)
			{
				fwrite(assembled, 1, assembled_size, stdout);
				fflush(stdout);
			}
			total_bytes += fifo_push_rx(assembled, assembled_size);
		}
	}
	else
	{
		// --- No compression: original per-message push ---
		// Only iterate data_batch_size slots for current batch
		for(int i=0;i<this->data_batch_size;i++)
		{
			if(messages_rx[i].status==ACKED)
			{
#ifdef MERCURY_GUI_ENABLED
				gui_push_monitor_text(messages_rx[i].data, messages_rx[i].length, false);
#endif
				if(monitor_stdout)
				{
					fwrite(messages_rx[i].data, 1, messages_rx[i].length, stdout);
					fflush(stdout);
				}
				total_bytes += fifo_push_rx(messages_rx[i].data, messages_rx[i].length);
				messages_rx[i].status=FREE;
				copied++;
			}
			else if(messages_rx[i].status!=FREE)
			{
				messages_rx[i].status=FREE;
			}
		}
		// Clear stale slots beyond batch boundary
		for(int i=this->data_batch_size;i<this->nMessages;i++)
		{
			if(messages_rx[i].status!=FREE)
				messages_rx[i].status=FREE;
		}
	}
copy_data_done:
#ifdef MERCURY_GUI_ENABLED
	if(total_bytes > 0)
		gui_add_throughput_bytes_rx(total_bytes);
#endif
	block_ready=1;
}

void cl_arq_controller::restore_tx_from_compressed()
{
	// R029: every caller of this helper is a recovery/config-change path that
	// frees messages_tx[] and re-queues plaintext to fifo_buffer_tx for re-send
	// under the new config/epoch (BREAK ACK-recovery, BREAK EXHAUSTED, gearshift
	// FRAME-UP-DATA-FAILED, gearshift FRAME-UP). The retransmit queue's frames
	// reference the OLD messages_tx positions and (under encryption) OLD-epoch
	// bytes, so they MUST be discarded here too — they'll re-send as fresh
	// new-data once re-queued.
	clear_retx_queue();

	// When streaming is active, messages_tx was compressed with streaming context
	// that has already advanced the PPMd model. We can't decompress it again.
	// Use the backup buffer (raw plaintext) and reset streaming context.
	if(compressor.is_streaming())
	{
		printf("[RESTORE_TX] Streaming active — resetting context, restoring from backup\n");
		fflush(stdout);
		compressor.streaming_reset();
		compressor.clear_pending();
		for(int i=0; i<nMessages; i++)
			messages_tx[i].status = FREE;
		restore_backup_buffer_data();
		return;
	}

	// When encryption is active, messages_tx contains encrypted+compressed data.
	// Decrypting requires the exact batch counter that was used, which is fragile.
	// The backup buffer always has raw plaintext, so use it directly.
	if(cipher_suite.is_active())
	{
		printf("[RESTORE_TX] Encryption active — restoring from backup buffer\n");
		fflush(stdout);
		for(int i=0; i<nMessages; i++)
			messages_tx[i].status = FREE;
		restore_backup_buffer_data();
		// Rewind TX counter so re-encrypted batch gets same counter
		if(tx_batch_counter > 0) tx_batch_counter--;
		return;
	}

	// Reassemble compressed chunks from messages_tx, decompress back to raw,
	// push raw data to fifo_buffer_tx for re-compression at new config.
	char assembled[16384];
	int assembled_size = 0;

	for(int i=0; i<nMessages; i++)
	{
		if(messages_tx[i].status != FREE && messages_tx[i].length > 0)
		{
			if(assembled_size + messages_tx[i].length <= (int)sizeof(assembled))
			{
				memcpy(assembled + assembled_size,
					messages_tx[i].data, messages_tx[i].length);
				assembled_size += messages_tx[i].length;
			}
		}
		messages_tx[i].status = FREE;
	}

	if(assembled_size >= COMPRESS_HEADER_SIZE_LEGACY)
	{
		char decomp_buf[COMPRESS_WORKSPACE_SIZE];
		int dec_size = compressor.decompress_block(
			assembled, assembled_size,
			decomp_buf, (int)sizeof(decomp_buf));
		if(dec_size > 0)
		{
			fifo_buffer_tx.push_front(decomp_buf, dec_size);
			printf("[RESTORE_TX] Decompressed %d -> %d bytes, pushed to FIFO\n",
				assembled_size, dec_size);
		}
		else
		{
			// Decompression failed — fall back to backup buffer
			printf("[RESTORE_TX] Decompress failed (%d bytes), restoring from backup\n",
				assembled_size);
			restore_backup_buffer_data();
			return;  // backup restore already handles fifo_buffer_backup
		}
	}
	else if(assembled_size > 0)
	{
		// Too small for header — push raw assembled data
		fifo_buffer_tx.push_front(assembled, assembled_size);
		printf("[RESTORE_TX] No header (%d bytes), pushed raw\n", assembled_size);
	}

	// Flush backup — no longer needed, we recovered from messages_tx
	fifo_buffer_backup.flush();
	fflush(stdout);
}

void cl_arq_controller::restore_backup_buffer_data()
{
	// SACK Design A Step 1 — effective DATA_LONG header length gates per-frame
	// payload capacity. In v1 (default) this is the legacy 4-byte value; in v2
	// it grows by 1 byte (matching the wire format the prior batch was sent in).
	int eff_long_hdr = effective_data_long_header_length(sack_v2_enabled);
	int nBackedup_bytes, data_read_size, nMessages;
	nBackedup_bytes=fifo_buffer_backup.get_size()-fifo_buffer_backup.get_free_size();
	if(nBackedup_bytes!=0 && (max_data_length+max_header_length-eff_long_hdr)!=0)
	{
		nMessages=nBackedup_bytes/(max_data_length+max_header_length-eff_long_hdr);

		char restore_buf[N_MAX/8 * 20];
		int total_restore = 0;
		for(int i=0;i<nMessages+1;i++)
		{
			data_read_size=fifo_buffer_backup.pop(restore_buf + total_restore,max_data_length+max_header_length-eff_long_hdr);
			if(data_read_size > 0)
				total_restore += data_read_size;
		}
		if(total_restore > 0)
			fifo_buffer_tx.push_front(restore_buf, total_restore);
	}
}

void cl_arq_controller::print_stats()
{
	printf("\033[2J");  // clean screen
	printf("\033[H");   // go to upper left corner

	if(this->current_configuration!=CONFIG_NONE)
	{
		printf("configuration:CONFIG_%d (%.1f bps)\n", (int)this->current_configuration, telecom_system->rbc);
	}
	else
	{
		printf("configuration: ERROR..( 0 bps)\n");
	}

	// Display audio devices
	extern char *input_dev;
	extern char *output_dev;
	printf("Audio_IN: %s\n", (input_dev ? input_dev : "default"));
	printf("Audio_OUT: %s\n", (output_dev ? output_dev : "default"));

	printf("\n");

	if(this->role==COMMANDER)
	{
		printf("Role:COM call sign= %s\n", this->my_call_sign.c_str());
	}
	else if (this->role==RESPONDER)
	{
		printf("Role:Res call sign= %s\n", this->my_call_sign.c_str());
	}

	if(this->link_status==DROPPED)
	{
		printf("link_status:Dropped\n");
#ifdef MERCURY_GUI_ENABLED
		gui_push_monitor_event("[DISCONNECTED]", true);
#endif
	}
	else if(this->link_status==IDLE)
	{
		printf("link_status:Idle\n");
	}
	else if (this->link_status==CONNECTING)
	{
		printf("link_status:Connecting to %s\n", this->destination_call_sign.c_str());
	}
	else if (this->link_status==CONNECTED)
	{
		printf("link_status:Connected to %s ID= %d\n", this->destination_call_sign.c_str(), (int)this->connection_id);
	}
	else if (this->link_status==DISCONNECTING)
	{
		printf("link_status:Disconnecting\n");
	}
	else if (this->link_status==LISTENING)
	{
		printf("link_status:Listening\n");
	}
	else if (this->link_status==CONNECTION_RECEIVED)
	{
		printf("link_status:Connection Received from %s\n", this->destination_call_sign.c_str());
	}
	else if (this->link_status==CONNECTION_ACCEPTED)
	{
		printf("link_status:Connection Accepted by %s\n", this->destination_call_sign.c_str());
	}
	else if (link_status==NEGOTIATING)
	{
		printf("link_status:Negotiating with %s\n", this->destination_call_sign.c_str());
	}

	if (this->connection_status==TRANSMITTING_DATA)
	{
		printf("connection_status:Transmitting data\n");
	}
	else if (this->connection_status==RECEIVING)
	{
		printf("connection_status:Receiving\n");
	}
	else if (this->connection_status==RECEIVING_ACKS_DATA)
	{
		printf("connection_status:Receiving data Ack\n");
	}
	else if(this->connection_status==ACKNOWLEDGING_DATA)
	{
		printf("connection_status:Acknowledging data\n");
	}
	else if (this->connection_status==TRANSMITTING_CONTROL)
	{
		printf("connection_status:Transmitting control\n");
	}
	else if (this->connection_status==RECEIVING_ACKS_CONTROL)
	{
		printf("connection_status:Receiving control Ack\n");
	}
	else if (this->connection_status==ACKNOWLEDGING_CONTROL)
	{
		printf("connection_status:Acknowledging control\n");
	}
	else if(this->connection_status==IDLE)
	{
		printf("connection_status:Idle\n");
	}

	printf("measurements.SNR_uplink= %.2f\n", measurements.SNR_uplink);
	printf("measurements.SNR_downlink= %.2f\n", measurements.SNR_downlink);
	printf("measurements.signal_stregth_dbm= %.2f\n", measurements.signal_stregth_dbm);
	printf("measurements.frequency_offset= %.2f\n", measurements.frequency_offset);

	printf("\n");

	printf("stats.nSent_data= %d\n", stats.nSent_data);
	printf("stats.nAcked_data= %d\n", stats.nAcked_data);
	printf("stats.nReceived_data= %d\n", stats.nReceived_data);
	printf("stats.nLost_data= %d\n", stats.nLost_data);
	printf("stats.nReSent_data= %d\n", stats.nReSent_data);
	printf("stats.nAcks_sent_data= %d\n", stats.nAcks_sent_data);
	printf("stats.nNAcked_data= %d\n", stats.nNAcked_data);
	printf("stats.ToSend_data:%d\n", this->get_nToSend_messages());

	printf("\n");

	printf("stats.nSent_control= %d\n", stats.nSent_control);
	printf("stats.nAcked_control= %d\n", stats.nAcked_control);
	printf("stats.nReceived_control= %d\n", stats.nReceived_control);
	printf("stats.nLost_control= %d\n", stats.nLost_control);
	printf("stats.nReSent_control= %d\n", stats.nReSent_control);
	printf("stats.nAcks_sent_control= %d\n", stats.nAcks_sent_control);
	printf("stats.nNAcked_control= %d\n", stats.nNAcked_control);

	printf("\n");
	printf("link_timer= %d\n", link_timer.get_elapsed_time_ms());
	printf("watchdog_timer= %d\n", watchdog_timer.get_elapsed_time_ms());
	printf("gear_shift_timer= %d\n", gear_shift_timer.get_elapsed_time_ms());
	printf("receiving_timer= %d\n", receiving_timer.get_elapsed_time_ms());

	printf("\n");
	printf("last_received_message_sequence= %d\n", (int)last_received_message_sequence);

	printf("last_transmission_block_success_rate= %d %%\n", (int)last_transmission_block_stats.success_rate_data);
	if(gear_shift_blocked_for_nBlocks<gear_shift_block_for_nBlocks_total)
	{
		printf("gear_shift_blocked_for_nBlocks= %d\n", (int)gear_shift_blocked_for_nBlocks);
	}
	else
	{
		printf("gear_shift_blocked_for_nBlocks=\n");
	}

	printf("\n");

	const char* msg_sent_str = "";
	if (this->last_message_sent_type==NONE)
	{
		msg_sent_str = "last_message_sent:";
	}
	else if (this->last_message_sent_type==DATA_LONG)
	{
		msg_sent_str = "last_message_sent:DATA:DATA_LONG";
	}
	else if (this->last_message_sent_type==DATA_SHORT)
	{
		msg_sent_str = "last_message_sent:DATA:DATA_SHORT";
	}
	else if (this->last_message_sent_type==ACK_MULTI)
	{
		msg_sent_str = "last_message_sent:DATA:ACK_MULTI";
	}
	else if (this->last_message_sent_type==ACK_RANGE)
	{
		msg_sent_str = "last_message_sent:DATA:ACK_RANGE";
	}
	else if (this->last_message_sent_type==CONTROL)
	{
		msg_sent_str = "last_message_sent:CONTROL:";
	}
	else if (this->last_message_sent_type==ACK_CONTROL)
	{
		msg_sent_str = "last_message_sent:ACK_CONTROL:";
	}

	const char* msg_sent_code_str = "";
	if(this->last_message_sent_type==CONTROL || this->last_message_sent_type==ACK_CONTROL)
	{
		if (this->last_message_sent_code==START_CONNECTION) msg_sent_code_str = "START_CONNECTION";
		else if (this->last_message_sent_code==TEST_CONNECTION) msg_sent_code_str = "TEST_CONNECTION";
		else if (this->last_message_sent_code==CLOSE_CONNECTION) msg_sent_code_str = "CLOSE_CONNECTION";
		else if (this->last_message_sent_code==KEEP_ALIVE) msg_sent_code_str = "KEEP_ALIVE";
		else if (this->last_message_sent_code==FILE_START) msg_sent_code_str = "FILE_START";
		else if (this->last_message_sent_code==FILE_END_) msg_sent_code_str = "FILE_END";
		else if (this->last_message_sent_code==PIPE_OPEN) msg_sent_code_str = "PIPE_OPEN";
		else if (this->last_message_sent_code==PIPE_CLOSE) msg_sent_code_str = "PIPE_CLOSE";
		else if (this->last_message_sent_code==SWITCH_ROLE) msg_sent_code_str = "SWITCH_ROLE";
		else if (this->last_message_sent_code==BLOCK_END) msg_sent_code_str = "BLOCK_END";
		else if (this->last_message_sent_code==SET_CONFIG) msg_sent_code_str = "SET_CONFIG";
		else if (this->last_message_sent_code==REPEAT_LAST_ACK) msg_sent_code_str = "REPEAT_LAST_ACK";
		else if (this->last_message_sent_code==SWITCH_BANDWIDTH) msg_sent_code_str = "SWITCH_BANDWIDTH";
	}
	printf("%s%s\n", msg_sent_str, msg_sent_code_str);

	const char* msg_recv_str = "";
	if (this->last_message_received_type==NONE)
	{
		msg_recv_str = "last_message_received:";
	}
	else if (this->last_message_received_type==DATA_LONG)
	{
		msg_recv_str = "last_message_received:DATA:DATA_LONG";
	}
	else if (this->last_message_received_type==DATA_SHORT)
	{
		msg_recv_str = "last_message_received:DATA:DATA_SHORT";
	}
	else if (this->last_message_received_type==ACK_MULTI)
	{
		msg_recv_str = "last_message_received:DATA:ACK_MULTI";
	}
	else if (this->last_message_received_type==ACK_RANGE)
	{
		msg_recv_str = "last_message_received:DATA:ACK_RANGE";
	}
	else if (this->last_message_received_type==CONTROL)
	{
		msg_recv_str = "last_message_received:CONTROL:";
	}
	else if (this->last_message_received_type==ACK_CONTROL)
	{
		msg_recv_str = "last_message_received:ACK_CONTROL:";
	}

	const char* msg_recv_code_str = "";
	if(this->last_message_received_type==CONTROL || this->last_message_received_type==ACK_CONTROL)
	{
		if (this->last_message_received_code==START_CONNECTION) msg_recv_code_str = "START_CONNECTION";
		else if (this->last_message_received_code==TEST_CONNECTION) msg_recv_code_str = "TEST_CONNECTION";
		else if (this->last_message_received_code==CLOSE_CONNECTION) msg_recv_code_str = "CLOSE_CONNECTION";
		else if (this->last_message_received_code==KEEP_ALIVE) msg_recv_code_str = "KEEP_ALIVE";
		else if (this->last_message_received_code==FILE_START) msg_recv_code_str = "FILE_START";
		else if (this->last_message_received_code==FILE_END_) msg_recv_code_str = "FILE_END";
		else if (this->last_message_received_code==PIPE_OPEN) msg_recv_code_str = "PIPE_OPEN";
		else if (this->last_message_received_code==PIPE_CLOSE) msg_recv_code_str = "PIPE_CLOSE";
		else if (this->last_message_received_code==SWITCH_ROLE) msg_recv_code_str = "SWITCH_ROLE";
		else if (this->last_message_received_code==BLOCK_END) msg_recv_code_str = "BLOCK_END";
		else if (this->last_message_received_code==SET_CONFIG) msg_recv_code_str = "SET_CONFIG";
		else if (this->last_message_received_code==REPEAT_LAST_ACK) msg_recv_code_str = "REPEAT_LAST_ACK";
		else if (this->last_message_received_code==SWITCH_BANDWIDTH) msg_recv_code_str = "SWITCH_BANDWIDTH";
	}
	printf("%s%s\n", msg_recv_str, msg_recv_code_str);

	printf("\n");
	printf("TX buffer occupancy= %.2f %%\n", (float)(fifo_buffer_tx.get_size()-fifo_buffer_tx.get_free_size())*100.0f/(float)fifo_buffer_tx.get_size());
	printf("RX buffer occupancy= %.2f %%\n", (float)(fifo_buffer_rx.get_size()-fifo_buffer_rx.get_free_size())*100.0f/(float)fifo_buffer_rx.get_size());
	printf("Backup buffer occupancy= %.2f %%\n", (float)(fifo_buffer_backup.get_size()-fifo_buffer_backup.get_free_size())*100.0f/(float)fifo_buffer_backup.get_size());
	fflush(stdout);
}

uint16_t cl_arq_controller::CRC12_calc(const char* data_byte, int nBytes)
{
	// MSB-first / forward CRC-12 with POLY_CRC12=0xF13 (CRC-12-CDMA2000),
	// init 0xFFF, no final XOR. Distinct from CRC8_calc() which uses the
	// reflected/right-shift variant for legacy compatibility.
	uint16_t crc = 0xFFF;
	for(int j=0; j < nBytes; j++)
	{
		crc ^= ((uint16_t)(uint8_t)data_byte[j]) << 4;
		for (int i = 0; i < 8; i++)
		{
			if (crc & 0x800)
				crc = (uint16_t)((crc << 1) ^ POLY_CRC12);
			else
				crc = (uint16_t)(crc << 1);
		}
		crc &= 0xFFF;
	}
	return crc;
}

uint8_t cl_arq_controller::CRC8_calc(char* data_byte, int nItems)
{
	uint8_t crc = 0xff;
	for(int j=0; j < nItems; j++)
	{
		crc ^= data_byte[j];
		for (int i = 0; i < 8; i++)
		{
			if ((crc & 0x01) == 0x01)
			{
				crc = crc >> 1;
				crc ^= POLY_CRC8;
			}
			else
			{
				crc = crc >> 1;
			}
		}
	}
	return crc;
	//ref: MODBUS over serial line specification and implementation guide V1.02, Dec 20,2006, available at https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
}

uint32_t cl_arq_controller::CRC32_calc(const char* data_byte, int nItems)
{
	// Reflected IEEE 802.3 CRC-32 (poly 0xEDB88320, init 0xFFFFFFFF, final XOR
	// 0xFFFFFFFF). Bit-serial reflected form so the implementation is allocator-free
	// and matches the standard zlib/Ethernet CRC-32 used across the in-tree ffbase
	// crc32 family. Used as the big-block whole-block integrity anchor on top of the
	// per-codeword CRC-8 (datalink_defines.h BIGBLOCK_BLOCK_CRC_*). No table needed —
	// runs over a 1374-byte block exactly once per RX block (negligible cost).
	uint32_t crc = 0xFFFFFFFFu;
	for(int j=0; j < nItems; j++)
	{
		crc ^= (uint32_t)(unsigned char)data_byte[j];
		for(int i=0; i<8; i++)
		{
			if(crc & 1u) crc = (crc >> 1) ^ 0xEDB88320u;
			else         crc = (crc >> 1);
		}
	}
	return crc ^ 0xFFFFFFFFu;
}

