/* Audio subsystem
 *
 * Copyright (C) 2024 Rhizomatica
 * Author: Rafael Diniz <rafael@rhizomatica.org>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */


#include <stdint.h>
#include <stdbool.h>
/* R006: shutdown_ is an atomic flag shared with main.cc (std::atomic<bool>).
 * NOTE: although named *.c, this TU is compiled as C++ (build.sh:463 — it pulls
 * in C++ headers via gui_state.h), so the C++ <atomic> header / std::atomic is
 * the correct cross-TU-compatible declaration here (a C11 _Atomic keyword would
 * not even parse under the C++ compiler). */
#ifdef __cplusplus
#include <atomic>
#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <vector>
#else
#include <stdatomic.h>
#endif
#include <limits.h>
#include <string.h>
#include <math.h>
#ifdef _WIN32
#include <wchar.h>
#endif
#include <ffaudio/audio.h>
#include "std.h"
#include "../../include/audioio/audioio.h"
#ifdef FF_LINUX
#include <time.h>
#endif

#include "common/ring_buffer_posix.h"
#include "common/shm_posix.h"
#include "common/common_defines.h"
#include "common/os_interop.h"
#include "common/sim_clock.h"

// SIM channel backend (-x sim) socket headers. winsock2.h is pulled in by
// os_interop.h on Windows; POSIX sockets on everything else.
#if !defined(_WIN32)
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif
#include <stdlib.h>   // getenv, atoi

#ifdef MERCURY_GUI_ENABLED
#ifdef __cplusplus
extern "C++" {
#include "gui/gui_state.h"
}
#endif
#endif

// R006 fix (race audit 2026-06-06): shutdown_ is written from this TU's audio
// capture/playback/sim threads and read in main.cc's main-thread spin loops.
// As a plain `bool` that concurrent unsynchronized access is a data race (UB).
// It is defined as `std::atomic<bool> shutdown_` inside main.cc's `extern "C"`
// block (C language linkage, unmangled name). This TU is compiled as C++
// (build.sh:463), so we MUST declare it with the SAME C language linkage and
// SAME type or the link will fail with an undefined symbol. std::atomic<bool>
// is a standard-layout type and may have C language linkage. The simple
// `shutdown_ = true` stores and `!shutdown_` / `if(shutdown_)` reads below
// resolve to the atomic's seq_cst store / load operators — drop-in for the
// existing plain-bool usage.
#ifdef __cplusplus
extern "C" { extern std::atomic<bool> shutdown_; }
#else
extern _Atomic bool shutdown_;
#endif
extern int radio_type;

// GUARD 1 (paired with sim_arq_channel.py require_guard_binary / GUARD 2).
//
// Under -x sim the device threads (radio_capture_thread/radio_playback_thread)
// are NEVER created — audioio_init_internal short-circuits to the bridge threads
// and returns (see the AUDIO_SUBSYSTEM_SIM branch). But a STALE or wrong build
// that predates the SIM backend would fall through to the real device and LEAK
// MODEM TONES out of the user's physical speakers (this happened twice; see the
// harness GUARD-2 comment). To make the leak structurally detectable AND
// impossible:
//   (a) the string SIM_AUDIO_GUARD_MARKER is compiled into every guard-protected
//       binary, so the harness can grep the binary and refuse to launch one that
//       lacks it (GUARD 2, tools/sim/sim_arq_channel.py), and
//   (b) g_sim_audio_guard_active is latched true when -x sim is selected; the two
//       device-open threads abort-before-render if they are ever entered while it
//       is set (belt-and-suspenders if any future refactor wires a device thread
//       under sim).
// Declared here (before the device threads that reference it) and kept in lockstep
// with SIM_AUDIO_GUARD_MARKER in tools/sim/sim_arq_channel.py.
static const char SIM_AUDIO_GUARD_MARKER[] = "[SIM-AUDIO-GUARD]";
static volatile int g_sim_audio_guard_active = 0;

// Audio channel configuration (set from main.cc / GUI settings)
// 0=LEFT, 1=RIGHT, 2=STEREO (L+R)
int configured_input_channel = 0;   // Default: LEFT (matches pre-GUI CLI default)
int configured_output_channel = 2;  // Default: STEREO
int multichannel_mode = 0;          // Set to 1 when -A flag is used (forces 16ch WASAPI)

// Internal AWGN noise injection (-Z flag)
// When noise_snr_db < 999, white noise is added to captured audio at the specified SNR.
// SNR is relative to the signal level AFTER TX/RX gain (i.e. the cable level).
double noise_snr_db = 999.0;       // 999 = disabled
double noise_signal_dbfs = -30.0;  // Expected signal level on wire (dBFS), set via NOISESIGNAL cmd
static uint64_t noise_rng_state[2] = {0x853c49e6748fea9bULL, 0xda3e39cb94b95bdbULL};

// xoshiro128+ PRNG — fast, good quality for noise generation
static inline uint64_t noise_rng_next(void) {
	uint64_t s0 = noise_rng_state[0], s1 = noise_rng_state[1];
	uint64_t result = s0 + s1;
	s1 ^= s0;
	noise_rng_state[0] = ((s0 << 55) | (s0 >> 9)) ^ s1 ^ (s1 << 14);
	noise_rng_state[1] = (s1 << 36) | (s1 >> 28);
	return result;
}

// Box-Muller: two uniform → two Gaussian
static inline double noise_gaussian(void) {
	double u1 = (noise_rng_next() >> 11) * (1.0 / 9007199254740992.0);  // (0,1)
	double u2 = (noise_rng_next() >> 11) * (1.0 / 9007199254740992.0);
	if (u1 < 1e-15) u1 = 1e-15;
	return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

// Headless RX digital gain (set from main.cc via -G flag or INI RxGainDb)
// 1.0 = unity (0 dB). Applied unconditionally in capture path.
double rx_gain_linear = 1.0;

// Tune tone state (for GUI tune button)
static long tune_sample_index = 0;

cbuf_handle_t capture_buffer;
cbuf_handle_t playback_buffer;

// START-ACK causal sample metadata. The ordinary capture FIFO and this
// generation-tag FIFO are always written/read/reset as one pair under
// capture_pair_mutex, so scheduler delay and capture FIFO backlog cannot make
// an old sample look newer than the commander's published causal deadline.
static cbuf_handle_t capture_causal_tag_buffer = NULL;
static uint8_t *capture_causal_tag_storage = NULL;
static cl_telecom_system *capture_telecom_system = NULL;
#ifdef __cplusplus
static std::mutex capture_pair_mutex;
// Conservative maximum age of samples still queued inside the hardware
// backend when audio->read has not returned them yet. Initialized fail-closed
// and tightened from the opened backend's actual buffer length. The software
// relay has no device queue and sets this to zero.
static std::atomic<uint64_t> capture_source_queue_guard_ns{
	100ULL * 1000000ULL};
// Upper bound from the playback thread consuming a sample to the opened
// backend rendering it. The sample-rate value is the post-open rate; a slower
// negotiated rate lengthens the queued-sample duration instead of silently
// making the START causal deadline early.
static std::atomic<uint64_t> playback_sink_queue_guard_ns{
	100ULL * 1000000ULL};
static std::atomic<uint32_t> playback_sink_sample_rate_hz{48000U};
static thread_local uint32_t capture_writer_seen_generation = 0;
static thread_local bool capture_writer_causal_ready = false;

// Device opens happen inside the capture/playback threads.  Publish their
// startup result so audioio_init_internal() can return a meaningful status
// instead of reporting success before either backend has opened.
enum audio_startup_status {
	AUDIO_START_PENDING = 0,
	AUDIO_START_READY = 1,
	AUDIO_START_FAILED = -1
};
static std::mutex audio_startup_mutex;
static std::condition_variable audio_startup_cv;
static int capture_startup_status = AUDIO_START_PENDING;
static int playback_startup_status = AUDIO_START_PENDING;
static bool capture_thread_started = false;
static bool playback_thread_started = false;
static bool capture_prep_thread_started = false;

static void publish_audio_startup(bool capture, int status)
{
	{
		std::lock_guard<std::mutex> lock(audio_startup_mutex);
		int& current = capture ? capture_startup_status : playback_startup_status;
		if(current == AUDIO_START_PENDING)
			current = status;
	}
	audio_startup_cv.notify_all();
}
#endif

int audio_subsystem;

// Phase-F validation: --alsa-buffer-ms=N override. 0 = use built-in default
// (30 ms on Linux). Tests whether shrinking the ALSA buffer reduces the
// per-cycle audio-path latency on Pi (suspected 506ms PI-vs-HOST gap source).
int g_audio_buffer_ms_override = 0;

#if defined(_WIN32)
    HANDLE            capture_prep_mutex;
#else
    pthread_mutex_t   capture_prep_mutex;
#endif


// tap to file FOR DEBUGGING PURPOSES //
#define ENABLE_FLOAT64_TAP 0
#define ENABLE_FLOAT64_TAP_BEFORE 0
#if ENABLE_FLOAT64_TAP_BEFORE == 1
	FILE *tap_play;
#endif
// FOR DEBUGGING PURPOSES //


struct conf {
	const char *cmd;
	ffaudio_conf buf;
	uint8_t flags;
	uint8_t exclusive;
	uint8_t hwdev;
	uint8_t loopback;
	uint8_t nonblock;
	uint8_t wav;
};


static inline void ffthread_sleep(ffuint msec)
{
#ifdef FF_WIN
	Sleep(msec);
#else
	struct timespec ts = {
		.tv_sec = msec / 1000,
		.tv_nsec = (msec % 1000) * 1000000,
	};
	nanosleep(&ts, NULL);
#endif
}

// SIM-mode cooperative wait. The -x sim bridge + RX-prep loops are paced by
// fixed ffthread_sleep(ms) waits that are correct for a real device (the
// device delivers audio in real time) but would throttle the device-free
// channel to ~1x real time and erase the faster-than-real-time speed-up. When
// the sim virtual clock is active, sleep a SHORT real interval (~200 us)
// instead: it hands the core to the thread that makes progress (relay TCP /
// capture_buffer) WITHOUT pegging a core, and — unlike a raw yield — keeps the
// two peer processes loosely paced together so their independent virtual
// clocks stay coupled through the relay (a hot yield here helped desync the
// CMD<->RSP half-duplex turnaround). When sim is disabled this is the stock
// ffthread_sleep(ms) — production unchanged.
static inline void sim_paced_wait(ffuint msec)
{
	if (sim_clock_enabled())
	{
		// ~200 us cooperative pace. On Windows the finest Sleep() granularity
		// is ~1 ms (Sleep(0) would just yield/hot-spin), so use a 1 ms floor
		// there; on POSIX nanosleep gives true sub-ms. Either way this is far
		// shorter than the virtual durations being waited, so the speed-up
		// holds while the two peer processes stay loosely paced.
#if defined(FF_WIN)
		Sleep(1);
#else
		struct timespec ts = { .tv_sec = 0, .tv_nsec = 200 * 1000 };  // 200 us
		nanosleep(&ts, NULL);
#endif
	}
	else
	{
		ffthread_sleep(msec);
	}
}

#if defined(_WIN32)
/**
 * Convert device name string to GUID for DirectSound
 * Returns allocated GUID pointer on success, NULL if device not found or on error
 * Caller must free the returned pointer
 */
static void* dsound_device_name_to_guid(const char *device_name, ffuint mode)
{
	if (device_name == NULL)
		return NULL;

	ffaudio_interface *audio = (ffaudio_interface *) &ffdsound;
	ffaudio_init_conf aconf = {};
	if (audio->init(&aconf) != 0)
		return NULL;

	ffaudio_dev *d = audio->dev_alloc(mode);
	if (d == NULL) {
		audio->uninit();
		return NULL;
	}

	void *result_guid = NULL;
	ffsize device_name_len = strlen(device_name);

	for (;;) {
		int r = audio->dev_next(d);
		if (r > 0)
			break;  // No more devices
		if (r < 0) {
			// Error
			break;
		}

		const char *name = audio->dev_info(d, FFAUDIO_DEV_NAME);
		if (name != NULL) {
			ffsize name_len = strlen(name);
			ffsize min_len = (name_len < device_name_len) ? name_len : device_name_len;

			// Match using prefix comparison to handle truncated device names
			// This allows "Microphone (2- USB Audio CODEC )" to match "Microphone (2- USB Audio CODEC " (truncated)
			if (strncmp(name, device_name, min_len) == 0) {
				// Found matching device
				const void *guid_ptr = audio->dev_info(d, FFAUDIO_DEV_ID);
				if (guid_ptr != NULL) {
					// Allocate and copy GUID
					result_guid = malloc(sizeof(GUID));
					if (result_guid != NULL) {
						memcpy(result_guid, guid_ptr, sizeof(GUID));
					}
				}
				break;
			}
		}
	}

	audio->dev_free(d);
	audio->uninit();
	return result_guid;
}

/**
 * Convert device name string to wide-string device ID for WASAPI
 * Returns allocated wchar_t* on success, NULL if device not found or on error
 * Caller must free the returned pointer
 */
static void* wasapi_device_name_to_id(const char *device_name, ffuint mode)
{
	if (device_name == NULL)
		return NULL;

	ffaudio_interface *audio = (ffaudio_interface *) &ffwasapi;
	ffaudio_init_conf aconf = {};
	if (audio->init(&aconf) != 0)
		return NULL;

	ffaudio_dev *d = audio->dev_alloc(mode);
	if (d == NULL) {
		audio->uninit();
		return NULL;
	}

	wchar_t *result_id = NULL;
	ffsize device_name_len = strlen(device_name);

	for (;;) {
		int r = audio->dev_next(d);
		if (r > 0)
			break;  // No more devices
		if (r < 0) {
			// Error
			break;
		}

		const char *name = audio->dev_info(d, FFAUDIO_DEV_NAME);
		if (name != NULL) {
			ffsize name_len = strlen(name);
			ffsize min_len = (name_len < device_name_len) ? name_len : device_name_len;

			// Match using prefix comparison to handle truncated device names
			if (strncmp(name, device_name, min_len) == 0) {
				// Found matching device - get the wide-string ID
				const wchar_t *id_ptr = (const wchar_t *)audio->dev_info(d, FFAUDIO_DEV_ID);
				if (id_ptr != NULL) {
					// Allocate and copy wide string
					ffsize id_len = wcslen(id_ptr) + 1;
					result_id = (wchar_t *)malloc(id_len * sizeof(wchar_t));
					if (result_id != NULL) {
						wcscpy(result_id, id_ptr);
					}
				}
				break;
			}
		}
	}

	audio->dev_free(d);
	audio->uninit();
	return result_id;
}


/**
 * Validate audio device configuration for Mercury
 * Checks that devices exist and are configured correctly (stereo, 48kHz)
 *
 * Returns bitmask of errors:
 *   0 = OK
 *   1 = Capture device not found
 *   2 = Playback device not found
 *   4 = Capture device not stereo (must be 2 channels)
 *   8 = Playback device not stereo (must be 2 channels)
 */
int validate_audio_config(const char *capture_dev, const char *playback_dev, int audio_system)
{
	int errors = 0;

	// Only WASAPI provides mix format info
	if (audio_system != AUDIO_SUBSYSTEM_WASAPI) {
		printf("[AUDIO CHECK] DirectSound selected - format validation not available\n");
		printf("[AUDIO CHECK] Recommendation: Use WASAPI (-x wasapi) for virtual audio cables\n");
		return 0;
	}

	ffaudio_interface *audio = (ffaudio_interface *) &ffwasapi;
	ffaudio_init_conf aconf = {};
	if (audio->init(&aconf) != 0) {
		printf("[AUDIO CHECK] ERROR: Failed to initialize WASAPI\n");
		return 1 | 2;
	}

	printf("\n");
	printf("========================================================================\n");
	printf("  MERCURY AUDIO CONFIGURATION CHECK\n");
	printf("========================================================================\n\n");

	// Check capture device
	printf("Checking CAPTURE device...\n");
	{
		ffaudio_dev *d = audio->dev_alloc(FFAUDIO_DEV_CAPTURE);
		if (d == NULL) {
			printf("  ERROR: Failed to enumerate capture devices\n");
			errors |= 1;
		} else {
			int found = 0;
			const char *target_name = capture_dev;
			ffsize target_len = target_name ? strlen(target_name) : 0;

			for (;;) {
				int r = audio->dev_next(d);
				if (r > 0) break;
				if (r < 0) break;

				const char *name = audio->dev_info(d, FFAUDIO_DEV_NAME);
				const char *is_default = audio->dev_info(d, FFAUDIO_DEV_IS_DEFAULT);

				int is_match = 0;
				if (target_name == NULL && is_default != NULL) {
					is_match = 1;
				} else if (target_name != NULL && name != NULL) {
					ffsize name_len = strlen(name);
					ffsize min_len = (name_len < target_len) ? name_len : target_len;
					if (strncmp(name, target_name, min_len) == 0) {
						is_match = 1;
					}
				}

				if (is_match) {
					found = 1;
					printf("  Device: %s%s\n", name, is_default ? " (DEFAULT)" : "");

					const ffuint *fmt = (const ffuint *)audio->dev_info(d, FFAUDIO_DEV_MIX_FORMAT);
					if (fmt != NULL) {
						ffuint format = fmt[0];
						ffuint sample_rate = fmt[1];
						ffuint channels = fmt[2];

						const char *fmt_name = "UNKNOWN";
						if (format == FFAUDIO_F_INT16) fmt_name = "INT16";
						else if (format == FFAUDIO_F_INT32) fmt_name = "INT32";
						else if (format == FFAUDIO_F_FLOAT32) fmt_name = "FLOAT32";

						printf("  Format: %s / %u Hz / %u channels\n", fmt_name, sample_rate, channels);

						if (channels == 1) {
							printf("  Channels: OK (mono)\n");
						} else if (channels == 2) {
							printf("  Channels: OK (stereo)\n");
						} else {
							printf("  Channels: %u (multi-channel)\n", channels);
						}

						if (sample_rate != 48000) {
							printf("  *** WARNING: Sample rate %u Hz, recommended 48000 Hz ***\n", sample_rate);
						} else {
							printf("  Sample rate: OK (48000 Hz)\n");
						}
					}
					break;
				}
			}

			if (!found) {
				printf("  ERROR: Device '%s' not found\n", target_name ? target_name : "(default)");
				errors |= 1;
			}
			audio->dev_free(d);
		}
	}

	printf("\n");

	// Check playback device
	printf("Checking PLAYBACK device...\n");
	{
		ffaudio_dev *d = audio->dev_alloc(FFAUDIO_DEV_PLAYBACK);
		if (d == NULL) {
			printf("  ERROR: Failed to enumerate playback devices\n");
			errors |= 2;
		} else {
			int found = 0;
			const char *target_name = playback_dev;
			ffsize target_len = target_name ? strlen(target_name) : 0;

			for (;;) {
				int r = audio->dev_next(d);
				if (r > 0) break;
				if (r < 0) break;

				const char *name = audio->dev_info(d, FFAUDIO_DEV_NAME);
				const char *is_default = audio->dev_info(d, FFAUDIO_DEV_IS_DEFAULT);

				int is_match = 0;
				if (target_name == NULL && is_default != NULL) {
					is_match = 1;
				} else if (target_name != NULL && name != NULL) {
					ffsize name_len = strlen(name);
					ffsize min_len = (name_len < target_len) ? name_len : target_len;
					if (strncmp(name, target_name, min_len) == 0) {
						is_match = 1;
					}
				}

				if (is_match) {
					found = 1;
					printf("  Device: %s%s\n", name, is_default ? " (DEFAULT)" : "");

					const ffuint *fmt = (const ffuint *)audio->dev_info(d, FFAUDIO_DEV_MIX_FORMAT);
					if (fmt != NULL) {
						ffuint format = fmt[0];
						ffuint sample_rate = fmt[1];
						ffuint channels = fmt[2];

						const char *fmt_name = "UNKNOWN";
						if (format == FFAUDIO_F_INT16) fmt_name = "INT16";
						else if (format == FFAUDIO_F_INT32) fmt_name = "INT32";
						else if (format == FFAUDIO_F_FLOAT32) fmt_name = "FLOAT32";

						printf("  Format: %s / %u Hz / %u channels\n", fmt_name, sample_rate, channels);

						if (channels == 1) {
							printf("  Channels: OK (mono)\n");
						} else if (channels == 2) {
							printf("  Channels: OK (stereo)\n");
						} else {
							printf("  Channels: %u (multi-channel)\n", channels);
						}

						if (sample_rate != 48000) {
							printf("  *** WARNING: Sample rate %u Hz, recommended 48000 Hz ***\n", sample_rate);
						} else {
							printf("  Sample rate: OK (48000 Hz)\n");
						}
					}
					break;
				}
			}

			if (!found) {
				printf("  ERROR: Device '%s' not found\n", target_name ? target_name : "(default)");
				errors |= 2;
			}
			audio->dev_free(d);
		}
	}

	printf("\n========================================================================\n");
	if (errors == 0) {
		printf("  AUDIO CONFIGURATION: OK\n");
	} else {
		printf("  AUDIO CONFIGURATION: ERRORS FOUND (code %d)\n", errors);
		printf("\n");
		printf("  Common issues with VB-Cable or virtual audio:\n");
		printf("  1. Device must be set to STEREO (2 channels) in Windows Sound settings\n");
		printf("  2. Both Input and Output should use same sample rate (48000 Hz)\n");
		printf("  3. Use WASAPI audio system (-x wasapi) for virtual cables\n");
	}
	printf("========================================================================\n\n");

	audio->uninit();
	return errors;
}
#endif


void *radio_playback_thread(void *device_ptr)
{
    // GUARD 1: never open a real playback device while -x sim is active.
    if (g_sim_audio_guard_active) {
        fprintf(stderr, "FATAL %s: radio_playback_thread entered under -x sim; "
                "aborting before touching a real audio device.\n",
                SIM_AUDIO_GUARD_MARKER);
        fflush(stderr);
        abort();
    }
    ffaudio_interface *audio;
	int device_is_mono = 0;  // Will be set after device opens
	int out_ch_idx = 0;
	int out_stereo = 0;
	int out_nch = 2;
	struct conf conf = {};
	conf.buf.app_name = "mercury_playback";
	conf.buf.format = FFAUDIO_F_INT32;
	conf.buf.sample_rate = 48000;
	// When -A flag is used (multichannel_mode=1), request 16 channels so
	// WASAPI shared-mode streams all use the same format. This allows noise
	// injection and multiple Mercury instances to mix on the same device.
	if (multichannel_mode)
		conf.buf.channels = 16;
	else
		conf.buf.channels = 2;
	conf.buf.device_id = (const char *) device_ptr;
	uint32_t period_ms;
	uint32_t period_bytes;

#if defined(_WIN32)
	void *dsound_guid = NULL;  // For DirectSound GUID allocated memory
	void *wasapi_id = NULL;    // For WASAPI device ID allocated memory
#endif

#if defined(_WIN32)
    conf.buf.buffer_length_msec = 40;
	period_ms = conf.buf.buffer_length_msec / 4;
    if (audio_subsystem == AUDIO_SUBSYSTEM_WASAPI) {
        audio = (ffaudio_interface *) &ffwasapi;
		// Convert device name to wide-string ID for WASAPI
		if (device_ptr != NULL) {
			wasapi_id = wasapi_device_name_to_id((const char*)device_ptr, FFAUDIO_DEV_PLAYBACK);
			if (wasapi_id != NULL) {
				conf.buf.device_id = (const char*)wasapi_id;
			} else {
				fprintf(stderr, "ERROR: WASAPI playback device '%s' not found\n",
				        (const char*)device_ptr);
				publish_audio_startup(false, AUDIO_START_FAILED);
				shutdown_ = true;
				return NULL;
			}
		}
	}
    if (audio_subsystem == AUDIO_SUBSYSTEM_DSOUND) {
        audio = (ffaudio_interface *) &ffdsound;
		// DirectSound: Keep INT32 format (DirectSound handles conversion to device format)
		// conf.buf.format = FFAUDIO_F_INT16;  // Disabled - using INT32 default
		// Convert device name to GUID for DirectSound
		if (device_ptr != NULL) {
			dsound_guid = dsound_device_name_to_guid((const char*)device_ptr, FFAUDIO_DEV_PLAYBACK);
			if (dsound_guid != NULL) {
				conf.buf.device_id = (const char*)dsound_guid;
			} else {
				fprintf(stderr, "ERROR: DirectSound playback device '%s' not found\n",
				        (const char*)device_ptr);
				publish_audio_startup(false, AUDIO_START_FAILED);
				shutdown_ = true;
				return NULL;
			}
		}
	}
#elif defined(__linux__)
    conf.buf.buffer_length_msec = 30;
    if (g_audio_buffer_ms_override > 0) conf.buf.buffer_length_msec = g_audio_buffer_ms_override;
	period_ms = conf.buf.buffer_length_msec / 3;
    if (audio_subsystem == AUDIO_SUBSYSTEM_ALSA)
        audio = (ffaudio_interface *) &ffalsa;
    if (audio_subsystem == AUDIO_SUBSYSTEM_PULSE)
        audio = (ffaudio_interface *) &ffpulse;
#elif defined(__FREEBSD__)
    conf.buf.buffer_length_msec = 40;
	period_ms = conf.buf.buffer_length_msec / 4;
    if (audio_subsystem == AUDIO_SUBSYSTEM_OSS)
        audio = (ffaudio_interface *) &ffoss;
#elif defined(__APPLE__)
    conf.buf.buffer_length_msec = 40;
	period_ms = conf.buf.buffer_length_msec / 4;
    if (audio_subsystem == AUDIO_SUBSYSTEM_COREAUDIO)
        audio = (ffaudio_interface *) &ffcoreaudio;
#endif

	period_bytes = conf.buf.sample_rate * sizeof(double) * period_ms / 1000;

	//printf("period_ms: %u\n", period_ms);
	//printf("period_size: %u\n", period_bytes);
	conf.flags = FFAUDIO_PLAYBACK;
	ffaudio_init_conf aconf = {};
	aconf.app_name = "mercury_playback";

	int r;
	ffaudio_buf *b;
	ffaudio_conf *cfg;

	ffuint frame_size;
	ffuint msec_bytes;

	uint8_t *buffer = (uint8_t *) malloc(AUDIO_PAYLOAD_BUFFER_SIZE * sizeof(double) * 2);
	double *buffer_double =  (double *) buffer;
	int out_nch_alloc = multichannel_mode ? 16 : 2;
	int32_t *buffer_internal_stereo = (int32_t *) malloc(AUDIO_PAYLOAD_BUFFER_SIZE * sizeof(int32_t) * out_nch_alloc);

	ffuint total_written = 0;

	// TX clipping detection
	int tx_clip_count = 0;
	int tx_clip_samples_total = 0;
	double tx_clip_peak = 0.0;
	int tx_clip_report_counter = 0;

#if ENABLE_FLOAT64_TAP == 1
	FILE *tap_pay = fopen("tap-playback.f64", "w");
#endif

	if ( audio->init(&aconf) != 0)
    {
        printf("Error in audio->init()\n");
		publish_audio_startup(false, AUDIO_START_FAILED);
        goto finish_play;
    }

    // playback code...
	b = audio->alloc();
	if (b == NULL)
	{
		printf("Error in audio->alloc()\n");
		publish_audio_startup(false, AUDIO_START_FAILED);
		goto finish_play;
	}

	cfg = &conf.buf;
	r = audio->open(b, cfg, conf.flags);
	if (r == FFAUDIO_EFORMAT)
		r = audio->open(b, cfg, conf.flags);
	if (r != 0)
	{
		printf("error in audio->open(): %d: %s\n", r, audio->error(b));
		publish_audio_startup(false, AUDIO_START_FAILED);
		goto cleanup_play;
	}
	publish_audio_startup(false, AUDIO_START_READY);

	printf("I/O playback (%s) format=%d (%s) / %dHz / %dch / %dms buffer\n",
		conf.buf.device_id ? conf.buf.device_id : "default",
		cfg->format,
		(cfg->format == FFAUDIO_F_INT16) ? "INT16" : (cfg->format == FFAUDIO_F_INT32) ? "INT32" : (cfg->format == FFAUDIO_F_FLOAT32) ? "FLOAT32" : "UNKNOWN",
		cfg->sample_rate, cfg->channels, cfg->buffer_length_msec);
	fflush(stdout);
	{
		const uint64_t buffer_ms = cfg->buffer_length_msec > 0
			? (uint64_t)cfg->buffer_length_msec : 100ULL;
		const uint32_t sample_rate = cfg->sample_rate > 0
			? (uint32_t)cfg->sample_rate : 48000U;
		// As on capture, use twice the opened backend buffer because several
		// backends partition the requested buffer internally.
		playback_sink_queue_guard_ns.store(
			2ULL * buffer_ms * 1000000ULL);
		playback_sink_sample_rate_hz.store(sample_rate);
	}


	frame_size = cfg->channels * (cfg->format & 0xff) / 8;
	msec_bytes = cfg->sample_rate * frame_size / 1000;

	// Determine output channel index (0-based)
	// For multi-channel devices (>2ch), configured_output_channel is used directly as index.
	// For 2ch devices: 0=LEFT, 1=RIGHT, 2=STEREO (backwards compatible).
	out_ch_idx = configured_output_channel;
	out_stereo = 0;
	device_is_mono = (cfg->channels == 1);
	if (!device_is_mono && cfg->channels == 2 && configured_output_channel == 2)
		out_stereo = 1;  // STEREO only for 2-channel devices
	if (out_ch_idx >= (int)cfg->channels)
		out_ch_idx = 0;  // safety fallback
	out_nch = cfg->channels;

	// Clock-drift instrumentation: track effective playback sample rate.
	struct timespec clk_tx_start, clk_tx_window_start, clk_tx_prev_call;
	long long clk_tx_window_frames;
	long long clk_tx_cum_frames;
	int clk_tx_glitch_count;
	clock_gettime(CLOCK_MONOTONIC, &clk_tx_start);
	clk_tx_window_start = clk_tx_start;
	clk_tx_prev_call = clk_tx_start;
	clk_tx_window_frames = 0;
	clk_tx_cum_frames = 0;
	clk_tx_glitch_count = 0;

    while (!shutdown_)
    {
		ffssize n;
		size_t buffer_size = size_buffer(playback_buffer);
		if (buffer_size >= period_bytes)
		{
			read_buffer(playback_buffer, buffer, period_bytes);
			n = period_bytes;
		}
		else
		{
			// we just play zeros if there is nothing to play
			memset(buffer, 0, period_bytes);
			if (buffer_size > frame_size)
				read_buffer(playback_buffer, buffer, buffer_size);
			n = period_bytes;
		}

#if ENABLE_FLOAT64_TAP == 1
		fwrite(buffer, 1, n, tap);
#endif

        total_written = 0;

		int samples_read = n / sizeof(double);

#ifdef MERCURY_GUI_ENABLED
		// Check if tune mode is active - generate 1500 Hz sine wave
		if (g_gui_state.tune_active.load()) {
			for (int i = 0; i < samples_read; i++) {
				buffer_double[i] = gui_generate_tune_tone(48000, tune_sample_index);
			}
		}

		// Apply TX gain from GUI
		gui_apply_tx_gain(buffer_double, samples_read);
#endif

		// convert from double to format-specific output
		// Check if format is FLOAT32 (WASAPI), INT32 (DirectSound/ALSA), or INT16 (DirectSound 16-bit)
		int is_float32 = (cfg->format == FFAUDIO_F_FLOAT32);
		int is_int16 = (cfg->format == FFAUDIO_F_INT16);
		float *buffer_float_out = (float*)buffer_internal_stereo;
		int16_t *buffer_int16_out = (int16_t*)buffer_internal_stereo;

		int clip_this_buffer = 0;
		for (int i = 0; i < samples_read; i++)
		{
			// Clamp to [-1.0, 1.0]
			double clamped = buffer_double[i];
			if (clamped > 1.0 || clamped < -1.0) {
				double absval = clamped > 0 ? clamped : -clamped;
				if (absval > tx_clip_peak) tx_clip_peak = absval;
				clip_this_buffer++;
				if (clamped > 1.0) clamped = 1.0;
				else clamped = -1.0;
			}

			if (device_is_mono)
			{
				// Mono device: single sample per frame
				if (is_float32)
					buffer_float_out[i] = (float)clamped;
				else if (is_int16)
					buffer_int16_out[i] = (int16_t)(clamped * 32767.0);
				else
					buffer_internal_stereo[i] = clamped * INT_MAX;
			}
			else
			{
				// Multi-channel: zero all channels, write active channel(s)
				int idx = i * out_nch;
				if (is_float32) {
					memset(&buffer_float_out[idx], 0, out_nch * sizeof(float));
					buffer_float_out[idx + out_ch_idx] = (float)clamped;
					if (out_stereo)
						buffer_float_out[idx + 1] = (float)clamped;
				} else if (is_int16) {
					memset(&buffer_int16_out[idx], 0, out_nch * sizeof(int16_t));
					buffer_int16_out[idx + out_ch_idx] = (int16_t)(clamped * 32767.0);
					if (out_stereo)
						buffer_int16_out[idx + 1] = buffer_int16_out[idx + out_ch_idx];
				} else {
					memset(&buffer_internal_stereo[idx], 0, out_nch * sizeof(int32_t));
					buffer_internal_stereo[idx + out_ch_idx] = clamped * INT_MAX;
					if (out_stereo)
						buffer_internal_stereo[idx + 1] = buffer_internal_stereo[idx + out_ch_idx];
				}
			}
		}

		// TX clipping report (every ~1s = 48000 samples)
		if (clip_this_buffer > 0) {
			tx_clip_count += clip_this_buffer;
			tx_clip_samples_total += samples_read;
		}
		tx_clip_report_counter += samples_read;
		if (tx_clip_report_counter >= 48000) {
			if (tx_clip_count > 0) {
				printf("[TX-CLIP] %d samples clipped (peak=%.3f, %.1f%% of buffer)\n",
					tx_clip_count, tx_clip_peak,
					100.0 * tx_clip_count / tx_clip_report_counter);
				fflush(stdout);
				tx_clip_count = 0;
				tx_clip_peak = 0.0;
			}
			tx_clip_report_counter = 0;
		}

		n = samples_read * frame_size;

        while (n >= frame_size)
        {
            r = audio->write(b, ((uint8_t *)buffer_internal_stereo) + total_written, n);

            if (r == -FFAUDIO_ESYNC) {
                // Underrun. ALSA/aaudio recover via continue; WASAPI returns
                // ESYNC after a bounded retry budget (wasapi.c:973+). In
                // either case, retrying immediately inside the same producer
                // iteration risks the same backpressure. Drop the remaining
                // bytes for this chunk and break out so the producer thread
                // moves on to the next iteration. The lost ~10 ms of audio
                // shows up as a push-rate glitch in [TX-PUSH-GLITCH] but
                // the thread stays real-time.
                printf("detected underrun, dropping %lld samples (%.1fms)\n",
                       (long long)n / (long long)frame_size,
                       (double)(n / (frame_size > 0 ? frame_size : 1)) / 48.0);
                break;
            }
            if (r < 0)
            {
                printf("ffaudio.write: %s", audio->error(b));
                break;
            }
#if 0 // print time measurement
            else
            {
                printf(" %dms\n", r / msec_bytes);
            }
#endif
            total_written += r;
            n -= r;
        }
        // printf("n = %lld total written = %u\n", n, total_written);

		// Push-rate: accumulate frames played and periodically report rate.
		// Tag is [TX-PUSH-RATE] — producer-push-rate vs wall, NOT the crystal;
		// true skew via tone method is ±8 ppm.
		//
		// CLARIFICATION (SIMFIDELITY_ROOTCAUSE.md §1, 2026-06-10; tag renamed
		// from the misleading [CLK-TX] in C4, 2026-06-10): the drift this
		// reports is a PRODUCER-PUSH-RATE / QUANTIZATION metric, NOT the codec
		// crystal frequency. `samples_read` is ALWAYS one whole `period_bytes`
		// quantum — including the "play zeros if there is nothing to play"
		// branch above (line ~734), so this window count folds in
		// silence/zero-fill and any ESYNC-dropped chunk. The 10 s window also
		// closes a fraction of a period late (the numerator is quantized in
		// whole periods; the denominator drifts up to one period), so a ~±10 ms
		// error on a 10 s window already reads as ±1000 ppm. That is why this
		// metric swings -631 -> +2163 -> -817 ppm across consecutive windows on
		// the SAME pair (a physical crystal cannot do that). The REAL inter-Pi
		// sample-clock skew is ±8.16 ppm (CLOCK_VERDICT.md §2, tone method);
		// the hundreds-of-ppm here is a software measurement artifact (push-rate
		// vs wall, dominated by 10 s-window quantization). Do NOT treat
		// [TX-PUSH-RATE] as the crystal.
		//
		// DRIFT FIELD FIX (window-quantization -> cumulative convergence): the
		// per-window rate carries a ±1-period-per-window quantization error that
		// swings hundreds of ppm and sign-flips window to window on a channel whose
		// true skew is ±8 ppm — a physical crystal cannot do that, so the field was
		// pure artifact. The reported `drift` is now the CUMULATIVE rate
		// (cum_frames / total_wall since stream start): the bounded per-window
		// quantization error averages out ~1/t, so the estimate CONVERGES on the
		// real rate instead of oscillating. The per-window rate is still printed as
		// `wrate` for glitch/underrun diagnostics; `cum_rate`/`drift` is the honest
		// clock estimate.
		clk_tx_window_frames += samples_read;
		clk_tx_cum_frames    += samples_read;
		{
			struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
			double dt_call = (now.tv_sec - clk_tx_prev_call.tv_sec) +
			                 (now.tv_nsec - clk_tx_prev_call.tv_nsec) * 1e-9;
			clk_tx_prev_call = now;
			if (dt_call > 0.025) {
				double dt_total = (now.tv_sec - clk_tx_start.tv_sec) +
				                  (now.tv_nsec - clk_tx_start.tv_nsec) * 1e-9;
				printf("[TX-PUSH-GLITCH] dt_call=%.1fms samples=%d total_t=%.3fs\n",
					dt_call * 1000.0, samples_read, dt_total);
				fflush(stdout);
				clk_tx_glitch_count++;
			}
			double dt_window = (now.tv_sec - clk_tx_window_start.tv_sec) +
			                   (now.tv_nsec - clk_tx_window_start.tv_nsec) * 1e-9;
			if (dt_window >= 10.0) {
				double dt_total = (now.tv_sec - clk_tx_start.tv_sec) +
				                  (now.tv_nsec - clk_tx_start.tv_nsec) * 1e-9;
				double wrate    = clk_tx_window_frames / dt_window;   // per-window (quantization-bounded, diagnostic)
				double cum_rate = (dt_total > 0.0)
					? (double)clk_tx_cum_frames / dt_total : 48000.0; // cumulative (converges on the true rate)
				double drift_ppm = (cum_rate - 48000.0) / 48000.0 * 1e6;
				printf("[TX-PUSH-RATE] dt=%.2fs frames=%lld cum_frames=%lld wrate=%.3f cum_rate=%.3f Hz drift=%+.1f ppm total_t=%.1fs glitches=%d\n",
					dt_window, (long long)clk_tx_window_frames, (long long)clk_tx_cum_frames,
					wrate, cum_rate, drift_ppm, dt_total, clk_tx_glitch_count);
				fflush(stdout);
				clk_tx_window_start = now;
				clk_tx_window_frames = 0;
				clk_tx_glitch_count = 0;
			}
		}
    }

#if ENABLE_FLOAT64_TAP == 1
	fclose(tap);
#endif

    r = audio->drain(b);
    if (r < 0)
        printf("ffaudio.drain: %s", audio->error(b));

    r = audio->stop(b);
    if (r != 0)
        printf("ffaudio.stop: %s", audio->error(b));

    r = audio->clear(b);
    if (r != 0)
        printf("ffaudio.clear: %s", audio->error(b));

cleanup_play:

    audio->free(b);

	audio->uninit();

	finish_play:

	free(buffer);
	free(buffer_internal_stereo);

#if defined(_WIN32)
	// Free DirectSound GUID if allocated
	if (dsound_guid != NULL)
		free(dsound_guid);
	// Free WASAPI device ID if allocated
	if (wasapi_id != NULL)
		free(wasapi_id);
#endif

	printf("radio_playback_thread exit\n");

    shutdown_ = true;

    return NULL;
}


static uint32_t capture_causal_begin_chunk(
	uint32_t generation,
	uint32_t *seen_generation,
	bool *causal_ready)
{
	if(generation != *seen_generation)
	{
		*seen_generation = generation;
		*causal_ready = false;
	}
	return (generation != 0 && *causal_ready) ? generation : 0;
}

static void capture_causal_commit_chunk(
	uint32_t generation,
	uint64_t deadline_ns,
	uint64_t now_ns,
	uint64_t source_guard_ns,
	uint32_t *seen_generation,
	bool *causal_ready)
{
	if(generation == 0 || *seen_generation != generation) return;
	if(now_ns >= deadline_ns
	   && now_ns - deadline_ns >= source_guard_ns)
	{
		// The crossing chunk may still contain queued/pre-deadline audio.
		// Keep its label zero; this state applies to the following chunk.
		*causal_ready = true;
	}
}

static void capture_publish_causal_ring_chunk(
	cl_data_container *dc,
	const uint32_t *tags,
	int symbol_period,
	int signal_period)
{
	if(!dc->start_ack_causal_tracking_available) return;
	const uint32_t generation =
		dc->start_ack_causal_generation.load();
	int trailing = 0;
	while(trailing < symbol_period
	      && generation != 0
	      && tags[(size_t)symbol_period - 1U - (size_t)trailing]
			== generation)
		trailing++;

	int causal_samples = 0;
	if(trailing == symbol_period)
	{
		causal_samples = dc->start_ack_causal_ring_samples.load();
		if(causal_samples < 0) causal_samples = 0;
		if(causal_samples <= signal_period - symbol_period)
			causal_samples += symbol_period;
		else
			causal_samples = signal_period;
	}
	else
	{
		// Producer writes and demod symbols can have different sizes.
		causal_samples = trailing;
	}
	if(causal_samples > signal_period)
		causal_samples = signal_period;
	dc->start_ack_causal_ring_samples = causal_samples;
	// Publish generation last: a matching generation observed by the
	// commander always sees this chunk's sample count.
	dc->start_ack_causal_ring_generation = generation;
}

uint64_t playback_causal_egress_bound_ns(size_t queued_samples)
{
	uint64_t rate = playback_sink_sample_rate_hz.load();
	if(rate == 0 || rate > 48000ULL) rate = 48000ULL;
	const uint64_t samples = (uint64_t)queued_samples;
	const uint64_t whole_seconds = samples / rate;
	const uint64_t remainder = samples % rate;
	uint64_t duration_ns = UINT64_MAX;
	if(whole_seconds <= UINT64_MAX / 1000000000ULL)
	{
		duration_ns = whole_seconds * 1000000000ULL;
		const uint64_t rem_ns =
			(remainder * 1000000000ULL + rate - 1ULL) / rate;
		if(duration_ns <= UINT64_MAX - rem_ns)
			duration_ns += rem_ns;
		else
			duration_ns = UINT64_MAX;
	}
	const uint64_t guard_ns = playback_sink_queue_guard_ns.load();
	if(duration_ns > UINT64_MAX - guard_ns) return UINT64_MAX;
	return duration_ns + guard_ns;
}

int capture_causal_tag_guard_selftest(cl_telecom_system *telecom_system)
{
	uint32_t seen = 0;
	bool ready = false;
	const uint32_t generation = 7;
	const uint64_t deadline = 1000;
	const uint64_t guard = 300;

	// Three immediately returned device chunks after the causal deadline are
	// still within the configured device-queue bound and must remain old.
	const uint64_t queued_times[] = {1000, 1100, 1200};
	for(size_t i = 0; i < sizeof(queued_times)/sizeof(queued_times[0]); i++)
	{
		if(capture_causal_begin_chunk(
			generation, &seen, &ready) != 0) return 1;
		capture_causal_commit_chunk(
			generation, deadline, queued_times[i], guard,
			&seen, &ready);
	}
	// The boundary-crossing chunk is also excluded; only its successor is
	// eligible for this generation.
	if(capture_causal_begin_chunk(generation, &seen, &ready) != 0)
		return 1;
	capture_causal_commit_chunk(
		generation, deadline, deadline + guard,
		guard, &seen, &ready);
	if(capture_causal_begin_chunk(generation, &seen, &ready)
	   != generation)
		return 1;

	// A new START generation immediately invalidates prior readiness.
	if(capture_causal_begin_chunk(
		generation + 1U, &seen, &ready) != 0)
		return 1;

	// Exercise the real paired write/read and the same prep-publication helper
	// used by radio_capture_prep_thread. Standalone focused/full tests have no
	// audio backend; if one is active, leave it untouched and retain the pure
	// producer-state coverage above.
	if(capture_buffer != NULL || capture_causal_tag_buffer != NULL
	   || telecom_system == NULL)
		return 0;

	const size_t sample_capacity = 4096;
	const size_t tag_capacity =
		(sample_capacity / sizeof(double)) * sizeof(uint32_t);
	uint8_t *sample_storage = (uint8_t *)malloc(sample_capacity);
	uint8_t *tag_storage = (uint8_t *)malloc(tag_capacity);
	if(sample_storage == NULL || tag_storage == NULL)
	{
		free(sample_storage);
		free(tag_storage);
		return 1;
	}

	capture_buffer = circular_buf_init(sample_storage, sample_capacity);
	capture_causal_tag_buffer =
		circular_buf_init(tag_storage, tag_capacity);
	capture_causal_tag_storage = tag_storage;
	capture_telecom_system = telecom_system;
	cl_data_container& dc = telecom_system->data_container;
	dc.start_ack_causal_tracking_available = 1;
	dc.start_ack_causal_generation = 19;
	dc.start_ack_causal_ring_generation = 0;
	dc.start_ack_causal_ring_samples = 0;

	capture_writer_seen_generation = 19;
	capture_writer_causal_ready = true;
	double samples[8] = {};
	uint32_t read_tags[8] = {};
	int failed = 0;
	if(capture_write_samples(samples, 8) != 0
	   || rx_transfer_with_causal_tags(samples, read_tags, 8) != 0)
		failed = 1;
	for(int i = 0; i < 8; i++)
		if(read_tags[i] != 19) failed = 1;
	capture_publish_causal_ring_chunk(&dc, read_tags, 8, 64);
	if(dc.start_ack_causal_ring_generation.load() != 19
	   || dc.start_ack_causal_ring_samples.load() != 8)
		failed = 1;

	// Reset invalidates already-read old-generation tags. Publishing that
	// in-flight chunk cannot recreate an eligible suffix.
	uint32_t old_tags[8];
	memcpy(old_tags, read_tags, sizeof(old_tags));
	capture_reset_samples();
	const uint32_t reset_generation =
		dc.start_ack_causal_generation.load();
	capture_publish_causal_ring_chunk(&dc, old_tags, 8, 64);
	if(reset_generation == 19
	   || dc.start_ack_causal_ring_generation.load() != reset_generation
	   || dc.start_ack_causal_ring_samples.load() != 0)
		failed = 1;

	// The next genuinely current-generation chunk advances through the same
	// paired FIFO and prep seam.
	capture_writer_seen_generation = reset_generation;
	capture_writer_causal_ready = true;
	memset(read_tags, 0, sizeof(read_tags));
	if(capture_write_samples(samples, 8) != 0
	   || rx_transfer_with_causal_tags(samples, read_tags, 8) != 0)
		failed = 1;
	capture_publish_causal_ring_chunk(&dc, read_tags, 8, 64);
	if(dc.start_ack_causal_ring_generation.load() != reset_generation
	   || dc.start_ack_causal_ring_samples.load() != 8)
		failed = 1;

	dc.start_ack_causal_tracking_available = 0;
	dc.start_ack_causal_ring_generation = 0;
	dc.start_ack_causal_ring_samples = 0;
	free(capture_buffer->buffer);
	circular_buf_free(capture_buffer);
	capture_buffer = NULL;
	free(capture_causal_tag_buffer->buffer);
	circular_buf_free(capture_causal_tag_buffer);
	capture_causal_tag_buffer = NULL;
	capture_causal_tag_storage = NULL;
	capture_telecom_system = NULL;
	capture_writer_seen_generation = 0;
	capture_writer_causal_ready = false;
	return failed;
}

int capture_write_samples(double *buffer, size_t len)
{
	if(capture_causal_tag_buffer == NULL || capture_telecom_system == NULL)
		return write_buffer(capture_buffer, (uint8_t *)buffer,
			len * sizeof(double));

	cl_data_container *dc = &capture_telecom_system->data_container;
	const uint32_t generation = dc->start_ack_causal_generation.load();
	const uint32_t chunk_tag = capture_causal_begin_chunk(
		generation, &capture_writer_seen_generation,
		&capture_writer_causal_ready);

	static thread_local std::vector<uint32_t> tags;
	tags.assign(len, chunk_tag);

	const size_t sample_bytes = len * sizeof(double);
	const size_t tag_bytes = len * sizeof(uint32_t);
	int result = -1;
	{
		std::lock_guard<std::mutex> pair_guard(capture_pair_mutex);
		if(circular_buf_free_size(capture_buffer) >= sample_bytes
		   && circular_buf_free_size(capture_causal_tag_buffer) >= tag_bytes)
		{
			// Tags first: a reader can never observe samples without their
			// paired generation metadata.
			if(write_buffer(capture_causal_tag_buffer,
					(uint8_t *)tags.data(), tag_bytes) == 0
			   && write_buffer(capture_buffer,
					(uint8_t *)buffer, sample_bytes) == 0)
				result = 0;
		}
	}

	const uint64_t now_ns = sim_clock_now_ns();
	const uint64_t deadline_ns =
		dc->start_ack_causal_deadline_ns.load();
	const uint64_t source_guard_ns =
		capture_source_queue_guard_ns.load();
	if(result == 0
	   && dc->start_ack_causal_generation.load() == generation)
	{
		capture_causal_commit_chunk(
			generation, deadline_ns, now_ns, source_guard_ns,
			&capture_writer_seen_generation,
			&capture_writer_causal_ready);
	}
	return result;
}

void capture_reset_samples(void)
{
	if(capture_buffer == NULL) return;
	std::lock_guard<std::mutex> pair_guard(capture_pair_mutex);
	circular_buf_reset(capture_buffer);
	if(capture_causal_tag_buffer != NULL)
		circular_buf_reset(capture_causal_tag_buffer);
	if(capture_telecom_system != NULL)
	{
		cl_data_container& dc = capture_telecom_system->data_container;
		const uint32_t current =
			dc.start_ack_causal_generation.load();
		if(current != 0)
		{
			uint32_t next = current + 1U;
			if(next == 0) next = 1U;
			// Invalidate any chunk that prep pulled before this reset but has
			// not yet published into the demod ring.
			dc.start_ack_causal_generation = next;
		}
		capture_telecom_system->data_container
			.start_ack_causal_ring_generation = 0;
		capture_telecom_system->data_container
			.start_ack_causal_ring_samples = 0;
	}
}

void *radio_capture_thread(void *device_ptr)
{
    // GUARD 1: never open a real capture device while -x sim is active.
    if (g_sim_audio_guard_active) {
        fprintf(stderr, "FATAL %s: radio_capture_thread entered under -x sim; "
                "aborting before touching a real audio device.\n",
                SIM_AUDIO_GUARD_MARKER);
        fflush(stderr);
        abort();
    }
    ffaudio_interface *audio;
	int device_is_mono = 0;  // Will be set after device opens
	int in_ch_idx = 0;
	int in_stereo = 0;
	int in_nch = 2;
	struct conf conf = {};
	conf.buf.app_name = "mercury_capture";
	conf.buf.format = FFAUDIO_F_INT32;
	conf.buf.sample_rate = 48000;
	// Match playback: request 16 channels when -A flag is used.
	if (multichannel_mode)
		conf.buf.channels = 16;
	else
		conf.buf.channels = 2;
	conf.buf.device_id = (const char *) device_ptr;

#if defined(_WIN32)
	void *dsound_guid = NULL;  // For DirectSound GUID allocated memory
	void *wasapi_id = NULL;    // For WASAPI device ID allocated memory
#endif

#if defined(_WIN32)
    conf.buf.buffer_length_msec = 40;
    if (audio_subsystem == AUDIO_SUBSYSTEM_WASAPI) {
        audio = (ffaudio_interface *) &ffwasapi;
		// Convert device name to wide-string ID for WASAPI
		if (device_ptr != NULL) {
			wasapi_id = wasapi_device_name_to_id((const char*)device_ptr, FFAUDIO_DEV_CAPTURE);
			if (wasapi_id != NULL) {
				conf.buf.device_id = (const char*)wasapi_id;
			} else {
				fprintf(stderr, "ERROR: WASAPI capture device '%s' not found\n",
				        (const char*)device_ptr);
				publish_audio_startup(true, AUDIO_START_FAILED);
				shutdown_ = true;
				return NULL;
			}
		}
		fflush(stdout);
	}
    if (audio_subsystem == AUDIO_SUBSYSTEM_DSOUND) {
        audio = (ffaudio_interface *) &ffdsound;
		// DirectSound: Keep INT32 format (DirectSound handles conversion to device format)
		// conf.buf.format = FFAUDIO_F_INT16;  // Disabled - using INT32 default
		// Convert device name to GUID for DirectSound
		if (device_ptr != NULL) {
			dsound_guid = dsound_device_name_to_guid((const char*)device_ptr, FFAUDIO_DEV_CAPTURE);
			if (dsound_guid != NULL) {
				conf.buf.device_id = (const char*)dsound_guid;
				printf("[CAPTURE INIT] Found device GUID, using specific device\n");
			} else {
				fprintf(stderr, "ERROR: DirectSound capture device '%s' not found\n",
				        (const char*)device_ptr);
				publish_audio_startup(true, AUDIO_START_FAILED);
				shutdown_ = true;
				return NULL;
			}
		} else {
			printf("[CAPTURE INIT] No device specified, using default\n");
		}
		fflush(stdout);
	}
#elif defined(__linux__)
    conf.buf.buffer_length_msec = 30;
    if (g_audio_buffer_ms_override > 0) conf.buf.buffer_length_msec = g_audio_buffer_ms_override;
    if (audio_subsystem == AUDIO_SUBSYSTEM_ALSA)
        audio = (ffaudio_interface *) &ffalsa;
    if (audio_subsystem == AUDIO_SUBSYSTEM_PULSE)
        audio = (ffaudio_interface *) &ffpulse;
#elif defined(__FREEBSD__)
    conf.buf.buffer_length_msec = 40;
    if (audio_subsystem == AUDIO_SUBSYSTEM_OSS)
        audio = (ffaudio_interface *) &ffoss;
#elif defined(__APPLE__)
    conf.buf.buffer_length_msec = 40;
    if (audio_subsystem == AUDIO_SUBSYSTEM_COREAUDIO)
        audio = (ffaudio_interface *) &ffcoreaudio;
#endif

    conf.flags = FFAUDIO_CAPTURE;
	ffaudio_init_conf aconf = {};
	aconf.app_name = "mercury_capture";

	int r;
	ffaudio_buf *b;
    ffaudio_conf *cfg;

    ffuint frame_size;
    ffuint msec_bytes;

	int32_t *buffer = NULL;

	double *buffer_internal = NULL;

	// RX overload detection — average energy based
	double rx_energy_sum = 0.0;
	int rx_energy_sample_count = 0;

#if ENABLE_FLOAT64_TAP == 1
	FILE *tap = fopen("tap-capture.f64", "w");
#endif

	if ( audio->init(&aconf) != 0)
    {
        printf("Error in audio->init()\n");
		publish_audio_startup(true, AUDIO_START_FAILED);
        goto finish_cap;
    }

    // capture code
	b = audio->alloc();
	if (b == NULL)
    {
        printf("Error in audio->alloc()\n");
		publish_audio_startup(true, AUDIO_START_FAILED);
        goto finish_cap;
    }

    cfg = &conf.buf;
	r = audio->open(b, cfg, conf.flags);
	if (r == FFAUDIO_EFORMAT)
		r = audio->open(b, cfg, conf.flags);
	if (r != 0)
    {
        printf("error in audio->open(): %d: %s\n", r, audio->error(b));
		publish_audio_startup(true, AUDIO_START_FAILED);
        goto cleanup_cap;
    }
	publish_audio_startup(true, AUDIO_START_READY);

	printf("I/O capture (%s) format=%d (%s) / %dHz / %dch / %dms buffer\n",
		conf.buf.device_id ? conf.buf.device_id : "default",
		cfg->format,
		(cfg->format == FFAUDIO_F_INT16) ? "INT16" : (cfg->format == FFAUDIO_F_INT32) ? "INT32" : (cfg->format == FFAUDIO_F_FLOAT32) ? "FLOAT32" : "UNKNOWN",
		cfg->sample_rate, cfg->channels, cfg->buffer_length_msec);
	fflush(stdout);
	// A backend may return several immediately available periods after a
	// scheduler stall. Its opened buffer length bounds the age of retained
	// device samples; use twice the reported value (WASAPI/DSound commonly
	// partition the requested buffer internally) and still exclude the whole
	// deadline-crossing chunk in capture_write_samples().
	{
		const uint64_t buffer_ms = cfg->buffer_length_msec > 0
			? (uint64_t)cfg->buffer_length_msec : 100ULL;
		capture_source_queue_guard_ns.store(
			2ULL * buffer_ms * 1000000ULL);
	}

    frame_size = cfg->channels * (cfg->format & 0xff) / 8;
    msec_bytes = cfg->sample_rate * frame_size / 1000;

	buffer_internal = (double *) malloc(AUDIO_PAYLOAD_BUFFER_SIZE * sizeof(double) * 2);

	// Determine input channel index (0-based)
	// For multi-channel devices (>2ch), configured_input_channel is used directly as index.
	// For 2ch devices: 0=LEFT, 1=RIGHT, 2=STEREO (backwards compatible).
	in_ch_idx = configured_input_channel;
	in_stereo = 0;
	device_is_mono = (cfg->channels == 1);
	if (!device_is_mono && cfg->channels == 2 && configured_input_channel == 2)
		in_stereo = 1;
	if (in_ch_idx >= (int)cfg->channels)
		in_ch_idx = 0;  // safety fallback
	in_nch = cfg->channels;

	static int read_loop_counter = 0;

	// Clock-drift instrumentation: track effective capture sample rate over
	// rolling 10 s windows. Reveals ALSA/codec clock divergence from 48 kHz.
	struct timespec clk_rx_start, clk_rx_window_start, clk_rx_prev_call;
	long long clk_rx_cum_frames, clk_rx_window_frames;
	clock_gettime(CLOCK_MONOTONIC, &clk_rx_start);
	clk_rx_window_start = clk_rx_start;
	clk_rx_prev_call = clk_rx_start;
	clk_rx_cum_frames = 0;
	clk_rx_window_frames = 0;
	// Per-call glitch detection: ALSA period is ~10ms (buffer/3 = 30/3).
	// Flag any inter-call wait >25 ms (>2.5x normal) — suggests scheduler
	// preemption or a dropped period.
	int clk_rx_glitch_count;
	clk_rx_glitch_count = 0;

	while (!shutdown_)
    {
		r = audio->read(b, (const void **)&buffer);

		if (r < 0)
        {
			printf("ffaudio.read: %s", audio->error(b));
            continue;
        }
#if 0
        else
        {
            printf(" %dms\n", r / msec_bytes);
        }
#endif

		int frames_read = r / frame_size;
		int frames_to_write = frames_read;

		// Deliver-rate: accumulate frames and periodically report rate.
		// Tag is [RX-DELIVER-RATE] — consumer-deliver-rate vs wall, NOT the
		// crystal; true skew via tone method is ±8 ppm. Same PRODUCER/CONSUMER
		// vs wall window-quantization artifact as [TX-PUSH-RATE] above (renamed
		// from the misleading [CLK-RX] in C4, 2026-06-10); see that comment +
		// SIMFIDELITY_ROOTCAUSE.md §1. Do NOT treat as the crystal.
		clk_rx_cum_frames += frames_read;
		clk_rx_window_frames += frames_read;
		{
			struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
			// Per-call timing: detect scheduler stalls or dropped periods.
			double dt_call = (now.tv_sec - clk_rx_prev_call.tv_sec) +
			                 (now.tv_nsec - clk_rx_prev_call.tv_nsec) * 1e-9;
			clk_rx_prev_call = now;
			if (dt_call > 0.025) {
				// >25 ms gap — log with absolute time for correlation w/ [T] events
				double dt_total = (now.tv_sec - clk_rx_start.tv_sec) +
				                  (now.tv_nsec - clk_rx_start.tv_nsec) * 1e-9;
				printf("[RX-DELIVER-GLITCH] dt_call=%.1fms frames=%d total_t=%.3fs\n",
					dt_call * 1000.0, frames_read, dt_total);
				fflush(stdout);
				clk_rx_glitch_count++;
			}
			double dt_window = (now.tv_sec - clk_rx_window_start.tv_sec) +
			                   (now.tv_nsec - clk_rx_window_start.tv_nsec) * 1e-9;
			if (dt_window >= 10.0) {
				// DRIFT FIELD FIX (see [TX-PUSH-RATE] above): report the CUMULATIVE
				// rate (cum_frames / total_wall) so the bounded per-window
				// quantization error averages out ~1/t and the estimate converges on
				// the true rate; the per-window rate is kept as `wrate` for diagnostics.
				double dt_total = (now.tv_sec - clk_rx_start.tv_sec) +
				                  (now.tv_nsec - clk_rx_start.tv_nsec) * 1e-9;
				double wrate    = clk_rx_window_frames / dt_window;
				double cum_rate = (dt_total > 0.0)
					? (double)clk_rx_cum_frames / dt_total : 48000.0;
				double drift_ppm = (cum_rate - 48000.0) / 48000.0 * 1e6;
				printf("[RX-DELIVER-RATE] dt=%.2fs frames=%lld cum_frames=%lld wrate=%.3f cum_rate=%.3f Hz drift=%+.1f ppm total_t=%.1fs glitches=%d\n",
					dt_window, (long long)clk_rx_window_frames, (long long)clk_rx_cum_frames,
					wrate, cum_rate, drift_ppm, dt_total, clk_rx_glitch_count);
				fflush(stdout);
				clk_rx_window_start = now;
				clk_rx_window_frames = 0;
				clk_rx_glitch_count = 0;
			}
		}

		// Check format: FLOAT32 (WASAPI), INT32 (DirectSound/ALSA), or INT16 (DirectSound with 16-bit)
		int is_float32 = (cfg->format == FFAUDIO_F_FLOAT32);
		int is_int16 = (cfg->format == FFAUDIO_F_INT16);
		float *buffer_float = (float*)buffer;  // For FLOAT32 interpretation
		int16_t *buffer_int16 = (int16_t*)buffer;  // For INT16 interpretation

		for (int i = 0; i < frames_to_write; i++)
		{
			if (device_is_mono)
			{
				// Mono device: single sample per frame
				if (is_float32)
					buffer_internal[i] = (double) buffer_float[i];
				else if (is_int16)
					buffer_internal[i] = (double) buffer_int16[i] / 32768.0;
				else
					buffer_internal[i] = (double) buffer[i] / (double) INT_MAX;
			}
			else if (in_stereo)
			{
				// STEREO: average channels 0+1 (only for 2ch devices)
				if (is_float32)
					buffer_internal[i] = (double) ((buffer_float[i*2] + buffer_float[i*2 + 1]) / 2.0);
				else if (is_int16)
					buffer_internal[i] = (double) ((buffer_int16[i*2] + buffer_int16[i*2 + 1]) / 2.0) / 32768.0;
				else
					buffer_internal[i] = (double) ((buffer[i*2] + buffer[i*2 + 1]) / 2.0) / (double) INT_MAX;
			}
			else
			{
				// Indexed channel: works for 2ch (LEFT/RIGHT) and multi-channel (0-15)
				int pos = i * in_nch + in_ch_idx;
				if (is_float32)
					buffer_internal[i] = (double) buffer_float[pos];
				else if (is_int16)
					buffer_internal[i] = (double) buffer_int16[pos] / 32768.0;
				else
					buffer_internal[i] = (double) buffer[pos] / (double) INT_MAX;
			}
		}

		// Internal AWGN noise injection (-Z flag or NOISESNR command)
		// Adds noise BEFORE RX gain, at the cable signal level.
		// noise_snr_db is referenced to 4 kHz standard HF bandwidth.
		if (noise_snr_db < 999.0) {
			static double noise_sigma = 0.0;
			static double last_snr_db = 999.0;
			static double last_signal_dbfs = -999.0;
			if (noise_snr_db != last_snr_db || noise_signal_dbfs != last_signal_dbfs) {
				double signal_amp = pow(10.0, noise_signal_dbfs / 20.0);
				double bw_correction = sqrt(48000.0 / (2.0 * 4000.0));
				noise_sigma = signal_amp * bw_correction * pow(10.0, -noise_snr_db / 20.0);
				printf("[NOISE-Z] SNR=%.1f dB, signal=%.1f dBFS, sigma=%.6f\n",
					noise_snr_db, noise_signal_dbfs, noise_sigma);
				last_snr_db = noise_snr_db;
				last_signal_dbfs = noise_signal_dbfs;
			}
			// DIAG: measure actual signal RMS before noise injection (every ~1s)
			{
				static int noise_diag_count = 0;
				static double noise_diag_sig_sum = 0;
				static int noise_diag_sig_n = 0;
				for (int i = 0; i < frames_to_write; i++) {
					noise_diag_sig_sum += buffer_internal[i] * buffer_internal[i];
				}
				noise_diag_sig_n += frames_to_write;
				if (++noise_diag_count % 100 == 0 && noise_diag_sig_n > 0) {
					double sig_rms = sqrt(noise_diag_sig_sum / noise_diag_sig_n);
					double sig_dbfs = (sig_rms > 1e-10) ? 20.0 * log10(sig_rms) : -999.0;
					printf("[NOISE-DIAG] signal_rms=%.6f (%.1f dBFS) noise_sigma=%.6f ratio=%.1f expect=%.1f dBFS\n",
						sig_rms, sig_dbfs, noise_sigma, sig_rms / noise_sigma, noise_signal_dbfs);
					noise_diag_sig_sum = 0;
					noise_diag_sig_n = 0;
				}
			}
			for (int i = 0; i < frames_to_write; i++) {
				buffer_internal[i] += noise_sigma * noise_gaussian();
			}
		}

		// Apply RX digital gain (headless path, always active)
		if (rx_gain_linear != 1.0) {
			for (int i = 0; i < frames_to_write; i++) {
				buffer_internal[i] *= rx_gain_linear;
			}
		}

#ifdef MERCURY_GUI_ENABLED
		// RX overload detection: average energy over 1-second window
		for (int i = 0; i < frames_to_write; i++) {
			rx_energy_sum += buffer_internal[i] * buffer_internal[i];
		}
		rx_energy_sample_count += frames_to_write;
		if (rx_energy_sample_count >= 48000) {
			double rms = sqrt(rx_energy_sum / rx_energy_sample_count);
			int overloaded = (rms > 0.794);  // -2 dBFS threshold
			g_gui_state.rx_overload.store(overloaded != 0);
			if (overloaded) {
				printf("[RX-OVERLOAD] avg energy too high: RMS=%.3f (%.1f dBFS)\n",
					rms, 20.0 * log10(rms));
				fflush(stdout);
			}
			rx_energy_sum = 0.0;
			rx_energy_sample_count = 0;
		}

		// Push to VU meter and waterfall
		gui_push_audio_samples(buffer_internal, frames_to_write);
#endif

#if ENABLE_FLOAT64_TAP == 1
		fwrite(buffer_internal, 1, frames_to_write * sizeof(double), tap);
#endif

		// Write (possibly gained) samples to capture_buffer for Mercury's core
		if (circular_buf_free_size(capture_buffer) >= frames_to_write * sizeof(double))
			capture_write_samples(buffer_internal, (size_t)frames_to_write);
		else
			printf("Buffer full in capture buffer!\n");
	}

	r = audio->stop(b);
	if (r != 0)
		printf("ffaudio.stop: %s", audio->error(b));

	r = audio->clear(b);
	if (r != 0)
		printf("ffaudio.clear: %s", audio->error(b));

	free(buffer_internal);

#if ENABLE_FLOAT64_TAP == 1
	fclose(tap);
#endif


cleanup_cap:

	audio->free(b);

    audio->uninit();

finish_cap:

#if defined(_WIN32)
	// Free DirectSound GUID if allocated
	if (dsound_guid != NULL)
		free(dsound_guid);
	// Free WASAPI device ID if allocated
	if (wasapi_id != NULL)
		free(wasapi_id);
#endif

	printf("radio_capture_thread exit\n");

    shutdown_ = true;

    return NULL;
}

void *radio_capture_prep_thread(void *telecom_ptr_void)
{
	cl_telecom_system *telecom_ptr = (cl_telecom_system *) telecom_ptr_void;

	double *buffer_temp = (double *) malloc(AUDIO_PAYLOAD_BUFFER_SIZE * sizeof(double) * 2);
	std::vector<uint32_t> causal_tags;

	// [C1-LOCKWAIT] benchmark-probe accumulators (function-scope so the exit
	// summary can report the true running max even when it never crossed the
	// per-event log threshold — proves Stage-1 sub-us, not merely "< thresh").
	int    c1_lw_probe = getenv("MERCURY_C1_LOCKWAIT") ? 1 : 0;
	double c1_lw_max_us = 0.0;
	double c1_lw_sum_us = 0.0;
	long   c1_lw_samples = 0;
	long   c1_lw_n_over  = 0;

	// [CBC-METER] construction meter (Step-0 prevention needle). Reads the gate
	// once and resets the process-lifetime accumulators so the summary is clean
	// regardless of struct init order. When off, the eat site below is
	// byte-identical to the pre-meter code.
	int          cbc_meter       = getenv("MERCURY_CBC_METER") ? 1 : 0;
	const double CBC_SILENCE_RMS  = 1e-4;   // ~-80 dBFS: below = true silence, ignore
	double       cbc_signal_rms   = 0.05;   // above = a real peer frame (not WGN channel noise)
	{ const char* e = getenv("MERCURY_CBC_SIGNAL_RMS"); if(e && *e) cbc_signal_rms = atof(e); }
	if (cbc_meter) {
		telecom_ptr->data_container.cbc_muted_total_samples  = 0;
		telecom_ptr->data_container.cbc_muted_noise_events   = 0;
		telecom_ptr->data_container.cbc_muted_signal_samples = 0;
		telecom_ptr->data_container.cbc_muted_signal_events  = 0;
		telecom_ptr->data_container.cbc_muted_peak           = 0.0;
		printf("[CBC-METER] enabled (silence_rms_floor=%.1e signal_rms_floor=%.4f)\n",
		       CBC_SILENCE_RMS, cbc_signal_rms);
		fflush(stdout);
	}

	while (!shutdown_)
    {
		cl_data_container *data_container_ptr = &telecom_ptr->data_container;
		int signal_period = data_container_ptr->Nofdm * data_container_ptr->buffer_Nsymb * data_container_ptr->interpolation_rate; // in samples
		int symbol_period = data_container_ptr->Nofdm * data_container_ptr->interpolation_rate;
		int location_of_last_frame = signal_period - symbol_period - 1; // TODO: do we need this "-1"?

		if (symbol_period == 0) {
			continue;
		}

		// Wait for enough data, checking shutdown_ to avoid blocking forever
		{
			size_t needed = symbol_period * sizeof(double);
			while (!shutdown_ && size_buffer(capture_buffer) < needed) {
				sim_paced_wait(1);
			}
			if (shutdown_) break;
		}

		causal_tags.resize((size_t)symbol_period);
		if(rx_transfer_with_causal_tags(
			buffer_temp, causal_tags.data(), (size_t)symbol_period) != 0)
		{
			// Paired FIFO metadata is a fail-closed integrity boundary. Do not
			// place an untagged sample into the demod ring.
			continue;
		}

		// DIAG: capture peak amplitude (every 200 symbols ~4.5s for WB)
		{
			static int cap_pk_count = 0;
			if(++cap_pk_count % 200 == 0) {
				double pk = 0;
				for(int ci = 0; ci < symbol_period; ci++)
					if(fabs(buffer_temp[ci]) > pk) pk = fabs(buffer_temp[ci]);
				printf("[CAP-PEAK] pk=%.6f sp=%d mute=%d\n", pk, symbol_period, (int)data_container_ptr->rx_mute);
				fflush(stdout);
			}
		}

		if(data_container_ptr->rx_mute) {
			// [CBC-METER] Measure the energy about to be discarded. In the
			// device-free two-process relay there is no self-echo, so any
			// above-floor energy captured here is incoming PEER signal being
			// eaten by the mute/flush blind window — the structural loss the
			// F1b sample-anchored re-arm must drive to zero.
			if(cbc_meter) {
				double e2 = 0.0, pk = 0.0;
				for(int ci = 0; ci < symbol_period; ci++) {
					double s = buffer_temp[ci];
					e2 += s * s;
					double a = fabs(s);
					if(a > pk) pk = a;
				}
				double rms = (symbol_period > 0) ? sqrt(e2 / symbol_period) : 0.0;
				data_container_ptr->cbc_muted_total_samples += symbol_period;
				if(rms > cbc_signal_rms) {
					// A full-amplitude peer FRAME landed inside the blind window
					// and is being zeroed = the structural loss (the needle).
					data_container_ptr->cbc_muted_signal_samples += symbol_period;
					data_container_ptr->cbc_muted_signal_events  += 1;
					if(pk > data_container_ptr->cbc_muted_peak)
						data_container_ptr->cbc_muted_peak = pk;
					printf("[CBC-METER-EAT] peer FRAME eaten while muted: rms=%.6f pk=%.6f sp=%d cum_signal_samp=%ld cum_signal_events=%ld\n",
						rms, pk, symbol_period,
						(long)data_container_ptr->cbc_muted_signal_samples,
						(long)data_container_ptr->cbc_muted_signal_events);
					fflush(stdout);
				} else if(rms > CBC_SILENCE_RMS) {
					// Channel noise captured while muted — not the loss, but
					// tracked so the needle is read against a visible denominator.
					data_container_ptr->cbc_muted_noise_events += 1;
				}
			}
			memset(buffer_temp, 0, symbol_period * sizeof(double));
			data_container_ptr->rx_mute_samples += symbol_period;
		}

		// [C1-LOCKWAIT] benchmark probe (default OFF, byte-identical when off).
		// Measures how long THIS capture-prep thread (C1) blocks acquiring
		// capture_prep_mutex. During a config switch the RSP's load_configuration
		// holds this mutex; the DEFEAT (legacy) path holds it across the whole
		// deinit/init rebuild (~100-200 ms) => C1 goes deaf; the Stage-1 leaf
		// publish holds it for a bounded scalar-store+memset (~sub-us). The wait
		// window == the deaf window (C1 not draining capture_buffer => RX ring is
		// not being fed). We log every wait over C1_LW_THRESH_US and print the
		// true running max + mean at thread exit. Enabled by MERCURY_C1_LOCKWAIT=1.
#ifdef FF_LINUX
		struct timespec _c1lw0, _c1lw1;
		if(c1_lw_probe)
			clock_gettime(CLOCK_MONOTONIC, &_c1lw0);
#endif

		MUTEX_LOCK(&capture_prep_mutex);

#ifdef FF_LINUX
		if(c1_lw_probe) {
			clock_gettime(CLOCK_MONOTONIC, &_c1lw1);
			double _wait_us = (_c1lw1.tv_sec - _c1lw0.tv_sec) * 1e6
				+ (_c1lw1.tv_nsec - _c1lw0.tv_nsec) / 1e3;
			const double C1_LW_THRESH_US = 200.0; // ignore normal uncontended jitter
			if(_wait_us > c1_lw_max_us) c1_lw_max_us = _wait_us;
			c1_lw_sum_us += _wait_us;
			c1_lw_samples++;
			if(_wait_us > C1_LW_THRESH_US) {
				c1_lw_n_over++;
				printf("[C1-LOCKWAIT] wait_us=%.1f wait_ms=%.3f max_us=%.1f n_over=%ld\n",
					_wait_us, _wait_us / 1000.0, c1_lw_max_us, c1_lw_n_over);
				fflush(stdout);
			}
		}
#endif

		// Re-read buffer parameters inside mutex to prevent use-after-free
		// during config switches that deinit/reinit passband_delayed_data.
		// The values read outside the mutex (signal_period) may be stale if
		// deinit zeroed Nofdm/buffer_Nsymb between the read and the lock.
		{
			int sp = data_container_ptr->Nofdm * data_container_ptr->buffer_Nsymb * data_container_ptr->interpolation_rate;
			if(sp != signal_period && sp != 0) {
				printf("[CAP-STALE] sp_old=%d sp_new=%d symb_old=%d buf=%p tid=%lu\n",
					signal_period, sp, symbol_period, (void*)data_container_ptr->passband_delayed_data,
					(unsigned long)pthread_self());
				fflush(stdout);
			}
			if(sp == 0 || data_container_ptr->passband_delayed_data == NULL || sp <= symbol_period) {
				MUTEX_UNLOCK(&capture_prep_mutex);
				continue;
			}

			// Only count overrun shifts: when buffer is full (ftr==0) but
			// processing thread hasn't consumed data yet (data_ready==1).
			// During normal fill (ftr>0), shifts are expected — NOT overruns.
			// Without this gate, NB MFSK fills of 500+ symbols inflate nUnder
			// to ~500, wiping out mfsk_search_raw and causing re-decode of
			// stale preambles (RSP stuck in FAIL decode loop).
			if(data_container_ptr->data_ready == 1 && data_container_ptr->frames_to_read <= 0) {
				data_container_ptr->nUnder_processing_events++;
				data_container_ptr->nUnder_processing_events_total++;
			}

			// Double-mapped ring buffer write: write at write_index AND
			// write_index+sp (mirror). Reading sp samples from any position
			// in [0,sp) gives a contiguous chronological view via the mirror.
			{
				int wi = data_container_ptr->ring_write_index;
				int remaining = sp - wi;
				if(remaining >= symbol_period) {
					// Common case: no wrap
					memcpy(&data_container_ptr->passband_delayed_data[wi],
						buffer_temp, symbol_period * sizeof(double));
					memcpy(&data_container_ptr->passband_delayed_data[wi + sp],
						buffer_temp, symbol_period * sizeof(double));
				} else {
					// Rare: write spans ring boundary
					memcpy(&data_container_ptr->passband_delayed_data[wi],
						buffer_temp, remaining * sizeof(double));
					memcpy(&data_container_ptr->passband_delayed_data[wi + sp],
						buffer_temp, remaining * sizeof(double));
					int wrap = symbol_period - remaining;
					memcpy(&data_container_ptr->passband_delayed_data[0],
						&buffer_temp[remaining], wrap * sizeof(double));
					memcpy(&data_container_ptr->passband_delayed_data[sp],
						&buffer_temp[remaining], wrap * sizeof(double));
				}
				data_container_ptr->ring_write_index =
					(wi + symbol_period) % sp;

				capture_publish_causal_ring_chunk(
					data_container_ptr, causal_tags.data(),
					symbol_period, sp);
			}

			data_container_ptr->frames_to_read--;
			if(data_container_ptr->frames_to_read < 0)
				data_container_ptr->frames_to_read = 0;

			data_container_ptr->data_ready = 1;
		}
		MUTEX_UNLOCK(&capture_prep_mutex);
	}


	printf("radio_capture_prep_thread exit\n");
	if(c1_lw_probe) {
		double _mean = c1_lw_samples ? (c1_lw_sum_us / (double)c1_lw_samples) : 0.0;
		printf("[C1-LOCKWAIT-SUMMARY] samples=%ld max_us=%.2f max_ms=%.4f mean_us=%.3f n_over_200us=%ld\n",
			c1_lw_samples, c1_lw_max_us, c1_lw_max_us / 1000.0, _mean, c1_lw_n_over);
		fflush(stdout);
	}
	if(cbc_meter) {
		long ss = telecom_ptr->data_container.cbc_muted_signal_samples;
		long se = telecom_ptr->data_container.cbc_muted_signal_events;
		long ne = telecom_ptr->data_container.cbc_muted_noise_events;
		long ts = telecom_ptr->data_container.cbc_muted_total_samples;
		double pk = telecom_ptr->data_container.cbc_muted_peak;
		printf("[CBC-METER-SUMMARY] signal_eaten_samples=%ld signal_eaten_events=%ld noise_muted_events=%ld total_muted_samples=%ld peak=%.6f\n",
			ss, se, ne, ts, pk);
		fflush(stdout);
	}
	shutdown_ = true;

	free(buffer_temp);

    return NULL;
}


void list_soundcards(int audio_system)
{
    ffaudio_interface *audio;
    audio_subsystem = audio_system;

#if defined(_WIN32)
    if (audio_subsystem == AUDIO_SUBSYSTEM_WASAPI)
        audio = (ffaudio_interface *) &ffwasapi;
    if (audio_subsystem == AUDIO_SUBSYSTEM_DSOUND)
        audio = (ffaudio_interface *) &ffdsound;
#elif defined(__linux__)
    if (audio_subsystem == AUDIO_SUBSYSTEM_ALSA)
        audio = (ffaudio_interface *) &ffalsa;
    if (audio_subsystem == AUDIO_SUBSYSTEM_PULSE)
        audio = (ffaudio_interface *) &ffpulse;
#elif defined(__FREEBSD__)
    if (audio_subsystem == AUDIO_SUBSYSTEM_OSS)
        audio = (ffaudio_interface *) &ffoss;
#elif defined(__APPLE__)
    if (audio_subsystem == AUDIO_SUBSYSTEM_COREAUDIO)
        audio = (ffaudio_interface *) &ffcoreaudio;
#elif defined(__ANDROID__)
    if (audio_subsystem == AUDIO_SUBSYSTEM_AAUDIO)
        audio = (ffaudio_interface *) &ffaaudio;
#endif

	ffaudio_init_conf aconf = {};
	if ( audio->init(&aconf) != 0)
    {
        printf("Error in audio->init()\n");
        return;
    }

	ffaudio_dev *d;

	// FFAUDIO_DEV_PLAYBACK, FFAUDIO_DEV_CAPTURE
	static const char* const mode[] = { "playback", "capture" };
	for (ffuint i = 0;  i != 2;  i++)
    {
		printf("%s devices:\n", mode[i]);
		d = audio->dev_alloc(i);
        if (d == NULL)
        {
            printf("Error in audio->dev_alloc\n");
            return;
        }

		for (;;)
        {
			int r = audio->dev_next(d);
			if (r > 0)
				break;
			else
                if (r < 0)
                {
                    printf("error: %s", audio->dev_error(d));
                    break;
                }

			printf("device: name: '%s'  id: '%s'  default: %s\n"
				, audio->dev_info(d, FFAUDIO_DEV_NAME)
				, audio->dev_info(d, FFAUDIO_DEV_ID)
				, audio->dev_info(d, FFAUDIO_DEV_IS_DEFAULT)
				);
		}

		audio->dev_free(d);
	}
}

// ===========================================================================
// SIM channel backend (-x sim): device-free software channel.
//
// Replaces radio_capture_thread + radio_playback_thread with two socket
// bridges to an external channel relay (tools/sim/sim_channel_relay.py):
//   * sim_tx_bridge_thread:  playback_buffer (this peer's TX passband) ->
//                            TCP send to relay.
//   * sim_rx_bridge_thread:  TCP recv from relay -> capture_buffer (this
//                            peer's RX passband, AWGN + loss already applied
//                            by the relay).
// radio_capture_prep_thread is reused unchanged (it consumes capture_buffer).
//
// Wire format: little-endian float64 (double) mono passband, 48 kHz, the SAME
// units tx_transfer/rx_transfer already move. The relay sums both peers'
// TX streams, applies a fixed-SNR AWGN floor (+ optional bursty sample
// dropout / Watterson fading) and fans the result back to both RX sockets.
//
// Connection: each peer connects to 127.0.0.1:<port> and immediately sends a
// 1-byte role tag ('A' commander / 'B' responder) so the relay can cross-wire
// the two directions. Port + role come from env:
//     MERCURY_SIM_PORT (default 52100)
//     MERCURY_SIM_ROLE ('A' or 'B'; default 'A')
// One TCP connection carries BOTH directions (TX out, RX in) for that peer.
// ===========================================================================

#if defined(_WIN32)
typedef SOCKET sim_sock_t;
#define SIM_BAD_SOCK INVALID_SOCKET
#else
typedef int sim_sock_t;
#define SIM_BAD_SOCK (-1)
#endif

static sim_sock_t sim_sock = SIM_BAD_SOCK;   // shared by both bridge threads
static int sim_connected = 0;

// GUARD 1 marker + g_sim_audio_guard_active are declared at file scope near the
// top of this TU (before the device threads that reference them).

// SIM transport chunk: number of double samples per relay packet. 1024 doubles
// = ~21 ms at 48 kHz, small enough that channel impairment granularity matches
// a fraction of an OFDM symbol (Nofdm*interp ~= 3072 samples WB).
#define SIM_CHUNK_SAMPLES 1024

static int sim_send_all(sim_sock_t s, const uint8_t *buf, int len)
{
	int sent = 0;
	while (sent < len) {
		int n = send(s, (const char *)(buf + sent), len - sent, 0);
		if (n <= 0) return -1;
		sent += n;
	}
	return 0;
}

static int sim_recv_all(sim_sock_t s, uint8_t *buf, int len)
{
	int got = 0;
	while (got < len) {
		int n = recv(s, (char *)(buf + got), len - got, 0);
		if (n <= 0) return -1;
		got += n;
	}
	return 0;
}

// Establish the single shared TCP connection to the relay (idempotent).
static int sim_connect_once(void)
{
	if (sim_connected) return 0;

#if defined(_WIN32)
	WSADATA wsa;
	WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

	const char *port_s = getenv("MERCURY_SIM_PORT");
	const char *role_s = getenv("MERCURY_SIM_ROLE");
	int port = port_s ? atoi(port_s) : 52100;
	char role = (role_s && role_s[0]) ? role_s[0] : 'A';

	sim_sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sim_sock == SIM_BAD_SOCK) {
		printf("[SIM] socket() failed\n");
		return -1;
	}
	int one = 1;
	setsockopt(sim_sock, IPPROTO_TCP, TCP_NODELAY, (const char *)&one, sizeof(one));

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons((unsigned short)port);
	addr.sin_addr.s_addr = inet_addr("127.0.0.1");

	// Retry: the relay or the peer may start a moment after us.
	int attempts = 0;
	while (connect(sim_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
		if (shutdown_) return -1;
		if (++attempts > 200) {   // ~20 s
			printf("[SIM] connect to 127.0.0.1:%d failed after %d attempts\n", port, attempts);
			return -1;
		}
		ffthread_sleep(100);
	}
	// Send role tag so the relay knows which direction we are.
	if (sim_send_all(sim_sock, (const uint8_t *)&role, 1) != 0) {
		printf("[SIM] role handshake send failed\n");
		return -1;
	}
	sim_connected = 1;
	printf("[SIM] connected to channel relay 127.0.0.1:%d as role '%c'\n", port, role);
	fflush(stdout);
	return 0;
}

// TX bridge: pull this peer's TX passband from playback_buffer and ship it
// to the relay in fixed-size chunks. When idle, send silence so the relay's
// per-direction sample clock keeps advancing (the channel must add noise even
// during TX gaps, exactly like RF — a silent peer still hears the channel).
//
// CAPTURE-DETERMINISM (data-flow-sim-tx-turnaround-phase.md, sim-arq-channel.md
// §15/§16): the IDLE silence-emission RATE used to be wall-clock-paced
// (sim_paced_wait: Sleep(1)=1 ms on Windows vs nanosleep(200 us) on POSIX), so
// during a half-duplex turnaround POSIX minted ~5x more silence chunks per unit
// of wall time than Windows. The relay forwards those chunks 1:1 and stamps a
// per-direction END-sample index, so the OS-dependent silence COUNT lands the
// reverse HAIL/control burst at an OS-dependent ABSOLUTE sample position in the
// receiver's capture window -> the fixed-geometry frame extraction / correlator
// captures a misaligned window on one OS but not the other (Linux 0/8 vs
// Windows 8/8 connect, OS-deterministic). The free 48 kHz capture DMA on real
// radios has no such artifact: the turnaround is a fixed PHYSICAL number of
// channel samples on any host. To restore that invariant in the 2-process sim
// WITHOUT touching the decode/correlator numerics, PACE THE IDLE SILENCE
// EMISSION TO THE SHARED VIRTUAL CLOCK: ship a silence chunk only when this
// peer's own TX sample count is BEHIND virtual time, so the number of silence
// chunks minted in a turnaround is a deterministic function of VIRTUAL elapsed
// samples — identical on every OS because under --wire-stamp BOTH peers read the
// SAME relay timeline (sim_clock_set_samples on RX arrival). Real signal is
// NEVER throttled (it always advances the wire + the other peer's clock), so
// the throttle cannot deadlock: whenever a peer is "ahead" of virtual time it
// has already put chunks on the wire that advance the shared clock, which then
// releases the throttle. Gated on sim_clock_wire_stamp() (the 2-process
// phase-lock path the harness drives); every other path keeps the legacy
// emit-one-per-wall-tick behavior byte-identically (HW never enters this thread
// at all — the SIM branch is the only producer of -x sim TX).
void *sim_tx_bridge_thread(void *unused)
{
	(void)unused;
	if (sim_connect_once() != 0) { shutdown_ = true; return NULL; }

	const int chunk_bytes = SIM_CHUNK_SAMPLES * (int)sizeof(double);
	double *chunk = (double *)malloc(chunk_bytes);

	// Clock-paced idle-silence throttle: engaged only on the wire-stamped
	// 2-process phase-lock path. tx_samples = this peer's TX direction sample
	// count (chunks shipped * SIM_CHUNK_SAMPLES).
	const int clock_paced_idle = sim_clock_wire_stamp();
	uint64_t  tx_samples = 0;

	while (!shutdown_) {
		size_t avail = size_buffer(playback_buffer);
		if (avail >= (size_t)chunk_bytes) {
			// Full chunk available: forward verbatim.
			read_buffer(playback_buffer, (uint8_t *)chunk, chunk_bytes);
		} else if (avail > 0) {
			// PARTIAL remainder. The modem busy-waits on
			// `size_buffer(playback_buffer) > 0` to know a frame's TX has
			// finished (arq_common.cc:4072 etc). If we only ever drain full
			// chunks, a sub-chunk tail (a HAIL/ACK pattern is not a multiple
			// of 1024 samples) is stranded forever -> the modem deadlocks in
			// the drain-wait and never switches to RX. So drain the partial
			// NOW and zero-pad it up to a full chunk before sending (the pad
			// is just inter-frame silence on the wire — harmless).
			memset(chunk, 0, chunk_bytes);
			read_buffer(playback_buffer, (uint8_t *)chunk, (int)avail);
		} else {
			// No TX queued: emit a silence chunk so the relay clock advances
			// and the RX side still receives a noise floor (RF realism).
			if (clock_paced_idle) {
				// Pace silence to the SHARED virtual clock so the per-turnaround
				// silence COUNT is OS-invariant. Only mint silence when this
				// peer's TX sample count is BEHIND virtual time; otherwise wait
				// and re-poll WITHOUT sending (do NOT advance tx_samples). The
				// clock advances on RX arrival (the other peer's chunks, relay-
				// driven), so this peer always catches up and never spins
				// unbounded. Bootstrap: before any chunk flows the shared clock
				// is 0; vnow==0 < tx_samples is false only at the very first
				// iteration (tx_samples==0), so the first silence chunk is sent
				// immediately to kick the handshake, after which both peers track
				// the same relay timeline.
				uint64_t vnow = sim_clock_now_samples();
				if (tx_samples > vnow) {
					// Ahead of channel time — yield the core and re-poll. No send,
					// no counter advance. The 5 ms arg is a wall-pace floor only;
					// the DECISION (whether to send) is on virtual time, so the
					// emitted COUNT is wall-rate-independent and OS-invariant.
					sim_paced_wait(5);
					continue;
				}
				memset(chunk, 0, chunk_bytes);
			} else {
				// Legacy (non-wire-stamp / bootstrap) path: one silence chunk per
				// wall tick — byte-identical to the pre-fix behavior.
				memset(chunk, 0, chunk_bytes);
				sim_paced_wait(5);
			}
		}
		if (sim_send_all(sim_sock, (const uint8_t *)chunk, chunk_bytes) != 0) {
			printf("[SIM] TX bridge send failed (relay closed?)\n");
			break;
		}
		// Count every chunk actually shipped (signal, partial, or paced silence)
		// so the idle throttle measures THIS peer's TX-direction sample position
		// against the shared virtual clock.
		tx_samples += SIM_CHUNK_SAMPLES;
	}
	free(chunk);
	return NULL;
}

// RX bridge: pull channel-impaired passband from the relay and push it into
// capture_buffer, where radio_capture_prep_thread + rx_transfer consume it.
void *sim_rx_bridge_thread(void *unused)
{
	(void)unused;
	// TX bridge owns the connect; wait for it.
	int waited = 0;
	while (!sim_connected && !shutdown_) {
		ffthread_sleep(20);
		if (++waited > 1500) { shutdown_ = true; return NULL; }  // ~30 s
	}
	if (shutdown_) return NULL;

	const int chunk_bytes = SIM_CHUNK_SAMPLES * (int)sizeof(double);
	double  *chunk = (double *)malloc(chunk_bytes);
	uint8_t  stamp_buf[8];
	// --wire-stamp mode (sim-arq-channel.md §10.5b/§11.2): the relay prepends an
	// 8-byte LE per-direction END-sample stamp ahead of each chunk and BOTH peers
	// adopt that ONE relay timeline (drift-immune phase-lock). Latched ONCE here
	// (set at startup before this thread exists) so the wire format is fixed for
	// the whole run — a mid-run flip would desync the 8-byte framing.
	const int wire_stamp = sim_clock_wire_stamp();
	int       first_chunk = 1;

	while (!shutdown_) {
		uint64_t stamp = 0;
		if (wire_stamp) {
			// Read the 8-byte stamp FIRST, then the CHUNK_BYTES payload. Both
			// ends change in one commit so the framing cannot drift. A non-stamp
			// modem (g_sim_wire_stamp==0) against a --wire-stamp relay would
			// silently mis-frame — the harness GUARD + first-vstamp canary below
			// catch a stale-binary mismatch.
			if (sim_recv_all(sim_sock, stamp_buf, 8) != 0) {
				printf("[SIM] RX bridge stamp recv failed (relay closed?)\n");
				break;
			}
		}
		if (sim_recv_all(sim_sock, (uint8_t *)chunk, chunk_bytes) != 0) {
			printf("[SIM] RX bridge recv failed (relay closed?)\n");
			break;
		}
		if (wire_stamp) {
			stamp = (uint64_t)stamp_buf[0]        | ((uint64_t)stamp_buf[1] << 8)
			      | ((uint64_t)stamp_buf[2] << 16) | ((uint64_t)stamp_buf[3] << 24)
			      | ((uint64_t)stamp_buf[4] << 32) | ((uint64_t)stamp_buf[5] << 40)
			      | ((uint64_t)stamp_buf[6] << 48) | ((uint64_t)stamp_buf[7] << 56);
			if (first_chunk) {
				// One-time canary: a stale relay/binary wire mismatch shows up as
				// a wild first stamp. The harness greps for this line to confirm
				// the modem actually parsed a stamp under --wire-stamp 1.
				printf("[SIM] RX bridge first vstamp=%llu\n",
				       (unsigned long long)stamp);
				fflush(stdout);
				first_chunk = 0;
			}
			// Adopt the shared channel clock the instant the chunk ARRIVES (not
			// when the prep thread later demods it via rx_transfer). This couples
			// THIS peer's virtual time to relay-chunk arrival in THIS direction,
			// so the commander's ACK-timeout window and the responder's reply
			// share one timeline regardless of host (wall) speed. CAS-max in
			// sim_clock_set_samples keeps it monotonic — a silence flood cannot
			// multiply virtual time because we SET to the relay's count, and a
			// reordered chunk cannot rewind it.
			if (sim_clock_enabled())
				sim_clock_set_samples(stamp);
		}
		// Backpressure: if the prep thread is behind, spin briefly rather
		// than overflow capture_buffer (mirrors the device-full guard).
		// In sim mode the short-sleep pace (sim_paced_wait) DON'T use the
		// wall-clock spin cap — virtual time only advances when the prep thread
		// consumes via rx_transfer, so a full capture_buffer is guaranteed to
		// drain as soon as we hand the core to the prep thread; the 5000-spin
		// "~10 s" cap is sized for the production 2 ms sleep and would trip far
		// too early under the sub-ms sim pace and drop a chunk.
		int spins = 0;
		while (!shutdown_ &&
		       circular_buf_free_size(capture_buffer) < (size_t)chunk_bytes) {
			sim_paced_wait(2);
			if (!sim_clock_enabled() && ++spins > 5000) break;  // ~10 s safety
		}
		if (shutdown_) break;
		capture_write_samples(chunk, SIM_CHUNK_SAMPLES);
	}
	free(chunk);
	return NULL;
}

// size in "double" samples
int tx_transfer(double *buffer, size_t len)
{
	uint8_t *buffer_internal = (uint8_t *) buffer;
	int buffer_size_bytes = len * sizeof(double);

#if ENABLE_FLOAT64_TAP_BEFORE == 1
	fwrite(buffer_internal, 1, buffer_size_bytes, tap_play);
#endif

	write_buffer(playback_buffer, buffer_internal, buffer_size_bytes);

	// printf("size %llu free %llu\n", size_buffer(playback_buffer), circular_buf_free_size(playback_buffer));

    return 0;
}

// size in "double" samples
int rx_transfer_with_causal_tags(double *buffer, uint32_t *tags, size_t len)
{
	uint8_t *buffer_internal = (uint8_t *) buffer;
	int buffer_size_bytes = len * sizeof(double);

	if(capture_causal_tag_buffer != NULL)
	{
		static thread_local std::vector<uint32_t> discard_tags;
		if(tags == NULL)
		{
			discard_tags.resize(len);
			tags = discard_tags.data();
		}
		const size_t tag_bytes = len * sizeof(uint32_t);
		std::lock_guard<std::mutex> pair_guard(capture_pair_mutex);
		if(size_buffer(capture_buffer) < (size_t)buffer_size_bytes
		   || size_buffer(capture_causal_tag_buffer) < tag_bytes)
			return -1;
		if(read_buffer(capture_causal_tag_buffer,
				(uint8_t *)tags, tag_bytes) != 0)
			return -1;
		if(read_buffer(capture_buffer,
				buffer_internal, buffer_size_bytes) != 0)
			return -1;
	}
	else
	{
		read_buffer(capture_buffer, buffer_internal, buffer_size_bytes);
		if(tags != NULL)
			memset(tags, 0, len * sizeof(uint32_t));
	}

	// SIM virtual clock: every double the modem pulls off the RX boundary is
	// one sample of channel time. Advancing here makes virtual time track the
	// modem's demod cadence — used on the SIM_INPROC pump path and the
	// 2-process -x sim path WITHOUT --wire-stamp.
	//
	// UNDER --wire-stamp the clock is instead SET by sim_rx_bridge_thread on
	// chunk ARRIVAL from the relay's authoritative stamp. Advancing HERE too
	// would DOUBLE-count virtual time (once on arrival, once on demod) and
	// re-introduce the idle-silence warp — so the add is suppressed in that mode
	// (sim_clock_wire_stamp()==1). SIM_INPROC NEVER sets g_sim_wire_stamp, so its
	// pump-driven rx_transfer keeps the ADD clock untouched (the 12 SIM_INPROC /
	// --test add-producers are unaffected — §5 cross-layer audit).
	//
	// No-op (two relaxed loads) when sim is disabled, so production -x
	// wasapi/alsa rx_transfer is byte-identical. See include/common/sim_clock.h.
	if (sim_clock_enabled() && !sim_clock_wire_stamp())
		sim_clock_add_samples((uint64_t) len);

    return 0;
}

int rx_transfer(double *buffer, size_t len)
{
	return rx_transfer_with_causal_tags(buffer, NULL, len);
}


int audioio_init_internal(char *capture_dev, char *playback_dev, int audio_subsys, pthread_t *radio_capture,
						  pthread_t *radio_playback, pthread_t *radio_capture_prep, cl_telecom_system *telecom_system)
{
    audio_subsystem = audio_subsys;
	capture_thread_started = false;
	playback_thread_started = false;
	capture_prep_thread_started = false;

#if ENABLE_FLOAT64_TAP_BEFORE == 1
	tap_play = fopen("tap-playback-b.f64", "w");
#endif

#if defined(_WIN32)
	uint8_t *buffer_cap = (uint8_t *)malloc(AUDIO_PAYLOAD_BUFFER_SIZE);
	uint8_t *buffer_play = (uint8_t *)malloc(AUDIO_PAYLOAD_BUFFER_SIZE);
    capture_buffer = circular_buf_init(buffer_cap, AUDIO_PAYLOAD_BUFFER_SIZE);
    playback_buffer = circular_buf_init(buffer_play, AUDIO_PAYLOAD_BUFFER_SIZE);
#else
    capture_buffer = circular_buf_init_shm(AUDIO_PAYLOAD_BUFFER_SIZE, (char *) AUDIO_CAPT_PAYLOAD_NAME);
    playback_buffer = circular_buf_init_shm(AUDIO_PAYLOAD_BUFFER_SIZE, (char *) AUDIO_PLAY_PAYLOAD_NAME);
#endif

	const size_t capture_tag_capacity =
		(AUDIO_PAYLOAD_BUFFER_SIZE / sizeof(double)) * sizeof(uint32_t);
	capture_causal_tag_storage =
		(uint8_t *)malloc(capture_tag_capacity);
	if(capture_causal_tag_storage != NULL)
		capture_causal_tag_buffer = circular_buf_init(
			capture_causal_tag_storage, capture_tag_capacity);
	capture_telecom_system = telecom_system;
	telecom_system->data_container.start_ack_causal_tracking_available =
		capture_causal_tag_buffer != NULL ? 1 : 0;
	capture_reset_samples();
	clear_buffer(playback_buffer);
	capture_source_queue_guard_ns.store(100ULL * 1000000ULL);
	playback_sink_queue_guard_ns.store(100ULL * 1000000ULL);
	playback_sink_sample_rate_hz.store(48000U);

#if defined(_WIN32)
    capture_prep_mutex = CreateMutex(NULL, FALSE, NULL);
#endif

    if (audio_subsys == AUDIO_SUBSYSTEM_SIM) {
        // Device-free software channel: TX/RX bridge threads instead of
        // WASAPI/ALSA device threads. radio_capture / radio_playback handles
        // are reused to carry the bridge threads so audioio_deinit joins them.
        // GUARD 1: latch the guard flag and print the compiled-in marker so the
        // device threads abort-before-render if ever entered, and the harness's
        // GUARD 2 can confirm this binary is sim-safe by grepping the marker.
        g_sim_audio_guard_active = 1;
        capture_source_queue_guard_ns.store(0);
        playback_sink_queue_guard_ns.store(0);
        printf("[SIM] software channel backend active (no audio device) %s\n",
               SIM_AUDIO_GUARD_MARKER);
        fflush(stdout);
        playback_thread_started = pthread_create(radio_playback, NULL, sim_tx_bridge_thread, NULL) == 0;
        capture_thread_started = pthread_create(radio_capture, NULL, sim_rx_bridge_thread, NULL) == 0;
        capture_prep_thread_started = pthread_create(radio_capture_prep, NULL, radio_capture_prep_thread,
                                                     (void *) telecom_system) == 0;
        if(!playback_thread_started || !capture_thread_started || !capture_prep_thread_started)
        {
            fprintf(stderr, "ERROR: could not start all audio threads\n");
            shutdown_ = true;
            return -1;
        }
        return 0;
    }

    {
        std::lock_guard<std::mutex> lock(audio_startup_mutex);
        capture_startup_status = AUDIO_START_PENDING;
        playback_startup_status = AUDIO_START_PENDING;
    }

    capture_thread_started = pthread_create(radio_capture, NULL, radio_capture_thread,
                                            (void *) capture_dev) == 0;
	playback_thread_started = pthread_create(radio_playback, NULL, radio_playback_thread,
                                           (void *) playback_dev) == 0;
	capture_prep_thread_started = pthread_create(radio_capture_prep, NULL, radio_capture_prep_thread,
                                               (void *) telecom_system) == 0;
	if(!capture_thread_started)
		publish_audio_startup(true, AUDIO_START_FAILED);
	if(!playback_thread_started)
		publish_audio_startup(false, AUDIO_START_FAILED);
	if(!capture_prep_thread_started)
	{
		fprintf(stderr, "ERROR: could not start audio preparation thread\n");
		shutdown_ = true;
		return -1;
	}

	std::unique_lock<std::mutex> lock(audio_startup_mutex);
	audio_startup_cv.wait(lock, [] {
		return capture_startup_status == AUDIO_START_FAILED
		    || playback_startup_status == AUDIO_START_FAILED
		    || (capture_startup_status == AUDIO_START_READY
		        && playback_startup_status == AUDIO_START_READY);
	});
	const bool ready = capture_startup_status == AUDIO_START_READY
	                && playback_startup_status == AUDIO_START_READY;
	lock.unlock();
	if(!ready)
		shutdown_ = true;
	return ready ? 0 : -1;
}

int audioio_deinit(pthread_t *radio_capture, pthread_t *radio_playback, pthread_t *radio_capture_prep)
{
    // Guard: if audio was never initialized (e.g. BER test modes), skip everything
	if(!capture_buffer)
	{
		return 0;
	}

	if(capture_prep_thread_started)
	{
		pthread_join(*radio_capture_prep, NULL);
	}
	if(capture_thread_started)
	{
		pthread_join(*radio_capture, NULL);
	}
	if(playback_thread_started)
	{
		pthread_join(*radio_playback, NULL);
	}
	capture_prep_thread_started = false;
	capture_thread_started = false;
	playback_thread_started = false;

#if ENABLE_FLOAT64_TAP_BEFORE == 1
	fclose(tap_play);
#endif

	if(capture_telecom_system != NULL)
		capture_telecom_system->data_container
			.start_ack_causal_tracking_available = 0;
	if(capture_causal_tag_buffer != NULL)
	{
		free(capture_causal_tag_buffer->buffer);
		circular_buf_free(capture_causal_tag_buffer);
		capture_causal_tag_buffer = NULL;
		capture_causal_tag_storage = NULL;
	}
	capture_telecom_system = NULL;

#if defined(_WIN32)
	CloseHandle(capture_prep_mutex);
	capture_prep_mutex = NULL;
	free(capture_buffer->buffer);
	circular_buf_free(capture_buffer);
	free(playback_buffer->buffer);
	circular_buf_free(playback_buffer);
#else
    circular_buf_destroy_shm(capture_buffer, AUDIO_PAYLOAD_BUFFER_SIZE, (char *) AUDIO_CAPT_PAYLOAD_NAME);
    circular_buf_free_shm(capture_buffer);

    circular_buf_destroy_shm(playback_buffer, AUDIO_PAYLOAD_BUFFER_SIZE, (char *) AUDIO_PLAY_PAYLOAD_NAME);
    circular_buf_free_shm(playback_buffer);
#endif
	capture_buffer = NULL;
	playback_buffer = NULL;
    return 0;
}
