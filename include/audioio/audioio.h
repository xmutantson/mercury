/* Audio subsystem
 *
 * Copyright (C) 2024 Rhizomatica
 * Author: Rafael Diniz <rafael@rhizomatica.org>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#pragma once

#include <ffbase/string.h>

#include <stdio.h>
#include <sys/types.h>

#include <fcntl.h>

#include "physical_layer/telecom_system.h"
#include "common/ring_buffer_posix.h"

#define AUDIO_SUBSYSTEM_ALSA 0
#define AUDIO_SUBSYSTEM_PULSE 1
#define AUDIO_SUBSYSTEM_WASAPI 2
#define AUDIO_SUBSYSTEM_DSOUND 3
#define AUDIO_SUBSYSTEM_COREAUDIO 4
#define AUDIO_SUBSYSTEM_OSS 5
#define AUDIO_SUBSYSTEM_AAUDIO 6
// Device-free software channel backend (SIM). Selected with "-x sim".
// Replaces the WASAPI/ALSA capture+playback device threads with two
// localhost TCP socket-bridge threads that connect to an external channel
// relay (tools/sim_channel_relay.py). The relay sums both peers' TX
// passband, applies AWGN at a fixed SNR (+ optional bursty sample dropout
// and Watterson fading), and fans the result back to both peers' RX. This
// drives sustained channel loss through the FULL ARQ loop with no audio
// hardware, so gearshift over-climb->collapse + deep-SNR STALL reproduce
// deterministically at CPU speed. See tools/sim_arq_channel.py and
// mercury/fact-documents/sim-arq-channel.md.
#define AUDIO_SUBSYSTEM_SIM 7

#define LEFT 0
#define RIGHT 1
#define STEREO 2

extern cbuf_handle_t capture_buffer;
extern cbuf_handle_t playback_buffer;

// Process-wide TX level shared by control/main code and the playback worker.
// Invalid values are rejected without changing the active level.
bool audioio_set_tx_level(double level);
double audioio_get_tx_level(void);

// Phase-F validation: override ALSA buffer length (Linux only). 0 = default 30ms.
extern int g_audio_buffer_ms_override;

#if defined(_WIN32)
extern HANDLE            capture_prep_mutex;
#else
extern pthread_mutex_t   capture_prep_mutex;
#endif


int audioio_init_internal(char *capture_dev, char *playback_dev, int audio_subsys, pthread_t *radio_capture,
						  pthread_t *radio_playback, pthread_t *radio_capture_prep, cl_telecom_system *telecom_system);

int audioio_deinit(pthread_t *radio_capture, pthread_t *radio_playback, pthread_t *radio_capture_prep);

// Test-only fault-injection seam for deterministic thread-start rollback tests.
// Passing NULL for either callback restores the corresponding pthread default.
typedef int (*audioio_pthread_create_fn)(pthread_t *, const pthread_attr_t *,
										 void *(*)(void *), void *);
typedef int (*audioio_pthread_join_fn)(pthread_t, void **);
void audioio_set_thread_functions_for_test(audioio_pthread_create_fn create_fn,
										 audioio_pthread_join_fn join_fn);

int tx_transfer(double *buffer, size_t len);
int rx_transfer(double *buffer, size_t len);
int rx_transfer_with_causal_tags(double *buffer, uint32_t *tags, size_t len);

// Paired capture/tag FIFO operations. Every production capture sample carries
// a START-ACK causal-generation tag through the same reset/read lifecycle.
int capture_write_samples(double *buffer, size_t len);
void capture_reset_samples(void);

// RX_TEST stream-ready wait. The capture-prep producer signals this immediately
// after publishing data_ready=1. The timeout is only a monotonic shutdown
// backstop; callers must check the result and must not decode on ERROR/TIMEOUT.
enum audioio_rx_ready_wait_result {
	AUDIOIO_RX_READY_WAIT_ERROR = -1,
	AUDIOIO_RX_READY_WAIT_TIMEOUT = 0,
	AUDIOIO_RX_READY_WAIT_READY = 1,
	AUDIOIO_RX_READY_WAIT_SHUTDOWN = 2
};
int audioio_wait_for_rx_ready(cl_data_container *data_container,
						  unsigned int timeout_ms);
void audioio_signal_rx_ready(void);

int capture_causal_tag_guard_selftest(cl_telecom_system *telecom_system);
int capture_enqueue_backpressure_selftest(void);
int rx_transfer_read_failure_selftest(void);
int audioio_capture_allocation_failure_selftest(void);
int audioio_payload_allocation_failure_selftest(void);
int sim_tx_bridge_allocation_failure_selftest(void);
int sim_rx_bridge_allocation_failure_selftest(void);
// Conservative time from queued passband samples to physical/simulated output
// egress, including the opened playback backend's retained-device bound.
uint64_t playback_causal_egress_bound_ns(size_t queued_samples);


void list_soundcards(int audio_system);
int list_soundcards_dev_alloc_failure_selftest(void);

#if defined(_WIN32)
// Validate audio device configuration (stereo, sample rate)
// Returns 0 if OK, non-zero bitmask if errors found
int validate_audio_config(const char *capture_dev, const char *playback_dev, int audio_system);
#endif
