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

int tx_transfer(double *buffer, size_t len);
int rx_transfer(double *buffer, size_t len);


void list_soundcards(int audio_system);

#if defined(_WIN32)
// Validate audio device configuration (stereo, sample rate)
// Returns 0 if OK, non-zero bitmask if errors found
int validate_audio_config(const char *capture_dev, const char *playback_dev, int audio_system);
#endif
