#ifndef MERCURY_TEST_AUDIOIO_H
#define MERCURY_TEST_AUDIOIO_H
#include <cstdio>
#include <atomic>
#include <mutex>
#include "audioio/audioio.h"
#include <ffaudio/audio.h>
int audioio_rt_priority_case(bool (*apply_priority)(const char*, bool, FILE*));
int capture_prep_geometry_change_case(bool (*matches)(cl_data_container*, int, int*, int*));

void* audioio_test_malloc_failure(size_t);
int audioio_test_read_failure(cbuf_handle_t, uint8_t*, size_t);
struct capture_backpressure_test_context
{
	double drained[2];
	int waits;
	int failed;
};

#if !defined(_WIN32)
extern std::atomic<bool> audioio_detach_uaf_release;
extern std::atomic<int> audioio_detach_uaf_workers;
extern cbuf_handle_t audioio_detach_uaf_ring;
int audioio_detach_uaf_create(pthread_t*, const pthread_attr_t*, void* (*)(void*), void*);
#endif

using AudioioTestMalloc = void* (*)(size_t);
using AudioioTestReadBuffer = int (*)(cbuf_handle_t, uint8_t*, size_t);
#if !defined(_WIN32)
int audioio_capture_allocation_failure_case(AudioioTestMalloc& audioio_capture_malloc, std::mutex& audio_startup_mutex,
    int& capture_startup_status, int AUDIO_START_PENDING, int AUDIO_START_FAILED,
    int (*allocate_capture_internal_buffer)(double**));
#endif
int sim_tx_bridge_allocation_failure_case(int& sim_connected, int& sim_tx_allocation_fails_for_test,
    void* (*sim_tx_bridge_thread)(void*));
int sim_rx_bridge_allocation_failure_case(int& sim_connected, int& sim_rx_allocation_fails_for_test,
    void* (*sim_rx_bridge_thread)(void*));
int rx_transfer_read_failure_case(cbuf_handle_t capture_causal_tag_buffer, AudioioTestReadBuffer& audioio_capture_read_buffer);
#if !defined(_WIN32)
int audioio_detach_leak_case(bool& audioio_payload_malloc_overridden, volatile int& g_sim_audio_guard_active,
    audioio_pthread_create_fn& audioio_pthread_create, bool& audioio_pthread_join_overridden);
#endif
int audioio_payload_allocation_failure_case(AudioioTestMalloc& audioio_payload_malloc, bool& audioio_payload_malloc_overridden);
int sim_playback_connection_failure_case(int& sim_port_for_test, int& sim_connect_attempt_limit_for_test,
    void* (*sim_tx_bridge_thread)(void*), bool (*sim_connection_dirty)(),
    void (*sim_close_connection)());




#endif
