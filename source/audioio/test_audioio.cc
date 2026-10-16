// Directed audio regression cases and device-free fixtures.
#include "test_audioio.h"
#include <cstdlib>
#include "common/sim_clock.h"
#include "datalink_layer/timer.h"
#include <cstring>
#if !defined(_WIN32)
#include <pthread.h>
#endif

extern "C" { extern std::atomic<bool> shutdown_; }

#if !defined(_WIN32)
struct rt_priority_test_args {
	const char *thread_name;
	bool audio_thread;
	FILE *witness;
	bool invoked;
    bool (*apply_priority)(const char*, bool, FILE*);
};

static void *audioio_rt_priority_test_thread(void *opaque)
{
	rt_priority_test_args *args = (rt_priority_test_args *)opaque;
	args->invoked = args->apply_priority(
		args->thread_name, args->audio_thread, args->witness);
	return NULL;
}

static int audioio_run_rt_priority_test_thread(const char *thread_name,
	                                           bool audio_thread,
	                                           FILE *witness,
	                                           bool expected_invoked,
    bool (*apply_priority)(const char*, bool, FILE*))
{
	rt_priority_test_args args = {
		thread_name, audio_thread, witness, false, apply_priority
	};
	pthread_t thread;
	if(pthread_create(&thread, NULL, audioio_rt_priority_test_thread, &args) != 0)
		return 1;
	if(pthread_join(thread, NULL) != 0)
		return 1;
	return args.invoked == expected_invoked ? 0 : 1;
}

static int audioio_count_text(const char *text, const char *needle)
{
	int count = 0;
	const size_t needle_len = strlen(needle);
	while((text = strstr(text, needle)) != NULL)
	{
		++count;
		text += needle_len;
	}
	return count;
}

int audioio_rt_priority_case(bool (*apply_priority)(const char*, bool, FILE*))
{
	const char *old_value = getenv("MERCURY_RT_PRIORITY");
	const bool had_old_value = old_value != NULL;
	char *saved_value = NULL;
	if(had_old_value)
	{
		saved_value = (char *)malloc(strlen(old_value) + 1);
		if(saved_value == NULL)
			return 1;
		strcpy(saved_value, old_value);
	}

	int failed = 0;
	FILE *off_witness = tmpfile();
	FILE *on_witness = tmpfile();
	if(off_witness == NULL || on_witness == NULL)
		failed = 1;
	else
	{
		setenv("MERCURY_RT_PRIORITY", "0", 1);
		failed += audioio_run_rt_priority_test_thread(
			"audio-capture", true, off_witness, false, apply_priority);
		failed += audioio_run_rt_priority_test_thread(
			"audio-playback", true, off_witness, false, apply_priority);
		failed += audioio_run_rt_priority_test_thread(
			"receive-decode", false, off_witness, false, apply_priority);
		fflush(off_witness);
		const long off_bytes = ftell(off_witness);
		if(off_bytes != 0)
			++failed;

		setenv("MERCURY_RT_PRIORITY", "2", 1);
		failed += audioio_run_rt_priority_test_thread(
			"audio-capture", true, on_witness, true, apply_priority);
		failed += audioio_run_rt_priority_test_thread(
			"audio-playback", true, on_witness, true, apply_priority);
		failed += audioio_run_rt_priority_test_thread(
			"receive-decode", false, on_witness, true, apply_priority);

		fflush(on_witness);
		rewind(on_witness);
		char text[4096] = {0};
		const size_t length = fread(text, 1, sizeof(text) - 1, on_witness);
		text[length] = '\0';
		if(audioio_count_text(text, "[RT-PRIORITY]") != 3
		   || strstr(text, "thread=audio-capture requested=") == NULL
		   || strstr(text, "thread=audio-playback requested=") == NULL
		   || strstr(text, "thread=receive-decode requested=") == NULL
		   || audioio_count_text(text, " -> achieved=") != 3
		   || audioio_count_text(text, "result=") != 3)
			++failed;
		fputs(text, stdout);
	}

	if(off_witness != NULL)
		fclose(off_witness);
	if(on_witness != NULL)
		fclose(on_witness);
	if(had_old_value)
		setenv("MERCURY_RT_PRIORITY", saved_value, 1);
	else
		unsetenv("MERCURY_RT_PRIORITY");
	free(saved_value);
	return failed;
}
#endif

int capture_prep_geometry_change_case(
    bool (*capture_prep_chunk_matches_geometry)(cl_data_container*, int, int*, int*))
{
	cl_data_container dc;
	dc.Nofdm = 2;
	dc.interpolation_rate = 2;
	dc.buffer_Nsymb = 3;
	const int captured_symbol_period = 4;

	// Deterministically model a config publish between FIFO dequeue and the
	// ring-write lock. Old code re-read only sp=24 and wrote 4 stale samples;
	// the guarded write must reject because the current symbol is 6 samples.
	dc.Nofdm = 3;
	dc.buffer_Nsymb = 4;
	int observed_symbol_period = 0;
	int observed_signal_period = 0;
	const bool stale_rejected = !capture_prep_chunk_matches_geometry(
		&dc, captured_symbol_period,
		&observed_symbol_period, &observed_signal_period);
	const bool current_accepted = capture_prep_chunk_matches_geometry(
		&dc, 6, &observed_symbol_period, &observed_signal_period);
	return stale_rejected && current_accepted
		&& observed_symbol_period == 6
		&& observed_signal_period == 24 ? 0 : 1;
}



#if !defined(_WIN32)
void *audioio_test_malloc_failure(size_t)
{
	return NULL;
}

#endif

int audioio_test_read_failure(cbuf_handle_t, uint8_t *, size_t)
{
	return -1;
}


#if !defined(_WIN32)
// --- detach-leak regression (use-after-free hardening) ------------------------
// Proves audioio_deinit SKIPS the payload-ring frees when the bounded shutdown
// join had to DETACH a still-wedged device thread. Such a thread is parked in a
// vendored blocking backend call; if it ever returns it writes capture_buffer,
// so freeing that ring during teardown is a use-after-free. The hardening leaks
// the rings instead (safe on an exiting process). The wedged worker below
// captures the live ring at start and writes into it AFTER teardown: safe with
// the hardening (ring still live), a heap-use-after-free without it (ASAN traps).
std::atomic<bool> audioio_detach_uaf_release{false};
std::atomic<int> audioio_detach_uaf_workers{0};
cbuf_handle_t audioio_detach_uaf_ring = NULL;

void *audioio_detach_uaf_worker(void *)
{
	// Capture the live capture ring now, while init has it allocated and valid.
	audioio_detach_uaf_ring = capture_buffer;
	audioio_detach_uaf_workers.fetch_add(1);
	// Ignore shutdown_ so the bounded teardown join times out and detaches us.
	while(!audioio_detach_uaf_release.load(std::memory_order_acquire))
	{
		struct timespec ts = { 0, 5 * 1000 * 1000 };
		nanosleep(&ts, NULL);
	}
	// Released only AFTER audioio_deinit has run. One small write into the ring we
	// captured: with the leak-on-detach hardening this lands in live memory; on a
	// tree that freed the ring it is a use-after-free.
	if(audioio_detach_uaf_ring != NULL)
	{
		uint8_t sample[64];
		memset(sample, 0, sizeof(sample));
		write_buffer(audioio_detach_uaf_ring, sample, sizeof(sample));
	}
	audioio_detach_uaf_workers.fetch_sub(1);
	return NULL;
}

int audioio_detach_uaf_create(pthread_t *thread, const pthread_attr_t *attr,
                                     void *(*)(void *), void *)
{
	return pthread_create(thread, attr, audioio_detach_uaf_worker, NULL);
}

#endif





#if !defined(_WIN32)
int audioio_capture_allocation_failure_case(AudioioTestMalloc& audioio_capture_malloc, std::mutex& audio_startup_mutex,
    int& capture_startup_status, int AUDIO_START_PENDING, int AUDIO_START_FAILED,
    int (*allocate_capture_internal_buffer)(double**))
{
	double *buffer = NULL;
	AudioioTestMalloc saved_malloc = audioio_capture_malloc;
	int saved_status;
	{
		std::lock_guard<std::mutex> lock(audio_startup_mutex);
		saved_status = capture_startup_status;
		capture_startup_status = AUDIO_START_PENDING;
	}
	audioio_capture_malloc = audioio_test_malloc_failure;
	const int result = allocate_capture_internal_buffer(&buffer);
	audioio_capture_malloc = saved_malloc;

	bool passed;
	{
		std::lock_guard<std::mutex> lock(audio_startup_mutex);
		passed = result != 0 && buffer == NULL
		      && capture_startup_status == AUDIO_START_FAILED;
		capture_startup_status = saved_status;
	}
	return passed ? 0 : 1;
}
#endif

int sim_tx_bridge_allocation_failure_case(int& sim_connected, int& sim_tx_allocation_fails_for_test,
    void* (*sim_tx_bridge_thread)(void*))
{
	const bool saved_shutdown = shutdown_.exchange(false);
	const int saved_connected = sim_connected;

	// Bypass only the relay connection so the real bridge entry point reaches
	// its allocation without network or audio hardware.
	sim_connected = 1;
	sim_tx_allocation_fails_for_test = 1;
	void *result = sim_tx_bridge_thread(NULL);
	sim_tx_allocation_fails_for_test = 0;
	sim_connected = saved_connected;

	const bool passed = result == NULL && shutdown_.load();
	shutdown_ = saved_shutdown;
	return passed ? 0 : 1;
}

int sim_rx_bridge_allocation_failure_case(int& sim_connected, int& sim_rx_allocation_fails_for_test,
    void* (*sim_rx_bridge_thread)(void*))
{
	const bool saved_shutdown = shutdown_.exchange(false);
	const int saved_connected = sim_connected;

	// Bypass the relay wait so the real bridge entry point reaches its
	// allocation without network or audio hardware.
	sim_connected = 1;
	sim_rx_allocation_fails_for_test = 1;
	void *result = sim_rx_bridge_thread(NULL);
	sim_rx_allocation_fails_for_test = 0;
	sim_connected = saved_connected;

	const bool passed = result == NULL && shutdown_.load();
	shutdown_ = saved_shutdown;
	return passed ? 0 : 1;
}

int rx_transfer_read_failure_case(cbuf_handle_t capture_causal_tag_buffer, AudioioTestReadBuffer& audioio_capture_read_buffer)
{
	if(capture_buffer != NULL || capture_causal_tag_buffer != NULL)
		return 1;

	double storage = 0.0;
	capture_buffer = circular_buf_init(
		(uint8_t *)&storage, sizeof(storage));
	AudioioTestReadBuffer saved_read = audioio_capture_read_buffer;
	const int saved_sim_clock_enabled = sim_clock_enabled();
	const int saved_wire_stamp = sim_clock_wire_stamp();
	sim_clock_set_enabled(1);
	sim_clock_set_wire_stamp(0);
	const uint64_t samples_before = sim_clock_now_samples();
	audioio_capture_read_buffer = audioio_test_read_failure;
	const int result = rx_transfer(&storage, 1);
	const uint64_t samples_after = sim_clock_now_samples();
	audioio_capture_read_buffer = saved_read;
	sim_clock_set_wire_stamp(saved_wire_stamp);
	sim_clock_set_enabled(saved_sim_clock_enabled);
	circular_buf_free(capture_buffer);
	capture_buffer = NULL;
	return result == -1 && samples_after == samples_before ? 0 : 1;
}


#if !defined(_WIN32)
int audioio_detach_leak_case(bool& audioio_payload_malloc_overridden, volatile int& g_sim_audio_guard_active,
    audioio_pthread_create_fn& audioio_pthread_create, bool& audioio_pthread_join_overridden)
{
	if(capture_buffer != NULL || playback_buffer != NULL)
		return 1;  // audio active / dirty state: do not run

	// Heap-backed rings: device-free, and a stray free becomes a clean
	// heap-use-after-free (ASAN-visible) instead of an munmap SIGSEGV.
	const bool saved_override = audioio_payload_malloc_overridden;
	audioio_payload_malloc_overridden = true;
	const int saved_guard = g_sim_audio_guard_active;

	char saved_grace[32] = {0};
	const char *g = getenv("MERCURY_AUDIO_SHUTDOWN_GRACE_MS");
	const bool had_grace = g != NULL;
	if(had_grace)
		strncpy(saved_grace, g, sizeof(saved_grace) - 1);
	setenv("MERCURY_AUDIO_SHUTDOWN_GRACE_MS", "300", 1);

	const bool saved_shutdown = shutdown_.exchange(false);
	audioio_detach_uaf_release.store(false);
	audioio_detach_uaf_workers.store(0);
	audioio_detach_uaf_ring = NULL;

	// Start three wedged device threads via the SIM backend; the join is NOT
	// overridden, so audioio_deinit's bounded join runs and detaches them.
	audioio_pthread_create_fn saved_create = audioio_pthread_create;
	audioio_pthread_create = audioio_detach_uaf_create;
	audioio_pthread_join_overridden = false;

	pthread_t cap, play, prep;
	cl_telecom_system telecom;
	const int rc = audioio_init_internal(NULL, NULL, AUDIO_SUBSYSTEM_SIM,
	                                     &cap, &play, &prep, &telecom);
	audioio_pthread_create = saved_create;
	const bool init_ok = (rc == 0 && capture_buffer != NULL);

	// Real teardown path: the bounded join detaches the wedged workers; the
	// hardening must SKIP the ring frees, leaving capture_buffer valid + non-NULL.
	audioio_deinit(&cap, &play, &prep);
	const bool leaked_valid = (capture_buffer != NULL);

	// Let the detached workers write their captured ring, then exit.
	audioio_detach_uaf_release.store(true);
	for(int i = 0; i < 500 && audioio_detach_uaf_workers.load() != 0; ++i)
	{
		struct timespec ts = { 0, 10 * 1000 * 1000 };
		nanosleep(&ts, NULL);
	}
	const bool reaped = (audioio_detach_uaf_workers.load() == 0);

	// Clean up the intentionally-leaked rings now that no thread touches them, so
	// the rest of the suite sees pristine audio state.
	bool cleaned = true;
	if(reaped && capture_buffer != NULL)
	{
		audioio_deinit(&cap, &play, &prep);
		cleaned = (capture_buffer == NULL && playback_buffer == NULL);
	}
	else if(!reaped)
	{
		cleaned = false;  // never free a ring a live worker may still touch
	}

	if(had_grace)
		setenv("MERCURY_AUDIO_SHUTDOWN_GRACE_MS", saved_grace, 1);
	else
		unsetenv("MERCURY_AUDIO_SHUTDOWN_GRACE_MS");
	shutdown_ = saved_shutdown;
	audioio_payload_malloc_overridden = saved_override;
	g_sim_audio_guard_active = saved_guard;

	const bool passed = init_ok && leaked_valid && reaped && cleaned;
	printf("[TEST-DETACH-LEAK] init_ok=%d detach_skipped_free=%d reaped=%d "
	       "cleaned=%d: %s\n",
	       init_ok ? 1 : 0, leaked_valid ? 1 : 0, reaped ? 1 : 0, cleaned ? 1 : 0,
	       passed ? "PASS" : "FAIL");
	return passed ? 0 : 1;
}
#endif

int audioio_payload_allocation_failure_case(AudioioTestMalloc& audioio_payload_malloc, bool& audioio_payload_malloc_overridden)
{
	if(capture_buffer != NULL || playback_buffer != NULL)
		return 1;

	pthread_t capture_thread, playback_thread, prep_thread;
	cl_telecom_system telecom_system;
	AudioioTestMalloc saved_malloc = audioio_payload_malloc;
	const bool saved_override = audioio_payload_malloc_overridden;
	audioio_payload_malloc = audioio_test_malloc_failure;
	audioio_payload_malloc_overridden = true;
	const int result = audioio_init_internal(
		NULL, NULL, AUDIO_SUBSYSTEM_SIM, &capture_thread, &playback_thread,
		&prep_thread, &telecom_system);
	audioio_payload_malloc = saved_malloc;
	audioio_payload_malloc_overridden = saved_override;

	return result != 0 && capture_buffer == NULL && playback_buffer == NULL
		? 0 : 1;
}

int sim_playback_connection_failure_case(int& sim_port_for_test, int& sim_connect_attempt_limit_for_test,
    void* (*sim_tx_bridge_thread)(void*), bool (*sim_connection_dirty)(),
    void (*sim_close_connection)())
{
	if (sim_connection_dirty())
		return 1;

	const bool saved_shutdown = shutdown_.exchange(false);
	const int saved_clock_enabled = sim_clock_enabled();
	const int saved_wire_stamp = sim_clock_wire_stamp();

	const int saved_port = sim_port_for_test;
	const int saved_limit = sim_connect_attempt_limit_for_test;
	bool passed = true;
	// Exercise both virtual wire formats and the disabled-clock control.
	for (int mode = 0; mode < 3; ++mode) {
		const int enabled = mode != 0;
		const int stamped = mode == 2;
		shutdown_ = false;
		// Port zero cannot have a listening relay; no retry or sleep is needed.
		sim_port_for_test = 0;
		sim_connect_attempt_limit_for_test = 0;
		sim_clock_set_enabled(enabled);
		sim_clock_set_wire_stamp(stamped);
		const uint64_t samples_before = sim_clock_now_samples();
		cl_timer timer;
		timer.start();
		void *result = sim_tx_bridge_thread(NULL);
		const bool stable = sim_clock_enabled() == enabled
			&& sim_clock_wire_stamp() == stamped
			&& sim_clock_now_samples() == samples_before;
		const int elapsed = timer.get_elapsed_time_ms();
		const bool case_passed = result == NULL && shutdown_.load()
			&& stable && !sim_connection_dirty()
			&& (!enabled || elapsed == 0);
		printf("[SIM-CONNECT-CLOCK] mode=%d stable=%d elapsed_ms=%d shutdown=%d clean=%d: %s\n",
			mode, stable, elapsed, (int)shutdown_.load(),
			!sim_connection_dirty(), case_passed ? "PASS" : "FAIL");
		passed = passed && case_passed;
		timer.stop();
	}
	sim_connect_attempt_limit_for_test = saved_limit;
	sim_port_for_test = saved_port;
	sim_close_connection();
	sim_clock_set_wire_stamp(saved_wire_stamp);
	sim_clock_set_enabled(saved_clock_enabled);
	shutdown_ = saved_shutdown;
	return passed ? 0 : 1;
}



#if defined(_WIN32)
int audioio_detach_leak_selftest(void)
{
	printf("[TEST-DETACH-LEAK] SKIP: bounded-join/detach is POSIX-only\n");
	return 0;
}
#endif
