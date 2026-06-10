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

#include "datalink_layer/timer.h"
#include "common/sim_clock.h"


// clock_gettime implementation for WIN32
// https://stackoverflow.com/questions/5404277/porting-clock-gettime-to-windows
#if defined(_WIN32)
#include <windows.h>
#define MS_PER_SEC      1000ULL     // MS = milliseconds
#define US_PER_MS       1000ULL     // US = microseconds
#define HNS_PER_US      10ULL       // HNS = hundred-nanoseconds (e.g., 1 hns = 100 ns)
#define NS_PER_US       1000ULL

#define HNS_PER_SEC     (MS_PER_SEC * US_PER_MS * HNS_PER_US)
#define NS_PER_HNS      (100ULL)    // NS = nanoseconds
#define NS_PER_SEC      (MS_PER_SEC * US_PER_MS * NS_PER_US)

#define CLOCK_REALTIME             0
#define CLOCK_MONOTONIC            1
#define CLOCK_MONOTONIC_RAW        4

int clock_gettime_monotonic(struct timespec *tv)
{
    static LARGE_INTEGER ticksPerSec;
    LARGE_INTEGER ticks;

    if (!ticksPerSec.QuadPart) {
        QueryPerformanceFrequency(&ticksPerSec);
        if (!ticksPerSec.QuadPart) {
            errno = ENOTSUP;
            return -1;
        }
    }

    QueryPerformanceCounter(&ticks);

    tv->tv_sec = (long)(ticks.QuadPart / ticksPerSec.QuadPart);
    tv->tv_nsec = (long)(((ticks.QuadPart % ticksPerSec.QuadPart) * NS_PER_SEC) / ticksPerSec.QuadPart);

    return 0;
}

int clock_gettime_realtime(struct timespec *tv)
{
    FILETIME ft;
    ULARGE_INTEGER hnsTime;

    GetSystemTimePreciseAsFileTime(&ft);

    hnsTime.LowPart = ft.dwLowDateTime;
    hnsTime.HighPart = ft.dwHighDateTime;

    // To get POSIX Epoch as baseline, subtract the number of hns intervals from Jan 1, 1601 to Jan 1, 1970.
    hnsTime.QuadPart -= (11644473600ULL * HNS_PER_SEC);

    // modulus by hns intervals per second first, then convert to ns, as not to lose resolution
    tv->tv_nsec = (long) ((hnsTime.QuadPart % HNS_PER_SEC) * NS_PER_HNS);
    tv->tv_sec = (long) (hnsTime.QuadPart / HNS_PER_SEC);

    return 0;
}

int clock_gettime(clockid_t type, struct timespec *tp)
{
    if (type == CLOCK_MONOTONIC || type == CLOCK_MONOTONIC_RAW)
    {
        return clock_gettime_monotonic(tp);
    }
    else if (type == CLOCK_REALTIME)
    {
        return clock_gettime_realtime(tp);
    }

    errno = ENOTSUP;
    return -1;
}
#endif

// Single funnel for every cl_timer clock read. When the -x sim virtual clock
// is active (sim_clock_enabled()), the timer measures VIRTUAL channel time
// (driven by samples flowing through rx_transfer) instead of wall-clock, so
// the entire ARQ control loop — PTT delays, ACK/HAIL/BREAK timeouts,
// retransmit timers — runs at host-compute speed, decoupled from real time.
// When disabled (every production mode), it falls through to the EXACT same
// clock_gettime(CLOCK_MONOTONIC_RAW) the stock timer used, so behavior is
// byte-identical. This is the ONLY change to the timer; reset()/stop() math
// is untouched.
static inline void cl_timer_clock_read(struct timespec *tp)
{
	if (sim_clock_enabled())
		sim_clock_fill_timespec(tp);
	else
		clock_gettime(CLOCK_MONOTONIC_RAW, tp);
}

cl_timer::cl_timer()
{
	seconds=0;
	miliSeconds=0;
	microseconds=0;
	nanoseconds=0;
	counting=NO;
	// Value-init the timespecs so an out-of-order stop()/_continue()/update()
	// (called before the matching start() on some recovery/config-transition
	// paths) reads tv_nsec=0 instead of uninitialized stack garbage. Without
	// this, stopTime.tv_nsec - startTime.tv_nsec subtracts garbage and can
	// overflow signed long (tv_nsec is 4-byte long on MinGW), which UBSan
	// flags as sub_overflow at timer.cc:152 (also :169/:187). With both
	// timespecs zeroed, the pre-start read computes 0-0=0 (defined). No effect
	// on the normal start->stop path (start() overwrites startTime first).
	startTime.tv_sec=0;
	startTime.tv_nsec=0;
	stopTime.tv_sec=0;
	stopTime.tv_nsec=0;
}

cl_timer::~cl_timer()
{

}

void cl_timer::reset()
{
	seconds=0;
	miliSeconds=0;
	microseconds=0;
	nanoseconds=0;
	// Keep the timespecs defined too (defensive; start() overwrites startTime
	// immediately after reset() on the normal path — see ctor note).
	startTime.tv_sec=0;
	startTime.tv_nsec=0;
	stopTime.tv_sec=0;
	stopTime.tv_nsec=0;
}

void cl_timer::start()
{
	this->reset();
	cl_timer_clock_read(&startTime);
	counting=YES;

}

void cl_timer::stop()
{
	cl_timer_clock_read(&stopTime);
	seconds=stopTime.tv_sec-startTime.tv_sec;
	nanoseconds=stopTime.tv_nsec-startTime.tv_nsec;
	if(nanoseconds<0)
	{
		nanoseconds= 1000000000+nanoseconds;
		seconds--;
	}
	miliSeconds=nanoseconds/1000000;
	microseconds=nanoseconds/1000-miliSeconds*1000;
	nanoseconds=nanoseconds-miliSeconds*1000000-microseconds*1000;
	counting=NO;
}

void cl_timer::_continue()
{

	cl_timer_clock_read(&stopTime);
	seconds=stopTime.tv_sec-startTime.tv_sec;
	nanoseconds=stopTime.tv_nsec-startTime.tv_nsec;
	if(nanoseconds<0)
	{
		nanoseconds= 1000000000+nanoseconds;
		seconds--;
	}
	miliSeconds=nanoseconds/1000000;
	microseconds=nanoseconds/1000-miliSeconds*1000;
	nanoseconds=nanoseconds-miliSeconds*1000000-microseconds*1000;
	counting=YES;
}

void cl_timer::update()
{
	if(counting==YES)
	{
		cl_timer_clock_read(&stopTime);
		seconds=stopTime.tv_sec-startTime.tv_sec;
		nanoseconds=stopTime.tv_nsec-startTime.tv_nsec;
		if(nanoseconds<0)
		{
			nanoseconds= 1000000000+nanoseconds;
			seconds--;
		}
		miliSeconds=nanoseconds/1000000;
		microseconds=nanoseconds/1000-miliSeconds*1000;
		nanoseconds=nanoseconds-miliSeconds*1000000-microseconds*1000;
	}
}

int cl_timer::get_elapsed_time_ms()
{
	this->update();
	return this->seconds*1000+miliSeconds;
}

int cl_timer::get_counter_status()
{
	return this->counting;
}

void cl_timer::print()
{
	std::cout<< seconds << "s and "<<miliSeconds<<" ms and "<<microseconds<<" us and "<<nanoseconds<<" ns";
}
