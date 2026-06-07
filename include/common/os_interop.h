/* Windows/Linux interoperability layer
 *
 * Copyright (C) 2020-2024 Rhizomatica
 * Author: Rafael Diniz <rafael@rhizomatica.org>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#pragma once


// threading support

#if defined(_WIN32)

#include <winsock2.h>
#include <windows.h>
#include <pthread.h>
#include <io.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif


union sigval {
    int           sival_int;     /* integer value */
    void          *sival_ptr;    /* pointer value */
};
struct sigevent {
    int           sigev_notify;  /* notification type */
    int           sigev_signo;   /* signal number */
    union sigval  sigev_value;   /* signal value */
};

int get_temp_path(char* pathBuffer, int pathBufferSize, const char* pathPart);
int MUTEX_LOCK(HANDLE *mqh_lock);
void MUTEX_UNLOCK(HANDLE *mqh_lock);
/* Returns 0 on success */
int COND_WAIT(HANDLE *mqh_wait, HANDLE *mqh_lock);
/* Returns 0 on success */
int COND_TIMED_WAIT(HANDLE *mqh_wait, HANDLE *mqh_lock, const struct timespec* abstime);
/* Returns 0 on success */
int COND_SIGNAL(HANDLE *mqh_wait);



#define TMP_ENV_NAME "TEMP"

#define O_NONBLOCK  0200000

#if 0
#define open            _open
#define read            _read
#define write           _write
#define close           _close
#define stat            _stat
#define fstat           _fstat
#define mkdir           _mkdir
#define snprintf        _snprintf
#define unlink _unlink
#define lseek _lseek
#if _MSC_VER <= 1200 /* Versions below VC++ 6 */
#define vsnprintf       _vsnprintf
#endif
#endif

#define O_RDONLY        _O_RDONLY
#define O_BINARY        _O_BINARY
#define O_CREAT         _O_CREAT
#define O_WRONLY        _O_WRONLY
#define O_TRUNC         _O_TRUNC
#define S_IREAD         _S_IREAD
#define S_IWRITE        _S_IWRITE
#define S_IFDIR         _S_IFDIR

#define S_IXUSR  0000100

#ifdef __cplusplus
}
#endif


#else

#include <pthread.h>
#include <sys/resource.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <unistd.h>

#define MUTEX_LOCK(x)   pthread_mutex_lock(x)
#define MUTEX_UNLOCK(x) pthread_mutex_unlock(x)
#define COND_WAIT(x, y)  pthread_cond_wait(x, y)
#define COND_TIMED_WAIT(x, y, z) pthread_cond_timedwait(x, y, z)
#define COND_SIGNAL(x)  pthread_cond_signal(x)


#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

// portable glibc-based srand/rand
long int __random (void);
void __srandom (unsigned int x);

// Per-instance RNG (single-process-sim-refactor.md §10.1, Landmine 1).
// The struct mirrors the file-static random_data_t in os_interop.cc; its layout
// MUST stay in sync with that definition. OS_RNG_STATE_WORDS is DEG_3 + 1 (the
// TYPE_3 state-table length) — a caller embeds an int32_t[OS_RNG_STATE_WORDS]
// alongside an os_random_data_t and calls os_rng_make() to bind+seed them. After
// that, __srandom_r2/__random_r2 drive that INDEPENDENT stream (residue-free of
// the file-static and of any other instance). Used by cl_telecom_system so two
// modem instances in one process do not cross-contaminate pre-eq channel /
// pilot / dispersal sequence generation.
#define OS_RNG_STATE_WORDS 32   /* == DEG_3 + 1 in os_interop.cc */

struct random_data_t
{
    int32_t *fptr;
    int32_t *rptr;
    int32_t *state;
    int rand_type;
    int rand_deg;
    int rand_sep;
    int32_t *end_ptr;
};

void     os_rng_make   (struct random_data_t *buf, int32_t *state_words, unsigned int seed);
void     __srandom_r2  (unsigned int seed, struct random_data_t *buf);
long int __random_r2   (struct random_data_t *buf);

#ifdef __cplusplus
};
#endif
