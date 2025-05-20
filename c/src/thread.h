#ifndef BLADERF_THREAD_H_
#define BLADERF_THREAD_H_

#include <assert.h>

/* Determine which threading implementation to use */
#if (defined(_WIN32) || defined(USE_NATIVE_THREADS)) && !defined(DO_NOT_USE_PTHREADS)
/* On Windows, default to native threads unless DO_NOT_USE_PTHREADS is defined */
#undef USE_PTHREADS
#else
#undef USE_PTHREADS
#define USE_PTHREADS
#endif

#ifdef USE_PTHREADS

#include <pthread.h>
#include <errno.h>
#include <time.h>

#define THREAD pthread_t
#define MUTEX  pthread_mutex_t
#define COND   pthread_cond_t

#ifdef ENABLE_LOCK_CHECKS
#   define MUTEX_INIT(m) do { \
        int status; \
        pthread_mutexattr_t attr; \
        status = pthread_mutexattr_init(&attr); \
        assert(status == 0); \
        status = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK); \
        assert(status == 0); \
        status = pthread_mutex_init(m, &attr); \
        assert(status == 0); \
    } while (0)
#   define MUTEX_LOCK(m) do { int status = pthread_mutex_lock(m); assert(status == 0); } while (0)
#   define MUTEX_UNLOCK(m) do { int status = pthread_mutex_unlock(m); assert(status == 0); } while (0)
#   define MUTEX_DESTROY(m) do { int status = pthread_mutex_destroy(m); assert(status == 0); } while (0)
#else
#   define MUTEX_INIT(m)    pthread_mutex_init(m, NULL)
#   define MUTEX_LOCK(m)    pthread_mutex_lock(m)
#   define MUTEX_UNLOCK(m)  pthread_mutex_unlock(m)
#   define MUTEX_DESTROY(m) pthread_mutex_destroy(m)
#endif

#define THREAD_CREATE(h,f,a) pthread_create(h, NULL, f, a)
#define THREAD_SUCCESS 0
#define THREAD_TIMEOUT ETIMEDOUT
#define THREAD_CANCEL(t) pthread_cancel(t)
#define THREAD_JOIN(t,s) pthread_join(t, s)
#define THREAD_EXIT(s) pthread_exit(s)

#define COND_INIT(c)    pthread_cond_init(c, NULL)
#define COND_SIGNAL(c)  pthread_cond_signal(c)
#define COND_DESTROY(c) pthread_cond_destroy(c)

static inline int posix_cond_timedwait(pthread_cond_t *c, pthread_mutex_t *m, unsigned int t)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += t / 1000;
    ts.tv_nsec += (t % 1000) * 1000000;
    if (ts.tv_nsec >= 1000000000) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000;
    }
    return pthread_cond_timedwait(c, m, &ts);
}
#define COND_TIMED_WAIT(c,m,t) posix_cond_timedwait(c,m,t)
#define COND_WAIT(c,m)         pthread_cond_wait(c,m)

#else /* USE_PTHREADS */

#include <windows.h>
#include <time.h>

#define THREAD HANDLE
#define MUTEX  CRITICAL_SECTION
#define COND   CONDITION_VARIABLE

#define MUTEX_INIT(m)    InitializeCriticalSection(m)
#define MUTEX_LOCK(m)    EnterCriticalSection(m)
#define MUTEX_UNLOCK(m)  LeaveCriticalSection(m)
#define MUTEX_DESTROY(m) DeleteCriticalSection(m)

#define THREAD_CREATE(h,f,a) \
    ((*(h) = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)(f),(LPVOID)(a),0,NULL)) == NULL ? GetLastError() : 0)
#define THREAD_SUCCESS 0
#define THREAD_TIMEOUT WAIT_TIMEOUT
#define THREAD_CANCEL(t) TerminateThread(t,0)
#define THREAD_JOIN(t,s) WaitForSingleObject(t, INFINITE)
#define THREAD_EXIT(s) ExitThread(s)

#define COND_INIT(c)    (InitializeConditionVariable(c), 0)
#define COND_SIGNAL(c)  WakeConditionVariable(c)
#define COND_DESTROY(c) ((void)0)
#define COND_TIMED_WAIT(c,m,t) (SleepConditionVariableCS(c,m,t) ? 0 : GetLastError())
#define COND_WAIT(c,m)  (!SleepConditionVariableCS(c,m,INFINITE))

#endif /* USE_PTHREADS */

#endif /* BLADERF_THREAD_H_ */
