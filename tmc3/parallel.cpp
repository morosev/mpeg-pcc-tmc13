/* The copyright in this software is being made available under the BSD
 * Licence, included below.  This software may be subject to other third
 * party and contributor rights, including patent rights, and no such
 * rights are granted under this licence.
 *
 * Copyright (c) 2026, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the ISO/IEC nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "parallel.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <exception>
#include <iostream>
#include <mutex>
#include <system_error>
#include <thread>
#include <vector>

#if defined(__unix__) || defined(__APPLE__)
#  define PCC_POSIX_THREADS 1
#  include <pthread.h>
#  include <sys/resource.h>
#endif

namespace pcc {

//============================================================================

namespace {

  // Set while a thread executes chunks, so that nested calls run inline
  thread_local bool tlInParallel = false;

  // The redirected coding message stream of the thread, if any
  thread_local std::ostream* tlCodingLog = nullptr;

#if defined(PCC_POSIX_THREADS)
  //--------------------------------------------------------------------------
  // The stack size of the worker threads: that of the main thread.
  // NB: secondary threads otherwise get a much smaller stack (eg, 512 kB on
  //     macOS) than the main thread (eg, 8 MB), which the coding functions,
  //     with their large stack frames, can exceed.

  size_t
  workerStackSize()
  {
    size_t size = size_t(8) << 20;
    struct rlimit limit;
    if (
      !getrlimit(RLIMIT_STACK, &limit) && limit.rlim_cur != RLIM_INFINITY
      && limit.rlim_cur > size)
      size = limit.rlim_cur;
    return size;
  }

  //--------------------------------------------------------------------------
  // The stack size of the calling thread, or 0 if unknown

  size_t
  currentStackSize()
  {
#  if defined(__APPLE__)
    return pthread_get_stacksize_np(pthread_self());
#  elif defined(__linux__)
    size_t size = 0;
    pthread_attr_t attr;
    if (!pthread_getattr_np(pthread_self(), &attr)) {
      pthread_attr_getstacksize(&attr, &size);
      pthread_attr_destroy(&attr);
    }
    return size;
#  else
    return 0;
#  endif
  }
#endif

  //--------------------------------------------------------------------------
  // A pool of worker threads that execute the chunks of one job at a time,
  // together with the thread that submitted the job.

  class ThreadPool {
  public:
    ~ThreadPool() { resize(0); }

    void resize(int numWorkers)
    {
      // NB: waits for a running job; jobs submitted meanwhile run inline
      std::lock_guard<std::mutex> runLock(_runMutex);
      {
        std::lock_guard<std::mutex> lock(_mutex);
        _stop = true;
      }
      _wake.notify_all();
      for (auto& worker : _workers)
        join(worker);
      _workers.clear();

      _stop = false;
      for (int i = 0; i < numWorkers; i++)
        _workers.push_back(start());
    }

    // Run the job on the pool, or return false if the pool is busy
    bool tryRun(
      int64_t count,
      int64_t grain,
      const std::function<void(int64_t, int64_t)>& fn)
    {
      std::unique_lock<std::mutex> runLock(_runMutex, std::try_to_lock);
      if (!runLock.owns_lock())
        return false;

      {
        std::lock_guard<std::mutex> lock(_mutex);
        _fn = &fn;
        _count = count;
        _grain = grain;
        _numChunks = (count + grain - 1) / grain;
        _nextChunk = 0;
        _error = nullptr;
        _active = int(_workers.size());
        _generation++;
      }
      _wake.notify_all();

      work();

      std::unique_lock<std::mutex> lock(_mutex);
      _done.wait(lock, [this] { return _active == 0; });
      _fn = nullptr;
      if (_error)
        std::rethrow_exception(_error);
      return true;
    }

  private:
#if defined(PCC_POSIX_THREADS)
    using Worker = pthread_t;

    Worker start()
    {
      pthread_attr_t attr;
      pthread_attr_init(&attr);
      pthread_attr_setstacksize(&attr, workerStackSize());
      pthread_t thread;
      int err = pthread_create(
        &thread, &attr,
        [](void* pool) -> void* {
          static_cast<ThreadPool*>(pool)->workerLoop();
          return nullptr;
        },
        this);
      pthread_attr_destroy(&attr);
      if (err)
        throw std::system_error(err, std::generic_category(), "pthread_create");
      return thread;
    }

    static void join(Worker& worker) { pthread_join(worker, nullptr); }
#else
    using Worker = std::thread;

    Worker start()
    {
      return std::thread([this] { workerLoop(); });
    }

    static void join(Worker& worker) { worker.join(); }
#endif

    void workerLoop()
    {
      uint64_t seen = 0;
      for (;;) {
        {
          std::unique_lock<std::mutex> lock(_mutex);
          _wake.wait(lock, [&] { return _stop || _generation != seen; });
          if (_stop)
            return;
          seen = _generation;
        }

        work();

        std::lock_guard<std::mutex> lock(_mutex);
        if (--_active == 0)
          _done.notify_all();
      }
    }

    // Execute chunks of the current job until none remain
    void work()
    {
      tlInParallel = true;
      for (;;) {
        const int64_t chunk = _nextChunk.fetch_add(1);
        if (chunk >= _numChunks)
          break;

        const int64_t begin = chunk * _grain;
        const int64_t end = std::min(_count, begin + _grain);
        try {
          (*_fn)(begin, end);
        }
        catch (...) {
          std::lock_guard<std::mutex> lock(_mutex);
          if (!_error)
            _error = std::current_exception();
          _nextChunk = _numChunks;
        }
      }
      tlInParallel = false;
    }

    std::vector<Worker> _workers;

    // Serialises jobs from different submitting threads
    std::mutex _runMutex;

    std::mutex _mutex;
    std::condition_variable _wake;
    std::condition_variable _done;
    bool _stop = false;
    uint64_t _generation = 0;

    // The current job
    const std::function<void(int64_t, int64_t)>* _fn = nullptr;
    int64_t _count = 0;
    int64_t _grain = 1;
    int64_t _numChunks = 0;
    std::atomic<int64_t> _nextChunk{0};
    int _active = 0;
    std::exception_ptr _error;
  };

  //--------------------------------------------------------------------------

  std::atomic<int> gNumThreads{1};

  ThreadPool&
  pool()
  {
    static ThreadPool instance;
    return instance;
  }

}  // namespace

//============================================================================

void
setNumThreads(int numThreads)
{
  gNumThreads = std::max(1, numThreads);
  pool().resize(gNumThreads - 1);
}

//----------------------------------------------------------------------------

int
numThreads()
{
  return gNumThreads;
}

//----------------------------------------------------------------------------

void
parallelFor(
  int64_t count,
  int64_t grain,
  const std::function<void(int64_t begin, int64_t end)>& fn)
{
  if (count <= 0)
    return;

  grain = std::max<int64_t>(1, grain);

  // NB: when the pool is busy with another caller, run inline
  if (gNumThreads > 1 && !tlInParallel && count > grain)
    if (pool().tryRun(count, grain, fn))
      return;

  fn(0, count);
}

//----------------------------------------------------------------------------

void
parallelInvoke(const std::function<void()>& a, const std::function<void()>& b)
{
  parallelFor(2, 1, [&](int64_t begin, int64_t end) {
    for (int64_t i = begin; i < end; i++)
      (i ? b : a)();
  });
}

//----------------------------------------------------------------------------

void
runWithCodingStack(const std::function<void()>& fn)
{
#if defined(PCC_POSIX_THREADS)
  const size_t stackSize = workerStackSize();
  const size_t current = currentStackSize();
  if (!current || current >= stackSize) {
    fn();
    return;
  }

  struct Call {
    const std::function<void()>* fn;
    std::exception_ptr error;
  } call{&fn, nullptr};

  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, stackSize);
  pthread_t thread;
  int err = pthread_create(
    &thread, &attr,
    [](void* arg) -> void* {
      auto* call = static_cast<Call*>(arg);
      try {
        (*call->fn)();
      }
      catch (...) {
        call->error = std::current_exception();
      }
      return nullptr;
    },
    &call);
  pthread_attr_destroy(&attr);
  if (err)
    throw std::system_error(err, std::generic_category(), "pthread_create");

  pthread_join(thread, nullptr);
  if (call.error)
    std::rethrow_exception(call.error);
#else
  fn();
#endif
}

//============================================================================

std::ostream&
codingLog()
{
  return tlCodingLog ? *tlCodingLog : std::cout;
}

//----------------------------------------------------------------------------

CodingLogRedirect::CodingLogRedirect(std::ostream& os) : _prev(tlCodingLog)
{
  tlCodingLog = &os;
}

//----------------------------------------------------------------------------

CodingLogRedirect::~CodingLogRedirect()
{
  tlCodingLog = _prev;
}

//============================================================================

}  // namespace pcc
