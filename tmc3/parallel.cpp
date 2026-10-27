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
#include <mutex>
#include <thread>
#include <vector>

namespace pcc {

//============================================================================

namespace {

  // Set while a thread executes chunks, so that nested calls run inline
  thread_local bool tlInParallel = false;

  //--------------------------------------------------------------------------
  // A pool of worker threads that execute the chunks of one job at a time,
  // together with the thread that submitted the job.

  class ThreadPool {
  public:
    ~ThreadPool() { resize(0); }

    void resize(int numWorkers)
    {
      {
        std::lock_guard<std::mutex> lock(_mutex);
        _stop = true;
      }
      _wake.notify_all();
      for (auto& worker : _workers)
        worker.join();
      _workers.clear();

      _stop = false;
      for (int i = 0; i < numWorkers; i++)
        _workers.emplace_back([this] { workerLoop(); });
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

    std::vector<std::thread> _workers;

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

  int gNumThreads = 1;

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

//============================================================================

}  // namespace pcc
