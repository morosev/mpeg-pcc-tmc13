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

#pragma once

#include <cstdint>
#include <functional>
#include <ostream>

namespace pcc {

//============================================================================
// Parallel execution of independent work.
//
// The number of threads defaults to one, in which case all work runs on the
// calling thread and no threads are created.  Callers must ensure that the
// result does not depend on how the work is divided between threads: each
// chunk may only write state that no other chunk reads or writes.

// Set the number of threads used by parallelFor (including the caller).
void setNumThreads(int numThreads);

// The number of threads used by parallelFor.
int numThreads();

// Call fn(begin, end) for consecutive chunks covering [0, count), and
// return once every chunk has completed.  When run on several threads, the
// chunks have at most grain items and are handed to threads dynamically, in
// increasing order; otherwise fn(0, count) is called on the calling thread.
//
// NB: calls made from within a chunk, or while the threads are busy with
//     work submitted by another thread, run on the calling thread.
//     An exception thrown by a chunk is rethrown to the caller once all
//     started chunks have completed.
void parallelFor(
  int64_t count,
  int64_t grain,
  const std::function<void(int64_t begin, int64_t end)>& fn);

// Run a and b, possibly concurrently, and return once both have completed.
void parallelInvoke(
  const std::function<void()>& a, const std::function<void()>& b);

// Call fn on the calling thread if its stack is large enough for the coding
// functions, otherwise on a temporary thread with the stack size of the main
// thread (eg, for threads created by an application, which get a small
// stack on some systems).  An exception thrown by fn is rethrown.
void runWithCodingStack(const std::function<void()>& fn);

//----------------------------------------------------------------------------
// The stream for the coding messages of the current thread: std::cout,
// unless redirected (eg, to keep the messages of a slice coded concurrently
// with others in order).

std::ostream& codingLog();

// Redirects codingLog() on the current thread while the object exists.
class CodingLogRedirect {
public:
  explicit CodingLogRedirect(std::ostream& os);
  ~CodingLogRedirect();

  CodingLogRedirect(const CodingLogRedirect&) = delete;
  CodingLogRedirect& operator=(const CodingLogRedirect&) = delete;

private:
  std::ostream* _prev;
};

//============================================================================

}  // namespace pcc
