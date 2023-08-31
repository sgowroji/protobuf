// Protocol Buffers - Google's data interchange format
// Copyright 2022 Google Inc.  All rights reserved.
// https://developers.google.com/protocol-buffers/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This file defines the internal class Prefetcher

#ifndef THIRD_PARTY_PROTOBUF_PREFETCHER_H
#define THIRD_PARTY_PROTOBUF_PREFETCHER_H

#include <algorithm>
#include <cstddef>

#include "absl/log/absl_check.h"


#ifdef SWIG
#error "You cannot SWIG proto headers"
#endif

// Must be included last.
#include "google/protobuf/port_def.inc"

namespace google {
namespace protobuf {
namespace internal {

// This class manages software prefetching by tracking previously-prefetched
// regions and prefetching the next regions by blocks.
class PROTOBUF_EXPORT Prefetcher {
 public:
  PROTOBUF_ALWAYS_INLINE Prefetcher(const char* ptr, const char* limit) {}

  PROTOBUF_ALWAYS_INLINE
  void MaybePrefetchForwards(const char* next) {}

  PROTOBUF_ALWAYS_INLINE
  void MaybePrefetchBackwards(const char* limit) {}
};
#else
class PROTOBUF_EXPORT Prefetcher {
  static constexpr ptrdiff_t kPrefetchForwardsDegree = ABSL_CACHELINE_SIZE * 16;
  static constexpr ptrdiff_t kPrefetchBackwardsDegree = ABSL_CACHELINE_SIZE * 6;

 public:
  PROTOBUF_ALWAYS_INLINE Prefetcher(const char* start, const char* end)
      : prefetch_start_(start), prefetch_end_(end) {}

  // Prefetch the next 1024 bytes after `prefetch_start_` and up to
  // `prefetch_end_`, if `next` is within 1024 bytes of `prefetch_start_`.
  PROTOBUF_ALWAYS_INLINE
  void MaybePrefetchForwards(const char* next) {
    ABSL_DCHECK_NE(prefetch_start_, nullptr);
    ABSL_DCHECK_LE(
        static_cast<const void*>(prefetch_start_),
        static_cast<const void*>(prefetch_end_ + ABSL_CACHELINE_SIZE));
    if (PROTOBUF_PREDICT_TRUE(prefetch_start_ - next > kPrefetchForwardsDegree))
      return;
    if (PROTOBUF_PREDICT_TRUE(prefetch_start_ < prefetch_end_)) {
      const char* prefetch_ptr =
          prefetch_start_;  // Use a temporary variable to
                            // avoid reloading prefetch_start_
      const char* end =
          std::min(prefetch_end_, prefetch_ptr + kPrefetchForwardsDegree);
      for (; prefetch_ptr < end; prefetch_ptr += ABSL_CACHELINE_SIZE) {
        absl::PrefetchToLocalCacheForWrite(prefetch_ptr);
      }
      prefetch_start_ = prefetch_ptr;
    }
  }

  PROTOBUF_ALWAYS_INLINE
  // Prefetch up to 6 cache lines before `prefetch_end_` and after
  // `prefetch_start_`, if `limit` is within  6 cache lines of `prefetch_end_`.
  void MaybePrefetchBackwards(const char* limit) {
    ABSL_DCHECK_NE(prefetch_end_, nullptr);
    ABSL_DCHECK_GE(
        static_cast<const void*>(prefetch_end_),
        static_cast<const void*>(prefetch_start_ - ABSL_CACHELINE_SIZE));
    if (PROTOBUF_PREDICT_TRUE(limit - prefetch_end_ > kPrefetchBackwardsDegree))
      return;
    if (PROTOBUF_PREDICT_TRUE(prefetch_end_ > prefetch_start_)) {
      // Use a temporary variable to avoid reloading prefetch_end_
      const char* prefetch_limit = prefetch_end_;
      const char* end =
          std::max(prefetch_start_, prefetch_limit - kPrefetchBackwardsDegree);
      for (; prefetch_limit > end; prefetch_limit -= ABSL_CACHELINE_SIZE) {
        absl::PrefetchToLocalCacheForWrite(prefetch_limit);
      }
      prefetch_end_ = prefetch_limit;
    }
  }

 private:
  // Current prefetch positions. Data from `ptr_` up to but not including
  // `prefetch_start_` is software prefetched. Similarly, data from `limit_`
  // down to but not including `prefetch_end_` is software prefetched.
  const char* prefetch_start_;
  const char* prefetch_end_;
};

}  // namespace internal
}  // namespace protobuf
}  // namespace google

#include "google/protobuf/port_undef.inc"

#endif  // THIRD_PARTY_PROTOBUF_PREFETCHER_H
