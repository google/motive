// Copyright 2014 Google Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOTIVE_UTIL_BENCHMARK_H_
#define MOTIVE_UTIL_BENCHMARK_H_

namespace motive {

#define FPL_TOKEN_PASTE_NESTED(a, b) a##b
#define FPL_TOKEN_PASTE(a, b) FPL_TOKEN_PASTE_NESTED(a, b)
#define FPL_UNIQUE(token) FPL_TOKEN_PASTE(token, __LINE__)

#if defined(BENCHMARK_MOTIVE)

/// A raw tick count from the system. Guaranteed to increase.
typedef unsigned long long BenchmarkTime;

/// Get the current system tick count. The unit varies from system to system.
BenchmarkTime GetBenchmarkTime();

/// Initialize the benchmark tracking system. Multiple things can be benchmarked
/// simultaneously. Each thing has its own 'id'. We allocate storage for ids
/// between 0~num_ids-1.
void InitBenchmarks(int num_ids);

/// Empty all samples that have been collected with 'Benchmark'.
void ClearBenchmarks();

/// Allocate storage on a tag to gather benchmark data for.
/// Returns the `id` to be passed into the `Benchmark` constructor.
int RegisterBenchmark(const char* name);

/// Dump an analysis of the samples to stdout.
void OutputBenchmarks();

/// @class Benchmark
/// @brief Record the time for the scope of this variable.
///
/// Creates a benchmark sample. We can sample several things at once. The thing
/// we're sampling is specified by 'id'.
/// The sample is the time between creation and destruction of the Benchmark.
class Benchmark {
 public:
  explicit Benchmark(int id) : id_(id), start_time_(GetBenchmarkTime()) {}
  ~Benchmark();
 private:
  int id_;
  BenchmarkTime start_time_;
};

#define FPL_BENCHMARK(name) \
  static int FPL_UNIQUE(id) = fplbase::RegisterBenchmark(name); \
  const fplbase::Benchmark FPL_UNIQUE(benchmark)(FPL_UNIQUE(id))

#else // not defined(BENCHMARK_MOTIVE)

// Stub out these calls so that they don't generate any code.
inline void InitBenchmarks(int /*num_ids*/) {}
inline void ClearBenchmarks() {}
inline int RegisterBenchmark(const char* /*name*/) { return -1; }
inline void OutputBenchmarks() {}
class Benchmark {
 public:
  explicit Benchmark(int /*id*/) {}
};

#define FPL_BENCHMARK(name)

#endif // not defined(BENCHMARK_MOTIVE)


}  // namespace motive

#endif  // MOTIVE_UTIL_BENCHMARK_H_