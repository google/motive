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

#if defined(BENCHMARK_MOTIVE)

#include <assert.h>
#include <limits>
#include <math.h>
#include <sstream>
#include <vector>
#include "motive/util/benchmark.h"
#include "benchmark_common.h" // From mathfu


namespace fpl {

static const double kMicrosecondsPerSecond = 1000000.0;

// Holds an array of sampled data. Provide statistical analysis functions.
template<class T>
class SampleAnalyzer {
 public:
  // Buckets are used in the histogram functions.
  typedef uint16_t BucketType;
  typedef typename std::vector<BucketType> BucketArray;
  static const int kDefaultHistogramWidth = 80;
  static const int kDefaultHistogramHeight = 10;

  // Functions to set the time data.
  explicit SampleAnalyzer(const char* name) : name_(name) {}
  void Append(T sample) { samples_.push_back(sample); }
  void Clear() { samples_.clear(); }
  void SetName(const char* name) { name_ = name; }

  // Const-functions to analyze the samples.
  size_t NumSamples() const { return samples_.size(); }
  BenchmarkTime Average() const;
  BenchmarkTime StandardDeviation(BenchmarkTime average) const;
  void MinMax(BenchmarkTime* min_time, BenchmarkTime* max_time) const;
  void Histogram(BenchmarkTime min, BenchmarkTime max,
                 BucketArray* buckets_pointer) const;
  std::string Statistics(double to_usec, int width = kDefaultHistogramWidth,
                         int height = kDefaultHistogramHeight) const;

 private:
  // The sampled data. This vector is never shrunk. It will incur some
  // overhead when it gets reallocated, but once it reaches its highwater
  // mark, there will be no reallocation.
  std::vector<T> samples_;

  // The name is used in functions that output text, like Statistics().
  std::string name_;
};

template<class T>
BenchmarkTime SampleAnalyzer<T>::Average() const {
  // Gather the sum of all the samples.
  // TODO: properly handle overflow in sum.
  BenchmarkTime sum = 0;
  for (auto it = samples_.begin(); it != samples_.end(); ++it) {
    sum += *it;
  }

  // Divide by the count, with proper rounding.
  const BenchmarkTime count = samples_.size();
  return (sum + count / 2) / count;
}

template<class T>
BenchmarkTime SampleAnalyzer<T>::StandardDeviation(BenchmarkTime average) const {
  // Gather the sum of squares.
  // TODO: handle possible overflow in sum.
  BenchmarkTime deviation_sum = 0;
  for (auto it = samples_.begin(); it != samples_.end(); ++it) {
    const BenchmarkTime diff = *it - average;
    deviation_sum += diff * diff;
  }

  // Use doubles to calculate the square root of deviation_sum.
  const BenchmarkTime count = samples_.size();
  const BenchmarkTime deviation_squared = (deviation_sum + count / 2) / count;
  const double deviation = sqrt(static_cast<double>(deviation_squared));
  const double deviation_rounded = round(deviation);
  return static_cast<BenchmarkTime>(deviation_rounded);
}

template<class T>
void SampleAnalyzer<T>::MinMax(BenchmarkTime* min_time, BenchmarkTime* max_time) const {
  BenchmarkTime min = std::numeric_limits<BenchmarkTime>::max();
  BenchmarkTime max = std::numeric_limits<BenchmarkTime>::min();
  for (auto it = samples_.begin(); it != samples_.end(); ++it) {
    min = std::min(min, *it);
    max = std::max(max, *it);
  }
  *min_time = min;
  *max_time = max;
}

template<class T>
void SampleAnalyzer<T>::Histogram(BenchmarkTime min, BenchmarkTime max,
                                  BucketArray* buckets_pointer) const {
  // Use all the buckets we can.
  BucketArray& buckets = *buckets_pointer;
  buckets.resize(buckets.capacity());

  // Empty the buckets.
  std::fill(buckets.begin(), buckets.end(), 0);

  // Handle case where all samples are the same, to avoid division by zero.
  const BenchmarkTime num_buckets = buckets.size();
  const BenchmarkTime width = max - min;
  if (width == 0) {
    buckets[0] = samples_.size();
    return;
  }

  // Place each sample in the appropriate bucket.
  // TODO: handle possible overflow in multiplication.
  for (auto it = samples_.begin(); it != samples_.end(); ++it) {
    const BenchmarkTime t = *it;
    const BenchmarkTime bucket = (num_buckets - 1) * (t - min) / (max - min);
    assert(bucket < num_buckets);
    buckets[bucket]++;
  }
}

template<class T>
std::string SampleAnalyzer<T>::Statistics(double to_usec, int width,
                                          int height) const {
  std::stringstream s;

  // Output general statistics.
  BenchmarkTime min = 0;
  BenchmarkTime max = 0;
  MinMax(&min, &max);
  const BenchmarkTime avg = Average();
  const BenchmarkTime stdev = StandardDeviation(avg);
  s << name_ << ": "
    << "average " << avg * to_usec << "usec"
    << ", min " << min * to_usec << "usec"
    << ", max " << max * to_usec << "usec"
    << ", standard deviation " << stdev * to_usec << "usec" << std::endl;

  // Gather a histogram with 'width' buckets.
  BucketArray buckets(width);
  Histogram(min, max, &buckets);

  // Find the histogram bucket with the most samples so that we can scale the
  // height.
  BucketType biggest_bucket = 0;
  for (int j = 1; j < width; ++j) {
    biggest_bucket = std::max(biggest_bucket, buckets[j]);
  }

  // Output an ASCII histogram.
  for (int i = 0; i < height; ++i) {
    const uint32_t bucket_cutoff = static_cast<uint32_t>(height - i - 1) *
                                       biggest_bucket / height;
    for (int j = 0; j < width; ++j) {
      s << (buckets[j] > bucket_cutoff ? '#' : ' ');
    }
    s << std::endl;
  }
  return s.str();
}


// One SampleAnalyzer per id being benchmarked.
typedef SampleAnalyzer<BenchmarkTime> TimeAnalyzer;
static std::vector<TimeAnalyzer> gTimes;

BenchmarkTime GetBenchmarkTime() { return Timer::GetTicks(); }

void InitBenchmarks(int num_ids) {
  Timer::InitializeTickPeriod();
  gTimes.reserve(num_ids);
}

void ClearBenchmarks() {
  for (auto it = gTimes.begin(); it != gTimes.end(); ++it) {
    it->Clear();
  }
}

int RegisterBenchmark(const char* name) {
  const int id = static_cast<int>(gTimes.size());
  gTimes.push_back(TimeAnalyzer(name));
  return id;
}

void OutputBenchmarks() {
  const double to_usec = Timer::tick_period() * kMicrosecondsPerSecond;
  printf("\n");
  for (size_t i = 0; i < gTimes.size(); ++i) {
    if (gTimes[i].NumSamples() > 0) {
      printf("%s\n", gTimes[i].Statistics(to_usec).c_str());
    }
  }
}

Benchmark::~Benchmark() {
  BenchmarkTime end_time = GetBenchmarkTime();

  assert(0 <= id_ && id_ < static_cast<int>(gTimes.size()));
  gTimes[id_].Append(end_time - start_time_);
}

}  // namespace fpl

#else
// Add a symbol to avoid the compiler warning about this file having no symbols.
// This symbol will be culled by the linker since it is never referenced.
int gMotiveBenchmarkCppSymbolToAvoidCompilerWarning;
#endif // defined(BENCHMARK_MOTIVE)
