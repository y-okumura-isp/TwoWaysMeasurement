#include <cstring>
#include <malloc.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>

#include <rttest/rttest.h>

#include "tw_utils.hpp"

void JitterReport::init(int64_t bin_size, int64_t round_ns, int64_t min)
{
  bin_size_ = bin_size;
  round_ns_ = round_ns;
  min_ = min;
  max_ns_ = 0;
  histogram_.resize(bin_size);
  accum_ = 0;
  cnt_ = 0;
}

bool JitterReport::add(int64_t ns)
{
  int64_t idx = (ns - min_) / round_ns_;
  if (idx < 0) {
    idx = 0;
  }

  histogram_[std::min(idx, bin_size_-1)] += 1;
  max_ns_ = std::max(max_ns_, ns);
  accum_ += ns;
  cnt_++;
  return max_ns_ == ns;
}

void JitterReport::print(const std::string & prefix)
{
  std::cout << prefix << std::endl;
  std::cout << "  max: " << max_ns_ << "[ns]" << std::endl;
  std::cout << "  avg: " << accum_ / cnt_ << "[ns]" << std::endl;

  std::cout << "  histogram"
            << " round_ns = " << round_ns_
            << " bin = " << bin_size_
            << " min = " << min_
            << std::endl;
  std::cout << "    ";
  std::for_each(histogram_.begin(),
                histogram_.end(),
                [](int64_t x)
                {
                  std::cout << x << ", ";
                });
  std::cout << std::endl;
  std::cout << std::endl;

}

void JitterReportWithSkip::init(int64_t bin_size, int64_t round_ns, int64_t num_skip, int64_t min)
{
  jr_.init(bin_size, round_ns, min);
  num_skip_ = num_skip;
  num_skipped_ = 0;
}

bool JitterReportWithSkip::add(int64_t ns)
{
  if(num_skipped_ < num_skip_) {
    num_skipped_++;
    return false;
  }
  return jr_.add(ns);
}

void JitterReportWithSkip::print(const std::string & prefix)
{
  jr_.print(prefix);
}

std::vector<int64_t> JitterReportWithSkip::getHistogram() const
{
  return jr_.getHistogram();
}

int lock_and_prefault_dynamic(size_t prefault_dynamic_size)
{
  std::cout << "prefault_dynamic_size = " << prefault_dynamic_size << std::endl;
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    perror("mlockall failed");
    return -1;
  }

  // Turn off malloc trimming.
  if (mallopt(M_TRIM_THRESHOLD, -1) == 0) {
    perror("mallopt for trim threshold failed");
    munlockall();
    return -1;
  }

  // Turn off mmap usage.
  if (mallopt(M_MMAP_MAX, 0) == 0) {
    perror("mallopt for mmap failed");
    mallopt(M_TRIM_THRESHOLD, 128 * 1024);
    munlockall();
    return -1;
  }

  struct rusage usage;
  size_t page_size = sysconf(_SC_PAGESIZE);
  getrusage(RUSAGE_SELF, &usage);
  size_t prev_minflts = usage.ru_minflt;
  size_t prev_majflts = usage.ru_majflt;
  size_t encountered_minflts = 1;
  size_t encountered_majflts = 1;

  size_t array_size = sizeof(char) * 64 * page_size;
  size_t total_size = 0;
  size_t max_size = prefault_dynamic_size;
  std::vector<char *> prefaulter;
  prefaulter.reserve((size_t)(max_size / array_size));

  // prefault until you see no more pagefaults
  while (encountered_minflts > 0 || encountered_majflts > 0) {
    char * ptr;
    try {
      ptr = new char[array_size];
      memset(ptr, 0, array_size);
      total_size += array_size;
    } catch (std::bad_alloc & e) {
      fprintf(stderr, "Caught exception: %s\n", e.what());
      fprintf(stderr, "Unlocking memory and continuing.\n");
      for (auto & ptr : prefaulter) {
        delete[] ptr;
      }

      mallopt(M_TRIM_THRESHOLD, 128 * 1024);
      mallopt(M_MMAP_MAX, 65536);
      munlockall();
      return -1;
    }

    // If we reached max_size then delete created char array.
    // This will prevent pagefault on next allocation.
    if (total_size >= max_size) {
      delete[] ptr;
    } else {
      prefaulter.push_back(ptr);
    }

    getrusage(RUSAGE_SELF, &usage);
    size_t current_minflt = usage.ru_minflt;
    size_t current_majflt = usage.ru_majflt;
    encountered_minflts = current_minflt - prev_minflts;
    encountered_majflts = current_majflt - prev_majflts;
    prev_minflts = current_minflt;
    prev_majflts = current_majflt;
  }

  for (auto & ptr : prefaulter) {
    delete[] ptr;
  }
  return 0;
}

void set_sched_priority(const std::string &thread_name,
                        size_t priority,
                        int policy)
{
  if (rttest_set_sched_priority(priority,
                                policy) != 0) {
    std::cout << "Couldn't ";
  }

  std::cout << "set scheduling priority(" << priority << ")"
            << " and policy(" << policy << ")"
            << " to " << thread_name
            << std::endl;
}
