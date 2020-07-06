#ifndef TW_UTILS_HPP_
#define TW_UTILS_HPP_

#include <time.h>
#include <array>

/**
 * Create jitter report such as histogram, max_value.
 *
 * Histogram with bin size = bin_size
 * Ranges are:
 *   [0,            round_ns),
 *   [round_ns,     round_ns * 2),
 *   [round_ns * 2, round_ns * 3),
 *   // snip
 *   [round_ns * (bin_size -1), infinity)
 *
 * max_value
 *   maximim_value
 */
class JitterReport
{
public:
  void init(int64_t bin_size, int64_t round_ns, int64_t min=0);
  /**
   * return true if ns become max value
   */
  bool add(int64_t ns);
  void print(const std::string & prefix);
  std::vector<int64_t> getHistogram() const {
    return histogram_;
  }

  int64_t get_max_ns() const { return max_ns_; }
  int64_t get_average() const { return accum_ / cnt_; }

private:

  int64_t bin_size_;
  int64_t round_ns_;
  int64_t min_;

  // histogram
  std::vector<int64_t> histogram_;
  // max value
  int64_t max_ns_;
  // accumurator & count to calculate average
  int64_t accum_;
  int64_t cnt_;
};

class JitterReportWithSkip
{
public:
  void init(int64_t bin_size, int64_t round_ns, int64_t num_skip=10, int64_t min=0);
  bool add(int64_t ns);
  void print(const std::string & prefix);
  std::vector<int64_t> getHistogram() const;

private:
  JitterReport jr_;
  int64_t num_skip_;
  int64_t num_skipped_;
};

inline void getnow(struct timespec *t, bool uses_clock_monotonic_raw = false)
{
  /* ROS2 uses CLOCK_MONOTONIC_RAW.
   * If you use another CLOCK_*, calcuration result may differ.
   */
  clockid_t clock = uses_clock_monotonic_raw ? CLOCK_MONOTONIC_RAW : CLOCK_REALTIME;
  // std::cout << "clock = " << clock
  // << " CLOCK_MONOTONIC_RAW: " << CLOCK_MONOTONIC_RAW
  // << " CLOCK_REALTIME " << CLOCK_REALTIME
  // << std::endl;
  clock_gettime(clock, t);
}

inline int64_t _timespec_to_long(const struct timespec *t)
{
  const int64_t nsec_per_sec = 1000 * 1000 * 1000;
  return (int64_t)t->tv_sec * nsec_per_sec + (int64_t)t->tv_nsec;
}

// Copied from rttest/rttest.cpp because we don't use Rttest class.
int lock_and_prefault_dynamic(size_t prefault_dynamic_size);

/// rttest_set_sched_priority wrapper
void set_sched_priority(const std::string &thread_name,
                        size_t priority,
                        int policy);


#endif  // TW_UTILS_HPP_
