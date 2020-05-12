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
  void init(int bin_size, int round_ns);
  void add(int ns);
  void print(const std::string & prefix);

  std::vector<int> histogram_;

private:
  int bin_size_;
  int round_ns_;
  int max_ns_;
  long accum_;
  int cnt_;
};

class JitterReportWithSkip
{
public:
  void init(int bin_size, int round_ns, int num_skip);
  void add(int ns);
  void print(const std::string & prefix);
  std::vector<int> getHistogram() const;

private:
  JitterReport jr_;
  int num_skip_;
  int num_skipped_;
};

inline void getnow(struct timespec *t)
{
  /* ROS2 uses CLOCK_MONOTONIC_RAW.
   * If you use another CLOCK_*, calcuration result may differ.
   */
  // clock_gettime(CLOCK_REALTIME, t);
  // clock_gettime(CLOCK_MONOTONIC, t);
  clock_gettime(CLOCK_MONOTONIC_RAW, t);
}

inline int64_t _timespec_to_long(const struct timespec *t)
{
  const int64_t nsec_per_sec = 1000 * 1000 * 1000;
  return (int64_t)t->tv_sec * nsec_per_sec + (int64_t)t->tv_nsec;
}

#endif  // TW_UTILS_HPP_
