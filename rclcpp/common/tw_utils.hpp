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

private:
  std::vector<int> histogram_;
  int bin_size_;
  int round_ns_;
  int max_ns_;
  long accum_;
  int cnt_;
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

#endif  // TW_UTILS_HPP_
