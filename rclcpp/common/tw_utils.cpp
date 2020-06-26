#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>

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

void JitterReport::add(int64_t ns)
{
  int64_t idx = (ns - min_) / round_ns_;
  if (idx < 0) {
    idx = 0;
  }

  histogram_[std::min(idx, bin_size_-1)] += 1;
  max_ns_ = std::max(max_ns_, ns);
  accum_ += ns;
  cnt_++;
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

void JitterReportWithSkip::add(int64_t ns)
{
  if(num_skipped_ < num_skip_) {
    num_skipped_++;
    return;
  }
  jr_.add(ns);
}

void JitterReportWithSkip::print(const std::string & prefix)
{
  jr_.print(prefix);
}

std::vector<int64_t> JitterReportWithSkip::getHistogram() const
{
  return jr_.getHistogram();
}
