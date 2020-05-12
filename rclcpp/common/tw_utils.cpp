#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>

#include "tw_utils.hpp"

void JitterReport::init(int bin_size, int round_ns)
{
  bin_size_ = bin_size;
  round_ns_ = round_ns;
  max_ns_ = 0;
  histogram_.resize(bin_size);
  accum_ = 0;
  cnt_ = 0;
}

void JitterReport::add(int ns)
{
  int idx = ns / round_ns_;
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
            << std::endl;
  std::cout << "    ";
  std::for_each(histogram_.begin(),
                histogram_.end(),
                [](int x)
                {
                  std::cout << x << ", ";
                });
  std::cout << std::endl;
  std::cout << std::endl;

}

void JitterReportWithSkip::init(int bin_size, int round_ns, int num_skip)
{
  jr_.init(bin_size, round_ns);
  num_skip_ = num_skip;
  num_skipped_ = 0;
}

void JitterReportWithSkip::add(int ns)
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

