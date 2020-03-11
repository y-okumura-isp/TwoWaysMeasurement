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
}

void JitterReport::add(int ns)
{
  int idx = ns / round_ns_;
  if (idx < 0) {
    idx = 0;
  }

  histogram_[std::min(idx, bin_size_-1)] += 1;
  max_ns_ = std::max(max_ns_, ns);
}

void JitterReport::print(const std::string & prefix)
{
  std::cout << prefix << std::endl;
  std::cout << "  max" << std::endl;
  std::cout << "    " << max_ns_ << std::endl;

  std::cout << "  histogram" << std::endl;
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
