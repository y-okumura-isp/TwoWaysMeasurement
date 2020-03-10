#include <iostream>
#include <vector>
#include <numeric>

#include "tw_utils.hpp"

void print_result(const std::string name,
                  const int *jitters,
                  int count, int max_count)
{
  int num_loop = std::min(count, max_count);
  std::vector<int> vec;
  std::cout << name << " recent values: " << std::endl;
  std::cout << "  ";
  // print wakeup jitter
  for(int i=0; i<num_loop; i++) {
    std::cout << jitters[i] << " ";
    vec.push_back(jitters[i]);
  }
  std::cout << std::endl;

  std::cout << name << " average: " << std::accumulate(vec.begin(), vec.end(), 0.0) / num_loop << std::endl;
}
