#include <gtest/gtest.h>

#include "tw_utils.hpp"

TEST(JitterReportTest, InitAdd) {
  JitterReport jr;
  int bin_size = 10;
  int round = 5;

  jr.init(bin_size, round);
  ASSERT_EQ(jr.histogram_.size(), (unsigned int) bin_size);

  jr.add(0); // add to histogram_[0]
  EXPECT_EQ(jr.histogram_[0], 1);

  jr.add(4); // add to histogram_[0]
  EXPECT_EQ(jr.histogram_[0], 2);

  jr.add(5); // add to histogram_[0]
  EXPECT_EQ(jr.histogram_[1], 1);
}


int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
