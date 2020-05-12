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

  jr.add(44);
  EXPECT_EQ(jr.histogram_[8], 1);

  jr.add(45);
  EXPECT_EQ(jr.histogram_[9], 1);

  jr.add(50);
  EXPECT_EQ(jr.histogram_[9], 2);
}

TEST(JitterReportWithSkip, InitAdd) {
  JitterReportWithSkip jr;
  int bin_size = 10;
  int round = 5;
  int num_skip = 3;

  jr.init(bin_size, round, num_skip);
  ASSERT_EQ(jr.getHistogram().size(), (unsigned int) bin_size);

  jr.add(0); // skip
  jr.add(0); // skip
  jr.add(0); // skip
  for(int i=0; i<bin_size; i++) {
    EXPECT_EQ(jr.getHistogram()[i], 0);
  }

  jr.add(0);
  EXPECT_EQ(jr.getHistogram()[0], 1);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
