#include <gtest/gtest.h>

#include "tw_utils.hpp"

TEST(JitterReportTest, InitAdd) {
  JitterReport jr;
  int bin_size = 10;
  int round = 5;

  jr.init(bin_size, round);
  ASSERT_EQ(jr.getHistogram().size(), (unsigned int) bin_size);

  jr.add(0); // add to histogram_[0]
  EXPECT_EQ(jr.getHistogram()[0], 1);

  jr.add(4); // add to histogram_[0]
  EXPECT_EQ(jr.getHistogram()[0], 2);

  jr.add(5); // add to histogram_[0]
  EXPECT_EQ(jr.getHistogram()[1], 1);

  jr.add(44);
  EXPECT_EQ(jr.getHistogram()[8], 1);

  jr.add(45);
  EXPECT_EQ(jr.getHistogram()[9], 1);

  jr.add(50);
  EXPECT_EQ(jr.getHistogram()[9], 2);
}

TEST(JitterReport, WithMinusMin) {
  JitterReport jr;
  int64_t bin_size = 10;
  int64_t round = 5;
  int64_t min = -10;

  jr.init(bin_size, round, min);
  jr.add(-10);
  EXPECT_EQ(jr.getHistogram()[0], 1);

  jr.add(-5);
  EXPECT_EQ(jr.getHistogram()[1], 1);

  jr.add(0);
  EXPECT_EQ(jr.getHistogram()[2], 1);

  EXPECT_EQ(jr.get_max_ns(), 0);
  EXPECT_EQ(jr.get_average(), (-10-5)/3);
}

TEST(JitterReport, WithPlusMin) {
  JitterReport jr;
  int64_t bin_size = 10;
  int64_t round = 5;
  int64_t min = 10;

  jr.init(bin_size, round, min);
  jr.add(0);
  EXPECT_EQ(jr.getHistogram()[0], 1);

  jr.add(5);
  EXPECT_EQ(jr.getHistogram()[0], 2);

  jr.add(10);
  EXPECT_EQ(jr.getHistogram()[0], 3);

  jr.add(15);
  EXPECT_EQ(jr.getHistogram()[1], 1);

  EXPECT_EQ(jr.get_max_ns(), 15);
  EXPECT_EQ(jr.get_average(), (5+10+15)/4);
}

// default num_skip
TEST(JitterReportWithSkip, InitAddDefault) {
  JitterReportWithSkip jr;
  int bin_size = 10;
  int round = 5;

  jr.init(bin_size, round);
  ASSERT_EQ(jr.getHistogram().size(), (unsigned int) bin_size);

  for(int i=0; i<10; i++) {
    jr.add(0); // skip
  }
  for(int i=0; i<bin_size; i++) {
    EXPECT_EQ(jr.getHistogram()[i], 0);
  }

  jr.add(0); // skip
  ASSERT_EQ(jr.getHistogram().size(), (unsigned int) bin_size);
}

// set num_skip
TEST(JitterReportWithSkip, InitAddSetNumSkip) {
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
