// Copyright 2024 Sebastian Theiler
#include "control_algorithms/algorithms/common.hpp"

#include <gtest/gtest.h>

#include <unordered_map>
#include <vector>

namespace control_algorithms {
using std::vector, std::unordered_map;

TEST(CellTest, EqualityOperators) {
  Cell c1{1, 2};
  Cell c2{1, 2};
  Cell c3{2, 3};

  EXPECT_TRUE(c1 == c2);
  EXPECT_FALSE(c1 == c3);
  EXPECT_TRUE(c1 != c3);
}

TEST(ComparePriorityCellTest, Comparison) {
  ComparePriorityCell comparator;

  PriorityCell p1{0, 0, 10};
  PriorityCell p2{0, 0, 20};

  EXPECT_FALSE(comparator(p1, p2));  // p1 has lower priority
  EXPECT_TRUE(comparator(p2, p1));   // p2 has higher priority
}

}  // namespace control_algorithms
