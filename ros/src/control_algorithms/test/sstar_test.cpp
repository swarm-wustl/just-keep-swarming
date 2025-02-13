// Copyright 2024 Sebastian Theiler
#include "control_algorithms/algorithms/sstar.hpp"

#include <gtest/gtest.h>

#include "control_algorithms/algorithms/common.hpp"

namespace control_algorithms {

// Helper function to compare Cells
bool CellsEqual(const Cell &a, const Cell &b) {
  return a.x == b.x && a.y == b.y;
}

// Test get_net_neighbors function
TEST(GetNetNeighborsTest, BasicFunctionality) {
  Map map = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  vector<Cell> state = {{1, 1}, {2, 1}};

  auto neighbors = get_net_neighbors(map, state);

  EXPECT_FALSE(neighbors.empty());
  EXPECT_EQ(neighbors.size(), 16);
}

TEST(GetNetNeighborsTest, WithObstacle) {
  Map map = {{1, 1, 1}, {1, 0, 1}, {1, 0, 1}, {1, 1, 1}};
  vector<Cell> state = {{0, 0}, {2, 0}};

  auto neighbors = get_net_neighbors(map, state);

  EXPECT_FALSE(neighbors.empty());
  EXPECT_LT(neighbors.size(), 9);  // Should be less than 9 due to obstacle
}

// Test count_moved function
TEST(CountMovedTest, NoMove) {
  vector<Cell> original = {{0, 0}, {1, 1}};
  vector<Cell> changed = {{0, 0}, {1, 1}};

  EXPECT_EQ(count_moved(original, changed), 0);
}

TEST(CountMovedTest, OneMove) {
  vector<Cell> original = {{0, 0}, {1, 1}};
  vector<Cell> changed = {{0, 1}, {1, 1}};

  EXPECT_EQ(count_moved(original, changed), 1);
}

TEST(CountMovedTest, AllMove) {
  vector<Cell> original = {{0, 0}, {1, 1}};
  vector<Cell> changed = {{0, 1}, {1, 2}};

  EXPECT_EQ(count_moved(original, changed), 2);
}

// Test sstar function
TEST(SStarTest, SimplePath) {
  Map map = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  vector<Cell> start = {{0, 0}, {2, 0}};
  vector<Cell> goal = {{0, 2}, {2, 2}};

  auto result = sstar(start, goal, map);

  EXPECT_FALSE(result.empty());
  EXPECT_EQ(result.size(), 3);
  EXPECT_TRUE(CellsEqual(result.front()[0], Cell{0, 0}));
  EXPECT_TRUE(CellsEqual(result.back()[0], Cell{0, 2}));
  EXPECT_TRUE(CellsEqual(result.front()[1], Cell{2, 0}));
  EXPECT_TRUE(CellsEqual(result.back()[1], Cell{2, 2}));
}

TEST(SStarTest, WithObstacle) {
  Map map = {{0, 1, 0}, {0, 1, 0}, {0, 0, 0}, {0, 0, 0}};
  vector<Cell> start = {{0, 0}, {0, 2}};
  vector<Cell> goal = {{0, 2}, {0, 0}};

  auto result = sstar(start, goal, map);

  EXPECT_FALSE(result.empty());
  EXPECT_GT(result.size(), 3);  // Should take more steps due to obstacle
}

TEST(SStarTest, ImpossiblePath) {
  Map map = {{0, 0, 0}, {1, 1, 1}, {0, 0, 0}};
  vector<Cell> start = {{0, 0}, {2, 2}};
  vector<Cell> goal = {{2, 2}, {0, 0}};

  auto result = sstar(start, goal, map);

  EXPECT_TRUE(result.empty());
}

TEST(SStarTest, InvalidStartConditions) {
  Map map = {{1, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  vector<Cell> start = {{0, 0}, {2, 0}};
  vector<Cell> goal = {{0, 2}, {2, 2}};

  auto result = sstar(start, goal, map);

  EXPECT_TRUE(result.empty());
}

TEST(SStarTest, RobotConflict) {
  Map map = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  vector<Cell> start = {{0, 0}, {0, 2}};
  vector<Cell> goal = {{0, 2}, {0, 0}};

  auto result = sstar(start, goal, map);

  EXPECT_FALSE(result.empty());
}

TEST(SStarTest, ComplexEnv) {
  Map map = {
      {1, 1, 1, 1, 1, 1, 0, 1, 1, 1},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
  };
  vector<Cell> start = {{1, 0}, {1, 7}};
  vector<Cell> goal = {{1, 9}, {1, 1}};

  auto result = sstar(start, goal, map);
  EXPECT_FALSE(result.empty());
}

}  // namespace control_algorithms
