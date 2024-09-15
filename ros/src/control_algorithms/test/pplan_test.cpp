// Copyright 2024 Sebastian Theiler
#include "control_algorithms/algorithms/pplan.hpp"

#include <gtest/gtest.h>

#include <queue>
#include <vector>

namespace control_algorithms {

using std::vector, std::queue;

// Helper function to compare Cells
bool CellsEqual(const Cell &a, const Cell &b) {
  return a.x == b.x && a.y == b.y;
}

// Test for update_map_occ function
TEST(UpdateMapOccTest, BasicFunctionality) {
  // Initial setup
  vector<vector<Cell>> map_occ = {
      {{0, 0}, {2, 0}},  // Initial positions of 2 robots
      {{0, 1}, {2, 1}},  // Positions at time step 1
  };
  vector<Cell> start = {{0, 0}, {2, 0}};
  vector<Cell> plan = {{0, 1}, {0, 2}, {0, 3}};  // Plan for robot 0
  int robot_index = 0;

  // Run the function
  auto result = update_map_occ(map_occ, plan, robot_index, start);

  ASSERT_EQ(result.size(), 3);

  // Check each time step for robot 0
  EXPECT_TRUE(CellsEqual(result[0][0], Cell{0, 1}));
  EXPECT_TRUE(CellsEqual(result[1][0], Cell{0, 2}));
  EXPECT_TRUE(CellsEqual(result[2][0], Cell{0, 3}));

  // Check robot 1 positions (should remain unchanged from map_occ)
  EXPECT_TRUE(CellsEqual(result[0][1], Cell{2, 0}));
  EXPECT_TRUE(CellsEqual(result[1][1], Cell{2, 1}));
  EXPECT_EQ(result[2][1], (Cell{2, 1}));
  std::cout << "-==============" << std::endl;
  std::cout << result[2][1].x << "," << result[2][1].y << std::endl;
}

TEST(UpdateMapOccTest, WithTimeOffset) {
  vector<vector<Cell>> map_occ = {
      {{0, 0}, {2, 0}},
      {{0, 1}, {2, 1}},
  };
  vector<Cell> start = {{0, 0}, {2, 0}};
  vector<Cell> plan = {{0, 0}, {0, 1}, {0, 2}};
  int robot_index = 0;
  int time_offset = 1;

  auto result = update_map_occ(map_occ, plan, robot_index, start, time_offset);

  ASSERT_EQ(result.size(), 3);
  EXPECT_TRUE(CellsEqual(result[0][0], Cell{0, 0}));
  EXPECT_TRUE(CellsEqual(result[1][0], Cell{0, 1}));
  EXPECT_TRUE(CellsEqual(result[2][0], Cell{0, 2}));
}

TEST(UpdateMapOccTest, LongerPlanThanMapOcc) {
  vector<vector<Cell>> map_occ = {
      {{0, 0}, {2, 0}},
  };
  vector<Cell> start = {{0, 0}, {2, 0}};
  vector<Cell> plan = {{0, 1}, {0, 2}, {0, 3}};
  int robot_index = 0;
  int time_offset = 0;

  auto result = update_map_occ(map_occ, plan, robot_index, start, time_offset);

  ASSERT_EQ(result.size(), 3);
  EXPECT_TRUE(CellsEqual(result[2][0], Cell{0, 3}));
  EXPECT_TRUE(CellsEqual(result[2][1], Cell{2, 0}));
}

// Test for pplan function
TEST(PPlanTest, SimplePath) {
  Map map = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  vector<Cell> start = {Cell{0, 0}, Cell{2, 0}};
  vector<Cell> goal = {Cell{0, 2}, Cell{2, 2}};

  auto result = pplan(start, goal, map);

  ASSERT_FALSE(result.empty());
  EXPECT_TRUE(CellsEqual(result[0][0], Cell{0, 0}));
  EXPECT_TRUE(CellsEqual(result[2][0], Cell{0, 2}));
  EXPECT_TRUE(CellsEqual(result[0][1], Cell{2, 0}));
  EXPECT_TRUE(CellsEqual(result[2][1], Cell{2, 2}));
}

TEST(PPlanTest, ObstaclePath) {
  Map map = {{0, 0, 0}, {0, 1, 0}, {0, 0, 0}};
  vector<Cell> start = {Cell{0, 0}, Cell{2, 0}};
  vector<Cell> goal = {Cell{0, 2}, Cell{2, 2}};

  auto result = pplan(start, goal, map);

  ASSERT_FALSE(result.empty());
  EXPECT_TRUE(CellsEqual(result[1][0], Cell{0, 1}));
  EXPECT_TRUE(CellsEqual(result[1][1], Cell{2, 1}));
}

TEST(PPlanTest, ImpossiblePath) {
  Map map = {{0, 0, 0}, {1, 1, 1}, {0, 0, 0}};
  vector<Cell> start = {Cell{0, 0}, Cell{2, 2}};
  vector<Cell> goal = {Cell{2, 2}, Cell{0, 0}};

  auto result = pplan(start, goal, map);

  EXPECT_TRUE(result.empty());
}

TEST(PPlanTest, InvalidStartConditions) {
  Map map = {{1, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  vector<Cell> start = {Cell{0, 0}, Cell{2, 0}};
  vector<Cell> goal = {Cell{0, 2}, Cell{2, 2}};

  auto result = pplan(start, goal, map);

  EXPECT_TRUE(result.empty());
}

TEST(PPlanTest, RobotConflict) {
  Map map = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  vector<Cell> start = {Cell{0, 0}, Cell{0, 2}};
  vector<Cell> goal = {Cell{0, 2}, Cell{0, 0}};

  auto result = pplan(start, goal, map);

  ASSERT_FALSE(result.empty());
  EXPECT_FALSE(CellsEqual(result[1][0], result[1][1]));
}

TEST(PPlanTest, ComplexEnv) {
  Map map = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  };
  vector<Cell> start = {{1, 0}, {1, 7}, {3, 6}, {2, 8},
                        {2, 2}, {0, 3}, {1, 2}, {1, 4}};
  vector<Cell> goal = {{1, 9}, {1, 1}, {1, 0}, {0, 0},
                       {3, 7}, {2, 7}, {2, 9}, {3, 9}};

  auto result = pplan(start, goal, map);
  ASSERT_FALSE(result.empty());
}

}  // namespace control_algorithms
