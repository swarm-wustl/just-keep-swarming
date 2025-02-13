// Copyright 2024 Sebastian Theiler
#include "control_algorithms/algorithms/astar.hpp"

#include <gtest/gtest.h>

#include <unordered_map>
#include <vector>

namespace control_algorithms {
using std::vector, std::unordered_map;

TEST(GetNeighborsTest, ValidNeighbors) {
  Map map = {{0, 0, 0}, {0, 1, 0}, {0, 0, 0}};

  vector<Cell> expected_neighbors = {Cell{0, 0}, Cell{0, 2}};

  vector<Cell> neighbors = get_neighbors(map, Cell{0, 1});
  EXPECT_EQ(neighbors.size(), expected_neighbors.size());

  for (const Cell &expected : expected_neighbors) {
    EXPECT_NE(std::find(neighbors.begin(), neighbors.end(), expected),
              neighbors.end());
  }
}

TEST(HTest, ManhattanDistance) {
  Cell a{0, 0};
  Cell b{3, 4};

  EXPECT_EQ(h(a, b), 7);  // 3 (x) + 4 (y) = 7
}

TEST(ReconstructPathTest, PathReconstruction) {
  unordered_map<Cell, Cell> came_from = {{Cell{1, 1}, Cell{1, 0}},
                                         {Cell{1, 0}, Cell{0, 0}},
                                         {Cell{0, 0}, Cell{0, 0}}};

  vector<Cell> expected_path = {Cell{0, 0}, Cell{1, 0}, Cell{1, 1}};

  vector<Cell> path = reconstruct_path(came_from, Cell{1, 1});
  EXPECT_EQ(path.size(), expected_path.size());

  for (size_t i = 0; i < expected_path.size(); ++i) {
    EXPECT_EQ(path[i], expected_path[i]);
  }
}

TEST(AStarTest, FindPath) {
  Map map = {{0, 0, 0, 0, 0}, {0, 1, 1, 1, 0}, {0, 0, 0, 0, 0}};

  vector<Cell> expected_path = {Cell{0, 0}, Cell{0, 1}, Cell{0, 2}, Cell{0, 3},
                                Cell{0, 4}, Cell{1, 4}, Cell{2, 4}};

  vector<Cell> path = astar(Cell{0, 0}, Cell{2, 4}, map);
  EXPECT_EQ(path.size(), expected_path.size());

  for (size_t i = 0; i < expected_path.size(); ++i) {
    EXPECT_EQ(path[i], expected_path[i]);
  }
}

TEST(AStarTest, NoPath) {
  Map map = {{0, 0, 1, 0, 0}, {0, 0, 1, 0, 0}, {0, 0, 1, 0, 0}};

  vector<Cell> path = astar(Cell{0, 0}, Cell{2, 4}, map);
  EXPECT_TRUE(path.empty());  // No path should be found
}

TEST(AStarTest, StartEqualsGoal) {
  Map map = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

  vector<Cell> path = astar(Cell{1, 1}, Cell{1, 1}, map);
  EXPECT_TRUE(path.empty());  // No path needed if start equals goal
}

TEST(AStarTest, ObstacleInStartOrGoal) {
  Map map = {{1, 0, 0}, {0, 0, 0}, {0, 0, 1}};

  EXPECT_TRUE(astar(Cell{0, 0}, Cell{2, 2}, map).empty());  // Start is blocked
  EXPECT_TRUE(astar(Cell{2, 2}, Cell{0, 0}, map).empty());  // Goal is blocked
}

}  // namespace control_algorithms
