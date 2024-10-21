// Copyright 2024 Sebastian Theiler
#include "control_algorithms/algorithms/assemble_graph.hpp"

#include <gtest/gtest.h>

namespace control_algorithms {

using std::size_t;

TEST(AdjMatrix, Initialize) {
  int n_robots = 11;
  AdjMatrix adj_matrix = init_adj_matrix(n_robots);
  EXPECT_EQ(adj_matrix.size(), n_robots);
  EXPECT_EQ(adj_matrix[0].size(), n_robots);
}

TEST(AdjMatrix, ConnectRobots) {
  int n_robots = 11;
  AdjMatrix adj_matrix = init_adj_matrix(n_robots);
  connect_robots(&adj_matrix, 0, 1, FRONT, BACK);
  EXPECT_EQ(adj_matrix[0][1], FRONT);
  EXPECT_EQ(adj_matrix[1][0], BACK);
}

TEST(AdjMatrix, LoadStructureWalker) {
  int n_robots = 11;
  AdjMatrix adj_matrix = load_structure(MOBILE_MANIPULATOR, n_robots);
}

TEST(AdjMatrix, LoadStructureMobileManipulator) {
  int n_robots = 11;
  AdjMatrix adj_matrix = load_structure(MOBILE_MANIPULATOR, n_robots);
}

TEST(StructureGraph, VertexHeight) {
  int n_robots = 5;
  AdjMatrix adj_matrix = init_adj_matrix(n_robots);
}

TEST(StructureLoading, MobileManipulator) {
  int num_robots = 10;
  AdjMatrix mat = load_structure(MOBILE_MANIPULATOR, num_robots);

  EXPECT_EQ(mat.size(), num_robots);
  for (const auto &row : mat) {
    EXPECT_EQ(row.size(), num_robots);
  }

  EXPECT_EQ(mat[0][1], LEFT);
  EXPECT_EQ(mat[1][0], RIGHT);
  EXPECT_EQ(mat[1][3], FRONT);
  EXPECT_EQ(mat[1][5], BACK);
  EXPECT_EQ(mat[0][2], RIGHT);
  EXPECT_EQ(mat[2][0], LEFT);
  EXPECT_EQ(mat[0][7], FRONT);
  EXPECT_EQ(mat[7][0], FRONT);

  for (size_t i = 8; i < num_robots; i++) {
    EXPECT_EQ(mat[i - 1][i], BACK);
    EXPECT_EQ(mat[i][i - 1], FRONT);
  }
}

TEST(StructureLoading, Walker) {
  int num_robots = 11;
  AdjMatrix mat = load_structure(WALKER, num_robots);

  EXPECT_EQ(mat.size(), num_robots);
  for (const auto &row : mat) {
    EXPECT_EQ(row.size(), num_robots);
  }

  EXPECT_EQ(mat[0][1], BACK);
  EXPECT_EQ(mat[1][0], FRONT);
  EXPECT_EQ(mat[1][3], LEFT);
  EXPECT_EQ(mat[3][1], LEFT);
  EXPECT_EQ(mat[3][4], FRONT);
  EXPECT_EQ(mat[4][3], FRONT);
  EXPECT_EQ(mat[0][2], FRONT);
  EXPECT_EQ(mat[2][0], FRONT);
  EXPECT_EQ(mat[2][7], LEFT);
  EXPECT_EQ(mat[7][2], RIGHT);
}

TEST(StructureLoading, InsufficientRobots) {
  AdjMatrix mat_manipulator = load_structure(MOBILE_MANIPULATOR, 7);
  EXPECT_TRUE(mat_manipulator.empty());

  AdjMatrix mat_walker = load_structure(WALKER, 10);
  EXPECT_TRUE(mat_walker.empty());
}

TEST(HeightCalculation, CalcHeights) {
  unsigned int num_robots = 10;
  AdjMatrix adj_matrix = load_structure(MOBILE_MANIPULATOR, num_robots);

  unordered_map<int, int> heights = calculate_matrix_heights(adj_matrix, 0);

  EXPECT_EQ(heights.size(), num_robots);
  EXPECT_EQ(heights[0], 4);
  EXPECT_EQ(heights[1], 2);
  EXPECT_EQ(heights[2], 2);
  EXPECT_EQ(heights[3], 1);
  EXPECT_EQ(heights[4], 1);
  EXPECT_EQ(heights[5], 1);
  EXPECT_EQ(heights[6], 1);
  EXPECT_EQ(heights[7], 3);
  EXPECT_EQ(heights[8], 2);
  EXPECT_EQ(heights[9], 1);
}

TEST(GraphUnfolding, UnfoldGraph) {
  int num_robots = 11;
  AdjMatrix adj_matrix = load_structure(WALKER, num_robots);

  Cell root = {10, 10};
  int root_n = 0;
  int radius = 1;

  vector<Cell> unfolded = unfold_graph(adj_matrix, root, root_n, radius);

  EXPECT_EQ(unfolded.size(), 11);
  EXPECT_EQ(unfolded[0], (Cell{10, 10}));
  EXPECT_EQ(unfolded[1], (Cell{10, 9}));
  EXPECT_EQ(unfolded[2], (Cell{10, 11}));
  EXPECT_EQ(unfolded[3], (Cell{9, 9}));
  EXPECT_EQ(unfolded[4], (Cell{9, 10}));
  EXPECT_EQ(unfolded[5], (Cell{11, 9}));
  EXPECT_EQ(unfolded[6], (Cell{11, 10}));
  EXPECT_EQ(unfolded[7], (Cell{9, 11}));
  EXPECT_EQ(unfolded[8], (Cell{9, 12}));
  EXPECT_EQ(unfolded[9], (Cell{11, 11}));
  EXPECT_EQ(unfolded[10], (Cell{11, 12}));
}

}  // namespace control_algorithms
