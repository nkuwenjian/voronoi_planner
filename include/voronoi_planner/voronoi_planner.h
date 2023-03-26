/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "voronoi_planner/heap.h"

namespace voronoi_planner {

struct VoronoiData {
  bool is_voronoi;
  double dist;
};

class Grid2DSearchState : public SearchStateBase {
 public:
  Grid2DSearchState(int x, int y) : x_(x), y_(y) {}
  ~Grid2DSearchState() override = default;

  int x() const { return x_; }
  int y() const { return y_; }
  int g() const { return g_; }
  const Grid2DSearchState* predecessor() const { return predecessor_; }
  void set_x(const int x) { x_ = x; }
  void set_y(const int y) { y_ = y; }
  void set_g(const int g) { g_ = g; }
  void set_predecessor(const Grid2DSearchState* predecessor) {
    predecessor_ = predecessor;
  }

  int x_ = 0;
  int y_ = 0;
  int g_ = INFINITECOST;
  const Grid2DSearchState* predecessor_ = nullptr;
};

class VoronoiPlanner {
 public:
  VoronoiPlanner();
  ~VoronoiPlanner();

  bool Search(int start_x, int start_y, int goal_x, int goal_y, int* path_cost,
              std::vector<std::pair<int, int>>* path, int size_x, int size_y,
              VoronoiData** voronoi_diagram, double circumscribed_radius);

 private:
  bool SearchInVoronoi(int start_x, int start_y, int goal_x, int goal_y,
                       int* path_cost, std::vector<std::pair<int, int>>* path,
                       VoronoiData** voronoi_diagram,
                       double circumscribed_radius);
  bool SearchShortestPathToVoronoi(int start_x, int start_y, int goal_x,
                                   int goal_y, int* voronoi_goal_x,
                                   int* voronoi_goal_y, int* path_cost,
                                   std::vector<std::pair<int, int>>* path,
                                   VoronoiData** voronoi_diagram,
                                   double circumscribed_radius);

 private:
  bool WithinSearchSpace(int x, int y) const {
    return (x >= 0 && x < size_x_ && y >= 0 && y < size_y_);
  }

  int Index(int x, int y) const { return x + y * size_x_; }

  static int Heuristic(int x, int y, int goal_x, int goal_y) {
    return 10 * std::max(abs(x - goal_x), abs(y - goal_y));
  }

  void CreateSearchSpace();
  void ReInitializeSearchSpace();
  void InitializeSearchState(Grid2DSearchState* search_state);
  void Computedxy();
  void Clear();

 private:
  int size_x_ = 0;
  int size_y_ = 0;
  std::vector<std::vector<Grid2DSearchState*>> search_space_;
  std::unique_ptr<Heap> open_list_ = nullptr;

  // largest optimal g-value computed by search
  int largestcomputedoptf_ = 0;

  std::array<int, 8> dx_;
  std::array<int, 8> dy_;
  std::array<int, 8> dxy_cost_;
  bool use_heuristic_ = true;
};

}  // namespace voronoi_planner
