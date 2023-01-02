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
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "voronoi_planner/heap.h"

namespace voronoi_planner {

struct VoronoiData {
  bool is_voronoi;
  double dist;
};

class Grid2DSearchState : public AbstractSearchState {
 public:
  /**
   * \brief coordinates
   */
  int x, y;

  /**
   * \brief search relevant data
   */
  int g;

  Grid2DSearchState* predecessor;

 public:
  Grid2DSearchState() = default;
  virtual ~Grid2DSearchState() = default;
};

class VoronoiPlanner {
 public:
  VoronoiPlanner();
  ~VoronoiPlanner();

  bool search(int start_x, int start_y, int goal_x, int goal_y, int* path_cost,
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
    if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_) {
      return false;
    }
    return true;
  }

  int index(int x, int y) const { return x + y * size_x_; }

  int heuristic(int x, int y, int goal_x, int goal_y) const {
    return 10 * std::max(abs(x - goal_x), abs(y - goal_y));
  }

  void CreateSearchSpace();
  void ReInitializeSearchSpace();
  void InitializeSearchState(Grid2DSearchState* search_state);
  void computedxy();
  void destroy();

 private:
  int size_x_ = 0;
  int size_y_ = 0;
  Grid2DSearchState** search_space_ = nullptr;
  CIntHeap* open_list_ = nullptr;

  // largest optimal g-value computed by search
  int largestcomputedoptf_ = 0;

  int dx_[8];
  int dy_[8];
  int dxy_cost_[8];
  bool use_heuristic_ = true;
};

}  // namespace voronoi_planner
