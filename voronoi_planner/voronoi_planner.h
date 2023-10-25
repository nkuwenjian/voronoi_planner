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

#include "voronoi_planner/common/constants.h"
#include "voronoi_planner/common/heap.h"
#include "voronoi_planner/node2d.h"

namespace voronoi_planner {

enum class SearchType : int { A_STAR, DP };

struct VoronoiData {
  bool is_voronoi = false;
  double dist = 0.0;
};

struct GridSearchPrimitives {
  std::array<int, common::kNumOfGridSearchActions> dx;
  std::array<int, common::kNumOfGridSearchActions> dy;
  std::array<int, common::kNumOfGridSearchActions> dxy_cost;
};

struct GridSearchResult {
  std::vector<std::pair<int, int>> grid_path;
  int path_cost = 0;
};

class VoronoiPlanner {
 public:
  VoronoiPlanner() = default;
  ~VoronoiPlanner();
  void Init(int max_grid_x, int max_grid_y, double circumscribed_radius);
  bool Search(int sx, int sy, int ex, int ey,
              std::vector<std::vector<VoronoiData>>&& gvd_map,
              std::vector<std::pair<int, int>>* path);

 private:
  bool SetStart(int start_x, int start_y);
  bool SetEnd(int end_x, int end_y);
  Node2d* GetNode(int grid_x, int grid_y);
  bool IsValidCell(int grid_x, int grid_y) const;
  int CalcHeuCost(int grid_x, int grid_y) const;
  bool SetStartAndEndConfiguration(int sx, int sy, int ex, int ey);
  bool IsWithinMap(int grid_x, int grid_y) const;
  bool CheckVoronoi(int grid_x, int grid_y) const;
  int CalcGridXYIndex(int grid_x, int grid_y) const;
  int GetKey(const Node2d* node) const;
  void UpdateSuccs(const Node2d* curr_node);
  int GetActionCost(int curr_x, int curr_y, int action_id) const;
  void LoadGridSearchResult(int end_x, int end_y,
                            GridSearchResult* result) const;
  void ComputeGridSearchActions();
  void Clear();
  bool SearchPathToVoronoiEdges(int sx, int sy, int ex, int ey,
                                int* voronoi_goal_x, int* voronoi_goal_y,
                                GridSearchResult* result);
  bool SearchPathAlongVoronoiEdges(int sx, int sy, int ex, int ey,
                                   GridSearchResult* result);

  int max_grid_x_ = 0;
  int max_grid_y_ = 0;
  std::vector<std::vector<Node2d>> dp_lookup_table_;
  std::unique_ptr<common::Heap> open_list_ = nullptr;
  std::vector<common::NodeStatus> closed_list_;
  double circumscribed_radius_;
  Node2d* start_node_ = nullptr;
  Node2d* end_node_ = nullptr;
  std::vector<std::vector<VoronoiData>> gvd_map_;

  GridSearchPrimitives actions_;
  SearchType search_type_;
  bool need_check_voronoi_ = false;
  std::size_t iterations_ = 0U;
  bool initialized_ = false;
};

}  // namespace voronoi_planner
