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

struct VoronoiData {
  bool is_voronoi = false;
  bool is_occupied = false;
  double dist_in_grid = 0.0;
  double dist_in_world = 0.0;
};

struct VoronoiSearchResult {
  std::vector<int> x;
  std::vector<int> y;
  int path_cost = 0;
};

class VoronoiPlanner {
 public:
  enum class SearchType { kAStar, kDP };

  VoronoiPlanner() = default;
  virtual ~VoronoiPlanner();

  void Init(int max_grid_x, int max_grid_y, double xy_grid_resolution,
            double circumscribed_radius);
  bool Search(int sx, int sy, int ex, int ey,
              const std::vector<std::vector<VoronoiData>>& gvd_map,
              VoronoiSearchResult* result);

 private:
  bool SetStart(int start_x, int start_y);
  bool SetEnd(int end_x, int end_y);
  bool SetStartAndEndConfiguration(int sx, int sy, int ex, int ey);
  Node2d* GetNode(int grid_x, int grid_y);
  int CalcHeuCost(int grid_x, int grid_y) const;
  bool IsWithinMap(int grid_x, int grid_y) const;
  bool IsValidCell(int grid_x, int grid_y) const;
  bool CheckVoronoi(int grid_x, int grid_y) const;
  int CalcGridXYIndex(int grid_x, int grid_y) const;
  int GetKey(const Node2d* node) const;
  void UpdateSuccs(const Node2d* curr_node);
  void ComputeGridSearchActions();
  int GetActionCost(int curr_x, int curr_y, int action_id) const;
  void LoadVoronoiSearchResult(int end_x, int end_y,
                               VoronoiSearchResult* result) const;
  void Clear();
  bool SearchPathToVoronoiEdges(int sx, int sy, int ex, int ey,
                                int* voronoi_end_x, int* voronoi_end_y,
                                VoronoiSearchResult* result);
  bool SearchPathAlongVoronoiEdges(int sx, int sy, int ex, int ey,
                                   VoronoiSearchResult* result);
  static void StitchSearchResult(const VoronoiSearchResult& start_to_voronoi,
                                 const VoronoiSearchResult& along_voronoi,
                                 const VoronoiSearchResult& voronoi_to_end,
                                 VoronoiSearchResult* result);

  int max_grid_x_ = 0;
  int max_grid_y_ = 0;
  double xy_grid_resolution_ = 0.0;
  double circumscribed_radius_ = 0.0;
  std::vector<std::vector<VoronoiData>> gvd_map_;
  Node2d* start_node_ = nullptr;
  Node2d* end_node_ = nullptr;
  SearchType search_type_;

  std::vector<std::vector<Node2d>> dp_lookup_table_;
  std::unique_ptr<common::Heap> open_list_ = nullptr;
  std::vector<common::Node::NodeStatus> closed_list_;
  std::size_t iterations_ = 0U;

  struct GridSearchPrimitives {
    std::array<int, common::kNumOfGridSearchActions> dx;
    std::array<int, common::kNumOfGridSearchActions> dy;
    std::array<int, common::kNumOfGridSearchActions> dxy_cost;
  };
  GridSearchPrimitives actions_;
  bool initialized_ = false;
  bool need_check_voronoi_ = false;
};

}  // namespace voronoi_planner
