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

#include "voronoi_planner/voronoi_planner.h"

#include <algorithm>
#include <chrono>  // NOLINT

#include "glog/logging.h"

#include "voronoi_planner/common/heap.h"

namespace voronoi_planner {

void VoronoiPlanner::Init(int max_grid_x, int max_grid_y,
                          double xy_grid_resolution,
                          double circumscribed_radius) {
  if (initialized_) {
    LOG(INFO) << "VoronoiPlanner has been initialized.";
    return;
  }

  max_grid_x_ = max_grid_x;
  max_grid_y_ = max_grid_y;
  xy_grid_resolution_ = xy_grid_resolution;
  circumscribed_radius_ = circumscribed_radius;

  open_list_ = std::make_unique<common::Heap>();

  dp_lookup_table_.resize(max_grid_x_);
  for (int grid_x = 0; grid_x < max_grid_x; ++grid_x) {
    for (int grid_y = 0; grid_y < max_grid_y; ++grid_y) {
      dp_lookup_table_[grid_x].emplace_back(grid_x, grid_y);
    }
  }

  ComputeGridSearchActions();
  initialized_ = true;
  LOG(INFO) << "VoronoiPlanner is initialized successfully.";
}

VoronoiPlanner::~VoronoiPlanner() { open_list_->Clear(); }

void VoronoiPlanner::Clear() {
  // clean up heap elements in open list
  open_list_->Clear();

  // clear closed list
  closed_list_.clear();
  closed_list_.resize(max_grid_x_ * max_grid_y_,
                      common::Node::NodeStatus::kOpen);

  start_node_ = nullptr;
  end_node_ = nullptr;
}

bool VoronoiPlanner::IsWithinMap(const int grid_x, const int grid_y) const {
  return grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 &&
         grid_y < max_grid_y_;
}

bool VoronoiPlanner::IsValidCell(const int grid_x, const int grid_y) const {
  if (!IsWithinMap(grid_x, grid_y)) {
    return false;
  }
  if (gvd_map_[grid_x][grid_y].dist_in_world < circumscribed_radius_) {
    return false;
  }
  return true;
}

int VoronoiPlanner::CalcGridXYIndex(const int grid_x, const int grid_y) const {
  DCHECK(IsWithinMap(grid_x, grid_y));
  return grid_x + grid_y * max_grid_x_;
}

int VoronoiPlanner::GetKey(const Node2d* node) const {
  CHECK_NOTNULL(node);
  return search_type_ == SearchType::kAStar ? node->g() + node->h() : node->g();
}

int VoronoiPlanner::GetActionCost(int curr_x, int curr_y, int action_id) const {
  CHECK(IsValidCell(curr_x, curr_y));
  const int succ_x = curr_x + actions_.dx[action_id];
  const int succ_y = curr_y + actions_.dy[action_id];
  CHECK(IsValidCell(succ_x, succ_y));
  return actions_.dxy_cost[action_id];
}

void VoronoiPlanner::ComputeGridSearchActions() {
  // initialize some constants for 2D search
  actions_.dx[0] = 1;
  actions_.dy[0] = 1;
  actions_.dx[1] = 1;
  actions_.dy[1] = 0;
  actions_.dx[2] = 1;
  actions_.dy[2] = -1;
  actions_.dx[3] = 0;
  actions_.dy[3] = 1;
  actions_.dx[4] = 0;
  actions_.dy[4] = -1;
  actions_.dx[5] = -1;
  actions_.dy[5] = 1;
  actions_.dx[6] = -1;
  actions_.dy[6] = 0;
  actions_.dx[7] = -1;
  actions_.dy[7] = -1;

  // compute distances
  for (int dind = 0; dind < common::kNumOfGridSearchActions; dind++) {
    if (actions_.dx[dind] != 0 && actions_.dy[dind] != 0) {
      actions_.dxy_cost[dind] = 14;
    } else {
      actions_.dxy_cost[dind] = 10;
    }
  }
}

bool VoronoiPlanner::SetStart(const int start_x, const int start_y) {
  if (!IsValidCell(start_x, start_y)) {
    return false;
  }
  start_node_ = GetNode(start_x, start_y);
  return true;
}

bool VoronoiPlanner::SetEnd(const int end_x, const int end_y) {
  if (!IsValidCell(end_x, end_y)) {
    return false;
  }
  end_node_ = GetNode(end_x, end_y);
  return true;
}

bool VoronoiPlanner::SetStartAndEndConfiguration(int sx, int sy, int ex,
                                                 int ey) {
  if (!SetStart(sx, sy)) {
    LOG(ERROR) << "VoronoiPlanner is called on invalid start (" << sx << ","
               << sy << ")";
    return false;
  }
  CHECK_NOTNULL(start_node_);
  // since the goal has not been set yet, the start node's h value is set to 0
  CHECK_EQ(start_node_->h(), 0);

  if (!SetEnd(ex, ey)) {
    LOG(ERROR) << "VoronoiPlanner is called on invalid end (" << ex << "," << ey
               << ")";
    return false;
  }
  CHECK_NOTNULL(end_node_);
  CHECK_EQ(end_node_->h(), 0);
  return true;
}

Node2d* VoronoiPlanner::GetNode(const int grid_x, const int grid_y) {
  DCHECK(IsWithinMap(grid_x, grid_y));
  Node2d* node = &dp_lookup_table_[grid_x][grid_y];
  if (node->iterations() != iterations_) {
    node->set_h(CalcHeuCost(grid_x, grid_y));
    node->set_g(common::kInfiniteCost);
    node->set_pre_node(nullptr);
    node->set_heap_index(0);
    node->set_iterations(iterations_);
  }
  return node;
}

void VoronoiPlanner::UpdateSuccs(const Node2d* curr_node) {
  CHECK_NOTNULL(curr_node);
  const int curr_x = curr_node->grid_x();
  const int curr_y = curr_node->grid_y();
  CHECK(CheckVoronoi(curr_x, curr_y));
  CHECK(IsValidCell(curr_x, curr_y));

  for (int action_id = 0; action_id < common::kNumOfGridSearchActions;
       ++action_id) {
    const int succ_x = curr_x + actions_.dx[action_id];
    const int succ_y = curr_y + actions_.dy[action_id];
    if (!IsValidCell(succ_x, succ_y)) {
      continue;
    }
    if (closed_list_[CalcGridXYIndex(succ_x, succ_y)] ==
        common::Node::NodeStatus::kClosed) {
      continue;
    }
    if (!CheckVoronoi(succ_x, succ_y)) {
      continue;
    }
    // get action cost
    int action_cost = GetActionCost(curr_x, curr_y, action_id);

    Node2d* succ_node = GetNode(succ_x, succ_y);
    // see if we can decrease the value of successive node taking into account
    // the cost of action
    if (succ_node->g() > curr_node->g() + action_cost) {
      succ_node->set_g(curr_node->g() + action_cost);
      succ_node->set_pre_node(curr_node);

      // re-insert into heap if not closed yet
      if (succ_node->heap_index() == 0) {
        open_list_->Insert(succ_node, GetKey(succ_node));
      } else {
        open_list_->Update(succ_node, GetKey(succ_node));
      }
    }
  }
}

int VoronoiPlanner::CalcHeuCost(const int grid_x, const int grid_y) const {
  if (end_node_ == nullptr) {
    return 0;
  }
  return 10 * std::max(std::abs(grid_x - end_node_->grid_x()),
                       std::abs(grid_y - end_node_->grid_y()));
}

void VoronoiPlanner::LoadVoronoiSearchResult(
    int end_x, int end_y, VoronoiSearchResult* result) const {
  if (result == nullptr) {
    return;
  }

  result->path_cost = dp_lookup_table_[end_x][end_y].g();
  const Node2d* node = &dp_lookup_table_[end_x][end_y];
  std::vector<int> grid_x;
  std::vector<int> grid_y;
  while (node != nullptr) {
    grid_x.push_back(node->grid_x());
    grid_y.push_back(node->grid_y());
    node = node->pre_node();
  }
  std::reverse(grid_x.begin(), grid_x.end());
  std::reverse(grid_y.begin(), grid_y.end());
  result->x = std::move(grid_x);
  result->y = std::move(grid_y);
}

bool VoronoiPlanner::CheckVoronoi(int grid_x, int grid_y) const {
  if (!need_check_voronoi_) {
    return true;
  }
  return gvd_map_[grid_x][grid_y].is_voronoi;
}

bool VoronoiPlanner::Search(
    int sx, int sy, int ex, int ey,
    const std::vector<std::vector<VoronoiData>>& gvd_map,
    VoronoiSearchResult* result) {
  if (!initialized_) {
    LOG(ERROR) << "VoronoiPlanner has not been initialized.";
    return false;
  }

  // Sanity checks.
  CHECK_NOTNULL(result);

  const auto start_timestamp = std::chrono::system_clock::now();

  gvd_map_ = gvd_map;

  // Find grid path from start to Voronoi edges.
  int voronoi_start_x = 0;
  int voronoi_start_y = 0;
  VoronoiSearchResult start_to_voronoi;
  if (!SearchPathToVoronoiEdges(sx, sy, ex, ey, &voronoi_start_x,
                                &voronoi_start_y, &start_to_voronoi)) {
    LOG(ERROR) << "Failed to find grid path from start to Voronoi edges.";
    return false;
  }

  if (voronoi_start_x == ex && voronoi_start_y == ey) {
    *result = start_to_voronoi;
    return true;
  }

  // Find grid path from end to Voronoi edges.
  int voronoi_end_x = 0;
  int voronoi_end_y = 0;
  VoronoiSearchResult voronoi_to_end;
  if (!SearchPathToVoronoiEdges(ex, ey, sx, sy, &voronoi_end_x, &voronoi_end_y,
                                &voronoi_to_end)) {
    LOG(ERROR) << "Failed to find grid path from end to Voronoi edges.";
    return false;
  }
  std::reverse(voronoi_to_end.x.begin(), voronoi_to_end.x.end());
  std::reverse(voronoi_to_end.y.begin(), voronoi_to_end.y.end());

  if (voronoi_end_x == sx && voronoi_end_y == sy) {
    *result = voronoi_to_end;
    return true;
  }

  // Find grid path along Voronoi edges.
  VoronoiSearchResult along_voronoi;
  if (!SearchPathAlongVoronoiEdges(voronoi_start_x, voronoi_start_y,
                                   voronoi_end_x, voronoi_end_y,
                                   &along_voronoi)) {
    LOG(ERROR) << "Failed to find grid path along Voronoi edges.";
    return false;
  }

  // Stitch the three grid path results together.
  StitchSearchResult(start_to_voronoi, along_voronoi, voronoi_to_end, result);

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  LOG(INFO) << std::fixed << "Total time used to search Voronoi grid path: "
            << diff.count() * 1e3 << " ms.";

  return true;
}

bool VoronoiPlanner::SearchPathToVoronoiEdges(int sx, int sy, int ex, int ey,
                                              int* voronoi_end_x,
                                              int* voronoi_end_y,
                                              VoronoiSearchResult* result) {
  const auto start_timestamp = std::chrono::system_clock::now();

  // clean up previous planning result
  Clear();

  search_type_ = SearchType::kDP;
  need_check_voronoi_ = false;

  ++iterations_;

  if (!SetStartAndEndConfiguration(sx, sy, ex, ey)) {
    return false;
  }

  // initialize start node and insert it into heap
  start_node_->set_g(0);
  start_node_->set_h(CalcHeuCost(sx, sy));
  open_list_->Insert(start_node_, GetKey(start_node_));

  // grid search begins
  std::size_t explored_node_num = 0U;
  while (!open_list_->Empty()) {
    auto* node = dynamic_cast<Node2d*>(open_list_->Pop());
    CHECK_NOTNULL(node);
    CHECK_NE(node->g(), common::kInfiniteCost);
    closed_list_[CalcGridXYIndex(node->grid_x(), node->grid_y())] =
        common::Node::NodeStatus::kClosed;

    int curr_x = node->grid_x();
    int curr_y = node->grid_y();

    if ((curr_x == ex && curr_y == ey) ||
        (gvd_map_[curr_x][curr_y].is_voronoi)) {
      *voronoi_end_x = curr_x;
      *voronoi_end_y = curr_y;
      break;
    }

    // new expand
    ++explored_node_num;
    UpdateSuccs(node);
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  LOG(INFO) << std::fixed << "Time used to search path to Voronoi edges: "
            << diff.count() * 1e3 << " ms.";

  LoadVoronoiSearchResult(*voronoi_end_x, *voronoi_end_y, result);
  return true;
}

bool VoronoiPlanner::SearchPathAlongVoronoiEdges(int sx, int sy, int ex, int ey,
                                                 VoronoiSearchResult* result) {
  const auto start_timestamp = std::chrono::system_clock::now();

  // clean up previous planning result
  Clear();

  search_type_ = SearchType::kAStar;
  need_check_voronoi_ = true;

  ++iterations_;

  if (!SetStartAndEndConfiguration(sx, sy, ex, ey)) {
    return false;
  }

  // initialize start node and insert it into heap
  start_node_->set_g(0);
  start_node_->set_h(CalcHeuCost(sx, sy));
  open_list_->Insert(start_node_, GetKey(start_node_));

  // grid search begins
  std::size_t explored_node_num = 0U;
  while (!open_list_->Empty() && end_node_->g() > open_list_->GetMinKey()) {
    auto* node = dynamic_cast<Node2d*>(open_list_->Pop());
    CHECK_NOTNULL(node);
    CHECK_NE(node->g(), common::kInfiniteCost);
    closed_list_[CalcGridXYIndex(node->grid_x(), node->grid_y())] =
        common::Node::NodeStatus::kClosed;

    // new expand
    ++explored_node_num;
    UpdateSuccs(node);
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  LOG(INFO) << std::fixed << "Time used to search path along Voronoi edges: "
            << diff.count() * 1e3 << " ms.";

  if (end_node_->g() == common::kInfiniteCost) {
    LOG(ERROR) << "VoronoiPlanner returns infinite cost (open_list run out)";
    return false;
  }

  LoadVoronoiSearchResult(end_node_->grid_x(), end_node_->grid_y(), result);
  return true;
}

void VoronoiPlanner::StitchSearchResult(
    const VoronoiSearchResult& start_to_voronoi,
    const VoronoiSearchResult& along_voronoi,
    const VoronoiSearchResult& voronoi_to_end, VoronoiSearchResult* result) {
  // Sanity checks.
  CHECK_NOTNULL(result);
  // Stitch x results.
  result->x.clear();
  result->x.insert(result->x.end(), start_to_voronoi.x.begin(),
                   start_to_voronoi.x.end());
  result->x.insert(result->x.end(), along_voronoi.x.begin(),
                   along_voronoi.x.end());
  result->x.insert(result->x.end(), voronoi_to_end.x.begin(),
                   voronoi_to_end.x.end());
  // Stitch y results.
  result->y.clear();
  result->y.insert(result->y.end(), start_to_voronoi.y.begin(),
                   start_to_voronoi.y.end());
  result->y.insert(result->y.end(), along_voronoi.y.begin(),
                   along_voronoi.y.end());
  result->y.insert(result->y.end(), voronoi_to_end.y.begin(),
                   voronoi_to_end.y.end());
  result->path_cost = start_to_voronoi.path_cost + along_voronoi.path_cost +
                      voronoi_to_end.path_cost;
}

}  // namespace voronoi_planner
