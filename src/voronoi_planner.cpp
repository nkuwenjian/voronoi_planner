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

#include "voronoi_planner/util.h"

namespace voronoi_planner {

VoronoiPlanner::VoronoiPlanner() {
  // compute dx, dy, dxintersects and dyintersects arrays
  Computedxy();
}

VoronoiPlanner::~VoronoiPlanner() { Clear(); }

void VoronoiPlanner::Clear() {
  if (open_list_ != nullptr) {
    open_list_->Clear();
  }

  for (int y = 0; y < size_y_; y++) {
    for (int x = 0; x < size_x_; x++) {
      delete search_space_[x][y];
    }
  }
}

void VoronoiPlanner::InitializeSearchState(Grid2DSearchState* search_state) {
  search_state->set_g(INFINITECOST);
  search_state->set_index(0);
  search_state->set_predecessor(nullptr);
}

void VoronoiPlanner::CreateSearchSpace() {
  search_space_.clear();
  search_space_.resize(size_x_);
  for (int x = 0; x < size_x_; x++) {
    search_space_[x].resize(size_y_);
    for (int y = 0; y < size_y_; y++) {
      search_space_[x][y] = new Grid2DSearchState(x, y);
      InitializeSearchState(search_space_[x][y]);
    }
  }
}

void VoronoiPlanner::ReInitializeSearchSpace() {
  for (int x = 0; x < size_x_; x++) {
    for (int y = 0; y < size_y_; y++) {
      InitializeSearchState(search_space_[x][y]);
    }
  }
}

void VoronoiPlanner::Computedxy() {
  // initialize some constants for 2D search
  dx_[0] = 1;
  dy_[0] = 1;
  dx_[1] = 1;
  dy_[1] = 0;
  dx_[2] = 1;
  dy_[2] = -1;
  dx_[3] = 0;
  dy_[3] = 1;
  dx_[4] = 0;
  dy_[4] = -1;
  dx_[5] = -1;
  dy_[5] = 1;
  dx_[6] = -1;
  dy_[6] = 0;
  dx_[7] = -1;
  dy_[7] = -1;

  // compute distances
  for (int dind = 0; dind < 8; dind++) {
    if (dx_[dind] != 0 && dy_[dind] != 0) {
      dxy_cost_[dind] = 14;
    } else {
      dxy_cost_[dind] = 10;
    }
  }
}

bool VoronoiPlanner::Search(int start_x, int start_y, int goal_x, int goal_y,
                            int* path_cost,
                            std::vector<std::pair<int, int>>* path, int size_x,
                            int size_y, VoronoiData** voronoi_diagram,
                            double circumscribed_radius) {
  if (size_x != size_x_ || size_y != size_y_) {
    Clear();

    size_x_ = size_x;
    size_y_ = size_y;

    if (open_list_ != nullptr) {
      open_list_.reset();
    }
    open_list_ = std::make_unique<Heap>(size_x_ * size_y_);
    CreateSearchSpace();
  }

  std::vector<std::pair<int, int>> path1, path2, path3;
  int path_cost1, path_cost2, path_cost3;
  int voronoi_start_x, voronoi_start_y;
  if (!SearchShortestPathToVoronoi(
          start_x, start_y, goal_x, goal_y, &voronoi_start_x, &voronoi_start_y,
          &path_cost1, &path1, voronoi_diagram, circumscribed_radius)) {
    LOG(ERROR) << "Failed to fine the path from start to voronoi";
    return false;
  }
  std::reverse(path1.begin(), path1.end());

  if (voronoi_start_x == goal_x && voronoi_start_y == goal_y) {
    path->clear();
    *path = path1;
    *path_cost = path_cost1;
    return true;
  }

  int voronoi_goal_x, voronoi_goal_y;
  if (!SearchShortestPathToVoronoi(
          goal_x, goal_y, start_x, start_y, &voronoi_goal_x, &voronoi_goal_y,
          &path_cost3, &path3, voronoi_diagram, circumscribed_radius)) {
    LOG(ERROR) << "Failed to the find the path from voronoi to goal";
    return false;
  }

  if (voronoi_goal_x == start_x && voronoi_goal_y == start_y) {
    path->clear();
    *path = path3;
    *path_cost = path_cost3;
    return true;
  }

  if (!SearchInVoronoi(voronoi_start_x, voronoi_start_y, voronoi_goal_x,
                       voronoi_goal_y, &path_cost2, &path2, voronoi_diagram,
                       circumscribed_radius)) {
    LOG(ERROR) << "Failed to find the path from voronoi start to voronoi goal";
    return false;
  }

  path1.insert(path1.end(), path2.begin(), path2.end());
  path1.insert(path1.end(), path3.begin(), path3.end());

  path->clear();
  *path = path1;
  *path_cost = path_cost1 + path_cost2 + path_cost3;
  return true;
}

bool VoronoiPlanner::SearchInVoronoi(int start_x, int start_y, int goal_x,
                                     int goal_y, int* path_cost,
                                     std::vector<std::pair<int, int>>* path,
                                     VoronoiData** voronoi_diagram,
                                     double circumscribed_radius) {
  UNUSED(path_cost);
  const auto start_timestamp = std::chrono::system_clock::now();

  ReInitializeSearchSpace();
  open_list_->Clear();

  Grid2DSearchState* search_exp_state = nullptr;
  Grid2DSearchState* search_succ_state = nullptr;
  Grid2DSearchState* search_goal_state = search_space_[goal_x][goal_y];
  int key;

  search_exp_state = search_space_[start_x][start_y];
  search_exp_state->set_g(0);
  key = search_exp_state->g();
  if (use_heuristic_) {
    key = key + Heuristic(start_x, start_y, goal_x, goal_y);
  }

  open_list_->Insert(search_exp_state, key);

  int num_of_expands = 0;
  std::vector<unsigned char> closed_list(size_x_ * size_y_, 0);

  while (!open_list_->Empty() &&
         search_goal_state->g() > open_list_->GetMinKey()) {
    search_exp_state = dynamic_cast<Grid2DSearchState*>(open_list_->Pop());
    CHECK_NOTNULL(search_exp_state);

    int exp_x = search_exp_state->x();
    int exp_y = search_exp_state->y();
    num_of_expands++;
    // close the state
    closed_list[exp_x + size_x_ * exp_y] = 1;

    for (int dir = 0; dir < 8; dir++) {
      int new_x = exp_x + dx_[dir];
      int new_y = exp_y + dy_[dir];

      if (!WithinSearchSpace(new_x, new_y)) {
        continue;
      }

      if (closed_list[new_x + size_x_ * new_y] == 1) {
        continue;
      }

      if (!voronoi_diagram[new_x][new_y].is_voronoi) {
        continue;
      }

      double dist = voronoi_diagram[new_x][new_y].dist;
      if (dist < circumscribed_radius) {
        continue;
      }

      int cost = dxy_cost_[dir];

      search_succ_state = search_space_[new_x][new_y];
      if (search_succ_state->g() > search_exp_state->g() + cost) {
        search_succ_state->set_g(search_exp_state->g() + cost);
        search_succ_state->set_predecessor(search_exp_state);

        key = search_succ_state->g();
        if (use_heuristic_) {
          key = key + Heuristic(new_x, new_y, goal_x, goal_y);
        }
        if (search_succ_state->index() == 0) {
          open_list_->Insert(search_succ_state, key);
        } else {
          open_list_->Update(search_succ_state, key);
        }
      }
    }  // over successors
  }    // while

  // set lower bounds for the remaining states
  if (!open_list_->Empty()) {
    largestcomputedoptf_ = open_list_->GetMinKey();
  } else {
    largestcomputedoptf_ = INFINITECOST;
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  LOG(INFO) << "Time used to search voronoi path: " << diff.count() * 1e3
            << " ms.";

  path->clear();
  const Grid2DSearchState* search_pred_state = search_goal_state;
  path->emplace_back(search_pred_state->x(), search_pred_state->y());
  while (search_pred_state->predecessor() != nullptr) {
    search_pred_state = search_pred_state->predecessor();
    path->emplace_back(search_pred_state->x(), search_pred_state->y());
  }
  std::reverse(path->begin(), path->end());

  return true;
}

bool VoronoiPlanner::SearchShortestPathToVoronoi(
    int start_x, int start_y, int goal_x, int goal_y, int* voronoi_goal_x,
    int* voronoi_goal_y, int* path_cost, std::vector<std::pair<int, int>>* path,
    VoronoiData** voronoi_diagram, double circumscribed_radius) {
  UNUSED(path_cost);
  if (!WithinSearchSpace(start_x, start_y)) {
    throw std::runtime_error("the start is out of range");
    return false;
  }

  const auto start_timestamp = std::chrono::system_clock::now();

  ReInitializeSearchSpace();
  open_list_->Clear();

  Grid2DSearchState* search_exp_state = nullptr;
  Grid2DSearchState* search_succ_state = nullptr;

  search_exp_state = search_space_[start_x][start_y];
  search_exp_state->set_g(0);
  open_list_->Insert(search_exp_state, search_exp_state->g());

  int num_of_expands = 0;
  std::vector<unsigned char> closed_list(size_x_ * size_y_, 0);

  while (!open_list_->Empty()) {
    search_exp_state = dynamic_cast<Grid2DSearchState*>(open_list_->Pop());
    CHECK_NOTNULL(search_exp_state);

    int exp_x = search_exp_state->x();
    int exp_y = search_exp_state->y();

    if ((exp_x == goal_x && exp_y == goal_y) ||
        (voronoi_diagram[exp_x][exp_y].is_voronoi)) {
      *voronoi_goal_x = exp_x;
      *voronoi_goal_y = exp_y;
      break;
    }

    num_of_expands++;
    // close the state
    closed_list[exp_x + size_x_ * exp_y] = 1;

    for (int dir = 0; dir < 8; dir++) {
      int new_x = exp_x + dx_[dir];
      int new_y = exp_y + dy_[dir];

      if (!WithinSearchSpace(new_x, new_y)) {
        continue;
      }

      if (closed_list[new_x + size_x_ * new_y] == 1) {
        continue;
      }

      double dist = voronoi_diagram[new_x][new_y].dist;
      if (dist < circumscribed_radius) {
        continue;
      }

      int cost = dxy_cost_[dir];

      search_succ_state = search_space_[new_x][new_y];
      if (search_succ_state->g() > search_exp_state->g() + cost) {
        search_succ_state->set_g(search_exp_state->g() + cost);
        search_succ_state->set_predecessor(search_exp_state);

        if (search_succ_state->index() == 0) {
          open_list_->Insert(search_succ_state, search_succ_state->g());
        } else {
          open_list_->Update(search_succ_state, search_succ_state->g());
        }
      }
    }  // over successors
  }    // while

  // set lower bounds for the remaining states
  if (!open_list_->Empty()) {
    largestcomputedoptf_ = open_list_->GetMinKey();
  } else {
    largestcomputedoptf_ = INFINITECOST;
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;

  path->clear();
  Grid2DSearchState* search_goal_state =
      search_space_[*voronoi_goal_x][*voronoi_goal_y];
  const Grid2DSearchState* search_pred_state = search_goal_state;
  path->emplace_back(search_pred_state->x(), search_pred_state->y());
  while (search_pred_state->predecessor() != nullptr) {
    search_pred_state = search_pred_state->predecessor();
    path->emplace_back(search_pred_state->x(), search_pred_state->y());
  }

  return true;
}

}  // namespace voronoi_planner
