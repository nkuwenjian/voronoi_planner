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

#include <string>
#include <vector>

#include "costmap_2d/costmap_2d_ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_core/base_global_planner.h"
#include "ros/ros.h"
#include "voronoi_layer/voronoi_layer.h"

#include "voronoi_planner/voronoi_planner.h"

#define CONTXY2DISC(X, CELLSIZE)               \
  ((X >= 0) ? (static_cast<int>(X / CELLSIZE)) \
            : (static_cast<int>(X / CELLSIZE) - 1))
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)

namespace voronoi_planner {

class VoronoiPlannerROS : public nav_core::BaseGlobalPlanner {
 public:
  /**
   * @brief  Default constructor for the VoronoiPlannerROS object
   */
  VoronoiPlannerROS() = default;

  /**
   * @brief  Constructor for the VoronoiPlannerROS object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  VoronoiPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Initialization function for the VoronoiPlannerROS object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  void initialize(std::string name,
                  costmap_2d::Costmap2DROS* costmap_ros) override;

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

  virtual ~VoronoiPlannerROS();

 private:
  void publishVoronoiPath(const std::vector<geometry_msgs::PoseStamped>& plan);
  void destroy();

 private:
  VoronoiPlanner* voronoi_planner_ = nullptr;
  costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
  std::string name_;
  unsigned int last_size_x_ = 0;
  unsigned int last_size_y_ = 0;
  VoronoiData** voronoi_diagram_ = nullptr;

  ros::Publisher path_pub_;

  bool initialized_ = false;
};

}  // namespace voronoi_planner
