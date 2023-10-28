/******************************************************************************
 * Copyright (c) 2023, NKU Mobile & Flying Robotics Lab
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
 *
 * Author: Jian Wen (nkuwenjian@gmail.com)
 *****************************************************************************/

#include "voronoi_planner/voronoi_planner_ros.h"

#include <memory>
#include <mutex>  // NOLINT

#include "costmap_2d/costmap_2d_ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "glog/logging.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf2_ros/transform_listener.h"

namespace voronoi_planner {

class VoronoiPlannerROSTest {
 public:
  explicit VoronoiPlannerROSTest(tf2_ros::Buffer& tf);  // NOLINT
  virtual ~VoronoiPlannerROSTest() = default;

  void Initialize();

 private:
  void SetStart(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);
  void SetGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);
  void MakePlan();

  ros::NodeHandle nh_;
  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;

  geometry_msgs::PoseStamped start_;
  geometry_msgs::PoseStamped goal_;
  bool start_received_ = false;
  bool goal_received_ = false;
  tf2_ros::Buffer& tf_;
  std::mutex start_mutex_;
  std::mutex goal_mutex_;

  std::unique_ptr<VoronoiPlannerROS> planner_ = nullptr;
  std::unique_ptr<costmap_2d::Costmap2DROS> costmap_ros_ = nullptr;
};

VoronoiPlannerROSTest::VoronoiPlannerROSTest(tf2_ros::Buffer& tf) : tf_(tf) {}

void VoronoiPlannerROSTest::Initialize() {
  start_sub_ =
      nh_.subscribe("initialpose", 1, &VoronoiPlannerROSTest::SetStart, this);
  goal_sub_ = nh_.subscribe("move_base_simple/goal", 1,
                            &VoronoiPlannerROSTest::SetGoal, this);

  costmap_ros_ =
      std::make_unique<costmap_2d::Costmap2DROS>("global_costmap", tf_);
  costmap_ros_->pause();
  planner_ = std::make_unique<VoronoiPlannerROS>();
  planner_->initialize("VoronoiPlannerROS", costmap_ros_.get());

  // Start actively updating costmaps based on sensor data.
  costmap_ros_->start();
}

void VoronoiPlannerROSTest::SetStart(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start) {
  {
    std::lock_guard<std::mutex> start_lock(start_mutex_);
    LOG(INFO) << "A new start is received.";
    start_.header = start->header;
    start_.pose = start->pose.pose;
    start_received_ = true;
  }

  MakePlan();
}

void VoronoiPlannerROSTest::SetGoal(
    const geometry_msgs::PoseStamped::ConstPtr& goal) {
  {
    std::lock_guard<std::mutex> goal_lock(goal_mutex_);
    LOG(INFO) << "A new goal is received.";
    goal_ = *goal;
    goal_received_ = true;
  }

  MakePlan();
}

void VoronoiPlannerROSTest::MakePlan() {
  std::lock_guard<std::mutex> start_lock(start_mutex_);
  std::lock_guard<std::mutex> goal_lock(goal_mutex_);
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> map_lock(
      *(costmap_ros_->getCostmap()->getMutex()));

  if (!start_received_ || !goal_received_) {
    return;
  }

  std::vector<geometry_msgs::PoseStamped> plan;
  if (planner_->makePlan(start_, goal_, plan)) {
    LOG(INFO) << "Successfully find a grid path via Voronoi planner.";
  } else {
    LOG(ERROR) << "Failed to find a grid path via Voronoi planner.";
  }
}

}  // namespace voronoi_planner

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "voronoi_planner_test");
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  voronoi_planner::VoronoiPlannerROSTest voronoi_planner_test(buffer);
  voronoi_planner_test.Initialize();
  ros::spin();

  return 0;
}
