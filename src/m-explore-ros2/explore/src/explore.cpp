/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include <thread>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

inline static bool snap_to_nearest_free(nav2_costmap_2d::Costmap2D* cm,
                                       const geometry_msgs::msg::Point& in,
                                       geometry_msgs::msg::Point& out,
                                       int max_radius_cells)
{
  if (!cm) {
    return false;
  }

  unsigned int mx, my;
  if (!cm->worldToMap(in.x, in.y, mx, my)) {
    return false;
  }

  const int w = static_cast<int>(cm->getSizeInCellsX());
  const int h = static_cast<int>(cm->getSizeInCellsY());

  auto inBounds = [&](int x, int y) { return x >= 0 && y >= 0 && x < w && y < h; };

  // square "growing ring" search; return the first FREE cell found
  for (int r = 0; r <= max_radius_cells; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        int x = static_cast<int>(mx) + dx;
        int y = static_cast<int>(my) + dy;
        if (!inBounds(x, y)) {
          continue;
        }
        unsigned char c = cm->getCost(static_cast<unsigned int>(x),
                                      static_cast<unsigned int>(y));
        if (c == nav2_costmap_2d::FREE_SPACE) {
          double wx, wy;
          cm->mapToWorld(static_cast<unsigned int>(x),
                         static_cast<unsigned int>(y), wx, wy);
          out = in;
          out.x = wx;
          out.y = wy;
          return true;
        }
      }
    }
  }
  return false;
}


namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);


  // extra tuning knobs
  this->declare_parameter<int>("tiny_frontier_cells", 3);
  this->declare_parameter<int>("tiny_frontier_patience", 5);
  this->declare_parameter<int>("snap_search_radius_cells", 10);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);


  this->get_parameter("tiny_frontier_cells", tiny_frontier_cells_);
  this->get_parameter("tiny_frontier_patience", tiny_frontier_patience_);
  this->get_parameter("snap_search_radius_cells", snap_search_radius_cells_);

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // Start exploration right away
  makePlan();
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
#ifdef ELOQUENT
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#elif DASHING
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#else
  m.lifetime = rclcpp::Duration::from_seconds(0);  // foxy onwards
#endif
  // m.lifetime = rclcpp::Duration::from_nanoseconds(0); // suggested in
  // galactic
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    // m.pose.position = {}; // compile warning
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  // 有 active goal 就不要重选 frontier
  // --- NEW: if navigating to an active goal, only do timeout checking ---
  if (has_active_goal_ && !resuming_) {
    if ((this->now() - last_progress_) > tf2::durationFromSec(progress_timeout_)) {
      frontier_blacklist_.push_back(active_goal_key_);
      RCLCPP_WARN(logger_,
                  "No progress for %.1fs (by nav2 feedback), blacklist active goal and retry later.",
                  progress_timeout_);

      move_base_client_->async_cancel_all_goals();
      has_active_goal_ = false;
    }
    return;  // don't replan while a goal is active
  }

  // 1) get current pose and frontiers
  auto pose = costmap_client_.getRobotPose();
  auto frontiers = search_.searchFrom(pose.position);

  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    stop(true);
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // 2) choose a frontier:
  //    - not blacklisted (by centroid)
  //    - not "tiny" (frontier.size is number of cells)
  auto it = frontiers.end();
  bool has_non_blacklisted = false;
  bool has_only_tiny = false;

  for (auto f_it = frontiers.begin(); f_it != frontiers.end(); ++f_it) {
    if (goalOnBlacklist(f_it->centroid)) {
      continue;
    }
    has_non_blacklisted = true;

    if (static_cast<int>(f_it->size) < tiny_frontier_cells_) {
      has_only_tiny = true;
      continue;
    }

    it = f_it;
    break;
  }

  if (it == frontiers.end()) {
    if (!has_non_blacklisted) {
      RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
      stop(true);
      return;
    }

    // Only tiny frontiers remain: allow a few cycles in case mapping/TF changes
    tiny_frontier_counter_++;
    RCLCPP_WARN(logger_,
                "Only tiny frontiers remain (size < %d cells). "
                "patience=%d/%d",
                tiny_frontier_cells_, tiny_frontier_counter_, tiny_frontier_patience_);

    if (tiny_frontier_counter_ >= tiny_frontier_patience_) {
      RCLCPP_WARN(logger_, "Tiny frontiers persist, stopping exploration.");
      stop(true);
    }
    return;
  }

  // got a valid frontier
  tiny_frontier_counter_ = 0;
  const auto& frontier = *it;

  // 3) create a "goal key" for progress + blacklist (centroid), and a real target for nav2 (snapped)
  geometry_msgs::msg::Point goal_key = frontier.centroid;   // stable identity
  geometry_msgs::msg::Point target_position = frontier.centroid;

  nav2_costmap_2d::Costmap2D* cm = costmap_client_.getCostmap();
  geometry_msgs::msg::Point snapped;

  if (snap_to_nearest_free(cm, frontier.centroid, snapped, snap_search_radius_cells_)) {
    target_position = snapped;
  } else if (snap_to_nearest_free(cm, frontier.middle, snapped, snap_search_radius_cells_)) {
    target_position = snapped;
  }

  // 4) progress / timeout logic uses the goal_key (NOT snapped), otherwise the blacklist never matches
  bool same_goal = same_point(prev_goal_key_, goal_key);
  prev_goal_key_ = goal_key;

  // 删掉了用 frontier.min_distance 的 progress 判定
  // if (!same_goal || prev_distance_ > frontier.min_distance) {
  //   last_progress_ = this->now();
  //   prev_distance_ = frontier.min_distance;
  // }

  // if ((this->now() - last_progress_ > tf2::durationFromSec(progress_timeout_)) && !resuming_) {
  //   frontier_blacklist_.push_back(goal_key);
  //   RCLCPP_WARN(logger_, "No progress for %.1fs, blacklist this frontier and retry later.",
  //               progress_timeout_);
  //   // DO NOT recurse into makePlan(): let the timer call again to avoid stack growth.
  //   return;
  // }

  if (resuming_) {
    resuming_ = false;
  }

  // still pursuing the same goal (key) -> do nothing
  if (same_goal) {
    return;
  }

  // 5) send goal to nav2
  // RCLCPP_DEBUG(logger_, "Sending goal to move base nav2 (target=%.3f,%.3f key=%.3f,%.3f size=%u dist=%.3f)",
  //              target_position.x, target_position.y, goal_key.x, goal_key.y,
  //              frontier.size, frontier.min_distance);

  // auto goal = nav2_msgs::action::NavigateToPose::Goal();
  // goal.pose.pose.position = target_position;
  // goal.pose.pose.orientation.w = 1.0;
  // goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  // goal.pose.header.stamp = this->now();

  // auto send_goal_options =
  //     rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  // // pass goal_key (centroid) for blacklisting on aborted
  // send_goal_options.result_callback =
  //     [this, goal_key](const NavigationGoalHandle::WrappedResult& result) {
  //       reachedGoal(result, goal_key);
  //     };

  // move_base_client_->async_send_goal(goal, send_goal_options);

  // 5) send goal to nav2
  RCLCPP_DEBUG(logger_,
               "Sending goal to nav2 (target=%.3f,%.3f key=%.3f,%.3f size=%u)",
               target_position.x, target_position.y,
               goal_key.x, goal_key.y, frontier.size);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  goal.pose.pose.orientation.w = 1.0;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  // --- NEW: lock active goal + progress based on nav2 feedback ---
  active_goal_key_ = goal_key;
  active_target_position_ = target_position;
  has_active_goal_ = true;
  last_feedback_distance_remaining_ = std::numeric_limits<double>::infinity();
  last_progress_ = this->now();  // reset progress timer on new goal

  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](std::shared_ptr<NavigationGoalHandle> handle) {
      if (!handle) {
        RCLCPP_WARN(logger_, "NavigateToPose goal was rejected by server");
        has_active_goal_ = false;
      } else {
        RCLCPP_DEBUG(logger_, "NavigateToPose goal accepted");
      }
    };

  send_goal_options.feedback_callback =
    [this](NavigationGoalHandle::SharedPtr,
           const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
      if (!feedback) { return; }

      const double d = feedback->distance_remaining;
      if (!std::isfinite(d)) { return; }

      // “有进展”的判据：剩余距离下降了一点点（防抖）
      constexpr double EPS = 0.05;  // 5cm
      if (d + EPS < last_feedback_distance_remaining_) {
        last_feedback_distance_remaining_ = d;
        last_progress_ = this->now();
        RCLCPP_DEBUG(logger_, "progress: distance_remaining=%.3f", d);
      }
    };

  // pass goal_key (centroid) for blacklisting on aborted
  send_goal_options.result_callback =
    [this, goal_key](const NavigationGoalHandle::WrappedResult& result) {
      reachedGoal(result, goal_key);
    };

  move_base_client_->async_send_goal(goal, send_goal_options);


}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  move_base_client_->async_send_goal(goal, send_goal_options);
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  has_active_goal_ = false;  // NEW: goal finished (any result)
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_DEBUG(logger_, "Goal was aborted");
      frontier_blacklist_.push_back(frontier_goal);
      RCLCPP_DEBUG(logger_, "Adding current goal to black list");
      // If it was aborted probably because we've found another frontier goal,
      // so just return and don't make plan again
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");
  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
