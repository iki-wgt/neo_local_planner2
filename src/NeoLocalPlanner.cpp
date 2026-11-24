/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Neobotix GmbH
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
 *   * Neither the name of the Neobotix nor the names of its
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
 *********************************************************************/

#include "NeoLocalPlanner.hpp"

#include <chrono>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <thread>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/line_iterator.hpp"
#include "nav2_core/goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace neo_local_planner2
{

namespace
{
void requireFinite(std::initializer_list<double> values, const char * context)
{
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw nav2_core::NoValidControl(std::string("Non-finite ") + context);
    }
  }
}

double finiteMin(double a, double b)
{
  requireFinite({a, b}, "minimum limiting operands");
  return std::min(a, b);
}

double finiteMax(double a, double b)
{
  requireFinite({a, b}, "maximum limiting operands");
  return std::max(a, b);
}

bool worldToMap(
  nav2_costmap_2d::Costmap2D * costmap, double x, double y,
  unsigned int & mx, unsigned int & my)
{
  const double resolution = costmap->getResolution();
  const double min_x = costmap->getOriginX();
  const double min_y = costmap->getOriginY();
  const double max_x = min_x + costmap->getSizeInCellsX() * resolution;
  const double max_y = min_y + costmap->getSizeInCellsY() * resolution;
  requireFinite({x, y, resolution, min_x, min_y, max_x, max_y}, "costmap geometry");
  if (resolution <= 0.0 || max_x <= min_x || max_y <= min_y) {
    throw nav2_core::NoValidControl("Costmap must have positive dimensions and resolution");
  }
  // Check bounds before Costmap2D converts floating point coordinates to integers.
  // Even finite coordinates can overflow that conversion far outside the map.
  return x >= min_x && x < max_x && y >= min_y && y < max_y &&
         costmap->worldToMap(x, y, mx, my);
}

void validatePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  requireFinite({p.x, p.y, p.z, q.x, q.y, q.z, q.w}, "pose");
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {
    throw nav2_core::NoValidControl("Pose quaternion must be normalized and nonzero");
  }
}

void validateTransform(const tf2::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  tf2::toMsg(transform, pose);
  validatePose(pose);
}

void validateVelocity(const geometry_msgs::msg::Twist & velocity)
{
  requireFinite({velocity.linear.x, velocity.linear.y, velocity.linear.z,
      velocity.angular.x, velocity.angular.y, velocity.angular.z}, "velocity");
}

void validatePath(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty() || path.header.frame_id.empty()) {
    throw nav2_core::InvalidPath("Path must have poses and a frame_id");
  }
  for (const auto & pose : path.poses) {
    if (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id) {
      throw nav2_core::InvalidPath("Path poses must use the path header frame_id");
    }
    try {
      validatePose(pose.pose);
    } catch (const nav2_core::NoValidControl & error) {
      throw nav2_core::InvalidPath(error.what());
    }
  }
}
}  // namespace

tf2::Quaternion createQuaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return q;
}

std::vector<tf2::Transform>::const_iterator find_closest_point(
  std::vector<tf2::Transform>::const_iterator begin,
  std::vector<tf2::Transform>::const_iterator end,
  const tf2::Vector3 & pos,
  double * actual_dist = 0)
{
  if (begin == end) {
    throw nav2_core::InvalidPath("Cannot search an empty local path");
  }
  auto iter_short = begin;
  double dist_short = std::numeric_limits<double>::infinity();

  for (auto iter = iter_short; iter != end; ++iter) {
    const double dist = (iter->getOrigin() - pos).length();
    requireFinite({dist}, "path distance");
    if (dist < dist_short) {
      dist_short = dist;
      iter_short = iter;
    }
  }
  if (actual_dist) {
    *actual_dist = dist_short;
  }
  return iter_short;
}

std::vector<tf2::Transform>::const_iterator move_along_path(
  std::vector<tf2::Transform>::const_iterator begin,
  std::vector<tf2::Transform>::const_iterator end,
  const double dist,
  double * actual_dist = 0)
{
  if (begin == end) {
    throw nav2_core::InvalidPath("No lookahead point in local path");
  }
  requireFinite({dist}, "lookahead distance");
  auto iter = begin;
  auto iter_prev = iter;
  double dist_left = dist;

  while (iter != end) {
    const double dist = (iter->getOrigin() - iter_prev->getOrigin()).length();
    requireFinite({dist}, "path segment length");
    dist_left -= dist;
    if (dist_left <= 0) {
      break;
    }
    iter_prev = iter;
    iter++;
  }

  if (iter == end) {
    iter = iter_prev;   // targeting final pose
  }

  if (actual_dist) {
    *actual_dist = dist - dist_left;
  }
  return iter;
}

std::vector<std::pair<int, int>> get_line_cells(
  nav2_costmap_2d::Costmap2D * cost_map,
  const tf2::Vector3 & world_pos_0,
  const tf2::Vector3 & world_pos_1)
{
  requireFinite({world_pos_0.x(), world_pos_0.y(), world_pos_1.x(), world_pos_1.y()},
    "costmap line coordinates");
  int coords[2][2] = {};
  cost_map->worldToMapEnforceBounds(world_pos_0.x(), world_pos_0.y(), coords[0][0], coords[0][1]);
  cost_map->worldToMapEnforceBounds(world_pos_1.x(), world_pos_1.y(), coords[1][0], coords[1][1]);

  // Creating a vector for storing the value of the cells

  std::vector<std::pair<int, int>> cells;

  // Line iterator for determining the cells between two points
  for (nav2_util::LineIterator
    line(
      coords[0][0], coords[0][1], coords[1][0], coords[1][1]);
    line.isValid();
    line.advance()
  )
  {
    cells.push_back(std::make_pair(line.getX(), line.getY()));
  }
  // Mandatory reversal
  // std::reverse( cells.begin(), cells.end() );

  return cells;
}

double get_cost(
  nav2_costmap_2d::Costmap2D * cost_map_,
  const tf2::Vector3 & world_pos)
{
  requireFinite({world_pos.x(), world_pos.y()}, "costmap coordinates");
  unsigned int x, y;
  if (!worldToMap(cost_map_, world_pos.x(), world_pos.y(), x, y)) {
    return 1.0;
  }
  return cost_map_->getCost(x, y) / 255.0;
}

double compute_avg_line_cost(
  nav2_costmap_2d::Costmap2D * cost_map_,
  const tf2::Vector3 & world_pos_0,
  const tf2::Vector3 & world_pos_1)
{
  if (get_cost(cost_map_, world_pos_0) == 1.0 || get_cost(cost_map_, world_pos_1) == 1.0) {
    return 1.0;
  }
  const std::vector<std::pair<int, int>> cells =
    get_line_cells(cost_map_, world_pos_0, world_pos_1);

  if (cells.empty()) {
    throw nav2_core::NoValidControl("Empty costmap line");
  }
  double avg_cost = 0;
  for (auto cell : cells) {
    avg_cost += static_cast<double>(cost_map_->getCost(cell.first, cell.second) / 255.);
  }

  return avg_cost / cells.size();
}

geometry_msgs::msg::TwistStamped NeoLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & position,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock_reinit(m_mutex);
  try {
    geometry_msgs::msg::Twist cmd_vel;
    geometry_msgs::msg::TwistStamped cmd_vel_final;
    validatePath(m_global_plan);
    validatePose(position.pose);
    validateVelocity(speed);
    validateVelocity(m_last_cmd_vel);
    requireFinite({m_last_control_values[0], m_last_control_values[1], m_last_control_values[2]},
      "stored control state");
    if (position.header.frame_id.empty()) {
      throw nav2_core::ControllerTFError("Robot pose frame_id is empty");
    }

    // A WallRate controller may run faster than /clock is published. Wait for
    // the next ROS timestamp instead of repeatedly resetting valid control state.
    // Steady time bounds this wait only; integration still uses ROS time.
    auto time_now = clock_->now();
    const auto wait_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(clock_wait_timeout);
    while (time_now == m_last_time && std::chrono::steady_clock::now() < wait_deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      time_now = clock_->now();
    }
    const double dt = (time_now - m_last_time).seconds();
    if (!std::isfinite(dt) || dt <= 0.0 || dt > max_control_dt) {
      throw nav2_core::NoValidControl("Control time delta " + std::to_string(dt) +
              " s must be in (0, max_control_dt]; ROS clock did not advance or jumped");
    }

    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getMutex());
    const auto global_to_local = lookupTransform(m_local_frame, m_global_plan.header.frame_id);
    std::vector<tf2::Transform> local_plan;
    local_plan.reserve(m_global_plan.poses.size());
    for (const auto & pose : m_global_plan.poses) {
      tf2::Transform path_pose;
      tf2::fromMsg(pose.pose, path_pose);
      local_plan.push_back(global_to_local * path_pose);
      validateTransform(local_plan.back());
    }
    if (local_plan.empty()) {
      throw nav2_core::InvalidPath("Transformed local path is empty");
    }
    requireFinite({m_robot_direction}, "robot direction");
    if (m_robot_direction != 1.0 && m_robot_direction != -1.0) {
      throw nav2_core::NoValidControl("Invalid robot direction state");
    }


    // get latest local pose
    tf2::Transform local_pose;
    tf2::fromMsg(position.pose, local_pose);
    local_pose = lookupTransform(m_local_frame, position.header.frame_id) * local_pose;
    validateTransform(local_pose);

    // Nav2 supplies the base pose. Use it for robot-relative path geometry
    // instead of a redundant base <- global TF lookup at a different common time.
    if (m_allow_reversing && count == 0) {
      const auto index = std::min<size_t>(5, local_plan.size() - 1);
      const auto robot_target = local_pose.inverse() * local_plan[index];
      validateTransform(robot_target);
      m_robot_direction = robot_target.getOrigin().x() >= 0.0 ? 1.0 : -1.0;
      count = 1;
    }

    const double start_yaw = tf2::getYaw(local_pose.getRotation());
    const double start_vel_x = speed.linear.x;
    const double start_vel_y = speed.linear.y;
    const double start_yawrate = speed.angular.z;

    double lookahead_dist = 0.0;
    double cost_y_lookahead_dist = 0.0;

    // reset last cmd vel if robot is not moving - avoids instant acceleration after EMStop
    if (start_vel_x == 0.0 && start_vel_y == 0.0 && start_yawrate == 0.0) {
      m_last_cmd_vel.linear.x = 0;
      m_last_cmd_vel.linear.y = 0;
      m_last_cmd_vel.angular.z = 0;
      std::fill(std::begin(m_last_control_values), std::end(m_last_control_values), 0.0);
    }

    // calc dynamic lookahead distances
    lookahead_dist = m_lookahead_dist + finiteMax(fabs(start_vel_x), 0) * lookahead_time;
    cost_y_lookahead_dist = m_cost_y_lookahead_dist + finiteMax(start_vel_x,
        0) * cost_y_lookahead_time;

    requireFinite({lookahead_dist, cost_y_lookahead_dist}, "dynamic lookahead");

    // predict future pose (using second order midpoint method)
    tf2::Vector3 actual_pos;
    double actual_yaw = 0;
    {
      const double midpoint_yaw = start_yaw + start_yawrate * lookahead_time / 2;
      actual_pos = local_pose.getOrigin() + tf2::Matrix3x3(createQuaternionFromYaw(midpoint_yaw)) *
        tf2::Vector3(start_vel_x, start_vel_y, 0) * lookahead_time;
      actual_yaw = start_yaw + start_yawrate * lookahead_time;
    }

    const tf2::Transform actual_pose =
      tf2::Transform(createQuaternionFromYaw(actual_yaw), actual_pos);

    validateTransform(actual_pose);

    // compute cost gradients
    const double delta_x = 0.3;
    const double delta_y = 0.2;
    const double delta_yaw = 0.1;

    const double center_cost = get_cost(costmap_, actual_pos);
    const double delta_cost_x = (
      compute_avg_line_cost(
        costmap_, actual_pos,
        actual_pose * tf2::Vector3(delta_x, 0, 0)) -
      compute_avg_line_cost(
        costmap_, actual_pos,
        actual_pose * tf2::Vector3(-delta_x, 0, 0))) /
      delta_x;

    const double delta_cost_y = (
      compute_avg_line_cost(
        costmap_, actual_pos,
        actual_pose * tf2::Vector3(cost_y_lookahead_dist, delta_y, 0)) -
      compute_avg_line_cost(
        costmap_, actual_pos,
        actual_pose * tf2::Vector3(cost_y_lookahead_dist, -delta_y, 0))) /
      delta_y;

    const double delta_cost_yaw = (
      (
        compute_avg_line_cost(
          costmap_,
          actual_pose *
          (tf2::Matrix3x3(createQuaternionFromYaw(delta_yaw)) * tf2::Vector3(delta_x, 0, 0)),
          actual_pose *
          (tf2::Matrix3x3(createQuaternionFromYaw(delta_yaw)) * tf2::Vector3(-delta_x, 0, 0)))
      ) - (
        compute_avg_line_cost(
          costmap_,
          actual_pose *
          (tf2::Matrix3x3(createQuaternionFromYaw(-delta_yaw)) * tf2::Vector3(delta_x, 0, 0)),
          actual_pose *
          (tf2::Matrix3x3(createQuaternionFromYaw(-delta_yaw)) * tf2::Vector3(-delta_x, 0, 0)))
      )
      ) / (2 * delta_yaw);

    // fill local plan later
    nav_msgs::msg::Path local_path;
    local_path.header.frame_id = m_local_frame;
    local_path.header.stamp = time_now;

    // compute obstacle distance
    bool have_obstacle = false;
    double obstacle_dist = 0;
    double obstacle_cost = 0;
    {
      const double delta_move = 0.05;
      const double translation_speed = std::hypot(start_vel_x, start_vel_y);
      const double sweep_speed = translation_speed + std::abs(start_yawrate) *
        costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
      requireFinite({sweep_speed}, "obstacle prediction speed");
      const double delta_time = sweep_speed > 1e-6 ? delta_move / sweep_speed : 0.0;

      tf2::Transform pose = actual_pose;

      while (obstacle_dist < 10) {
        const double cost = footprintCost(pose);

        bool is_contained = false;
        {
          unsigned int dummy[2] = {};
          is_contained = worldToMap(costmap_,
            pose.getOrigin().x(),
            pose.getOrigin().y(), dummy[0], dummy[1]);
        }
        have_obstacle = cost >= max_cost || !is_contained;
        obstacle_cost = finiteMax(obstacle_cost, cost);

        {
          geometry_msgs::msg::PoseStamped tmp;
          auto tmp1 = tf2::toMsg(pose);
          tmp.header = local_path.header;
          tmp.pose.position.x = tmp1.translation.x;
          tmp.pose.position.y = tmp1.translation.y;
          tmp.pose.position.z = tmp1.translation.z;
          tmp.pose.orientation.x = tmp1.rotation.x;
          tmp.pose.orientation.y = tmp1.rotation.y;
          tmp.pose.orientation.z = tmp1.rotation.z;
          tmp.pose.orientation.w = tmp1.rotation.w;
          local_path.poses.push_back(tmp);
        }
        if (!is_contained || have_obstacle) {
          break;
        }

        if (delta_time == 0.0) {
          break;
        }
        const double yaw = tf2::getYaw(pose.getRotation());
        const double midpoint_yaw = yaw + start_yawrate * delta_time / 2;
        pose = tf2::Transform(
          createQuaternionFromYaw(yaw + start_yawrate * delta_time),
          pose.getOrigin() + tf2::Matrix3x3(createQuaternionFromYaw(midpoint_yaw)) *
          tf2::Vector3(start_vel_x, start_vel_y, 0) * delta_time);
        validateTransform(pose);
        obstacle_dist += delta_move;
      }
    }
    m_local_plan_pub->publish(local_path);

    obstacle_dist -= min_stop_dist;

    // publish local plan

    // compute situational max velocities
    const double max_trans_vel =
      finiteMax(
      translationLimit() * (max_cost - center_cost) / max_cost,
      std::min(min_vel_trans, translationLimit()));
    const double max_rot_vel =
      finiteMax(max_vel_theta * (max_cost - center_cost) / max_cost, min_vel_theta);

    // find closest point on path to future position
    auto iter_target = find_closest_point(local_plan.cbegin(), local_plan.cend(), actual_pos);

    // check if goal target
    bool is_goal_target = false;
    {
      // check if goal is within reach
      auto iter_next = move_along_path(iter_target, local_plan.cend(), max_goal_dist);
      is_goal_target = iter_next + 1 >= local_plan.cend();

      if (is_goal_target) {
        // go straight to goal
        iter_target = iter_next;
      }
    }
    // figure out target orientation
    double target_yaw = 0;
    if (is_goal_target) {
      // take goal orientation
      target_yaw = tf2::getYaw(iter_target->getRotation());
    } else {
      // compute path based target orientation
      auto iter_next = move_along_path(iter_target, local_plan.cend(), lookahead_dist);
      const auto direction = iter_next->getOrigin() - iter_target->getOrigin();
      requireFinite({direction.length2()}, "lookahead direction");
      if (direction.length2() <= 1e-18) {
        throw nav2_core::InvalidPath("No distinct lookahead point for path orientation");
      }
      target_yaw = ::atan2(
        iter_next->getOrigin().y() - iter_target->getOrigin().y(),
        iter_next->getOrigin().x() - iter_target->getOrigin().x());
    }

    // get target position
    const tf2::Vector3 target_pos = iter_target->getOrigin();
    double yaw_error = 0.0;

    if (m_robot_direction == 1 || is_goal_target) {
      yaw_error = angles::shortest_angular_distance(actual_yaw, target_yaw);
    } else {
      yaw_error = angles::shortest_angular_distance(actual_yaw + M_PI, target_yaw);
    }

    // compute errors
    const double goal_dist = (local_plan.back().getOrigin() - actual_pos).length();
    const tf2::Vector3 pos_error =
      tf2::Transform(createQuaternionFromYaw(actual_yaw), actual_pos).inverse() * target_pos;

    requireFinite({center_cost, delta_cost_x, delta_cost_y, delta_cost_yaw,
        obstacle_dist, obstacle_cost, max_trans_vel, max_rot_vel, target_yaw, yaw_error,
        goal_dist, pos_error.x(), pos_error.y(), pos_error.z()}, "path/cost control intermediates");

    // compute control values
    bool is_emergency_brake = false;
    double control_vel_x = 0;
    double control_vel_y = 0;
    double control_yawrate = 0;

    if (is_goal_target) {
      // use term for final stopping position
      control_vel_x = pos_error.x() * pos_x_gain;
      requireFinite({control_vel_x}, "goal X control velocity");
    } else {
      control_vel_x = m_robot_direction * max_trans_vel;

      // wait to start moving
      if (m_state != state_t::STATE_TRANSLATING && fabs(yaw_error) > start_yaw_error) {
        control_vel_x = 0;
      }

      // limit curve velocity
      {
        // A straight path has no curvature restriction; do not divide by zero.
        const double max_vel_x = std::abs(yaw_error) > 1e-9 ?
          max_curve_vel * (lookahead_dist / std::abs(yaw_error)) : max_trans_vel;
        requireFinite({max_vel_x}, "curve velocity");
        if (m_robot_direction == -1.0) {
          control_vel_x = m_robot_direction * finiteMin(fabs(control_vel_x), max_vel_x);
        } else {
          control_vel_x = finiteMin(control_vel_x, max_vel_x);
        }
      }

      // limit velocity when approaching goal position
      if (fabs(start_vel_x) > 0) {
        const double stop_accel = 0.8 * acc_lim_x;
        const double stop_time = sqrt(2 * std::abs(goal_dist) / stop_accel);
        requireFinite({stop_time, stop_accel * stop_time}, "goal braking velocity");

        if (m_robot_direction == -1.0) {
          const double max_vel_x = m_robot_direction * finiteMax(
            fabs(
              stop_accel) * stop_time, min_vel_trans);
          control_vel_x = m_robot_direction * finiteMin(fabs(control_vel_x), fabs(max_vel_x));
        } else {
          const double max_vel_x = finiteMax(stop_accel * stop_time, min_vel_trans);
          control_vel_x = finiteMin(control_vel_x, max_vel_x);
        }
      }

      // Limit forward motion without rejecting a safe lateral or reverse command.
      if (have_obstacle && fabs(start_vel_x) > 0) {
        const double stop_accel = 0.9 * acc_lim_x;
        const double stop_time = sqrt(2 * finiteMax(obstacle_dist, 0) / stop_accel);
        const double max_vel_x = stop_accel * stop_time;
        requireFinite({stop_time, max_vel_x}, "obstacle braking velocity");

        // check if it's much lower than current velocity
        if (fabs(max_vel_x) < 0.5 * fabs(start_vel_x)) {
          is_emergency_brake = true;
        }

        if (m_robot_direction == -1.0) {
          control_vel_x = m_robot_direction * finiteMin(fabs(control_vel_x), max_vel_x);
        } else {
          control_vel_x = finiteMin(control_vel_x, max_vel_x);
        }
      }

      // stop before hitting obstacle
      if (have_obstacle && obstacle_dist <= 0) {
        control_vel_x = 0;
      }

      // only allow forward velocity depending on the parameter setting
      if (!m_allow_reversing) {
        control_vel_x = finiteMax(control_vel_x, 0);
      }
    }
    // limit backing up
    if (differential_drive && !m_allow_reversing && is_goal_target && max_backup_dist > 0 &&
      pos_error.x() < (m_state == state_t::STATE_TURNING ? 0 : -max_backup_dist))
    {
      control_vel_x = 0;
      m_state = state_t::STATE_TURNING;
    } else if (m_state == state_t::STATE_TURNING) {
      m_state = state_t::STATE_IDLE;
    }
    if (differential_drive) {
      if (fabs(start_vel_x) > (m_state == state_t::STATE_TRANSLATING ?
        trans_stopped_vel : 2 * trans_stopped_vel))
      {
        // we are translating, use term for lane keeping
        control_yawrate = pos_error.y() / start_vel_x * pos_y_yaw_gain;

        if (!is_goal_target) {
          // additional term for lane keeping
          control_yawrate += yaw_error * yaw_gain;

          // add cost terms
          control_yawrate -= delta_cost_y / start_vel_x * cost_y_yaw_gain;
          control_yawrate -= delta_cost_yaw * cost_yaw_gain;
        }

        m_state = state_t::STATE_TRANSLATING;
      } else if (m_state == state_t::STATE_TURNING) {
        // continue on current yawrate
        control_yawrate = (start_yawrate > 0 ? 1 : -1) * max_rot_vel;
      } else if (is_goal_target &&  // NOLINT
        (m_state == state_t::STATE_ADJUSTING ||
        fabs(yaw_error) < M_PI / 6) &&
        fabs(pos_error.y()) > (m_state == state_t::STATE_ADJUSTING ?
        0.25 * xy_goal_tolerance : 0.5 * xy_goal_tolerance))
      {
        // we are not translating, but we have too large y error
        control_yawrate = (pos_error.y() > 0 ? 1 : -1) * max_rot_vel;

        m_state = state_t::STATE_ADJUSTING;
      } else {
        // use term for static target orientation
        control_yawrate = yaw_error * static_yaw_gain;

        m_state = state_t::STATE_ROTATING;
      }
    } else {
      // simply correct y with holonomic drive
      control_vel_y = pos_error.y() * pos_y_gain;

      if (m_state == state_t::STATE_TURNING) {
        // continue on current yawrate
        control_yawrate = (start_yawrate > 0 ? 1 : -1) * max_rot_vel;
      } else {
        // use term for static target orientation
        control_yawrate = yaw_error * static_yaw_gain;

        if (fabs(start_vel_x) > trans_stopped_vel) {
          m_state = state_t::STATE_TRANSLATING;
        } else {
          m_state = state_t::STATE_ROTATING;
        }
      }

      // apply x cost term only when rotating
      if (m_state == state_t::STATE_ROTATING && fabs(yaw_error) > M_PI / 6) {
        control_vel_x -= delta_cost_x * cost_x_gain;
      }

      // apply y cost term when not approaching goal or if we are rotating
      if (!is_goal_target || (m_state == state_t::STATE_ROTATING && fabs(yaw_error) > M_PI / 6)) {
        control_vel_y -= delta_cost_y * cost_y_gain;
      }

      // apply yaw cost term when not approaching goal
      if (!is_goal_target) {
        control_yawrate -= delta_cost_yaw * cost_yaw_gain;
      }
    }
    // Measured-motion lookahead cannot decide that every command is blocked:
    // a stationary omni robot may have a valid lateral or reverse escape.
    // The final candidate and measured stopping trajectories remain authoritative.
    requireFinite({control_vel_x, control_vel_y, control_yawrate}, "raw control velocity");

    // apply low pass filter

    control_vel_x = control_vel_x * low_pass_gain + m_last_control_values[0] * (1 - low_pass_gain);
    control_vel_y = control_vel_y * low_pass_gain + m_last_control_values[1] * (1 - low_pass_gain);
    control_yawrate = control_yawrate * low_pass_gain + m_last_control_values[2] *
      (1 - low_pass_gain);

    requireFinite({control_vel_x, control_vel_y, control_yawrate}, "filtered control velocity");

    // Apply hard speed bounds to both the target and previous command first.
    // Then interpolate XY with a shared fraction: the speed disk and signed axis
    // bounds are convex, so this preserves both speed and per-axis acceleration.
    const double translation_limit = translationLimit();
    const auto limit_velocity = [&](geometry_msgs::msg::Twist & velocity) {
        velocity.linear.x = std::clamp(velocity.linear.x, min_vel_x, max_vel_x);
        velocity.linear.y = differential_drive ? 0.0 :
          std::clamp(velocity.linear.y, min_vel_y, max_vel_y);
        velocity.angular.z = std::clamp(velocity.angular.z, -max_vel_theta, max_vel_theta);
        const double translation_speed = std::hypot(velocity.linear.x, velocity.linear.y);
        requireFinite({translation_speed, translation_limit}, "translation limit");
        if (translation_speed > translation_limit) {
          const double scale = translation_limit / translation_speed;
          velocity.linear.x *= scale;
          velocity.linear.y *= scale;
        }
      };
    cmd_vel.linear.x = control_vel_x;
    cmd_vel.linear.y = control_vel_y;
    cmd_vel.angular.z = control_yawrate;
    limit_velocity(cmd_vel);
    auto previous = m_last_cmd_vel;
    // An externally reduced hard speed limit takes precedence over deceleration.
    limit_velocity(previous);
    const double dx = cmd_vel.linear.x - previous.linear.x;
    const double dy = cmd_vel.linear.y - previous.linear.y;
    const double dw = cmd_vel.angular.z - previous.angular.z;
    const bool braking_x = (dx < 0 && previous.linear.x > 0) ||
      (dx > 0 && previous.linear.x < 0);
    const double accel_x = is_emergency_brake && braking_x ?
      emergency_acc_lim_x : acc_lim_x;
    requireFinite({dx, dy, dw, accel_x * dt, acc_lim_y * dt, acc_lim_theta * dt},
      "acceleration increments");
    double fraction = 1.0;
    if (std::abs(dx) > 0.0) {
      fraction = std::min(fraction, accel_x * dt / std::abs(dx));
    }
    if (std::abs(dy) > 0.0) {
      fraction = std::min(fraction, acc_lim_y * dt / std::abs(dy));
    }
    requireFinite({fraction}, "translation acceleration fraction");
    cmd_vel.linear.x = previous.linear.x + fraction * dx;
    cmd_vel.linear.y = previous.linear.y + fraction * dy;
    cmd_vel.angular.z = previous.angular.z +
      std::clamp(dw, -acc_lim_theta * dt, acc_lim_theta * dt);
    validateVelocity(cmd_vel);
    // Check the actual candidate command and current motion, including stopping distance.
    checkTrajectory(local_pose, speed, std::max(dt, lookahead_time));
    checkTrajectory(local_pose, cmd_vel, std::max(dt, lookahead_time));

    m_last_time = time_now;
    m_last_control_values[0] = cmd_vel.linear.x;
    m_last_control_values[1] = cmd_vel.linear.y;
    m_last_control_values[2] = cmd_vel.angular.z;
    m_last_cmd_vel = cmd_vel;

    m_update_counter++;
    cmd_vel_final.header.stamp = clock_->now();
    cmd_vel_final.header.frame_id = m_base_frame;
    cmd_vel_final.twist.linear = cmd_vel.linear;
    cmd_vel_final.twist.angular = cmd_vel.angular;

    validateVelocity(cmd_vel_final.twist);
    return cmd_vel_final;
  } catch (const nav2_core::ControllerException & error) {
    resetState();
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "%s", error.what());
    throw;
  }
}

void NeoLocalPlanner::cleanup()
{
  dyn_params_handler_.reset();
  post_params_handler_.reset();
  m_odom_sub.reset();
  m_local_plan_pub.reset();
  m_odometry.reset();
  m_global_plan = nav_msgs::msg::Path();
  reset();
}

void NeoLocalPlanner::activate()
{
  reset();
  m_local_plan_pub->on_activate();
}

void NeoLocalPlanner::deactivate()
{
  m_local_plan_pub->on_deactivate();
  reset();
}

const std::vector<NeoLocalPlanner::NumericParameter> & NeoLocalPlanner::numericParameters()
{
  static const std::vector<NumericParameter> parameters = {
    {"acc_lim_x", &NeoLocalPlanner::acc_lim_x, 0.5},
    {"acc_lim_y", &NeoLocalPlanner::acc_lim_y, 0.5},
    {"acc_lim_theta", &NeoLocalPlanner::acc_lim_theta, 0.5},
    {"min_vel_x", &NeoLocalPlanner::min_vel_x, -0.1},
    {"max_vel_x", &NeoLocalPlanner::max_vel_x, 0.5},
    {"min_vel_y", &NeoLocalPlanner::min_vel_y, -0.5},
    {"max_vel_y", &NeoLocalPlanner::max_vel_y, 0.5},
    {"min_rot_vel", &NeoLocalPlanner::min_vel_theta, 0.1},
    {"max_rot_vel", &NeoLocalPlanner::max_vel_theta, 0.5},
    {"min_vel_trans", &NeoLocalPlanner::min_vel_trans, 0.1},
    {"max_vel_trans", &NeoLocalPlanner::max_vel_trans, 0.5},
    {"rot_stopped_vel", &NeoLocalPlanner::theta_stopped_vel, 0.05},
    {"trans_stopped_vel", &NeoLocalPlanner::trans_stopped_vel, 0.05},
    {"yaw_goal_tolerance", &NeoLocalPlanner::yaw_goal_tolerance, 0.02},
    {"xy_goal_tolerance", &NeoLocalPlanner::xy_goal_tolerance, 0.1},
    {"goal_tune_time", &NeoLocalPlanner::goal_tune_time, 0.5},
    {"lookahead_time", &NeoLocalPlanner::lookahead_time, 0.5},
    {"lookahead_dist", &NeoLocalPlanner::m_lookahead_dist, 0.5},
    {"start_yaw_error", &NeoLocalPlanner::start_yaw_error, 0.2},
    {"pos_x_gain", &NeoLocalPlanner::pos_x_gain, 1.0},
    {"pos_y_gain", &NeoLocalPlanner::pos_y_gain, 1.0},
    {"pos_y_yaw_gain", &NeoLocalPlanner::pos_y_yaw_gain, 1.0},
    {"yaw_gain", &NeoLocalPlanner::yaw_gain, 1.0},
    {"static_yaw_gain", &NeoLocalPlanner::static_yaw_gain, 3.0},
    {"cost_x_gain", &NeoLocalPlanner::cost_x_gain, 0.1},
    {"cost_y_gain", &NeoLocalPlanner::cost_y_gain, 0.1},
    {"cost_y_yaw_gain", &NeoLocalPlanner::cost_y_yaw_gain, 0.1},
    {"cost_y_lookahead_dist", &NeoLocalPlanner::m_cost_y_lookahead_dist, 0.0},
    {"cost_y_lookahead_time", &NeoLocalPlanner::cost_y_lookahead_time, 1.0},
    {"cost_yaw_gain", &NeoLocalPlanner::cost_yaw_gain, 1.0},
    {"low_pass_gain", &NeoLocalPlanner::low_pass_gain, 0.5},
    {"max_cost", &NeoLocalPlanner::max_cost, 0.9},
    {"max_curve_vel", &NeoLocalPlanner::max_curve_vel, 0.2},
    {"max_goal_dist", &NeoLocalPlanner::max_goal_dist, 0.5},
    {"max_backup_dist", &NeoLocalPlanner::max_backup_dist, 0.5},
    {"min_stop_dist", &NeoLocalPlanner::min_stop_dist, 0.5},
    {"emergency_acc_lim_x", &NeoLocalPlanner::emergency_acc_lim_x, 0.5},
    {"max_control_dt", &NeoLocalPlanner::max_control_dt, 0.5},
    {"clock_wait_timeout", &NeoLocalPlanner::clock_wait_timeout, 0.05},
  };
  return parameters;
}

std::string NeoLocalPlanner::validateParameters(const std::map<std::string, double> & v)
{
  for (const auto & item : v) {
    if (!std::isfinite(item.second)) {
      return item.first + " must be finite";
    }
    if (item.first != "min_vel_x" && item.first != "min_vel_y" && item.second < 0.0) {
      return item.first + " must be nonnegative";
    }
  }
  for (const auto * name : {"acc_lim_x", "acc_lim_theta", "max_vel_trans",
      "max_rot_vel", "trans_stopped_vel", "rot_stopped_vel", "lookahead_dist",
      "max_control_dt", "max_cost", "xy_goal_tolerance"})
  {
    if (v.at(name) <= 0.0) {
      return std::string(name) + " must be positive";
    }
  }
  if (v.at("clock_wait_timeout") > v.at("max_control_dt")) {
    return "clock_wait_timeout must not exceed max_control_dt";
  }
  if (v.at("min_vel_x") > 0 || v.at("min_vel_y") > 0) {
    return "Signed min_vel_x and min_vel_y must be <= 0 so zero commands remain possible";
  }
  if (v.at("min_vel_trans") > v.at("max_vel_trans") ||
    v.at("min_rot_vel") > v.at("max_rot_vel"))
  {
    return "Minimum translation/rotation speeds must not exceed their maxima";
  }
  if (v.at("trans_stopped_vel") > v.at("max_vel_trans") ||
    v.at("rot_stopped_vel") > v.at("max_rot_vel"))
  {
    return "Stopped velocity thresholds must not exceed configured speed maxima";
  }
  if (v.at("low_pass_gain") > 1.0 || v.at("max_cost") > 1.0) {
    return "low_pass_gain must be in [0,1] and max_cost in (0,1]";
  }
  if (v.at("emergency_acc_lim_x") < v.at("acc_lim_x")) {
    return "emergency_acc_lim_x must be >= acc_lim_x";
  }
  return {};
}

rcl_interfaces::msg::SetParametersResult NeoLocalPlanner::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  rcl_interfaces::msg::SetParametersResult result;
  std::map<std::string, double> values;
  for (const auto & spec : numericParameters()) {
    values[spec.name] = this->*spec.member;
  }
  bool next_differential_drive = differential_drive;
  for (const auto & parameter : parameters) {
    const std::string prefix = plugin_name_ + ".";
    if (parameter.get_name().compare(0, prefix.size(), prefix) != 0) {
      continue;
    }
    const auto name = parameter.get_name().substr(prefix.size());
    if (values.count(name)) {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.reason = name + " must be a double";
        return result;
      }
      values[name] = parameter.as_double();
    } else if (name == "differential_drive" || name == "allow_reversing") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.reason = name + " must be a bool";
        return result;
      }
      if (name == "differential_drive") {
        next_differential_drive = parameter.as_bool();
      }
    } else {
      result.reason = name + " requires controller cleanup and reconfiguration; "
        "use max_vel_trans/min_vel_trans for runtime translation limit updates";
      return result;
    }
  }
  result.reason = validateParameters(values);
  if (!next_differential_drive && values.at("acc_lim_y") <= 0.0) {
    result.reason = "acc_lim_y must be positive for omnidirectional drive";
  }
  result.successful = result.reason.empty();
  return result;
}

void NeoLocalPlanner::applyParameters(const std::vector<rclcpp::Parameter> & parameters)
{
  // Apply only after ALL node callbacks accepted the transaction.
  std::lock_guard<std::mutex> lock(m_mutex);
  bool changed = false;
  for (const auto & parameter : parameters) {
    for (const auto & spec : numericParameters()) {
      if (parameter.get_name() == plugin_name_ + "." + spec.name) {
        this->*spec.member = parameter.as_double();
        changed = true;
      }
    }
    if (parameter.get_name() == plugin_name_ + ".differential_drive") {
      differential_drive = parameter.as_bool();
      changed = true;
    } else if (parameter.get_name() == plugin_name_ + ".allow_reversing") {
      m_allow_reversing = parameter.as_bool();
      changed = true;
    }
  }
  if (changed) {
    resetState();
  }
}

void NeoLocalPlanner::resetState()
{
  m_state = state_t::STATE_IDLE;
  std::fill(std::begin(m_last_control_values), std::end(m_last_control_values), 0.0);
  m_last_cmd_vel = geometry_msgs::msg::Twist();
  m_last_time = clock_->now();
  m_robot_direction = 1.0;
  count = 0;
}

void NeoLocalPlanner::reset()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  resetState();
}

void NeoLocalPlanner::setPlan(const nav_msgs::msg::Path & plan)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  try {
    validatePath(plan);
  } catch (const nav2_core::ControllerException &) {
    m_global_plan = nav_msgs::msg::Path();
    resetState();
    throw;
  }
  if (m_global_plan.poses.empty() ||
    plan.poses.back().pose != m_global_plan.poses.back().pose ||
    plan.header.frame_id != m_global_plan.header.frame_id)
  {
    resetState();
  }
  m_global_plan = plan;
}

double NeoLocalPlanner::translationLimit() const
{
  if (speed_limit_ == nav2_costmap_2d::NO_SPEED_LIMIT) {
    return max_vel_trans;
  }
  return speed_limit_percentage_ ? max_vel_trans * (speed_limit_ / 100.0) :
         std::min(max_vel_trans, speed_limit_);
}

void NeoLocalPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  if (!std::isfinite(speed_limit) || speed_limit < 0.0 || (percentage && speed_limit > 100.0)) {
    // Called from a subscription: do not throw out of the executor or clear an active limit.
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Ignoring invalid speed limit");
    return;
  }
  speed_limit_ = speed_limit;
  speed_limit_percentage_ = percentage;
}

double NeoLocalPlanner::footprintCost(const tf2::Transform & pose) const
{
  validateTransform(pose);
  const auto footprint = costmap_ros_->getRobotFootprint();
  if (footprint.size() < 3) {
    throw nav2_core::NoValidControl("A nondegenerate convex robot footprint is required");
  }
  double winding = 0.0;
  for (size_t i = 0; i < footprint.size(); ++i) {
    const auto & a = footprint[i];
    const auto & b = footprint[(i + 1) % footprint.size()];
    const auto & c = footprint[(i + 2) % footprint.size()];
    requireFinite({a.x, a.y, a.z}, "footprint vertex");
    const double cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    requireFinite({cross}, "footprint convexity");
    if (cross * winding < 0.0) {
      throw nav2_core::NoValidControl("Robot footprint must be convex");
    }
    if (std::abs(cross) > 1e-12) {
      winding = cross;
    }
  }
  if (winding == 0.0) {
    throw nav2_core::NoValidControl("Robot footprint has zero area");
  }
  std::vector<geometry_msgs::msg::Point> oriented;
  nav2_costmap_2d::transformFootprint(pose.getOrigin().x(), pose.getOrigin().y(),
    tf2::getYaw(pose.getRotation()), footprint, oriented);
  std::vector<nav2_costmap_2d::MapLocation> polygon;
  for (const auto & point : oriented) {
    requireFinite({point.x, point.y}, "transformed footprint");
    nav2_costmap_2d::MapLocation cell;
    if (!worldToMap(costmap_, point.x, point.y, cell.x, cell.y)) {
      return 1.0;
    }
    polygon.push_back(cell);
  }
  // Nav2's outline-only checker misses obstacles wholly inside the polygon.
  // Use the existing costmap rasterizer to check both interior and boundary cells.
  std::vector<nav2_costmap_2d::MapLocation> cells;
  costmap_->convexFillCells(polygon, cells);
  if (cells.empty()) {
    throw nav2_core::NoValidControl("Robot footprint rasterization is empty");
  }
  unsigned char cost = nav2_costmap_2d::FREE_SPACE;
  for (const auto & cell : cells) {
    cost = std::max(cost, costmap_->getCost(cell.x, cell.y));
  }
  // Inflation encodes clearance for the robot CENTER, not occupied geometry
  // at every point under its polygon. Applying it to the entire footprint
  // inflates the robot twice and can block collision-free narrow passages.
  if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    return 1.0;  // Also rejects unknown cells anywhere inside the footprint.
  }
  return get_cost(costmap_, pose.getOrigin());
}

void NeoLocalPlanner::checkTrajectory(
  const tf2::Transform & start_pose, const geometry_msgs::msg::Twist & velocity,
  double reaction_time) const
{
  validateVelocity(velocity);
  requireFinite({reaction_time}, "collision reaction time");
  tf2::Transform pose = start_pose;
  double vx = velocity.linear.x;
  double vy = velocity.linear.y;
  double wz = velocity.angular.z;
  double radius = 0.0;
  for (const auto & point : costmap_ros_->getRobotFootprint()) {
    requireFinite({point.x, point.y}, "footprint radius");
    radius = std::max(radius, std::hypot(point.x, point.y));
  }
  requireFinite({radius}, "footprint radius");
  const double resolution = costmap_->getResolution();
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    throw nav2_core::NoValidControl("Invalid costmap resolution");
  }
  const double threshold = std::min(max_cost,
    nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE / 255.0);
  for (size_t step = 0; step < 10000; ++step) {
    if (footprintCost(pose) >= threshold) {
      throw nav2_core::NoValidControl(
          "Predicted footprint intersects obstacle, unknown or map boundary");
    }
    if (vx == 0.0 && vy == 0.0 && wz == 0.0) {
      return;
    }
    const double sweep_speed = std::hypot(vx, vy) + radius * std::abs(wz);
    requireFinite({sweep_speed}, "trajectory sweep speed");
    double step_time = std::min(0.05, resolution / (2.0 * sweep_speed));
    if (reaction_time > 0.0) {
      step_time = std::min(step_time, reaction_time);
    }
    if (!std::isfinite(step_time) || step_time <= 0.0) {
      throw nav2_core::NoValidControl("Invalid collision integration interval");
    }
    const double yaw = tf2::getYaw(pose.getRotation());
    pose = tf2::Transform(createQuaternionFromYaw(yaw + wz * step_time),
      pose.getOrigin() + tf2::Matrix3x3(createQuaternionFromYaw(yaw + wz * step_time / 2)) *
      tf2::Vector3(vx, vy, 0) * step_time);
    validateTransform(pose);
    if (reaction_time > 0.0) {
      reaction_time = std::max(0.0, reaction_time - step_time);
    } else {
      // Conservative step: integrate at the previous speed before reducing it.
      vx = std::copysign(std::max(0.0, std::abs(vx) - acc_lim_x * step_time), vx);
      vy = std::copysign(std::max(0.0, std::abs(vy) - acc_lim_y * step_time), vy);
      wz = std::copysign(std::max(0.0, std::abs(wz) - acc_lim_theta * step_time), wz);
    }
  }
  throw nav2_core::NoValidControl("Collision prediction exceeded its sampling budget");
}

tf2::Transform NeoLocalPlanner::lookupTransform(
  const std::string & target, const std::string & source) const
{
  tf2::Transform transform;
  try {
    const auto msg = tf_->lookupTransform(target, source, tf2::TimePointZero);
    tf2::fromMsg(msg.transform, transform);
  } catch (const tf2::TransformException & error) {
    throw nav2_core::ControllerTFError(
            "TF " + source + " -> " + target + " failed: " + error.what());
  }
  validateTransform(transform);
  return transform;
}

void NeoLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node || !tf || !costmap_ros || !costmap_ros->getCostmap()) {
    throw nav2_core::ControllerException("NeoLocalPlanner requires a node, TF buffer and costmap");
  }
  node_ = parent;
  plugin_name_ = name;
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;

  const auto & overrides = node->get_node_parameters_interface()->get_parameter_overrides();
  // Accept both historical spellings at startup, reject conflicting explicit values.
  for (const auto & names : {std::make_pair("max_vel_trans", "max_trans_vel"),
      std::make_pair("min_vel_trans", "min_trans_vel")})
  {
    const std::string canonical = plugin_name_ + "." + names.first;
    const std::string alias = plugin_name_ + "." + names.second;
    if (overrides.count(alias) && overrides.count(canonical) &&
      overrides.at(alias) != overrides.at(canonical))
    {
      throw nav2_core::ControllerException(canonical + " conflicts with " + alias);
    }
  }
  std::map<std::string, double> values;
  for (const auto & spec : numericParameters()) {
    double default_value = spec.default_value;
    if (std::string(spec.name) == "max_vel_trans" || std::string(spec.name) == "min_vel_trans") {
      const auto alias = plugin_name_ + (std::string(spec.name) == "max_vel_trans" ?
        ".max_trans_vel" : ".min_trans_vel");
      if (overrides.count(alias)) {
        default_value = overrides.at(alias).get<double>();
        RCLCPP_WARN(logger_, "%s is deprecated; use %s.%s", alias.c_str(),
          plugin_name_.c_str(), spec.name);
      }
    }
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + "." + spec.name, rclcpp::ParameterValue(default_value));
    node->get_parameter(plugin_name_ + "." + spec.name, this->*spec.member);
    values[spec.name] = this->*spec.member;
  }
  const auto error = validateParameters(values);
  if (!error.empty()) {
    throw nav2_core::ControllerException(plugin_name_ + ": " + error);
  }
  for (const auto & item : {std::make_pair("max_trans_vel", max_vel_trans),
      std::make_pair("min_trans_vel", min_vel_trans)})
  {
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + "." + item.first, rclcpp::ParameterValue(item.second));
  }
  for (auto item : {std::make_pair("differential_drive", &differential_drive),
      std::make_pair("allow_reversing", &m_allow_reversing)})
  {
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + "." + item.first,
      rclcpp::ParameterValue(std::string(item.first) == "differential_drive"));
    node->get_parameter(plugin_name_ + "." + item.first, *item.second);
  }
  if (!differential_drive && acc_lim_y <= 0.0) {
    throw nav2_core::ControllerException("acc_lim_y must be positive for omnidirectional drive");
  }
  for (auto item : {std::make_pair("odom_topic", &odom_topic),
      std::make_pair("local_plan_topic", &local_plan_topic),
      std::make_pair("local_frame", &m_local_frame), std::make_pair("base_frame", &m_base_frame)})
  {
    std::string default_value = *item.second;
    if (std::string(item.first) == "local_frame") {
      default_value = costmap_ros_->getGlobalFrameID();
    } else if (std::string(item.first) == "base_frame") {
      default_value = costmap_ros_->getBaseFrameID();
    }
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + "." + item.first, rclcpp::ParameterValue(default_value));
    node->get_parameter(plugin_name_ + "." + item.first, *item.second);
    if (item.second->empty()) {
      throw nav2_core::ControllerException(std::string(item.first) + " must not be empty");
    }
  }
  if (m_local_frame != costmap_ros_->getGlobalFrameID() ||
    m_base_frame != costmap_ros_->getBaseFrameID())
  {
    throw nav2_core::ControllerException("local_frame/base_frame must match the costmap frames");
  }
  resetState();
  // Topic resolution follows ROS namespaces independently of explicitly configured TF frames.
  m_odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&NeoLocalPlanner::odomCallback, this, std::placeholders::_1));
  m_local_plan_pub = node->create_publisher<nav_msgs::msg::Path>(local_plan_topic, 1);
  // Validate configured parameters even while inactive; apply only accepted transactions.
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &NeoLocalPlanner::dynamicParametersCallback,
      this, std::placeholders::_1));
  post_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(&NeoLocalPlanner::applyParameters, this, std::placeholders::_1));
}

void NeoLocalPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock_reinit(m_mutex);
  m_odometry = msg;
}

}  // namespace neo_local_planner2

PLUGINLIB_EXPORT_CLASS(neo_local_planner2::NeoLocalPlanner, nav2_core::Controller)
