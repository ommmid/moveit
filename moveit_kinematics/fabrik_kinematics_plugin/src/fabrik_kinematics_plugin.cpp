/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Omid Heidari */

#include "moveit/fabrik_kinematics_plugin/fabrik_kinematics_plugin.h"
#include "moveit/fabrik_kinematics_plugin/fabrik_model.h"

#include <moveit/kinematics_base/kinematics_base.h>

#include <tf2/transform_datatypes.h>

// FABRIK
#include <fabrik/base/fabrik.h>
#include "fabrik/util/math.h"

// register KDLKinematics as a KinematicsBase implementation
#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(fabrik_kinematics_plugin::FabrikKinematicsPlugin, kinematics::KinematicsBase)

namespace fabrik_kinematics_plugin
{
  
FabrikKinematicsPlugin::FabrikKinematicsPlugin() : initialized_(false)
{
    
}

bool FabrikKinematicsPlugin::initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                                     const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                     double search_discretization)
{
  ROS_INFO("-------------------------------- 1");
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
    return false;

  if (!joint_model_group_->isChain())
  {
    ROS_ERROR_NAMED("kdl", "Group '%s' is not a chain", group_name.c_str());
    return false;
  }
  if (!joint_model_group_->isSingleDOFJoints())
  {
    ROS_ERROR_NAMED("kdl", "Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
    return false;
  }
ROS_INFO("-------------------------------- 2");
  dimension_ = joint_model_group_->getActiveJointModels().size() + joint_model_group_->getMimicJointModels().size();
  for (std::size_t i = 0; i < joint_model_group_->getJointModels().size(); ++i)
  {
    if (joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::REVOLUTE ||
        joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::PRISMATIC)
    {
      solver_info_.joint_names.push_back(joint_model_group_->getJointModelNames()[i]);
      const std::vector<moveit_msgs::JointLimits>& jvec =
          joint_model_group_->getJointModels()[i]->getVariableBoundsMsg();
      solver_info_.limits.insert(solver_info_.limits.end(), jvec.begin(), jvec.end());
    }
  }
ROS_INFO("-------------------------------- 3");
  if (!joint_model_group_->hasLinkModel(getTipFrame()))
  {
    ROS_ERROR_NAMED("kdl", "Could not find tip name in joint group '%s'", group_name.c_str());
    return false;
  }
  solver_info_.link_names.push_back(getTipFrame());

  joint_min_.resize(solver_info_.limits.size());
  joint_max_.resize(solver_info_.limits.size());

  for (unsigned int i = 0; i < solver_info_.limits.size(); i++)
  {
    joint_min_(i) = solver_info_.limits[i].min_position;
    joint_max_(i) = solver_info_.limits[i].max_position;
  }
ROS_INFO("-------------------------------- 4");
  // Get Solver Parameters
  lookupParam("max_solver_iterations", max_solver_iterations_, 500);
  lookupParam("epsilon", epsilon_, 1e-5);
  lookupParam("orientation_vs_position", orientation_vs_position_weight_, 1.0);
ROS_INFO("-------------------------------- 5");
  bool position_ik;
  lookupParam("position_only_ik", position_ik, false);
  if (position_ik)  // position_only_ik overrules orientation_vs_position
    orientation_vs_position_weight_ = 0.0;
  if (orientation_vs_position_weight_ == 0.0)
    ROS_INFO_NAMED("kdl", "Using position only ik");
ROS_INFO("-------------------------------- 6");
  // Setup the joint state groups that we need
  state_.reset(new moveit::core::RobotState(robot_model_));

 // do a forward kinematic reset ???
//  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
ROS_INFO("-------------------------------- 7");
  initialized_ = true;
  ROS_DEBUG_NAMED("fabrik", "Fabrik solver initialized");
  return true;
}

bool FabrikKinematicsPlugin::checkConsistency(const Eigen::VectorXd& seed_state,
                                           const std::vector<double>& consistency_limits,
                                           const Eigen::VectorXd& solution) const
{
  for (std::size_t i = 0; i < dimension_; ++i)
    if (fabs(seed_state(i) - solution(i)) > consistency_limits[i])
      return false;
  return true;
}

bool FabrikKinematicsPlugin::timedOut(const ros::WallTime& start_time, double duration) const
{
  return ((ros::WallTime::now() - start_time).toSec() >= duration);
}

bool FabrikKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                        std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                        const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  // limit search to a single attempt by setting a timeout of zero
  return searchPositionIK(ik_pose, ik_seed_state, 0.0, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool FabrikKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, std::vector<double>& solution,
                                           moveit_msgs::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool FabrikKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, const std::vector<double>& consistency_limits,
                                           std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool FabrikKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, std::vector<double>& solution,
                                           const IKCallbackFn& solution_callback,
                                           moveit_msgs::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

// here the actual solving happens
bool FabrikKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, const std::vector<double>& consistency_limits,
                                           std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                           moveit_msgs::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_INFO("-------------------------------- 21");
 ros::WallTime start_time = ros::WallTime::now();
  if (!initialized_)
  {
    ROS_ERROR_NAMED("kdl", "kinematics solver not initialized");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED("kdl", "Seed state must have size " << dimension_ << " instead of size "
                                                               << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // ---------------------- make an instance of fabrik
  // convert joint_model_group_ to fabrik robot model
  FabrikModel fabrik_model(joint_model_group_);
  // FabrikModel fabrik_model;
  fabrik::FABRIKPtr fabrik(new fabrik::FABRIK(fabrik_model.fabrik_robot_model));

  solution.resize(dimension_);
  ROS_DEBUG_STREAM_NAMED("kdl", "searchPositionIK: Position request pose is "
                                    << ik_pose.position.x << " " << ik_pose.position.y << " " << ik_pose.position.z
                                    << " " << ik_pose.orientation.x << " " << ik_pose.orientation.y << " "
                                    << ik_pose.orientation.z << " " << ik_pose.orientation.w);

  unsigned int attempt = 0;
  do
  {
    ++attempt;
    if (attempt > 1)  // randomly re-seed after first attempt
    {
    //   if (!consistency_limits_mimic.empty())
    //     getRandomConfiguration(jnt_seed_state.data, consistency_limits_mimic, jnt_pos_in.data);
    //   else
    //     getRandomConfiguration(jnt_pos_in.data);
    //   ROS_DEBUG_STREAM_NAMED("kdl", "New random configuration (" << attempt << "): " << jnt_pos_in);
    }

    // ===>>> here the IK should solve and the solution goes into jnt_pose_out
    //int ik_valid = 
        // CartToJnt(ik_solver_vel, jnt_pos_in, pose_desired, jnt_pos_out, max_solver_iterations_,
        //           Eigen::Map<const Eigen::VectorXd>(joint_weights_.data(), joint_weights_.size()), cartesian_weights);

     
    Eigen::Quaternion<double> ik_pose_quaternion(ik_pose.orientation.w, ik_pose.orientation.x, ik_pose.orientation.y,ik_pose.orientation.z);
    Eigen::Affine3d target(ik_pose_quaternion); 
    Eigen::Vector3d ik_pose_translation(ik_pose.position.x, ik_pose.position.y, ik_pose.position.z);
    target.translation() = ik_pose_translation;
    fabrik->setInverseKinematicsInput(target,
                                      epsilon_,
                                      max_solver_iterations_,
                                      fabrik::CalculatorType::POSITION);

    fabrik::IKOutput output;
    bool solved = fabrik->solveIK(output);

    // 0 means it IK solver passed
    // int ik_valid = 0;

    if (solved || options.return_approximate_solution)  // found acceptable solution
    {
    //   if (!consistency_limits_mimic.empty() &&
    //       !checkConsistency(jnt_seed_state.data, consistency_limits_mimic, jnt_pos_out.data))
    //     continue;

      // map joint values from fabrik solution which is a double vector to Eigen::VectorXd
      Eigen::VectorXd fabrik_joint_values = Eigen::VectorXd::Map(output.solution_joints_values.data(), output.solution_joints_values.size());
      // map fabrik_joint_values to solution
      Eigen::Map<Eigen::VectorXd>(solution.data(), solution.size()) = fabrik_joint_values; 
      if (!solution_callback.empty())
      {
        solution_callback(ik_pose, solution, error_code);
        if (error_code.val != error_code.SUCCESS)
          continue;
      }

      // solution passed consistency check and solution callback
      error_code.val = error_code.SUCCESS;
      ROS_DEBUG_STREAM_NAMED("kdl", "Solved after " << (ros::WallTime::now() - start_time).toSec() << " < " << timeout
                                                    << "s and " << attempt << " attempts");
      return true;
    }
  } while (!timedOut(start_time, timeout));
  
  ROS_INFO("-------------------------------- 24");

  ROS_DEBUG_STREAM_NAMED("kdl", "IK timed out after " << (ros::WallTime::now() - start_time).toSec() << " > " << timeout
                                                      << "s and " << attempt << " attempts");
  error_code.val = error_code.TIMED_OUT;

  return false;
}


bool FabrikKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                        const std::vector<double>& joint_angles,
                                        std::vector<geometry_msgs::Pose>& poses) const
{
if (!initialized_)
  {
    ROS_ERROR_NAMED("fabrik", "kinematics solver not initialized");
    return false;
  }

  poses.resize(link_names.size());
  if (joint_angles.size() != dimension_)
  {
    ROS_ERROR_NAMED("kdl", "Joint angles vector must have size: %d", dimension_);
    return false;
  }

//   KDL::Frame p_out;
//   KDL::JntArray jnt_pos_in(dimension_);
//   jnt_pos_in.data = Eigen::Map<const Eigen::VectorXd>(joint_angles.data(), joint_angles.size());

  FabrikModel fabrik_model(joint_model_group_);
  fabrik::FABRIKPtr fabrik(new fabrik::FABRIK(fabrik_model.fabrik_robot_model));

  Eigen::Affine3d target = Eigen::Affine3d::Identity();;
  fabrik->solveFK(joint_angles, target);

  bool valid = true;
  for (unsigned int i = 0; i < poses.size(); i++)
  {
    // fill up poses here
    Eigen::Quaternion<double> target_quaternion(target.rotation());
    poses[i].orientation.w = static_cast<float>(target_quaternion.w());
    poses[i].orientation.x = static_cast<float>(target_quaternion.x());
    poses[i].orientation.y = static_cast<float>(target_quaternion.y());
    poses[i].orientation.z = static_cast<float>(target_quaternion.z());

    poses[i].position.x = static_cast<float>(target.translation().x());
    poses[i].position.y = static_cast<float>(target.translation().y());
    poses[i].position.z = static_cast<float>(target.translation().z());
  }

  return valid;  
}

const std::vector<std::string>& FabrikKinematicsPlugin::getJointNames() const
{
  return solver_info_.joint_names;
}

const std::vector<std::string>& FabrikKinematicsPlugin::getLinkNames() const
{
  return solver_info_.link_names;
}

}  // namespace kdl_kinematics_plugin
