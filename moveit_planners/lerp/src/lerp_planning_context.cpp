#include "lerp_planning_context.h"
#include "moveit/planning_interface/planning_request.h"
#include "moveit/planning_interface/planning_response.h"
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/planning_scene/planning_scene.h>

LERPPlanningContext::LERPPlanningContext(const std::string& context_name, const std::string& group_name,
                                         const robot_model::RobotModelConstPtr& model)
  : planning_interface::PlanningContext(context_name, group_name), robot_model_(model)
{
    std::cout << "===>>> LERPPlanningContext is constructed" << std::endl;

    dof = robot_model_->getJointModelGroup(group_)->getActiveJointModelNames().size();
    robot_state_  = robot_state::RobotStatePtr(new robot_state::RobotState(robot_model_));
    // plot the jointvalues here
    robot_state_->setToDefaultValues();
    robot_state_->update();

    // plot the jointvalues here
}

bool LERPPlanningContext::solve(planning_interface::MotionPlanResponse& resp)
{
  std::cout << "====>>> solve() is called" << std::endl;

  // get the start state joint values
  std::vector<double> start_joint_values = request_.start_state.joint_state.position;

  std::vector<moveit_msgs::Constraints> goal_constraints = request_.goal_constraints;

  std::cout << "===>>> number of goal constraints: " << goal_constraints.size() << std::endl;
  int goal_constraint_index_with_joint_constraint = -1;
  for(int a = 0; a < goal_constraints.size(); ++a){
    int joint_constraints_size =  goal_constraints[a].joint_constraints.size();
    printf( "===>>> goal constraints: %i and the number of joint constraints: %i \n", a+1,  joint_constraints_size);
    if (joint_constraints_size != 0){
        goal_constraint_index_with_joint_constraint = a;
    }
  }

  if( goal_constraint_index_with_joint_constraint == -1){
      printf("===>>> no joint constraint is found in goals");
      resp.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
      exit (EXIT_FAILURE);
  }

  std::vector<moveit_msgs::JointConstraint> goal_joint_constraint = goal_constraints[goal_constraint_index_with_joint_constraint].joint_constraints;

  std::vector<double> goal_joint_values;
  for (auto x : goal_joint_constraint)
  {
    goal_joint_values.push_back(x.position);
  }

  resp.trajectory_ =
      robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model_, group_));



  trajectory_msgs::JointTrajectory rob_joint_traj = interpolateMultDOF(start_joint_values, goal_joint_values, 40);


  resp.trajectory_->setRobotTrajectoryMsg(*robot_state_, rob_joint_traj);

  return true;
};

bool LERPPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  ROS_ERROR_NAMED("LERPP", "PlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res) undefined");
  return true;
};

bool LERPPlanningContext::terminate()
{
  return true;
};

void LERPPlanningContext::clear(){

};

trajectory_msgs::JointTrajectory LERPPlanningContext::interpolateMultDOF(const std::vector<double>& v1,
                                                                         const std::vector<double>& v2, const int& num)
{
    trajectory_msgs::JointTrajectory traj;


    const robot_state::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(group_);

    const std::vector<std::string> j_names = joint_model_group->getVariableNames();

    //robot_model_->getJointModelGroup(group_);

  std::cout << "===>>> degrees of freedom " << dof << std::endl;

  traj.points.resize(num+1);
  std::cout << "===>>> traj.point.size: " << traj.points.size() << std::endl;

  std::vector<double> dt_vector;
  for (int j = 0; j < dof; ++j){
      double dt = ( v2[j] - v1[j] )/num;
      dt_vector.push_back(dt);
  }

  for (int i = 0; i <= num ; ++i)
  {
      std::vector<double> v;
      for (int k = 0; k < dof; ++k){
          double j_value = v1[k] + i * dt_vector[k];
          v.push_back(j_value);

          robot_state_->setJointPositions(j_names[k], &j_value);
          robot_state_->update();
      }
      bool isValid = planning_scene_->isStateValid(*robot_state_, group_, false);
      printf("the robot at state %i is valid ? %s", i, isValid ? "true" : "false \n");

      plotVector("===>>> ", v);
      traj.joint_names = j_names;
      traj.points[i].positions = v;
      ros::Duration t(i * 0.5);
      traj.points[i].time_from_start = t;
  }

  return traj;
}

std::vector<double> LERPPlanningContext::interpolateSingleDOF(const double& d1, const double& d2, const int& num)
{

  std::vector<double> v;

  int dt = ( d2 - d1 )/num;
  for(int k = 0; k <= num; k++){

      v.push_back(d1 + k * dt); // it includes d1 and d2 as well
  }


  return v;
}

void LERPPlanningContext::plotVector(const std::string& str, const std::vector<double>& v){

    std::cout << str << " ";
    for (int i = 0; i < v.size(); ++i){
        std::cout << v[i] << " ";
    }
    std::cout << std::endl;
}

