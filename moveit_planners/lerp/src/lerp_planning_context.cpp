#include "lerp_planning_context.h"
#include "moveit/planning_interface/planning_request.h"
#include "moveit/planning_interface/planning_response.h"
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>

LERPPlanningContext::LERPPlanningContext(const std::string& context_name, const std::string& group_name,
                                         const robot_model::RobotModelConstPtr& model)
  : planning_interface::PlanningContext(context_name, group_name), robot_model_(model)
{
    dof = robot_model_->getJointModelGroup(group_)->getActiveJointModelNames().size();
    std::cout << "groooooooooooooooooooop name: " << group_ << std::endl;
}

// LERPPlanningContext::~LERPPlanningContext() = default;

bool LERPPlanningContext::solve(planning_interface::MotionPlanResponse& resp)
{
  std::cout << "=====>>>>> solve() is called" << std::endl;

  // get the start state joint values
  std::vector<double> joint_start = request_.start_state.joint_state.position;

  //planning_scene_->getRobotStateUpdated(request_.start_state);


  std::vector<moveit_msgs::Constraints> goal_constraints = request_.goal_constraints;

  std::cout << "===>>> number of goal constraints: " << goal_constraints.size() << std::endl;
  int goal_constraint_index_with_joint_constraint = -1;
  for(int a = 0; a < goal_constraints.size(); ++a){
    int g =  goal_constraints[a].joint_constraints.size();
    printf( "===>>> goal constraints: %i and the number of joint constraints: %i \n", a+1,  g);
    if (g != 0){
        goal_constraint_index_with_joint_constraint = a;
    }
  }

  if( goal_constraint_index_with_joint_constraint == -1){
      printf("===>>> no joint constraint is found in the goals");
      resp.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
      exit (EXIT_FAILURE);
  }

  std::vector<moveit_msgs::JointConstraint> goal_joint_constraint = goal_constraints[goal_constraint_index_with_joint_constraint].joint_constraints;

  std::vector<double> joint_goal;
  for (auto x : goal_joint_constraint)
  {
    joint_goal.push_back(x.position);
  }


  resp.trajectory_ =
      robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model_, getGroupName()));

  robot_state::RobotStatePtr rob_state(new robot_state::RobotState(robot_model_));

  trajectory_msgs::JointTrajectory rob_joint_traj = interpolateMultDOF(joint_start, joint_goal, 5);


  resp.trajectory_->setRobotTrajectoryMsg(*rob_state, rob_joint_traj);

  return true;
};

bool LERPPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
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
    plotVector("v1 ==>>", v1);
    plotVector("v2 ==>>", v2);
  trajectory_msgs::JointTrajectory traj;

  std::cout << " degrees of freedom " << dof << std::endl;

  traj.points.resize(num+1);
  std::cout << "========== " << traj.points.size() << std::endl;


  std::vector<double> dt_vector;
  for (int j = 0; j < dof; ++j){
      double dt = ( v2[j] - v1[j] )/num;
      dt_vector.push_back(dt);
  }

  for (int i = 0; i <= num ; ++i)
  {
      std::vector<double> v;
      for (int k = 0; k < dof; ++k){
          v.push_back(v1[k] + i * dt_vector[k]);
      }
      plotVector("===>>>", v);
      traj.points[i].positions = v;
      ros::Duration t(i * 0.1);
      traj.points[i].time_from_start = t;
      std::cout << "t: " << t << std::endl;
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

