#include "lerp_planning_context.h"
#include "moveit/planning_interface/planning_request.h"
#include "moveit/planning_interface/planning_response.h"
#include <moveit/planning_interface/planning_interface.h>




LERPPlanningContext::LERPPlanningContext(const std::string& name,
          const std::string& groupName, const robot_model::RobotModelConstPtr& model)
    : planning_interface::PlanningContext(name, groupName), robot_model_(model)
{

}

//LERPPlanningContext::~LERPPlanningContext() = default;

bool LERPPlanningContext::solve(planning_interface::MotionPlanResponse& resp){

    std::cout << "=====>>>>> solve() is called" << std::endl;

    // get the start state joint values
    std::vector<double> joint_start = request_.start_state.joint_state.position;


    std::vector<moveit_msgs::Constraints> goal_constraints = request_.goal_constraints;
   std::cout << "===>>> number of constraints in goal: " << goal_constraints.size() << std::endl;

    std::vector<moveit_msgs::JointConstraint> goal_joint_constraint =
            goal_constraints.back().joint_constraints;

    std::vector<double> joint_goal;
    for(auto x : goal_joint_constraint){
        joint_goal.push_back(x.position);
        //std::cout << "==>> joint position " << x.position << std::endl;
    }

    resp.trajectory_ =
        robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model_, getGroupName()));

robot_state::RobotStatePtr rob_state(new robot_state::RobotState(robot_model_));




    trajectory_msgs::JointTrajectory rob_joint_traj = interpolateMultDOF(joint_start, joint_goal, 5);


resp.trajectory_->setRobotTrajectoryMsg(*rob_state, rob_joint_traj);



    return true;
};

bool LERPPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res){
return  true;
};

bool LERPPlanningContext::terminate(){

    return  true;
};

void LERPPlanningContext::clear(){

};


trajectory_msgs::JointTrajectory LERPPlanningContext::interpolateMultDOF(const std::vector<double>& v1, const std::vector<double>& v2, const int& num){

    trajectory_msgs::JointTrajectory traj;
    for (int j = 0; j < 7 ; j++) {
// get robot model number DOF
        // interpolate for each joint between start and goal: between v1[j] and v2[j]
        std::vector<double> v = interpolateSingleDOF(v1[j], v2[j], num);
        for(int i = 0; i < num; i++){
            traj.points[i].positions = v;
        }
    }


    return traj;
}


std::vector<double> LERPPlanningContext::interpolateSingleDOF(const double& d1, const double& d2, const int& num){
std::vector<double> v;
    return  v;
}


