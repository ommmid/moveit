#include "model_based_planning_context.h"
#include "moveit/planning_interface/planning_request.h"
#include "moveit/planning_interface/planning_response.h"


ModelBasedPlanningContext::ModelBasedPlanningContext(const std::string& name,
                                                     const std::string& groupName)
    : planning_interface::PlanningContext(name, groupName)
{

}



bool ModelBasedPlanningContext::solve(planning_interface::MotionPlanResponse& res){

    // get the start state joint values
    std::vector<double> startPosition = request_.start_state.joint_state.position;

    // get the goal state joint values
    std::vector<moveit_msgs::PositionConstraint> goalPosition = request_.goal_constraints[0].position_constraints;

    for(auto x : goalPosition){
        std::cout << x << std::endl;
    }

    // do the interpolation between them

    // set the trajectory above to res

};

bool ModelBasedPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res){

};

bool ModelBasedPlanningContext::terminate(){

};

void ModelBasedPlanningContext::clear(){

};
