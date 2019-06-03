#ifndef LERP_PLANNING_CONTEXT_H
#define LERP_PLANNING_CONTEXT_H


#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_interface/planning_interface.h>

MOVEIT_CLASS_FORWARD(LERPPlanningContext);

class LERPPlanningContext : public planning_interface::PlanningContext
{
public:

  LERPPlanningContext(const std::string& name, const std::string& group, const robot_model::RobotModelConstPtr& model);
  ~LERPPlanningContext() override{

  }

  bool solve(planning_interface::MotionPlanResponse& res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;
  void clear() override;


private:
  trajectory_msgs::JointTrajectory interpolateMultDOF(const std::vector<double>& v1, const std::vector<double>& v2, const int& num);
  std::vector<double> interpolateSingleDOF(const double& d1, const double& d2, const int& num);

  moveit::core::RobotModelConstPtr robot_model_;

};

#endif // LERP_PLANNING_CONTEXT_H
