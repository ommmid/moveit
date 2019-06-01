#ifndef MODEL_BASED_PLANNING_CONTEXT_H
#define MODEL_BASED_PLANNING_CONTEXT_H


#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_interface/planning_interface.h>


class ModelBasedPlanningContext : public planning_interface::PlanningContext
{
public:
  ModelBasedPlanningContext(const std::string& name, const std::string& groupName);


  bool solve(planning_interface::MotionPlanResponse& res) override;

  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;

  void clear() override;


};

#endif // MODEL_BASED_PLANNING_CONTEXT_H
