#include "planner/strategy/strategy_kinodynamic.h"
#include "planner/cspace/cspace_kinodynamic.h"
#include "planner/strategy/benchmark.h"

#include "util.h"
#include "elements/plannerdata_vertex_annotated.h"

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/util/Time.h>

static ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  obj->setCostThreshold(ob::Cost(dInf));
  return obj;
}

StrategyKinodynamicMultiLevel::StrategyKinodynamicMultiLevel()
{
}

ob::PlannerPtr StrategyKinodynamicMultiLevel::GetPlanner(std::string algorithm,
    std::vector<oc::SpaceInformationPtr> si_vec, 
    std::vector<ob::ProblemDefinitionPtr> pdef_vec)

{
  ob::PlannerPtr planner;
  const oc::SpaceInformationPtr si = si_vec.back();

  if(algorithm=="ompl:rrtconnect"){
    planner = std::make_shared<oc::RRT>(si);
  }else{
    std::cout << "Planner algorithm " << algorithm << " is unknown." << std::endl;
    exit(0);
  }
  planner->setProblemDefinition(pdef_vec.back());
  return planner;

}

void StrategyKinodynamicMultiLevel::plan( const StrategyInput &input, StrategyOutput &output)
{
  std::string algorithm = input.name_algorithm;

  std::vector<oc::SpaceInformationPtr> si_vec; 
  std::vector<ob::ProblemDefinitionPtr> pdef_vec; 

  for(uint k = 0; k < input.cspace_levels.size(); k++){
    KinodynamicCSpaceOMPL* cspace_levelk = static_cast<KinodynamicCSpaceOMPL*>(input.cspace_levels.at(k));
    oc::SpaceInformationPtr sik = static_pointer_cast<oc::SpaceInformation>(cspace_levelk->SpaceInformationPtr());
    setStateSampler(input.name_sampler, sik);

    ob::ScopedState<> startk = cspace_levelk->ConfigVelocityToOMPLState(input.q_init, input.dq_init);
    ob::ScopedState<> goalk  = cspace_levelk->ConfigVelocityToOMPLState(input.q_goal, input.dq_goal);

    std::cout << *cspace_levelk << std::endl;

    ob::ProblemDefinitionPtr pdefk = std::make_shared<ob::ProblemDefinition>(sik);
    pdefk->addStartState(startk);
    auto goal=std::make_shared<ob::GoalState>(sik);
    goal->setState(goalk);
    goal->setThreshold(input.epsilon_goalregion);
    pdefk->setGoal(goal);
    pdefk->setOptimizationObjective( getThresholdPathLengthObj(sik) );

    si_vec.push_back(sik);
    pdef_vec.push_back(pdefk);
  }

  //###########################################################################
  // choose planner
  //###########################################################################

  const oc::SpaceInformationPtr si = static_pointer_cast<oc::SpaceInformation>(si_vec.back());
  si->setMinMaxControlDuration(0.01, 0.1);
  si->setPropagationStepSize(1);

  ob::PlannerPtr planner = GetPlanner(algorithm, si_vec, pdef_vec);
  planner->clear();
  planner->setup();

  double max_planning_time= input.max_planning_time;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max_planning_time) );

  ompl::time::point start = ompl::time::now();
  planner->solve(ptc);
  output.planner_time = ompl::time::seconds(ompl::time::now() - start);
  output.max_planner_time = max_planning_time;

  //###########################################################################
  ob::PlannerDataPtr pd( new ob::PlannerData(si_vec.back()) );
  planner->getPlannerData(*pd);

  output.SetPlannerData(pd);
  output.SetProblemDefinition(planner->getProblemDefinition());
}
