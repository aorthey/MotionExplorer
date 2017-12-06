#include "planner/strategy/strategy_geometric_multilevel.h"
#include "planner/strategy/ompl/rrt_plain.h"
#include "planner/strategy/ompl/prm_basic.h"
#include "planner/strategy/ompl/prm_multislice.h"
#include "planner/strategy/ompl/prm_slice.h"

#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/Time.h>

StrategyGeometricMultiLevel::StrategyGeometricMultiLevel()
{
}

void StrategyGeometricMultiLevel::plan( const StrategyInput &input, StrategyOutput &output)
{

  Config p_init = input.q_init;
  Config p_goal = input.q_goal;
  std::string algorithm = input.name_algorithm;

  std::vector<ob::SpaceInformationPtr> si_vec; 
  std::vector<ob::ProblemDefinitionPtr> pdef_vec; 


  for(uint k = 0; k < input.cspace_levels.size(); k++){
    CSpaceOMPL* cspace_levelk = input.cspace_levels.at(k);
    ob::SpaceInformationPtr sik = cspace_levelk->SpaceInformationPtr();
    SetSampler(input.name_sampler, sik);

    ob::ScopedState<> startk = cspace_levelk->ConfigToOMPLState(p_init);
    ob::ScopedState<> goalk  = cspace_levelk->ConfigToOMPLState(p_goal);

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

  ob::PlannerPtr planner;

  if(algorithm=="ompl:rrt_plain"){
    planner = std::make_shared<og::RRTPlain>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
  }else if(algorithm=="ompl:prm_plain"){
    planner = std::make_shared<og::PRMBasic>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
  }else if(algorithm=="ompl:prm_slice"){
    planner = std::make_shared<og::PRMSlice>(si_vec.back(),nullptr);
    planner->setProblemDefinition(pdef_vec.back());
  }else if(algorithm=="ompl:prm_multislice"){
    planner = std::make_shared<og::PRMMultiSlice>(si_vec);
    static_pointer_cast<og::PRMMultiSlice>(planner)->setProblemDefinition(pdef_vec);
  }else{
    std::cout << "Planner algorithm " << algorithm << " is unknown." << std::endl;
    exit(0);
  }

  planner->setup();

  //###########################################################################
  // solve
  //
  // termination condition: 
  //    reached duration or found solution in epsilon-neighborhood
  //###########################################################################

  double max_planning_time= input.max_planning_time;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max_planning_time) );

  ompl::time::point start = ompl::time::now();
  ob::PlannerStatus status = planner->solve(ptc);
  output.planner_time = ompl::time::seconds(ompl::time::now() - start);
  output.max_planner_time = max_planning_time;

  std::cout << status << std::endl;
  //###########################################################################

  ob::PlannerDataPtr pd( new ob::PlannerData(si_vec.back()) );
  planner->getPlannerData(*pd);

  output.SetPlannerData(pd);
  output.SetProblemDefinition(pdef_vec.back());

}
