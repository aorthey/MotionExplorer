#include "planner/strategy/strategy_geometric_multilevel.h"
#include "planner/strategy/ompl/slicespace_prm.h"
#include "planner/strategy/ompl/plain_rrt.h"

#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

StrategyGeometricMultiLevel::StrategyGeometricMultiLevel()
{
}

void StrategyGeometricMultiLevel::plan( const StrategyInput &input, StrategyOutput &output)
{

  Config p_init = input.q_init;
  Config p_goal = input.q_goal;
  std::string algorithm = input.name_algorithm;

  CSpaceOMPL* cspace_level0 = input.cspace_level0;
  CSpaceOMPL* cspace_level1 = input.cspace_level1;

  ob::SpaceInformationPtr si0 = cspace_level0->SpaceInformationPtr();
  ob::SpaceInformationPtr si1 = cspace_level1->SpaceInformationPtr();

  //###########################################################################
  // Config init,goal to OMPL start/goal
  //###########################################################################

  ob::ScopedState<> start0 = cspace_level0->ConfigToOMPLState(p_init);
  ob::ScopedState<> goal0  = cspace_level0->ConfigToOMPLState(p_goal);
  ob::ScopedState<> start1 = cspace_level1->ConfigToOMPLState(p_init);
  ob::ScopedState<> goal1  = cspace_level1->ConfigToOMPLState(p_goal);

  ob::ProblemDefinitionPtr pdef0 = std::make_shared<ob::ProblemDefinition>(si0);
  pdef0->addStartState(start0);
  auto goal=std::make_shared<ob::GoalState>(si0);
  goal->setState(goal0);
  goal->setThreshold(input.epsilon_goalregion);
  pdef0->setGoal(goal);
  pdef0->setOptimizationObjective( getThresholdPathLengthObj(si0) );

  ob::ProblemDefinitionPtr pdef1 = std::make_shared<ob::ProblemDefinition>(si1);
  pdef1->addStartState(start1);
  goal=std::make_shared<ob::GoalState>(si1);
  goal->setState(goal1);
  goal->setThreshold(input.epsilon_goalregion);
  pdef1->setGoal(goal);
  pdef1->setOptimizationObjective( getThresholdPathLengthObj(si1) );

  //###########################################################################
  // choose planner
  //###########################################################################

  typedef std::shared_ptr<og::SliceSpacePRM> SliceSpacePRMPtr;
  typedef std::shared_ptr<og::PlainRRT> PlainRRTPtr;
  //SliceSpacePRMPtr planner = std::make_shared<og::SliceSpacePRM>(input.world, si0, si1);
  ob::PlannerPtr planner;

  if(algorithm=="ompl:plain_rrt"){
    planner = std::make_shared<og::PlainRRT>(si1);
    planner->setProblemDefinition(pdef1);
  }else if(algorithm=="ompl:slicespace_prm"){
    planner = std::make_shared<og::SliceSpacePRM>(input.world, si0, si1);
    static_pointer_cast<og::SliceSpacePRM>(planner)->setProblemDefinitionLevel0(pdef0);
    static_pointer_cast<og::SliceSpacePRM>(planner)->setProblemDefinitionLevel1(pdef1);
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

  ob::PlannerStatus status = planner->solve(ptc);
  std::cout << status << std::endl;
  //ob::PlannerStatus status = ss.solve(ptc);

  //###########################################################################

  ob::PlannerDataPtr pd( new ob::PlannerData(si1) );
  planner->getPlannerData(*pd);

  output.SetPlannerData(pd);
  output.SetProblemDefinition(pdef1);

}
