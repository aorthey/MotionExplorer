#include "planner/strategy/strategy_input.h"
#include "planner/strategy/strategy_output.h"
#include "planner/strategy/strategy_geometric.h"

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/stride/STRIDE.h>

#include <ompl/util/Time.h>

static ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  obj->setCostThreshold(ob::Cost(dInf));
  return obj;
}
StrategyGeometric::StrategyGeometric()
{
}
void StrategyGeometric::plan( const StrategyInput &input, StrategyOutput &output)
{

  Config p_init = input.q_init;
  Config p_goal = input.q_goal;
  std::string algorithm = input.name_algorithm;
  CSpaceOMPL* cspace = input.cspace;

  //###########################################################################
  // Config init,goal to OMPL start/goal
  //###########################################################################

  ob::ScopedState<> start = cspace->ConfigToOMPLState(p_init);
  ob::ScopedState<> goal  = cspace->ConfigToOMPLState(p_goal);

  og::SimpleSetup ss(cspace->SpaceInformationPtr());
  const ob::SpaceInformationPtr si = ss.getSpaceInformation();

  ss.setStateValidityChecker(cspace->StateValidityCheckerPtr(si));

  SetSampler(input.name_sampler, si);


  //###########################################################################
  // choose planner
  //###########################################################################
  ob::PlannerPtr ompl_planner;

  if(algorithm=="ompl:rrt") ompl_planner = std::make_shared<og::RRT>(si);
  else if(algorithm=="ompl:rrtconnect") ompl_planner = std::make_shared<og::RRTConnect>(si);
  else if(algorithm=="ompl:prrt") ompl_planner = std::make_shared<og::pRRT>(si);
  else if(algorithm=="ompl:rrtsharp") ompl_planner = std::make_shared<og::RRTsharp>(si);
  else if(algorithm=="ompl:rrtstar") ompl_planner = std::make_shared<og::RRTstar>(si);
  else if(algorithm=="ompl:rrtxstatic") ompl_planner = std::make_shared<og::RRTXstatic>(si);
  else if(algorithm=="ompl:informedrrtstar") ompl_planner = std::make_shared<og::InformedRRTstar>(si);
  else if(algorithm=="ompl:lazyrrt") ompl_planner = std::make_shared<og::LazyRRT>(si);
  else if(algorithm=="ompl:btrrt") ompl_planner = std::make_shared<og::BiTRRT>(si);
  else if(algorithm=="ompl:lbtrrt") ompl_planner = std::make_shared<og::LBTRRT>(si);

  else if(algorithm=="ompl:prm") ompl_planner = std::make_shared<og::PRM>(si);
  else if(algorithm=="ompl:prmstar") ompl_planner = std::make_shared<og::PRMstar>(si);
  else if(algorithm=="ompl:lazyprm") ompl_planner = std::make_shared<og::LazyPRM>(si);
  else if(algorithm=="ompl:lazyprmstar") ompl_planner = std::make_shared<og::LazyPRMstar>(si);
  else if(algorithm=="ompl:spars") ompl_planner = std::make_shared<og::SPARS>(si);
  else if(algorithm=="ompl:spars2") ompl_planner = std::make_shared<og::SPARStwo>(si);

  else if(algorithm=="ompl:stride") ompl_planner = std::make_shared<og::STRIDE>(si);
  else if(algorithm=="ompl:sst") ompl_planner = std::make_shared<og::SST>(si);
  else if(algorithm=="ompl:pdst") ompl_planner = std::make_shared<og::PDST>(si);
  else if(algorithm=="ompl:kpiece") ompl_planner = std::make_shared<og::KPIECE1>(si);
  else if(algorithm=="ompl:bkpiece") ompl_planner = std::make_shared<og::BKPIECE1>(si);
  else if(algorithm=="ompl:lbkpiece") ompl_planner = std::make_shared<og::LBKPIECE1>(si);
  else if(algorithm=="ompl:est") ompl_planner = std::make_shared<og::EST>(si);
  else if(algorithm=="ompl:biest") ompl_planner = std::make_shared<og::BiEST>(si);
  else if(algorithm=="ompl:sbl") ompl_planner = std::make_shared<og::SBL>(si);
  else if(algorithm=="ompl:psbl") ompl_planner = std::make_shared<og::pSBL>(si);
  else if(algorithm=="ompl:fmt") ompl_planner = std::make_shared<og::FMT>(si);
  else if(algorithm=="ompl:bfmt") ompl_planner = std::make_shared<og::BFMT>(si);

  else{
    std::cout << "Planner algorithm " << algorithm << " is unknown." << std::endl;
    exit(0);
  }

  //###########################################################################
  // setup and projection
  //###########################################################################

  ss.setStartState(start);
  ss.setGoal(input.GetGoalPtr(si));

  ss.setPlanner(ompl_planner);
  ss.setup();
  ss.getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new SE3Project0r(ss.getStateSpace())));

  //set objective to infinite path to just return first solution
  ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
  pdef->setOptimizationObjective( getThresholdPathLengthObj(si) );
  //pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si)));

  //###########################################################################
  // solve
  //
  // termination condition: 
  //    reached duration or found solution in epsilon-neighborhood
  //###########################################################################

  double max_planning_time= input.max_planning_time;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max_planning_time) );

  ompl::time::point t_start = ompl::time::now();
  ob::PlannerStatus status = ss.solve(ptc);
  output.planner_time = ompl::time::seconds(ompl::time::now() - t_start);
  output.max_planner_time = max_planning_time;

  //###########################################################################

  ob::PlannerDataPtr pd( new ob::PlannerData(si) );
  ss.getPlannerData(*pd);

  output.SetPlannerData(pd);
  output.SetProblemDefinition(pdef);

}
