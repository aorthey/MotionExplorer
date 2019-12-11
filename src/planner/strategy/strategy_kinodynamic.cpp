#include "planner/strategy/strategy_kinodynamic.h"
#include "planner/cspace/cspace_kinodynamic.h"
#include "planner/benchmark/benchmark_input.h"
#include "util.h"
#include <ompl/geometric/planners/quotientspace/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/geometric/planners/quotientspace/Explorer.h>
#include <ompl/geometric/planners/quotientspace/QRRT.h>
#include <ompl/control/optimizers/Optimizer.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/ltl/LTLPlanner.h>

#include <ompl/control/PlannerData.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/util/Time.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <boost/lexical_cast.hpp>

namespace ot = ompl::tools;

static ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  obj->setCostThreshold(ob::Cost(ob::dInf));
  return obj;
}

void PostRunEventKinodynamic(const ob::PlannerPtr &planner, ot::Benchmark::RunProperties &run)
{
  static uint pid = 0;

  ob::SpaceInformationPtr si = planner->getSpaceInformation();
  ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();

  bool solved = pdef->hasExactSolution();

  uint states = boost::lexical_cast<int>(run["graph states INTEGER"]);
  double time = boost::lexical_cast<double>(run["time REAL"]);
  double memory = boost::lexical_cast<double>(run["memory REAL"]);

  std::cout << "Run " << pid << " [" << planner->getName() << "] " << (solved?"solved":"no solution") << "(time: "<< time << ", states: " << states << ", memory: " << memory << ")" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  pid++;

}

StrategyKinodynamicMultiLevel::StrategyKinodynamicMultiLevel()
{
}

ob::PlannerPtr StrategyKinodynamicMultiLevel::GetPlanner(std::string algorithm,
    std::vector<ob::SpaceInformationPtr> si_vec, 
    ob::ProblemDefinitionPtr pdef,
    const StrategyInput& input)
{
  ob::PlannerPtr planner;
  //assume last cspace is dynamic
  const oc::SpaceInformationPtr si = static_pointer_cast<oc::SpaceInformation>(si_vec.back());

  if(algorithm=="ompl:dynamic:rrt"){
    planner = std::make_shared<oc::RRT>(si);
  }else if(algorithm=="ompl:dynamic:est"){
    planner = std::make_shared<oc::EST>(si);
  }else if(algorithm=="ompl:dynamic:sst"){
    planner = std::make_shared<oc::SST>(si);
  }else if(algorithm=="ompl:dynamic:pdst"){
    planner = std::make_shared<oc::PDST>(si);
  }else if(algorithm=="ompl:dynamic:kpiece"){
    planner = std::make_shared<oc::KPIECE1>(si);
  }else if(algorithm=="hierarchy:explorer"){
    planner = std::make_shared<og::MotionExplorer>(si_vec);
  }else if(algorithm=="optimizer"){
    si->setup();
    CSpaceOMPL* cspace = input.cspace_levels.back();
    PathPiecewiseLinear *path = new PathPiecewiseLinear(cspace);
    std::string fpath = input.name_loadPath;
    path->Load(fpath.c_str());
    std::cout << "load current path from : " << fpath << std::endl;
    planner = std::make_shared<oc::Optimizer>(si, path->GetOMPLPath());
  }else{
    std::cout << "Planner algorithm " << algorithm << " is unknown." << std::endl;
    throw "Planner unknown.";
  }
  planner->setProblemDefinition(pdef);
  return planner;

}
void StrategyKinodynamicMultiLevel::Init( const StrategyInput &input )
{
  std::string algorithm = input.name_algorithm;

  std::vector<ob::SpaceInformationPtr> si_vec; 
  ob::ProblemDefinitionPtr pdef; 

  for(uint k = 0; k < input.cspace_levels.size(); k++){
    CSpaceOMPL* cspace_levelk = input.cspace_levels.at(k);
    bool dynamic = cspace_levelk->isDynamic();
    ob::SpaceInformationPtr sik = cspace_levelk->SpaceInformationPtr();

    ob::ScopedState<> startk(sik);
    ob::ScopedState<> goalk(sik);
    if(dynamic){

      KinodynamicCSpaceOMPL* cspace_dynamic = dynamic_cast<KinodynamicCSpaceOMPL*>(cspace_levelk);
      if(cspace_dynamic==nullptr)
      {
        std::cout << "ERROR: not dynamic even though declared dynamic" << std::endl;
        std::cout << *cspace_levelk << std::endl;
        throw "Not dynamic.";
      }

      oc::SpaceInformationPtr sik_dynamic = static_pointer_cast<oc::SpaceInformation>(sik);
      sik_dynamic->setMinMaxControlDuration(0.01, 0.1);
      sik_dynamic->setPropagationStepSize(1);

      startk = cspace_dynamic->ConfigVelocityToOMPLState(input.q_init, input.dq_init);
      goalk  = cspace_dynamic->ConfigVelocityToOMPLState(input.q_goal, input.dq_goal);
    }else{
      startk = cspace_levelk->ConfigToOMPLState(input.q_init);
      goalk  = cspace_levelk->ConfigToOMPLState(input.q_goal);
    }
    setStateSampler(input.name_sampler, sik);
    si_vec.push_back(sik);

    if(k >= input.cspace_levels.size()-1){
      pdef = std::make_shared<ob::ProblemDefinition>(sik);
      pdef->addStartState(startk);
      auto goal=std::make_shared<ob::GoalState>(sik);
      goal->setState(goalk);
      goal->setThreshold(input.epsilon_goalregion);
      pdef->setGoal(goal);
      pdef->setOptimizationObjective( getThresholdPathLengthObj(sik) );
    }
  }

  ob::StateSpacePtr spacek = si_vec.back()->getStateSpace();

  max_planning_time = input.max_planning_time;

  //###########################################################################
  // choose planner
  //###########################################################################

  if(util::StartsWith(algorithm,"benchmark")){
    //No Init, directly execute benchmark
    std::cout << "NYI" << std::endl;
    throw "NYI";
  }else{
    planner = GetPlanner(algorithm, si_vec, pdef, input);
    planner->setup();
    planner->clear();
    isInitialized = true;
  }
}

void StrategyKinodynamicMultiLevel::Plan( StrategyOutput &output)
{

  //###########################################################################
  // choose planner
  //###########################################################################
  // const oc::SpaceInformationPtr si = static_pointer_cast<oc::SpaceInformation>(planner->getSpaceInformation());
  // si->getStateSpace()->registerProjections();
  // si->setMinMaxControlDuration(0.01, 0.1);
  // si->setPropagationStepSize(1);
  // si->getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new SE3Project0r(si->getStateSpace())));

    // ob::PlannerPtr planner = GetPlanner(algorithm, si_vec, pdef_vec);
    // planner->setup();


    // double max_planning_time= input.max_planning_time;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max_planning_time) );

  double minimalCostAcceptable = 5;
  planner->getProblemDefinition()->getOptimizationObjective()->setCostThreshold(ob::Cost(minimalCostAcceptable));
  std::cout << planner->getProblemDefinition()->getOptimizationObjective()->getCostThreshold() << std::endl;


  ompl::time::point start = ompl::time::now();
  planner->solve(ptc);
  output.planner_time = ompl::time::seconds(ompl::time::now() - start);
  output.max_planner_time = max_planning_time;

  //###########################################################################
  ob::PlannerDataPtr pd( new ob::PlannerData(planner->getSpaceInformation()) );
  planner->getPlannerData(*pd);

  unsigned int N = planner->getProblemDefinition()->getSolutionCount();
  if(N>0){
      ob::PlannerSolution solution = planner->getProblemDefinition()->getSolutions().at(0);
      pd->path_ = solution.path_;
  }

  output.SetPlannerData(pd);
  output.SetProblemDefinition(planner->getProblemDefinition());
}
void StrategyKinodynamicMultiLevel::Clear()
{
  planner->clear();
}
void StrategyKinodynamicMultiLevel::Step(StrategyOutput &output)
{
  OMPL_ERROR("NYI");
  throw "NYI";
}
