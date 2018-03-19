#include "planner/strategy/strategy_kinodynamic.h"
#include "planner/cspace/cspace_kinodynamic.h"
#include "planner/strategy/benchmark.h"
#include "planner/strategy/ompl/kRRT.h"

#include "util.h"
#include "elements/plannerdata_vertex_annotated.h"

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/ltl/LTLPlanner.h>
// #include <ompl/control/planners/syclop/SyclopEST.h>
// #include <ompl/control/planners/syclop/Syclop.h>
// #include <ompl/control/planners/syclop/SyclopRRT.h>

#include <ompl/control/PlannerData.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/util/Time.h>

static ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  obj->setCostThreshold(ob::Cost(dInf));
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
    std::vector<oc::SpaceInformationPtr> si_vec, 
    std::vector<ob::ProblemDefinitionPtr> pdef_vec)

{
  ob::PlannerPtr planner;
  const oc::SpaceInformationPtr si = si_vec.back();

  if(algorithm=="ompl:rrt"){
    planner = std::make_shared<oc::RRT>(si);
  }else if(algorithm=="ompl:est"){
    planner = std::make_shared<oc::EST>(si);
  }else if(algorithm=="ompl:sst"){
    planner = std::make_shared<oc::SST>(si);
  }else if(algorithm=="ompl:pdst"){
    planner = std::make_shared<oc::PDST>(si);
  }else if(algorithm=="ompl:kpiece"){
    planner = std::make_shared<oc::KPIECE1>(si);
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

  if(util::StartsWith(algorithm,"benchmark")){
    RunBenchmark(input, si_vec, pdef_vec);
  }else{
    const oc::SpaceInformationPtr si = static_pointer_cast<oc::SpaceInformation>(si_vec.back());
    si->setMinMaxControlDuration(0.01, 0.1);
    si->setPropagationStepSize(1);
    si->getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new SE3Project0r(si->getStateSpace())));

    ob::PlannerPtr planner = GetPlanner(algorithm, si_vec, pdef_vec);
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
}
void StrategyKinodynamicMultiLevel::RunBenchmark(
    const StrategyInput& input,
    std::vector<oc::SpaceInformationPtr> si_vec, 
    std::vector<ob::ProblemDefinitionPtr> pdef_vec)
{
  std::cout << input.name_algorithm << std::endl;
  BenchmarkInformation binfo(input.name_algorithm);

  const oc::SpaceInformationPtr si = static_pointer_cast<oc::SpaceInformation>(si_vec.back());
  std::string file_benchmark = "benchmark_kinodynamic_"+util::GetCurrentDateTimeString();
  oc::SimpleSetup ss(si);
  ot::Benchmark benchmark(ss, "Benchmark");

  for(uint k = 0; k < binfo.algorithms.size(); k++){
    if(util::StartsWith(binfo.algorithms.at(k), "syclop"))
    {
      // ob::PlannerPtr planner = GetPlanner(binfo.algorithms.at(k), si_vec, pdef_vec);
      // GridDecomposition(20, 3, const base::RealVectorBounds &b);
      std::cout << "cannot handle syclop" << std::endl;
      exit(0);
    }else{
      benchmark.addPlanner(GetPlanner(binfo.algorithms.at(k), si_vec, pdef_vec));
    }
  }



  ob::ProblemDefinitionPtr pdef = pdef_vec.back();

  CSpaceOMPL *cspace = input.cspace_levels.back();
  ob::ScopedState<> start = cspace->ConfigToOMPLState(input.q_init);
  ob::ScopedState<> goal  = cspace->ConfigToOMPLState(input.q_goal);
  ss.setStartAndGoalStates(start,goal,input.epsilon_goalregion);

  ss.getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new SE3Project0r(ss.getStateSpace())));
  ss.setup();

  pdef->setOptimizationObjective( getThresholdPathLengthObj(si) );

  ot::Benchmark::Request req;
  req.maxTime = binfo.maxPlanningTime;
  req.maxMem = binfo.maxMemory;
  req.runCount = binfo.runCount;
  req.displayProgress = true;

  benchmark.setPostRunEvent(std::bind(&PostRunEventKinodynamic, std::placeholders::_1, std::placeholders::_2));
  benchmark.benchmark(req);

  std::string res = file_benchmark+".log";
  std::string cmd;

  benchmark.saveResultsToFile(res.c_str());

  BenchmarkFileToPNG(file_benchmark);
}

