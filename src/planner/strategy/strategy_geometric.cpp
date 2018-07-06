#include "util.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/strategy/strategy_geometric.h"
#include "planner/strategy/benchmark.h"

#include "planner/strategy/quotient/multiquotient.h"
#include "planner/strategy/quotient/qmp_connect.h"
#include "planner/strategy/quotient/qmp_connect_fast.h"
#include "planner/strategy/quotient/qmp.h"
#include "planner/strategy/quotientchart/multichart.h"
#include "planner/strategy/quotientchart/quotient_chart.h"
#include "planner/strategy/quotientchart/qcp.h"
#include "planner/strategy/ompl/rrt_unidirectional.h"
#include "planner/strategy/ompl/rrt_unidirectional_cover.h"
#include "planner/strategy/ompl/rrt_bidirectional.h"

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
#include <ompl/geometric/planners/cforest/CForest.h>


#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>


#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/Time.h>

static ob::OptimizationObjectivePtr GetOptimizationObjective(const ob::SpaceInformationPtr& si)
{
  ////path length
  //ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  //obj->setCostThreshold(ob::Cost(dInf));
  //return obj;
  ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
  ob::OptimizationObjectivePtr clearObj(new ob::MaximizeMinClearanceObjective(si));
  ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
  opt->addObjective(lengthObj, 0.0);
  opt->addObjective(clearObj, 1.0);
  return ob::OptimizationObjectivePtr(opt);
}

void PostRunEvent(const ob::PlannerPtr &planner, ot::Benchmark::RunProperties &run)
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

StrategyGeometricMultiLevel::StrategyGeometricMultiLevel()
{
}

ob::PlannerPtr StrategyGeometricMultiLevel::GetPlanner(std::string algorithm,
    std::vector<ob::SpaceInformationPtr> si_vec, 
    std::vector<ob::ProblemDefinitionPtr> pdef_vec)

{
  ob::PlannerPtr planner;
  const ob::SpaceInformationPtr si = si_vec.back();

  if(algorithm=="ompl:rrt") planner = std::make_shared<og::RRT>(si);
  else if(algorithm=="ompl:rrtconnect") planner = std::make_shared<og::RRTConnect>(si);
  else if(algorithm=="ompl:prrt") planner = std::make_shared<og::pRRT>(si);
  else if(algorithm=="ompl:rrtsharp") planner = std::make_shared<og::RRTsharp>(si);
  else if(algorithm=="ompl:rrtstar") planner = std::make_shared<og::RRTstar>(si);
  else if(algorithm=="ompl:rrtxstatic") planner = std::make_shared<og::RRTXstatic>(si);
  else if(algorithm=="ompl:informedrrtstar") planner = std::make_shared<og::InformedRRTstar>(si);
  else if(algorithm=="ompl:lazyrrt") planner = std::make_shared<og::LazyRRT>(si);
  else if(algorithm=="ompl:btrrt") planner = std::make_shared<og::BiTRRT>(si);
  else if(algorithm=="ompl:lbtrrt") planner = std::make_shared<og::LBTRRT>(si);

  else if(algorithm=="ompl:prm") planner = std::make_shared<og::PRM>(si);
  else if(algorithm=="ompl:prmstar") planner = std::make_shared<og::PRMstar>(si);
  else if(algorithm=="ompl:lazyprm") planner = std::make_shared<og::LazyPRM>(si);
  else if(algorithm=="ompl:lazyprmstar") planner = std::make_shared<og::LazyPRMstar>(si);
  else if(algorithm=="ompl:spars") planner = std::make_shared<og::SPARS>(si);
  else if(algorithm=="ompl:spars2") planner = std::make_shared<og::SPARStwo>(si);

  else if(algorithm=="ompl:cforest") planner = std::make_shared<og::CForest>(si);
  else if(algorithm=="ompl:sst") planner = std::make_shared<og::SST>(si);
  else if(algorithm=="ompl:pdst") planner = std::make_shared<og::PDST>(si);
  else if(algorithm=="ompl:stride") planner = std::make_shared<og::STRIDE>(si);
  else if(algorithm=="ompl:kpiece") planner = std::make_shared<og::KPIECE1>(si);
  else if(algorithm=="ompl:bkpiece") planner = std::make_shared<og::BKPIECE1>(si);
  else if(algorithm=="ompl:lbkpiece") planner = std::make_shared<og::LBKPIECE1>(si);
  else if(algorithm=="ompl:est") planner = std::make_shared<og::EST>(si);
  else if(algorithm=="ompl:biest") planner = std::make_shared<og::BiEST>(si);
  else if(algorithm=="ompl:sbl") planner = std::make_shared<og::SBL>(si);
  else if(algorithm=="ompl:psbl") planner = std::make_shared<og::pSBL>(si);
  else if(algorithm=="ompl:fmt") planner = std::make_shared<og::FMT>(si);
  else if(algorithm=="ompl:bfmt") planner = std::make_shared<og::BFMT>(si);
  else if(algorithm=="hierarchy:qmp_connect"){
    typedef og::MultiQuotient<og::QMPConnect> MultiQuotient;
    planner = std::make_shared<MultiQuotient>(si_vec, "QMPConnect");
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
  }else if(algorithm=="hierarchy:qmp_connect_rrt"){
    typedef og::MultiQuotient<og::QMPConnect, og::RRTBidirectional> MultiQuotient;
    planner = std::make_shared<MultiQuotient>(si_vec, "QMPConnect+RRT");
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
  }else if(algorithm=="hierarchy:qmp_connect_fast"){
    typedef og::MultiQuotient<og::QMPConnectFast, og::RRTBidirectional> MultiQuotient;
    planner = std::make_shared<MultiQuotient>(si_vec, "QMPConnectFast");
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
  }else if(algorithm=="hierarchy:qmp_rrt"){
    typedef og::MultiQuotient<og::QMP, og::RRTBidirectional> MultiQuotient;
    planner = std::make_shared<MultiQuotient>(si_vec, "QMP+RRT");
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
  }else if(algorithm=="hierarchy:qmp"){
    typedef og::MultiQuotient<og::QMP> MultiQuotient;
    planner = std::make_shared<MultiQuotient>(si_vec, "QMP");
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
  }else if(algorithm=="hierarchy:qcp"){
    typedef og::MultiChart<og::QCP> MultiChart;
    planner = std::make_shared<MultiChart>(si_vec, "QCP");
    static_pointer_cast<MultiChart>(planner)->setProblemDefinition(pdef_vec);
  }else{
    std::cout << "Planner algorithm " << algorithm << " is unknown." << std::endl;
    exit(0);
  }
  planner->setProblemDefinition(pdef_vec.back());
  return planner;

}

void StrategyGeometricMultiLevel::plan( const StrategyInput &input, StrategyOutput &output)
{
  std::string algorithm = input.name_algorithm;

  std::vector<ob::SpaceInformationPtr> si_vec; 
  std::vector<ob::ProblemDefinitionPtr> pdef_vec; 

  for(uint k = 0; k < input.cspace_levels.size(); k++){
    CSpaceOMPL* cspace_levelk = input.cspace_levels.at(k);
    ob::SpaceInformationPtr sik = cspace_levelk->SpaceInformationPtr();
    setStateSampler(input.name_sampler, sik);

    ob::ScopedState<> startk = cspace_levelk->ConfigToOMPLState(input.q_init);
    ob::ScopedState<> goalk  = cspace_levelk->ConfigToOMPLState(input.q_goal);

    //std::cout << *cspace_levelk << std::endl;

    ob::ProblemDefinitionPtr pdefk = std::make_shared<ob::ProblemDefinition>(sik);
    pdefk->addStartState(startk);
    auto goal=std::make_shared<ob::GoalState>(sik);
    goal->setState(goalk);
    goal->setThreshold(input.epsilon_goalregion);
    pdefk->setGoal(goal);
    pdefk->setOptimizationObjective( GetOptimizationObjective(sik) );

    si_vec.push_back(sik);
    pdef_vec.push_back(pdefk);
  }

  //###########################################################################
  // choose planner
  //###########################################################################

  if(util::StartsWith(algorithm,"benchmark")){
    RunBenchmark(input, si_vec, pdef_vec);
  }else{
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

    if(util::StartsWith(algorithm,"hierarchy")){
      planner->getPlannerData(*pd);
      output.SetPlannerData(pd);
    }else{
      planner->getPlannerData(*pd);
      output.SetPlannerData(pd);
    }
    //output->SetShortestPath( output.getShortestPathOMPL(), input.cspace_levels.back() );
    //std::vector<Config> path = output.GetShortestPath();
    output.SetProblemDefinition(planner->getProblemDefinition());
  }
}
void StrategyGeometricMultiLevel::RunBenchmark(
    const StrategyInput& input,
    std::vector<ob::SpaceInformationPtr> si_vec, 
    std::vector<ob::ProblemDefinitionPtr> pdef_vec)
{
  std::cout << input.name_algorithm << std::endl;
  BenchmarkInformation binfo(input.name_algorithm);

  const ob::SpaceInformationPtr si = si_vec.back();
  std::string file_benchmark = "benchmark_"+util::GetCurrentDateTimeString();
  og::SimpleSetup ss(si);
  ot::Benchmark benchmark(ss, "Benchmark");

  for(uint k = 0; k < binfo.algorithms.size(); k++){
    benchmark.addPlanner(GetPlanner(binfo.algorithms.at(k), si_vec, pdef_vec));
  }

  ob::ProblemDefinitionPtr pdef = pdef_vec.back();

  CSpaceOMPL *cspace = input.cspace_levels.back();
  ob::ScopedState<> start = cspace->ConfigToOMPLState(input.q_init);
  ob::ScopedState<> goal  = cspace->ConfigToOMPLState(input.q_goal);
  ss.setStartAndGoalStates(start,goal,input.epsilon_goalregion);

  ss.getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new SE3Project0r(ss.getStateSpace())));
  ss.setup();

  pdef->setOptimizationObjective( GetOptimizationObjective(si) );

  ot::Benchmark::Request req;
  req.maxTime = binfo.maxPlanningTime;
  req.maxMem = binfo.maxMemory;
  req.runCount = binfo.runCount;
  req.displayProgress = true;

  benchmark.setPostRunEvent(std::bind(&PostRunEvent, std::placeholders::_1, std::placeholders::_2));
  benchmark.benchmark(req);

  std::string res = file_benchmark+".log";
  std::string cmd;

  benchmark.saveResultsToFile(res.c_str());

  BenchmarkFileToPNG(file_benchmark);
}

