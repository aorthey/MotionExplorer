#include "util.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/strategy/strategy_geometric.h"
#include "planner/benchmark/benchmark_input.h"
#include "planner/benchmark/benchmark_output.h"

#include "planner/strategy/quotient/multiquotient.h"
//#include "planner/strategy/quotientgraph/algorithms/qmp_connect.h"
//#include "planner/strategy/quotientgraph/algorithms/qmp.h"
//#include "planner/strategy/quotientgraph/algorithms/q_prm.h"
#include "planner/strategy/quotientgraph/algorithms/q_rrt.h"
#include "planner/strategy/quotientchart/multichart.h"
#include "planner/strategy/quotient/algorithms/qcp.h"
#include "planner/strategy/quotient/algorithms/qsampler.h"

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
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>

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

static uint all_runs{0};

void PostRunEvent(const ob::PlannerPtr &planner, ot::Benchmark::RunProperties &run)
{
  static uint pid = 0;

  ob::SpaceInformationPtr si = planner->getSpaceInformation();
  ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();

  bool solved = pdef->hasExactSolution();

  uint states = boost::lexical_cast<int>(run["graph states INTEGER"]);
  double time = boost::lexical_cast<double>(run["time REAL"]);
  double memory = boost::lexical_cast<double>(run["memory REAL"]);
  //uint states = boost::lexical_cast<int>(run["sampled states INTEGER"]);

  typedef og::MultiQuotient<og::QRRT> MultiQuotient;
  std::shared_ptr<MultiQuotient> qplanner = dynamic_pointer_cast<MultiQuotient>(planner);
  if(qplanner != nullptr){
    uint N = qplanner->GetLevels();
    std::vector<int> nodes = qplanner->GetNodes();
    std::vector<int> fnodes = qplanner->GetFeasibleNodes();
    std::vector<int> dimensions = qplanner->GetDimensionsPerLevel();
    run["stratification levels INTEGER"] = to_string(N);
    for(uint k = 0; k < nodes.size(); k++){
      std::string strd = "stratification level"+to_string(k)+" dimension INTEGER";
      run[strd] = to_string(dimensions.at(k));
      std::string strk = "stratification level"+to_string(k)+" nodes INTEGER";
      run[strk] = to_string(nodes.at(k));
      std::string strkf = "stratification level"+to_string(k)+" feasible nodes INTEGER";
      run[strkf] = to_string(fnodes.at(k));
    }
  }else{
    uint N = si->getStateDimension();
    run["stratification levels INTEGER"] = to_string(1);
    std::string strd = "stratification level"+to_string(0)+" dimension INTEGER";
    run[strd] = to_string(N);
    std::string strk = "stratification level"+to_string(0)+" nodes INTEGER";
    run[strk] = to_string(states);
    std::string strkf = "stratification level"+to_string(0)+" feasible nodes INTEGER";
    run[strkf] = to_string(states);
  }

  std::cout << "Run " << pid << "/" << all_runs << " [" << planner->getName() << "] " << (solved?"solved":"no solution") << "(time: "<< time << ", states: " << states << ", memory: " << memory << ")" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  pid++;

}

ob::PlannerPtr StrategyGeometricMultiLevel::GetPlanner(std::string algorithm,
  OMPLGeometricStratificationPtr stratification)
{
  ob::PlannerPtr planner;
  const ob::SpaceInformationPtr si = stratification->si_vec.back();
  const ob::ProblemDefinitionPtr pdef = stratification->pdef_vec.back();

  if(algorithm=="ompl:rrt") planner = std::make_shared<og::RRT>(si);
  else if(algorithm=="ompl:rrtconnect") planner = std::make_shared<og::RRTConnect>(si);
  else if(algorithm=="ompl:rrtsharp") planner = std::make_shared<og::RRTsharp>(si);
  else if(algorithm=="ompl:rrtstar") planner = std::make_shared<og::RRTstar>(si);
  else if(algorithm=="ompl:rrtxstatic") planner = std::make_shared<og::RRTXstatic>(si);
  else if(algorithm=="ompl:informedrrtstar") planner = std::make_shared<og::InformedRRTstar>(si);
  else if(algorithm=="ompl:lazyrrt") planner = std::make_shared<og::LazyRRT>(si);
  else if(algorithm=="ompl:trrt") planner = std::make_shared<og::TRRT>(si);
  else if(algorithm=="ompl:btrrt") planner = std::make_shared<og::BiTRRT>(si);
  else if(algorithm=="ompl:lbtrrt") planner = std::make_shared<og::LBTRRT>(si);
  else if(algorithm=="ompl:sorrtstar") planner = std::make_shared<og::SORRTstar>(si);

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
  else if(algorithm=="ompl:projest") planner = std::make_shared<og::ProjEST>(si);
  else if(algorithm=="ompl:sbl") planner = std::make_shared<og::SBL>(si);
  else if(algorithm=="ompl:fmt") planner = std::make_shared<og::FMT>(si);
  else if(algorithm=="ompl:bfmt") planner = std::make_shared<og::BFMT>(si);
  else if(algorithm=="ompl:bitstar"){
    std::cout << "Planner " << algorithm << " returns seg-fault. Removed." << std::endl;
    exit(0);
  }
  else if(algorithm=="ompl:prrt" || algorithm=="ompl:psbl"){
    std::cout << "Planner " << algorithm << " is returning infeasible paths and has been removed" << std::endl;
    exit(0);
  // }else if(algorithm=="hierarchy:qmp"){
  //   planner = GetSharedMultiQuotientPtr<og::QMP>(stratification);
  //   planner->setName("QMP");
  }else if(algorithm=="hierarchy:qcp"){
    planner = GetSharedMultiQuotientPtr<og::QCP>(stratification);
    planner->setName("QCP");
  }else if(algorithm=="hierarchy:q_rrt"){
    planner = GetSharedMultiQuotientPtr<og::QRRT>(stratification);
    planner->setName("QRRT");
  }else if(algorithm=="hierarchy:sampler"){
    planner = GetSharedMultiQuotientPtr<og::QSampler>(stratification);
    planner->setName("QSampler");
  }else{
    std::cout << "Planner algorithm " << algorithm << " is unknown." << std::endl;
    exit(0);
  }
  planner->setProblemDefinition(pdef);
  return planner;

}

template<typename T_Algorithm> 
ob::PlannerPtr StrategyGeometricMultiLevel::GetSharedMultiChartPtr( 
  OMPLGeometricStratificationPtr stratification)
{
  typedef og::MultiChart<T_Algorithm> MultiChart;
  ob::PlannerPtr planner = std::make_shared<MultiChart>(stratification->si_vec);
  static_pointer_cast<MultiChart>(planner)->setProblemDefinition(stratification->pdef_vec);
  return planner;
}

template<typename T_Algorithm> 
ob::PlannerPtr StrategyGeometricMultiLevel::GetSharedMultiQuotientPtr( 
  OMPLGeometricStratificationPtr stratification)
{
  typedef og::MultiQuotient<T_Algorithm> MultiQuotient;
  ob::PlannerPtr planner = std::make_shared<MultiQuotient>(stratification->si_vec);
  static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(stratification->pdef_vec);
  return planner;
}
template<typename T_Algorithm, typename T_Algorithm_Two> 
ob::PlannerPtr StrategyGeometricMultiLevel::GetSharedMultiQuotientPtr( 
  OMPLGeometricStratificationPtr stratification)
{
  typedef og::MultiQuotient<T_Algorithm, T_Algorithm_Two> MultiQuotient;
  ob::PlannerPtr planner = std::make_shared<MultiQuotient>(stratification->si_vec);
  static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(stratification->pdef_vec);
  return planner;
}

OMPLGeometricStratificationPtr StrategyGeometricMultiLevel::OMPLGeometricStratificationFromCSpaceStratification
(const StrategyInput &input, std::vector<CSpaceOMPL*> cspace_levels )
{
  std::vector<ob::SpaceInformationPtr> si_vec; 
  std::vector<ob::ProblemDefinitionPtr> pdef_vec; 

  for(uint k = 0; k < cspace_levels.size(); k++){
    CSpaceOMPL* cspace_levelk = cspace_levels.at(k);
    ob::SpaceInformationPtr sik = cspace_levelk->SpaceInformationPtr();
    setStateSampler(input.name_sampler, sik);

    ob::ScopedState<> startk = cspace_levelk->ConfigToOMPLState(input.q_init);
    ob::ScopedState<> goalk  = cspace_levelk->ConfigToOMPLState(input.q_goal);

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
  OMPLGeometricStratificationPtr stratification = std::make_shared<OMPLGeometricStratification>(si_vec, pdef_vec);
  return stratification;
}

void StrategyGeometricMultiLevel::Init( const StrategyInput &input )
{
  std::string algorithm = input.name_algorithm;

  if(util::StartsWith(algorithm,"benchmark")){
    //No Init, directly execute benchmark
    RunBenchmark(input);
  }else{
    OMPLGeometricStratificationPtr stratification = 
      OMPLGeometricStratificationFromCSpaceStratification(input, input.cspace_levels);
    planner = GetPlanner(algorithm, stratification);
    planner->setup();
    planner->clear();
    isInitialized = true;
  }
  max_planning_time = input.max_planning_time;
}

void StrategyGeometricMultiLevel::Step(StrategyOutput &output)
{
  ob::IterationTerminationCondition itc(1);
  ob::PlannerTerminationCondition ptc(itc);

  ompl::time::point start = ompl::time::now();
  planner->solve(ptc);
  output.planner_time = ompl::time::seconds(ompl::time::now() - start);
  output.max_planner_time = max_planning_time;

  //###########################################################################
  ob::PlannerDataPtr pd( new ob::PlannerData(planner->getSpaceInformation()) );
  planner->getPlannerData(*pd);
  output.SetPlannerData(pd);
  ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
  output.SetProblemDefinition(pdef);
}
//void StrategyGeometricMultiLevel::StepOneLevel(StrategyOutput &output)
//{
//  ompl::time::point start = ompl::time::now();
//  std::string algorithm = input.name_algorithm;
//  //###########################################################################
//
//  ob::IterationTerminationCondition itc(1);
//  ob::PlannerTerminationCondition ptc_step(itc);
//  ob::PlannerTerminationCondition ptc_time( ob::timedPlannerTerminationCondition(max_planning_time) );
//  ob::PlannerTerminationCondition ptc(plannerAndTerminationCondition(ptc_time, ptc_step));
//
//
//  planner->solve(ptc);
//
//  ob::PlannerDataPtr pd( new ob::PlannerData(planner->getSpaceInformation()) );
//  planner->getPlannerData(*pd);
//  output.SetPlannerData(pd);
//  ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
//  output.SetProblemDefinition(pdef);
//  //###########################################################################
//
//  output.planner_time = ompl::time::seconds(ompl::time::now() - start);
//  output.max_planner_time = max_planning_time;
//}

void StrategyGeometricMultiLevel::Clear()
{
  planner->clear();
}
void StrategyGeometricMultiLevel::Plan(StrategyOutput &output)
{
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max_planning_time) );
  ompl::time::point start = ompl::time::now();
  planner->solve(ptc);
  output.planner_time = ompl::time::seconds(ompl::time::now() - start);
  output.max_planner_time = max_planning_time;

  //###########################################################################

  ob::PlannerDataPtr pd( new ob::PlannerData(planner->getSpaceInformation()) );
  planner->getPlannerData(*pd);
  output.SetPlannerData(pd);
  ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
  output.SetProblemDefinition(pdef);

}

void StrategyGeometricMultiLevel::RunBenchmark(const StrategyInput& input)
{
  BenchmarkInput binput(input.name_algorithm);

  std::vector<OMPLGeometricStratificationPtr> stratifications;
  for(uint k = 0; k < input.cspace_stratifications.size(); k++){
    std::vector<CSpaceOMPL*> cspace_strat_k = input.cspace_stratifications.at(k);
    OMPLGeometricStratificationPtr stratification = OMPLGeometricStratificationFromCSpaceStratification(input, cspace_strat_k);
    stratifications.push_back(stratification);
  }
  if(stratifications.empty()) 
  {
    OMPL_INFORM("No stratifications specified. No algorithm is run");
    return;
  }
  uint k_largest_ambient_space = 0;
  uint largest_ambient_space_dimension = 0;
  for(uint k = 0; k < stratifications.size(); k++){
    const ob::SpaceInformationPtr sik = stratifications.at(k)->si_vec.back();
    uint dk = sik->getStateDimension();
    if(dk > largest_ambient_space_dimension){
      k_largest_ambient_space = k;
      largest_ambient_space_dimension = dk;
    }
  }

  std::cout << "Largest Ambient Space Dimension for Benchmark:" << largest_ambient_space_dimension << std::endl;
  const ob::SpaceInformationPtr si = stratifications.at(k_largest_ambient_space)->si_vec.back();
  const ob::ProblemDefinitionPtr pdef = stratifications.at(k_largest_ambient_space)->pdef_vec.back();


  std::string environment_name = util::GetFileBasename(input.environment_name);
  std::string file_benchmark = environment_name+"_"+util::GetCurrentDateTimeString();
  std::string output_file_without_extension = util::GetDataFolder()+"/benchmarks/"+file_benchmark;
  std::string log_file = output_file_without_extension+".log";
  std::string xml_file = output_file_without_extension+".xml";

  og::SimpleSetup ss(si);
  ot::Benchmark benchmark(ss, environment_name);

  uint planner_ctr = 0;
  for(uint k = 0; k < binput.algorithms.size(); k++){
    std::string name_algorithm = binput.algorithms.at(k);

    if(util::StartsWith(name_algorithm, "hierarchy")){
      for(uint i = 0; i < stratifications.size(); i++){
        const ob::SpaceInformationPtr sii = stratifications.at(i)->si_vec.back();
        uint di = sii->getStateDimension();
        bool shortStratification = (di < largest_ambient_space_dimension);
        if(shortStratification){
          std::cout << "algorithm " << name_algorithm << " is too short." << std::endl;
          stratifications.at(i)->si_vec.push_back(si);
          stratifications.at(i)->pdef_vec.push_back(pdef);
        }else{
          if(i != k_largest_ambient_space){
            //found another stratification with maximum ambient space. we need
            //to swap  its last space out and swap the chosen one in. (otherwise
            //OMPL will claim that those two spaces are different)
            stratifications.at(i)->si_vec.erase(stratifications.at(i)->si_vec.end()-1);
            stratifications.at(i)->si_vec.push_back(si);
          }

        }
        ob::PlannerPtr planner_k_i = GetPlanner(binput.algorithms.at(k), stratifications.at(i));
        if(shortStratification){
          typedef og::MultiQuotient<og::QRRT> MultiQuotient;
          std::shared_ptr<MultiQuotient> qplanner = dynamic_pointer_cast<MultiQuotient>(planner_k_i);
          if(qplanner != nullptr){
            qplanner->SetStopLevel(stratifications.at(i)->si_vec.size()-1);
          }else{
            std::cout << "Detected " << di << "/" << largest_ambient_space_dimension << " dimensions." << std::endl;
            std::cout << "at algorithm: " << name_algorithm << std::endl;
            std::cout << "failed to cast" << std::endl;
            OMPL_INFORM("Could not cast algorithm");
            exit(0);
          }
        }

        std::string name_algorithm_strat = planner_k_i->getName()+"_(";

        std::vector<ob::SpaceInformationPtr> si_vec_k = stratifications.at(i)->si_vec;
        for(uint j = 0; j < si_vec_k.size(); j++){
          if(j>=si_vec_k.size()-1 && shortStratification) break;
          uint Nj = si_vec_k.at(j)->getStateDimension();
          name_algorithm_strat += std::to_string(Nj);
        }
        name_algorithm_strat += ")";

        planner_k_i->setName(name_algorithm_strat);
        std::cout << "adding planner with ambient space " << si_vec_k.back()->getStateDimension() << std::endl;
        benchmark.addPlanner(planner_k_i);
        planner_ctr++;
      }
    }else{
      benchmark.addPlanner(GetPlanner(binput.algorithms.at(k), stratifications.at(0)));
      planner_ctr++;
    }
  }

  //CSpaceOMPL *cspace = input.cspace_levels.back();
  CSpaceOMPL *cspace = input.cspace_stratifications.at(k_largest_ambient_space).back();

  ob::ScopedState<> start = cspace->ConfigToOMPLState(input.q_init);
  ob::ScopedState<> goal  = cspace->ConfigToOMPLState(input.q_goal);
  ss.setStartAndGoalStates(start, goal, input.epsilon_goalregion);

  //ss.getStateSpace()->registerProjection("SE3", ob::ProjectionEvaluatorPtr(new SE3Project0r(ss.getStateSpace())));
  ss.getStateSpace()->registerProjections();
  ss.setup();

  pdef->setOptimizationObjective( GetOptimizationObjective(si) );

  ot::Benchmark::Request req;
  req.maxTime = binput.maxPlanningTime;
  req.maxMem = binput.maxMemory;
  req.runCount = binput.runCount;
  req.useThreads = false;
  req.simplify = false;
  req.displayProgress = true;

  benchmark.setPostRunEvent(std::bind(&PostRunEvent, std::placeholders::_1, std::placeholders::_2));

  //############################################################################
  // Estimate time requirement
  //############################################################################
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "BENCHMARKING" << std::endl;

  double worst_case_time_estimate_in_seconds = planner_ctr*binput.runCount*binput.maxPlanningTime;
  double worst_case_time_estimate_in_minutes = worst_case_time_estimate_in_seconds/60.0;
  double worst_case_time_estimate_in_hours = worst_case_time_estimate_in_minutes/60.0;
  all_runs = planner_ctr * binput.runCount;
  std::cout << "Number of Runs             : " << planner_ctr * binput.runCount << std::endl;
  std::cout << "Worst-case time requirement: ";
  if(worst_case_time_estimate_in_hours < 1){
    if(worst_case_time_estimate_in_minutes < 1){
      std::cout << worst_case_time_estimate_in_seconds << "s" << std::endl;
    }else{
      std::cout << worst_case_time_estimate_in_minutes << "m" << std::endl;
    }
  }else{
    std::cout << worst_case_time_estimate_in_hours << "h" << std::endl;
  }
  std::cout << std::string(80, '-') << std::endl;
  //############################################################################

  benchmark.benchmark(req);
  benchmark.saveResultsToFile(log_file.c_str());

  //BenchmarkFileToPNG(file_benchmark);
  BenchmarkOutput boutput(benchmark.getRecordedExperimentData());
  boutput.Save(xml_file.c_str());
  //boutput.PrintPDF();
}

