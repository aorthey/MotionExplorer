#include "util.h"
#include "planner/strategy/strategy_geometric.h"
#include "planner/benchmark/benchmark_input.h"
#include "planner/benchmark/benchmark_output.h"
#include "planner/strategy/infeasibility_sampler.h"
#include "planner/cspace/cspace_geometric_contact.h"

#include <ompl/multilevel/datastructures/PlannerMultiLevel.h>
#include <ompl/multilevel/planners/explorer/MotionExplorer.h>
#include <ompl/multilevel/planners/explorer/MotionExplorerQMP.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>
#include <ompl/multilevel/planners/qrrt/QRRTStar.h>
#include <ompl/multilevel/planners/qmp/QMP.h>
#include <ompl/multilevel/planners/qmp/QMPStar.h>
#include <ompl/multilevel/planners/spqr/SPQR.h>

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
#include <ompl/geometric/planners/prm/SPARStwo.h> //requires C++14
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
#include <ompl/geometric/planners/bitstar/ABITstar.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>

#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/Time.h>
#include <boost/lexical_cast.hpp>

namespace om = ompl::multilevel;
namespace og = ompl::geometric;
namespace ob = ompl::base;

static ob::OptimizationObjectivePtr GetOptimizationObjective(const ob::SpaceInformationPtr& si)
{
  ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
  ob::OptimizationObjectivePtr clearObj(new ob::MaximizeMinClearanceObjective(si));
  ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
  opt->addObjective(lengthObj, 1.0);
  // opt->addObjective(clearObj, 1.0);
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

  uint N = si->getStateDimension();
  run["stratification levels INTEGER"] = to_string(1);
  std::string strd = "stratification level"+to_string(0)+" dimension INTEGER";
  run[strd] = to_string(N);
  std::string strk = "stratification level"+to_string(0)+" nodes INTEGER";
  run[strk] = to_string(states);
  std::string strkf = "stratification level"+to_string(0)+" feasible nodes INTEGER";
  run[strkf] = to_string(states);

  std::cout << "Run " << pid << "/" << all_runs << " [" << planner->getName() << "] " << (solved?"solved":"no solution") << "(time: "<< time << ", states: " << states << ", memory: " << memory << ")" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  pid++;

}

ob::PlannerPtr StrategyGeometricMultiLevel::GetPlanner(std::string algorithm,
  OMPLGeometricStratificationPtr stratification)
{
  ob::PlannerPtr planner;
  const ob::SpaceInformationPtr si = stratification->si_vec.back();
  std::vector<ob::SpaceInformationPtr> siVec = stratification->si_vec;
  const ob::ProblemDefinitionPtr pdef = stratification->pdef;

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

  else if(algorithm=="ompl:bitstar") planner = std::make_shared<og::BITstar>(si);
  else if(algorithm=="ompl:abitstar") planner = std::make_shared<og::ABITstar>(si);
  else if(algorithm=="ompl:fmt") planner = std::make_shared<og::FMT>(si);
  else if(algorithm=="ompl:bfmt") planner = std::make_shared<og::BFMT>(si);

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

  else if(algorithm=="hierarchy:qrrt") planner = std::make_shared<om::QRRT>(siVec, "QRRT");
  else if(algorithm=="hierarchy:qrrtstar") planner = std::make_shared<om::QRRTStar>(siVec, "QRRTStar");
  else if(algorithm=="hierarchy:qmp") planner = std::make_shared<om::QMP>(siVec, "QMP");
  else if(algorithm=="hierarchy:qmpstar") planner = std::make_shared<om::QMPStar>(siVec, "QMPStar");
  else if(algorithm=="hierarchy:spqr") planner = std::make_shared<om::SPQR>(siVec, "SPQR");

  else if(algorithm=="hierarchy:explorer") planner = std::make_shared<om::MotionExplorer>(siVec, "Explorer");
  else if(algorithm=="hierarchy:explorer2") planner = std::make_shared<om::MotionExplorerQMP>(siVec, "ExplorerQMP");
  else if(algorithm=="sampler") planner = std::make_shared<og::InfeasibilitySampler>(si);

  else if(algorithm=="ompl:prrt" || algorithm=="ompl:psbl"){
    std::cout << "Planner " << algorithm << " is returning infeasible paths and has been removed" << std::endl;
    throw "Invalid planner.";
  }
  else{
    std::cout << "Planner algorithm " << algorithm << " is unknown." << std::endl;
    throw "Invalid planner.";
  }
  std::cout << "Planner algorithm " << planner->getName() << " initialized." << std::endl;
  planner->setProblemDefinition(pdef);
  return planner;

}

OMPLGeometricStratificationPtr StrategyGeometricMultiLevel::OMPLGeometricStratificationFromCSpaceStratification
(const StrategyInput &input, std::vector<CSpaceOMPL*> cspace_levels )
{
  std::vector<ob::SpaceInformationPtr> si_vec; 

  for(uint k = 0; k < cspace_levels.size(); k++)
  {
    CSpaceOMPL* cspace_levelk = cspace_levels.at(k);
    ob::SpaceInformationPtr sik = cspace_levelk->SpaceInformationPtr();
    setStateSampler(input.name_sampler, sik);
    si_vec.push_back(sik);
    std::cout << "CSPACE LEVEL" << k << " DIMENSION:" << cspace_levelk->GetDimensionality() << std::endl;
    // sik->printSettings();
  }

  CSpaceOMPL* cspace = cspace_levels.back();
  ob::SpaceInformationPtr sik = si_vec.back();

  ob::ScopedState<> startk(sik);
  ob::ScopedState<> goalk(sik);
  if(!cspace->isDynamic())
  {
      auto *cspaceContact  = dynamic_cast<GeometricCSpaceContact*>(cspace);
      if(cspaceContact!=nullptr)
      {
        cspaceContact->setInitialConstraints();
        startk = cspace->ConfigToOMPLState(input.q_init);
        cspaceContact->setGoalConstraints();
        goalk  = cspace->ConfigToOMPLState(input.q_goal);
        cspaceContact->setInitialConstraints();
      }else{
        startk = cspace->ConfigToOMPLState(input.q_init);
        goalk  = cspace->ConfigToOMPLState(input.q_goal);
      }
  }else{
      startk = cspace->ConfigVelocityToOMPLState(input.q_init, input.dq_init);
      goalk  = cspace->ConfigVelocityToOMPLState(input.q_goal, input.dq_goal);
  }


  ob::ProblemDefinitionPtr pdefk = std::make_shared<ob::ProblemDefinition>(sik);
  pdefk->addStartState(startk);
  auto goal=std::make_shared<ob::GoalState>(sik);
  goal->setState(goalk);
  goal->setThreshold(input.epsilon_goalregion);
  pdefk->setGoal(goal);
  pdefk->setOptimizationObjective( GetOptimizationObjective(sik) );
  OMPLGeometricStratificationPtr stratification = std::make_shared<OMPLGeometricStratification>(si_vec, pdefk);

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

void StrategyGeometricMultiLevel::Clear()
{
  if(isInitialized) planner->clear();
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

  auto mPlanner = dynamic_pointer_cast<om::PlannerMultiLevel>(planner);
  if(mPlanner != nullptr)
  {
      const std::vector<ob::ProblemDefinitionPtr> pdefVec = mPlanner->getProblemDefinitionVector();
      output.SetProblemDefinition(pdefVec);

  }else{
      ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
      output.SetProblemDefinition(pdef);
  }

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
  const ob::ProblemDefinitionPtr pdef = stratifications.at(k_largest_ambient_space)->pdef;


  std::string environment_name = util::GetFileBasename(input.environment_name);
  std::string file_benchmark = environment_name+"_"+util::GetCurrentDateTimeString();
  std::string output_file_without_extension = util::GetDataFolder()+"/benchmarks/"+file_benchmark;
  std::string log_file = output_file_without_extension+".log";
  std::string xml_file = output_file_without_extension+".xml";
  std::string xml_file_minimal = util::GetDataFolder()+"/benchmarks/"+environment_name+".xml";

  std::string cmd = std::string("cp ")+xml_file+std::string(" ")+xml_file_minimal;

  int rvalue = std::system(cmd.c_str());
  if(rvalue)
  {
    std::cout << "Copied xml file " << xml_file << " to " << xml_file_minimal << std::endl;
  }

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
          stratifications.at(i)->pdef = pdef;
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

        std::string name_algorithm_strat = planner_k_i->getName()+"_(";

        std::vector<ob::SpaceInformationPtr> si_vec_k = stratifications.at(i)->si_vec;
        for(uint j = 0; j < si_vec_k.size(); j++){
          if(j>=si_vec_k.size()-1 && shortStratification) break;
          uint Nj = si_vec_k.at(j)->getStateDimension();
          name_algorithm_strat += std::to_string(Nj);
          if(j < si_vec_k.size()-1) name_algorithm_strat += "|";
        }
        name_algorithm_strat += ")";

        if(stratifications.size() > 1)
        {
          planner_k_i->setName(name_algorithm_strat);
        }
        std::cout << "adding planner with ambient space " << si_vec_k.back()->getStateDimension() << std::endl;
        benchmark.addPlanner(planner_k_i);
        planner_ctr++;
      }
    }else{
      benchmark.addPlanner(GetPlanner(binput.algorithms.at(k), stratifications.at(0)));
      planner_ctr++;
    }
  }

  CSpaceOMPL *cspace = input.cspace_stratifications.at(k_largest_ambient_space).back();

  auto *cspaceContact  = dynamic_cast<GeometricCSpaceContact*>(cspace);
  if(cspaceContact!=nullptr)
  {
      cspaceContact->setInitialConstraints();
  }
  ob::ScopedState<> start = cspace->ConfigToOMPLState(input.q_init);
  if(cspaceContact!=nullptr)
  {
      cspaceContact->setGoalConstraints();
  }
  ob::ScopedState<> goal  = cspace->ConfigToOMPLState(input.q_goal);
  ss.setStartAndGoalStates(start, goal, input.epsilon_goalregion);

  ss.getStateSpace()->registerProjections();
  ss.setup();

  pdef->setOptimizationObjective( GetOptimizationObjective(si) );

  ot::Benchmark::Request req;
  req.maxTime = binput.maxPlanningTime;
  req.maxMem = binput.maxMemory;
  req.runCount = binput.runCount;
  // req.useThreads = false;
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
  std::cout << "Number of Planners           : " << planner_ctr << std::endl;
  std::cout << "Number of Runs Per Planner   : " << binput.runCount << std::endl;
  std::cout << "Time Per Run (s)             : " << binput.maxPlanningTime << std::endl;
  std::cout << "Worst-case time requirement  : ";

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

  BenchmarkOutput boutput(benchmark.getRecordedExperimentData());
  boutput.Save(xml_file.c_str());
  boutput.PrintPDF();

  boutput.Save(xml_file_minimal.c_str());
  boutput.PrintPDF();

  //BenchmarkFileToPNG(file_benchmark);
}

