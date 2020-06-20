#include "util.h"
#include "planner/strategy/strategy_geometric.h"
#include "planner/benchmark/benchmark_input.h"
#include "planner/benchmark/benchmark_output.h"
#include "planner/strategy/infeasibility_sampler.h"

#include <ompl/geometric/planners/explorer/Explorer.h>
#include <ompl/geometric/planners/multilevel/QRRT.h>
#include <ompl/geometric/planners/multilevel/QRRTStar.h>
#include <ompl/geometric/planners/multilevel/QMP.h>
#include <ompl/geometric/planners/multilevel/QMPStar.h>
#include <ompl/geometric/planners/multilevel/SPQR.h>

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

static ob::OptimizationObjectivePtr GetOptimizationObjective(const ob::SpaceInformationPtr& si)
{
  ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
  ob::OptimizationObjectivePtr clearObj(new ob::MaximizeMinClearanceObjective(si));
  ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
  opt->addObjective(lengthObj, 1.0);
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

  else if(algorithm=="hierarchy:qrrt") planner = std::make_shared<og::QRRT>(siVec, "QRRT");
  else if(algorithm=="hierarchy:qrrtstar") planner = std::make_shared<og::QRRTStar>(siVec, "QRRTStar");
  else if(algorithm=="hierarchy:qmp") planner = std::make_shared<og::QMP>(siVec, "QMP");
  else if(algorithm=="hierarchy:qmpstar") planner = std::make_shared<og::QMPStar>(siVec, "QMPStar");
  else if(algorithm=="hierarchy:spqr") planner = std::make_shared<og::SPQR>(siVec, "SPQR");

  else if(algorithm=="hierarchy:explorer") planner = std::make_shared<og::MotionExplorer>(siVec, "Explorer");
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
      startk = cspace->ConfigToOMPLState(input.q_init);
      goalk  = cspace->ConfigToOMPLState(input.q_goal);
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
  unsigned int N = planner->getProblemDefinition()->getSolutionCount();
  if(N>0){
      ob::PlannerSolution solution = planner->getProblemDefinition()->getSolutions().at(0);
      pd->path_ = solution.path_;
  }
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
  const ob::ProblemDefinitionPtr pdef = stratifications.at(k_largest_ambient_space)->pdef;


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

        if(util::StartsWith(name_algorithm, "hierarchy:qmpk")){

          for ( unsigned int knn = 3; knn <= 19; knn += 2)
          {

            ob::PlannerPtr planner_k_i = GetPlanner("hierarchy:qmp", stratifications.at(i));
            planner_k_i->params().setParam("knn", std::to_string( knn ));

            planner_k_i->as<og::QMP>()->setK(knn);

            std::string name_algorithm_strat = planner_k_i->getName()+"_(";

            std::vector<ob::SpaceInformationPtr> si_vec_k = stratifications.at(i)->si_vec;
            for(uint j = 0; j < si_vec_k.size(); j++){
              if(j>=si_vec_k.size()-1 && shortStratification) break;
              uint Nj = si_vec_k.at(j)->getStateDimension();
              name_algorithm_strat += std::to_string(Nj);
              if(j < si_vec_k.size()-1) name_algorithm_strat += "|";
            }
            name_algorithm_strat += ")";

            name_algorithm_strat += "(k_" + std::to_string( knn );
            name_algorithm_strat += ")";

            //if(stratifications.size() > 1)
            {
              planner_k_i->setName(name_algorithm_strat);
            }
            std::cout << "adding planner with ambient space " << si_vec_k.back()->getStateDimension() << std::endl;
            benchmark.addPlanner(planner_k_i);
            planner_ctr++;
          }
          
        }
        else if(util::StartsWith(name_algorithm, "hierarchy:metric")){

          std::string algo_name = name_algorithm.replace(9,7,"");

          std::cout << "distance metric benchmark algorithm  :  " << algo_name << std::endl;

          
          ob::PlannerPtr planner_k_geodesic = GetPlanner(algo_name, stratifications.at(i));

          ob::PlannerPtr planner_k_shortestpath = GetPlanner(algo_name, stratifications.at(i));


          if(name_algorithm.find("qrrt") != std::string::npos){
            planner_k_geodesic->as<og::QRRT>()->setMetric("geodesic");
            planner_k_shortestpath->as<og::QRRT>()->setMetric("shortestpath");
          }
          else if(name_algorithm.find("qrrtstar") != std::string::npos){
            planner_k_geodesic->as<og::QRRTStar>()->setMetric("geodesic");
            planner_k_shortestpath->as<og::QRRTStar>()->setMetric("shortestpath");
          }
          else if(name_algorithm.find("qmp") != std::string::npos){
            planner_k_geodesic->as<og::QMP>()->setMetric("geodesic");
            planner_k_shortestpath->as<og::QMP>()->setMetric("shortestpath");
          }
          else if(name_algorithm.find("qmpstar") != std::string::npos){
            planner_k_geodesic->as<og::QMPStar>()->setMetric("geodesic");
            planner_k_shortestpath->as<og::QMPStar>()->setMetric("shortestpath");
          }
          else if(name_algorithm.find("spqr") != std::string::npos){
            planner_k_geodesic->as<og::SPQR>()->setMetric("geodesic");
            planner_k_shortestpath->as<og::SPQR>()->setMetric("shortestpath");
          }
          
          
          std::string name_algorithm_strat = planner_k_geodesic->getName() ;//+ "_(";

          std::vector<ob::SpaceInformationPtr> si_vec_k = stratifications.at(i)->si_vec;
          /*for (uint j = 0; j < si_vec_k.size(); j++)
          {
            if (j >= si_vec_k.size() - 1 && shortStratification)
              break;
            uint Nj = si_vec_k.at(j)->getStateDimension();
            name_algorithm_strat += std::to_string(Nj);
            if (j < si_vec_k.size() - 1)
              name_algorithm_strat += "|";
          }
          name_algorithm_strat += ")";*/

          std::string name_algorithm_1 = name_algorithm_strat + "(intrinsic)";
          std::string name_algorithm_2 = name_algorithm_strat + "(QS metric)";

          planner_k_geodesic->setName(name_algorithm_1);
          planner_k_shortestpath->setName(name_algorithm_2);
          
          std::cout << "adding planner with ambient space " << si_vec_k.back()->getStateDimension() << std::endl;
          
          
          benchmark.addPlanner(planner_k_geodesic);
          planner_ctr++;

          benchmark.addPlanner(planner_k_shortestpath);
          planner_ctr++;
        }
        else if(util::StartsWith(name_algorithm, "hierarchy:importance")){

          std::string algo_name = name_algorithm.replace(9,11,"");

          std::cout << "importance benchmark algorithm  :  " << algo_name << std::endl;

          
          ob::PlannerPtr planner_k_uniform = GetPlanner(algo_name, stratifications.at(i));

          ob::PlannerPtr planner_k_greedy = GetPlanner(algo_name, stratifications.at(i));

          ob::PlannerPtr planner_k_exponential = GetPlanner(algo_name, stratifications.at(i));


          if(name_algorithm.find("qrrt") != std::string::npos){
            planner_k_uniform->as<og::QRRT>()->setImportance("uniform");
            planner_k_greedy->as<og::QRRT>()->setImportance("greedy");
            planner_k_exponential->as<og::QRRT>()->setImportance("exponential");
          }
          else if(name_algorithm.find("qrrtstar") != std::string::npos){
            planner_k_uniform->as<og::QRRTStar>()->setImportance("uniform");
            planner_k_greedy->as<og::QRRTStar>()->setImportance("greedy");
            planner_k_exponential->as<og::QRRTStar>()->setImportance("exponential");
          }
          else if(name_algorithm.find("qmp") != std::string::npos){
            planner_k_uniform->as<og::QMP>()->setImportance("uniform");
            planner_k_greedy->as<og::QMP>()->setImportance("greedy");
            planner_k_exponential->as<og::QMP>()->setImportance("exponential");
          }
          else if(name_algorithm.find("qmpstar") != std::string::npos){
            planner_k_uniform->as<og::QMPStar>()->setImportance("uniform");
            planner_k_greedy->as<og::QMPStar>()->setImportance("greedy");
            planner_k_exponential->as<og::QMPStar>()->setImportance("exponential");
          }
          else if(name_algorithm.find("spqr") != std::string::npos){
            planner_k_uniform->as<og::SPQR>()->setImportance("uniform");
            planner_k_greedy->as<og::SPQR>()->setImportance("greedy");
            planner_k_exponential->as<og::SPQR>()->setImportance("exponential");
          }
          
          
          std::string name_algorithm_strat = planner_k_uniform->getName() ;//;//;//+ "_(";

          std::vector<ob::SpaceInformationPtr> si_vec_k = stratifications.at(i)->si_vec;
          /*for (uint j = 0; j < si_vec_k.size(); j++)
          {
            if (j >= si_vec_k.size() - 1 && shortStratification)
              break;
            uint Nj = si_vec_k.at(j)->getStateDimension();
            name_algorithm_strat += std::to_string(Nj);
            if (j < si_vec_k.size() - 1)
              name_algorithm_strat += "|";
          }
          name_algorithm_strat += ")";*/

          std::string name_algorithm_1 = name_algorithm_strat + "(uniform)";
          std::string name_algorithm_2 = name_algorithm_strat + "(greedy)";
          std::string name_algorithm_3 = name_algorithm_strat + "(exponential)";

          planner_k_uniform->setName(name_algorithm_1);
          planner_k_greedy->setName(name_algorithm_2);
          planner_k_exponential->setName(name_algorithm_3);
          
          std::cout << "adding planner with ambient space " << si_vec_k.back()->getStateDimension() << std::endl;
          
          
          benchmark.addPlanner(planner_k_uniform);
          planner_ctr++;

          benchmark.addPlanner(planner_k_greedy);
          planner_ctr++;

          benchmark.addPlanner(planner_k_exponential);
          planner_ctr++;
        }
        else if(util::StartsWith(name_algorithm, "hierarchy:GraphSampler")){

          std::string algo_name = name_algorithm.replace(9,13,"");

          std::cout << "Graph Sampler benchmark algorithm  :  " << algo_name << std::endl;

          
          ob::PlannerPtr planner_k_randomvertex = GetPlanner(algo_name, stratifications.at(i));

          ob::PlannerPtr planner_k_randomedge = GetPlanner(algo_name, stratifications.at(i));

          ob::PlannerPtr planner_k_randomdegreevertex = GetPlanner(algo_name, stratifications.at(i));


          if(name_algorithm.find("qrrt") != std::string::npos){
            planner_k_randomvertex->as<og::QRRT>()->setGraphSampler("randomvertex");
            planner_k_randomedge->as<og::QRRT>()->setGraphSampler("randomedge");
            planner_k_randomdegreevertex->as<og::QRRT>()->setGraphSampler("randomdegreevertex");
          }
          else if(name_algorithm.find("qrrtstar") != std::string::npos){
            planner_k_randomvertex->as<og::QRRTStar>()->setGraphSampler("randomvertex");
            planner_k_randomedge->as<og::QRRTStar>()->setGraphSampler("randomedge");
            planner_k_randomdegreevertex->as<og::QRRTStar>()->setGraphSampler("randomdegreevertex");
          }
          else if(name_algorithm.find("qmp") != std::string::npos){
            planner_k_randomvertex->as<og::QMP>()->setGraphSampler("randomvertex");
            planner_k_randomedge->as<og::QMP>()->setGraphSampler("randomedge");
            planner_k_randomdegreevertex->as<og::QMP>()->setGraphSampler("randomdegreevertex");
          }
          else if(name_algorithm.find("qmpstar") != std::string::npos){
            planner_k_randomvertex->as<og::QMPStar>()->setGraphSampler("randomvertex");
            planner_k_randomedge->as<og::QMPStar>()->setGraphSampler("randomedge");
            planner_k_randomdegreevertex->as<og::QMPStar>()->setGraphSampler("randomdegreevertex");
          }
          else if(name_algorithm.find("spqr") != std::string::npos){
            planner_k_randomvertex->as<og::SPQR>()->setGraphSampler("randomvertex");
            planner_k_randomedge->as<og::SPQR>()->setGraphSampler("randomedge");
            planner_k_randomdegreevertex->as<og::SPQR>()->setGraphSampler("randomdegreevertex");
          }
          
          
          std::string name_algorithm_strat = planner_k_randomvertex->getName(); //;//+ "_(";

          std::vector<ob::SpaceInformationPtr> si_vec_k = stratifications.at(i)->si_vec;
          /*for (uint j = 0; j < si_vec_k.size(); j++)
          {
            if (j >= si_vec_k.size() - 1 && shortStratification)
              break;
            uint Nj = si_vec_k.at(j)->getStateDimension();
            name_algorithm_strat += std::to_string(Nj);
            if (j < si_vec_k.size() - 1)
              name_algorithm_strat += "|";
          }
          name_algorithm_strat += ")";*/

          std::string name_algorithm_1 = name_algorithm_strat + "(randomvertex)";
          std::string name_algorithm_2 = name_algorithm_strat + "(randomedge)";
          std::string name_algorithm_3 = name_algorithm_strat + "(degreevertex)";

          planner_k_randomvertex->setName(name_algorithm_1);
          planner_k_randomedge->setName(name_algorithm_2);
          planner_k_randomdegreevertex->setName(name_algorithm_3);
          
          std::cout << "adding planner with ambient space " << si_vec_k.back()->getStateDimension() << std::endl;
          
          
          benchmark.addPlanner(planner_k_randomvertex);
          planner_ctr++;

          benchmark.addPlanner(planner_k_randomedge);
          planner_ctr++;

          benchmark.addPlanner(planner_k_randomdegreevertex);
          planner_ctr++;
        }
        else if(util::StartsWith(name_algorithm, "hierarchy:FeasiblePathRestriction")){

          std::string algo_name = name_algorithm.replace(9,24,"");

          std::cout << "distance metric benchmark algorithm  :  " << algo_name << std::endl;

          
          ob::PlannerPtr planner_k_true = GetPlanner(algo_name, stratifications.at(i));

          ob::PlannerPtr planner_k_false = GetPlanner(algo_name, stratifications.at(i));


          if(name_algorithm.find("qrrt") != std::string::npos){
            planner_k_true->as<og::QRRT>()->setFeasiblePathRestriction(true);
            planner_k_false->as<og::QRRT>()->setFeasiblePathRestriction(false);
          }
          else if(name_algorithm.find("qrrtstar") != std::string::npos){
            planner_k_true->as<og::QRRTStar>()->setFeasiblePathRestriction(true);
            planner_k_false->as<og::QRRTStar>()->setFeasiblePathRestriction(false);
          }
          else if(name_algorithm.find("qmp") != std::string::npos){
            planner_k_true->as<og::QMP>()->setFeasiblePathRestriction(true);
            planner_k_false->as<og::QMP>()->setFeasiblePathRestriction(false);
          }
          else if(name_algorithm.find("qmpstar") != std::string::npos){
            planner_k_true->as<og::QMPStar>()->setFeasiblePathRestriction(true);
            planner_k_false->as<og::QMPStar>()->setFeasiblePathRestriction(false);
          }
          else if(name_algorithm.find("spqr") != std::string::npos){
            planner_k_true->as<og::SPQR>()->setFeasiblePathRestriction(true);
            planner_k_false->as<og::SPQR>()->setFeasiblePathRestriction(false);
          }
          
          
          std::string name_algorithm_strat = planner_k_true->getName() ;//+ "_(";

          std::vector<ob::SpaceInformationPtr> si_vec_k = stratifications.at(i)->si_vec;
         /* for (uint j = 0; j < si_vec_k.size(); j++)
          {
            if (j >= si_vec_k.size() - 1 && shortStratification)
              break;
            uint Nj = si_vec_k.at(j)->getStateDimension();
            name_algorithm_strat += std::to_string(Nj);
            if (j < si_vec_k.size() - 1)
              name_algorithm_strat += "|";
          }
          name_algorithm_strat += ")";*/

          std::string name_algorithm_1 = name_algorithm_strat + "(Find Section)";
          std::string name_algorithm_2 = name_algorithm_strat + "(No Find Section)";

          planner_k_true->setName(name_algorithm_1);
          planner_k_false->setName(name_algorithm_2);
          
          std::cout << "adding planner with ambient space " << si_vec_k.back()->getStateDimension() << std::endl;
          
          
          benchmark.addPlanner(planner_k_true);
          planner_ctr++;

          benchmark.addPlanner(planner_k_false);
          planner_ctr++;
        }
        else {
          

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
      }
    }else{
      benchmark.addPlanner(GetPlanner(binput.algorithms.at(k), stratifications.at(0)));
      planner_ctr++;
    }
  }

  CSpaceOMPL *cspace = input.cspace_stratifications.at(k_largest_ambient_space).back();

  ob::ScopedState<> start = cspace->ConfigToOMPLState(input.q_init);
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

  //BenchmarkFileToPNG(file_benchmark);
}

