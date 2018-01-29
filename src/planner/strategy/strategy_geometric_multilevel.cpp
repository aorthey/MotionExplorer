#include "planner/strategy/strategy_geometric_multilevel.h"
#include "planner/strategy/ompl/multiquotient.h"

#include "planner/strategy/ompl/prm_basic.h"
#include "planner/strategy/ompl/prm_quotient.h"
#include "planner/strategy/ompl/prm_quotient_connect.h"
#include "planner/strategy/ompl/prm_quotient_narrowness.h"

#include "planner/strategy/ompl/rrt_plain.h"
#include "planner/strategy/ompl/rrt_quotient.h"
#include "planner/strategy/ompl/rrt_quotient_sufficiency.h"
#include "util.h"

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
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/Time.h>

void PostRunEvent(const ob::PlannerPtr &planner, ot::Benchmark::RunProperties &run)
{
  static uint pid = 0;

  //run["some extra property name INTEGER"] = "some value";
  // The format of added data is string key, string value pairs,
  // with the convention that the last word in string key is one of
  // REAL, INTEGER, BOOLEAN, STRING. (this will be the type of the field
  // when the log file is processed and saved as a database).
  // The values are always converted to string.
  ob::SpaceInformationPtr si = planner->getSpaceInformation();
  ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();


  bool solved = pdef->hasExactSolution();

  uint states = boost::lexical_cast<int>(run["graph states INTEGER"]);
  double time = boost::lexical_cast<double>(run["time REAL"]);
  double memory = boost::lexical_cast<double>(run["memory REAL"]);

  //if(!solved && states < 5){
  //  std::cout << "ERROR: Planner output has only " << states << std::endl;
  //  std::cout << "   this is an indicator of abnormal behavior." << std::endl;
  //  for (ot::Benchmark::RunProperties::iterator it=run.begin(); it!=run.end(); ++it)
  //  {
  //    std::cout << it->first << " => " << it->second << '\n';
  //  }
  //  exit(1);
  //}

  //if(solved){
  //  std::cout << "Found Solution at run " << pid << std::endl;
  //  util::PrintCurrentTime();
  //  const ob::PathPtr &pp = pdef->getSolutionPath();
  //  oc::PathControl path_control = static_cast<oc::PathControl&>(*pp);
  //  og::PathGeometric path = path_control.asGeometric();

  //  //og::PathSimplifier shortcutter(si);
  //  //shortcutter.shortcutPath(path);

  //  vector<Config> keyframes;
  //  for(int i = 0; i < path.getStateCount(); i++)
  //  {
  //    ob::State *state = path.getState(i);
  //    Config cur = OMPLStateToConfig(state, cspace->getPtr());
  //    keyframes.push_back(cur);
  //  }
  //  std::string sfile = "random_"+std::to_string(pid)+".xml";
  //  std::cout << "Saving keyframes"<< std::endl;
  //  Save(keyframes, sfile.c_str());
  //}else{
  //  std::cout << "Run " << pid << " no solution" << std::endl;

  //}
  std::cout << "Run " << pid << " " << (solved?"solved":"no solution") << "(time: "<< time << ", states: " << states << ", memory: " << memory << ")" << std::endl;
  pid++;

}

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
    setStateSampler(input.name_sampler, sik);

    ob::ScopedState<> startk = cspace_levelk->ConfigToOMPLState(p_init);
    ob::ScopedState<> goalk  = cspace_levelk->ConfigToOMPLState(p_goal);

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

  const ob::SpaceInformationPtr si = si_vec.back();
  og::SimpleSetup ss(si);

  //###########################################################################
  // choose planner
  //###########################################################################

  std::string file_benchmark = "benchmark_"+util::GetCurrentDateTimeString();
  ob::PlannerPtr planner;

  if(algorithm=="ompl:rrt_plain"){
    planner = std::make_shared<og::RRTPlain>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
  }else if(algorithm=="ompl:qmp_rrt"){
    //typedef og::MultiQuotient<og::PRMQuotientNarrowEdgeDegree> MultiQuotient;
    //typedef og::MultiQuotient<og::PRMQuotient, og::RRTQuotient> MultiQuotient;
    typedef og::MultiQuotient<og::RRTQuotient> MultiQuotient;
    planner = std::make_shared<MultiQuotient>(si_vec,"RRT");
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
  }else if(algorithm=="ompl:qmp_rrt_sufficiency"){
    typedef og::MultiQuotient<og::RRTQuotientSufficiency> MultiQuotient;
    planner = std::make_shared<MultiQuotient>(si_vec,"RRTSufficient");
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
  }else if(algorithm=="ompl:qmp"){
    typedef og::MultiQuotient<og::PRMQuotientNarrowEdgeDegree> MultiQuotient;
    planner = std::make_shared<MultiQuotient>(si_vec);
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
  // }else if(algorithm=="ompl:qmp_reject"){

  //   planner = std::make_shared<og::RRTQuotientRejectionSampling>(si_vec);
  //   static_pointer_cast<og::RRTQuotientRejectionSampling>(planner)->setProblemDefinition(pdef_vec);

  }else if(algorithm=="ompl:qmpconnect"){
    typedef og::MultiQuotient<og::PRMQuotientConnect> MultiQuotient;
    planner = std::make_shared<MultiQuotient>(si_vec, "Connect");
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
  }else if(algorithm=="ompl:qmpnarrow"){
    typedef og::MultiQuotient<og::PRMQuotientNarrow> MultiQuotient;
    planner = std::make_shared<MultiQuotient>(si_vec, "Narrow");
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
  }else if(algorithm=="ompl:qmpnarrow_edgedegree"){
    typedef og::MultiQuotient<og::PRMQuotientNarrowEdgeDegree> MultiQuotient;
    planner = std::make_shared<MultiQuotient>(si_vec, "NarrowEdgeDegree");
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
  }else if(algorithm=="ompl:qmpnarrow_mincut"){
    typedef og::MultiQuotient<og::PRMQuotientNarrowMinCut> MultiQuotient;
    planner = std::make_shared<MultiQuotient>(si_vec, "NarrowMinCut");
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
  }else if(algorithm=="ompl:benchmark_narrow"){
    //### BENCHMARK #########################################################
    ot::Benchmark benchmark(ss, "BenchmarkNarrowPassage");
    typedef og::MultiQuotient<og::PRMQuotientNarrowEdgeDegree> MultiQuotient;
    planner = std::make_shared<MultiQuotient>(si_vec);
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
    benchmark.addPlanner(planner);

    planner = std::make_shared<og::RRTConnect>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);

    ob::ProblemDefinitionPtr pdef = pdef_vec.back();
    CSpaceOMPL *cspace = input.cspace_levels.back();
    ob::ScopedState<> start = cspace->ConfigToOMPLState(p_init);
    ob::ScopedState<> goal  = cspace->ConfigToOMPLState(p_goal);
    ss.setStartAndGoalStates(start,goal,input.epsilon_goalregion);

    ss.setup();

    pdef->setOptimizationObjective( getThresholdPathLengthObj(si) );

    ot::Benchmark::Request req;
    req.maxTime = 120;
    req.maxMem = 10000.0;
    req.runCount = 5;
    req.displayProgress = true;

    benchmark.setPostRunEvent(std::bind(&PostRunEvent, std::placeholders::_1, std::placeholders::_2));
    benchmark.benchmark(req);

    std::string res = file_benchmark+".log";
    std::string cmd;

    benchmark.saveResultsToFile(res.c_str());

    BenchmarkFileToPNG(file_benchmark);

    exit(0);
  }else if(algorithm=="ompl:benchmark_initial"){
    //### BENCHMARK INITIAL #########################################################
    // TRY OUT ALL PLANNERS FOR SOME TIME T, but only single run to access which
    // ones are best suited
    ot::Benchmark benchmark(ss, "BenchmarkGeometricInitial");

    planner = std::make_shared<og::RRT>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);

    planner = std::make_shared<og::RRTConnect>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);
    planner = std::make_shared<og::RRTstar>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);
    planner = std::make_shared<og::FMT>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);
    planner = std::make_shared<og::BFMT>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);

    planner = std::make_shared<og::KPIECE1>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);
    planner = std::make_shared<og::BKPIECE1>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);

    planner = std::make_shared<og::EST>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);
    planner = std::make_shared<og::BiEST>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);
    planner = std::make_shared<og::STRIDE>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);

    planner = std::make_shared<og::SST>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);
    planner = std::make_shared<og::PDST>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);

    planner = std::make_shared<og::PRM>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);
    planner = std::make_shared<og::LazyPRM>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);
    planner = std::make_shared<og::SBL>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);

    typedef og::MultiQuotient<og::PRMQuotient> MultiQuotient;
    typedef og::MultiQuotient<og::PRMQuotientConnect> MultiQuotientConnect;
    typedef og::MultiQuotient<og::PRMQuotient, og::RRTQuotient> MultiQuotientRRT;

    planner = std::make_shared<MultiQuotientRRT>(si_vec,"RRT");
    static_pointer_cast<MultiQuotientRRT>(planner)->setProblemDefinition(pdef_vec);
    benchmark.addPlanner(planner);

    planner = std::make_shared<MultiQuotient>(si_vec);
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
    benchmark.addPlanner(planner);

    ob::ProblemDefinitionPtr pdef = pdef_vec.back();
    CSpaceOMPL *cspace = input.cspace_levels.back();
    ob::ScopedState<> start = cspace->ConfigToOMPLState(p_init);
    ob::ScopedState<> goal  = cspace->ConfigToOMPLState(p_goal);
    ss.setStartAndGoalStates(start,goal,input.epsilon_goalregion);

    ss.setup();

    pdef->setOptimizationObjective( getThresholdPathLengthObj(si) );

    ot::Benchmark::Request req;
    req.maxTime = 180;
    req.maxMem = 10000.0;
    req.runCount = 5;
    req.displayProgress = true;

    benchmark.benchmark(req);

    std::string res = file_benchmark+".log";

    benchmark.saveResultsToFile(res.c_str());
    BenchmarkFileToPNG(file_benchmark);

  }else if(algorithm=="ompl:benchmark"){

    //### BENCHMARK #########################################################
    ot::Benchmark benchmark(ss, "BenchmarkGeometric");

    //planner = std::make_shared<og::RRTConnect>(si_vec.back());
    //planner->setProblemDefinition(pdef_vec.back());
    //benchmark.addPlanner(planner);

    //planner = std::make_shared<og::BFMT>(si_vec.back());
    //planner->setProblemDefinition(pdef_vec.back());
    //benchmark.addPlanner(planner);

    //planner = std::make_shared<og::BFMT>(si_vec.back());
    //planner->setProblemDefinition(pdef_vec.back());
    //benchmark.addPlanner(planner);

    planner = std::make_shared<og::RRTConnect>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);

    planner = std::make_shared<og::PRM>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);

    typedef og::MultiQuotient<og::PRMQuotient> MultiQuotient;
    typedef og::MultiQuotient<og::PRMQuotientConnect> MultiQuotientConnect;

    planner = std::make_shared<MultiQuotient>(si_vec);
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
    benchmark.addPlanner(planner);

    planner = std::make_shared<MultiQuotientConnect>(si_vec);
    static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);
    benchmark.addPlanner(planner);

    ob::ProblemDefinitionPtr pdef = pdef_vec.back();
    CSpaceOMPL *cspace = input.cspace_levels.back();
    ob::ScopedState<> start = cspace->ConfigToOMPLState(p_init);
    ob::ScopedState<> goal  = cspace->ConfigToOMPLState(p_goal);
    ss.setStartAndGoalStates(start,goal,input.epsilon_goalregion);

    ss.setup();

    pdef->setOptimizationObjective( getThresholdPathLengthObj(si) );

    ot::Benchmark::Request req;
    req.maxTime = 30;
    req.maxMem = 10000.0;
    req.runCount = 10;
    req.displayProgress = true;

    benchmark.setPostRunEvent(std::bind(&PostRunEvent, std::placeholders::_1, std::placeholders::_2));
    benchmark.benchmark(req);

    std::string res = file_benchmark+".log";
    std::string cmd;

    benchmark.saveResultsToFile(res.c_str());

    BenchmarkFileToPNG(file_benchmark);

    exit(0);

    //### BENCHMARK #########################################################

  }else{
    std::cout << "Planner algorithm " << algorithm << " is unknown." << std::endl;
    exit(0);
  }
  planner->clear();

  //###########################################################################
  //SIMPLE SETUP
  //###########################################################################
  //ss.setStartState(start);
  //ss.setGoal(input.GetGoalPtr(si));

  //ss.setPlanner(planner);
  //ss.setup();
  ////ss.getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new SE3Project0r(ss.getStateSpace())));

  //ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
  //pdef->setOptimizationObjective( getThresholdPathLengthObj(si) );

  //double max_planning_time= input.max_planning_time;
  //ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max_planning_time) );

  //ompl::time::point t_start = ompl::time::now();
  //ob::PlannerStatus status = ss.solve(ptc);
  //output.planner_time = ompl::time::seconds(ompl::time::now() - t_start);
  //output.max_planner_time = max_planning_time;

  //ob::PlannerDataPtr pd( new ob::PlannerData(si) );
  //ss.getPlannerData(*pd);
  //output.SetPlannerData(pd);
  //output.SetProblemDefinition(pdef);

  //###########################################################################
  //  WITHOUT SimpleSetup
  //###########################################################################
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

  //###########################################################################

  ob::PlannerDataPtr pd( new ob::PlannerData(si_vec.back()) );
  planner->getPlannerData(*pd);

  output.SetPlannerData(pd);
  output.SetProblemDefinition(planner->getProblemDefinition());
}
void StrategyGeometricMultiLevel::BenchmarkFileToPNG(const std::string &file){
  std::string cmd;

  cmd = "ompl_benchmark_statistics.py "+file+".log -d "+file+".db";
  int s1 = std::system(cmd.c_str());

  cmd = "cp "+file+".db"+" ../data/benchmarks/";
  int s2 = std::system(cmd.c_str());

  cmd = "python ../scripts/ompl_output_benchmark.py "+file+".db";
  int s3 = std::system(cmd.c_str());

  cmd = "python ../scripts/ompl_benchmark_statistics_simple.py "+file+".log -d "+file+".db -p "+file+".pdf";
  int s4 = std::system(cmd.c_str());

  cmd = "convert -density 150 "+file+".pdf -trim -quality 100 "+file+".png";
  int s5 = std::system(cmd.c_str());

  cmd = "eog "+file+".png";
  int s6 = std::system(cmd.c_str());

  if(s1&s2&s3&s4&s5&s6){
    std::cout << "Successfully wrote benchmark to " << file << ".png" << std::endl;
  }else{
    std::cout << "benchmark to png failed" << std::endl;
  }
}
