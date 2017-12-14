#include "planner/strategy/strategy_geometric_multilevel.h"
#include "planner/strategy/ompl/rrt_plain.h"
#include "planner/strategy/ompl/prm_basic.h"
#include "planner/strategy/ompl/prm_multislice.h"
#include "planner/strategy/ompl/prm_slice.h"
#include "planner/strategy/ompl/prm_slice_connect.h"

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/goals/GoalState.h>
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
    typedef og::PRMMultiSlice<og::PRMSlice> MultiSlice;
    planner = std::make_shared<MultiSlice>(si_vec);
    static_pointer_cast<MultiSlice>(planner)->setProblemDefinition(pdef_vec);
  }else if(algorithm=="ompl:prm_multislice_connect"){
    typedef og::PRMMultiSlice<og::PRMSliceConnect> MultiSlice;
    planner = std::make_shared<MultiSlice>(si_vec, "Connect");
    static_pointer_cast<MultiSlice>(planner)->setProblemDefinition(pdef_vec);
  }else if(algorithm=="ompl:benchmark"){

    //### BENCHMARK #########################################################
    ot::Benchmark benchmark(ss, "BenchmarkGeometric");

    planner = std::make_shared<og::PRM>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);

    planner = std::make_shared<og::SBL>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);

    planner = std::make_shared<og::RRTConnect>(si_vec.back());
    planner->setProblemDefinition(pdef_vec.back());
    benchmark.addPlanner(planner);

    typedef og::PRMMultiSlice<og::PRMSlice> MultiSlice;
    planner = std::make_shared<MultiSlice>(si_vec);
    static_pointer_cast<MultiSlice>(planner)->setProblemDefinition(pdef_vec);
    benchmark.addPlanner(planner);

    ob::ProblemDefinitionPtr pdef = pdef_vec.back();
    CSpaceOMPL *cspace = input.cspace_levels.back();
    ob::ScopedState<> start = cspace->ConfigToOMPLState(p_init);
    ob::ScopedState<> goal  = cspace->ConfigToOMPLState(p_goal);
    ss.setStartAndGoalStates(start,goal,input.epsilon_goalregion);

    std::cout << "calling ss.setup()" << std::endl;
    ss.setup();

    pdef->setOptimizationObjective( getThresholdPathLengthObj(si) );

    ot::Benchmark::Request req;
    req.maxTime = 60;
    req.maxMem = 10000.0;
    req.runCount = 10;
    req.displayProgress = true;

    benchmark.benchmark(req);

    std::string file = "benchmark";
    std::string res = file+".log";
    std::string cmd;

    benchmark.saveResultsToFile(res.c_str());

    cmd = "ompl_benchmark_statistics.py "+file+".log -d "+file+".db";
    std::system(cmd.c_str());

    cmd = "cp "+file+".db"+" ../data/benchmarks/";
    std::system(cmd.c_str());

    cmd = "python ../scripts/ompl_output_benchmark.py "+file+".db";
    std::system(cmd.c_str());

    cmd = "python ../scripts/ompl_benchmark_statistics_simple.py "+file+".log -d "+file+".db -p "+file+".pdf";
    std::system(cmd.c_str());

    cmd = "pdf2png "+file+".pdf";
    std::system(cmd.c_str());

    cmd = "eog "+file+"-0.png";
    std::system(cmd.c_str());

    //exit(0);

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
