#include "planner/planner_benchmark.h"
#include <ompl/tools/benchmark/Benchmark.h>

namespace ot = ompl::tools;

MotionPlannerBenchmark::MotionPlannerBenchmark(RobotWorld *world_, PlannerMultiInput& input_):
  MotionPlanner(world_, *input_.inputs.at(0)), multi_input(input_)
{
}

void MotionPlannerBenchmark::Expand(){
  //ot::Benchmark benchmark(ss, "BenchmarkHumanoidWall");
  for(uint k = 0; k < multi_input.inputs.size(); k++){
    PlannerInput *in = multi_input.inputs.at(k);
    std::cout << "adding algorithm " << in->name_algorithm << std::endl;
  }
  exit(0);
  ////benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::PDST>(si)));
  ////benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::SST>(si)));
  ////benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::KPIECE1>(si)));
  //benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::RRT>(si)));

  //ot::Benchmark::Request req;
  //req.maxTime = duration;
  //req.maxMem = 10000.0;
  //req.runCount = 10;
  //req.displayProgress = true;

  //benchmark.setPostRunEvent(std::bind(&PostRunEventHumanoid, std::placeholders::_1, std::placeholders::_2, &cspace));

  //benchmark.benchmark(req);
  //benchmark.saveResultsToFile();

  //std::string file = "ompl_humanoid_benchmark_wall";
  //std::string res = file+".log";
  //benchmark.saveResultsToFile(res.c_str());

  //std::string cmd = "ompl_benchmark_statistics.py "+file+".log -d "+file+".db";
  //std::system(cmd.c_str());
  //cmd = "cp "+file+".db"+" ../data/benchmarks/";
  //std::system(cmd.c_str());
}

void MotionPlannerBenchmark::Collapse(){
}

void MotionPlannerBenchmark::Next(){
  return;
}
void MotionPlannerBenchmark::Previous(){
  return;
}
std::string MotionPlannerBenchmark::getName() const{
  return "Benchmark";
}
