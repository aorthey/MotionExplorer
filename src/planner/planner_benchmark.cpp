#include "planner/planner_benchmark.h"

MotionPlannerBenchmark::MotionPlannerBenchmark(RobotWorld *world_, PlannerMultiInput& input_):
  MotionPlanner(world_, *input_.inputs.at(0))
{
}

void MotionPlannerBenchmark::Expand(){
}
void MotionPlannerBenchmark::Collapse(){
}

void MotionPlannerBenchmark::Next(){
  return;
}
void MotionPlannerBenchmark::Previous(){
  return;
}
