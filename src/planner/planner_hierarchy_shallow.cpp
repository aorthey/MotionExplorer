#include "planner/planner_hierarchy_shallow.h"

ShallowHierarchicalMotionPlanner::ShallowHierarchicalMotionPlanner(RobotWorld *world_, PlannerInput& input_):
  HierarchicalMotionPlanner(world_,input_)
{
}
void ShallowHierarchicalMotionPlanner::ExpandPath(){
  if(!active) return;

  if(current_level==0){
    WorldPlannerSettings worldsettings;
    worldsettings.InitializeDefault(*world);

    CSpaceFactory factory(input);
    CSpaceOMPL* cspace;
    SingleRobotCSpace* kcspace;

    int robot_idx = input.robot_idx;
    Robot *robot = world->robots[robot_idx];

    kcspace = new SingleRobotCSpace(*world,robot_idx,&worldsettings);

    //if(!IsFeasible( robot, *kcspace, input.q_init)) return;
    //if(!IsFeasible( robot, *kcspace, input.q_goal)) return;

    cspace = factory.MakeGeometricCSpace(robot, kcspace);

    cspace->print();
    PlannerStrategyGeometric strategy;
    strategy.DisableOnetopicReduction();
    strategy.plan(input, cspace, output);

    output.robot = robot;

    if(output.success){
      current_level++;
      current_level_node=0;
      solution_path = output.GetKeyframes();
    }else{
      std::cout << "Error: Path could not be expanded" << std::endl;
    }
  }
}
void ShallowHierarchicalMotionPlanner::CollapsePath(){
  if(!active) return;

  if(current_level>0){
    current_level = 0;
    solution_path.clear();
  }
}

void ShallowHierarchicalMotionPlanner::NextPath(){
}
void ShallowHierarchicalMotionPlanner::PreviousPath(){
}


Robot* ShallowHierarchicalMotionPlanner::GetOriginalRobot(){
  return world->robots[input.robot_idx];
}
const Config ShallowHierarchicalMotionPlanner::GetOriginalInitConfig(){
  return input.q_init;
}
const Config ShallowHierarchicalMotionPlanner::GetOriginalGoalConfig(){
  return input.q_goal;
}
const std::vector<Config>& ShallowHierarchicalMotionPlanner::GetSelectedPath(){
  return solution_path;
}
std::vector< std::vector<Config> > ShallowHierarchicalMotionPlanner::GetSiblingPaths(){
  std::vector< std::vector<Config> > emptyset;
  return emptyset;
}
const SweptVolume& ShallowHierarchicalMotionPlanner::GetSelectedPathSweptVolume(){
  return output.GetSweptVolume();
}
Robot* ShallowHierarchicalMotionPlanner::GetSelectedPathRobot(){
  return world->robots[input.robot_idx];
}

const Config ShallowHierarchicalMotionPlanner::GetSelectedPathInitConfig(){
  return input.q_init;
}
const Config ShallowHierarchicalMotionPlanner::GetSelectedPathGoalConfig(){
  return input.q_goal;
}
const std::vector<int>& ShallowHierarchicalMotionPlanner::GetSelectedPathIndices(){
  const std::vector<int> emptyset;
  return emptyset;
}
