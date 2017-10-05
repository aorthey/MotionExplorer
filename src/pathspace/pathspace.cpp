#include "pathspace.h"
#include "algorithms/onetopic_reduction.h"

PathSpace::PathSpace(RobotWorld *world_, PlannerInput& input_):
  world(world_), input(input_)
{
  sv = NULL;
}

RobotWorld* PathSpace::GetWorldPtr(){
  return world;
}
PlannerInput& PathSpace::GetPlannerInput(){
  return input;
}

std::vector<Config> PathSpace::GetShortestPath(){
  return vantage_path;
}
std::vector<Config> PathSpace::GetVertices(){
  return vertices;
}
void PathSpace::SetShortestPath(std::vector<Config> path_){
  vantage_path = path_;
}
void PathSpace::SetVertices(std::vector<Config> vertices_){
  vertices = vertices_;
}

SweptVolume& PathSpace::GetSweptVolume(Robot *robot){
  if(!sv){
    sv = new SweptVolume(robot, vantage_path, 0);
  }
  return *sv;
}
std::ostream& operator<< (std::ostream& out, const PathSpace& space) 
{
  out << std::string(80, '-') << std::endl;
  out << "[PathSpace]" << std::endl;
  out << std::string(80, '-') << std::endl;
  std::cout << " atomic      : " << (space.isAtomic()?"yes":"no") << std::endl;
  //uint ii = space.input.robot_inner_idx;
  //uint io = space.input.robot_outer_idx;
  //std::cout << " robot inner : " << ii << " " << space.world->robots[ii]->name << std::endl;
  //std::cout << " robot outer : " << io << " " << space.world->robots[io]->name << std::endl;
  out << std::string(80, '-') << std::endl;
  return out;
}
