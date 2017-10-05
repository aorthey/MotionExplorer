#include "pathspace.h"
#include "algorithms/onetopic_reduction.h"

PathSpace::PathSpace(RobotWorld *world_, PlannerInput& input_):
  world(world_), input(input_)
{
  sv = NULL;
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
