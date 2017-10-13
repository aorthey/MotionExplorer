#include "pathspace.h"
#include "algorithms/onetopic_reduction.h"

PathSpace::PathSpace(RobotWorld *world_, PathSpaceInput* input_):
  world(world_), input(input_)
{
  sv = NULL;
}

RobotWorld* PathSpace::GetWorldPtr(){
  return world;
}
PathSpaceInput* PathSpace::GetPathSpaceInput(){
  return input;
}

std::vector<Config> PathSpace::GetShortestPath(){
  return vantage_path;
}
std::vector<Config> PathSpace::GetVertices(){
  return vertices;
}
std::vector<std::pair<Config,Config>> PathSpace::GetEdges(){
  return edges;
}
std::vector<std::vector<Config>> PathSpace::GetPaths(){
  return paths;
}
Roadmap PathSpace::GetRoadmap(){
  return roadmap;
}

void PathSpace::SetEdges(const std::vector<std::pair<Config,Config>>& edges_){
  edges = edges_;
}
void PathSpace::SetShortestPath(const std::vector<Config>& path_){
  vantage_path = path_;
}
void PathSpace::SetVertices(const std::vector<Config>& vertices_){
  vertices = vertices_;
}
void PathSpace::SetPaths(const std::vector<std::vector<Config>>& paths_){
  paths = paths_;
}
void PathSpace::SetRoadmap(const Roadmap& roadmap_){
  roadmap = roadmap_;
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
  std::cout << " init        : " << space.input->q_init << std::endl;
  std::cout << " goal        : " << space.input->q_goal << std::endl;
  std::cout << " type        : " << space.input->type << std::endl;
  std::cout << " #shortest path vertices  : " << space.vantage_path.size() << std::endl;
  std::cout << " #graph vertices          : " << space.vertices.size() << std::endl;
  std::cout << " #graph edges             : " << space.edges.size() << std::endl;
  std::cout << " #paths size              : " << space.paths.size() << std::endl;
  //uint ii = space.input.robot_inner_idx;
  //uint io = space.input.robot_outer_idx;
  //std::cout << " robot inner : " << ii << " " << space.world->robots[ii]->name << std::endl;
  //std::cout << " robot outer : " << io << " " << space.world->robots[io]->name << std::endl;
  out << std::string(80, '-') << std::endl;
  return out;
}
