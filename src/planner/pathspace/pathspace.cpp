#include "planner/pathspace/pathspace.h"
#include "planner/pathspace/pathspace_input.h"
#include "algorithms/onetopic_reduction.h"
#include "elements/roadmap.h"
#include "planner/cspace/cspace.h"
#include "planner/cspace/cspace_factory.h"
#include "elements/swept_volume.h"
#include <Planning/RobotCSpace.h>
#include <ompl/base/PlannerData.h>

PathSpace::PathSpace(RobotWorld *world_, PathSpaceInput* input_):
  world(world_), input(input_)
{
  sv = NULL;
  roadmap = NULL;
  if(!input){
    std::cout << "Input does not exists" << std::endl;
    exit(0);
  }
}

RobotWorld* PathSpace::GetWorldPtr(){
  return world;
}
PathSpaceInput* PathSpace::GetPathSpaceInput(){
  return input;
}

void PathSpace::SetShortestPath(const ob::PathPtr p, CSpaceOMPL *cspace){
  path_ompl = new PathPiecewiseLinear(p, cspace);
}
PathPiecewiseLinear* PathSpace::GetShortestPath(){
  return path_ompl;
}

const RoadmapPtr PathSpace::GetRoadmap(){
  return roadmap;
}

void PathSpace::SetRoadmap(const RoadmapPtr roadmap_){
  roadmap = roadmap_;
}

SweptVolume& PathSpace::GetSweptVolume(Robot *robot){
  std::cout << "Needs reimplementation to OMPL pathptr" << std::endl;
  exit(0);
  //if(!sv){
  //  sv = new SweptVolume(robot, vantage_path, 0);
  //}
  return *sv;
}
std::ostream& operator<< (std::ostream& out, const PathSpace& space) 
{
  out << std::string(80, '-') << std::endl;
  out << "[PathSpace]" << std::endl;
  out << std::string(80, '-') << std::endl;
  out << " atomic      : " << (space.isAtomic()?"yes":"no") << std::endl;
  out << " init        : " << space.input->q_init << std::endl;
  out << " goal        : " << space.input->q_goal << std::endl;
  out << " type        : " << space.input->type << std::endl;
  out << std::string(80, '-') << std::endl;
  return out;
}
