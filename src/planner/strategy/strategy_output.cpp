#include "planner/strategy/strategy_output.h"

StrategyOutput::StrategyOutput(CSpaceOMPL *cspace_):
  cspace(cspace_)
{
}
void StrategyOutput::SetPlannerData( ob::PlannerDataPtr pd_ ){
  pd = pd_;
  pd->decoupleFromPlanner();
  pd->computeEdgeWeights();
}
void StrategyOutput::SetShortestPath( std::vector<Config> path_){
  shortest_path = path_;
}
std::vector<Config> StrategyOutput::GetShortestPath(){
  return shortest_path;
}

Roadmap StrategyOutput::GetRoadmap(){
  roadmap.CreateFromPlannerData(pd, cspace);
  return roadmap;
}
void StrategyOutput::SetRoadmap(Roadmap roadmap_){
  roadmap = roadmap_;
}
