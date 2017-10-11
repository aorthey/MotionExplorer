#include "strategy_output.h"

StrategyOutput::StrategyOutput(){
}
void StrategyOutput::SetPlannerData( ob::PlannerDataPtr pd_ ){
  pd = pd_;
  pd->decoupleFromPlanner();
  /// @todo{do that only when necessary}
  pd->computeEdgeWeights();
}
void StrategyOutput::SetShortestPath( std::vector<Config> path_){
  shortest_path = path_;
}
std::vector<Config> StrategyOutput::GetShortestPath(){
  return shortest_path;
}
