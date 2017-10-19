#include "planner/strategy/strategy_output.h"

StrategyOutput::StrategyOutput(CSpaceOMPL *cspace_):
  cspace(cspace_)
{
}
void StrategyOutput::SetPlannerData( ob::PlannerDataPtr pd_ ){
  pd = pd_;
  pd->decoupleFromPlanner();
  pd->computeEdgeWeights();
  roadmap = std::make_shared<Roadmap>();
  roadmap->CreateFromPlannerData(pd, cspace);
}
void StrategyOutput::SetProblemDefinition( ob::ProblemDefinitionPtr pdef_ ){
  pdef = pdef_;
}
void StrategyOutput::SetShortestPath( std::vector<Config> path_){
  shortest_path = path_;
}
ob::PlannerDataPtr StrategyOutput::GetPlannerDataPtr(){
  return pd;
}
ob::ProblemDefinitionPtr StrategyOutput::GetProblemDefinitionPtr(){
  return pdef;
}

std::vector<Config> StrategyOutput::PathGeometricToConfigPath(og::PathGeometric &path){
  og::PathSimplifier shortcutter(pdef->getSpaceInformation());
  shortcutter.shortcutPath(path);

  path.interpolate();

  std::vector<ob::State *> states = path.getStates();
  std::vector<Config> keyframes;
  for(uint i = 0; i < states.size(); i++)
  {
    ob::State *state = states.at(i);

    int N = cspace->GetDimensionality();
    Config cur = cspace->OMPLStateToConfig(state);
    if(N>cur.size()){
      Config qq;qq.resize(N);
      qq.setZero();
      for(int k = 0; k < cur.size(); k++){
        qq(k) = cur(k);
      }
      keyframes.push_back(qq);
    }else keyframes.push_back(cur);
  }

    //uint istep = max(int(keyframes.size()/10.0),1);
    //for(uint i = 0; i < keyframes.size(); i+=istep)
    //{
    //  std::cout << i << "/" << keyframes.size() << " : "  <<  keyframes.at(i) << std::endl;
    //}
    //std::cout << keyframes.size() << "/" << keyframes.size() << " : "  <<  keyframes.back() << std::endl;
  return keyframes;
}

std::vector<Config> StrategyOutput::GetShortestPath(){
  const ob::PathPtr p = pdef->getSolutionPath();
  std::vector<Config> path;
  if(p){
    og::PathGeometric gpath = static_cast<og::PathGeometric&>(*p);
    path = PathGeometricToConfigPath(gpath);
  }
  return path;
}

std::vector<std::vector<Config>> StrategyOutput::GetSolutionPaths(){
  solution_paths.clear();
  std::vector<ob::PlannerSolution> paths = pdef->getSolutions();
  for(uint k = 0; k < paths.size(); k++){
    og::PathGeometric gpath = static_cast<og::PathGeometric&>(*paths.at(k).path_);
    solution_paths.push_back( PathGeometricToConfigPath(gpath) );
  }
  return solution_paths;
}

RoadmapPtr StrategyOutput::GetRoadmapPtr(){
  return roadmap;
}

void StrategyOutput::SetRoadmap(RoadmapPtr roadmap_){
  roadmap = roadmap_;
}
