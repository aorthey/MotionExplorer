#include "pathspace.h"
#include "algorithms/onetopic_reduction.h"

PathSpace::PathSpace(){

}
PathSpace::PathSpace( ob::PlannerDataPtr pd_, CSpaceOMPL *cspace_):
  pd(pd_), cspace(cspace_)
{
  pd->decoupleFromPlanner();
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

std::vector<PathSpace> PathSpace::Decompose(){

  //get onetopic covers and their vantage paths
  OnetopicPathSpaceModifier onetopic_pathspace = OnetopicPathSpaceModifier(*pd, cspace);
  std::vector<std::vector<Config>> vantage_paths = onetopic_pathspace.GetConfigPaths();

  std::vector<PathSpace> covering;
  for(uint k = 0; k < vantage_paths.size(); k++){
    PathSpace pk(pd,cspace);
    pk.SetShortestPath(vantage_paths.at(k));
    covering.push_back(pk);
  }
  return covering;
}

