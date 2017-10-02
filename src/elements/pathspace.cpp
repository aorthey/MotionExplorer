#include "pathspace.h"
#include "algorithms/onetopic_reduction.h"

PathSpace::PathSpace(){

}
PathSpace::PathSpace( ob::PlannerDataPtr pd_, CSpaceOMPL *cspace_):
  pd(pd_), cspace(cspace_)
{
  pd->decoupleFromPlanner();
}

void PathSpace::Decompose(){

  //get onetopic covers and their vantage paths
  OnetopicPathSpaceModifier onetopic_pathspace = OnetopicPathSpaceModifier(*pd, cspace);
  vantage_paths = onetopic_pathspace.GetConfigPaths();

  for(uint k = 0; k < vantage_paths.size(); k++){
    //compute all visible vertices
  }
}

int PathSpace::GetNumberOfDecompositions() const{
  return vantage_paths.size();  
}
const std::vector<std::vector<Config> > PathSpace::GetDecompositionVantagePaths() const{
  return vantage_paths;
}

const std::vector<Config> PathSpace::GetDecompositionVantagePath(uint i) const{
  return vantage_paths.at(i);
}

const std::vector<Config> PathSpace::GetDecompositionVertices() const{

}
