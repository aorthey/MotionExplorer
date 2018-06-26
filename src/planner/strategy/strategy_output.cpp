#include "planner/strategy/strategy_output.h"
#include "elements/plannerdata_vertex_annotated.h"

StrategyOutput::StrategyOutput(CSpaceOMPL *cspace_):
  cspace(cspace_)
{
}
void StrategyOutput::SetPlannerData( ob::PlannerDataPtr pd_ ){
  pd = pd_;
  pd->decoupleFromPlanner();
  pd->computeEdgeWeights();
}
void StrategyOutput::SetProblemDefinition( ob::ProblemDefinitionPtr pdef_ ){
  pdef = pdef_;
}
void StrategyOutput::SetShortestPath( std::vector<Config> path_){
  shortest_path = path_;
}
bool StrategyOutput::hasExactSolution(){
  return pdef->hasExactSolution();
}
bool StrategyOutput::hasApproximateSolution(){
  return pdef->hasApproximateSolution();
}
ob::PlannerDataPtr StrategyOutput::GetPlannerDataPtr(){
  return pd;
}
ob::ProblemDefinitionPtr StrategyOutput::GetProblemDefinitionPtr(){
  return pdef;
}

std::vector<Config> StrategyOutput::PathGeometricToConfigPath(og::PathGeometric &path){
  //og::PathSimplifier shortcutter(pdef->getSpaceInformation());
  //shortcutter.shortcutPath(path);
  //shortcutter.simplify(path,0.1);

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

  return keyframes;
}

ob::PathPtr StrategyOutput::getShortestPathOMPL(){

  ob::PathPtr path = pdef->getSolutionPath();
  if(cspace->isDynamic()){
    oc::PathControl cpath = static_cast<oc::PathControl&>(*path);
    cpath.interpolate();
  }else{

    og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
    gpath.interpolate();

    og::PathSimplifier shortcutter(pdef->getSpaceInformation());
    shortcutter.simplifyMax(gpath);
    shortcutter.smoothBSpline(gpath);

    bool valid = false;
    uint ctr = 0;

    while((!valid) && (ctr++ < 5)){
      const std::pair<bool, bool> &p = gpath.checkAndRepair(10);
      valid = p.second;
    }

    if(!gpath.check()){
      std::cout << "WARNING: path is not valid. Unsuccessfully tried to repair it for " << ctr-1 << " iterations." << std::endl;
    }
  }

  return path;
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

void StrategyOutput::GetHierarchicalRoadmap( HierarchicalRoadmapPtr hierarchy, std::vector<CSpaceOMPL*> cspace_levels)
{
  uint N = hierarchy->NumberLevels()-1;
  assert(N == cspace_levels.size());
  if(!pd){
    std::cout << "planner data not set." << std::endl;
    return;
  }

  std::vector<ob::PlannerDataPtr> pd_level;

  PlannerDataVertexAnnotated *v0 = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(0));

  if(v0==nullptr){
    pd_level.push_back(pd);
  }else{
    //only hierarchical data is annotated
    for(uint k = 0; k < N; k++){
      ob::PlannerDataPtr pdi = std::make_shared<ob::PlannerData>(cspace_levels.back()->SpaceInformationPtr());
      pd_level.push_back(pdi);
    }
    for(uint i = 0; i < pd->numVertices(); i++){
      PlannerDataVertexAnnotated *v = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(i));

      ob::PlannerDataPtr pdi = pd_level.at(v->GetLevel());

      if(pd->isStartVertex(i)){
        pdi->addStartVertex(*v);
      }else if(pd->isGoalVertex(i)){
        pdi->addGoalVertex(*v);
      }else{
        pdi->addVertex(*v);
      }

      std::vector<uint> edgeList;
      pd->getEdges(i, edgeList);

      for(uint j = 0; j < edgeList.size(); j++){
        PlannerDataVertexAnnotated *w = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(edgeList.at(j)));
        pdi->addVertex(*w);
        uint vi = pdi->vertexIndex(*v);
        uint wi = pdi->vertexIndex(*w);

        uint vo = pd->vertexIndex(*v);
        uint wo = pd->vertexIndex(*w);
        ob::PlannerDataEdge evw = pd->getEdge(vo,wo);
        ob::Cost weight;
        pd->getEdgeWeight(vo, wo, &weight);

        pdi->addEdge(vi, wi, evw, weight);
      }
    }
  }
  for(uint k = 0; k < N; k++){
    //RoadmapPtr roadmap_k = std::make_shared<Roadmap>(pd_level.at(k), cspace_levels.at(k));
    ob::PlannerDataPtr pdi = pd_level.at(k);
    pdi->decoupleFromPlanner();
    std::cout << "level " << k << " : " << pdi->numVertices() << " | " << pdi->numEdges() << std::endl;
    RoadmapPtr roadmap_k = std::make_shared<Roadmap>(pdi, cspace_levels.back());
    std::vector<int> path(k+1);
    hierarchy->UpdateNode( roadmap_k, path);
  }
}

std::ostream& operator<< (std::ostream& out, const StrategyOutput& so) 
{
  out << std::string(80, '-') << std::endl;
  out << "Planning Output" << std::endl;
  out << std::string(80, '-') << std::endl;
  out << " robot                : " << so.cspace->GetRobotPtr()->name << std::endl;
  out << " exact solution       : " << (so.pdef->hasExactSolution()? "Yes":"No")<< std::endl;
  out << " approximate solution : " << (so.pdef->hasApproximateSolution()? "Yes":"No")<< std::endl;
  double dg = so.pdef->getSolutionDifference();
  out << " solution difference  : " << dg << std::endl;
  out << " roadmap vertices     : " << so.pd->numVertices() << std::endl;
  out << " roadmap edges        : " << so.pd->numEdges() << std::endl;
  out << " planner time         : " << so.planner_time << std::endl;
  out << " max planner time     : " << so.max_planner_time << std::endl;
  out << std::string(80, '-') << std::endl;
  return out;
}
