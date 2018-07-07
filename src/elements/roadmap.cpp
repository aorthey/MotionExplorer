#include "roadmap.h"
#include "gui/common.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace GLDraw;
using Graph = ob::PlannerData::Graph;
using Vertex = Graph::Vertex;

Roadmap::Roadmap()
{
}

Roadmap::Roadmap(const ob::PlannerDataPtr pd_, CSpaceOMPL *cspace_): 
  pd(pd_), cspace(cspace_)
{
  std::cout << "roadmap from planner data with " << pd->numVertices() << " vertices and " << pd->numEdges() << " edges" << std::endl;
  path_ompl = GetShortestPath();
}
Roadmap::Roadmap(const ob::PlannerDataPtr pd_, CSpaceOMPL *cspace_, CSpaceOMPL *quotient_space_): 
  pd(pd_), cspace(cspace_), quotient_space(quotient_space_)
{
  std::cout << "roadmap from planner data with " << pd->numVertices() << " vertices and " << pd->numEdges() << " edges" << std::endl;
  path_ompl = GetShortestPath();
}

uint Roadmap::numEdges()
{
  if(pd == nullptr) return 0;
  return pd->numEdges();
}

uint Roadmap::numVertices()
{
  if(pd == nullptr) return 0;
  return pd->numVertices();
}

PathPiecewiseLinear* Roadmap::GetShortestPath(){
  if(path_ompl==nullptr)
  {
    if(pd==nullptr) return nullptr;

    LemonInterface lemon(pd);
    std::vector<Vertex> pred = lemon.GetShortestPath();
    og::PathGeometric *gpath = new og::PathGeometric(cspace->SpaceInformationPtr()); 
    shortest_path.clear();
    for(uint i = 0; i < pred.size(); i++)
    {
      Vertex pi = pred.at(i);
      const ob::State *s = pd->getVertex(pi).getState();
      gpath->append(s);
      Vector3 q = cspace->getXYZ(s);
      shortest_path.push_back(q);
    }
    gpath->interpolate();
    ob::PathPtr path_ompl_ptr(gpath);
    if(pred.size()>0){
      path_ompl = new PathPiecewiseLinear(path_ompl_ptr, cspace, quotient_space);
    }
  }
  return path_ompl;
}

//void Roadmap::CreateFromPlannerData(const ob::PlannerDataPtr pd, CSpaceOMPL *quotient_space_){
//  quotient_space = quotient_space_;
//  ob::SpaceInformationPtr si = pd->getSpaceInformation();
//  std::cout << "roadmap from planner data with " << pd->numVertices() << " vertices and " << pd->numEdges() << " edges" << std::endl;
//
//  OMPLValidityCheckerPtr validity_checker = std::static_pointer_cast<OMPLValidityChecker>(quotient_space->StateValidityCheckerPtr());
//  PlannerDataVertexAnnotated *v0 = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(0));
//
//  if(v0==nullptr){
//    //shallow hierarchy, take plannerdata as it is, add shortest path
//    roadmaps_level.push_back(pd);
//    roadmaps_level.at(0)->decoupleFromPlanner();
//    LemonInterface lemon(pd);
//    std::vector<Vertex> pred = lemon.GetShortestPath();
//    std::vector<Vector3> path;
//    for(uint i = 0; i < pred.size(); i++)
//    {
//      Vertex pi = pred.at(i);
//      Vector3 q = quotient_space->getXYZ(pd->getVertex(pi).getState());
//      path.push_back(q);
//    }
//    shortest_path_level.push_back(path);
//    return;
//  }
//
//  uint N = v0->GetMaxLevel();
//  std::cout << "max level: " << N << std::endl;
//
//  for(uint k = 0; k < N; k++){
//    ob::PlannerDataPtr pdi = std::make_shared<ob::PlannerData>(si);
//    roadmaps_level.push_back(pdi);
//  }
//
//  for(uint i = 0; i < pd->numVertices(); i++){
//    PlannerDataVertexAnnotated *v = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(i));
//
//    ob::PlannerDataPtr pdi = roadmaps_level.at(v->GetLevel());
//
//    if(pd->isStartVertex(i)){
//      pdi->addStartVertex(*v);
//    }else if(pd->isGoalVertex(i)){
//      pdi->addGoalVertex(*v);
//    }else{
//      pdi->addVertex(*v);
//    }
//
//    std::vector<uint> edgeList;
//    pd->getEdges(i, edgeList);
//
//    for(uint j = 0; j < edgeList.size(); j++){
//      PlannerDataVertexAnnotated *w = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(edgeList.at(j)));
//      pdi->addVertex(*w);
//      uint vi = pdi->vertexIndex(*v);
//      uint wi = pdi->vertexIndex(*w);
//
//      uint vo = pd->vertexIndex(*v);
//      uint wo = pd->vertexIndex(*w);
//      ob::PlannerDataEdge evw = pd->getEdge(vo,wo);
//      ob::Cost weight;
//      pd->getEdgeWeight(vo, wo, &weight);
//
//      pdi->addEdge(vi, wi, evw, weight);
//    }
//  }
//
//  for(uint k = 0; k < N; k++){
//    ob::PlannerDataPtr pdi = roadmaps_level.at(k);
//    pdi->decoupleFromPlanner();
//    std::cout << "level " << k << " : " << pdi->numVertices() << " | " << pdi->numEdges() << std::endl;
//
//    LemonInterface lemon(pdi);
//    std::vector<Vertex> pred = lemon.GetShortestPath();
//    std::vector<Vector3> path;
//
//    og::PathGeometric *gpath = new og::PathGeometric(quotient_space->SpaceInformationPtr()); 
//    for(uint i = 0; i < pred.size(); i++)
//    {
//      Vertex pi = pred.at(i);
//      const ob::State *s = pdi->getVertex(pi).getState();
//      gpath->append(s);
//      Vector3 q = quotient_space->getXYZ(s);
//      path.push_back(q);
//    }
//    shortest_path_level.push_back(path);
//    gpath->interpolate();
//    ob::PathPtr path_ompl_ptr(gpath);
//    if(pred.size()>0){
//      path_ompl = new PathPiecewiseLinear(path_ompl_ptr, quotient_space);
//    }
//  }
//
//}

void Roadmap::DrawShortestPath(GUIState &state)
{
  const std::vector<Vector3>& q = shortest_path;
  if(q.size()>1)
  {
    glPushMatrix();
    glLineWidth(widthPath);
    setColor(cPath);
    for(uint k = 0; k < q.size()-1; k++){
      Vector3 v1 = q.at(k);
      Vector3 v2 = q.at(k+1);
      drawLineSegment(v1,v2);
    }
    glPopMatrix();
  }
}

void Roadmap::DrawPlannerData(GUIState &state)
{
  if(state("draw_roadmap_vertices")){
    for(uint vidx = 0; vidx < pd->numVertices(); vidx++){
      glPointSize(sizeVertex);
      setColor(cVertex);
      glPushMatrix();

      ob::PlannerDataVertex *vd = &pd->getVertex(vidx);
      Vector3 q = cspace->getXYZ(vd->getState());
      PlannerDataVertexAnnotated *v = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(vidx));

      if(v!=nullptr){
        if(v->GetComponent()==0){
          setColor(cVertex);
        }else if(v->GetComponent()==1){
          setColor(cVertexGoal);
        }else{
          setColor(cVertexOut);
        }
        if(pd->isStartVertex(vidx))
        {
          glPointSize(2*sizeVertex);
          setColor(cVertex);
        }
        if(pd->isGoalVertex(vidx))
        {
          glPointSize(2*sizeVertex);
          setColor(cVertexGoal);
        }
        v->DrawGL(state);
      }
      drawPoint(q);
      glPopMatrix();
    }
  }

  if(state("draw_roadmap_edges")){
    glPushMatrix();
    glLineWidth(widthEdge);
    setColor(cEdge);
    for(uint vidx = 0; vidx < pd->numVertices(); vidx++){
      ob::PlannerDataVertex *v = &pd->getVertex(vidx);
      PlannerDataVertexAnnotated *va = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(vidx));

      std::vector<uint> edgeList;
      pd->getEdges(vidx, edgeList);
      for(uint j = 0; j < edgeList.size(); j++){
        ob::PlannerDataVertex *w = &pd->getVertex(edgeList.at(j));
        PlannerDataVertexAnnotated *wa = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(edgeList.at(j)));
        Vector3 v1 = cspace->getXYZ(v->getState());
        Vector3 v2 = cspace->getXYZ(w->getState());
        if(va==nullptr || wa==nullptr){
          drawLineSegment(v1,v2);
        }else{
          if(va->GetComponent()==0 || wa->GetComponent()==0){
            setColor(cEdge);
          }else if(va->GetComponent()==1 || wa->GetComponent()==1){
            setColor(cVertexGoal);
          }else{
            setColor(cVertexOut);
          }
          drawLineSegment(v1,v2);
        }
      }
    }
    glPopMatrix();
  }
}

void Roadmap::DrawGL(GUIState& state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND); 

  if(state("draw_roadmap_shortest_path")){
    DrawShortestPath(state);
  }
  if(pd!=nullptr) DrawPlannerData(state);


  //uint N = roadmaps_level.size();
  //for(uint k = 0; k < N; k++){

  //  std::string str = "roadmap_visualize_level_" + to_string(k);
  //  if(state(str.c_str())){
  //    if(!roadmaps_level.at(k)) return;
  //    //std::cout << "level " << k << " vertices: " << roadmaps_level.at(k)->numVertices() << std::endl;
  //    //cVertex = (k%2==0?green:lightred);
  //    //cEdge = cVertex;
  //    //cPath = (k%2==0?magenta:magenta);
  //    DrawSingleLevelGL(state, roadmaps_level.at(k));
  //    if(state("draw_roadmap_shortest_path")){
  //      DrawPathGL(state, shortest_path_level.at(k));
  //    }
  //  }
  //}

  //if(state("draw_roadmap_swathvolume")){
  //  if(!swv){
  //    std::vector<Config> q;
  //    ob::PlannerDataPtr pd = roadmaps_level.at(N-1);
  //    for(uint vidx = 0; vidx < pd->numVertices(); vidx++){
  //      PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(vidx));
  //      Config qi = quotient_space->OMPLStateToConfig(v.getState());
  //      q.push_back(qi);
  //    }
  //    swv = new SwathVolume(quotient_space->GetRobotPtr(), q);
  //  }
  //  swv->DrawGL(state);
  //}
  glEnable(GL_LIGHTING);
  glDisable(GL_BLEND); 

}

