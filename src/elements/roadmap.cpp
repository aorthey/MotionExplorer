#include "roadmap.h"

#include "gui/drawMotionPlanner.h"
#include "gui/colors.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/validitychecker/validity_checker_ompl.h"
#include <boost/graph/dijkstra_shortest_paths.hpp>


using namespace GLDraw;
using Graph = ob::PlannerData::Graph;
using Vertex = Graph::Vertex;

Roadmap::Roadmap()
{
  cVertex = green;
  cEdge = green;
  cPath = magenta;
}
int Roadmap::numEdges()
{
  uint edge_ctr = 0;
  for(uint k = 0; k < roadmaps_level.size(); k++){
    edge_ctr += roadmaps_level.at(k)->numEdges();
  }
  return edge_ctr;
}
int Roadmap::numVertices()
{
  uint vertex_ctr = 0;
  for(uint k = 0; k < roadmaps_level.size(); k++){
    vertex_ctr += roadmaps_level.at(k)->numVertices();
  }
  return vertex_ctr;
}

void Roadmap::CreateFromPlannerData(const ob::PlannerDataPtr pd, CSpaceOMPL *cspace_){
  cspace = cspace_;
  ob::SpaceInformationPtr si = pd->getSpaceInformation();
  std::cout << "roadmap from planner data with " << pd->numVertices() << " vertices and " << pd->numEdges() << " edges" << std::endl;

  OMPLValidityCheckerPtr validity_checker = std::static_pointer_cast<OMPLValidityChecker>(cspace->StateValidityCheckerPtr());
  PlannerDataVertexAnnotated *v0 = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(0));
  if(v0==nullptr){
    roadmaps_level.push_back(pd);
    roadmaps_level.at(0)->decoupleFromPlanner();
    return;
  }
  uint N = v0->GetMaxLevel();
  std::cout << "max level: " << N << std::endl;

  for(uint k = 0; k < N; k++){
    ob::PlannerDataPtr pdi = std::make_shared<ob::PlannerData>(si);
    roadmaps_level.push_back(pdi);
  }

  for(uint i = 0; i < pd->numVertices(); i++){
    PlannerDataVertexAnnotated *v = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(i));

    ob::PlannerDataPtr pdi = roadmaps_level.at(v->GetLevel());

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

  for(uint k = 0; k < N; k++){
    ob::PlannerDataPtr pdi = roadmaps_level.at(k);
    pdi->decoupleFromPlanner();
    std::cout << "level " << k << " : " << pdi->numVertices() << " | " << pdi->numEdges() << std::endl;

    //std::vector<ob::PlannerData::Graph::Vertex> pred(pdi->numVertices());
    //const ob::PlannerData::Graph &graph = pdi->toBoostGraph();
    //const uint v = pdi->getStartIndex(0);

    //ob::PathLengthOptimizationObjective opt(si);

    //boost::dijkstra_shortest_paths(graph, v, boost::predecessor_map(&pred[0])
    //                                               .distance_compare([&opt](ob::Cost c1, ob::Cost c2)
    //                                                                 {
    //                                                                     return opt.isCostBetterThan(c1, c2);
    //                                                                 })
    //                                               .distance_combine([](ob::Cost, ob::Cost c)
    //                                                                 {
    //                                                                     return c;
    //                                                                 })
    //                                               .distance_inf(opt.infiniteCost())
    //                                               .distance_zero(opt.identityCost()));


    LemonInterface lemon(pdi);
    std::vector<Vertex> pred = lemon.GetShortestPath();
    std::vector<Vector3> path;
    for(uint i = 0; i < pred.size(); i++)
    {
      Vertex pi = pred.at(i);
      Vector3 q = cspace->getXYZ(pdi->getVertex(pi).getState());
      path.push_back(q);
    }
    shortest_path_level.push_back(path);
  }


}

void Roadmap::DrawPathGL(GUIState &state, std::vector<Vector3> &q)
{
  glPushMatrix();
  glLineWidth(10);
  setColor(cPath);
  for(uint k = 0; k < q.size()-1; k++){
    Vector3 v1 = q.at(k);
    Vector3 v2 = q.at(k+1);
    drawLineSegment(v1,v2);
  }
  glPopMatrix();
}

void Roadmap::DrawSingleLevelGL(GUIState &state, ob::PlannerDataPtr pd)
{
  if(state("draw_roadmap_vertices")){
    setColor(cVertex);
    for(uint vidx = 0; vidx < pd->numVertices(); vidx++){
      glPushMatrix();

      ob::PlannerDataVertex *vd = &pd->getVertex(vidx);
      Vector3 q = cspace->getXYZ(vd->getState());
      drawPoint(q);

      PlannerDataVertexAnnotated *v = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(vidx));
      if(v!=nullptr){
        if(state("draw_roadmap_volume")){
          glTranslate(q);
          double d = v->GetOpenNeighborhoodDistance();
          GUIVariable& v=state("draw_cover_threshold");
          if(v.type!=GUIVariable::SLIDER){
            std::cout << "error: not slider" << std::endl;
            exit(0);
          }
          if(d < v.value){
            drawSphere(d,16,8);
          }
        }
      }
      glPopMatrix();
    }
  }

  glLineWidth(5);
  if(state("draw_roadmap_edges")){
    setColor(cEdge);
    for(uint vidx = 0; vidx < pd->numVertices(); vidx++){
      ob::PlannerDataVertex *v = &pd->getVertex(vidx);
      std::vector<uint> edgeList;
      pd->getEdges(vidx, edgeList);
      for(uint j = 0; j < edgeList.size(); j++){
        ob::PlannerDataVertex *w = &pd->getVertex(edgeList.at(j));
        Vector3 v1 = cspace->getXYZ(v->getState());
        Vector3 v2 = cspace->getXYZ(w->getState());
        drawLineSegment(v1,v2);
      }
    }
  }
}

void Roadmap::DrawGL(GUIState& state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND); 

  glPointSize(10);

  uint N = roadmaps_level.size();
  for(uint k = 0; k < N; k++){

    std::string str = "roadmap_visualize_level_" + to_string(k);
    if(state(str.c_str())){
      if(!roadmaps_level.at(k)) return;
      //std::cout << "level " << k << " vertices: " << roadmaps_level.at(k)->numVertices() << std::endl;
      cVertex = (k%2==0?green:red);
      cEdge = (k%2==0?green:red);
      DrawSingleLevelGL(state, roadmaps_level.at(k));
      //DrawPathGL(state, shortest_path_level.at(k));
    }
  }

  glEnable(GL_LIGHTING);
  glDisable(GL_BLEND); 

  if(state("draw_roadmap_swathvolume")){
    if(!swv){
      std::vector<Config> q;
      ob::PlannerDataPtr pd = roadmaps_level.at(N-1);
      for(uint vidx = 0; vidx < pd->numVertices(); vidx++){
        PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(vidx));
        Config qi = cspace->OMPLStateToConfig(v.getState());
        q.push_back(qi);
      }
      swv = new SwathVolume(cspace->GetRobotPtr(), q);
    }
    swv->DrawGL(state);
  }
}

