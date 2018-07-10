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
  std::cout << "start index: " << pd->getStartIndex(0) << " goal: " << pd->getGoalIndex(0) << std::endl;
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
          setColor(cVertexComponentGoal);
        }else{
          setColor(cVertexComponentOut);
        }
        if(pd->isStartVertex(vidx))
        {
          glPointSize(2*sizeVertex);
          setColor(cVertexStart);
        }
        if(pd->isGoalVertex(vidx))
        {
          glPointSize(2*sizeVertex);
          setColor(cVertexGoal);
        }

        double d = v->GetOpenNeighborhoodDistance();

        if(state("draw_roadmap_infeasible_vertices"))
        {
          if(d<=0){
            setColor(red);
          }
          drawPoint(q);
        }else{
          if(d>0){
            drawPoint(q);
          }
        }

        if(state("draw_roadmap_volume")){
          glTranslate(q);
          if(quotient_space->GetDimensionality()<=2){
            Vector3 q2(0,0,1);
            drawCircle(q2, d);
          }else{
            drawSphere(d, 16, 8);
          }
        }
      }else{
        drawPoint(q);
      }

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
            setColor(cVertexComponentGoal);
          }else{
            setColor(cVertexComponentOut);
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

  glEnable(GL_LIGHTING);
  glDisable(GL_BLEND); 

}

