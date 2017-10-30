#include "elements/roadmap_decorator.h"
#include "gui/drawMotionPlanner.h"
#include "gui/colors.h"

RoadmapDecoratorSE2::RoadmapDecoratorSE2(RoadmapPtr component_):
  component(component_){
}


void RoadmapDecoratorSE2::DrawGL(GUIState& state){
  using namespace GLDraw;
  ob::PlannerDataPtr pds = component->GetPlannerDataPtr();
  CSpaceOMPL *cspace = component->GetCSpacePtr();
    
  if(!pds) return;

  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND); 

  glPointSize(10);

  if(state("draw_roadmap_vertices")){
    setColor(cVertex);
    for(uint vidx = 0; vidx < component->numVertices(); vidx++){
      ob::PlannerDataVertex v = pds->getVertex(vidx);
      if(v!=ob::PlannerData::NO_VERTEX){
        Config q = cspace->OMPLStateToConfig(v.getState());
        if(q(3) < 0) q(3) += 2*M_PI;
        Vector3 v(q(0),q(1),q(3));
        drawPoint(v);
      }
    }
  }
  glLineWidth(5);
  if(state("draw_roadmap_edges")){
    setColor(cEdge);
    for(uint vidx = 0; vidx < component->numVertices(); vidx++){
      ob::PlannerDataVertex v = pds->getVertex(vidx);
      if(v==ob::PlannerData::NO_VERTEX) continue;
      Config q1 = cspace->OMPLStateToConfig(v.getState());

      std::vector<uint> edgeList;
      pds->getEdges(vidx, edgeList);
      for(uint j = 0; j < edgeList.size(); j++){
        ob::PlannerDataVertex w = pds->getVertex(edgeList.at(j));
        if(w==ob::PlannerData::NO_VERTEX) continue;
        Config q2 = cspace->OMPLStateToConfig(w.getState());
        if(q1(3) < 0) q1(3) += 2*M_PI;
        if(q2(3) < 0) q2(3) += 2*M_PI;

        double d = fabs(q1(3)-q2(3));
        if(d>M_PI){
          //needs to loop back onto itself
          if(q1(3)>q2(3)){
            Vector3 v1(q1(0),q1(1),q1(3));
            Vector3 v2(q2(0),q2(1),2*M_PI);
            drawLineSegment(v1,v2);
            Vector3 w1(q1(0),q1(1),q2(3));
            Vector3 w2(q2(0),q2(1),0);
            drawLineSegment(w1,w2);
          }else{
            Vector3 v1(q2(0),q2(1),q2(3));
            Vector3 v2(q1(0),q1(1),2*M_PI);
            drawLineSegment(v1,v2);
            Vector3 w1(q2(0),q2(1),q2(3));
            Vector3 w2(q1(0),q1(1),0);
            drawLineSegment(w1,w2);
          }
        }else{
          Vector3 v1(q1(0),q1(1),q1(3));
          Vector3 v2(q2(0),q2(1),q2(3));
          drawLineSegment(v1,v2);
        }
      }
    }
  }

  glEnable(GL_LIGHTING);
  glDisable(GL_BLEND); 
}
