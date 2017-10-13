#include "roadmap.h"
#include "drawMotionPlanner.h"

Roadmap::Roadmap(){
};

void Roadmap::CreateFromPlannerData(const ob::PlannerDataPtr pd, CSpaceOMPL *cspace_){
{
  cspace = cspace_;
  ob::SpaceInformationPtr si = pd->getSpaceInformation();
  std::cout << "roadmap from planner data with " << pd->numVertices() << " vertices and " << pd->numEdges() << " edges" << std::endl;

  for(uint i = 0; i < pd->numVertices(); i++){
    ob::PlannerDataVertex v = pd->getVertex(i);
    const ob::State* s = v.getState();

    Config q1 = cspace->OMPLStateToConfig(s);
    V.push_back(q1);

    std::vector<uint> edgeList;
    pd->getEdges(i, edgeList);
    for(int j = 0; j < edgeList.size(); j++){
      ob::PlannerDataVertex w = pd->getVertex(edgeList.at(j));
      const ob::State* sj = w.getState();
      Config q2 = cspace->OMPLStateToConfig(sj);
      Config qedge  = q2 - q1;
      E.push_back(std::make_pair(q1,q2));
    }

  }
}

}
void Roadmap::GLDraw(){
  GLColor green(0.2,0.9,0.2,0.5);
  GLColor red(0.9,0.2,0.2,0.5);
  GLColor magenta(0.9,0.1,0.9,0.5);
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND); 

  glPointSize(10);
  setColor(green);
  for(uint k = 0; k < V.size(); k++){
    Config q = V.at(k);
    Vector3 v(q(0),q(1),q(2));
    drawPoint(v);
  }
  glLineWidth(5);
  for(uint k = 0; k < E.size(); k++){
    Config q1 = E.at(k).first;
    Config q2 = E.at(k).second;
    Vector3 v1(q1(0),q1(1),q1(2));
    Vector3 v2(q2(0),q2(1),q2(2));
    drawLineSegment(v1,v2);
  }

  glEnable(GL_LIGHTING);
  glDisable(GL_BLEND); 
}

