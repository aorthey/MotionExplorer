#include "roadmap.h"
#include "drawMotionPlanner.h"
#include "planner/validity_checker_ompl.h"

Roadmap::Roadmap(){
}

void Roadmap::CreateFromPlannerData(const ob::PlannerDataPtr pd, CSpaceOMPL *cspace_){
  cspace = cspace_;
  ob::SpaceInformationPtr si = pd->getSpaceInformation();
  std::cout << "roadmap from planner data with " << pd->numVertices() << " vertices and " << pd->numEdges() << " edges" << std::endl;
  OMPLValidityCheckerPtr validity_checker = std::static_pointer_cast<OMPLValidityChecker>(cspace->StateValidityCheckerPtr(si));

  for(uint i = 0; i < pd->numVertices(); i++){
    ob::PlannerDataVertex v = pd->getVertex(i);
    const ob::State* s = v.getState();

    Config q1 = cspace->OMPLStateToConfig(s);
    //ob::StateValidityCheckerPtr validity_checker = cspace->StateValidityCheckerPtr(si);


    V.push_back(q1);
    Vsufficient.push_back(validity_checker->isSufficient(s));

    std::vector<uint> edgeList;
    pd->getEdges(i, edgeList);
    for(int j = 0; j < edgeList.size(); j++){
      ob::PlannerDataVertex w = pd->getVertex(edgeList.at(j));
      const ob::State* sj = w.getState();
      Config q2 = cspace->OMPLStateToConfig(sj);
      Config qedge  = q2 - q1;
      E.push_back(std::make_pair(q1,q2));
      Esufficient.push_back(validity_checker->isSufficient(s) || validity_checker->isSufficient(sj));
    }
  }
}

void Roadmap::DrawGL(GUIState& state)
{
  GLColor green(0.2,0.9,0.2,0.5);
  GLColor red(0.9,0.2,0.2,0.5);
  GLColor magenta(0.9,0.1,0.9,0.5);

  GLColor sufficient = green;
  GLColor necessary = magenta;
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND); 

  glPointSize(10);
  std::cout << "roadmap: v=" << V.size() << " e="<<E.size() << std::endl;
  if(state("draw_roadmap_vertices")){
    for(uint k = 0; k < V.size(); k++){
      Config q = V.at(k);
      Vector3 v(q(0),q(1),q(2));
      if(Vsufficient.at(k)){
        if(state("draw_roadmap_sufficient")) setColor(sufficient);
      }else{
        if(state("draw_roadmap_necessary")) setColor(necessary);
      }
      drawPoint(v);
    }
  }
  glLineWidth(5);
  if(state("draw_roadmap_edges")){
    for(uint k = 0; k < E.size(); k++){
      Config q1 = E.at(k).first;
      Config q2 = E.at(k).second;
      Vector3 v1(q1(0),q1(1),q1(2));
      Vector3 v2(q2(0),q2(1),q2(2));
      if(Esufficient.at(k)){
        setColor(sufficient);
        if(state("draw_roadmap_sufficient")) drawLineSegment(v1,v2);
      }else{
        setColor(necessary);
        if(state("draw_roadmap_necessary")) drawLineSegment(v1,v2);
      }
    }
  }

  glEnable(GL_LIGHTING);
  glDisable(GL_BLEND); 
}

