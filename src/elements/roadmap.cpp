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
  //std::cout << "roadmap from planner data with " << pd->numVertices() << " vertices and " << pd->numEdges() << " edges" << std::endl;
  path_ompl = GetShortestPath();
}
Roadmap::Roadmap(const ob::PlannerDataPtr pd_, CSpaceOMPL *cspace_, CSpaceOMPL *quotient_space_): 
  pd(pd_), cspace(cspace_), quotient_space(quotient_space_)
{
  //std::cout << "roadmap from planner data with " << pd->numVertices() << " vertices and " << pd->numEdges() << " edges" << std::endl;
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

void Roadmap::SetShortestPathOMPL(ob::PathPtr& path_ompl_ptr){
  path_ompl = new PathPiecewiseLinear(path_ompl_ptr, cspace, quotient_space);
}
PathPiecewiseLinear* Roadmap::GetShortestPath(){
  if(path_ompl==nullptr)
  {
    if(pd==nullptr) return nullptr;

    LemonInterface lemon(pd);
    std::vector<Vertex> pred = lemon.GetShortestPath();
    og::PathGeometric *gpath = new og::PathGeometric(quotient_space->SpaceInformationPtr()); 
    shortest_path.clear();

    for(uint i = 0; i < pred.size(); i++)
    {
      Vertex pi = pred.at(i);
      PlannerDataVertexAnnotated *v = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(pi));
      const ob::State *s;
      if(v==nullptr){
        s = pd->getVertex(pi).getState();
      }else{
        s = v->getQuotientState();
      }
      gpath->append(s);
      Vector3 q = quotient_space->getXYZ(s);
      shortest_path.push_back(q);
    }
    if(pred.size()>0){
      gpath->interpolate();
      ob::PathPtr path_ompl_ptr(gpath);
      path_ompl = new PathPiecewiseLinear(path_ompl_ptr, cspace, quotient_space);
    }
  }
  return path_ompl;
}

void Roadmap::DrawShortestPath(GUIState &state)
{
  glEnable(GL_BLEND); 
  glDisable(GL_LIGHTING);
  const std::vector<Vector3>& q = shortest_path;
  if(q.size()>1)
  {
    glPushMatrix();
    glLineWidth(widthPath);
    setColor(cPath);

    for(uint k = 0; k < q.size()-1; k++){
      Vector3 v1 = q.at(k);
      Vector3 v2 = q.at(k+1);
      if(quotient_space->GetDimensionality()<=2 || 
          quotient_space->SpaceInformationPtr()->getStateSpace()->getType()==ob::STATE_SPACE_SE2){
        double offset = +0.05;
        v1[2]+=offset;v2[2]+=offset;
      }
      drawLineSegment(v1,v2);
    }
    glPopMatrix();
  }
  glEnable(GL_LIGHTING);
  glDisable(GL_BLEND); 
}

void Roadmap::DrawPlannerData(GUIState &state)
{
  glEnable(GL_BLEND); 
  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  if(state("draw_roadmap_volume")){
    if(state("draw_roadmap_volume_wired")){
      wiredNeighborhood = true;
    }else{
      wiredNeighborhood = false;
    }
  }

  for(uint vidx = 0; vidx < pd->numVertices(); vidx++)
  {
    glPointSize(sizeVertex);
    glLineWidth(widthEdge);
    setColor(cVertex);
    glPushMatrix();

    ob::PlannerDataVertex *vd = &pd->getVertex(vidx);
    Vector3 q = cspace->getXYZ(vd->getState());

    PlannerDataVertexAnnotated *v = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(vidx));

    if(v!=nullptr)
    {
      q = quotient_space->getXYZ(v->getQuotientState());
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
      }else{
        if(pd->isGoalVertex(vidx))
        {
          glPointSize(2*sizeVertex);
          setColor(cVertexGoal);
        }
      }

      double d = v->GetOpenNeighborhoodDistance();

      if(state("draw_roadmap_vertices")){
        if(!v->IsInfeasible()){
          drawPoint(q);
        }
      }
      if(state("draw_roadmap_infeasible_vertices"))
      {
        if(v->IsInfeasible()){
          setColor(red);
          drawPoint(q);
        }
      }
      if(state("draw_roadmap_volume")){
        glTranslate(q);

        using FeasibilityType = PlannerDataVertexAnnotated::FeasibilityType;
        FeasibilityType feasibility_t = v->GetFeasibility();
        if(feasibility_t == FeasibilityType::SUFFICIENT_FEASIBLE){
          setColor(cNeighborhoodVolumeSufficient);
        }else{
          setColor(cNeighborhoodVolume);
        }

        ob::SpaceInformationPtr si = quotient_space->SpaceInformationPtr();

        if(quotient_space->GetDimensionality()<=2 || si->getStateSpace()->getType()==ob::STATE_SPACE_SE2){
          Vector3 q2(0,0,1);
          (wiredNeighborhood?drawWireCircle(q2, d):drawCircle(q2,d));
        }else{
          (wiredNeighborhood?drawWireSphere(d, 16):drawSphere(d, 16, 8));
        }
      }


    }else{
      if(state("draw_roadmap_vertices"))
      {
        drawPoint(q);
      }
    }

    glPopMatrix();
  }

  if(state("draw_roadmap_edges")){
    glPushMatrix();
    glLineWidth(widthEdge);
    setColor(cEdge);
    for(uint vidx = 0; vidx < pd->numVertices(); vidx++){
      ob::PlannerDataVertex *v = &pd->getVertex(vidx);
      Vector3 v1 = cspace->getXYZ(v->getState());

      PlannerDataVertexAnnotated *va = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(vidx));
      if(va!=nullptr) v1 = quotient_space->getXYZ(va->getQuotientState());

      std::vector<uint> edgeList;
      pd->getEdges(vidx, edgeList);
      for(uint j = 0; j < edgeList.size(); j++){
        ob::PlannerDataVertex *w = &pd->getVertex(edgeList.at(j));
        Vector3 v2 = cspace->getXYZ(w->getState());

        PlannerDataVertexAnnotated *wa = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(edgeList.at(j)));
        if(wa!=nullptr) v2 = quotient_space->getXYZ(wa->getQuotientState());
        if(va!=nullptr && wa!=nullptr){
          if(va->GetComponent()==0 || wa->GetComponent()==0){
            setColor(cEdge);
          }else if(va->GetComponent()==1 || wa->GetComponent()==1){
            setColor(cVertexComponentGoal);
          }else{
            setColor(cVertexComponentOut);
          }
        }
        drawLineSegment(v1,v2);
      }
    }
    glPopMatrix();
  }
  if(state("draw_roadmap_complex")){
    glPushMatrix();
    glLineWidth(widthEdge);
    setColor(cComplex);
    for(uint vidx = 0; vidx < pd->numVertices(); vidx++){

      PlannerDataVertexAnnotated *v1a = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(vidx));
      if(v1a==nullptr) break;
      std::vector<std::vector<long unsigned int>> simplices = v1a->GetComplex();
      //if(simplices.size()>0) std::cout << "vertex " << vidx << " has " << simplices.size() << " simplices." << std::endl;

      for(uint i = 0; i < simplices.size(); i++){
        const std::vector<long unsigned int>& svertices = simplices.at(i);

        uint K = svertices.size();
        std::vector<Vector3> pvec;
        for(uint k = 0; k < K; k++){
          ob::PlannerDataVertex *vk = &pd->getVertex(svertices.at(k));
          Vector3 p = quotient_space->getXYZ(vk->getState());
          pvec.push_back(p);
        }
        switch (pvec.size()) {
          case 2:
            {
              drawLineSegment(pvec.at(0), pvec.at(1));
              break;
            }
          case 3:
            {
              setColor(cComplex);
              drawTriangle(pvec.at(0), pvec.at(1), pvec.at(2));
              break;
            }
          case 4:
            {
              setColor(cComplexQuad);
              drawQuad(pvec.at(0), pvec.at(1), pvec.at(2), pvec.at(3));
              break;
            }
          default:
            {
              //std::cout << "cannot visualize simplex of dimensionality " << K << std::endl;
              break;
            }
                   
        }

      }

    }
    glPopMatrix();
  }
  glEnable(GL_CULL_FACE);
  glEnable(GL_LIGHTING);
  glDisable(GL_BLEND); 
}

void Roadmap::DrawGL(GUIState& state)
{
  if(pd!=nullptr) DrawPlannerData(state);
  if(state("draw_roadmap_shortest_path")){
    DrawShortestPath(state);
  }
}

bool Roadmap::Save(const char* fn)
{
  TiXmlDocument doc;
  TiXmlElement *node = CreateRootNodeInDocument(doc);
  Save(node);
  doc.LinkEndChild(node);
  doc.SaveFile(fn);
  return true;
}
bool Roadmap::Save(TiXmlElement *node)
{
  node->SetValue("roadmap");

  AddSubNode(*node, "num_vertices", numVertices());
  AddSubNode(*node, "num_edges", numEdges());

  {
    AddComment(*node, "vertices");
    ob::SpaceInformationPtr si = cspace->SpaceInformationPtr();
    ob::StateSpacePtr space = si->getStateSpace();

    for(uint vidx = 0; vidx < pd->numVertices(); vidx++){
      ob::PlannerDataVertex *vd = &pd->getVertex(vidx);
      std::vector<double> state_serialized;
      space->copyToReals(state_serialized, vd->getState());
      TiXmlElement *subnode = ReturnSubNodeVector(*node, "state", state_serialized);

      PlannerDataVertexAnnotated *v = dynamic_cast<PlannerDataVertexAnnotated*>(&pd->getVertex(vidx));
      if(v==nullptr){
        subnode->SetAttribute("feasible", "unknown");
        subnode->SetAttribute("sufficient", "unknown");
      }else{
        using FeasibilityType = PlannerDataVertexAnnotated::FeasibilityType;
        FeasibilityType feasibility_t = v->GetFeasibility();
        if(feasibility_t == FeasibilityType::INFEASIBLE){
          subnode->SetAttribute("feasible", "no");
          subnode->SetAttribute("sufficient", "no");
        }else{
          subnode->SetAttribute("feasible", "yes");
          if(feasibility_t == FeasibilityType::SUFFICIENT_FEASIBLE){
            subnode->SetAttribute("sufficient", "yes");
          }else{
            subnode->SetAttribute("sufficient", "no");
          }
        }
        double d = v->GetOpenNeighborhoodDistance();
        subnode->SetDoubleAttribute("open_ball_radius", d);
      }
      node->InsertEndChild(*subnode);
    }

  }

  return true;
}

