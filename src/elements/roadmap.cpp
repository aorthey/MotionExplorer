#include "roadmap.h"
#include "gui/common.h"
#include "common.h"
#include "planner/cspace/cspace_multiagent.h"
#include <ompl/geometric/planners/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace GLDraw;
using Graph = ob::PlannerData::Graph;
using Vertex = Graph::Vertex;

double sizeVertex{6};
double widthEdge{1};
double widthPath{25};

Roadmap::Roadmap()
{
}

Roadmap::Roadmap(const ob::PlannerDataPtr pd_, CSpaceOMPL *cspace_): 
  pd(pd_), cspace(cspace_)
{
  // std::cout << "roadmap from planner data with " << pd->numVertices() << " vertices and " << pd->numEdges() << " edges" << std::endl;
  path_ompl = GetShortestPath();
}
Roadmap::Roadmap(const ob::PlannerDataPtr pd_, CSpaceOMPL *cspace_, CSpaceOMPL *quotient_space_): 
  pd(pd_), cspace(cspace_), quotient_space(quotient_space_)
{
  // std::cout << "roadmap from planner data with " << pd->numVertices() << " vertices and " << pd->numEdges() << " edges" << std::endl;
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

void Roadmap::SetShortestPathOMPL(ob::PathPtr& path_ompl_ptr)
{
  path_ompl = new PathPiecewiseLinear(path_ompl_ptr, cspace, quotient_space);
}

PathPiecewiseLinear* Roadmap::GetShortestPath()
{
  if(path_ompl==nullptr)
  {
    if(pd==nullptr) return nullptr;

    if(pd->path_){
      path_ompl = new PathPiecewiseLinear(pd->path_, cspace, quotient_space);
    }else{

      LemonInterface lemon(pd);
      std::vector<Vertex> pred = lemon.GetShortestPath();

      ob::SpaceInformationPtr si = quotient_space->SpaceInformationPtr();

      ob::PathPtr path;
      if(quotient_space->isDynamic())
      {
        path = std::make_shared<oc::PathControl>(quotient_space->SpaceInformationPtr()); 
      }else{
        path = std::make_shared<og::PathGeometric>(quotient_space->SpaceInformationPtr()); 
      }

      shortest_path.clear();

      for(uint i = 0; i < pred.size(); i++)
      {
        Vertex pi = pred.at(i);
        ob::PlannerDataVertexAnnotated *v = dynamic_cast<ob::PlannerDataVertexAnnotated*>(&pd->getVertex(pi));
        const ob::State *s;
        if(v==nullptr){
          s = pd->getVertex(pi).getState();
        }else{
          s = v->getBaseState();
        }
        if(quotient_space->isDynamic()){
          static_pointer_cast<oc::PathControl>(path)->append(s);
        }else{
          static_pointer_cast<og::PathGeometric>(path)->append(s);
        }
        Vector3 q = quotient_space->getXYZ(s);
        if(draw_planar) q[2] = 0.0;
        shortest_path.push_back(q);
      }

      if(pred.size()>0){
        if(quotient_space->isDynamic())
        {
          // static_pointer_cast<oc::PathControl>(path)->interpolate();
          oc::PathControl cpath = static_cast<oc::PathControl&>(*path);
          std::vector<ob::State *> states = cpath.getStates();
        }else{
          static_pointer_cast<og::PathGeometric>(path)->interpolate();
        }
        path_ompl = new PathPiecewiseLinear(path, cspace, quotient_space);
      }
    }

  }
  return path_ompl;
}


Vector3 Roadmap::VectorFromVertex(const ob::PlannerDataVertex *v, int ridx)
{
  const ob::PlannerDataVertexAnnotated *va = dynamic_cast<const ob::PlannerDataVertexAnnotated*>(v);

  Vector3 q;

  if(va!=nullptr)
  {
    q = quotient_space->getXYZ(va->getBaseState(), ridx);
  }else{
    q = cspace->getXYZ(v->getState(), ridx);
  }

  if(draw_planar) q[2] = 0.0;
  return q;
}

void Roadmap::DrawGLRoadmapVertices(GUIState &state, int ridx)
{
  for(uint vidx = 0; vidx < pd->numVertices(); vidx++)
  {
    glPointSize(sizeVertex);
    setColor(cVertex);
    glPushMatrix();

    ob::PlannerDataVertex *vd = &pd->getVertex(vidx);
    
    Vector3 q = VectorFromVertex(vd, ridx);

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

    drawPoint(q);

    glPopMatrix();
  }
}

void Roadmap::DrawGLEdgeStateToState(CSpaceOMPL *space, const ob::State *s, const ob::State *t, int ridx)
{
    Vector3 a = space->getXYZ(s, ridx);
    Vector3 b = space->getXYZ(t, ridx);
    if(draw_planar) a[2] = b[2] = 0.0;
    drawLineSegment(a, b);
}

void Roadmap::DrawGLEdge(CSpaceOMPL *space, const ob::State *s, const ob::State *t, int ridx)
{
  ob::StateSpacePtr stateSpace = space->SpaceInformationPtr()->getStateSpace();
  DrawGLEdgeStateToState(space, s, t, ridx);

  // int nd = stateSpace->validSegmentCount(s, t);
  // if(nd <= 1)
  // {
  //     DrawGLEdgeStateToState(space, s, t, ridx);
  // }else
  // {
  //     stateSpace->copyState(stateTmpOld, s);
  //     for (int j = 1; j <= nd; j++)
  //     {
  //         stateSpace->interpolate(s, t, (double)j / (double)nd, stateTmpCur);

  //         DrawGLEdgeStateToState(space, stateTmpOld, stateTmpCur, ridx);

  //         stateSpace->copyState(stateTmpOld, stateTmpCur);
  //     }
  // }
}

void Roadmap::DrawGLRoadmapEdges(GUIState &state, int ridx)
{
  if(pd->numVertices() <= 0) return;

  glPushMatrix();
  glLineWidth(widthEdge);
  setColor(cEdge);

  //############################################################################
  //Get Space
  ob::PlannerDataVertex *v = &pd->getVertex(0);
  const ob::PlannerDataVertexAnnotated *va = dynamic_cast<const ob::PlannerDataVertexAnnotated*>(v);

  CSpaceOMPL *curSpace;
  if(va!=nullptr)
  {
      curSpace = quotient_space;
  }else{
      curSpace = cspace;
  }

  ob::StateSpacePtr stateSpace = curSpace->SpaceInformationPtr()->getStateSpace();
  stateTmpCur = stateSpace->allocState();
  stateTmpOld = stateSpace->allocState();
  //############################################################################

  for(uint vidx = 0; vidx < pd->numVertices(); vidx++)
  {
    ob::PlannerDataVertex *v = &pd->getVertex(vidx);
  //############################################################################
    const ob::PlannerDataVertexAnnotated *va = dynamic_cast<const ob::PlannerDataVertexAnnotated*>(v);
    const ob::State *vState;

    if(va!=nullptr)
    {
      vState = va->getBaseState();
    }else{
      vState = v->getState();
    }
  //############################################################################

    std::vector<uint> edgeList;
    pd->getEdges(vidx, edgeList);

    for(uint j = 0; j < edgeList.size(); j++){

      ob::PlannerDataVertex *w = &pd->getVertex(edgeList.at(j));
  //############################################################################
      const ob::PlannerDataVertexAnnotated *wa = dynamic_cast<const ob::PlannerDataVertexAnnotated*>(w);
      const ob::State *wState;
      if(wa!=nullptr)
      {
        wState = wa->getBaseState();
      }else{
        wState = w->getState();
      }
  //############################################################################

      DrawGLEdge(curSpace, vState, wState, ridx);

    }
  }
  stateSpace->freeState(stateTmpCur);
  stateSpace->freeState(stateTmpOld);
  glPopMatrix();
}

void Roadmap::drawLineWorkspaceStateToState(const ob::State *from, const ob::State *to, int ridx)
{
    Vector3 a = quotient_space->getXYZ(from, ridx);
    Vector3 b = quotient_space->getXYZ(to, ridx);
    if(draw_planar){
      a[2] = 0.0;
      b[2] = 0.0;
    }
    drawLineSegment(a, b);
}

void Roadmap::DrawGLPlannerData(GUIState &state)
{
  glEnable(GL_BLEND); 
  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  if(quotient_space->isMultiAgent()){
    CSpaceOMPLMultiAgent *cma = static_cast<CSpaceOMPLMultiAgent*>(quotient_space);
    std::vector<int> idxs = cma->GetRobotIdxs();
    foreach(int i, idxs)
    {
      if(state("draw_roadmap_vertices")) DrawGLRoadmapVertices(state, i);
      if(state("draw_roadmap_edges")) DrawGLRoadmapEdges(state, i);
    }
  }else{
    if(state("draw_roadmap_vertices")) DrawGLRoadmapVertices(state);
    if(state("draw_roadmap_edges")) DrawGLRoadmapEdges(state);
  }

  glEnable(GL_CULL_FACE);
  glEnable(GL_LIGHTING);
  glDisable(GL_BLEND); 
}

void Roadmap::DrawGL(GUIState& state)
{
  if(quotient_space != nullptr)
  {
    draw_planar = (quotient_space->IsPlanar());
    if(draw_planar && (quotient_space->GetFirstSubspace()->getType()==ob::STATE_SPACE_SE2) && state("planner_draw_spatial_representation_of_SE2")){
      draw_planar = false;
    }
  }

  if(pd!=nullptr) DrawGLPlannerData(state);
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

      ob::PlannerDataVertexAnnotated *v = dynamic_cast<ob::PlannerDataVertexAnnotated*>(&pd->getVertex(vidx));
      if(v==nullptr){
          subnode->SetAttribute("feasible", "no");
      }
      // if(v==nullptr){
      //   subnode->SetAttribute("feasible", "unknown");
      //   // subnode->SetAttribute("sufficient", "unknown");
      // }else{
      //   using FeasibilityType = ob::PlannerDataVertexAnnotated::FeasibilityType;
      //   FeasibilityType feasibility_t = v->GetFeasibility();
      //   if(feasibility_t == FeasibilityType::INFEASIBLE){
      //     subnode->SetAttribute("feasible", "no");
      //     // subnode->SetAttribute("sufficient", "no");
      //   }else{
      //     subnode->SetAttribute("feasible", "yes");
      //     // if(feasibility_t == FeasibilityType::SUFFICIENT_FEASIBLE){
      //     //   subnode->SetAttribute("sufficient", "yes");
      //     // }else{
      //     //   subnode->SetAttribute("sufficient", "no");
      //     // }
      //   }
      //   // double d = v->GetOpenNeighborhoodDistance();
      //   // subnode->SetDoubleAttribute("open_ball_radius", d);
      // }
      node->InsertEndChild(*subnode);
    }

  }

  return true;
}

