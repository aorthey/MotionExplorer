#include "roadmap.h"
#include "gui/common.h"
#include "common.h"
#include "planner/cspace/cspace_multiagent.h"
#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace GLDraw;
using Graph = ob::PlannerData::Graph;
namespace om = ompl::multilevel;

double sizeVertex{6};
double widthEdge{1};
double widthPath{25};

Roadmap::Roadmap(const ob::PlannerDataPtr pd, std::vector<CSpaceOMPL*> cspace_levels):
  pd_(pd), cspace_levels_(cspace_levels)
{
}

uint Roadmap::numEdges()
{
  if(pd_ == nullptr) return 0;
  return pd_->numEdges();
}

uint Roadmap::numVertices()
{
  if(pd_ == nullptr) return 0;
  return pd_->numVertices();
}

Vector3 Roadmap::VectorFromVertex(const ob::PlannerDataVertex *v, int ridx)
{
  const om::PlannerDataVertexAnnotated *va = dynamic_cast<const om::PlannerDataVertexAnnotated*>(v);

  Vector3 q;

  CSpaceOMPL *cspace = cspace_levels_.at(current_level_);
  if(va!=nullptr)
  {
    int vLevel = va->getLevel();
    q = cspace_levels_.at(vLevel)->getXYZ(va->getBaseState(), ridx);
  }else{
    q = cspace->getXYZ(v->getState(), ridx);
  }

  if(draw_planar) q[2] = 0.0;
  return q;
}

void Roadmap::DrawGLRoadmapVertices(GUIState &state, int ridx)
{
  for(uint vidx = 0; vidx < pd_->numVertices(); vidx++)
  {
    ob::PlannerDataVertex *vd = &pd_->getVertex(vidx);
    const om::PlannerDataVertexAnnotated *va = dynamic_cast<const om::PlannerDataVertexAnnotated*>(vd);
    if(va != nullptr)
    {
      if((int)va->getLevel() != current_level_) continue;
    }

    glPointSize(sizeVertex);
    setColor(cVertex);
    glPushMatrix();

    
    Vector3 q = VectorFromVertex(vd, ridx);

    if(pd_->isStartVertex(vidx))
    {
      glPointSize(2*sizeVertex);
      setColor(cVertexStart);
    }else{
      if(pd_->isGoalVertex(vidx))
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
  // DrawGLEdgeStateToState(space, s, t, ridx);

  int nd = stateSpace->validSegmentCount(s, t);
  if(nd <= 1)
  {
      DrawGLEdgeStateToState(space, s, t, ridx);
  }else
  {
      stateSpace->copyState(stateTmpOld, s);
      for (int j = 1; j <= nd; j++)
      {
          stateSpace->interpolate(s, t, (double)j / (double)nd, stateTmpCur);

          DrawGLEdgeStateToState(space, stateTmpOld, stateTmpCur, ridx);

          stateSpace->copyState(stateTmpOld, stateTmpCur);
      }
  }
}

void Roadmap::DrawGLRoadmapEdges(GUIState &state, int ridx)
{
  if(pd_->numVertices() <= 0) return;

  glPushMatrix();
  glLineWidth(widthEdge);
  setColor(cEdge);

  CSpaceOMPL *cspace = cspace_levels_.at(current_level_);

  ob::StateSpacePtr stateSpace = cspace->SpaceInformationPtr()->getStateSpace();
  stateTmpCur = stateSpace->allocState();
  stateTmpOld = stateSpace->allocState();

  for(uint vidx = 0; vidx < pd_->numVertices(); vidx++)
  {
    ob::PlannerDataVertex *v = &pd_->getVertex(vidx);
    const om::PlannerDataVertexAnnotated *va = dynamic_cast<const om::PlannerDataVertexAnnotated*>(v);
    if(va != nullptr)
    {
      if((int)va->getLevel() != current_level_) continue;
    }

    const ob::State *vState;
    if(va!=nullptr)
    {
      vState = va->getBaseState();
    }else{
      vState = v->getState();
    }

    std::vector<uint> edgeList;
    pd_->getEdges(vidx, edgeList);

    for(uint j = 0; j < edgeList.size(); j++){

      ob::PlannerDataVertex *w = &pd_->getVertex(edgeList.at(j));
      const om::PlannerDataVertexAnnotated *wa = dynamic_cast<const om::PlannerDataVertexAnnotated*>(w);
      const ob::State *wState;
      if(wa!=nullptr)
      {
        wState = wa->getBaseState();
      }else{
        wState = w->getState();
      }
      DrawGLEdge(cspace, vState, wState, ridx);

    }
  }
  stateSpace->freeState(stateTmpCur);
  stateSpace->freeState(stateTmpOld);
  glPopMatrix();
}

void Roadmap::DrawGLPlannerData(GUIState &state)
{
  glEnable(GL_BLEND); 
  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  CSpaceOMPL* cspace = cspace_levels_.at(current_level_);
  if(cspace->isMultiAgent())
  {
    CSpaceOMPLMultiAgent *cma = static_cast<CSpaceOMPLMultiAgent*>(cspace);
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

void Roadmap::DrawGL(GUIState& state, int level)
{
  current_level_ = level;
  if(pd_!=nullptr) DrawGLPlannerData(state);
}

bool Roadmap::Save(const char* fn)
{
  if(numVertices() <= 0 && numEdges() <= 0) return false;

  TiXmlDocument doc;
  TiXmlElement *node = CreateRootNodeInDocument(doc);
  bool s = Save(node);
  doc.LinkEndChild(node);
  doc.SaveFile(fn);
  return s;
}
bool Roadmap::Save(TiXmlElement *node)
{
  node->SetValue("roadmap");

  AddSubNode(*node, "num_vertices", numVertices());
  AddSubNode(*node, "num_edges", numEdges());

  {
    AddComment(*node, "vertices");
    ob::SpaceInformationPtr si = cspace_levels_.back()->SpaceInformationPtr();
    ob::StateSpacePtr space = si->getStateSpace();

    for(uint vidx = 0; vidx < pd_->numVertices(); vidx++){
      om::PlannerDataVertexAnnotated *v = 
        dynamic_cast<om::PlannerDataVertexAnnotated*>(&pd_->getVertex(vidx));
      if(v != nullptr)
      {
        if((int)v->getLevel() != current_level_) continue;
      }

      ob::PlannerDataVertex *vd = &pd_->getVertex(vidx);
      std::vector<double> state_serialized;
      space->copyToReals(state_serialized, vd->getState());
      TiXmlElement *subnode = ReturnSubNodeVector(*node, "state", state_serialized);

      if(v==nullptr){
          subnode->SetAttribute("feasible", "yes");
      }
      node->InsertEndChild(*subnode);
    }

  }
  {
    AddComment(*node, "edges");
    ob::SpaceInformationPtr si = cspace_levels_.back()->SpaceInformationPtr();
    ob::StateSpacePtr space = si->getStateSpace();

    ob::State* stateTmp = space->allocState();

    for(uint vidx = 0; vidx < pd_->numVertices(); vidx++)
    {
      om::PlannerDataVertexAnnotated *va = 
        dynamic_cast<om::PlannerDataVertexAnnotated*>(&pd_->getVertex(vidx));
      if(va != nullptr)
      {
        if((int)va->getLevel() != current_level_) continue;
      }

      std::vector<unsigned int> edgeList;
      pd_->getEdges(vidx, edgeList);
      ob::PlannerDataVertex *v = &pd_->getVertex(vidx);
      for(uint k = 0; k < edgeList.size(); k++)
      {
          ob::PlannerDataVertex *vk = &pd_->getVertex(edgeList.at(k));

          TiXmlElement* node_edge = new TiXmlElement("edge");

          std::vector<double> state_serialized;

          const ob::State *s = v->getState();
          const ob::State *t = vk->getState();
          const int nd = space->validSegmentCount(s, t);
          if(nd <= 1)
          {
              // DrawGLEdgeStateToState(space, s, t, ridx);
              space->copyToReals(state_serialized, s);
              TiXmlElement *s1 = ReturnSubNodeVector(*node, "state", state_serialized);
              node_edge->InsertEndChild(*s1);

              space->copyToReals(state_serialized, t);
              TiXmlElement *s2 = ReturnSubNodeVector(*node, "state", state_serialized);
              node_edge->InsertEndChild(*s2);
          }else
          {
              for (int j = 0; j <= nd; j++)
              {
                  space->interpolate(s, t, (double)j / (double)nd, stateTmp);
                  space->copyToReals(state_serialized, stateTmp);
                  TiXmlElement *snode = ReturnSubNodeVector(*node, "state", state_serialized);
                  node_edge->InsertEndChild(*snode);

              }
          }

          node->InsertEndChild(*node_edge);
      }

    }
    space->freeState(stateTmp);


  }

  return true;
}

