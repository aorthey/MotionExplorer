#include "qcp.h"
#include "planner/cover/open_set_convex.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/validitychecker/validity_checker_simplicial_complex.h"
#include <ompl/util/Console.h>

using namespace og;
QCP::QCP(const ob::SpaceInformationPtr &si, Quotient *previous_):
  Base(si, previous_)
{
  openNeighborhood_ = (boost::get(vertex_open_neighborhood_t(), g_));
  setName("QCP"+std::to_string(id));
}

QCP::~QCP()
{
  workspace_regions.clear();
}

void QCP::Init()
{
  if(previous==nullptr)
  {
    const ob::State *start = stateProperty_[startM_.at(0)];
    const ob::State *goal  = stateProperty_[goalM_.at(0)];
    auto checkerPtr = static_pointer_cast<ValidityCheckerSimplicialComplex>(si_->getStateValidityChecker());
    workspace_regions.clear();
    workspace_regions = checkerPtr->GetConvexWorkspaceCover(start, goal);

    for(uint k = 0; k < workspace_regions.size(); k++){
      ob::State *state = si_->allocState();
      si_->copyState(state, workspace_regions.at(k)->GetCenter());
      Vertex m = Base::CreateNewVertex(state);
      openNeighborhood_[m] = new cover::OpenSetConvex(*workspace_regions.at(k));
      ConnectVertexToNeighbors(m);
    }
  }
}

bool QCP::SampleGraph(ob::State *q_random_graph)
{
  uint k_cover = rng.uniformInt(0, workspace_regions.size()-1);
  cover::OpenSetConvex *O = workspace_regions.at(k_cover);
  O->RandomState(q_random_graph);
  return true;
}

void QCP::Grow(double t)
{
  if(previous!=nullptr){
    Base::Grow(t);
  }else{
    std::cout << "[warning] tried growing workspace" << std::endl;
  }
}

void QCP::CheckForSolution(ob::PathPtr &solution)
{
  if(previous==nullptr){
    hasSolution = true;
  }else{
    return Base::CheckForSolution(solution);
  }
}

double QCP::GetSamplingDensity()
{
  if(previous==nullptr){
    return +dInf;
  }else{
    return Base::GetSamplingDensity();
  }
}

void QCP::getPlannerData(ob::PlannerData &data) const
{
  if(previous==nullptr){

    uint N = data.numVertices();
    Base::getPlannerData(data);
    std::cout << "added " << data.numVertices()-N << " vertices." << std::endl;

    //std::vector<cover::OpenSet*> sets = cspace_cover.GetCover();
    // for(uint k = 0; k < workspace_regions.size(); k++){
    //   std::cout << "region: " << k << "/" << workspace_regions.size() << std::endl;
    //   const ob::State *s = workspace_regions.at(k)->GetCenter();
    //   M1->printState(s);
    //   PlannerDataVertexAnnotated v(s, 0);
    //   v.SetOpenSet(workspace_regions.at(k));
    //   // if((int)k==cspace_cover.GetStartSetIndex())
    //   //   data.addStartVertex(v);
    //   // else if((int)k==cspace_cover.GetGoalSetIndex())
    //   //   data.addGoalVertex(v);
    //   // else 
    //   data.addVertex(v);
    //   std::cout << data.numVertices() << std::endl;
    // }
    auto checkerPtr = static_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());
    for(uint vidx = 0; vidx < data.numVertices(); vidx++){
      PlannerDataVertexAnnotated *v = static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
      //const ob::State *s = v->getState();
      v->SetOpenSet(openNeighborhood_[vidx]);
      // double d1 = checkerPtr->Distance(s);
      // v->SetOpenNeighborhoodDistance(d1);
    }

  }else{
    Base::getPlannerData(data);
  }
}
