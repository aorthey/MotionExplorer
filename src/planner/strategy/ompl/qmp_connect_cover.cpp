#include "qmp_connect_cover.h"
#include "planner/cspace/cspace.h"
#include "planner/cover/open_set.h"
#include "planner/cover/open_set_bubble.h"
#include "planner/validitychecker/validity_checker_ompl.h"
#include "elements/plannerdata_vertex_annotated.h"
#include <ompl/base/PlannerData.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace og;
using namespace ob;
using namespace cover;

QMPConnectCover::QMPConnectCover(const ob::SpaceInformationPtr &si, Quotient *previous_ ):
  og::QMPConnect(si, previous_)
{
  openNeighborhood_ = (boost::get(vertex_open_neighborhood_t(), g_));
  openNeighborhoodDistance_ = (boost::get(vertex_open_neighborhood_distance_t(), g_));
  setName("QMPCover"+std::to_string(id));
}

PRMBasic::Vertex QMPConnectCover::CreateNewVertex(ob::State *state)
{
  Vertex m = QMPConnect::CreateNewVertex(state);
  auto checkerPtr = static_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());
  double d1 = checkerPtr->Distance(stateProperty_[m]);
  openNeighborhoodDistance_[m] = d1;
  openNeighborhood_[m] = new cover::OpenSetBubble(si_, stateProperty_[m], d1);
  return m;
}

void QMPConnectCover::ClearVertices()
{
  foreach (Vertex v, boost::vertices(g_)){
    delete openNeighborhood_[v];
  }
  QMPConnect::ClearVertices();
}

void QMPConnectCover::getPlannerData(ob::PlannerData &data) const
{
  uint N = data.numVertices();
  QMPConnect::getPlannerData(data);
  std::cout << "added " << data.numVertices()-N << " vertices." << std::endl;

  auto checkerPtr = static_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());
  for(uint vidx = N; vidx < data.numVertices(); vidx++){
    PlannerDataVertexAnnotated *v = static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
    const ob::State *s = v->getState();
    double d1 = checkerPtr->Distance(s);
    v->SetOpenNeighborhoodDistance(d1);
  }
}

//bool QMPCover::SampleGraph(ob::State *q_random_graph)
//{
//  PDF<Edge> pdf = GetEdgePDF();
//  if(pdf.empty()){
//    std::cout << "cannot sample empty(?) graph" << std::endl;
//    exit(0);
//  }
//
//  auto checkerPtr = static_pointer_cast<OMPLValidityCheckerNecessarySufficient>(si_->getStateValidityChecker());
//  bool foundNecessary = false;
//
//  //double epsilon = 0.05;
//  while(!foundNecessary)
//  {
//    Edge e = pdf.sample(rng_.uniform01());
//    double t = rng_.uniform01();
//    const Vertex v1 = boost::source(e, g_);
//    const Vertex v2 = boost::target(e, g_);
//    const ob::State *from = stateProperty_[v1];
//    const ob::State *to = stateProperty_[v2];
//
//    double d1 = openNeighborhoodDistance_[v1];
//    double d2 = openNeighborhoodDistance_[v2];
//
//    M1->getStateSpace()->interpolate(from, to, t, q_random_graph);
//
//    M1_sampler->sampleGaussian(q_random_graph, q_random_graph, min(d1,d2));
//    if(!checkerPtr->IsSufficient(q_random_graph)){
//      foundNecessary = true;
//    }
//  }
//
//  return true;
//
//}
//bool QMPCover::Connect(const Vertex a, const Vertex b)
//{
//  OpenSet *o1 = openNeighborhood_[a];
//  //OpenSet *o2 = openNeighborhood_[b];
//
//  ob::State *s1 = stateProperty_[a];
//  ob::State *s2 = stateProperty_[b];
//
//  auto checker = static_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());
//  ob::StateSpacePtr space = si_->getStateSpace();
//
//  Vertex v_last = a;
//  Vertex v_next = a;
//  ob::State *s_next = s1;
//  OpenSet *o_next = o1;
//
//  double epsilon = 0.01;
//
//  //TODO: bisect that
//
//  while( checker->isValid(s_next) && !o_next->Contains(s2) && o_next->GetRadius() > epsilon){
//    ob::State *s_m = si_->allocState();
//    o_next->IntersectionTowards(s2, s_m);
//    //si_->printState(s_m);
//
//    v_next = CreateNewVertex(s_m);
//    o_next = openNeighborhood_[v_next];
//
//    ob::Cost weight = opt_->motionCost(stateProperty_[v_last], stateProperty_[v_next]);
//    EdgeProperty properties(weight);
//    boost::add_edge(v_last, v_next, properties, g_);
//    uniteComponents(v_last, v_next);
//
//    v_last = v_next;
//    si_->freeState(s_m);
//  }
//  //std::cout << std::string(80, '-') << std::endl;
//  if(o_next->Contains(s2)){
//    v_next = b;
//    ob::Cost weight = opt_->motionCost(stateProperty_[v_last], stateProperty_[v_next]);
//    EdgeProperty properties(weight);
//    boost::add_edge(v_last, v_next, properties, g_);
//    uniteComponents(v_last, v_next);
//  }
//  return false;
//}
