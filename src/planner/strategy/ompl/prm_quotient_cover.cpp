#include "prm_quotient_cover.h"
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

PRMQuotientCover::PRMQuotientCover(const ob::SpaceInformationPtr &si, Quotient *previous_ ):
  og::PRMQuotient(si, previous_)
  //og::PRMQuotientNarrowEdgeDegree(si, previous_)
  //openNeighborhoodDistance_(boost::get(vertex_open_neighborhood_distance_t(), g_))
{
  openNeighborhood_ = (boost::get(vertex_open_neighborhood_t(), g_));
  openNeighborhoodDistance_ = (boost::get(vertex_open_neighborhood_distance_t(), g_));
  setName("PRMQuotientCover"+std::to_string(id));
}


PRMBasic::Vertex PRMQuotientCover::CreateNewVertex(ob::State *state)
{
  Vertex m = boost::add_vertex(g_);
  stateProperty_[m] = si_->cloneState(state);
  totalConnectionAttemptsProperty_[m] = 1;
  successfulConnectionAttemptsProperty_[m] = 0;
  auto checkerPtr = static_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());
  double d1 = checkerPtr->Distance(stateProperty_[m]);
  openNeighborhoodDistance_[m] = d1;
  openNeighborhood_[m] = new cover::OpenSetBubble(si_, stateProperty_[m], d1);

  disjointSets_.make_set(m);

  return m;
}
void PRMQuotientCover::ClearVertices()
{
  PRMBasic::ClearVertices();
  foreach (Vertex v, boost::vertices(g_)){
    delete openNeighborhood_[v];
  }
}

void PRMQuotientCover::getPlannerData(ob::PlannerData &data) const
{
  for (unsigned long i : startM_)
  {
    double d = openNeighborhoodDistance_[i];
    data.addStartVertex(
      PlannerDataVertexAnnotated(stateProperty_[i], const_cast<PRMQuotientCover *>(this)->disjointSets_.find_set(i),d)
      );
  }


  for (unsigned long i : goalM_)
  {
    double d = openNeighborhoodDistance_[i];
    data.addGoalVertex(
      PlannerDataVertexAnnotated(stateProperty_[i], const_cast<PRMQuotientCover *>(this)->disjointSets_.find_set(i),d)
      );
  }

  foreach (const Edge e, boost::edges(g_))
  {
    const Vertex v1 = boost::source(e, g_);
    const Vertex v2 = boost::target(e, g_);
    double d1 = openNeighborhoodDistance_[v1];
    double d2 = openNeighborhoodDistance_[v2];
    data.addEdge(PlannerDataVertexAnnotated(stateProperty_[v1], 0, d1), PlannerDataVertexAnnotated(stateProperty_[v2], 0, d2));
    //data.addEdge(ob::PlannerDataVertex(stateProperty_[v2]), ob::PlannerDataVertex(stateProperty_[v1]));
    data.tagState(stateProperty_[v1], const_cast<PRMQuotientCover *>(this)->disjointSets_.find_set(v1));
    data.tagState(stateProperty_[v2], const_cast<PRMQuotientCover *>(this)->disjointSets_.find_set(v2));
  }
}
bool PRMQuotientCover::SampleGraph(ob::State *q_random_graph)
{
  PDF<Edge> pdf = GetEdgePDF();
  if(pdf.empty()){
    std::cout << "cannot sample empty(?) graph" << std::endl;
    exit(0);
  }

  auto checkerPtr = static_pointer_cast<OMPLValidityCheckerNecessarySufficient>(si_->getStateValidityChecker());
  bool foundNecessary = false;

  //double epsilon = 0.05;
  while(!foundNecessary)
  {
    Edge e = pdf.sample(rng_.uniform01());
    double t = rng_.uniform01();
    const Vertex v1 = boost::source(e, g_);
    const Vertex v2 = boost::target(e, g_);
    const ob::State *from = stateProperty_[v1];
    const ob::State *to = stateProperty_[v2];

    double d1 = openNeighborhoodDistance_[v1];
    double d2 = openNeighborhoodDistance_[v2];

    M1->getStateSpace()->interpolate(from, to, t, q_random_graph);

    //simpleSampler_->sampleGaussian(q_random_graph, q_random_graph, epsilon);
    //simpleSampler_->sampleUniformNear(q_random_graph, q_random_graph, min(d1,d2));
    simpleSampler_->sampleGaussian(q_random_graph, q_random_graph, min(d1,d2));
    //simpleSampler_->sampleUniformNear(q_random_graph, q_random_graph, epsilon);
    if(!checkerPtr->IsSufficient(q_random_graph)){
      foundNecessary = true;
    }
  }

  isSampled = true;

  return true;

}
bool PRMQuotientCover::Connect(const Vertex a, const Vertex b)
{
  OpenSet *o1 = openNeighborhood_[a];
  OpenSet *o2 = openNeighborhood_[b];

  ob::State *s1 = stateProperty_[a];
  ob::State *s2 = stateProperty_[b];

  auto checker = static_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());
  ob::StateSpacePtr space = si_->getStateSpace();

  Vertex v_last = a;
  Vertex v_next = a;
  ob::State *s_next = s1;
  OpenSet *o_next = o1;

  double epsilon = 0.01;
  //std::cout << "radius: " << o_next->GetRadius() << " contains: " << (o_next->Contains(s2)?"Yes":"No") << std::endl;

  //TODO: bisect that

  while( checker->isValid(s_next) && !o_next->Contains(s2) && o_next->GetRadius() > epsilon){
    ob::State *s_m = si_->allocState();
    o_next->IntersectionTowards(s2, s_m);
    //si_->printState(s_m);

    v_next = CreateNewVertex(s_m);
    o_next = openNeighborhood_[v_next];

    ob::Cost weight = opt_->motionCost(stateProperty_[v_last], stateProperty_[v_next]);
    EdgeProperty properties(weight);
    boost::add_edge(v_last, v_next, properties, g_);
    uniteComponents(v_last, v_next);

    v_last = v_next;
    si_->freeState(s_m);
  }
  //std::cout << std::string(80, '-') << std::endl;
  if(o_next->Contains(s2)){
    v_next = b;
    ob::Cost weight = opt_->motionCost(stateProperty_[v_last], stateProperty_[v_next]);
    EdgeProperty properties(weight);
    boost::add_edge(v_last, v_next, properties, g_);
    uniteComponents(v_last, v_next);
  }
  return false;
}
