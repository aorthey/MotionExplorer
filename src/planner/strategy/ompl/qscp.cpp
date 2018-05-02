#include "qscp.h"
#include "planner/cover/open_set_convex.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/validitychecker/validity_checker_simplicial_complex.h"
#include <ompl/util/Console.h>

using namespace og;
QSCP::QSCP(const ob::SpaceInformationPtr &si, Quotient *previous_):
  Base(si, previous_)
{
  openNeighborhood_ = (boost::get(vertex_open_neighborhood_t(), g_));
  setName("QSCP"+std::to_string(id));
}

QSCP::~QSCP()
{
  workspace_regions.clear();
}

void QSCP::Init()
{

  Base::Init();

  if(previous==nullptr)
  {
    const ob::State *start = stateProperty_[startM_.at(0)];
    const ob::State *goal  = stateProperty_[goalM_.at(0)];
    auto checkerPtr = static_pointer_cast<ValidityCheckerSimplicialComplex>(si_->getStateValidityChecker());
    workspace_regions.clear();
    workspace_regions = checkerPtr->GetConvexWorkspaceCover(start, goal);

    //start/goal vertices have already been added!

    for(uint k = 0; k < workspace_regions.size(); k++){
      ob::State *state = si_->allocState();
      si_->copyState(state, workspace_regions.at(k)->GetCenter());
      Vertex m = Base::CreateNewVertex(state);
      openNeighborhood_[m] = new cover::OpenSetConvex(*workspace_regions.at(k));
      ConnectVertexToNeighbors(m);
    }
  }
}

bool QSCP::SampleGraph(ob::State *q_random_graph)
{
  uint k_cover = rng.uniformInt(0, workspace_regions.size()-1);
  cover::OpenSetConvex *O = workspace_regions.at(k_cover);
  O->RandomState(q_random_graph);
  //double t = rng_.uniform01();
  //if(t<percentageSamplesOnShortestPath)
  //{
  //  //diminishing shortest path sampling
  //  PDF<Edge> pdf;
  //  percentageSamplesOnShortestPath = exp(-decay_constant*pow(((double)samplesOnShortestPath++/1000.0),2));

  //  BOOST_FOREACH (Edge e, boost::edges(g_))
  //  {
  //    const Vertex v1 = boost::source(e, g_);
  //    const Vertex v2 = boost::target(e, g_);
  //    if(onShortestPath_[v1] && onShortestPath_[v2])
  //    {
  //      ob::Cost weight = get(boost::edge_weight_t(), g_, e).getCost();
  //      pdf.add(e, weight.value());
  //    }
  //  }
  //  Edge e = pdf.sample(rng_.uniform01());
  //  double t = rng_.uniform01();

  //  const Vertex v1 = boost::source(e, g_);
  //  const Vertex v2 = boost::target(e, g_);
  //  const ob::State *from = stateProperty_[v1];
  //  const ob::State *to = stateProperty_[v2];

  //  M1->getStateSpace()->interpolate(from, to, t, q_random_graph);
  //  if(epsilon>0) M1_sampler->sampleGaussian(q_random_graph, q_random_graph, epsilon);
  //}else{
  //  uint k_cover = rng.uniformInt(0, workspace_regions.size()-1);
  //  cover::OpenSetConvex *O = workspace_regions.at(k_cover);
  //  O->RandomState(q_random_graph);
  //}
  //si_->printState(q_random_graph);
  return true;
}

void QSCP::Grow(double t)
{
  if(previous!=nullptr){
    Base::Grow(t);
  }else{
    if(totalGrowTime>0) std::cout << "[WARNING] tried growing workspace" << std::endl;
    totalGrowTime+=t;
  }
}

double QSCP::GetSamplingDensity()
{
  if(previous==nullptr){
    return +dInf;
  }else{
    return Base::GetSamplingDensity();
  }
}

void QSCP::getPlannerData(ob::PlannerData &data) const
{
  if(previous==nullptr){

    uint N = data.numVertices();
    Base::getPlannerData(data);
    //std::cout << "added " << data.numVertices()-N << " vertices." << std::endl;

    for(uint vidx = N; vidx < data.numVertices(); vidx++){
      PlannerDataVertexAnnotated *v = static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
      v->SetOpenSet(openNeighborhood_[vidx]);
    }

  }else{
    Base::getPlannerData(data);
  }
}
