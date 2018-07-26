#include "quotient_chart_complex.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "common.h"
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>

using namespace og;
using namespace ogt;
#define foreach BOOST_FOREACH

QuotientChartComplex::QuotientChartComplex(const ob::SpaceInformationPtr &si, og::Quotient *parent_)
  : BaseT(si, parent_)
{
  // rng_.setSeed(1);
  // rng_boost.seed(1);
}
void QuotientChartComplex::setup() 
{
  if(pdef_ && !setup_){
    simplicial_complex = new ogt::SimplicialComplex(si_, this);

    if(const ob::State *s = pis_.nextStart()){
      simplicial_complex->AddStart(s);
    }else{
      OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
      exit(0);
    }
    if(const ob::State *s = pis_.nextGoal())
    {
      simplicial_complex->AddGoal(s);
    }else{
      OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
      exit(0);
    }

    setup_ = true;
  }else{
    setup_ = false;
  }
}

bool QuotientChartComplex::Sample(ob::State *q_random)
{
  totalNumberOfSamples++;
  if(parent == nullptr){
    M1_sampler->sampleUniform(q_random);
  }else{
    //Adjusted sampling function: Sampling in G0 x C1
    ob::SpaceInformationPtr M0 = parent->getSpaceInformation();
    base::State *s_C1 = C1->allocState();
    base::State *s_M0 = M0->allocState();

    C1_sampler->sampleUniform(s_C1);
    parent->SampleGraph(s_M0);
    mergeStates(s_M0, s_C1, q_random);

    C1->freeState(s_C1);
    M0->freeState(s_M0);
  }
  return M1->isValid(q_random);
}

void QuotientChartComplex::Grow(double t){
  ob::State *workState = xstates[0];
  iterations_++;
  bool found_feasible = Sample(workState);

  if(found_feasible)
  {
    simplicial_complex->AddFeasible(workState);
  }else
  {
    simplicial_complex->AddInfeasible(workState);
  }
}

double QuotientChartComplex::Distance(const Vertex a, const Vertex b) const
{
  return BaseT::Distance(a,b);
}
//#include "3rdparty/matplotlibcpp.h"
  // namespace plt = matplotlibcpp;
  // plt::plot({1,3,2,4});
  // plt::show();
void QuotientChartComplex::getPlannerData(ob::PlannerData &data) const
{


  uint Nvertices = data.numVertices();
  uint Nedges = data.numEdges();

  //###########################################################################
  //Get Path for this chart
  //###########################################################################
  std::vector<int> path;
  path.push_back(GetHorizontalIndex());
  og::QuotientChart *parent = static_cast<og::QuotientChart*>(GetParent());
  while(parent!=nullptr)
  {
    path.push_back(parent->GetHorizontalIndex());
    parent = static_cast<og::QuotientChart*>(parent->GetParent());
  }

  std::reverse(std::begin(path), std::end(path));

  //###########################################################################
  //Get Data from this chart
  //###########################################################################
  std::cout << "vertices " << GetNumberOfVertices() << " edges " << GetNumberOfEdges() << std::endl;

  //if the chart is local, we need to clone new states such that we have
  //duplicate vertices (sometimes charts are overlapping). 

  const SimplicialComplex::Graph& G = simplicial_complex->GetGraph();


  foreach( const Vertex v, boost::vertices(G))
  {
    ob::State *s = (local_chart?si_->cloneState(G[v].state):G[v].state);
    PlannerDataVertexAnnotated p(s);
    p.SetOpenNeighborhoodDistance(G[v].open_neighborhood_distance);
    p.SetLevel(level);
    p.SetPath(path);

    if(G[v].isInfeasible) p.SetInfeasible();

    if(G[v].isStart) data.addStartVertex(p);
    else if(G[v].isGoal) data.addGoalVertex(p);
    else data.addVertex(p);
  }
  foreach (const Edge e, boost::edges(G))
  {
    const Vertex v1 = boost::source(e, G);
    const Vertex v2 = boost::target(e, G);
    data.addEdge(v1,v2);
  }

  //###########################################################################
  //extract simplicial complex
  //###########################################################################

  uint N = si_->getStateDimension();
  std::vector<std::vector<SimplicialComplex::Vertex>> k_skeleton = simplicial_complex->GetSimplicesOfDimension(N+1);
  std::cout << "simplices of dimension " << N+1 << ":" << k_skeleton.size() << std::endl;

  for(uint i = 0; i < k_skeleton.size(); i++){
    std::vector<SimplicialComplex::Vertex> ks = k_skeleton.at(i);
    PlannerDataVertexAnnotated *v = static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(ks.at(0)));
    v->AddComplex(ks);
  }

  //###########################################################################
  //Get Data From all siblings
  //###########################################################################

  std::cout << "QuotientChart vIdx " << level << " | hIdx " << horizontal_index 
    << " | siblings " << siblings.size() << " | path " << path 
    << " | vertices " << data.numVertices() - Nvertices 
    << " | edges " << data.numEdges() - Nedges
    << std::endl;

  for(uint i = 0; i < siblings.size(); i++){
    siblings.at(i)->getPlannerData(data);
  }
  if(child!=nullptr) child->getPlannerData(data);
}
