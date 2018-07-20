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
  rng_.setSeed(0);
  rng_boost.seed(0);
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

//void QuotientChartComplex::RemoveSimplices(Vertex v1, Vertex v2)
//{
//  //removal of edge leads to removal of all associated simplices
//  //simplicial_complex[boost::edge(v1,v2,G).first].clear();
//}
//
//void QuotientChartComplex::AddSimplices(Vertex v1, Vertex v2)
//{
//
//  // double d1 = G[v1].open_neighborhood_distance;
//  // double d2 = G[v2].open_neighborhood_distance;
//
//  // nn_->nearestR(v1, G[v1].open_neighborhood_distance, neighbors1);
//  // nn_->nearestR(v2, G[v2].open_neighborhood_distance, neighbors2);
//
//  std::vector<Vertex> neighbors1;
//  std::vector<Vertex> neighbors2;
//
//  OEIterator e1, e1_end, e2, e2_end, next;
//  tie(e1, e1_end) = boost::out_edges(v1, G);
//  tie(e2, e2_end) = boost::out_edges(v2, G);
//
//  for (next = e1; e1 != e1_end; e1 = next) {
//    ++next;
//    const Vertex vn1 = boost::target(*e1, G);
//    neighbors1.push_back(vn1);
//  }
//  for (next = e2; e2 != e2_end; e2 = next) {
//    ++next;
//    const Vertex vn2 = boost::target(*e2, G);
//    neighbors2.push_back(vn2);
//  }
//
//  for(uint i = 0; i < neighbors1.size(); i++){
//    for(uint j = 0; j < neighbors2.size(); j++){
//      const Vertex vn1 = neighbors1.at(i);
//      const Vertex vn2 = neighbors2.at(j);
//      if(vn1==vn2)
//      {
//        ////same vertex, so there exists a clique between them
//        //std::vector<int> ksimplex;
//        //ksimplex.push_back(v1);
//        //ksimplex.push_back(v2);
//        //ksimplex.push_back(vn1);
//        //// ksimplex.push_back(0);
//        //// ksimplex.push_back(1);
//        //// ksimplex.push_back(2);
//        //// ksimplex.push_back(3);
//
//        //Ksimplex *ks = new Ksimplex(ksimplex);
//        //simplicial_complex[ksimplex] = ks;
//        ////simplicial_complex[boost::edge(vn1,vn2,G).first].push_back(ksimplex);
//      }
//    }
//  }
//  EdgeInternalState properties(ob::Cost(Distance(v1,v2)));
//  boost::add_edge(v1, v2, properties, G);
//}

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

  // simplicial_complex->simplexTree.expansion(N);

  // std::vector<int> ctr(N+1);

  // for(auto k_simplex: simplicial_complex->simplexTree.skeleton_simplex_range(N))
  // {
  //   uint k_size = simplicial_complex->simplexTree.dimension(k_simplex);
  //   ctr.at(k_size)++;
  //   if(k_size <= N){
  //     std::vector<int> ks;
  //     for (auto vertex : simplicial_complex->simplexTree.simplex_vertex_range(k_simplex)) ks.push_back(vertex);
  //     PlannerDataVertexAnnotated *v = static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(ks.at(0)));
  //     v->AddComplex(ks);
  //   }
  // }

  // std::cout << std::string(80, '-') << std::endl;
  // std::cout << "* The complex contains " << simplicial_complex->simplexTree.num_simplices() << " simplices " << ctr;
  // std::cout << "   - dimension " << simplicial_complex->simplexTree.dimension() << "\n";
  // std::cout << std::string(80, '-') << std::endl;


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
