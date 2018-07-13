#include "quotient_chart_complex.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "common.h"
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <boost/foreach.hpp>

// #include <gudhi/graph_simplicial_complex.h>
// #include <gudhi/Simplex_tree.h>
// #include <gudhi/GIC.h>

using namespace og;
#define foreach BOOST_FOREACH

QuotientChartComplex::QuotientChartComplex(const ob::SpaceInformationPtr &si, og::Quotient *parent_)
  : BaseT(si, parent_)
{
  //nn_infeasible = std::make_shared<NN<Vertex>>();
  nn_infeasible.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
  nn_infeasible->setDistanceFunction([this](const Vertex a, const Vertex b)
                           {
                             return Distance(a,b);
                           });

  //stree.insert_simplex({0, 1}, 0.);
  //stree.insert_simplex_and_subfaces(simplex, filt);

  // exit(0);

  //Step1: Fill cover complex with graph plus the radius
  //Cover_complex -> create_complex -> SimplexTree (efficient representation of a simplicial complex) -> betti_numbers
}
void QuotientChartComplex::setup() 
{
  BaseT::setup();
  for(uint k = 0; k < startM_.size(); k++){
    G[startM_.at(k)].open_neighborhood_distance = epsilon_max_neighborhood;
  }
  for(uint k = 0; k < goalM_.size(); k++){
    G[goalM_.at(k)].open_neighborhood_distance = epsilon_max_neighborhood;
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

void QuotientChartComplex::RemoveSimplices(Vertex v1, Vertex v2)
{
  //removal of edge leads to removal of all associated simplices
  //simplicial_complex[boost::edge(v1,v2,G).first].clear();
}

void QuotientChartComplex::AddSimplices(Vertex v1, Vertex v2)
{

  // double d1 = G[v1].open_neighborhood_distance;
  // double d2 = G[v2].open_neighborhood_distance;

  // nn_->nearestR(v1, G[v1].open_neighborhood_distance, neighbors1);
  // nn_->nearestR(v2, G[v2].open_neighborhood_distance, neighbors2);

  std::vector<Vertex> neighbors1;
  std::vector<Vertex> neighbors2;

  OEIterator e1, e1_end, e2, e2_end, next;
  tie(e1, e1_end) = boost::out_edges(v1, G);
  tie(e2, e2_end) = boost::out_edges(v2, G);

  for (next = e1; e1 != e1_end; e1 = next) {
    ++next;
    const Vertex vn1 = boost::target(*e1, G);
    neighbors1.push_back(vn1);
  }
  for (next = e2; e2 != e2_end; e2 = next) {
    ++next;
    const Vertex vn2 = boost::target(*e2, G);
    neighbors2.push_back(vn2);
  }

  for(uint i = 0; i < neighbors1.size(); i++){
    for(uint j = 0; j < neighbors2.size(); j++){
      const Vertex vn1 = neighbors1.at(i);
      const Vertex vn2 = neighbors2.at(j);
      if(vn1==vn2)
      {
        //same vertex, so there exists a clique between them
        std::vector<int> ksimplex;
        ksimplex.push_back(v1);
        ksimplex.push_back(v2);
        ksimplex.push_back(vn1);
        // ksimplex.push_back(0);
        // ksimplex.push_back(1);
        // ksimplex.push_back(2);
        // ksimplex.push_back(3);

        Ksimplex *ks = new Ksimplex(ksimplex);
        simplicial_complex[ksimplex] = ks;
        //simplicial_complex[boost::edge(vn1,vn2,G).first].push_back(ksimplex);
      }
    }
  }
  EdgeInternalState properties(ob::Cost(Distance(v1,v2)));
  boost::add_edge(v1, v2, properties, G);
}

void QuotientChartComplex::Grow(double t){
  ob::State *workState = xstates[0];
  iterations_++;
  bool found_feasible = Sample(workState);

  if(found_feasible)
  {
    //#######################################################################
    //Add feasible vertex to graph
    //#######################################################################
    Vertex vf = boost::add_vertex(G);
    G[vf].state = si_->cloneState(workState);
    //disjointSets_.make_set(vf);

    //#######################################################################
    //Compute size of the sphere
    //#######################################################################
    if(nn_infeasible->size()>0)
    {
      const Vertex vfi = nn_infeasible->nearest(vf);
      double dn = Distance(vf, vfi);
      G[vf].open_neighborhood_distance = min(dn, epsilon_max_neighborhood);
    }else{
      G[vf].open_neighborhood_distance = epsilon_max_neighborhood;
    }

    //#######################################################################
    //Connect to all intersecting samples, create local 1-skeleton
    //#######################################################################
    std::vector<Vertex> neighbors;
    nn_->nearestR(vf, 2*epsilon_max_neighborhood, neighbors);
    double d = G[vf].open_neighborhood_distance;

    for(uint k = 0; k < neighbors.size(); k++){
      Vertex vk = neighbors.at(k);
      double dall = Distance(vk, vf);
      double dk = G[vk].open_neighborhood_distance;
      if(d+dk > dall){
        AddSimplices(vf, vk);
        //double d = Distance(v1,v2);
      }
    }
    nn_->add(vf);


  }else{
    //#######################################################################
    //Add infeasible vertex 
    //#######################################################################
    Vertex v_infeasible = boost::add_vertex(G);
    G[v_infeasible].state = si_->cloneState(workState);
    nn_infeasible->add(v_infeasible);

    //#######################################################################
    //Update all sphere radii. If a sphere radius is changed, remove edges
    //which are not contained in the sphere anymore
    //#######################################################################
    std::vector<Vertex> neighbors;
    nn_->nearestR(v_infeasible, epsilon_max_neighborhood, neighbors);

    for(uint k = 0; k < neighbors.size(); k++){
      Vertex vk = neighbors.at(k);
      double dk = Distance(v_infeasible, vk);
      double dold = G[vk].open_neighborhood_distance;
      if(dk < dold){
        G[vk].open_neighborhood_distance = dk;

        //sphere got updated. We need to remove all edges which have a segment
        //lying outside the sphere 
        OEIterator eo, eo_end, next;

        tie(eo, eo_end) = boost::out_edges(vk, G);
        std::vector<Vertex> edge_removal;
        for (next = eo; eo != eo_end; eo = next) {
          ++next;
          const Vertex v2 = boost::target(*eo, G);
          double dall = Distance(vk, v2);
          double dn = G[v2].open_neighborhood_distance;
          if(dn + dk < dall){
            edge_removal.push_back(v2);
          }
        }
        for(uint k = 0; k < edge_removal.size(); k++){
          const Vertex vn = edge_removal.at(k);
          Edge ekn = boost::edge(vk, vn, G).first;
          boost::remove_edge(ekn, G);
        }
      }
    }

  }
}

double QuotientChartComplex::Distance(const Vertex a, const Vertex b) const
{
  return BaseT::Distance(a,b);
}
void QuotientChartComplex::getPlannerData(ob::PlannerData &data) const
{
  //uint Nvertices = data.numVertices();
  BaseT::getPlannerData(data);

  //for(uint vidx = Nvertices; vidx < data.numVertices(); vidx++){
  //  PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
  //  const ob::State *s = v.getState();
  //  // v.SetComplex(const_cast<QuotientChartComplex*>(this)->simplicial_complex[s]);
  //  // const std::vector<std::vector<int>> &cmplx = v.GetComplex();
  //  //std::cout << "vertex " << vidx << " has " << cmplx.size() << " simplices." << std::endl;
  //}
  std::cout << "Graph induced complex is of dimension " << simplex.upper_bound_dimension() 
                    << " - " << simplex.num_vertices() << " vertices." << std::endl;

}
