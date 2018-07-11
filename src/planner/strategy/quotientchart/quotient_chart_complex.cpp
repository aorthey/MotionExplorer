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

bool QuotientChartComplex::Sample(ob::State *q_random)
{
  totalNumberOfSamples++;
  if(parent == nullptr){
    M1_sampler->sampleUniform(q_random);
    return M1->isValid(q_random);
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

    return M1->isValid(q_random);
  }
}
void QuotientChartComplex::Grow(double t){
  growRoadmap(ob::timedPlannerTerminationCondition(t), xstates[0]);
}

void QuotientChartComplex::growRoadmap(const ob::PlannerTerminationCondition &ptc, ob::State *workState)
{
  while (!ptc)
  {
    iterations_++;
    bool found_feasible = Sample(workState);

    if(found_feasible)
    {
      //#######################################################################
      //Add feasible vertex to graph
      //#######################################################################
      Vertex vf = addMilestone(si_->cloneState(workState));

      //#######################################################################
      //Compute size of the sphere
      //#######################################################################
      if(nn_infeasible->size()>0)
      {
        si_->printState(workState);
        const Vertex vfi = nn_infeasible->nearest(vf);
        double dn = Distance(vf, vfi);
        G[vf].open_neighborhood_distance = min(dn, epsilon_max_neighborhood);
      }else{
        G[vf].open_neighborhood_distance = epsilon_max_neighborhood;
      }

      //#######################################################################
      //add simplices
      //#######################################################################
      simplex.insert_simplex({(int)vf}, 0.);

      std::vector<Vertex> neighbors;
      nn_->nearestR(vf, 2*epsilon_max_neighborhood, neighbors);
      //add all vertices in neighborhood
      double ds = G[vf].open_neighborhood_distance;

      std::vector<Vertex> kneighbors;
      for(uint k = 0; k < neighbors.size(); k++){
        const Vertex vk = neighbors.at(k);
        double dsk = G[vk].open_neighborhood_distance;
        double d = Distance(vf, vk);
        if(ds+dsk > d){
          simplex.insert_simplex({(int)vf,(int)vk}, 0.);
          kneighbors.push_back(vk);

          std::vector<int> sc;
          sc.push_back(vk);
          simplicial_complex[G[vf].state].push_back(sc);
        }
      }
      for(uint k = 0; k < kneighbors.size(); k++){
        const Vertex vk = kneighbors.at(k);
        double dk = G[vk].open_neighborhood_distance;
        for(uint j = k+1; j < kneighbors.size(); j++){
          const Vertex vj = kneighbors.at(j);
          double dj = G[vj].open_neighborhood_distance;
          double d = Distance(vj, vk);
          if(dj + dk > d){
            simplex.insert_simplex({(int)vf,(int)vk,(int)vj}, 0.);

            std::vector<int> sc;
            sc.push_back(vk);
            sc.push_back(vj);
            simplicial_complex[G[vf].state].push_back(sc);
          }
        }
      }

    }else{
      Vertex v_infeasible = boost::add_vertex(G);
      G[v_infeasible].state = si_->cloneState(workState);
      nn_infeasible->add(v_infeasible);

      //update the feasible radii
      if(nn_->size()>0){
        std::vector<Vertex> neighbors;
        nn_->nearestR(v_infeasible, epsilon_max_neighborhood, neighbors);

        for(uint k = 0; k < neighbors.size(); k++){
          Vertex nk = neighbors.at(k);
          double dn = Distance(nk, v_infeasible)/2.0;
          double dold = G[nk].open_neighborhood_distance;
          G[nk].open_neighborhood_distance = min(dn, dold);
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
  uint Nvertices = data.numVertices();
  //uint Nedges = data.numEdges();
  BaseT::getPlannerData(data);

  //get all the simplices
  //Complex_simplex_range Gudhi::Simplex_tree< SimplexTreeOptions >::complex_simplex_range 
  //const Simplex_tree::Complex_simplex_range range = const_cast<const QuotientChartComplex*>(this)->simplex.complex_simplex_range();

  for(uint vidx = Nvertices; vidx < data.numVertices(); vidx++){
    PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
    const ob::State *s = v.getState();
    //const LocalSimplicialComplex& lsc = 
      //simplicial_complex[s];
    //dirty 
    v.SetComplex(const_cast<QuotientChartComplex*>(this)->simplicial_complex[s]);
    const std::vector<std::vector<int>> &cmplx = v.GetComplex();
    //std::cout << "vertex " << vidx << " has " << cmplx.size() << " simplices." << std::endl;

  }

  std::cout << "Graph induced complex is of dimension " << simplex.upper_bound_dimension() 
                    << " - " << simplex.num_vertices() << " vertices." << std::endl;

}
