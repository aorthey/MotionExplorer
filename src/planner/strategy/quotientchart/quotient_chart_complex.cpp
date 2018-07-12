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
      Vertex vf = boost::add_vertex(G);
      G[vf].state = si_->cloneState(workState);
      //disjointSets_.make_set(vf);

      //#######################################################################
      //Compute size of the sphere
      //#######################################################################
      nn_->add(vf);
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

      std::cout << "vertex: " << vf << " neighbors: "<< neighbors << std::endl;
      for(uint k = 0; k < neighbors.size(); k++){
        Vertex vk = neighbors.at(k);
        //if(boost::edge(vf, vk, G).second) continue;

        double dall = Distance(vk, vf);
        double dk = G[vk].open_neighborhood_distance;
        if(d+dk > dall){
          // ob::Cost weight = opt_->motionCost(G[vf].state, G[vk].state);
          EdgeInternalState properties(ob::Cost(Distance(vf,vk)));
          boost::add_edge(vf, vk, properties, G);
          std::cout << "added edge: " << vf <<"-" << vk << " Dist " << dall << std::endl;
          //uniteComponents(vf, vk);
        }
      }

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
          std::cout << std::string(80, '-') << std::endl;
          std::cout << "removing neighbors of vertex " << vk << std::endl;
          G[vk].open_neighborhood_distance = dk;

          //sphere got updated. We need to remove all edges which have a segment
          //lying outside the sphere 

          IEIterator eo, eo_end, next;

          std::cout << "edges: " << boost::out_degree(vk,G) << std::endl;
          tie(eo, eo_end) = boost::in_edges(vk, G);
          for (next = eo; eo != eo_end; eo = next) {
            ++next;
            const Vertex v2 = boost::source(*eo, G);
            //double dall = G[*eo].getCost().value();
            std::cout << "edge " << vk << "-" << v2 << std::endl;
            double dall = Distance(vk, v2);
            double dn = G[v2].open_neighborhood_distance;
            if(dn + dk < dall){
              std::cout << "... removing edge " << vk << "-" << v2 << std::endl;
              boost::remove_edge(*eo++, G);
            }
          }
          std::cout << "done" << std::endl;
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
  BaseT::getPlannerData(data);

  for(uint vidx = Nvertices; vidx < data.numVertices(); vidx++){
    PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
    const ob::State *s = v.getState();
    v.SetComplex(const_cast<QuotientChartComplex*>(this)->simplicial_complex[s]);
    const std::vector<std::vector<int>> &cmplx = v.GetComplex();
    //std::cout << "vertex " << vidx << " has " << cmplx.size() << " simplices." << std::endl;
  }
  std::cout << "Graph induced complex is of dimension " << simplex.upper_bound_dimension() 
                    << " - " << simplex.num_vertices() << " vertices." << std::endl;

}
