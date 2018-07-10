#include "quotient_chart_complex.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "common.h"
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <boost/foreach.hpp>

#include <gudhi/graph_simplicial_complex.h>
#include <gudhi/Simplex_tree.h>
#include <gudhi/GIC.h>

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

  using Simplex_tree = Gudhi::Simplex_tree<>;
  Simplex_tree stree;
  stree.insert_simplex({0, 1}, 0.);

  using Point = Vertex;
  Gudhi::cover_complex::Cover_complex<Point> SC;
  SC.set_type("Nerve");
  SC.create_complex(stree);

  std::cout << "Graph induced complex is of dimension " << stree.dimension() << " - " << stree.num_simplices()
                    << " simplices - " << stree.num_vertices() << " vertices." << std::endl;

  exit(0);

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
      Vertex vf = addMilestone(si_->cloneState(workState));
      if(nn_infeasible->size()>0)
      {
        si_->printState(workState);
        const Vertex vfi = nn_infeasible->nearest(vf);
        double dn = Distance(vf, vfi);
        G[vf].open_neighborhood_distance = min(dn, epsilon_max_neighborhood);
      }else{
        G[vf].open_neighborhood_distance = epsilon_max_neighborhood;
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
