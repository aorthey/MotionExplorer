#include "quotient_chart_complex.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "common.h"
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <boost/foreach.hpp>

using namespace og;
#define foreach BOOST_FOREACH

QuotientChartComplex::QuotientChartComplex(const ob::SpaceInformationPtr &si, og::Quotient *parent_)
  : BaseT(si, parent_)
{
  //nn_infeasible = std::make_shared<NN<Vertex>>();
  nn_infeasible.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
  nn_infeasible->setDistanceFunction([this](const Vertex a, const Vertex b)
                           {
                             return Distance(a, b);
                           });
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
        const Vertex vfi = nn_infeasible->nearest(vf);
        G[vf].open_neighborhood_distance = Distance(vf, vfi);
      }else{
        G[vf].open_neighborhood_distance = 0;
      }

    }else{
      Vertex vi = boost::add_vertex(G_infeasible);
      G_infeasible[vi].state = si_->cloneState(workState);
      nn_infeasible->add(vi);
    }
  }
}
