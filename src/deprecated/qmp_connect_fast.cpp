#include "common.h"
#include "qmp_connect_fast.h"
#include "planner/cspace/cspace.h"

#include <gudhi/graph_simplicial_complex.h>
#include <gudhi/Simplex_tree.h>


#include <ompl/datastructures/PDF.h>
#include <boost/foreach.hpp>
#include <boost/graph/graphviz.hpp>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/config/MagicConstants.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>

using namespace og;
using namespace ob;

QMPConnectFast::QMPConnectFast(const ob::SpaceInformationPtr &si, Quotient *parent_ ):
  BaseT(si, parent_)
{
  setName("QMPConnectFast"+to_string(id));
  percentageSamplesOnShortestPath = 0.8;
  uint N = si->getStateDimension();
  simplex = new TDS(N);
  std::cout << "created simplex" << std::endl;
  std::cout << simplex->current_dimension() << std::endl;
  std::cout << simplex->maximal_dimension() << std::endl;
  using Simplex_tree = Gudhi::Simplex_tree<>;
  Simplex_tree stree;
  stree.insert_simplex({0, 1}, 0.);
  stree.insert_simplex({0, 2}, 1.);
  stree.insert_simplex({0, 3}, 2.);
  stree.insert_simplex({1, 2}, 3.);
  stree.insert_simplex({1, 3}, 4.);
  stree.insert_simplex({2, 3}, 5.);
  stree.insert_simplex({2, 4}, 6.);
  stree.insert_simplex({3, 6}, 7.);
  stree.insert_simplex({4, 5}, 8.);
  stree.insert_simplex({4, 6}, 9.);
  stree.insert_simplex({5, 6}, 10.);
  stree.insert_simplex({6}, 10.);
  std::cout << "********************************************************************\n";
  std::cout << "* The complex contains " << stree.num_simplices() << " simplices";
  std::cout << "   - dimension " << stree.dimension() << "\n";
  std::cout << "* Iterator on Simplices in the filtration, with [filtration value]:\n";
  for (auto f_simplex : stree.filtration_simplex_range()) {
    std::cout << "   "
              << "[" << stree.filtration(f_simplex) << "] ";
    for (auto vertex : stree.simplex_vertex_range(f_simplex)) std::cout << "(" << vertex << ")";
    std::cout << std::endl;
  }
  exit(0);
}

QMPConnectFast::~QMPConnectFast(){
}

bool QMPConnectFast::Sample(ob::State *q_random)
{
  totalNumberOfSamples++;
  if(parent == nullptr){
    M1_valid_sampler->sample(q_random);
  }else{
    ob::SpaceInformationPtr M0 = parent->getSpaceInformation();
    base::State *s_C1 = C1->allocState();
    base::State *s_M0 = M0->allocState();

    C1_sampler->sampleUniform(s_C1);
    parent->SampleGraph(s_M0);
    mergeStates(s_M0, s_C1, q_random);

    C1->freeState(s_C1);
    M0->freeState(s_M0);
  }
  return true;
}

void QMPConnectFast::Grow(double t){
  //ob::State *workState = xstates[0];
  //bool feasible = Sample(workState);
  BaseT::Grow(t);
  //if(feasible) addMilestone(si_->cloneState(workState));
  //else
}
