#include "common.h"
#include "qmp_connect_fast.h"
#include "planner/cspace/cspace.h"

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

QMPConnectFast::QMPConnectFast(const ob::SpaceInformationPtr &si, Quotient *previous_ ):
  BaseT(si, previous_)
{
  setName("QMPConnectFast"+to_string(id));
  percentageSamplesOnShortestPath = 0.8;
}

QMPConnectFast::~QMPConnectFast(){
}

bool QMPConnectFast::Sample(ob::State *q_random)
{
  totalNumberOfSamples++;
  if(previous == nullptr){
    M1_valid_sampler->sample(q_random);
  }else{
    ob::SpaceInformationPtr M0 = previous->getSpaceInformation();
    base::State *s_C1 = C1->allocState();
    base::State *s_M0 = M0->allocState();

    C1_sampler->sampleUniform(s_C1);
    previous->SampleGraph(s_M0);
    mergeStates(s_M0, s_C1, q_random);

    C1->freeState(s_C1);
    M0->freeState(s_M0);
  }
  return M1->isValid(q_random);
}
void QMPConnectFast::Grow(double t){
  BaseT::Grow(t);
  //growRoadmap(ob::timedPlannerTerminationCondition(t), xstates[0]);
}
