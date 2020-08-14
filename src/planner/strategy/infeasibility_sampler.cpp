#include "infeasibility_sampler.h"
#include "util.h"


using namespace ompl::geometric;
namespace ob = ompl::base;

InfeasibilitySampler::InfeasibilitySampler(const ob::SpaceInformationPtr &si):
  Planner(si, "InfeasibilitySampler")
{
  testState = si->allocState();
  sampler = si_->allocStateSampler();
}

ompl::base::PlannerStatus InfeasibilitySampler::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
  while(!ptc())
  {
    sampler->sampleUniform(testState);
    if(!si_->isValid(testState)){
      ob::State *q = si_->cloneState(testState);
      states.push_back(q);
    }
  }
  return ompl::base::PlannerStatus::EXACT_SOLUTION;

}


void InfeasibilitySampler::clear()
{
  BaseT::clear();
  // si_->freeState(testState);
}
void InfeasibilitySampler::setup()
{
  BaseT::setup();
}
void InfeasibilitySampler::getPlannerData(ob::PlannerData &data) const
{
  for(uint k = 0; k < states.size(); k++){
    ob::PlannerDataVertex p(states.at(k));
    data.addVertex(p);
  }
}
