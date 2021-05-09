#include "feasibility_sampler.h"
#include "util.h"


using namespace ompl::geometric;
namespace ob = ompl::base;

FeasibilitySampler::FeasibilitySampler(const ob::SpaceInformationPtr &si):
  Planner(si, "FeasibilitySampler")
{
  testState = si->allocState();
  sampler = si_->allocStateSampler();
}

ompl::base::PlannerStatus FeasibilitySampler::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
  while(!ptc())
  {
    sampler->sampleUniform(testState);
    if(si_->isValid(testState))
    {
      ob::State *q = si_->cloneState(testState);
      states.push_back(q);
      std::cout << "FEASIBLE STATE:" << std::endl;
      si_->printState(testState);
    }
  }
  return ompl::base::PlannerStatus::EXACT_SOLUTION;
}

void FeasibilitySampler::clear()
{
  BaseT::clear();
  // si_->freeState(testState);
}
void FeasibilitySampler::setup()
{
  BaseT::setup();
}
void FeasibilitySampler::getPlannerData(ob::PlannerData &data) const
{
  for(uint k = 0; k < states.size(); k++){
    ob::PlannerDataVertex p(states.at(k));
    data.addVertex(p);
  }
}
