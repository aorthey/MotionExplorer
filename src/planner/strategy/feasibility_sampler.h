#pragma once
#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>

namespace ompl
{
  namespace geometric
  {

    //return infeasible samples
    //(e.g. for visualization purposes)
    class FeasibilitySampler: public ompl::base::Planner
    {
      using BaseT = ompl::base::Planner;

    public:

      FeasibilitySampler(const ompl::base::SpaceInformationPtr &si);
      ~FeasibilitySampler(void) = default;

      ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override final;
      virtual void clear() override;
      virtual void setup() override;
      virtual void getPlannerData(ompl::base::PlannerData &data) const override;
    private:
      std::vector<ompl::base::State*> states;
      ompl::base::StateSamplerPtr sampler;
      ompl::base::State* testState;


    };
  }
}



