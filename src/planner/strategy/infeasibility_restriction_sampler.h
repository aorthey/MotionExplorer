#pragma once
#include <ompl/base/Planner.h>
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/multilevel/datastructures/BundleSpaceSequence.h>
#include <ompl/multilevel/planners/qmp/QMPImpl.h>
#include <ompl/base/SpaceInformation.h>

namespace ompl
{
  namespace multilevel
  {

    //return infeasible samples
    //(e.g. for visualization purposes)
    class RestrictionSamplerImpl: public QMPImpl
    {
      using BaseT = QMPImpl;

    public:

      RestrictionSamplerImpl(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);

      ~RestrictionSamplerImpl(void) = default;

      virtual void grow() override;
      virtual void clear() override;
      virtual void setup() override;
      virtual void getPlannerData(ompl::base::PlannerData &data) const override;

    };
    using RestrictionSampler = BundleSpaceSequence<RestrictionSamplerImpl>;
  }
}



