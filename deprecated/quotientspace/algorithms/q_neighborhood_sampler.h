#pragma once
#include "planner/strategy/quotient/quotient_cover_queue.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {

    //QNeighborhoodSampler: just uniformly sample space, do not search for a path (for
    //visualization purposes)
    class QNeighborhoodSampler: public og::QuotientCoverQueue
    {
      typedef og::QuotientCoverQueue BaseT;
    public:

      QNeighborhoodSampler(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QNeighborhoodSampler(void) = default;

      virtual bool GetSolution(ob::PathPtr &solution) override;

      virtual void Grow() override;
    };
  }
}


