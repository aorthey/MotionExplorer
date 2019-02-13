#pragma once
#include "planner/strategy/quotient/quotient_cover_queue.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {

    //QCP: Quotient-space Cover Planner
    OMPL_CLASS_FORWARD(CoverExpansionStrategy);
    class QCP: public og::QuotientCoverQueue
    {
      typedef og::QuotientCoverQueue BaseT;
      const int verbose{0};
    public:

      QCP(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QCP(void) = default;

      void clear() override;
      void setup() override;

      virtual void Grow(double t) override;
      void GrowWithSolution(ob::PlannerTerminationCondition &ptc);
      void GrowWithoutSolution(ob::PlannerTerminationCondition &ptc);

      virtual double GetImportance() const override;

      void RewireCover(ob::PlannerTerminationCondition &ptc);

    private:
      //TODO: remove adaptive goal bias, replace by percentage of checking while
      const double rewireBias{0.1}; //when solution has been found, this bias trades off exploration vs exploitation
      bool progressMadeTowardsGoal{true};

      CoverExpansionStrategyPtr expansion_strategy_goal;
      CoverExpansionStrategyPtr expansion_strategy_outwards;
      CoverExpansionStrategyPtr expansion_strategy_random_voronoi;
      CoverExpansionStrategyPtr expansion_strategy_random_boundary;
    };
  }
}


