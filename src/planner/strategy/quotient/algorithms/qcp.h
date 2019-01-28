#pragma once
#include "planner/strategy/quotient/quotient_cover_queue.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {

    //QCP: Quotient-space Cover Planner
    class QCP: public og::QuotientCoverQueue
    {
      typedef og::QuotientCoverQueue BaseT;
      const int verbose{0};
    public:

      QCP(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QCP(void);

      void clear() override;
      void setup() override;

      virtual void Grow(double t) override;
      void GrowWithSolution(ob::PlannerTerminationCondition &ptc);
      void GrowWithoutSolution(ob::PlannerTerminationCondition &ptc);

      void RewireCover(ob::PlannerTerminationCondition &ptc);
      virtual Vertex AddConfigurationToCover(Configuration *q) override;
      virtual void RemoveConfigurationFromCover(Configuration *q) override;

    private:
      //TODO: remove adaptive goal bias, replace by percentage of checking while
      const double rewireBias{0.2}; //when solution has been found, this bias trades off exploration vs exploitation
      bool progressMadeTowardsGoal{true};

      //PDF which assigns a value to each configuration, depending on its connectivity
      PDF pdf_connectivity_configurations;
      double ValueConnectivity(Configuration *q);
    };
  }
}


