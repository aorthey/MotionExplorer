#pragma once
#include "qng.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {

    class QNGGoalDirected: public og::QNG
    {
      typedef og::QNG BaseT;
      int verbose{0};
    public:

      QNGGoalDirected(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QNGGoalDirected(void);

      void clear() override;
      void setup() override;
      virtual void Grow(double t) override;

      void IncreaseGoalBias();
      void DecreaseGoalBias();
      bool ExpandTowardsGoal(ob::PlannerTerminationCondition &ptc);
      void ExpandTowardsFreeSpace(ob::PlannerTerminationCondition &ptc);

      bool SteerTowards(Configuration *q_from, Configuration *q_next);
      std::vector<Configuration*> GenerateCandidateDirections(Configuration *q_from, Configuration *q_next);

    private:
      double goalDirectionAdaptiveBias{1.0};
      bool progressMade{false};

      struct CmpGoalDistancePtrs
      {
        bool operator()(const Configuration* lhs, const Configuration* rhs) const
        {
           return lhs->GetGoalDistance() < rhs->GetGoalDistance();
        }
      };
      typedef std::priority_queue<Configuration*, std::vector<Configuration*>, CmpGoalDistancePtrs> GoalDistancePriorityQueue;
      GoalDistancePriorityQueue configurations_sorted_by_nearest_to_goal;

    };
  }
}


