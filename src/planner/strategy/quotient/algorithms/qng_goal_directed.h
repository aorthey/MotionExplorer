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

      bool ExpandTowardsGoal(ob::PlannerTerminationCondition &ptc);
      void ExpandTowardsFreeSpace(ob::PlannerTerminationCondition &ptc);

      bool SteerTowards(Configuration *q_from, Configuration *q_next);
      std::vector<Configuration*> GenerateCandidateDirections(Configuration *q_from, Configuration *q_next);

    private:
      //TODO: remove adaptive goal bias, replace by percentage of checking while
      //not progressMade=false

      const double goalDirectionBias{0.1}; //when not making progress, how often should we check if progress can be made?
      const double thresholdObstaclesHorizon{0.1}; //if moving towards a configuration, do not repell this movement while above obstaclesHorizon. If below, repell to steer robot away from obstacles.
      bool progressMadeTowardsGoal{true};


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


