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
      const int verbose{0};
    public:

      QNGGoalDirected(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QNGGoalDirected(void);

      void clear() override;
      void setup() override;

      virtual void Grow(double t) override;
      void GrowWithSolution(ob::PlannerTerminationCondition &ptc);
      void GrowWithoutSolution(ob::PlannerTerminationCondition &ptc);

      void RewireCover(ob::PlannerTerminationCondition &ptc);
      bool ConfigurationHasNeighborhoodLargerThan(Configuration *q, double radius);
      virtual Vertex AddConfigurationToCover(Configuration *q) override;

      bool StepTowards(Configuration *q_from, Configuration *q_next);
      bool StepTowardsGoal(ob::PlannerTerminationCondition &ptc);
      void StepTowardsFreeSpace(ob::PlannerTerminationCondition &ptc);
      void StepTowardsFreeSpaceVoronoiBias(const ob::PlannerTerminationCondition &ptc);
      std::vector<Configuration*> GenerateCandidateDirections(Configuration *q_from, Configuration *q_next);
      
      double GetImportance() const;

    private:
      //TODO: remove adaptive goal bias, replace by percentage of checking while
      //not progressMade=false

      // const double goalDirectionBias{0.05}; //when not making progress, how often should we check if progress can be made?
      const double thresholdObstaclesHorizon{0.5}; //if moving towards a configuration, do not repell this movement while above obstaclesHorizon. If below, repell to steer robot away from obstacles.
      const double rewireBias{0.2}; //when solution has been found, this bias trades off exploration vs exploitation
      bool progressMadeTowardsGoal{true};
      bool terminated{false};

      struct CmpGoalDistancePtrs
      {
        // ">" operator: smallest value is top in queue
        // "<" operator: largest value is top in queue (default)
        bool operator()(const Configuration* lhs, const Configuration* rhs) const
        {
           return lhs->GetGoalDistance() > rhs->GetGoalDistance();
        }
      };
      typedef std::priority_queue<Configuration*, std::vector<Configuration*>, CmpGoalDistancePtrs> GoalDistancePriorityQueue;
      GoalDistancePriorityQueue configurations_sorted_by_nearest_to_goal;

      //PDF which assigns a value to each configuration, depending on its connectivity
      PDF pdf_connectivity_configurations;
      double ValueConnectivity(Configuration *q);
    };
  }
}


