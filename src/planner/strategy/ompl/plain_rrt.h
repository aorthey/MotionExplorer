#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace ompl
{
  namespace geometric
  {

    class PlainRRT : public RRT
    {
    public:

      PlainRRT(const ob::SpaceInformationPtr &si);
      ~PlainRRT(void);
      base::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;
      void clear(void) override;
      void setup(void) override;
      void getPlannerData(ob::PlannerData &data) const override;

    private:

      class Configuration
      {
      public:
        Configuration() = default;
        Configuration(const base::SpaceInformationPtr &si) : state(si->allocState())
        {}
        ~Configuration() = default;
        base::State *state{nullptr};
        Configuration *parent{nullptr};
      };

      std::shared_ptr<NearestNeighbors<Configuration *>> G_;
      Configuration *lastGoalConfiguration_{nullptr};

      void Sample(Configuration *q_random);
      Configuration* Nearest(Configuration *q_random);
      Configuration* Connect(Configuration *q_near, Configuration *q_random);
      bool ConnectedToGoal(Configuration* q);
      void ConstructSolution(Configuration *q_goal);
      void Init();

      ob::GoalSampleableRegion *goal;

    };
  }
}

