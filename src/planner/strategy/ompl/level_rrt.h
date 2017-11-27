#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace ompl
{
  namespace geometric
  {

    class LevelRRT : public RRT
    {
    public:
      LevelRRT(const ob::SpaceInformationPtr &si, const ob::SpaceInformationPtr &si2);
      ~LevelRRT(void);
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
      double distanceFunction(const Configuration *a, const Configuration *b) const
      {
        return si_->distance(a->state, b->state);
      }


      ob::SpaceInformationPtr si_level2;
    };
  }
}

