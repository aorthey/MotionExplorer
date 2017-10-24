#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class NecessaryRRT : public RRT
    {
    public:
      NecessaryRRT(const base::SpaceInformationPtr &si, const base::SpaceInformationPtr &si2);
      ~NecessaryRRT(void);
      base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
      void clear(void) override;
      void setup(void) override;
      void getPlannerData(base::PlannerData &data) const override;
    };
  }
}

