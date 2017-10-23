#include <ompl/base/Planner.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>

namespace ompl
{
  class NecessaryRRT : public base::Planner
  {
  public:
    NecessaryRRT(const base::SpaceInformationPtr &si);
    virtual ~NecessaryRRT(void);
    virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);
    virtual void clear(void);
    virtual void setup(void);
    virtual void getPlannerData(base::PlannerData &data) const;
  };
}

