#include <ompl/base/Planner.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace ompl
{
  namespace geometric
  {

    //create a set of sliceRRTs, the i-th sliceRRT is equipped with 
    //
    // -- Mi (full configuration space)
    // -- Ci (standalone configuration space = (Mi / Mi-1)) 
    // -- qI (start configuration in Mi) (in pdef)
    // -- qG (goal configuration in Mi) (in pdef)
    // -- Gi (graph of vertices in Mi)
    // -- *Si-1 (a pointer to the (i-1)-th slicespace)
    //

    class MultiSliceRRT : public ob::Planner
    {
    public:

      MultiSliceRRT(const std::vector<ob::SpaceInformationPtr> Cvec,  const std::vector<ob::SpaceInformationPtr> Mvec);
      ~MultiSliceRRT(void);

      base::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;
      void clear(void) override;
      void setup(void) override;
      void getPlannerData(ob::PlannerData &data) const override;

    protected:

      std::vector<SliceRRT*> slicespaces;

    };
  }
}


