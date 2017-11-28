#include "plain_rrt.h"
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

    class SliceRRT : public PlainRRT
    {
    public:

      SliceRRT(const ob::SpaceInformationPtr &Ccur, const ob::SpaceInformationPtr &Mcur, SliceRRT *previous = nullptr);
      ~SliceRRT(void);

    protected:

      //std::shared_ptr<NearestNeighbors<Configuration *>> G_;

      virtual void SampleGraph(Configuration *q_random); //samples a configuration along the graph G_
      virtual void Sample(Configuration *q_random) override; //samples Ccur
      virtual Configuration* Nearest(Configuration *q_random) override;
      virtual Configuration* Connect(Configuration *q_near, Configuration *q_random) override;

      ob::GoalSampleableRegion *goal;

      double Lgraph{0.0}; //sum of edge weights in graph G_
      ob::SpaceInformationPtr C;
      ob::SpaceInformationPtr M;
      SliceRRT *previous;

    };
  }
}

