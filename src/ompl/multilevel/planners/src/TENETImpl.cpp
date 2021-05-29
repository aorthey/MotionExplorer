#include <ompl/multilevel/planners/TENETImpl.h> 
//Load Differentiable Structures
#include <ompl/base/StateValidityCheckerDifferentiable.h>
#include <ompl/base/goals/GoalStateDifferentiable.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>
#include <Eigen/Core>


#define foreach BOOST_FOREACH
using namespace ompl::multilevel;

TENETImpl::TENETImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_)
  : BaseT(si, parent_)
{
    setName("TENETImpl" + std::to_string(id_));
    setImportance("exponential");
    setGraphSampler("randomvertex");
    getGraphSampler()->disableSegmentBias();


}

TENETImpl::~TENETImpl()
{
}

void TENETImpl::grow()
{
    //(0) If first run, add start configuration
    if (firstRun_)
    {
        init();
        firstRun_ = false;

        findSection();
    }

    //(1) Get Random Sample
    sampleBundleGoalBias(xRandom_->state);

    //(2) Get Nearest in Tree
    const Configuration *xNearest = nearest(xRandom_);

    //(3) Connect Nearest to Random (within range)
    Configuration *xNext = extendGraphTowards_Range(xNearest, xRandom_);

    //(4) If extension was successful, check if we reached goal
    if (xNext && !hasSolution_)
    {
        bool satisfied = getGoalPtr()->isSatisfied(xNext->state);
        if (satisfied)
        {
            goalConfigurations_.push_back(xNext);

            // if(qGoal_ != nullptr)
            // {
            //     addConfiguration(qGoal_);
            //     addEdge(xNext->index, getGoalIndex());
            // }
            hasSolution_ = true;
        }
    }
}
