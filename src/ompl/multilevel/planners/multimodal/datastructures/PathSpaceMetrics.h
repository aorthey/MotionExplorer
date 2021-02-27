#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <vector>
#include <limits>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(SpaceInformation);
        OMPL_CLASS_FORWARD(Path);
    }
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathGeometric);
    }
}

double pathMetric_Maximum(
    const std::vector<ompl::base::State*> f,
    const std::vector<ompl::base::State*> g,
    const ompl::base::SpaceInformationPtr si);

double pathMetric_Maximum(
    const ompl::geometric::PathGeometricPtr& f, 
    const ompl::geometric::PathGeometricPtr& g);

double pathMetric_MaxMin(
    const std::vector<ompl::base::State*> f,
    const std::vector<ompl::base::State*> g,
    const ompl::base::SpaceInformationPtr si);

double pathMetric_MaxMin(
    const ompl::geometric::PathGeometricPtr& f, 
    const ompl::geometric::PathGeometricPtr& g);
