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
    const ompl::base::SpaceInformationPtr si)
{

  double maxDistance = 0.0;
  foreach(const ompl::base::State* fs, f)
  {
    double dmin = std::numeric_limits<double>::infinity();
    foreach(const ompl::base::State* gs, g)
    {
      double d = si->distance(fs, gs);
      if(d < dmin)
      {
        dmin = d;
      }
    }
    if(dmin > maxDistance)
    {
      maxDistance = dmin;
    }
  }
  return maxDistance;
}

double pathMetric_Maximum(
    const ompl::geometric::PathGeometricPtr& f, 
    const ompl::geometric::PathGeometricPtr& g)
{
  const std::vector<ompl::base::State*> fstates = f->getStates();
  const std::vector<ompl::base::State*> gstates = g->getStates();
  ompl::base::SpaceInformationPtr si = f->getSpaceInformation();

  return pathMetric_Maximum(fstates, gstates, si);
}

