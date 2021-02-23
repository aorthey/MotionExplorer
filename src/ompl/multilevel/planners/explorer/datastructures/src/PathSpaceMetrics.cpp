#include <ompl/multilevel/planners/explorer/datastructures/PathSpaceMetrics.h>

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


