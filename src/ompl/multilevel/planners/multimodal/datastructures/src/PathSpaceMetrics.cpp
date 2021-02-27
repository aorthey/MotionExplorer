#include <ompl/multilevel/planners/multimodal/datastructures/PathSpaceMetrics.h>

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


double pathMetric_MaxMin(
    const std::vector<ompl::base::State*> f,
    const std::vector<ompl::base::State*> g,
    const ompl::base::SpaceInformationPtr si)
{
  double dfg = pathMetric_Maximum(f, g, si);
  double dgf = pathMetric_Maximum(g, f, si);
  // std::cout << "f:" << std::endl;
  // // si->printState(f.at(0));
  // // si->printState(f.at(1));
  // // std::cout << "..." << std::endl;
  // // si->printState(f.at(f.size()-2));
  // // si->printState(f.at(f.size()-1));
  // for(uint k = 0; k < f.size(); k++){
  //   si->printState(f.at(k));
  // }
  // std::cout << "g:" << std::endl;
  // // si->printState(g.at(0));
  // // si->printState(g.at(1));
  // // std::cout << "..." << std::endl;
  // // si->printState(g.at(g.size()-2));
  // // si->printState(g.at(g.size()-1));
  // for(uint k = 0; k < g.size(); k++){
  //   si->printState(g.at(k));
  // }
  // std::cout << "Path: " << dfg << "," << dgf << std::endl;
  return std::min(dfg, dgf);
}

double pathMetric_MaxMin(
    const ompl::geometric::PathGeometricPtr& f, 
    const ompl::geometric::PathGeometricPtr& g)
{
  const std::vector<ompl::base::State*> fstates = f->getStates();
  const std::vector<ompl::base::State*> gstates = g->getStates();
  ompl::base::SpaceInformationPtr si = f->getSpaceInformation();

  return pathMetric_MaxMin(fstates, gstates, si);

}


