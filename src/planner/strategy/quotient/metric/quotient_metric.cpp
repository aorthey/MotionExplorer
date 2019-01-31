#include "quotient_metric.h"
using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

QuotientMetric::QuotientMetric(og::QuotientCover* quotient_cover_):
  quotient_cover(quotient_cover_)
{
}

double QuotientMetric::DistanceQ1(const og::QuotientCover::Configuration *q_from, const og::QuotientCover::Configuration *q_to)
{
  return quotient_cover->GetQ1()->distance(q_from->state, q_to->state);
}

double QuotientMetric::DistanceX1(const QuotientCover::Configuration *q_from, const QuotientCover::Configuration *q_to)
{
  ob::State *stateFrom = quotient_cover->GetX1()->allocState();
  ob::State *stateTo = quotient_cover->GetX1()->allocState();
  quotient_cover->ProjectX1Subspace(q_from->state, stateFrom);
  quotient_cover->ProjectX1Subspace(q_to->state, stateTo);
  double d = quotient_cover->GetX1()->distance(stateFrom, stateTo);
  quotient_cover->GetX1()->freeState(stateFrom);
  quotient_cover->GetX1()->freeState(stateTo);
  return d;
}

double QuotientMetric::DistanceConfigurationNeighborhood(const Configuration *q_from, const Configuration *q_to)
{
  double d_to = q_to->GetRadius();
  double d = DistanceConfigurationConfiguration(q_from, q_to);
  return std::max(d - d_to, 0.0);
}
double QuotientMetric::DistanceNeighborhoodNeighborhood(const Configuration *q_from, const Configuration *q_to)
{
  double d_from = q_from->GetRadius();
  double d_to = q_to->GetRadius();
  double d = DistanceConfigurationConfiguration(q_from, q_to);
  if(d!=d){
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "NaN detected." << std::endl;
    std::cout << "d_from " << d_from << std::endl;
    std::cout << "d_to " << d_to << std::endl;
    std::cout << "d " << d << std::endl;
    std::cout << "configuration 1: " << std::endl;
    quotient_cover->Print(q_from, false);
    std::cout << "configuration 2: " << std::endl;
    quotient_cover->Print(q_to, false);
    std::cout << std::string(80, '-') << std::endl;
    throw "";
    exit(1);
  }

  double d_open_neighborhood_distance = (double)std::max(d - d_from - d_to, 0.0); 
  return d_open_neighborhood_distance;
}
