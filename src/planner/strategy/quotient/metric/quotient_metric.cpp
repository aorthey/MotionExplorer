#include "quotient_metric.h"
using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

QuotientMetric::QuotientMetric(og::QuotientCover* quotient_cover_):
  quotient_cover(quotient_cover_)
{
}

//############################################################################
//Distance Functions
//############################################################################
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
//############################################################################
//Interpolate Functions
//############################################################################
void QuotientMetric::InterpolateQ1(const Configuration *q_from, const Configuration *q_to, double step, Configuration* q_out)
{
  return quotient_cover->GetQ1()->getStateSpace()->interpolate(q_from->state, q_to->state, step, q_out->state);
}
void QuotientMetric::Interpolate(const Configuration *q_from, Configuration *q_to)
{
  return Interpolate(q_from, q_to, q_to);
}
void QuotientMetric::Interpolate(const Configuration *q_from, const Configuration *q_to, Configuration *q_interp)
{
  double d = DistanceConfigurationConfiguration(q_from, q_to);
  double radius = q_from->GetRadius();
  double step_size = radius/d;
  Interpolate(q_from, q_to, step_size, q_interp);
}

//stepsize \in [0,1]
//bool QuotientMetric::Interpolate(const Configuration *q_from, const Configuration *q_to, double step_size, Configuration *q_interp)
//{
//  if(parent==nullptr){
//    Q1->getStateSpace()->interpolate(q_from->state, q_to->state, step_size, q_interp->state);
//  }else{

//    //X1->getStateSpace()->interpolate(q_from->state, q_to->state, step_size, q_interp->state);

//    if(q_to->coset == nullptr || q_from->coset == nullptr)
//    {
//      Q1->getStateSpace()->interpolate(q_from->state, q_to->state, step_size, q_interp->state);
//    }else{
//      std::vector<const Configuration*> path = GetInterpolationPath(q_from, q_to);

//      double d_from_to = metric->DistanceConfigurationConfiguration(q_from, q_to);
//      double d_step = d_from_to*step_size;

//      double d = 0;
//      double d_last_to_next = 0;
//      uint ctr = 0;

//      while(d < d_step && ctr < path.size()-1){
//        d_last_to_next = metric->DistanceQ1(path.at(ctr), path.at(ctr+1));
//        d += d_last_to_next;
//        ctr++;
//      }

//      //TODO: Needs revision

//      const Configuration *q_next = path.at(ctr);
//      const Configuration *q_last = path.at(ctr-1);
//      double step = d_last_to_next - (d - d_step);
//      Q1->getStateSpace()->interpolate(q_last->state, q_next->state, step/d_last_to_next, q_interp->state);
//    }
//  }

//  return true;
//}
//void QuotientCover::InterpolateUntilNeighborhoodBoundary(const Configuration *q_center, const Configuration *q_desired, Configuration *q_out)
//{
//  double radius = q_center->GetRadius();
//  double distance_center_desired = metric->DistanceConfigurationConfiguration(q_center, q_desired);
//  double step = (radius)/distance_center_desired;

//  metric->InterpolateQ1(q_center, q_desired, step, q_out);

//  //############################################################################
//  //DEBUG
//  //############################################################################
//  double d_center_outward = metric->DistanceConfigurationConfiguration(q_center, q_out);
//  if(fabs(d_center_outward - radius) > 1e-10){
//    std::cout << "WARNING: interpolated point outside boundary" << std::endl;
//    QuotientCover::Print(q_out, false);
//    QuotientCover::Print(q_center, false);
//    std::cout << "Distance: " << d_center_outward << " Radius: " << radius << std::endl;
//    exit(0);
//  }
//}

// bool QuotientCover::InterpolateOnBoundary(const Configuration* q_center, const Configuration* q1, const Configuration* q2, double step, Configuration* q_out)
// {
//   Q1->getStateSpace()->interpolate(q1->state, q2->state, step, q_out->state);
//   ProjectConfigurationOntoNeighborhoodBoundary(q_center, q_out);
//   return true;
// }
