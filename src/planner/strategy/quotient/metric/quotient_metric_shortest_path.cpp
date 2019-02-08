#include "quotient_metric_shortest_path.h"
using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

QuotientMetricShortestPath::QuotientMetricShortestPath(og::QuotientCover* quotient_cover_):
  QuotientMetric(quotient_cover_)
{
}

//############################################################################
//Distance Functions
//############################################################################
double QuotientMetricShortestPath::DistanceConfigurationConfiguration(const Configuration *q_from, const Configuration *q_to) 
{
  return DistanceQ1(q_from, q_to);
}
//############################################################################
//Interpolate Functions
//############################################################################
void QuotientMetricShortestPath::Interpolate(const Configuration *q_from, const Configuration *q_to, double step_size, Configuration *q_interp)
{
  return quotient_cover->GetQ1()->getStateSpace()->interpolate(q_from->state, q_to->state, step_size, q_interp->state);
}

//Weigth can be set to zero (there will be only one edge anyway)
//
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
//std::vector<const QuotientCover::Configuration*> QuotientCover::GetInterpolationPath(const Configuration *q_from, const Configuration *q_to)
//{
//  if(q_from->coset == nullptr || q_to->coset == nullptr){
//    OMPL_ERROR("Cannot interpolate without cosets");
//    std::cout << std::string(80, '#') << std::endl;
//    std::cout << "[ERROR] could not find coset for a configuration" << std::endl;
//    std::cout << std::string(80, '#') << std::endl;
//    std::cout << "from:"; Print(q_from, false);
//    std::cout << std::string(80, '-') << std::endl;
//    std::cout << "to:"; Print(q_to);
//    std::cout << std::string(80, '-') << std::endl;
//    std::cout << *this << std::endl;
//    throw "InterpolateWithoutCosets";
//    exit(1);
//  }
//  std::vector<const Configuration*> path_Q1;

//  og::QuotientCover *parent_chart = dynamic_cast<og::QuotientCover*>(parent);

//  //(0) get shortest path on Q0 between q_from->coset to q_to->coset
//  std::vector<const Configuration*> path_Q0_cover = parent_chart->GetCoverPath(q_from->coset, q_to->coset);

//  if(path_Q0_cover.size()<=1){
//    //(1) cosets are equivalent => straight line interpolation
//    path_Q1.push_back(q_from);
//    path_Q1.push_back(q_to);
//  }else{

//    //Assume that we have at least two non-equal configurations in cover
//    path_Q0_cover.erase(path_Q0_cover.begin());
//    path_Q0_cover.pop_back();

//    if(path_Q0_cover.empty()){
//      //(2) cosets are neighbors => straight line interpolation
//      path_Q1.push_back(q_from);
//      path_Q1.push_back(q_to);
//    }else{
//      //(3) cosets have nonzero path between them => interpolate along that path
//      ob::State *s_fromQ0 = Q0->allocState();
//      ProjectQ0Subspace(q_from->state, s_fromQ0);
//      Configuration *q_fromQ0 = new Configuration(Q0, s_fromQ0);

//      ob::State *s_toQ0 = Q0->allocState();
//      ProjectQ0Subspace(q_to->state, s_toQ0);
//      Configuration *q_toQ0 = new Configuration(Q0, s_toQ0);

//      //distance along cover path
//      double d = 0;
//      std::vector<double> d_vec;
//      double d0 = parent_chart->GetMetric()->DistanceConfigurationConfiguration(q_fromQ0, path_Q0_cover.at(0));
//      d_vec.push_back(d0); d+=d0;
//      for(uint k = 1; k < path_Q0_cover.size(); k++){
//        double dk = parent_chart->GetMetric()->DistanceConfigurationConfiguration(path_Q0_cover.at(k-1), path_Q0_cover.at(k));
//        d_vec.push_back(dk); d+=dk;
//      }
//      double d1= parent_chart->GetMetric()->DistanceConfigurationConfiguration(path_Q0_cover.back(), q_toQ0);
//      d_vec.push_back(d1); d+=d1;

//      q_toQ0->Remove(Q0);
//      q_fromQ0->Remove(Q0);
//      Q0->freeState(s_toQ0);
//      Q0->freeState(s_fromQ0);

//      //interpolate on X1
//      ob::State *s_fromX1 = X1->allocState();
//      ProjectX1Subspace(q_from->state, s_fromX1);
//      ob::State *s_toX1 = X1->allocState();
//      ProjectX1Subspace(q_to->state, s_toX1);

//      double d_next = 0;
//      path_Q1.push_back(q_from);
//      for(uint k = 0; k < path_Q0_cover.size(); k++){
//        ob::State *s_kX1 = X1->allocState();
//        d_next += d_vec.at(k);
//        X1->getStateSpace()->interpolate(s_fromX1, s_toX1, d_next/d, s_kX1);

//        Configuration *qk = new Configuration(Q1);
//        MergeStates(path_Q0_cover.at(k)->state, s_kX1, qk->state);
//        path_Q1.push_back(qk);
//        X1->freeState(s_kX1);
//      }
//      path_Q1.push_back(q_to);

//      X1->freeState(s_toX1);
//      X1->freeState(s_fromX1);
//    }
//  }
//  return path_Q1;
//}
