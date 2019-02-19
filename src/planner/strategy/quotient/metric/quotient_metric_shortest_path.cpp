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
  if(quotient_cover->GetParent()==nullptr){
    return DistanceQ1(q_from, q_to);
  }else{
    std::vector<const Configuration*> path = GetInterpolationPath(q_from, q_to);
    double d = 0;
    for(uint k = 0; k < path.size()-1; k++){
      d += quotient_cover->GetMetric()->DistanceQ1(path.at(k), path.at(k+1));
    }
    return d;
  }
}
//############################################################################
//Interpolate Functions
//############################################################################
void QuotientMetricShortestPath::Interpolate(const Configuration *q_from, const Configuration *q_to, Configuration *q_interp)
{
  if(quotient_cover->GetParent()==nullptr){
    BaseT::Interpolate(q_from, q_to, q_interp);
  }else{
    Interpolate(q_from, q_to, q_from->GetRadius(), q_interp);
  }
}
void QuotientMetricShortestPath::Interpolate(const Configuration *q_from, const Configuration *q_to, const double step_size, Configuration *q_interp)
{
  if(quotient_cover->GetParent()==nullptr){
    InterpolateQ1(q_from, q_to, step_size, q_interp);
  }else{
    std::vector<const Configuration*> path = GetInterpolationPath(q_from, q_to);
    InterpolateAlongPath(q_from, path, step_size, q_interp);
  }
}
std::vector<const QuotientCover::Configuration*> QuotientMetricShortestPath::GetInterpolationPath(const Configuration *q_from, const Configuration *q_to)
{
  std::cout << "Interpolate Shortest Path" << std::endl;
  std::vector<const Configuration*> path_Q1;

  og::QuotientCover *parent_chart = dynamic_cast<og::QuotientCover*>(quotient_cover->GetParent());
  const ob::SpaceInformationPtr &Q0 = quotient_cover->GetQ0();
  const ob::SpaceInformationPtr &Q1 = quotient_cover->GetQ1();

  ob::State* s_from_Q0 = Q0->allocState();
  ob::State* s_to_Q0 = Q0->allocState();

  quotient_cover->ProjectQ0Subspace(q_from->state, s_from_Q0);
  quotient_cover->ProjectQ0Subspace(q_to->state, s_to_Q0);

  Configuration *q_from_Q0 = new Configuration(Q0, s_from_Q0);
  Configuration *q_to_Q0 = new Configuration(Q0, s_to_Q0);

  const Configuration *q_from_nearest_Q0 = parent_chart->NearestConfiguration( q_from_Q0 );
  const Configuration *q_to_nearest_Q0 = parent_chart->NearestConfiguration( q_to_Q0 );

  std::vector<const Configuration*> path_Q0_cover = parent_chart->GetCoverPath(q_from_nearest_Q0, q_to_nearest_Q0);

  if(path_Q0_cover.size()<=1){
    //(1) cosets are equivalent => straight line interpolation
    path_Q1.push_back(q_from);
    path_Q1.push_back(q_to);
  }else{

    //Assume that we have at least two non-equal configurations in cover
    path_Q0_cover.erase(path_Q0_cover.begin());
    path_Q0_cover.pop_back();

    if(path_Q0_cover.empty()){
      //(2) cosets are neighbors => straight line interpolation
      path_Q1.push_back(q_from);
      path_Q1.push_back(q_to);
    }else{
      //(3) cosets have nonzero path between them => interpolate along that path

      //distance along cover path
      double d = 0;
      std::vector<double> d_vec;
      double d0 = parent_chart->GetMetric()->DistanceConfigurationConfiguration(q_from_Q0, path_Q0_cover.at(0));
      d_vec.push_back(d0); d+=d0;
      for(uint k = 1; k < path_Q0_cover.size(); k++){
        double dk = parent_chart->GetMetric()->DistanceConfigurationConfiguration(path_Q0_cover.at(k-1), path_Q0_cover.at(k));
        d_vec.push_back(dk); d+=dk;
      }
      double d1= parent_chart->GetMetric()->DistanceConfigurationConfiguration(path_Q0_cover.back(), q_to_Q0);
      d_vec.push_back(d1); d+=d1;

      // q_to_Q0->Remove(Q0);
      // q_from_Q0->Remove(Q0);

      //interpolate on X1 (if not identity space)
      if(quotient_cover->GetX1Dimension()>0){
        const ob::SpaceInformationPtr &X1 = quotient_cover->GetX1();
        ob::State *s_fromX1 = X1->allocState();
        quotient_cover->ProjectX1Subspace(q_from->state, s_fromX1);
        ob::State *s_toX1 = X1->allocState();
        quotient_cover->ProjectX1Subspace(q_to->state, s_toX1);

        double d_next = 0;
        path_Q1.push_back(q_from);
        for(uint k = 0; k < path_Q0_cover.size(); k++){
          ob::State *s_kX1 = X1->allocState();
          d_next += d_vec.at(k);
          X1->getStateSpace()->interpolate(s_fromX1, s_toX1, d_next/d, s_kX1);

          Configuration *qk = new Configuration(Q1);
          quotient_cover->MergeStates(path_Q0_cover.at(k)->state, s_kX1, qk->state);
          path_Q1.push_back(qk);
          X1->freeState(s_kX1);
        }
        path_Q1.push_back(q_to);

        X1->freeState(s_toX1);
        X1->freeState(s_fromX1);
      }else{
        //just copy from Q0 (is identity space, so copy function should be well
        //defined)
        path_Q1.push_back(q_from);
        for(uint k = 0; k < path_Q0_cover.size(); k++){
          Configuration *qk = new Configuration(Q1);
          Q1->copyState(qk->state, path_Q0_cover.at(k)->state);
          path_Q1.push_back(qk);
        }
        path_Q1.push_back(q_to);
      }
    }
  }

  q_to_Q0->Remove(Q0);
  q_from_Q0->Remove(Q0);
  Q0->freeState(s_from_Q0);
  Q0->freeState(s_to_Q0);
  return path_Q1;
}

uint QuotientMetricShortestPath::InterpolateAlongPath(const Configuration *q_from, std::vector<const Configuration*> path, const double step_size, Configuration *q_interp)
{
  double d = 0;
  double d_last_to_next = 0;
  uint ctr = 0;

  // std::cout << "radius: " << q_from->GetRadius() << std::endl;
  // std::cout << "step s: " << step_size << std::endl;
  while(d < q_from->GetRadius() && ctr < path.size()-1){
    d_last_to_next = quotient_cover->GetMetric()->DistanceQ1(path.at(ctr), path.at(ctr+1));
    d += d_last_to_next;
    ctr++;
  }

  //NOTE: We only need to find the single Configuration at distance step_size

  //std::cout << "interpolate between config " << ctr-1 << " and " << ctr << " along path." << std::endl;
  const Configuration *q_next = path.at(ctr);
  const Configuration *q_last = path.at(ctr-1);

  //|--------------------- d ----------------------------|
  //|----------------- step_size ----------|
  //                                |-- d_last_to_next --|
  //                                |-step-|
  //q1 ----- q2 ------ ... ----- q_last ---|---------- q_next
  //
  // it follows that step = d_last_to_next - (d-step_size)

  double step = d_last_to_next - (d - q_from->GetRadius());

  InterpolateQ1(q_last, q_next, step/d_last_to_next, q_interp);

  ////SANITY CHECK
  double dfp = DistanceQ1(q_from, q_interp);
  //double d_m_last = DistanceQ1(q_interp, q_last);
  //double d_m_next = DistanceQ1(q_interp, q_next);
  //double d_last_next = DistanceQ1(q_last, q_next);

  //// std::cout << "milestone distance to first: " << dfp << " (radius="<<q_from->GetRadius()<<")" << std::endl;
  //// std::cout << "milestone distance to q_last: " << DistanceQ1(q_interp, q_last) << std::endl;
  //// std::cout << "milestone distance to q_next: " << DistanceQ1(q_interp, q_next) << std::endl;
  //// std::cout << "q_last    distance to q_next: " << DistanceQ1(q_last, q_next) << std::endl;
  //if( (d_m_last + d_m_next) < d_last_next-1e-5 ){
  //  std::cout << "TRIANGLE INEQ not OK" << std::endl;
  //  std::cout << "d_m_last+d_m_next = " << d_m_last + d_m_next << " < " << d_last_next << std::endl;
  //  exit(0);
  //}

  if(fabs(dfp-q_from->GetRadius()) > 1e-10){
    std::cout << "needs adjust" << std::endl;
    //Project onto NBH
    double step_size = q_from->GetRadius()/dfp;
    InterpolateQ1(q_from, q_interp, step_size, q_interp);
    if(fabs(DistanceQ1(q_from, q_interp)-q_from->GetRadius()) > 1e-10){
      std::cout << "Interpolated outside cover" << std::endl;
      quotient_cover->QuotientCover::Print(q_from, false);
      quotient_cover->QuotientCover::Print(q_interp, false);
      std::cout << "desired step size: " << step_size << std::endl;
      std::cout << "actual step size : " << dfp << std::endl;
      std::cout << "radius NBH       : " << q_from->GetRadius() << std::endl;
      std::cout << "path size: " << path.size() << std::endl;
      std::cout << "ctr size: " << ctr << std::endl;
      std::cout << "dist q_from-q_last   : " << DistanceQ1(q_from, q_last) << std::endl;
      std::cout << "dist q_from-q_next   : " << d << std::endl;
      exit(0);
    }
  }
  return ctr;
}

