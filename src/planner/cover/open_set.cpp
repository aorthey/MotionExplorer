#include "open_set.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace cover;

OpenSet::OpenSet(ob::SpaceInformationPtr si_, ob::State *s, double dist_robot_env_):
  si(si_), sCenter(s), dist_robot_env(dist_robot_env_)
{
}

ob::State* OpenSet::GetCenter() const{
  return sCenter;
}
double OpenSet::GetRadius() const{
  return dist_robot_env;
}
void OpenSet::SetRadius(double d_)
{
  dist_robot_env = d_;
}

void OpenSet::IntersectionTowards(const ob::State *sGoal, ob::State *sIntersected){
  double d = Distance(sCenter, sGoal);
  if(d < dist_robot_env){
    sIntersected = si->cloneState(sGoal);
    return;
  }
  double t1 = +dist_robot_env/d;
  //double t2 = -dist_robot_env/d;
  si->getStateSpace()->interpolate(sCenter, sGoal, t1, sIntersected);
}

bool OpenSet::Contains(ob::State *sPrime)
{
  return Distance(sCenter, sPrime) < dist_robot_env;
}
