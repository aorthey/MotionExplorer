#include "open_set_hypersphere.h"
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>

using namespace cover;

OpenSetHypersphere::OpenSetHypersphere(ob::SpaceInformationPtr si_, ob::State *s, double dist_robot_env_):
  OpenSet(si_, s, dist_robot_env_)
{
}

double OpenSetHypersphere::Distance(const ob::State *s_lhs, const ob::State *s_rhs){
  return si->distance(s_lhs, s_rhs);
}

