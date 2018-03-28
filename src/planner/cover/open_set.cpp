#include "open_set.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace cover;

OpenSet::OpenSet(const ob::State *s):
  sCenter(s)
{
}

OpenSet::~OpenSet()
{
}

const ob::State* OpenSet::GetCenter() const{
  return sCenter;
}
