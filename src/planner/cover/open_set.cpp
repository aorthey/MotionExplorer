#include "open_set.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace cover;

OpenSet::OpenSet(CSpaceOMPL *cspace_, const ob::State *s):
  cspace(cspace_), sCenter(s)
{
}

OpenSet::~OpenSet()
{
}

const ob::State* OpenSet::GetCenter() const{
  return sCenter;
}
