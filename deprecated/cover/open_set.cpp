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

bool OpenSet::IsSubsetOf(const cover::OpenSet *rhs, double tolerance) const
{
  std::cout << "NYI" << std::endl;
  throw "NYI";
}

const ob::State* OpenSet::GetCenter() const
{
  return sCenter;
}

std::ostream& OpenSet::Print(std::ostream& out) const
{
  out << std::string(80, '-') << std::endl;
  out << "OpenSet" << std::endl;
  out << std::string(80, '-') << std::endl;
  out << "Center: " << std::endl;
  cspace->SpaceInformationPtr()->printState(sCenter, out);
  return out;
}

namespace cover{
  std::ostream& operator<< (std::ostream& out, const OpenSet& set)
  {
    set.Print(out);
    return out;
  }
}
