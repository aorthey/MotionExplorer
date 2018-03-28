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
namespace cover{
  std::ostream& operator<< (std::ostream& out, const OpenSet& set)
  {
    out << std::string(80, '-') << std::endl;
    out << "OpenSet" << std::endl;
    out << std::string(80, '-') << std::endl;
    out << "Center: " << std::endl;
    set.cspace->SpaceInformationPtr()->printState(set.sCenter, out);
    return out;
  }
}
