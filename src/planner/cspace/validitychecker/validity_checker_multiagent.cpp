#include "planner/cspace/validitychecker/validity_checker_multiagent.h"
#include "common.h"
#include <ompl/base/StateSpaceTypes.h>

OMPLValidityCheckerMultiAgent::OMPLValidityCheckerMultiAgent(const ob::SpaceInformationPtr &si, std::vector<CSpaceOMPL*> cspaces):
  ob::StateValidityChecker(si), cspaces_(cspaces)
{
}
bool OMPLValidityCheckerMultiAgent::isValid(const ob::State* state) const
{
  OMPL_WARN("NYI: always true");
  return true;
}

