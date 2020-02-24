#include "multiagent.h"

namespace oc = ompl::control;
MultiAgentIntegrator::MultiAgentIntegrator(
    const oc::SpaceInformationPtr &si, 
    CSpaceOMPLMultiAgent *cspace, 
    std::vector<CSpaceOMPL*> cspaces):

    oc::StatePropagator(si), 
    cspace_(cspace), 
    cspaces_(cspaces)
{
  for(uint k = 0; k < cspaces_.size(); k++){
    SingleRobotCSpace *kck = static_cast<SingleRobotCSpace*>(cspaces.at(k)->GetCSpaceKlamptPtr());
    klampt_single_robot_cspaces_.push_back(kck);
  }
}

void MultiAgentIntegrator::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const
{
  OMPL_ERROR("NYI");
  throw "NYI";
}

// protected:
// CSpaceOMPLMultiAgent *cspace_;
// std::vector<CSpaceOMPL*> cspaces_;
// std::vector<SingleRobotCSpace*> klampt_single_robot_cspaces_;
// };

