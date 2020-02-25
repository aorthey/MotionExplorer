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
  for(uint k = 0; k < cspaces_.size(); k++){
    CSpaceOMPL *ck = cspaces_.at(k);

    if(ck->isDynamic())
    {
      const ob::State *statek = static_cast<const ob::CompoundState*>(state)->as<ob::State>(k);
      const oc::Control *controlk = static_cast<const oc::CompoundControl*>(control)->as<oc::Control>(k);
      ob::State *resultk = static_cast<ob::CompoundState*>(result)->as<ob::State>(k);

      ob::SpaceInformationPtr sik = ck->SpaceInformationPtr();
      oc::SpaceInformationPtr siC = static_pointer_cast<oc::SpaceInformation>(sik);
      siC->getStatePropagator()->propagate(statek, controlk, duration, resultk);
    }else{
      OMPL_ERROR("NYI");
      throw "NYI";
    }
  }
}

// protected:
// CSpaceOMPLMultiAgent *cspace_;
// std::vector<CSpaceOMPL*> cspaces_;
// std::vector<SingleRobotCSpace*> klampt_single_robot_cspaces_;
// };

