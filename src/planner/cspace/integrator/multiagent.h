#pragma once
#include "tangentbundle.h"

// using namespace Math3D;
class CSpaceOMPL;
class CSpaceOMPLMultiAgent;

#include <ompl/base/StateSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace oc = ompl::control;
namespace ob = ompl::base;

class MultiAgentIntegrator : public oc::StatePropagator
{
  public:
    MultiAgentIntegrator(const oc::SpaceInformationPtr &si, CSpaceOMPLMultiAgent *cspace, std::vector<CSpaceOMPL*> cspaces);

    virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override;

  protected:
    CSpaceOMPLMultiAgent *cspace_;
    std::vector<CSpaceOMPL*> cspaces_;
    std::vector<SingleRobotCSpace*> klampt_single_robot_cspaces_;
};

