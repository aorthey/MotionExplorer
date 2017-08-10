#pragma once
#include <KrisLibrary/planning/KinodynamicCSpace.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math/diffeq.h>
#include <Planning/RobotCSpace.h>
#include <vector>
#include "liegroupintegrator.h"
#include "principalfibrebundle.h"
#include "cspace.h"

using namespace Math3D;
class CSpaceOMPL;

// Convention:
//  The tangent bundle is represented as
//
//  M = SE(3) x R^N x se(3) x R^N = (SE(3) x R^N) x R^{6+N}
//
//  whereby SE(3)xR^N is the base space which is itself a principal fibre bundle
//
//  the control space is the tangent space of the tangent bundle plus one
//  time step dimension (the last dimension)
//
//  TM \union R = U = R^{6+N+1} 
//
//   the time step-dimension is used as an adaptive time step for integration.
//   We use usual values between 0.01 and 0.1 for the time step.
//
//   each configuration is with respect to an interial frame centered at (0,0,0)
//   each velocity is spatial, i.e. w.r.t. (0,0,0)
//

#include <ompl/base/StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace oc = ompl::control;
namespace ob = ompl::base;

class TangentBundleIntegrator : public oc::StatePropagator
{
public:

    TangentBundleIntegrator(oc::SpaceInformationPtr si, Robot *robot_, CSpaceOMPL *ompl_space_) : 
        oc::StatePropagator(si.get()), robot(robot_), ompl_space(ompl_space_)
    {
    }
    virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override;

    Robot *robot;
    CSpaceOMPL *ompl_space;
};

class TangentBundleOMPLValidityChecker : public ob::StateValidityChecker
{
  public:
    TangentBundleOMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpace *space, CSpaceOMPL *ompl_space_);
    virtual bool isValid(const ob::State* state) const;
    CSpace *cspace_;
    CSpaceOMPL *ompl_space;
};

