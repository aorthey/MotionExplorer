#pragma once
#include <KrisLibrary/planning/KinodynamicCSpace.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math/diffeq.h>
#include <Planning/RobotCSpace.h>
#include <vector>
#include "planner/cspace/integrator/liegroupintegrator.h"
#include "planner/cspace/cspace.h"

using namespace Math3D;
class KinodynamicCSpaceOMPL;

//  The tangent bundle (or state space or phase space)  is represented as
//
//  X = SE(3) x R^N x se(3) x R^N = (SE(3) x R^N) x R^{6+N}
//
//  the acceleration space is TX, the tangent bundle of X
//  plus a time step dimension (the last dimension)
//
//  TX \union R = R^{6+N+1} 
//
//   the time step-dimension is used as an adaptive time step for integration.
//   We use usual values between 0.01 and 0.1 for the time step.
//
//  the control space is U

#include <ompl/base/StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace oc = ompl::control;
namespace ob = ompl::base;

class TangentBundleIntegrator : public oc::StatePropagator
{
public:

    TangentBundleIntegrator(oc::SpaceInformationPtr si, KinodynamicCSpaceOMPL *cspace_) : 
        oc::StatePropagator(si.get()), cspace(cspace_)
    {
    }
    virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override;
    virtual void propagate_deprecated(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const;

    KinodynamicCSpaceOMPL *cspace;
};

