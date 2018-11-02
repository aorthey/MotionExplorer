#pragma once
#include <KrisLibrary/planning/KinodynamicCSpace.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math/diffeq.h>
#include <Planning/RobotCSpace.h>
#include <vector>
#include "planner/integrator/liegroupintegrator.h"

using namespace Math3D;

class KinodynamicCSpaceOMPL;
// Convention:
//  The principal fibre bundle is represented as
//
//  M = SE(3) x R^N 
//
//  its tangent space is 
//
//  TM = R^{6+N}
//
//  and the control space is the tangent space of the tangent space plus one
//  time step dimension (the last dimension)
//
//  TTM \union R = U = R^{6+N+1} 
//
//   the time step-dimension is used as an adaptive time step for integration.
//   We use usual values between 0.01 and 0.1 for the time step.

#include <ompl/base/StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
namespace oc = ompl::control;
namespace ob = ompl::base;

class PrincipalFibreBundleIntegrator : public oc::StatePropagator
{
public:

    PrincipalFibreBundleIntegrator(oc::SpaceInformationPtr si, KinodynamicCSpaceOMPL *cspace_) : 
        oc::StatePropagator(si), cspace(cspace_)
    {
    }
    virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override;
    KinodynamicCSpaceOMPL *cspace;
};

// class PrincipalFibreBundleOMPLValidityChecker : public ob::StateValidityChecker
// {
//   public:
//     PrincipalFibreBundleOMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpace *space, CSpaceOMPL *ompl_space_);
//     virtual bool isValid(const ob::State* state) const;
//     CSpace *cspace_;
//     CSpaceOMPL *ompl_space;
// };
