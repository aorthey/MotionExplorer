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
#include "omplklamptconverter.h"


using namespace Math3D;

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


#include <ompl/base/StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
namespace oc = ompl::control;
namespace ob = ompl::base;

class TangentBundleIntegrator : public oc::StatePropagator
{
public:

    TangentBundleIntegrator(oc::SpaceInformationPtr si) : 
        oc::StatePropagator(si.get())
    {
    }
    virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override;
};

class TangentBundleOMPLValidityChecker : public ob::StateValidityChecker
{
  public:
    TangentBundleOMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpace *space);
    virtual bool isValid(const ob::State* state) const;
    CSpace *cspace_;
};

