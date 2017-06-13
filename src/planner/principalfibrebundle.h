#pragma once
#include <KrisLibrary/planning/KinodynamicCSpace.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math/diffeq.h>
#include <Planning/RobotCSpace.h>
#include <vector>
#include "liegroupintegrator.h"
#include "ompl_space.h"


using namespace Math3D;

class CSpaceOMPL;
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

class PrincipalFibreBundleAdaptor: public KinematicCSpaceAdaptor
{

  //TODO: only IsFeasible is used in validitychecker right now, maybe we can
  //ompl-merize everything using PQP
  public:

    PrincipalFibreBundleAdaptor(CSpace *base);

    virtual void Sample(Config& x) 
    { 
      base->Sample(x);
    }

    virtual Real Distance(const Config& x, const Config& y);

    virtual bool IsFeasible(const Config& x) { return base->IsFeasible(x); }
    //virtual void Simulate(const State& x0, const ControlInput& u,std::vector<State>& p);
    virtual void SimulateEndpoint(const State& x0, const ControlInput& u,State& x1);

    //required implementation
    virtual EdgePlanner* TrajectoryChecker(const std::vector<State>& p);
    virtual bool IsValidControl(const State& x,const ControlInput& u);
    virtual void SampleControl(const State& x,ControlInput& u);

    virtual bool ConnectionControl(const State& x,const State& xGoal,ControlInput& u) { return false; }
    virtual bool ReverseControl(const State& x0,const State& x1,ControlInput& u) { return false; }
    virtual void BiasedSampleReverseControl(const State& x1,const State& xDest,ControlInput& u);
    virtual void BiasedSampleControl(const State& x,const State& xGoal,ControlInput& u);

};

#include <ompl/base/StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
namespace oc = ompl::control;
namespace ob = ompl::base;

class PrincipalFibreBundleIntegrator : public oc::StatePropagator
{
public:

    PrincipalFibreBundleIntegrator(oc::SpaceInformationPtr si, CSpaceOMPL *ompl_space_) : 
        oc::StatePropagator(si.get()), ompl_space(ompl_space_)
    {
    }
    virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override;
    CSpaceOMPL *ompl_space;
};

class PrincipalFibreBundleOMPLValidityChecker : public ob::StateValidityChecker
{
  public:
    PrincipalFibreBundleOMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpace *space, CSpaceOMPL *ompl_space_);
    virtual bool isValid(const ob::State* state) const;
    CSpace *cspace_;
    CSpaceOMPL *ompl_space;
};
