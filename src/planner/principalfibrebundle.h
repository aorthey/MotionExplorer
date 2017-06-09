#pragma once
#include <KrisLibrary/planning/KinodynamicCSpace.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math/diffeq.h>
#include <Planning/RobotCSpace.h>
#include <vector>
#include "liegroupintegrator.h"
#include "omplklamptconverter.h"


using namespace Math3D;
class PrincipalFibreBundleAdaptor: public KinematicCSpaceAdaptor
{

  protected:
    LieGroupIntegrator integrator;

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
    //virtual bool ReverseSimulate(const State& x1, const ControlInput& u,std::vector<State>& p) { return false; }

//  virtual void SampleReverseControl(const State& x,ControlInput& u);
//  virtual void BiasedSampleReverseControl(const State& x1,const State& xDest,ControlInput& u);

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

    PrincipalFibreBundleIntegrator(oc::SpaceInformationPtr si, PrincipalFibreBundleAdaptor *cspace) : 
        oc::StatePropagator(si.get()), cspace_(cspace)
    {
    }
    virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override;

    PrincipalFibreBundleAdaptor *cspace_;
};

class PrincipalFibreBundleValidityChecker : public oc::StatePropagator
{
public:

    PrincipalFibreBundleValidityChecker(oc::SpaceInformationPtr si, PrincipalFibreBundleAdaptor *cspace) : 
        oc::StatePropagator(si.get()), cspace_(cspace)
    {
    }
    virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override;

    PrincipalFibreBundleAdaptor *cspace_;

};
