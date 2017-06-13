#pragma once
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include "principalfibrebundle.h"
#include "tangentbundle.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

namespace ompl{
  namespace control{
    typedef std::shared_ptr<RealVectorControlSpace> RealVectorControlSpacePtr;
    typedef std::shared_ptr<StatePropagator> StatePropagatorPtr;
  }
  namespace base{
    typedef std::shared_ptr<StateValidityChecker> StateValidityCheckerPtr;
  }
}

class CSpaceOMPL
{
  public:

    CSpaceOMPL(Robot *robot_, CSpace *kspace_);

    const ob::StateSpacePtr SpacePtr(){
      return space;
    }
    const oc::RealVectorControlSpacePtr ControlSpacePtr(){
      return control_space;
    }

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si) = 0;
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(oc::SpaceInformationPtr si) = 0;

    virtual ob::ScopedState<> ConfigToOMPLState(const Config &q) = 0;
    virtual ob::State* ConfigToOMPLStatePtr(const Config &q) = 0;

    virtual Config OMPLStateToConfig(const ob::ScopedState<> &qompl) = 0;
    virtual Config OMPLStateToConfig(const ob::State *qompl) = 0;

    virtual void initSpace() = 0;
    virtual void initControlSpace() = 0;

  protected:

    ob::StateSpacePtr space;
    oc::RealVectorControlSpacePtr control_space;

    CSpace *kspace;
    Robot *robot;
};
class GeometricCSpaceOMPL: public CSpaceOMPL
{
  public:
    GeometricCSpaceOMPL(Robot *robot_, CSpace *kspace_);

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si);
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(oc::SpaceInformationPtr si);

    virtual void initSpace();
    virtual void initControlSpace();

    virtual ob::ScopedState<> ConfigToOMPLState(const Config &q);
    virtual ob::State* ConfigToOMPLStatePtr(const Config &q);

    virtual Config OMPLStateToConfig(const ob::ScopedState<> &qompl);
    virtual Config OMPLStateToConfig(const ob::State *qompl);

    Config OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState);
};


class KinodynamicCSpaceOMPL: public CSpaceOMPL
{
  public:
    KinodynamicCSpaceOMPL(Robot *robot_, CSpace *kspace_);

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si);
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(oc::SpaceInformationPtr si);

    virtual void initSpace();
    virtual void initControlSpace();

    virtual ob::ScopedState<> ConfigToOMPLState(const Config &q);
    virtual ob::State* ConfigToOMPLStatePtr(const Config &q);

    virtual Config OMPLStateToConfig(const ob::ScopedState<> &qompl);
    virtual Config OMPLStateToConfig(const ob::State *qompl);

    Config OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState, const ob::RealVectorStateSpace::StateType *qomplTMState);
};


class CSpaceFactory{
  public:
    CSpaceFactory(){};
    virtual GeometricCSpaceOMPL* MakeGeometricCSpace( Robot *robot, CSpace *kspace){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPL(robot, kspace);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    virtual KinodynamicCSpaceOMPL* MakeKinodynamicCSpace( Robot *robot, CSpace *kspace){
      KinodynamicCSpaceOMPL *cspace = new KinodynamicCSpaceOMPL(robot, kspace);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
};
