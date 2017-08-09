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
#include "planner_input.h"

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

    uint GetDimensionality(){
      return space->getDimension();
    }
    uint GetControlDimensionality(){
      return control_space->getDimension();
    }
    void SetPlannerInput(PlannerInput &input_){
      input = input_;
    }

    virtual void print() = 0;

  protected:
    PlannerInput input;

    ob::StateSpacePtr space;
    oc::RealVectorControlSpacePtr control_space;

    CSpace *kspace;
    Robot *robot;
};
//GeometricCSpaceOMPL: Space = Configuration manifold; control space = tangent
//space of configuration manifold, i.e. control happens in velocity space
//
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

    virtual void print();
};

//KinodynamicCSpaceOMPL: Space = tangent bundle of configuration manifold;
//control space = tangent space of tangent bundle, i.e. control happens in
//acceleration space, i.e. we can control torques of revolute joints, forces
//of prismatic joints, and any additional booster/thruster which act directly
//on the se(3) component

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

    virtual void print();
};

//KinodynamicCSpaceOMPLInnerOuter: same as KinodynamicCSpaceOMPL but there is
//one more robot added. This robot is called outer shell robot. A configuration
//is valid iff the original robot is feasible AND the outer shell robot is
//infeasible. In this case the original robot can be in contact with the
//environment. 
//

class KinodynamicCSpaceOMPLInnerOuter: public KinodynamicCSpaceOMPL
{
  public:
    KinodynamicCSpaceOMPLInnerOuter(Robot *robot_inner, CSpace *inner, CSpace *outer);

    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(oc::SpaceInformationPtr si);

    CSpace *inner;
    CSpace *outer;
};
class GeometricCSpaceOMPLInnerOuter: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLInnerOuter(Robot *robot_inner, CSpace *inner, CSpace *outer);

    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(oc::SpaceInformationPtr si);

    CSpace *inner;
    CSpace *outer;
};

class CSpaceFactory{
  private:
    PlannerInput input;
  public:
    CSpaceFactory(PlannerInput &input_): input(input_){};
    virtual GeometricCSpaceOMPL* MakeGeometricCSpace( Robot *robot, CSpace *kspace){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPL(robot, kspace);
      cspace->SetPlannerInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    virtual KinodynamicCSpaceOMPL* MakeKinodynamicCSpace( Robot *robot, CSpace *kspace){
      KinodynamicCSpaceOMPL *cspace = new KinodynamicCSpaceOMPL(robot, kspace);
      cspace->SetPlannerInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    virtual KinodynamicCSpaceOMPL* MakeKinodynamicCSpaceInnerOuter( Robot *robot, CSpace *inner, CSpace *outer ){
      KinodynamicCSpaceOMPL *cspace = new KinodynamicCSpaceOMPLInnerOuter(robot, inner, outer);
      cspace->SetPlannerInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceInnerOuter( Robot *robot, CSpace *inner, CSpace *outer ){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLInnerOuter(robot, inner, outer);
      cspace->SetPlannerInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
};
