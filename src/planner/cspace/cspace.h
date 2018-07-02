#pragma once
#include "planner/cspace/cspace_input.h"
#include "klampt.h"

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
#include <Planning/RobotCSpace.h>

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
  friend class CSpaceOMPLDecorator;
  friend class CSpaceOMPLDecoratorNecessarySufficient;
  public:

    CSpaceOMPL(RobotWorld *world_, int robot_idx_);

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si) = 0;
    virtual void initSpace() = 0;
    virtual void print() const = 0;

    virtual ob::ScopedState<> ConfigToOMPLState(const Config &q);
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) = 0;
    virtual Config OMPLStateToConfig(const ob::State *qompl) = 0;

    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr();
    virtual Vector3 getXYZ(const ob::State*);
    virtual void SetCSpaceInput(const CSpaceInput &input_);
    virtual uint GetDimensionality() const;
    virtual uint GetKlamptDimensionality() const;
    virtual uint GetControlDimensionality() const;
    virtual Config OMPLStateToConfig(const ob::ScopedState<> &qompl);
    virtual Robot* GetRobotPtr();
    virtual RobotWorld* GetWorldPtr();
    virtual CSpaceKlampt* GetCSpacePtr();
    virtual const ob::StateSpacePtr SpacePtr();
    virtual ob::SpaceInformationPtr SpaceInformationPtr();
    virtual const oc::RealVectorControlSpacePtr ControlSpacePtr();

    std::vector<double> EulerXYZFromOMPLSO3StateSpace( const ob::SO3StateSpace::StateType *q );
    void OMPLSO3StateSpaceFromEulerXYZ( double x, double y, double z, ob::SO3StateSpace::StateType *q );

    bool isDynamic();
    bool isFixedBase();
    bool isFreeFloating();

    friend std::ostream& operator<< (std::ostream& out, const CSpaceOMPL& space);
    virtual void print(std::ostream& out) const;

  protected:
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si) = 0;

    CSpaceInput input;

    uint Nklampt;
    uint Nompl;
    bool fixedBase{false};
    bool kinodynamic{false};

    //klampt:
    //
    // SE(3) x R^Nklampt
    //
    //ompl:
    //
    // SE(3) x R^Nompl
    //
    // ompl_to_klampt: maps a dimension in R^Nompl to SE(3)xR^Nklampt
    // klampt_to_ompl: maps a dimension in SE(3)xR^Nklampt to R^Nompl
    std::vector<int> ompl_to_klampt;
    std::vector<int> klampt_to_ompl;

    ob::SpaceInformationPtr si;
    ob::StateSpacePtr space;
    oc::RealVectorControlSpacePtr control_space;

    Robot *robot;
    CSpaceKlampt *kspace;
    RobotWorld *world;
    WorldPlannerSettings worldsettings;
    int robot_idx;

};
