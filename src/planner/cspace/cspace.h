#pragma once
#include "planner/cspace/cspace_input.h"
#include "klampt.h"

#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
// #include <ompl/base/PlannerData.h>
// #include <ompl/base/PlannerDataGraph.h>
//#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <Planning/RobotCSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

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

    void Init();
    virtual ob::SpaceInformationPtr SpaceInformationPtr();

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si) = 0;
    virtual void print() const = 0;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) = 0;
    virtual Config OMPLStateToConfig(const ob::State *qompl) = 0;
    virtual double GetTime(const ob::State *qompl);
    virtual bool isTimeDependent();

    Config OMPLStateToConfig(const ob::ScopedState<> &qompl);
    ob::ScopedState<> ConfigToOMPLState(const Config &q);

    const ob::StateValidityCheckerPtr StateValidityCheckerPtr();

    //function to display vertices/edges in the workspace (different from
    //possible quotient-spaces)
    Vector3 getXYZ(const ob::State*);
    Vector3 getXYZ_freeFloating_geometric(const ob::State *s);
    Vector3 getXYZ_freeFloating_dynamic(const ob::State *s);
    Vector3 getXYZ_fixedBase(const ob::State *s);

    bool IsPlanar();

    void SetCSpaceInput(const CSpaceInput &input_);
    uint GetDimensionality() const;
    uint GetKlamptDimensionality() const;
    uint GetControlDimensionality() const;
    Robot* GetRobotPtr();
    int GetRobotIndex() const;
    RobotWorld* GetWorldPtr();
    CSpaceKlampt* GetCSpaceKlamptPtr();
    const ob::StateSpacePtr SpacePtr();
    const oc::RealVectorControlSpacePtr ControlSpacePtr();

    std::vector<double> EulerXYZFromOMPLSO3StateSpace( const ob::SO3StateSpace::StateType *q );
    void OMPLSO3StateSpaceFromEulerXYZ( double x, double y, double z, ob::SO3StateSpace::StateType *q );

    virtual bool isDynamic() const = 0;
    bool isFixedBase();
    bool isFreeFloating();

    friend std::ostream& operator<< (std::ostream& out, const CSpaceOMPL& space);
    virtual void print(std::ostream& out) const;

    void SetSufficient(const uint robot_outer_idx);
    ob::StateSpacePtr GetFirstSubspace();

  protected:
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si);
    virtual void initSpace() = 0;

    CSpaceInput input;

    uint Nklampt;
    uint Nompl;
    bool fixedBase{false};
    bool enableSufficiency{false};

    //klampt:
    // SE(3) x R^Nklampt
    //
    //ompl:
    // SE(3) x R^Nompl
    //
    // ompl_to_klampt: maps a dimension in R^Nompl to SE(3)xR^Nklampt
    // klampt_to_ompl: maps a dimension in SE(3)xR^Nklampt to R^Nompl
    // Note that Nompl <= Nklampt, i.e. all the zero-measure dimensions if any are collapsed
    std::vector<int> ompl_to_klampt;
    std::vector<int> klampt_to_ompl;

    ob::StateValidityCheckerPtr validity_checker;
    ob::SpaceInformationPtr si{nullptr};
    ob::StateSpacePtr space{nullptr};
    oc::RealVectorControlSpacePtr control_space{nullptr};

    Robot *robot{nullptr};
    CSpaceKlampt *klampt_cspace{nullptr};
    CSpaceKlampt *klampt_cspace_outer{nullptr};
    RobotWorld *world{nullptr};

    WorldPlannerSettings worldsettings;
    int robot_idx{0};
    int robot_idx_outer{0};

};
