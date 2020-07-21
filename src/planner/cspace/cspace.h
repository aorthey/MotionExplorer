#pragma once
#include "planner/cspace/cspace_input.h"
#include "klampt.h"
#include "gui/gui_state.h"

#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ControlSpace.h>
#include <Planning/RobotCSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace ompl{
  namespace control{
    typedef std::shared_ptr<ControlSpace> ControlSpacePtr;
    typedef std::shared_ptr<StatePropagator> StatePropagatorPtr;
  }
  namespace base{
    typedef std::shared_ptr<StateValidityChecker> StateValidityCheckerPtr;
  }
}

class CSpaceOMPL
{
  friend class CSpaceOMPLMultiAgent;
  public:

    CSpaceOMPL(RobotWorld *world_, int robot_idx_);

    void Init();
    virtual ob::SpaceInformationPtr SpaceInformationPtr();

    //############################################################################
    //Mapping Functions OMPL <--> KLAMPT
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) = 0;
    virtual void ConfigVelocityToOMPLState(const Config &q, const Config &dq, ob::State *qompl);
    virtual ob::ScopedState<> ConfigVelocityToOMPLState(const Config &q, const Config &dq);

    virtual Config OMPLStateToConfig(const ob::State *qompl) = 0;
    virtual Config OMPLStateToVelocity(const ob::State *qompl);

    Config OMPLStateToConfig(const ob::ScopedState<> &qompl);
    ob::ScopedState<> ConfigToOMPLState(const Config &q);
    //############################################################################

    virtual double GetTime(const ob::State *qompl);
    virtual bool isTimeDependent();
    virtual bool SatisfiesBounds(const ob::State*);
    virtual bool UpdateRobotConfig(Config &q);

    virtual void setStateValidityCheckerConstraintRelaxation(ob::State *xCenter, double r);
    const ob::StateValidityCheckerPtr StateValidityCheckerPtr();
    virtual Vector3 getXYZ(const ob::State*) = 0;
    virtual Vector3 getXYZ(const ob::State*, int ridx);
    virtual bool IsPlanar();

    void SetCSpaceInput(const CSpaceInput &input_);
    virtual uint GetDimensionality() const;
    virtual uint GetKlamptDimensionality() const;
    uint GetControlDimensionality() const;
    Robot* GetRobotPtr();
    std::string GetName();
    int GetRobotIndex() const;
    RobotWorld* GetWorldPtr();
    CSpaceKlampt* GetCSpaceKlamptPtr();
    const ob::StateSpacePtr SpacePtr();
    ob::StateSpacePtr SpacePtrNonConst();
    const oc::ControlSpacePtr ControlSpacePtr();

    virtual void drawConfig(const Config &q, GLDraw::GLColor color=GLDraw::GLColor(1,0,0), double scale = 1.0);
    virtual void drawConfig(const Config &q, const Config &dq, GLDraw::GLColor color=GLDraw::GLColor(1,0,0));
    virtual void DrawGL(GUIState& state);

    //Yaw, Pitch, Roll
    std::vector<double> EulerZYXFromOMPLSO3StateSpace( const ob::SO3StateSpace::StateType *q );
    void OMPLSO3StateSpaceFromEulerZYX( double rz, double ry, double rx, ob::SO3StateSpace::StateType *q );

    virtual bool isDynamic() const = 0;
    bool isFixedBase();
    bool isFreeFloating();

    friend std::ostream& operator<< (std::ostream& out, const CSpaceOMPL& space);
    virtual void print(std::ostream& out = std::cout) const;

    void SetSufficient(const uint robot_outer_idx);
    ob::StateSpacePtr GetFirstSubspace();

    virtual bool isMultiAgent() const;

    virtual Config ControlToConfig(const double*);
  protected:
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si);
    virtual void initSpace() = 0;

    CSpaceInput input;

    uint Nklampt;
    uint Nompl;
    bool fixedBase{false};
    bool enableSufficiency{false};

    //KLAMPT (regardless of fixedbase, freefloating, dynamic)
    // SE(3) x R^Nklampt
    //
    //OMPL:
    // space with Nompl dimensions
    //
    // ompl_to_klampt: maps a dimension in OMPL space to SE(3)xR^Nklampt
    // klampt_to_ompl: maps a dimension in SE(3)xR^Nklampt to OMPL space
    // Note that in general Nompl <= Nklampt (all zero-measure dimensions are collapsed)
    std::vector<int> ompl_to_klampt;
    std::vector<int> klampt_to_ompl;

    ob::StateValidityCheckerPtr validity_checker;
    ob::SpaceInformationPtr si{nullptr};
    ob::StateSpacePtr space{nullptr};
    // oc::RealVectorControlSpacePtr control_space{nullptr};
    oc::ControlSpacePtr control_space{nullptr};

    Robot *robot{nullptr};
    CSpaceKlampt *klampt_cspace{nullptr};
    CSpaceKlampt *klampt_cspace_outer{nullptr};
    RobotWorld *world{nullptr};

    WorldPlannerSettings worldsettings;
    int robot_idx{0};
    int robot_idx_outer{0};

};
